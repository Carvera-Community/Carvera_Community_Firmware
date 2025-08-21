/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"

#include "libs/Kernel.h"
#include "Robot.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"
#include "SerialConsole.h"
#include "libs/SerialMessage.h"
#include "libs/StreamOutputPool.h"
#include "libs/StreamOutput.h"
#include "Gcode.h"
#include "checksumm.h"
#include "Config.h"
#include "ConfigValue.h"
#include "SDFAT.h"
#include "md5.h"

#include "modules/robot/Conveyor.h"
#include "DirHandle.h"
#include "ATCHandlerPublicAccess.h"
#include "PublicDataRequest.h"
#include "PublicData.h"
#include "PlayerPublicAccess.h"
#include "TemperatureControlPublicAccess.h"
#include "TemperatureControlPool.h"
#include "StepTicker.h"
#include "Block.h"
#include "quicklz.h"

#include <math.h>

#include <cstddef>
#include <cmath>
#include <algorithm>

#include "mbed.h"

#define home_on_boot_checksum             CHECKSUM("home_on_boot")
#define on_boot_gcode_checksum            CHECKSUM("on_boot_gcode")
#define on_boot_gcode_enable_checksum     CHECKSUM("on_boot_gcode_enable")
#define after_suspend_gcode_checksum      CHECKSUM("after_suspend_gcode")
#define before_resume_gcode_checksum      CHECKSUM("before_resume_gcode")
#define leave_heaters_on_suspend_checksum CHECKSUM("leave_heaters_on_suspend")
#define laser_module_clustering_checksum 	  CHECKSUM("laser_module_clustering")

extern SDFAT mounter;

#define XBUFF_LENGTH	8208
unsigned char xbuff[XBUFF_LENGTH] __attribute__((section("AHBSRAM"))); /* 2 for data length, 8192 for XModem + 3 head chars + 2 crc + nul */
unsigned char fbuff[4096] __attribute__((section("AHBSRAM")));
// used for XMODEM
#define WAIT_MD5  0x01
#define WAIT_FILE_VIEW  0x02
#define READ_FILE_DATA  0x03

#define MAXRETRANS 50
#define RETRYTIME  50
#define TIMEOUT_MS 10
#define RETRYTIMES 10


Player::Player()
{
    this->playing_file = false;
    this->current_file_handler = nullptr;
    this->booted = false;
    this->elapsed_secs = 0;
    this->reply_stream = nullptr;
    this->inner_playing = false;
    this->slope = 0.0;
}

void Player::on_module_loaded()
{
    this->register_for_event(ON_CONSOLE_LINE_RECEIVED);
    this->register_for_event(ON_MAIN_LOOP);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
    this->register_for_event(ON_GCODE_RECEIVED);
    this->register_for_event(ON_HALT);

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->by_default("/sd/on_boot.gcode")->as_string();
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->by_default(false)->as_bool();

    this->home_on_boot = THEKERNEL->config->value(home_on_boot_checksum)->by_default(true)->as_bool();

    this->after_suspend_gcode = THEKERNEL->config->value(after_suspend_gcode_checksum)->by_default("")->as_string();
    this->before_resume_gcode = THEKERNEL->config->value(before_resume_gcode_checksum)->by_default("")->as_string();
    std::replace( this->after_suspend_gcode.begin(), this->after_suspend_gcode.end(), '_', ' '); // replace _ with space
    std::replace( this->before_resume_gcode.begin(), this->before_resume_gcode.end(), '_', ' '); // replace _ with space
    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->by_default(false)->as_bool();

    this->laser_clustering = THEKERNEL->config->value(laser_module_clustering_checksum)->by_default(false)->as_bool();
}

void Player::on_halt(void* argument)
{
    this->clear_buffered_queue();

    if(argument == nullptr && this->playing_file ) {
        abort_command("1", &(StreamOutput::NullStream));
	}

	if(argument == nullptr && (THEKERNEL->is_suspending() || THEKERNEL->is_waiting())) {
		// clean up from suspend
		THEKERNEL->set_waiting(false);
		THEKERNEL->set_suspending(false);
		THEROBOT->pop_state();
		THEKERNEL->streams->printf("Suspend cleared\n");
	}
}

void Player::on_second_tick(void *)
{
    if(this->playing_file) this->elapsed_secs++;
}

void Player::select_file(string argument)
{

    this->filename = argument;

    this->filename.erase(std::remove(argument.begin(), argument.end(), '"'), argument.end());

    if ((this->filename.rfind("/", 0) == 0)){ //remove starting /
        this->filename.erase(0,1);
    }
    if ((this->filename.rfind("sd/gcodes/", 0) == 0)){
        this->filename = "/" + this->filename;

    }else if ((this->filename.rfind("gcodes/", 0) == 0)){
        this->filename = "/sd/" + this->filename;
    }else {
        this->filename = "/sd/gcodes/" + this->filename;
    }
    if (this->filename.rfind(".cnc") != this->filename.length() - 4) {
        this->filename += ".cnc";
    }
    this->current_stream = nullptr;

    if(this->current_file_handler != NULL) {
        this->playing_file = false;
        fclose(this->current_file_handler);
    }
    this->current_file_handler = fopen( this->filename.c_str(), "r");

    if(this->current_file_handler == NULL) {
        THEKERNEL->streams->printf("file.open failed: %s\r\n", this->filename.c_str());
        return;

    } else {
        // get size of file
        int result = fseek(this->current_file_handler, 0, SEEK_END);
        if (0 != result) {
            this->file_size = 0;
        } else {
            this->file_size = ftell(this->current_file_handler);
            fseek(this->current_file_handler, 0, SEEK_SET);
        }
        THEKERNEL->streams->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(), this->file_size);
        THEKERNEL->streams->printf("File selected\r\n");
    }
    this->played_cnt = 0;
    this->played_lines = 0;
    this->elapsed_secs = 0;
    this->playing_lines = 0;
    this->goto_line = 0;
}

void Player::goto_line_number(unsigned long line_number)
{
    this->goto_line = line_number;
    this->goto_line = this->goto_line < 1 ? 1 : this->goto_line;
    THEKERNEL->streams->printf("Goto line %lu...\r\n", this->goto_line);
    // goto line
    char buf[130]; // lines upto 128 characters are allowed, anything longer is discarded

    // goto file begin
    fseek(this->current_file_handler, 0, SEEK_SET);
    played_lines = 0;
    played_cnt   = 0;

    while (fgets(buf, sizeof(buf), this->current_file_handler) != NULL) {
        if (played_lines % 100 == 0) {
            THEKERNEL->call_event(ON_IDLE);
        }
        int len = strlen(buf);
        if (len == 0) continue; // empty line? should not be possible

        played_lines += 1;
        played_cnt += len;
        if (played_lines >= this->goto_line) {
            break;
        }
    }
}

void Player::end_of_file()
{
    if (this->macro_file_queue.empty()) {
        // Exit or handle the empty queue case
        return;
    }
    std::tuple<std::string, unsigned long> queueItem = this->macro_file_queue.front();
    this->macro_file_queue.pop();

    THEKERNEL->streams->printf("return filepath:  %s return line: %ld \r\n", std::get<0>(queueItem).c_str(),  std::get<1>(queueItem));
    //this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);


    this->select_file(std::get<0>(queueItem));
    this->goto_line_number(std::get<1>(queueItem));
    this->play_opened_file();
}

void Player::play_opened_file()
{
    if (this->current_file_handler != NULL) {
        this->playing_file = true;
        // this would be a problem if the stream goes away before the file has finished,
        // so we attach it to the kernel stream, however network connections from pronterface
        // do not connect to the kernel streams so won't see this FIXME
        this->reply_stream = THEKERNEL->streams;
    }
}
// extract any options found on line, terminates args at the space before the first option (-v)
// eg this is a file.gcode -v
//    will return -v and set args to this is a file.gcode
string Player::extract_options(string& args)
{
    string opts;
    size_t pos= args.find(" -");
    if(pos != string::npos) {
        opts= args.substr(pos);
        args= args.substr(0, pos);
    }

    return opts;
}

void Player::on_gcode_received(void *argument)
{
    Gcode *gcode = static_cast<Gcode *>(argument);
    string args = get_arguments(gcode->get_command());
    if (gcode->has_m) {
        if (gcode->m == 1) { //optiional stop
            if (THEKERNEL->get_optional_stop_mode()){
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);
            }

        }      
        else if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            mounter.remount();
            gcode->stream->printf("SD card ok\r\n");

        } else if (gcode->m == 23) { // select file
            this->clear_macro_file_queue();
            this->select_file(args);
        } else if (gcode->m == 24) { // start print
            this->play_opened_file();

        } else if (gcode->m == 25) { // pause print
            this->playing_file = false;

        } else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            //empty macro queue
            this->clear_macro_file_queue();
            if(this->current_file_handler != NULL) {
                string currentfn = this->filename.c_str();
                unsigned long old_size = this->file_size;

                // abort the print
                abort_command("", gcode->stream);

                if(!currentfn.empty()) {
                    // reload the last file opened
                    this->current_file_handler = fopen(currentfn.c_str() , "r");

                    if(this->current_file_handler == NULL) {
                        gcode->stream->printf("file.open failed: %s\r\n", currentfn.c_str());
                    } else {
                        this->filename = currentfn;
                        this->file_size = old_size;
                        this->current_stream = nullptr;
                    }
                }
            } else {
                gcode->stream->printf("No file loaded\r\n");
            }

        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
            progress_command("-b", gcode->stream);

        //} else if (gcode->m == 30) { // end file implementation for M30 returning from macros. M99 is the proper formatting.
        //    this->end_of_file();
        } else if (gcode->m == 32) { // select file and start print
            
            
            //empty macro queue
            this->clear_macro_file_queue();

            this->select_file(args);

            this->play_opened_file();           

        } else if (gcode->m == 97) {
            if (gcode->has_letter('P')) {
                this->goto_line_number(gcode->get_value('P'));
                return;
            }else{
                    //error: no filepath found
                    THEKERNEL->streams->printf("M97 Command missing P parameter for line to goto, aborting \n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(MANUAL);
                    return;
            }
        } else if (gcode->m == 98) { // run macro
            string new_filepath;
            //int current_gcode_line = 14; //gcode->stream;
            int num_repeats = 1;
            
            if (gcode->has_letter('P')) {
                

                int filenumber_int = static_cast<int>(gcode->get_value('P'));

                if (filenumber_int){
                    

                    // Manually convert the integer part to a string
                    while (filenumber_int > 0) {
                        new_filepath = char((filenumber_int % 10) + '0') + new_filepath;
                        filenumber_int /= 10;
                    }
                    new_filepath = "/sd/gcodes/macros/" + new_filepath + ".cnc";
                }else{
                    //error:
                    THEKERNEL->streams->printf("invalid number in M98 command \n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(MANUAL);
                    return;
                }
            }
            if (gcode->has_letter('L')) {
                num_repeats = floor(gcode->get_value('L'));
                if (num_repeats <1){
                    //error:
                    THEKERNEL->streams->printf("M98 command has an invalid value, which will lead to errors \n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(MANUAL);
                    return;
                }
            }

            if (gcode->subcode == 1){
                std::string input = gcode->get_command();
                size_t first_quote = input.find('\"');
                size_t last_quote = input.rfind('\"');

                if (first_quote != std::string::npos && last_quote != std::string::npos && first_quote != last_quote) {
                    new_filepath = input.substr(first_quote + 1, last_quote - first_quote - 1);
                    
                    if (!(new_filepath.rfind("/sd/gcodes/", 0) == 0)){
                        new_filepath = "/sd/gcodes/" + new_filepath;
                    }
                }else{
                    //error: no filepath found
                    THEKERNEL->streams->printf("no filepath found in M98.1 command \n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(MANUAL);
                    return;
                }
            }

            //TODO: test new_filepath length to make sure it is valid
            std::tuple<std::string, unsigned long> queueItem (this->filename,(played_lines + 2));
            //set up return queue
            this->macro_file_queue.push(queueItem);

            //set up repeats
            if (num_repeats > 1){
                for (int i = 1; i < num_repeats; i++){
                    std::tuple<std::string, unsigned long> queueItem_repeat (new_filepath,0);
                    this->macro_file_queue.push(queueItem_repeat);
                }
            }
            
            //open file and play
            this->select_file(new_filepath);
            this->play_opened_file();

        } else if (gcode->m == 99) { // return from macro to main program
            this->end_of_file();

        } else if (gcode->m == 600) { // suspend print, Not entirely Marlin compliant, M600.1 will leave the heaters on
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream, (gcode->subcode == 5)?true:false);

        } else if (gcode->m == 601) { // resume print
            this->resume_command("", gcode->stream);
        }

    }else if(gcode->has_g) {
        if(gcode->g == 28) { // homing cancels suspend
            if (THEKERNEL->is_suspending()) {
                // clean up
            	THEKERNEL->set_suspending(false);
                THEROBOT->pop_state();
            }
        }
    }
}

// When a new line is received, check if it is a command, and if it is, act upon it
void Player::on_console_line_received( void *argument )
{
    if(THEKERNEL->is_halted()) return; // if in halted state ignore any commands

    SerialMessage new_message = *static_cast<SerialMessage *>(argument);

    string possible_command = new_message.message;

    // ignore anything that is not lowercase or a letter
    if(possible_command.empty() || !islower(possible_command[0]) || !isalpha(possible_command[0])) {
        return;
    }

    string cmd = shift_parameter(possible_command);

	// new_message.stream->printf("Play Received %s\r\n", possible_command.c_str());

    // Act depending on command
    if (cmd == "play"){
        this->play_command( possible_command, new_message.stream );
    }else if (cmd == "progress"){
        this->progress_command( possible_command, new_message.stream );
    }else if (cmd == "abort") {
        this->abort_command( possible_command, new_message.stream );
    }else if (cmd == "suspend") {
        this->suspend_command( possible_command, new_message.stream );
    }else if (cmd == "resume") {
        this->resume_command( possible_command, new_message.stream );
    }else if (cmd == "goto") {
    	this->goto_command( possible_command, new_message.stream );
    }else if (cmd == "buffer") {
    	this->buffer_command( possible_command, new_message.stream );
    }else if (cmd == "upload") {
    	this->upload_command( possible_command, new_message.stream );
    }else if (cmd == "download") {
        memset(md5_str, 0, sizeof(md5_str));
    	if (possible_command.find("config.txt") != string::npos) {
        	this->test_command( possible_command, new_message.stream );
    	}
    	this->download_command( possible_command, new_message.stream );
    }
}

// Buffer gcode to queue
void Player::buffer_command( string parameters, StreamOutput *stream )
{
	this->buffered_queue.push(parameters);
	stream->printf("Command buffered: %s\r\n", parameters.c_str());
}

// Play a gcode file by considering each line as if it was received on the serial console
void Player::play_command( string parameters, StreamOutput *stream )
{
//    // current tool number and tool offset
//    struct tool_status tool;
//    bool tool_ok = PublicData::get_value( atc_handler_checksum, get_tool_status_checksum, &tool );
//    if (tool_ok) {
//    	tool_ok = tool.active_tool > 0;
//    }
//	// check if is tool -1 or tool 0
//	if (!tool_ok) {
//		THEKERNEL->set_halt_reason(MANUAL);
//		THEKERNEL->call_event(ON_HALT, nullptr);
//		THEKERNEL->streams->printf("ERROR: No tool or probe tool!\n");
//		return;
//	}

    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
    // Get filename which is the entire parameter line upto any options found or entire line
    this->filename = absolute_from_relative(shift_parameter(parameters));
    this->last_filename = this->filename;

    if (this->playing_file || THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

    if (this->current_file_handler != NULL) { // must have been a paused print
        fclose(this->current_file_handler);
    }

//    this->temp_file_handler = fopen ("/sd/gcodes/temp.nc", "w");
//    if (this->temp_file_handler == NULL) {
//        stream->printf("File open error: /sd/gcodes/temp.nc\n");
//        return;
//    }

    //empty macro queue
    this->clear_macro_file_queue();

    this->current_file_handler = fopen( this->filename.c_str(), "r");
    if(this->current_file_handler == NULL) {
        stream->printf("File not found: %s\r\n", this->filename.c_str());
        return;
    }

    stream->printf("Playing %s\r\n", this->filename.c_str());

    this->playing_file = true;

    // Output to the current stream if we were passed the -v ( verbose ) option
    if( options.find_first_of("Vv") == string::npos ) {
        this->current_stream = nullptr;
    } else {
        // we send to the kernels stream as it cannot go away
        this->current_stream = THEKERNEL->streams;
    }

    // get size of file
    int result = fseek(this->current_file_handler, 0, SEEK_END);
    if (0 != result) {
        stream->printf("WARNING - Could not get file size\r\n");
        file_size = 0;
    } else {
        file_size = ftell(this->current_file_handler);
        fseek(this->current_file_handler, 0, SEEK_SET);
        stream->printf("  File size %ld\r\n", file_size);
    }
    this->played_cnt = 0;
    this->played_lines = 0;
    this->elapsed_secs = 0;
    this->playing_lines = 0;
    this->goto_line = 0;

    // force into absolute mode
    THEROBOT->absolute_mode = true;
    THEROBOT->e_absolute_mode = true;

    // reset current position;
    THEROBOT->reset_position_from_current_actuator_position();
}

// Goto a certain line when playing a file
void Player::goto_command( string parameters, StreamOutput *stream )
{
    if (!THEKERNEL->is_suspending()) {
        stream->printf("Can only jump when pausing!\r\n");
        return;
    }

    if (this->current_file_handler == NULL) {
    	stream->printf("Missing file handle!\r\n");
    	return;
    }

    string line_str = shift_parameter(parameters);
    if (!line_str.empty()) {
        char *ptr = NULL;
        this->goto_line = strtol(line_str.c_str(), &ptr, 10);
        this->goto_line_number(this->goto_line);
        
    }
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && current_file_handler != NULL) {
        if(sdprinting)
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        else
            stream->printf("SD print is paused at %lu/%lu\r\n", played_cnt, file_size);
        return;

    } else if(!playing_file) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    if(file_size > 0) {
        unsigned long est = 0;
        if(this->elapsed_secs > 10) {
            unsigned long bytespersec = played_cnt / this->elapsed_secs;
            if(bytespersec > 0)
                est = (file_size - played_cnt) / bytespersec;
        }

        float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
        // If -b or -B is passed, report in the format used by Marlin and the others.
        if (!sdprinting) {
            stream->printf("file: %s, %u %% complete, elapsed time: %02lu:%02lu:%02lu", this->filename.c_str(), (unsigned int)roundf(pcnt), this->elapsed_secs / 3600, (this->elapsed_secs % 3600) / 60, this->elapsed_secs % 60);
            if(est > 0) {
                stream->printf(", est time: %02lu:%02lu:%02lu",  est / 3600, (est % 3600) / 60, est % 60);
            }
            stream->printf("\r\n");
        } else {
            stream->printf("SD printing byte %lu/%lu\r\n", played_cnt, file_size);
        }

    } else {
        stream->printf("File size is unknown\r\n");
    }
}

void Player::abort_command( string parameters, StreamOutput *stream )
{
    if(!playing_file && current_file_handler == NULL) {
        stream->printf("Not currently playing\r\n");
        return;
    }

    this->playing_file = false;
    this->played_cnt = 0;
    this->played_lines = 0;
    this->playing_lines = 0;
    this->goto_line = 0;
    this->file_size = 0;
    this->clear_buffered_queue();
    this->filename = "";
    this->current_stream = NULL;

    fclose(current_file_handler);
    current_file_handler = NULL;

    THEKERNEL->set_suspending(false);
    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);


    // turn off spindle
    {
		struct SerialMessage message;
		message.message = "M5";
		message.stream = THEKERNEL->streams;
		message.line = 0;
		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }

    if (parameters.empty()) {
        // clear out the block queue, will wait until queue is empty
        // MUST be called in on_main_loop to make sure there are no blocked main loops waiting to put something on the queue
        THEKERNEL->conveyor->flush_queue();

        // now the position will think it is at the last received pos, so we need to do FK to get the actuator position and reset the current position
        THEROBOT->reset_position_from_current_actuator_position();
        stream->printf("Aborted playing or paused file. \r\n");
    }

}

void Player::clear_buffered_queue(){
	while (!this->buffered_queue.empty()) {
		this->buffered_queue.pop();
	}
}

void Player::clear_macro_file_queue(){
	while (!this->macro_file_queue.empty()) {
		this->macro_file_queue.pop();
	}
}

void Player::on_main_loop(void *argument)
{
    if( !this->booted ) {
        this->booted = true;
        if (this->home_on_boot) {
    		struct SerialMessage message;
    		message.message = "$H";
    		message.stream = THEKERNEL->streams;
    		message.line = 0;
    		THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
        }

        if (this->on_boot_gcode_enable) {
            this->play_command(this->on_boot_gcode, THEKERNEL->serial);
        }

    }

    if ( this->playing_file ) {
        if(THEKERNEL->is_halted() || THEKERNEL->is_suspending() || THEKERNEL->is_waiting() || this->inner_playing) {
            return;
        }

        // check if there are bufferd command
        while (!this->buffered_queue.empty()) {
        	THEKERNEL->streams->printf("%s\r\n", this->buffered_queue.front().c_str());
			struct SerialMessage message;
			message.message = this->buffered_queue.front();
			message.stream = THEKERNEL->streams;
			message.line = 0;
			this->buffered_queue.pop();

			// waits for the queue to have enough room
			THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            return;
        }

        char buf[130]; // lines up to 128 characters are allowed, anything longer is discarded
        bool discard = false;

        // 2024
        /*
        bool is_cluster = false;
        int cluster_index = 0;
        float x_value = 0.0, y_value = 0.0, sum_x_value = 0.0, sum_y_value = 0.0,
        		s_value = 0.0, distance = 0.0, min_distance = 10000.0, min_value = 0.0,
				sum_distance = 0.0, sum_value = 0.0;
        string clustered_gcode = "";
        float clustered_s_value[8];
        float clustered_distance[8];
        */

        while (fgets(buf, sizeof(buf), this->current_file_handler) != NULL) {

            int len = strlen(buf);
            if (len == 0) continue; // empty line? should not be possible
            if (buf[len - 1] == '\n' || feof(this->current_file_handler)) {
                if(discard) { // we are discarding a long line
                    discard = false;
                    continue;
                }

                if (len == 1) continue; // empty line

                /*
            	// Add laser cluster support when in laser mode
            	if (this->laser_clustering && THEKERNEL->get_laser_mode() && !THEROBOT->absolute_mode && played_lines > 100) {
            		// G1 X0.5 Y 0.8 S1:0:0.5:0.75:0:0.2
            		is_cluster = this->check_cluster(buf, &x_value, &y_value, &distance, &slope, &s_value);
                    min_value = fmin(min_distance, distance);
                    sum_value = sum_distance + distance;
            		if (is_cluster && (min_value > 0 && sum_value * 1.0 / min_value < 8.1)) {
                        min_distance = min_value;
                        sum_distance = sum_value;
                        sum_x_value += x_value;
                        sum_y_value += y_value;
                        cluster_index ++;
                        clustered_s_value[cluster_index - 1] = s_value;
                        clustered_distance[cluster_index - 1] = distance;
                        played_lines += 1;
                        played_cnt += len;
						if (cluster_index >= 8 || (min_distance > 0 && sum_distance * 1.0 / min_distance > 7.9)) {
	                        sprintf(md5_str, "G1 X%.3f Y%.3f S", sum_x_value, sum_y_value);
	                        clustered_gcode = md5_str;
	                        for (int i = 0; i < cluster_index; i ++) {
	                            for (float j = min_distance; j < clustered_distance[i] + 0.01; j+= min_distance) {
	                            	sprintf(md5_str, "%s%.2f", (i == 0 ? "" : ":"), clustered_s_value[i]);
	                                clustered_gcode.append(md5_str);
	                            }
	                        }

							struct SerialMessage message;
							message.message = clustered_gcode;
							message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;
							message.line = played_lines + 1;

							// waits for the queue to have enough room
							THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
							// fputs(clustered_gcode.c_str(), this->temp_file_handler);
							// fputs("\n", this->temp_file_handler);
							// THEKERNEL->streams->printf("1-[Line: %d] %s\n", message.line, clustered_gcode.c_str());
							return;
						}
						continue;
            		} else {
                		if (cluster_index > 0) {
	                        sprintf(md5_str, "G1 X%.3f Y%.3f S", sum_x_value, sum_y_value);
	                        clustered_gcode = md5_str;
	                        for (int i = 0; i < cluster_index; i ++) {
	                            for (float j = min_distance; j < clustered_distance[i] + 0.01; j+= min_distance) {
	                            	sprintf(md5_str, "%s%.2f", (i == 0 ? "" : ":"), clustered_s_value[i]);
	                                clustered_gcode.append(md5_str);
	                            }
	                        }
	                        clustered_gcode.append("\n");

    						struct SerialMessage message;
    						message.message = clustered_gcode;
    						message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;
    						message.line = played_lines + 1;

    						// waits for the queue to have enough room
    						THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    						// fputs(clustered_gcode.c_str(), this->temp_file_handler);
    						// fputs("\n", this->temp_file_handler);
    						// THEKERNEL->streams->printf("2-[Line: %d] %s\n", message.line, clustered_gcode.c_str());
                		}
                        sum_x_value = 0.0;
                        sum_y_value = 0.0;
                        sum_distance = 0.0;
                        cluster_index = 0;
                        min_distance = 10000.0;
            		}
            	}
*/

                if (this->current_stream != nullptr) {
                    this->current_stream->printf("%s", buf);
                }

                struct SerialMessage message;
                message.message = buf;
                message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;
                message.line = played_lines + 1;

                // waits for the queue to have enough room
                // this->current_stream->printf("Run: %s", buf);
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                // fputs(buf, this->temp_file_handler);
                // THEKERNEL->streams->printf("0-[Line: %d] %s\n", message.line, buf);
                played_lines += 1;
                played_cnt += len;
                //M335 disables line by line, M336 Enables. Pauses after every valid gcode line
                if (THEKERNEL->get_line_by_line_exec_mode() && len > 2 && buf[0] != ';' && buf[0] != '('){
                    this->suspend_command("", THEKERNEL->streams);
                }

                return; // we feed one line per main loop

            } else {
                // discard long line
                if (this->current_stream != nullptr) { this->current_stream->printf("Warning: Discarded long line\n"); }
                discard = true;
            }
        }

        this->playing_file = false;
        this->filename = "";
        played_cnt = 0;
        played_lines = 0;
        playing_lines = 0;
        goto_line = 0;
        file_size = 0;

        fclose(this->current_file_handler);
        current_file_handler = NULL;

        this->current_stream = NULL;

        if(this->reply_stream != NULL) {
            // if we were printing from an M command from pronterface we need to send this back
            this->reply_stream->printf("Done printing file\r\n");
            this->reply_stream = NULL;
        }
        
        bool bbb = true;
        PublicData::set_value( atc_handler_checksum, set_job_complete_checksum, &bbb );
    }
}

/*
bool Player::check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value)
{
	float new_slope = 0.0;
	bool is_cluster = false;
	Gcode *gcode = new Gcode(gcode_str, &StreamOutput::NullStream);
	if (!gcode->has_m && gcode->has_g && gcode->g == 1) {
		*x_value = gcode->get_value('X');
		*y_value = gcode->get_value('Y');
		*s_value = gcode->get_value('S');
		*distance = sqrtf((*x_value) * (*x_value) + (*y_value) * (*y_value));
		if (*x_value == 0) {
			new_slope = *y_value > 0 ? 1000 : -1000;
		} else if (*y_value == 0) {
			new_slope = *x_value > 0 ? 0.001 : -0.001;
		} else {
			new_slope = *y_value / *x_value;
		}
		if ((*distance) < 1.0 && fabs (new_slope - *slope) < 0.1) {
			is_cluster = true;
		}
		*slope = new_slope;
	}
	delete gcode;

	return is_cluster;
}
*/

void Player::on_get_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(is_playing_checksum) || pdr->second_element_is(is_suspended_checksum)) {
        static bool bool_data;
        bool_data = pdr->second_element_is(is_playing_checksum) ? this->playing_file : THEKERNEL->is_suspending();
        pdr->set_data_ptr(&bool_data);
        pdr->set_taken();

    } else if(pdr->second_element_is(get_progress_checksum)) {
        static struct pad_progress p;
        if(file_size > 0 && playing_file) {
        	if (!this->inner_playing) {
                const Block *block = StepTicker::getInstance()->get_current_block();
                // Note to avoid a race condition where the block is being cleared we check the is_ready flag which gets cleared first,
                // as this is an interrupt if that flag is not clear then it cannot be cleared while this is running and the block will still be valid (albeit it may have finished)
                if (block != nullptr && block->is_ready && block->is_g123) {
                	this->playing_lines = block->line;
                	p.played_lines = this->playing_lines;
                } else {
                	p.played_lines = this->played_lines;
                }
        	} else {
        		p.played_lines = this->played_lines;
        	}
            p.elapsed_secs = this->elapsed_secs;
            float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
            p.percent_complete = roundf(pcnt);
            p.filename = this->filename;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        }
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = this->inner_playing;
        pdr->set_data_ptr(&b);
        pdr->set_taken();
    }
}

void Player::on_set_public_data(void *argument)
{
    PublicDataRequest *pdr = static_cast<PublicDataRequest *>(argument);

    if(!pdr->starts_with(player_checksum)) return;

    if(pdr->second_element_is(abort_play_checksum)) {
        abort_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
    } else if (pdr->second_element_is(inner_playing_checksum)) {
    	bool b = *static_cast<bool *>(pdr->get_data_ptr());
    	this->inner_playing = b;
    	if (this->playing_file) pdr->set_taken();
    } else if (pdr->second_element_is(restart_job_checksum)) {
    	if (!this->last_filename.empty()) {
    		THEKERNEL->streams->printf("Job restarted: %s.\r\n", this->last_filename.c_str());
        	this->play_command(this->last_filename, &(StreamOutput::NullStream));
    	}
    }
}

/**
Suspend a print in progress
1. send pause to upstream host, or pause if printing from sd
1a. loop on_main_loop several times to clear any buffered commmands
2. wait for empty queue
3. save the current position, extruder position, temperatures - any state that would need to be restored
4. retract by specifed amount either on command line or in config
5. turn off heaters.
6. optionally run after_suspend gcode (either in config or on command line)

User may jog or remove and insert filament at this point, extruding or retracting as needed

*/
void Player::suspend_command(string parameters, StreamOutput *stream, bool pause_outside_play_mode )
{
    if (THEKERNEL->is_suspending() || THEKERNEL->is_waiting()) {
        stream->printf("Already suspended!\n");
        return;
    }

    if(!this->playing_file && !pause_outside_play_mode) {
        stream->printf("Can not suspend when not playing file!\n");
        return;
    }

    stream->printf("Suspending , waiting for queue to empty...\n");

    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Suspend aborted by halt\n");
        THEKERNEL->set_waiting(false);
        return;
    }

    THEKERNEL->set_waiting(false);
    THEKERNEL->set_suspending(true);

    // save current XYZ position in WCS
    Robot::wcs_t mpos= THEROBOT->get_axis_position();
    Robot::wcs_t wpos= THEROBOT->mcs2wcs(mpos);
    saved_position[0]= std::get<X_AXIS>(wpos);
    saved_position[1]= std::get<Y_AXIS>(wpos);
    saved_position[2]= std::get<Z_AXIS>(wpos);

    // save current state
    THEROBOT->push_state();
    current_motion_mode = THEROBOT->get_current_motion_mode();

    // execute optional gcode if defined
    if(!after_suspend_gcode.empty()) {
        struct SerialMessage message;
        message.message = after_suspend_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    THEKERNEL->streams->printf("Suspended, resume to continue playing\n");
}

/**
resume the suspended print
1. restore the temperatures and wait for them to get up to temp
2. optionally run before_resume gcode if specified
3. restore the position it was at and E and any other saved state
4. resume sd print or send resume upstream
*/
void Player::resume_command(string parameters, StreamOutput *stream )
{
    if(!THEKERNEL->is_suspending()) {
        stream->printf("Not suspended\n");
        return;
    }

    stream->printf("Resuming playing...\n");

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEROBOT->pop_state();
        THEKERNEL->set_suspending(false);
        return;
    }

    // execute optional gcode if defined
    if(!before_resume_gcode.empty()) {
        stream->printf("Executing before resume gcode...\n");
        struct SerialMessage message;
        message.message = before_resume_gcode;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );
    }

    if (this->goto_line == 0) {
        // Restore position
        stream->printf("Restoring saved XYZ positions and state...\n");

        THEROBOT->absolute_mode = true;

        char buf[128];
        snprintf(buf, sizeof(buf), "G1 X%.3f Y%.3f Z%.3f F%.3f", saved_position[0], saved_position[1], saved_position[2], THEROBOT->from_millimeters(1000));
        struct SerialMessage message;
        message.message = buf;
        message.stream = &(StreamOutput::NullStream);
        message.line = 0;
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message );

    	if (current_motion_mode > 1) {
            snprintf(buf, sizeof(buf), "G%d", current_motion_mode - 1);
            message.message = buf;
            message.line = 0;
            THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    	}
    }

    THEROBOT->pop_state();

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEKERNEL->set_suspending(false);
        return;
    }

	THEKERNEL->set_suspending(false);

	stream->printf("Playing file resumed\n");
}

unsigned int Player::crc16_ccitt(unsigned char *data, unsigned int len)
{
	static const unsigned short crc_table[] = {
		0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
		0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
		0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
		0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
		0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
		0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
		0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
		0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
		0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
		0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
		0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
		0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
		0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
		0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
		0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
		0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
		0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
		0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
		0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
		0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
		0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
		0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
		0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
		0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
		0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
		0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
		0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
		0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0,
	};

	unsigned char tmp;
	unsigned short crc = 0;

	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}

	return crc & 0xffff;
}

int Player::check_crc(int crc, unsigned char *data, unsigned int len)
{
    if (crc) {
        unsigned short crc = crc16_ccitt(data, len);
        unsigned short tcrc = (data[len] << 8) + data[len+1];
        if (crc == tcrc)
            return 1;
    }
    else {
        unsigned char cks = 0;
        for (unsigned int i = 0; i < len; ++i) {
            cks += data[i];
        }
        if (cks == data[len])
        return 1;
    }

    return 0;
}

int Player::inbyte(StreamOutput *stream, unsigned int timeout_ms)
{
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->_getc();
        safe_delay_us(100);
    }
    return -1;
}

int Player::inbytes(StreamOutput *stream, char **buf, int size, unsigned int timeout_ms)
{
	uint32_t tick_us = us_ticker_read();
    while (us_ticker_read() - tick_us < timeout_ms * 1000) {
        if (stream->ready())
            return stream->gets(buf, size);
        safe_delay_us(100);
    }
    return -1;
}

void Player::set_serial_rx_irq(bool enable)
{
	// disable serial rx irq
    bool enable_irq = enable;
    PublicData::set_value( atc_handler_checksum, set_serial_rx_irq_checksum, &enable_irq );
}
int Player::decompress(string sfilename, string dfilename, uint32_t sfilesize, StreamOutput* stream)
{
	FILE *f_in = NULL, *f_out = NULL;
	uint16_t  u16Sum = 0;
	uint8_t u8ReadBuffer_hdr[BLOCK_HEADER_SIZE] = { 0 };
	uint32_t u32DcmprsSize = 0, u32BlockSize = 0, u32BlockNum = 0, u32TotalDcmprsSize = 0, i = 0,j = 0,k=0;
	qlz_state_decompress s_stDecompressState;
	char error_msg[80];
	memset(error_msg, 0, sizeof(error_msg));
	sprintf(error_msg, "decompress!");
	
	f_in= fopen(sfilename.c_str(), "rb");
	f_out= fopen(dfilename.c_str(), "w+");
	if (f_in == NULL || f_out == NULL)
	{
		sprintf(error_msg, "Error: failed to create file [%s]!\r\n", filename.substr(0, 30).c_str());
		goto _exit;
	}
	for(i = 0; i < sfilesize-2; i+= BLOCK_HEADER_SIZE + u32BlockSize)
	{

		fread(u8ReadBuffer_hdr, sizeof(char), BLOCK_HEADER_SIZE, f_in);
		u32BlockSize = u8ReadBuffer_hdr[0] * (1 << 24) + u8ReadBuffer_hdr[1] * (1 << 16) + u8ReadBuffer_hdr[2] * (1 << 8) + u8ReadBuffer_hdr[3];
		if(!u32BlockSize)
		{
			sprintf(error_msg, "Error: decompress file error,bad block num.");
			goto _exit;
		}
		fread(xbuff, sizeof(char), u32BlockSize, f_in);
		u32DcmprsSize = qlz_decompress((const char *)xbuff, fbuff, &s_stDecompressState);
		if(!u32DcmprsSize)
		{
			sprintf(error_msg, "Error: decompress file error,bad decompress size.");
			goto _exit;
		}
		for(j = 0; j < u32DcmprsSize; j++)
		{
			u16Sum += fbuff[j];
		}
		// Set the file write system buffer 4096 Byte
		setvbuf(f_out, (char*)&xbuff[4096], _IOFBF, 4096);
		fwrite(fbuff, sizeof(char),u32DcmprsSize, f_out);
		u32TotalDcmprsSize += u32DcmprsSize;
		u32BlockNum += 1;
		if(++k>10)
		{
			k=0;
			THEKERNEL->call_event(ON_IDLE);
					
			sprintf(error_msg, "#Info: decompart = %lu\r\n", u32BlockNum);
			stream->printf(error_msg);
		}
	}
	fread(fbuff, sizeof(char), 2, f_in);
	if(u16Sum != ((fbuff[0] <<8) + fbuff[1]))
	{
		sprintf(error_msg, "Error: decompress file sum check error.");
		goto _exit;
	}

	if (f_in != NULL)
		fclose(f_in);
	if (f_out!= NULL)
		fclose(f_out);
	
	sprintf(error_msg, "#Info: decompart = %lu\r\n", u32BlockNum);
	stream->printf(error_msg);
	return 1;
_exit:
	if (f_in != NULL)
		fclose(f_in );
	if (f_out != NULL)
		fclose(f_out);
	stream->printf(error_msg);
	return 0;
}

void Player::upload_command( string parameters, StreamOutput *stream )
{
    char *recv_buff;
    char FileRcvState = WAIT_MD5;
    int cmdType = 0;
    int retry = 0;
    int tatalretry = 0;
    int crc = 0;
    uint32_t total_packet = 0;
    uint16_t packet_size = 0;
    uint16_t data_len = 0;
    uint32_t sequence = 0;
    uint32_t u32filesize = 0;
    uint32_t starttime;
    uint32_t seq;
    
    char buf[] = "ok\r\n";

    // open file
	char error_msg[64];
	memset(error_msg, 0, sizeof(error_msg));
	sprintf(error_msg, "Nothing!");
    string filename = absolute_from_relative(shift_parameter(parameters));
    string md5_filename = change_to_md5_path(filename);
    string lzfilename = change_to_lz_path(filename);
    check_and_make_path(md5_filename);
    check_and_make_path(lzfilename);

	// diasble serial rx irq in case of serial stream, and internal process in case of wifi
    if (stream->type() == 0) {
    	set_serial_rx_irq(false);
    }
    THEKERNEL->set_uploading(true);

    if (!THECONVEYOR->is_idle()) {
		SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
        if (stream->type() == 0) {
        	set_serial_rx_irq(true);
        }
        THEKERNEL->set_uploading(false);
	    THEKERNEL->set_cachewait(true);
	    safe_delay_ms(1000);
	    THEKERNEL->set_cachewait(false);
        return;
    }
	
	//if file is lzCompress file,then need to put .lz dir
	unsigned int start_pos = filename.find(".lz");
	FILE *fd;
	if (start_pos != string::npos) {
		start_pos = lzfilename.rfind(".lz");
		lzfilename=lzfilename.substr(0, start_pos);
    	fd = fopen(lzfilename.c_str(), "wb");
    }
    else {
    	fd = fopen(filename.c_str(), "wb");
    }
		
    FILE *fd_md5 = NULL;
    //if file is lzCompress file,then need to Decompress
	start_pos = md5_filename.find(".lz");
	if (start_pos != string::npos) {
		md5_filename=md5_filename.substr(0, start_pos);
	}
    fd_md5 = fopen(md5_filename.c_str(), "wb");

    if (fd == NULL || fd_md5 == NULL) {
        
		SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
    	sprintf(error_msg, "Error: failed to open file [%s]!\r\n", fd == NULL ? filename.substr(0, 30).c_str() : md5_filename.substr(0, 30).c_str() );
    	goto upload_error;
    }
	
	// stop TIMER0 and TIMER1 for save time
	NVIC_DisableIRQ(TIMER0_IRQn);
	NVIC_DisableIRQ(TIMER1_IRQn);
	starttime = us_ticker_read();
	
	stream->reset();
	
    for (;;) {
    	cmdType = inbytes(stream, &recv_buff, 0, TIMEOUT_MS);
        if (cmdType > 0) 
        {
        	starttime = us_ticker_read();
		    if ( cmdType == PTYPE_FILE_CAN )
		    {
		    	FileRcvState = WAIT_MD5;
	            retry = 0;
                sprintf(error_msg, "Info: Upload canceled by Controller!\r\n");
				goto upload_error;
			}
				
        	switch (FileRcvState) {
	            case WAIT_MD5:
	                if (cmdType == PTYPE_FILE_MD5)
	                {
	                	fwrite(&recv_buff[3], sizeof(char), 32, fd_md5);
				        SendMessage(PTYPE_FILE_VIEW, buf, 0, stream);	//request File view
	                	FileRcvState = WAIT_FILE_VIEW;
	                	retry = 0;
	                	tatalretry = 0;
	                }
	                else
	                {
	                	retry ++;
	                	if(retry > RETRYTIME)
	                	{
				        	SendMessage(PTYPE_FILE_MD5, buf, 0, stream);		//request MD5
				        	retry = 0;
	                		tatalretry ++;
				        }
				        else
				        {
							THEKERNEL->call_event(ON_IDLE);
				        	continue;
				        }
	                }
	                break;
	            case WAIT_FILE_VIEW:
	            	if (cmdType == PTYPE_FILE_VIEW)
	                {
	                	total_packet = (recv_buff[3]<<24) | (recv_buff[4]<<16) | (recv_buff[5]<<8) | recv_buff[6];
	                	packet_size = (recv_buff[7]<<8) | recv_buff[8];
	                	sequence = 1;   
	                	xbuff[0] = (HEADER>>8)&0xFF;
						xbuff[1] = HEADER&0xFF;
						char len = 4 + 3;
						xbuff[2] = (len>>8)&0xFF;
						xbuff[3] = len&0xFF;
						xbuff[4] = PTYPE_FILE_DATA;		
						xbuff[5] = (sequence>>24)&0xff;
						xbuff[6] = (sequence>>16)&0xff;
						xbuff[7] = (sequence>>8)&0xff;
						xbuff[8] = sequence&0xff;
						crc = crc16_ccitt(&xbuff[2], len);
						xbuff[len+2] = (crc>>8)&0xFF;
						xbuff[len+3] = crc&0xFF;
						xbuff[len+4] = (FOOTER>>8)&0xFF;
						xbuff[len+5] = FOOTER&0xFF;
						
						stream->puts((char *)xbuff, len+6);
						FileRcvState = READ_FILE_DATA;
	                	retry = 0;    	
	                	tatalretry = 0;
	                }
	                else
	                {
	                	retry ++;		      
	                	if(retry > RETRYTIME)
	                	{
				        	SendMessage(PTYPE_FILE_VIEW, buf, 0, stream);	//request File view
				        	retry = 0;
	                		tatalretry ++;		  
				        }
				        else
				        {
							THEKERNEL->call_event(ON_IDLE);
				        	continue;
				        }
	                }
	                break;
	            case READ_FILE_DATA:
	                seq = (recv_buff[3]<<24) | (recv_buff[4]<<16) | (recv_buff[5]<<8) | recv_buff[6];
	                if ((cmdType == PTYPE_FILE_DATA) && (seq == sequence))
	                {
	                	data_len = ((recv_buff[0]<<8) | recv_buff[1]) - 7;
	                	if(data_len > 8192)
						{
							sprintf(error_msg, "Error: Wrong data len:%d!,retry...\r\n",data_len);
							stream->printf(error_msg);
							break;
						}
	                	// Set the file write system buffer 4096 Byte
			        	setvbuf(fd, (char*)fbuff, _IOFBF, 4096);
						if( fwrite(&recv_buff[7], sizeof(char), data_len, fd) != data_len)
						{
							sprintf(error_msg, "Error: File Write error!retry...\r\n");
							stream->printf(error_msg);
							break;
						}
						fflush(fd);
						u32filesize += data_len;
						
						if(sequence < total_packet)
						{
							sequence += 1;
							xbuff[0] = (HEADER>>8)&0xFF;
							xbuff[1] = HEADER&0xFF;
							char len = 4 + 3;
							xbuff[2] = (len>>8)&0xFF;
							xbuff[3] = len&0xFF;
							xbuff[4] = PTYPE_FILE_DATA;		
							xbuff[5] = (sequence>>24)&0xff;
							xbuff[6] = (sequence>>16)&0xff;
							xbuff[7] = (sequence>>8)&0xff;
							xbuff[8] = sequence&0xff;
							crc = crc16_ccitt(&xbuff[2], len);
							xbuff[len+2] = (crc>>8)&0xFF;
							xbuff[len+3] = crc&0xFF;
							xbuff[len+4] = (FOOTER>>8)&0xFF;
							xbuff[len+5] = FOOTER&0xFF;
							
							stream->puts((char *)xbuff, len+6);				
						}
						else
						{
					        SendMessage(PTYPE_FILE_END, buf, 0, stream);	//the end flag of upload
							FileRcvState = WAIT_MD5;
	                		retry = 0;
	                		
							goto upload_success;
						}
	                	retry = 0;    	
	                	tatalretry = 0;
	                }
	                else
	                {
	                	
						retry ++;				    
	                	if(retry > RETRYTIME)
	                	{
				        	xbuff[0] = (HEADER>>8)&0xFF;
							xbuff[1] = HEADER&0xFF;
							char len = 4 + 3;
							xbuff[2] = (len>>8)&0xFF;
							xbuff[3] = len&0xFF;
							xbuff[4] = PTYPE_FILE_DATA;		
							xbuff[5] = (sequence>>24)&0xff;
							xbuff[6] = (sequence>>16)&0xff;
							xbuff[7] = (sequence>>8)&0xff;
							xbuff[8] = sequence&0xff;
							crc = crc16_ccitt(&xbuff[2], len);
							xbuff[len+2] = (crc>>8)&0xFF;
							xbuff[len+3] = crc&0xFF;
							xbuff[len+4] = (FOOTER>>8)&0xFF;
							xbuff[len+5] = FOOTER&0xFF;
							
							stream->puts((char *)xbuff, len+6);	
							retry = 0;
	                		tatalretry ++;    
						}
				        else
				        {
							THEKERNEL->call_event(ON_IDLE);
				        	continue;
				        }
	                }
	                break;
	            default:
					tatalretry ++;
					THEKERNEL->call_event(ON_IDLE);
                    break;
            }
        }
		else
		{
			retry ++;
			if(retry > RETRYTIME*10)
	        {
				SendMessage(PTYPE_FILE_RETRY, buf, 0, stream);	//resend the last package
				retry = 0;				
	        	tatalretry ++;
				stream->reset();
			}
			
		}
		
		THEKERNEL->call_event(ON_IDLE);		
		if(tatalretry > MAXRETRANS)
		{
			sprintf(error_msg, "Info: Machine receive file too many retry error!\r\n");			
	        SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
            goto upload_error;
        }
        if(us_ticker_read()-starttime > 29000000)
		{
			sprintf(error_msg, "Info: Machine receive file time out!\r\n");			
	        SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
            goto upload_error;
        }
    }
    
upload_error:
	// renable TIME0 and TIME1
	NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
	NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler

	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
		remove(filename.c_str());
	}
	if (fd_md5 != NULL) {
		fclose(fd_md5);
		fd_md5 = NULL;
		remove(md5_filename.c_str());
	}
    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }
    THEKERNEL->set_uploading(false);    
    THEKERNEL->set_cachewait(true);
    safe_delay_ms(1000);
    THEKERNEL->set_cachewait(false);
	stream->printf(error_msg);
	return;
upload_success:

	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
	if (fd_md5 != NULL) {
		fclose(fd_md5);
		fd_md5 = NULL;
	}

    THEKERNEL->set_uploading(false);
	//if file is lzCompress file,then need to Decompress
	start_pos = filename.find(".lz");
	string srcfilename=lzfilename;
	string desfilename= filename;
	if (start_pos != string::npos) {
		desfilename=filename.substr(0, start_pos);
		if(!decompress(srcfilename,desfilename,u32filesize,stream))
		{
			sprintf(error_msg, "error: error in decompressing file!\r\n");	
			goto upload_error;
		}
    }

	// renable TIME0 and TIME1
	NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
	NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler
    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }
	stream->printf("Info: upload success: %s.\r\n", desfilename.c_str());
}


void Player::test_command( string parameters, StreamOutput* stream ) {
    string filename = absolute_from_relative(shift_parameter(parameters));
	FILE *fd = fopen(filename.c_str(), "rb");
	if (NULL != fd) {
        MD5 md5;
        uint8_t md5_buf[64];
        do {
            size_t n = fread(md5_buf, 1, sizeof(md5_buf), fd);
            if (n > 0) md5.update(md5_buf, n);
            THEKERNEL->call_event(ON_IDLE);
        } while (!feof(fd));
        strcpy(md5_str, md5.finalize().hexdigest().c_str());
        fclose(fd);
        fd = NULL;
	}
}

void Player::download_command( string parameters, StreamOutput *stream )
{
    long packetno = 0;
    long file_size = 0;
    long filesendseq = 0;
    char lastcmd = 0;
    char *recv_buff;
    char errorcmd = 0;
	int bufsz = 8192;
    int cmd = 0;
    int c = 0;
    unsigned int len;
    int crc = 0;
    uint32_t starttime;
    bool beretry = false;
    char buf[] = "ok\r\n";

    // open file
	char error_msg[64];
	memset(error_msg, 0, sizeof(error_msg));
	sprintf(error_msg, "Nothing!");
    string filename = absolute_from_relative(shift_parameter(parameters));
    string md5_filename = change_to_md5_path(filename);
    string lz_filename = change_to_lz_path(filename);

	// diasble irq
    if (stream->type() == 0) {
    	bufsz = 128;
    	set_serial_rx_irq(false);
    }
    THEKERNEL->set_uploading(true);

    if (!THECONVEYOR->is_idle()) {
        
		SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
		stream->printf("error: Machine is busy.\r\n");
        if (stream->type() == 0) {
        	set_serial_rx_irq(true);
        }
        THEKERNEL->set_uploading(false);
	    THEKERNEL->set_cachewait(true);
	    safe_delay_ms(1000);
	    THEKERNEL->set_cachewait(false);
        
        return;
    }

    char md5[64];
    memset(md5, 0, sizeof(md5));

    FILE *fd = fopen(md5_filename.c_str(), "rb");
    if (fd != NULL) {
        fread(md5, sizeof(char), 64, fd);
        fclose(fd);
        fd = NULL;
    } else {
    	strcpy(md5, this->md5_str);
    }
	
	fd = fopen(lz_filename.c_str(), "rb");		//first try to open /.lz/filename
	if (NULL == fd) {	
	    fd = fopen(filename.c_str(), "rb");
	    if (NULL == fd) {
			SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
			sprintf(error_msg, "Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
			goto download_error;
	    }
	}
	
	stream->reset();
	
	starttime = us_ticker_read();
	//Send MD5 first
	SendMessage(PTYPE_FILE_MD5, md5, 0, stream);
	lastcmd = PTYPE_FILE_MD5;

	for(;;) {
		cmd = inbytes(stream, &recv_buff, 0, TIMEOUT_MS);
		if (cmd > 0) {			
			starttime = us_ticker_read();

			if( cmd == PTYPE_FILE_RETRY)
			{
				cmd = lastcmd;
				beretry = true;
			}		
			switch (cmd) {
                case PTYPE_FILE_MD5:         
					SendMessage(PTYPE_FILE_MD5, md5, 0, stream);
					lastcmd = PTYPE_FILE_MD5;
					errorcmd = 0;
                    break;
                case PTYPE_FILE_VIEW:
                	fseek(fd, 0, SEEK_END);
					file_size = ftell(fd);
					rewind(fd);
					packetno = file_size/bufsz + ((file_size%bufsz) >0 ? 1: 0) ;	
					xbuff[0] = (HEADER>>8)&0xFF;
					xbuff[1] = HEADER&0xFF;
					len = 6 + 3;
					xbuff[2] = (len>>8)&0xFF;
					xbuff[3] = len&0xFF;
					xbuff[4] = PTYPE_FILE_VIEW;					
					xbuff[5] = (packetno>>24)&0xff;
					xbuff[6] = (packetno>>16)&0xff;
					xbuff[7] = (packetno>>8)&0xff;
					xbuff[8] = packetno&0xff;
					xbuff[9] = (bufsz>>8)&0xff;
					xbuff[10] = bufsz&0xff;
					crc = crc16_ccitt(&xbuff[2], len);
					xbuff[6+5] = (crc>>8)&0xFF;
					xbuff[6+6] = crc&0xFF;
					xbuff[6+7] = (FOOTER>>8)&0xFF;
					xbuff[6+8] = FOOTER&0xFF;
					
					stream->puts((char *)xbuff, len+6);
					lastcmd = PTYPE_FILE_VIEW;
					errorcmd = 0;
                    break;
                case PTYPE_FILE_DATA:
                	if( !beretry )
						filesendseq = recv_buff[3]<<24 | recv_buff[4]<<16 | recv_buff[5]<<8 | recv_buff[6];
					xbuff[0] = (HEADER>>8)&0xFF;
					xbuff[1] = HEADER&0xFF;
					xbuff[4] = PTYPE_FILE_DATA;
					xbuff[5] = (filesendseq >>24) & 0xff;
					xbuff[6] = (filesendseq >>16) & 0xff;
					xbuff[7] = (filesendseq >>8) & 0xff;
					xbuff[8] = filesendseq & 0xff;
					fseek(fd, (filesendseq-1)*bufsz, SEEK_SET);
					c = fread(&xbuff[9], sizeof(char), bufsz, fd);
					if (c <= 0) {
						sprintf(error_msg, "Error: Machine read file error!\r\n");
						SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
						goto download_error;
					}                	
					len = c + 7;
					xbuff[2] = (len>>8)&0xFF;
					xbuff[3] = len&0xFF;
					crc = crc16_ccitt(&xbuff[2], len);
					xbuff[c+9] = (crc>>8)&0xFF;
					xbuff[c+10] = crc&0xFF;
					xbuff[c+11] = (FOOTER>>8)&0xFF;
					xbuff[c+12] = FOOTER&0xFF;
					stream->puts((char *)xbuff, len+6);
					lastcmd = PTYPE_FILE_DATA;
					errorcmd = 0;
					beretry = false;
                    break;
                case PTYPE_FILE_END:
                    SendMessage(PTYPE_FILE_END, buf, 0, stream);
					errorcmd = 0;
                    goto download_success; /* normal end */
                    break;
                case PTYPE_FILE_CAN:
                	sprintf(error_msg, "Info: Download canceled by Controller!\r\n");
					errorcmd = 0;
                    goto download_error;
                    break;
                default:                	
					errorcmd ++;
					THEKERNEL->call_event(ON_IDLE);
                    break;
            }
		}
		else
		{
			if(us_ticker_read()-starttime > 29000000)
			{
				sprintf(error_msg, "Error: Machine received cmd timeout!\r\n");
				SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
                goto download_error;
            }
		}
		if ( errorcmd > MAXRETRANS)
		{
			sprintf(error_msg, "Error: Machine received too many wrong command!\r\n");
			SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
            goto download_error;
		}
		
	}


download_error:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
	if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }
    THEKERNEL->set_uploading(false);
    THEKERNEL->set_cachewait(true);
    safe_delay_ms(1000);
    THEKERNEL->set_cachewait(false);
	stream->printf(error_msg);
	return;
download_success:
	if (fd != NULL) {
		fclose(fd);
		fd = NULL;
	}
    if (stream->type() == 0) {
    	set_serial_rx_irq(true);
    }
    THEKERNEL->set_uploading(false);
	stream->printf("Info: download success: %s.\r\n", filename.c_str());
	return;
}

void Player::SendMessage(char cmd, char* s, int size , StreamOutput *stream)
{	
    int crc = 0;
    unsigned int len = 0;
	size_t total_length = size == 0 ? strlen(s) : size;
	
	xbuff[0] = (HEADER>>8)&0xFF;
	xbuff[1] = HEADER&0xFF;
	xbuff[4] = cmd;
	
	memcpy(&xbuff[5], s, total_length);
	len = total_length + 3;
	xbuff[2] = (len>>8)&0xFF;
	xbuff[3] = len&0xFF;
	crc = crc16_ccitt(&xbuff[2], len);
	xbuff[total_length+5] = (crc>>8)&0xFF;
	xbuff[total_length+6] = crc&0xFF;
	xbuff[total_length+7] = (FOOTER>>8)&0xFF;
	xbuff[total_length+8] = FOOTER&0xFF;
	
	stream->puts((char *)xbuff, len+6);
}
