/*
    This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
    Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
    Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
    You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Player.h"
#include "libs/FirmwareFileSystem.h"

#include "libs/Kernel.h"
#include "libs/CRC16.h"
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
#include "modules/robot/Planner.h"
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
#define spindle_suspend_restore_enable_checksum CHECKSUM("spindle_suspend_restore_enable")
#define laser_module_clustering_checksum 	  CHECKSUM("laser_module_clustering")

extern SDFAT mounter;

#define XBUFF_LENGTH	8208
unsigned char xbuff[XBUFF_LENGTH]; /* 2 for data length, 8192 for XModem + 3 head chars + 2 crc + nul */
unsigned char fbuff[4096];

#if !defined(STREAMED_JOB_PLAYBACK)
// used for XMODEM - smoothie
#define SOH  0x01
#define STX  0x02
#define EOT  0x04
#define ACK  0x06
#define NAK  0x15
#define CAN  0x16 //0x18
#define CTRLZ 0x1A


char error_msg[64];
char md5buf[64];
// used for XMODEM 
#define WAIT_MD5  0x01
#define WAIT_FILE_VIEW  0x02
#define READ_FILE_DATA  0x03
#define MAXRETRANS 50
#define RETRYTIME  50
#define TIMEOUT_MS 10
#define RETRYTIMES 10
#endif

#if defined(STREAMED_JOB_PLAYBACK)
namespace {
constexpr std::size_t streamed_line_count = 20;
// Refill once only six of the twenty buffered lines remain.
constexpr std::size_t streamed_refill_threshold = 6;
constexpr uint32_t streamed_retry_us = 5000000;
StreamedJobBuffer::Line streamed_line_storage[streamed_line_count];
}
#endif


Player::Player()
{
    this->playing_file = false;
#if defined(STREAMED_JOB_PLAYBACK)
    this->line_source = StreamedJobBuffer(streamed_line_storage, streamed_line_count);
    this->streamed_state = StreamedState::idle;
    this->filename_crc = 0;
    this->last_request_line = 0;
    this->streamed_last_request_us = 0;
    this->play_data_resend_pending = false;
#else
    this->line_source.attach(nullptr);
#endif
    this->booted = false;
    this->elapsed_secs = 0;
    this->reply_stream = nullptr;
    this->inner_playing = false;
    this->slope = 0.0;
    this->has_last_progress = false;
    this->last_played_lines = 0;
    this->last_percent_complete = 0;
    this->last_elapsed_secs = 0;
    this->saved_spindle_on = false;
    this->saved_spindle_rpm = 0.0f;
    this->spindle_suspend_restore_enable = true;
    this->last_spindle_on = false;
    this->last_spindle_rpm = 0.0f;
    this->last_spindle_ccw = false;
    this->saved_spindle_ccw = false;
    this->file_line = 0;
}

#if !defined(STREAMED_JOB_PLAYBACK)
void Player::set_current_file(FILE *file)
{
    this->line_source.attach(file);
}
#endif

void Player::close_line_source()
{
    this->line_source.close();
}

void Player::save_last_progress(unsigned int unknown_size_percent)
{
    this->last_played_lines = this->played_lines;
    this->last_percent_complete = this->file_size > 0
        ? static_cast<unsigned int>(roundf((this->played_cnt * 100.0F) / this->file_size))
        : unknown_size_percent;
    this->last_elapsed_secs = this->elapsed_secs;
    this->last_filename = this->filename;
    this->has_last_progress = true;
}

void Player::sync_progress_max()
{
    const unsigned long pos = this->line_source.position();
    if(pos > this->played_cnt)
        this->played_cnt = pos;
    if(this->file_line > this->played_lines)
        this->played_lines = this->file_line;
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

    this->on_boot_gcode = THEKERNEL->config->value(on_boot_gcode_checksum)->as_string("/sd/on_boot.gcode");
    this->on_boot_gcode_enable = THEKERNEL->config->value(on_boot_gcode_enable_checksum)->as_bool(false);

    this->home_on_boot = THEKERNEL->config->value(home_on_boot_checksum)->as_bool(true);

    this->after_suspend_gcode = THEKERNEL->config->value(after_suspend_gcode_checksum)->as_string("");
    this->before_resume_gcode = THEKERNEL->config->value(before_resume_gcode_checksum)->as_string("");
    std::replace( this->after_suspend_gcode.begin(), this->after_suspend_gcode.end(), '_', ' '); // replace _ with space
    std::replace( this->before_resume_gcode.begin(), this->before_resume_gcode.end(), '_', ' '); // replace _ with space
    this->leave_heaters_on = THEKERNEL->config->value(leave_heaters_on_suspend_checksum)->as_bool(false);
    this->spindle_suspend_restore_enable = THEKERNEL->config->value(spindle_suspend_restore_enable_checksum)->as_bool(true);

    this->laser_clustering = THEKERNEL->config->value(laser_module_clustering_checksum)->as_bool(false);
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
		this->clear_saved_spindle();
		THEKERNEL->streams->printf("Suspend cleared\n");
	}
}

void Player::on_second_tick(void *)
{
    if(THEKERNEL->is_suspending() || THEKERNEL->is_waiting() || THEKERNEL->is_tool_waiting()) return;
    if(this->playing_file) this->elapsed_secs++;
}

#if !defined(STREAMED_JOB_PLAYBACK)
bool Player::prepare_ocode_prescan(StreamOutput* stream, const char* fail_msg)
{
    this->ocode_handler.reset();
    this->ocode_handler.pre_scan(this->line_source.file(), stream);
    if(this->ocode_handler.pre_scan_failed()) {
        stream->printf("%s\r\n", fail_msg);
        this->close_line_source();
        return false;
    }
    return true;
}
void Player::select_file(string argument, bool force_prescan)
{
    this->filename = argument;

    this->filename.erase((std::remove)(this->filename.begin(), this->filename.end(), '"'), this->filename.end());

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

    if(this->line_source.file() != NULL) {
        this->playing_file = false;
        this->close_line_source();
    }
    this->set_current_file(fwfs::fopen(this->filename.c_str(), "r"));

    if(this->line_source.file() == NULL) {
        THEKERNEL->streams->printf("file.open failed: %s\r\n", this->filename.c_str());
        return;

    } else {
        // get size of file
        int result = fwfs::fseek(this->line_source.file(), 0, SEEK_END);
        if (0 != result) {
            this->file_size = 0;
        } else {
            this->file_size = fwfs::ftell(this->line_source.file());
            fwfs::fseek(this->line_source.file(), 0, SEEK_SET);
        }
        THEKERNEL->streams->printf("File opened:%s Size:%ld\r\n", this->filename.c_str(), this->file_size);

        if(force_prescan || !this->skip_ocodes_prescan) {
            if(!this->prepare_ocode_prescan(THEKERNEL->streams, "File rejected: O-code pre-scan failed")) {
                return;
            }
        } else {
            this->ocode_handler.reset();
        }
        THEKERNEL->streams->printf("File selected\r\n");
    }
    this->played_cnt = 0;
    this->played_lines = 0;
    this->file_line = 0;
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

    // Jumping to an arbitrary line discards any in-progress O-code block stack
    // (it can't be reconstructed from a line number; stray closers afterwards
    // warn rather than halt). The subroutine table is (re)built here, before
    // playback resumes, so no file scan ever happens mid-cut.
    this->ocode_handler.prepare_jump(this->line_source.file(), THEKERNEL->streams);

    // goto file begin
    fwfs::fseek(this->line_source.file(), 0, SEEK_SET);
    played_lines = 0;
    played_cnt   = 0;
    file_line    = 0;

    // Read lines until we've positioned at the target line
    // We want to break BEFORE reading the target line, so the file pointer is at the target
    while (file_line < this->goto_line - 1) {
        if (this->line_source.read(buf, sizeof(buf)) != LineReadResult::data) {
            break; // EOF reached
        }

        if (file_line % 100 == 0) {
            THEKERNEL->call_event(ON_IDLE);
        }
        int len = strlen(buf);
        if (len == 0) continue; // empty line? should not be possible

        file_line++;
        this->sync_progress_max();
    }

    // Keep status report current line in sync with file position
    this->playing_lines = this->played_lines;
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
    if (this->line_source.file() != NULL) {
        this->playing_file = true;
        // this would be a problem if the stream goes away before the file has finished,
        // so we attach it to the kernel stream, however network connections from pronterface
        // do not connect to the kernel streams so won't see this FIXME
        this->reply_stream = THEKERNEL->streams;
    }
}
#endif
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
        // Track spindle state from the job stream so suspend/resume can work
        if (gcode->m == 3) {
            this->last_spindle_on = true;
            this->last_spindle_ccw = false;
            if (gcode->has_letter('S')) {
                this->last_spindle_rpm = gcode->get_value('S');
            }
        } else if (gcode->m == 4) {
            this->last_spindle_on = true;
            this->last_spindle_ccw = true;
            if (gcode->has_letter('S')) {
                this->last_spindle_rpm = gcode->get_value('S');
            }
        } else if (gcode->m == 5) {
            this->last_spindle_on = false;
        }

        if (gcode->m == 1) { //optiional stop
            if (THEKERNEL->get_optional_stop_mode()){
            this->suspend_command((gcode->subcode == 1)?"h":"", gcode->stream);
            }

        }      
        else if (gcode->m == 21) { // Dummy code; makes Octoprint happy -- supposed to initialize SD card
            mounter.remount();
            gcode->stream->printf("SD card ok\r\n");

#if !defined(STREAMED_JOB_PLAYBACK)
        } else if (gcode->m == 23) { // select file
            this->clear_macro_file_queue();
            this->select_file(args, true);
#endif
        } else if (gcode->m == 24) { // start print
#if defined(STREAMED_JOB_PLAYBACK)
            if (this->streamed_session_active()) {
                this->resume_command("", gcode->stream);
            } else {
                gcode->stream->printf("ERROR: File playback is not supported on this machine\r\n");
            }
#else
            this->play_opened_file();
#endif

        } else if (gcode->m == 25) { // pause print
#if defined(STREAMED_JOB_PLAYBACK)
            if (this->streamed_session_active()) {
                this->suspend_command("", gcode->stream);
            }
#else
            this->playing_file = false;
#endif

        }
#if !defined(STREAMED_JOB_PLAYBACK)
        else if (gcode->m == 26) { // Reset print. Slightly different than M26 in Marlin and the rest
            //empty macro queue
            this->clear_macro_file_queue();
            if(this->line_source.file() != NULL) {
                string currentfn = this->filename.c_str();
                unsigned long old_size = this->file_size;

                // abort the print
                abort_command("", gcode->stream);

                if(!currentfn.empty()) {
                    // reload the last file opened
                    this->set_current_file(fwfs::fopen(currentfn.c_str(), "r"));

                    if(this->line_source.file() == NULL) {
                        gcode->stream->printf("file.open failed: %s\r\n", currentfn.c_str());
                    } else {
                        this->current_stream = nullptr;
                        if(this->prepare_ocode_prescan(gcode->stream, "Reset failed: O-code pre-scan failed")) {
                            this->filename = currentfn;
                            this->file_size = old_size;
                        }
                    }
                }
            } else {
                gcode->stream->printf("No file loaded\r\n");
            }
        } else if (gcode->m == 27) { // report print progress, in format used by Marlin
#else
        else if (gcode->m == 27) { // report print progress, in format used by Marlin
#endif
            progress_command("-b", gcode->stream);

        //} else if (gcode->m == 30) { // end file implementation for M30 returning from macros. M99 is the proper formatting.
        //    this->end_of_file();
#if !defined(STREAMED_JOB_PLAYBACK)
        } else if (gcode->m == 32) { // select file and start print
            
            
            //empty macro queue
            this->clear_macro_file_queue();

            this->select_file(args, true);

            this->play_opened_file();           

        } else if (gcode->m == 97) {
            if (gcode->has_letter('P')) {
                this->goto_line_number(gcode->get_value('P'));
                return;
            }else{
                    //error: no filepath found
                    THEKERNEL->streams->printf("ERROR: M97 Command missing P parameter for line to goto, aborting \n");
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
                    THEKERNEL->streams->printf("ERROR: invalid number in M98 command \n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(MANUAL);
                    return;
                }
            }
            if (gcode->has_letter('L')) {
                num_repeats = floor(gcode->get_value('L'));
                if (num_repeats <1){
                    //error:
                    THEKERNEL->streams->printf("ERROR: M98 command has an invalid value, which will lead to errors \n");
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
                    THEKERNEL->streams->printf("ERROR: no filepath found in M98.1 command \n");
                    THEKERNEL->call_event(ON_HALT, nullptr);
                    THEKERNEL->set_halt_reason(MANUAL);
                    return;
                }
            }

            //TODO: test new_filepath length to make sure it is valid
            // Resume on the line after M98: file_line is already the M98 line
            std::tuple<std::string, unsigned long> queueItem (this->filename, this->file_line + 1);
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
#endif
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
                this->clear_saved_spindle();
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
        bool pause_outside_play_mode = (possible_command.find_first_of("5") != string::npos);
        this->suspend_command(possible_command, new_message.stream, pause_outside_play_mode);
    }else if (cmd == "resume") {
        this->resume_command(possible_command, new_message.stream);
    }else if (cmd == "goto") {
    	this->goto_command( possible_command, new_message.stream );
    }else if (cmd == "buffer") {
    	this->buffer_command( possible_command, new_message.stream );
    }else if (cmd == "upload") {
#if defined(STREAMED_JOB_PLAYBACK)
        new_message.stream->printf("ERROR: local file transfers are not available on this machine\r\n");
#else
    	this->upload_command( possible_command, new_message.stream );
#endif
    }else if (cmd == "download") {
#if defined(STREAMED_JOB_PLAYBACK)
        new_message.stream->printf("ERROR: local file transfers are not available on this machine\r\n");
#else
        memset(md5_str, 0, sizeof(md5_str));
    	if (possible_command.find("config.txt") != string::npos) {
        	this->test_command( possible_command, new_message.stream );
    	}
    	this->download_command( possible_command, new_message.stream );
#endif
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
    if (!THEROBOT->is_homed_all_axes()) {
		return;
	}
    // extract any options from the line and terminate the line there
    string options= extract_options(parameters);
#if defined(STREAMED_JOB_PLAYBACK)
    if (options.find_first_of("Oo") != string::npos) {
        stream->printf("ERROR: O-code programs are not supported on this machine\r\n");
        return;
    }
#else
    this->skip_ocodes_prescan = (options.find_first_of("Oo") == string::npos);
#endif
    // Get filename which is the entire parameter line upto any options found or entire line
    this->filename = absolute_from_relative(shift_parameter(parameters));
    this->last_filename = this->filename;

    if (this->playing_file || THEKERNEL->is_suspending() || THEKERNEL->is_waiting()
#if defined(STREAMED_JOB_PLAYBACK)
        || this->streamed_session_active()
#endif
    ) {
        stream->printf("Currently printing, abort print first\r\n");
        return;
    }

#if defined(STREAMED_JOB_PLAYBACK)
    this->start_streamed_playback(stream, options);
#else

    if (this->line_source.file() != NULL) { // must have been a paused print
        this->close_line_source();
    }

//    this->temp_file_handler = fopen ("/sd/gcodes/temp.nc", "w");
//    if (this->temp_file_handler == NULL) {
//        stream->printf("File open error: /sd/gcodes/temp.nc\n");
//        return;
//    }

    //empty macro queue
    this->clear_macro_file_queue();

    this->set_current_file(fwfs::fopen(this->filename.c_str(), "r"));
    if(this->line_source.file() == NULL) {
        stream->printf("File not found: %s\r\n", this->filename.c_str());
        return;
    }

    if(!this->skip_ocodes_prescan) {
        if(!this->prepare_ocode_prescan(stream, "Playback aborted: O-code pre-scan failed")) {
            return;
        }
    } else {
        this->ocode_handler.reset();
    }

    if(THEKERNEL->is_halted()) return;

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
    int result = fwfs::fseek(this->line_source.file(), 0, SEEK_END);
    if (0 != result) {
        stream->printf("WARNING - Could not get file size\r\n");
        file_size = 0;
    } else {
        file_size = fwfs::ftell(this->line_source.file());
        fwfs::fseek(this->line_source.file(), 0, SEEK_SET);
        stream->printf("  File size %ld\r\n", file_size);
    }
    this->played_cnt = 0;
    this->played_lines = 0;
    this->file_line = 0;
    this->elapsed_secs = 0;
    this->playing_lines = 0;
    this->goto_line = 0;
    this->has_last_progress = false;  // new job started, stop reporting previous job's last progress

    // force into absolute mode
    THEROBOT->absolute_mode = true;
    THEROBOT->e_absolute_mode = true;

    // reset current position;
    THEROBOT->reset_position_from_current_actuator_position();
#endif
}

#if defined(STREAMED_JOB_PLAYBACK)
void Player::start_streamed_playback(StreamOutput *stream, const string& options)
{
    this->close_line_source();
    this->filename_crc = crc16::ccitt(
        reinterpret_cast<const uint8_t *>(this->filename.data()), this->filename.size());
    this->streamed_state = StreamedState::opening;
    this->play_data_resend_pending = false;
    this->current_stream = options.find_first_of("Vv") == string::npos
        ? nullptr : THEKERNEL->streams;

    char request_data[2];
    makera::write_be16(request_data, this->filename_crc);
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_START, request_data, sizeof(request_data));
    this->streamed_last_request_us = us_ticker_read();
    stream->printf("Playing %s\r\n", this->filename.c_str());
}

void Player::request_streamed_lines()
{
    if (!this->line_source.is_open() || this->line_source.at_end()) return;

    const uint32_t line = this->line_source.next_expected_line();
    const uint16_t count = static_cast<uint16_t>(this->line_source.available());
    char request_data[8];
    makera::write_be16(request_data, this->filename_crc);
    makera::write_be32(request_data + 2, line);
    makera::write_be16(request_data + 6, count);
    THEKERNEL->serial->PacketMessage(PTYPE_PLAY_DATA, request_data, sizeof(request_data));
    this->last_request_line = line;
    this->streamed_last_request_us = us_ticker_read();
    this->play_data_resend_pending = false;
}

void Player::maintain_streamed_source()
{
    if (this->streamed_state == StreamedState::opening) {
        if (us_ticker_read() - this->streamed_last_request_us > streamed_retry_us) {
            char request_data[2];
            makera::write_be16(request_data, this->filename_crc);
            THEKERNEL->serial->PacketMessage(PTYPE_PLAY_START, request_data, sizeof(request_data));
            this->streamed_last_request_us = us_ticker_read();
        }
        return;
    }
    if (this->streamed_state == StreamedState::seeking) {
        if (us_ticker_read() - this->streamed_last_request_us > streamed_retry_us) {
            char request_data[6];
            makera::write_be16(request_data, this->filename_crc);
            makera::write_be32(request_data + 2, this->goto_line);
            THEKERNEL->serial->PacketMessage(PTYPE_GOTO_START, request_data, sizeof(request_data));
            this->streamed_last_request_us = us_ticker_read();
        }
        return;
    }
    if (this->line_source.queued() > streamed_refill_threshold ||
        this->line_source.at_end()) return;

    const uint32_t expected = this->line_source.next_expected_line();
    if (!this->play_data_resend_pending && expected == this->last_request_line &&
        us_ticker_read() - this->streamed_last_request_us <= streamed_retry_us) return;
    this->request_streamed_lines();
}

bool Player::streamed_session_active() const
{
    return this->streamed_state != StreamedState::idle;
}

void Player::handle_link_packet(const player_link_packet& packet)
{
    if (packet.type == PTYPE_PLAY_VIEW) {
        if (this->streamed_state != StreamedState::opening || packet.data_length < 6) return;
        const uint16_t filename_crc = makera::read_be16(packet.data);
        if (filename_crc != this->filename_crc) {
            THEKERNEL->streams->printf("ERROR: streamed job identity mismatch\r\n");
            this->streamed_state = StreamedState::idle;
            THEKERNEL->serial->PacketMessage(PTYPE_PLAY_CAN, nullptr, 0);
            return;
        }
        const uint32_t size = makera::read_be32(packet.data + 2);
        this->line_source.begin(filename_crc);
        this->file_size = size;
        this->played_cnt = 0;
        this->played_lines = 0;
        this->file_line = 0;
        this->elapsed_secs = 0;
        this->playing_lines = 0;
        this->goto_line = 0;
        this->has_last_progress = false;
        this->streamed_state = StreamedState::playing;
        this->playing_file = true;
        THEROBOT->absolute_mode = true;
        THEROBOT->e_absolute_mode = true;
        THEROBOT->reset_position_from_current_actuator_position();
        this->request_streamed_lines();
        return;
    }

    if (packet.type == PTYPE_PLAY_DATA) {
        if (packet.data_length < 6 || !this->line_source.is_open()) return;
        const uint16_t filename_crc = makera::read_be16(packet.data);
        const uint32_t first_line = makera::read_be32(packet.data + 2);
        const auto result = this->line_source.append_lines(
            filename_crc, first_line, packet.data + 6, packet.data_length - 6);
        if (result == StreamedJobBuffer::AppendResult::line_too_long) {
            THEKERNEL->streams->printf("ERROR: streamed job line exceeds 128 bytes\r\n");
            THEKERNEL->serial->PacketMessage(PTYPE_PLAY_CAN, nullptr, 0);
            this->reset_streamed_playback();
        } else if (result == StreamedJobBuffer::AppendResult::unexpected_line ||
                   result == StreamedJobBuffer::AppendResult::queue_full) {
            this->play_data_resend_pending = true;
        }
        return;
    }

    if (packet.type == PTYPE_GOTO_LINES) {
        if (this->streamed_state != StreamedState::seeking || packet.data_length < 10) return;
        const uint16_t filename_crc = makera::read_be16(packet.data);
        if (filename_crc != this->filename_crc) return;
        const uint32_t line = makera::read_be32(packet.data + 2);
        const uint32_t offset = makera::read_be32(packet.data + 6);
        this->played_lines = line;
        this->file_line = line;
        this->playing_lines = line;
        this->played_cnt = offset;
        this->streamed_last_request_us = us_ticker_read();
        if (line < this->goto_line - 1) return;
        this->line_source.begin(filename_crc, line, offset);
        this->streamed_state = StreamedState::playing;
        this->request_streamed_lines();
        return;
    }

    if (packet.type == PTYPE_PLAY_END) {
        this->line_source.mark_end(this->filename_crc);
    } else if (packet.type == PTYPE_PLAY_CAN) {
        if (!this->streamed_session_active() || THEKERNEL->is_aborted()) return;
        this->save_last_progress();
        if (THEKERNEL->is_suspending()) {
            THEKERNEL->set_waiting(false);
            THEKERNEL->set_suspending(false);
            THEROBOT->pop_state();
            this->clear_saved_spindle();
        }
        this->reset_streamed_playback();
        this->request_job_abort();
    }
}

void Player::reset_streamed_playback()
{
    this->streamed_state = StreamedState::idle;
    this->playing_file = false;
    this->play_data_resend_pending = false;
    this->close_line_source();
    this->played_cnt = 0;
    this->played_lines = 0;
    this->file_line = 0;
    this->playing_lines = 0;
    this->goto_line = 0;
    this->file_size = 0;
    this->filename.clear();
    this->current_stream = nullptr;
    this->clear_buffered_queue();
}
#endif

void Player::request_job_abort()
{
    THEKERNEL->conveyor->request_motion_abort();
    THEKERNEL->set_aborted(true);
}

void Player::finish_job_abort()
{
    struct SerialMessage message;
    message.message = "M5";
    message.stream = THEKERNEL->streams;
    message.line = 0;
    THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);

#if defined(STREAMED_JOB_PLAYBACK)
    if (THEKERNEL->get_laser_mode()) {
        message.message = "laserabort";
        THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
    }
#endif

    THEROBOT->reset_position_from_current_actuator_position();
    THEKERNEL->planner->reset_after_abort();
    THEKERNEL->conveyor->clear_motion_abort();
    THEKERNEL->set_aborted(false);
#if defined(STREAMED_JOB_PLAYBACK)
    this->streamed_state = StreamedState::idle;
#endif
    THEKERNEL->streams->printf("Aborted playing or paused file. \r\n");
}

// Goto a certain line when playing a file
void Player::goto_command( string parameters, StreamOutput *stream )
{
    if (!THEKERNEL->is_suspending()) {
        stream->printf("Can only jump when pausing!\r\n");
        return;
    }

#if defined(STREAMED_JOB_PLAYBACK)
    if (this->streamed_state != StreamedState::playing || !this->line_source.is_open()) {
        stream->printf("No streamed job is active\r\n");
        return;
    }
    string line_str = shift_parameter(parameters);
    if (line_str.empty()) return;
    char *end = nullptr;
    this->goto_line = strtoul(line_str.c_str(), &end, 10);
    this->goto_line = this->goto_line < 1 ? 1 : this->goto_line;
    char request_data[6];
    makera::write_be16(request_data, this->filename_crc);
    makera::write_be32(request_data + 2, this->goto_line);
    this->line_source.close();
    this->streamed_state = StreamedState::seeking;
    THEKERNEL->serial->PacketMessage(PTYPE_GOTO_START, request_data, sizeof(request_data));
    this->streamed_last_request_us = us_ticker_read();
    stream->printf("Goto line %lu...\r\n", this->goto_line);
#else

    if (this->line_source.file() == NULL) {
    	stream->printf("Missing file handle!\r\n");
    	return;
    }

    string line_str = shift_parameter(parameters);
    if (!line_str.empty()) {
        char *ptr = NULL;
        this->goto_line = strtol(line_str.c_str(), &ptr, 10);
        this->goto_line_number(this->goto_line);
        
    }
#endif
}

void Player::progress_command( string parameters, StreamOutput *stream )
{

    // get options
    string options = shift_parameter( parameters );
    bool sdprinting= options.find_first_of("Bb") != string::npos;

    if(!playing_file && line_source.is_open()) {
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
    if (THEKERNEL->is_aborted()) {
        stream->printf("Abort already pending\r\n");
        return;
    }

    PublicData::set_value( atc_handler_checksum, abort_checksum, nullptr );

    if(!playing_file && !line_source.is_open()
#if defined(STREAMED_JOB_PLAYBACK)
        && !this->streamed_session_active()
#endif
    ) {
        stream->printf("Not currently playing\r\n");
        return;
    }

#if defined(STREAMED_JOB_PLAYBACK)
    if (this->streamed_session_active()) {
        this->save_last_progress();
        THEKERNEL->serial->PacketMessage(PTYPE_PLAY_CAN, nullptr, 0);
        this->reset_streamed_playback();
        if (THEKERNEL->is_suspending()) {
            THEKERNEL->set_waiting(false);
            THEKERNEL->set_suspending(false);
            THEROBOT->pop_state();
            this->clear_saved_spindle();
        }
        this->request_job_abort();
        return;
    }
#endif

    // save last progress so status (?) continues to show |P:played_lines,percent_complete,elapsed_secs|
    this->save_last_progress();

    this->current_stream = NULL;

    //smoothie
    this->playing_file = false;
    this->played_cnt = 0;
    this->played_lines = 0;
    this->file_line = 0;
    this->playing_lines = 0;
    this->goto_line = 0;
    this->file_size = 0;
    this->clear_buffered_queue();
#if !defined(STREAMED_JOB_PLAYBACK)
    this->clear_macro_file_queue();
    this->ocode_handler.reset();
#endif
    this->filename = "";
    // end smoothie

    this->close_line_source();

    THEKERNEL->set_suspending(false);
    if (parameters.empty()) {
        THEKERNEL->set_waiting(false);
        this->request_job_abort();
        return;
    }

    THEKERNEL->set_waiting(true);

    // wait for queue to empty
    THEKERNEL->conveyor->wait_for_idle();


    if (communication_protocol == PROTOCOL_SMOOTHIE) {
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

    } else {
        if(THEKERNEL->is_halted()) {
            THEKERNEL->streams->printf("Aborted by halt\n");
            THEKERNEL->set_waiting(false);
            this->playing_file = false;
            this->played_cnt = 0;
            this->played_lines = 0;
            this->playing_lines = 0;
            this->goto_line = 0;
            this->file_size = 0;
            this->clear_buffered_queue();
            this->filename = "";
        }
        else
        {
            THEKERNEL->set_waiting(false);
            // turn off spindle
            {
                struct SerialMessage message;
                message.message = "M5";
                message.stream = THEKERNEL->streams;
                message.line = 0;
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            }
            this->playing_file = false;
            this->played_cnt = 0;
            this->played_lines = 0;
            this->playing_lines = 0;
            this->goto_line = 0;
            this->file_size = 0;
            this->clear_buffered_queue();
            this->filename = "";
        }
    }
}

void Player::clear_buffered_queue(){
	while (!this->buffered_queue.empty()) {
		this->buffered_queue.pop();
	}
}

#if !defined(STREAMED_JOB_PLAYBACK)
void Player::clear_macro_file_queue(){
	while (!this->macro_file_queue.empty()) {
		this->macro_file_queue.pop();
	}
}
#endif

void Player::on_main_loop(void *argument)
{
    if (THEKERNEL->is_aborted()) {
        if (THEKERNEL->conveyor->is_idle()) {
            this->finish_job_abort();
        }
        return;
    }
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
            StreamOutput *stream = THEKERNEL->serial != nullptr
                                 ? static_cast<StreamOutput *>(THEKERNEL->serial)
                                 : &StreamOutput::NullStream;
            this->play_command(this->on_boot_gcode, stream);
        }

    }

#if defined(STREAMED_JOB_PLAYBACK)
    this->maintain_streamed_source();
#endif

    if ( this->playing_file ) {
        if(THEKERNEL->is_halted() || THEKERNEL->is_suspending() || THEKERNEL->is_waiting() || this->inner_playing) {
            return;
        }

#if defined(STREAMED_JOB_PLAYBACK)
        this->maintain_streamed_source();
#endif

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

        uint32_t last_idle_us = us_ticker_read();
        LineReadResult read_result;
        while ((read_result = this->line_source.read(buf, sizeof(buf))) == LineReadResult::data) {

            int len = strlen(buf);
            if (len == 0) continue; // empty line? should not be possible
            if (buf[len - 1] == '\n' || this->line_source.at_end()) {
                if(discard) { // we are discarding a long line
                    discard = false;
                    continue;
                }

                if (len == 1) {
                    this->file_line++;
                    this->sync_progress_max();
                    continue; // empty line
                }

                this->file_line++;

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

                // O-code flow control: intercept O-prefixed lines before dispatch
                struct SerialMessage message;
                message.message = buf;
                message.stream = this->current_stream == nullptr ? &(StreamOutput::NullStream) : this->current_stream;

#if !defined(STREAMED_JOB_PLAYBACK)
                if (this->line_source.file() != nullptr &&
                    this->ocode_handler.process_line(buf, this->line_source.file(), message.stream, this->file_line)) {
                    this->sync_progress_max();
                    if (this->ocode_handler.is_skipping()) {
                        // Fast-forwarding through a skipped block stays in this
                        // loop without returning, so feed the watchdog periodically.
                        uint32_t now_us = us_ticker_read();
                        if((now_us - last_idle_us) >= 200000) {
                            THEKERNEL->call_event(ON_IDLE);
                            last_idle_us = now_us;
                        }
                        continue;
                    }
                    return;
                }
                if (this->line_source.file() != nullptr && this->ocode_handler.is_skipping()) {
                    this->sync_progress_max();
                    uint32_t now_us = us_ticker_read();
                    if((now_us - last_idle_us) >= 200000) {
                        THEKERNEL->call_event(ON_IDLE);
                        last_idle_us = now_us;
                    }
                    continue;
                }
#endif

                if (this->current_stream != nullptr) {
                    this->current_stream->printf("%s", buf);
                }
                message.line = this->file_line;

                // waits for the queue to have enough room
                // this->current_stream->printf("Run: %s", buf);
                THEKERNEL->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
                // fputs(buf, this->temp_file_handler);
                // THEKERNEL->streams->printf("0-[Line: %d] %s\n", message.line, buf);
                this->sync_progress_max();
                //M335 disables line by line, M336 Enables. Pauses after every valid gcode line
                if (THEKERNEL->get_line_by_line_exec_mode() && len > 2 && buf[0] != ';' && buf[0] != '('){
                    this->suspend_command("", THEKERNEL->streams);
                }

                return; // we feed one line per main loop

            } else {
                // discard long line
                if (!discard && this->current_stream != nullptr) {
                    this->current_stream->printf("Warning: Discarded long line\n");
                }
                discard = true;

                uint32_t now_us = us_ticker_read();
                if ((now_us - last_idle_us) >= 200000) {
                    THEKERNEL->call_event(ON_IDLE);
                    last_idle_us = now_us;
                }
            }
        }

        if (read_result == LineReadResult::waiting) {
#if defined(STREAMED_JOB_PLAYBACK)
            this->maintain_streamed_source();
#endif
            return;
        }
#if defined(STREAMED_JOB_PLAYBACK)
        if (!THEKERNEL->conveyor->is_idle()) return;
#endif

        // save last progress so status (?) continues to show |P:played_lines,percent_complete,elapsed_secs|
        this->save_last_progress(100);

#if defined(STREAMED_JOB_PLAYBACK)
        THEKERNEL->serial->PacketMessage(PTYPE_PLAY_END, nullptr, 0);
#endif

#if !defined(STREAMED_JOB_PLAYBACK)
        this->ocode_handler.reset();
#endif
        this->playing_file = false;
        this->filename = "";
        played_cnt = 0;
        played_lines = 0;
        file_line = 0;
        playing_lines = 0;
        goto_line = 0;
        file_size = 0;

        this->close_line_source();
#if defined(STREAMED_JOB_PLAYBACK)
        this->streamed_state = StreamedState::idle;
#endif

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
                // Only advance played lines, never go backward
                if (block != nullptr && block->is_ready && block->is_g123 && block->line > this->playing_lines) {
                	this->playing_lines = block->line;
                }
                p.played_lines = this->playing_lines;
        	} else {
        		p.played_lines = this->played_lines;
        	}
            p.elapsed_secs = this->elapsed_secs;
            float pcnt = (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size;
            p.percent_complete = roundf(pcnt);
            p.filename = this->filename;
            p.is_playing = true;
            p.parsed_lines = this->played_lines;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        } else if(file_size > 0 && !playing_file) {
            // paused: show current progress (elapsed_secs does not increment while paused)
            p.played_lines = this->played_lines;
            p.elapsed_secs = this->elapsed_secs;
            float pcnt = (file_size > 0) ? (((float)file_size - (file_size - played_cnt)) * 100.0F) / file_size : 0;
            p.percent_complete = (unsigned int)roundf(pcnt);
            p.filename = this->filename;
            p.is_playing = false;
            p.parsed_lines = this->played_lines;
            pdr->set_data_ptr(&p);
            pdr->set_taken();
        } else if(this->has_last_progress) {
            // playback finished or was interrupted: show last progress (elapsed_secs frozen at stop time)
            p.played_lines = this->last_played_lines;
            p.percent_complete = this->last_percent_complete;
            p.elapsed_secs = this->last_elapsed_secs;
            p.filename = this->last_filename;
            p.is_playing = false;
            p.parsed_lines = this->last_played_lines;
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
        // Resync when entering or leaving ATC so status report current line does not jump
        if (b != this->inner_playing && this->playing_file) {
            this->playing_lines = this->played_lines;
        }
    	this->inner_playing = b;
    	if (this->playing_file) pdr->set_taken();
    } else if (pdr->second_element_is(restart_job_checksum)) {
    	if (!this->last_filename.empty()) {
    		THEKERNEL->streams->printf("Job restarted: %s.\r\n", this->last_filename.c_str());
    		// Quote the filename to handle spaces properly
    		string quoted_filename = "\"" + this->last_filename + "\"";
        	this->play_command(quoted_filename, &(StreamOutput::NullStream));
    	}
    } else if (pdr->second_element_is(suspend_play_checksum)) {
        bool pause_outside_play_mode = false;
        if (pdr->get_data_ptr() != nullptr) {
            pause_outside_play_mode = *static_cast<bool *>(pdr->get_data_ptr());
        }
        this->suspend_command("", &(StreamOutput::NullStream), pause_outside_play_mode);
        pdr->set_taken();
    } else if (pdr->second_element_is(resume_play_checksum)) {
        this->resume_command("", &(StreamOutput::NullStream));
        pdr->set_taken();
#if defined(STREAMED_JOB_PLAYBACK)
    } else if (pdr->second_element_is(link_packet_checksum)) {
        this->handle_link_packet(*static_cast<player_link_packet *>(pdr->get_data_ptr()));
        pdr->set_taken();
#endif
    }
}

void Player::dispatch_gcode(const char *gcode_line)
{
    Gcode gcode(gcode_line, &(StreamOutput::NullStream));
    THEKERNEL->call_event(ON_GCODE_RECEIVED, &gcode);
}

void Player::clear_saved_spindle()
{
    this->saved_spindle_on = false;
    this->saved_spindle_rpm = 0.0f;
    this->saved_spindle_ccw = false;
}

void Player::save_and_stop_spindle_on_suspend()
{
    this->clear_saved_spindle();

    if (!this->spindle_suspend_restore_enable) {
        return;
    }

    if (THEKERNEL->get_laser_mode()) {
        return;
    }

    if (this->last_spindle_on) {
        this->saved_spindle_on = true;
        this->saved_spindle_rpm = this->last_spindle_rpm;
        this->saved_spindle_ccw = this->last_spindle_ccw;
        this->dispatch_gcode("M5");
    }
}

void Player::restore_spindle_on_resume()
{
    if (!this->spindle_suspend_restore_enable) {
        this->clear_saved_spindle();
        return;
    }

    if (!this->saved_spindle_on || THEKERNEL->get_laser_mode()) {
        this->clear_saved_spindle();
        return;
    }

    // Spindle already on (e.g. user sent M3/M4 while paused); keep their setting.
    if (this->last_spindle_on) {
        this->clear_saved_spindle();
        return;
    }

    const bool ccw = this->saved_spindle_ccw;
    char buf[32];
    if (this->saved_spindle_rpm > 0.0f) {
        snprintf(buf, sizeof(buf), "%s S%.0f", ccw ? "M4" : "M3", this->saved_spindle_rpm);
    } else {
        snprintf(buf, sizeof(buf), "%s", ccw ? "M4" : "M3");
    }
    this->clear_saved_spindle();
    this->dispatch_gcode(buf);

    this->last_spindle_on = true;
    this->last_spindle_ccw = ccw;
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

    this->playing_lines = this->played_lines;
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

    this->save_and_stop_spindle_on_suspend();

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
#if defined(STREAMED_JOB_PLAYBACK)
    if (this->streamed_state == StreamedState::seeking) {
        stream->printf("Stream seek is still pending\n");
        return;
    }
#endif

    stream->printf("Resuming playing...\n");

    if(THEKERNEL->is_halted()) {
        THEKERNEL->streams->printf("Resume aborted by kill\n");
        THEROBOT->pop_state();
        this->clear_saved_spindle();
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

    this->restore_spindle_on_resume();

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
        this->clear_saved_spindle();
        THEKERNEL->set_suspending(false);
        return;
    }

	THEKERNEL->set_suspending(false);

	stream->printf("Playing file resumed\n");
}

#if !defined(STREAMED_JOB_PLAYBACK)
int Player::check_crc(int crc, unsigned char *data, unsigned int len)
{
    if (crc) {
        unsigned short crc = crc16::ccitt(data, len);
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

void Player::flush_input(StreamOutput *stream)
{
    while (inbyte(stream, TIMEOUT_MS) >= 0)
        continue;
}

void Player::cancel_transfer(StreamOutput *stream)
{
	stream->_putc(CAN);
	//stream->_putc(CAN);
	//stream->_putc(CAN);
	flush_input(stream);
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
    char error_msg[80]; //makera
    if (communication_protocol == PROTOCOL_MAKERA) {
        memset(error_msg, 0, sizeof(error_msg));
        sprintf(error_msg, "decompress!");
    }
	f_in= fwfs::fopen(sfilename.c_str(), "rb");
	f_out= fwfs::fopen(dfilename.c_str(), "w+");
	if (f_in == NULL || f_out == NULL)
	{
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            memset(fbuff, 0, sizeof(fbuff));
		    sprintf((char*)fbuff, "Error: failed to create file [%s]!\r\n", filename.substr(0, 30).c_str());
        } else {
            sprintf(error_msg, "Error: failed to create file [%s]!\r\n", filename.substr(0, 30).c_str());
        }
		goto _exit;
	}
	for(i = 0; i < sfilesize-2; i+= BLOCK_HEADER_SIZE + u32BlockSize)
	{

		fwfs::fread(u8ReadBuffer_hdr, sizeof(char), BLOCK_HEADER_SIZE, f_in);
		u32BlockSize = u8ReadBuffer_hdr[0] * (1 << 24) + u8ReadBuffer_hdr[1] * (1 << 16) + u8ReadBuffer_hdr[2] * (1 << 8) + u8ReadBuffer_hdr[3];
		if(!u32BlockSize)
		{
            if (communication_protocol == PROTOCOL_SMOOTHIE) {
			    sprintf(error_msg, "Error: decompress file error,bad block num.");
            }
            goto _exit;
		}
		fwfs::fread(xbuff, sizeof(char), u32BlockSize, f_in);
		u32DcmprsSize = qlz_decompress((const char *)xbuff, fbuff, &s_stDecompressState);
		if(!u32DcmprsSize)
		{
            if (communication_protocol == PROTOCOL_SMOOTHIE) {
                sprintf(error_msg, "Error: decompress file error,bad decompress size.");
            }
			goto _exit;
		}
		for(j = 0; j < u32DcmprsSize; j++)
		{
			u16Sum += fbuff[j];
		}
		// Set the file write system buffer 4096 Byte
		setvbuf(f_out, (char*)&xbuff[4096], _IOFBF, 4096);
		fwfs::fwrite(fbuff, sizeof(char),u32DcmprsSize, f_out);
		u32TotalDcmprsSize += u32DcmprsSize;
		u32BlockNum += 1;
		if(++k>10)
		{
			k=0;
			THEKERNEL->call_event(ON_IDLE);
            if (communication_protocol == PROTOCOL_SMOOTHIE) {
                memset(fbuff, 0, sizeof(fbuff));
                sprintf((char*)fbuff, "#Info: decompart = %lu\r\n", u32BlockNum);
                stream->printf((char*)fbuff);
            } else {
                sprintf(error_msg, "#Info: decompart = %lu\r\n", u32BlockNum);
			    stream->printf(error_msg);
            }
		}
	}
	fwfs::fread(fbuff, sizeof(char), 2, f_in);
	if(u16Sum != ((fbuff[0] <<8) + fbuff[1]))
	{
        if (communication_protocol == PROTOCOL_MAKERA) {
		    sprintf(error_msg, "Error: decompress file sum check error.");
        }
        goto _exit;
	}

	if (f_in != NULL)
		fwfs::fclose(f_in);
	if (f_out!= NULL)
		fwfs::fclose(f_out);
    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        memset(fbuff, 0, sizeof(fbuff));
        sprintf((char*)fbuff, "#Info: decompart = %lu\r\n", u32BlockNum);
        stream->printf((char*)fbuff);
    } else {
        sprintf(error_msg, "#Info: decompart = %lu\r\n", u32BlockNum);
        stream->printf(error_msg);
    }
	return 1;
_exit:
	if (f_in != NULL)
		fwfs::fclose(f_in );
	if (f_out != NULL)
		fwfs::fclose(f_out);
	if (communication_protocol == PROTOCOL_SMOOTHIE) {
        stream->printf((char*)fbuff);
    } else {
        stream->printf(error_msg);
    }
	return 0;
}

void Player::upload_command( string parameters, StreamOutput *stream )
{
    uint32_t u32filesize = 0;
    char *recv_buff;
    int retry = 0;
    int crc = 0;

    //smoothie
    unsigned char *p;
    int retrans = MAXRETRANS;
    int timeouts = MAXRETRANS;
    int recv_count = 0;
    bool md5_received = false;
    int bufsz = 0, is_stx = 0;
    unsigned char trychar = 'C';
    unsigned char packetno = 1;
    int c, len = 0;

    //makera
    char FileRcvState = WAIT_MD5;
    int cmdType = 0;
    int tatalretry = 0;
    uint32_t total_packet = 0;
    uint16_t packet_size = 0;
    uint16_t data_len = 0;
    uint32_t sequence = 0;
    uint32_t starttime;
    uint32_t seq;
    char buf[] = "ok\r\n";
    // open file
    char error_msg[64]; //Smoothie

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
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            stream->_putc(EOT);
        } else {
            SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
        }
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
    	fd = fwfs::fopen(lzfilename.c_str(), "wb");
    }
    else {
    	fd = fwfs::fopen(filename.c_str(), "wb");
    }
		
    FILE *fd_md5 = NULL;
    //if file is lzCompress file,then need to Decompress
	start_pos = md5_filename.find(".lz");
	if (start_pos != string::npos) {
		md5_filename=md5_filename.substr(0, start_pos);
	}


    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        if (filename.find("firmware.bin") == string::npos) {
            fd_md5 = fwfs::fopen(md5_filename.c_str(), "wb");
        }
        if (fd == NULL || (filename.find("firmware.bin") == string::npos && fd_md5 == NULL)) {
            stream->_putc(EOT);    
            sprintf(error_msg, "Error: failed to open file [%s]!\r\n", fd == NULL ? filename.substr(0, 30).c_str() : md5_filename.substr(0, 30).c_str() );
            goto upload_error;
        }
    } else {
        fd_md5 = fwfs::fopen(md5_filename.c_str(), "wb");
        if (fd == NULL || fd_md5 == NULL) {
            SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
            sprintf(error_msg, "Error: failed to open file [%s]!\r\n", fd == NULL ? filename.substr(0, 30).c_str() : md5_filename.substr(0, 30).c_str() );
            goto upload_error;
        }
    }


    
	
	// stop TIMER0 and TIMER1 for save time
	NVIC_DisableIRQ(TIMER0_IRQn);
	NVIC_DisableIRQ(TIMER1_IRQn);
    if (communication_protocol == PROTOCOL_MAKERA) {
        starttime = us_ticker_read();
        stream->reset();
    }

    for (;;) {
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            for (retry = 0; retry < MAXRETRANS; ++retry) {  // approx 3 seconds allowed to make connection
                if (trychar)
                    stream->_putc(trychar);
                if ((c = inbyte(stream, TIMEOUT_MS)) >= 0) {
                    retry = 0;
                    switch (c) {
                    case SOH:
                        bufsz = 128;
                        is_stx = 0;
                        goto start_recv;
                    case STX:
                        bufsz = 8192;
                        is_stx = 1;
                        goto start_recv;
                    case EOT:
                        stream->_putc(ACK);
                        flush_input(stream);
                        goto upload_success; /* normal end */
                    case CAN:
                        if ((c = inbyte(stream, TIMEOUT_MS)) == CAN) {
                            stream->_putc(ACK);
                            flush_input(stream);
                            sprintf(error_msg, "Info: Upload canceled by remote!\r\n");
                            goto upload_error;
                        }
                        goto upload_error;
                        break;
                    default:
                        break;
                    }
                }
                else
                {
                    safe_delay_ms(10);
                }
            }

            if (trychar == 'C') {
                trychar = NAK;
                continue;
            }
            cancel_transfer(stream);
            sprintf(error_msg, "Error: upload sync error! get char [%d], retry [%d]!\r\n", c, retry);
            goto upload_error;

        start_recv:
            if (trychar == 'C')
                crc = 1;
            trychar = 0;
            p = xbuff;
            *p++ = c;

            recv_count = 1 + bufsz + (crc ? 1 : 0) + 3 + is_stx;

            timeouts = MAXRETRANS;

            while (recv_count > 0) {
                c = inbytes(stream, &recv_buff, recv_count, TIMEOUT_MS);
                if (c < 0) {
                    safe_delay_ms(10);
                    timeouts --;
                    if (timeouts < 0) {
                        goto reject;
                    }
                } else {
                    timeouts = MAXRETRANS;
                    for (int i = 0; i < c; i ++) {
                        *p++ = recv_buff[i];
                    }
                    recv_count -= c;
                }
            }
            len = is_stx ? (xbuff[3] << 8 | xbuff[4]) : xbuff[3];
            if (!md5_received && xbuff[1] == 0 && xbuff[1] == (unsigned char)(~xbuff[2])
                    && check_crc(crc, &xbuff[3], bufsz + 1 + is_stx) && len == 32) {
                // received md5
                if (NULL != fd_md5) {
                    fwfs::fwrite(&xbuff[4 + is_stx], sizeof(char), 32, fd_md5);
                }
                THEKERNEL->call_event(ON_IDLE);
                stream->_putc(ACK);
                md5_received = true;
                continue;
            } else if (xbuff[1] == (unsigned char)(~xbuff[2]) &&
                    xbuff[1] == packetno && check_crc(crc, &xbuff[3], bufsz + 1 + is_stx)) {

                // Set the file write system buffer 4096 Byte
                setvbuf(fd, (char*)fbuff, _IOFBF, 4096);
                fwfs::fwrite(&xbuff[4 + is_stx], sizeof(char), len, fd);
                u32filesize += len;
                ++ packetno;
                retrans = MAXRETRANS + 1;
                THEKERNEL->call_event(ON_IDLE);
                stream->_putc(ACK);
                continue;
            }
        reject:
            stream->_putc(NAK);
            if (-- retrans <= 0) {
                cancel_transfer(stream);
                sprintf(error_msg, "Error: too many retry error!\r\n");
                goto upload_error; /* too many retry error */
            }
        } else { //Makera protocol
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
                            crc = crc16::ccitt(&xbuff[2], len);
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
                                crc = crc16::ccitt(&xbuff[2], len);
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
                                crc = crc16::ccitt(&xbuff[2], len);
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
        }
    }
upload_error:
	// renable TIME0 and TIME1
	NVIC_EnableIRQ(TIMER0_IRQn);     // Enable interrupt handler
	NVIC_EnableIRQ(TIMER1_IRQn);     // Enable interrupt handler

	if (fd != NULL) {
		fwfs::fclose(fd);
		fd = NULL;
		fwfs::remove(filename.c_str());
	}
	if (fd_md5 != NULL) {
		fwfs::fclose(fd_md5);
		fd_md5 = NULL;
		fwfs::remove(md5_filename.c_str());
	}
    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        flush_input(stream);
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
		fwfs::fclose(fd);
		fd = NULL;
	}
	if (fd_md5 != NULL) {
		fwfs::fclose(fd_md5);
		fd_md5 = NULL;
	}
    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        flush_input(stream);
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
            if (communication_protocol == PROTOCOL_MAKERA) { 
                sprintf(error_msg, "error: error in decompressing file!\r\n");	
            }
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
	FILE *fd = fwfs::fopen(filename.c_str(), "rb");
	if (NULL != fd) {
        MD5 md5;
        uint8_t smoothie_md5_buf[64];
        do {
            if (communication_protocol == PROTOCOL_SMOOTHIE) { 
                size_t n = fwfs::fread(smoothie_md5_buf, 1, sizeof(smoothie_md5_buf), fd);
                if (n > 0) md5.update(smoothie_md5_buf, n);
            } else {
                size_t n = fwfs::fread(md5buf, 1, sizeof(md5buf), fd);
                if (n > 0) md5.update(md5buf, n);
            }
            THEKERNEL->call_event(ON_IDLE);
        } while (!fwfs::feof(fd));
        strcpy(md5_str, md5.finalize().hexdigest().c_str());
        fwfs::fclose(fd);
        fd = NULL;
	}
}

void Player::download_command( string parameters, StreamOutput *stream )
{
    int c = 0;
    int bufsz = 8192;
    int crc = 0;

    //smoothie
    
    int is_stx = 1;
    unsigned char smooth_packetno = 0;
    int i = 0;
    int retry = 0;
    bool resend = true;

    //makera
	long packetno = 0;
    long file_size = 0;
    long filesendseq = 0;
    char lastcmd = 0;
    char *recv_buff;
    char errorcmd = 0;
    int cmd = 0;
    
    unsigned int len;
    uint32_t starttime;
    bool beretry = false;
    char buf[] = "ok\r\n";

    // open file
    char error_msg[64]; //Smoothie
    unsigned char md5_sent = 0; //smoothie

	memset(error_msg, 0, sizeof(error_msg));
    if (communication_protocol == PROTOCOL_MAKERA) { 
        sprintf(error_msg, "Nothing!");
    }
    string filename = absolute_from_relative(shift_parameter(parameters));
    string md5_filename = change_to_md5_path(filename);
    string lz_filename = change_to_lz_path(filename);

	// diasble irq
    if (stream->type() == 0) {
    	bufsz = 128;
        if (communication_protocol == PROTOCOL_SMOOTHIE) { 
            is_stx = 0;
        }
    	set_serial_rx_irq(false);
    }
    THEKERNEL->set_uploading(true);

    if (!THECONVEYOR->is_idle()) {
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            cancel_transfer(stream);
        } else {
            SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
		    stream->printf("error: Machine is busy.\r\n");
        }
        if (stream->type() == 0) {
        	set_serial_rx_irq(true);
        }
        THEKERNEL->set_uploading(false);
	    THEKERNEL->set_cachewait(true);
	    safe_delay_ms(1000);
	    THEKERNEL->set_cachewait(false);
        
        return;
    }
    char md5[64]; //Smoothie need to replace with md5buf
    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        memset(md5, 0, sizeof(md5));
    } else {
        memset(md5buf, 0, sizeof(md5buf));
    }
    

    FILE *fd = fwfs::fopen(md5_filename.c_str(), "rb");
    if (fd != NULL) {
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            fwfs::fread(md5, sizeof(char), 64, fd);
        } else {
            fwfs::fread(md5buf, sizeof(char), 64, fd);
        }
        fwfs::fclose(fd);
        fd = NULL;
    } else {
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            strcpy(md5, this->md5_str);
        } else {
            strcpy(md5buf, this->md5_str);
        }
    }
	
	fd = fwfs::fopen(lz_filename.c_str(), "rb");		//first try to open /.lz/filename
	if (NULL == fd) {	
	    fd = fwfs::fopen(filename.c_str(), "rb");
	    if (NULL == fd) {
            if (communication_protocol == PROTOCOL_SMOOTHIE) {
                cancel_transfer(stream);
            } else {
                SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
            }
			sprintf(error_msg, "Error: failed to open file [%s]!\r\n", filename.substr(0, 30).c_str());
			goto download_error;
	    }
	}


    if (communication_protocol == PROTOCOL_MAKERA) {
        stream->reset();
        starttime = us_ticker_read();
        //Send MD5 first
        SendMessage(PTYPE_FILE_MD5, md5buf, 0, stream);
        lastcmd = PTYPE_FILE_MD5;
    }




    for(;;) {
        if (communication_protocol == PROTOCOL_SMOOTHIE) {
            for (retry = 0; retry < MAXRETRANS; ++retry) {
                if ((c = inbyte(stream, TIMEOUT_MS)) >= 0) {
                    retry = 0;
                    switch (c) {
                    case 'C':
                        crc = 1;
                        goto start_trans;
                    case NAK:
                        crc = 0;
                        goto start_trans;
                    case CAN:
                        if ((c = inbyte(stream, TIMEOUT_MS)) == CAN) {
                            stream->_putc(ACK);
                            flush_input(stream);
                            sprintf(error_msg, "Info: canceled by remote!\r\n");
                            goto download_error;
                        }
                        break;
                    default:
                        break;
                    }
                }
                else {
				    safe_delay_ms(10);
			    }


                }
            cancel_transfer(stream);
            sprintf(error_msg, "Error: download sync error! get char [%02X], retry [%d]!\r\n", c, retry);
            goto download_error;

            for(;;) {
            start_trans:
                if (smooth_packetno == 0 && md5_sent == 0) {
                    c = strlen(md5);
                    memcpy(&xbuff[4 + is_stx], md5, c);
                    md5_sent = 1;
                } else {
                    c = fwfs::fread(&xbuff[4 + is_stx], sizeof(char), bufsz, fd);
                    if (c <= 0) {
                        for (retry = 0; retry < MAXRETRANS; ++retry) {
                            stream->_putc(EOT);
                            if ((c = inbyte(stream, TIMEOUT_MS)) == ACK) break;
                        }
                        flush_input(stream);
                        if (c == ACK) {
                            goto download_success;
                        } else {
                            sprintf(error_msg, "Error: get finish ACK error!\r\n");
                            goto download_error;
                        }
                    }
                }
                xbuff[0] = is_stx ? STX : SOH;
                xbuff[1] = smooth_packetno;
                xbuff[2] = ~smooth_packetno;
                xbuff[3] = is_stx ? c >> 8 : c;
                if (is_stx) {
                    xbuff[4] = c & 0xff;
                }
                if (c < bufsz) {
                    memset(&xbuff[4 + is_stx + c], CTRLZ, bufsz - c);
                }

                if (crc) {
                    unsigned short ccrc = crc16::ccitt(&xbuff[3], bufsz + 1 + is_stx);
                    xbuff[bufsz + 4 + is_stx] = (ccrc >> 8) & 0xFF;
                    xbuff[bufsz + 5 + is_stx] = ccrc & 0xFF;
                } else {
                    unsigned char ccks = 0;
                    for (i = 3; i < bufsz + 1 + is_stx; ++i) {
                        ccks += xbuff[i];
                    }
                    xbuff[bufsz + 4 + is_stx] = ccks;
                }

                resend = true;
                for (retry = 0; retry < MAXRETRANS; ++retry) {
                    if (resend) {
                        stream->puts((char *)xbuff, bufsz + 5 + is_stx + (crc ? 1:0));
                        resend = false;
                    }
                    if ((c = inbyte(stream, TIMEOUT_MS)) >= 0 ) {
                        retry = 0;
                        switch (c) {
                        case ACK:
                            ++smooth_packetno;
                            goto start_trans;
                        case CAN:
                            if ((c = inbyte(stream, TIMEOUT_MS)) == CAN) {
                                stream->_putc(ACK);
                                flush_input(stream);
                                sprintf(error_msg, "Info: canceled by remote!\r\n");
                                goto download_error;
                            }
                            break;
                        case NAK:
                            resend = true;
                        default:
                            break;
                        }
                    }
                    else
                    {
                        safe_delay_ms(500);
                    }

                }

                cancel_transfer(stream);
                sprintf(error_msg, "Error: transmit error, char: [%d], retry: [%d]!\r\n", c, retry);
                goto download_error;
            }
        } else{
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
                        SendMessage(PTYPE_FILE_MD5, md5buf, 0, stream);
                        lastcmd = PTYPE_FILE_MD5;
                        errorcmd = 0;
                        break;
                    case PTYPE_FILE_VIEW:
                        fwfs::fseek(fd, 0, SEEK_END);
                        file_size = fwfs::ftell(fd);
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
                        crc = crc16::ccitt(&xbuff[2], len);
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
                        fwfs::fseek(fd, (filesendseq-1)*bufsz, SEEK_SET);
                        c = fwfs::fread(&xbuff[9], sizeof(char), bufsz, fd);
                        if (c <= 0) {
                            sprintf(error_msg, "Error: Machine read file error!\r\n");
                            SendMessage(PTYPE_FILE_CAN, buf, sizeof(buf), stream);
                            goto download_error;
                        }                	
                        len = c + 7;
                        xbuff[2] = (len>>8)&0xFF;
                        xbuff[3] = len&0xFF;
                        crc = crc16::ccitt(&xbuff[2], len);
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
    }
download_error:
	if (fd != NULL) {
		fwfs::fclose(fd);
		fd = NULL;
	}
    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        flush_input(stream);
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
		fwfs::fclose(fd);
		fd = NULL;
	}
    if (communication_protocol == PROTOCOL_SMOOTHIE) {
        flush_input(stream);
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
	crc = crc16::ccitt(&xbuff[2], len);
	xbuff[total_length+5] = (crc>>8)&0xFF;
	xbuff[total_length+6] = crc&0xFF;
	xbuff[total_length+7] = (FOOTER>>8)&0xFF;
	xbuff[total_length+8] = FOOTER&0xFF;
	stream->puts((char *)xbuff, len+6);
}
#endif
