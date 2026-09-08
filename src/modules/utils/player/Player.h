/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/


#pragma once

#include "Module.h"
#include "PlayerLineSources.h"
#if !defined(STREAMED_JOB_PLAYBACK)
#include "OCodeHandler.h"
#endif

#include <stdio.h>
#include <string>
#include <vector>
#include <queue>
#include <cstdint>

using std::string;

class StreamOutput;
struct player_link_packet;

class Player : public Module {
    public:
        Player();

        void on_module_loaded();
        void on_console_line_received( void* argument );
        void on_main_loop( void* argument );
        void on_second_tick(void* argument);
#if !defined(STREAMED_JOB_PLAYBACK)
        void select_file(string argument, bool force_prescan = false);
        void goto_line_number(unsigned long line_number);
        void play_opened_file();
        void end_of_file();
#endif
        void on_get_public_data(void* argument);
        void on_set_public_data(void* argument);
        void on_gcode_received(void *argument);
        void on_halt(void *argument);

    private:
#if !defined(STREAMED_JOB_PLAYBACK)
        bool prepare_ocode_prescan(StreamOutput* stream, const char* fail_msg);
        void set_current_file(FILE* file);
#endif
        void close_line_source();
        void play_command( string parameters, StreamOutput* stream );
        void progress_command( string parameters, StreamOutput* stream );
        void abort_command( string parameters, StreamOutput* stream );
        void suspend_command( string parameters, StreamOutput* stream , bool pause_outside_play_mode = false);
        void resume_command( string parameters, StreamOutput* stream );
        void save_and_stop_spindle_on_suspend();
        void restore_spindle_on_resume();
        void clear_saved_spindle();
        void save_last_progress(unsigned int unknown_size_percent = 0);
        void dispatch_gcode(const char *gcode_line);
        void goto_command( string parameters, StreamOutput* stream );
        void buffer_command( string parameters, StreamOutput* stream );
        void request_job_abort();
        void finish_job_abort();
#if !defined(STREAMED_JOB_PLAYBACK)
        void upload_command( string parameters, StreamOutput* stream );
        void download_command( string parameters, StreamOutput* stream );
        void test_command(string parameters, StreamOutput* stream );
#endif

        void sync_progress_max();
#if defined(STREAMED_JOB_PLAYBACK)
        void start_streamed_playback(StreamOutput* stream, const string& options);
        void handle_link_packet(const player_link_packet& packet);
        void request_streamed_lines();
        void maintain_streamed_source();
        bool streamed_session_active() const;
        void reset_streamed_playback();
#endif
        
        string extract_options(string& args);

#if !defined(STREAMED_JOB_PLAYBACK)
        void set_serial_rx_irq(bool enable);
        int inbyte(StreamOutput *stream, unsigned int timeout_ms);
        int inbytes(StreamOutput *stream, char **buf, int size, unsigned int timeout_ms);
        void flush_input(StreamOutput *stream);
        void cancel_transfer(StreamOutput *stream);
        int check_crc(int crc, unsigned char *data, unsigned int len);
        int decompress(string sfilename, string dfilename, uint32_t sfilesize, StreamOutput* stream);
#endif
        // 2024
        // bool check_cluster(const char *gcode_str, float *x_value, float *y_value, float *distance, float *slope, float *s_value);
#if !defined(STREAMED_JOB_PLAYBACK)
        void SendMessage(char cmd, char* s, int size , StreamOutput *stream);
#endif

        string filename;
        string last_filename;
        string after_suspend_gcode;
        string before_resume_gcode;
        string on_boot_gcode;
        StreamOutput* current_stream;
        StreamOutput* reply_stream;

#if !defined(STREAMED_JOB_PLAYBACK)
        char md5_str[64];
#endif

        std::queue<string> buffered_queue;
        void clear_buffered_queue();

#if !defined(STREAMED_JOB_PLAYBACK)
        using macro_file_queue_item= std::tuple<std::string, unsigned long>; // allows running macros. This forms a stact filepath, line number, to return to when the internal file is complete
        std::queue<macro_file_queue_item> macro_file_queue;
        void clear_macro_file_queue();
        OCodeHandler ocode_handler;
#endif

#if defined(STREAMED_JOB_PLAYBACK)
        enum class StreamedState : uint8_t {
            idle,
            opening,
            playing,
            seeking,
        };

        StreamedJobBuffer line_source;
        StreamedState streamed_state;
        uint16_t filename_crc;
        uint32_t last_request_line;
        uint32_t streamed_last_request_us;
        bool play_data_resend_pending;
#else
        FileLineSource line_source;
#endif
        // FILE* temp_file_handler;
        long file_size;
        unsigned long played_cnt;
        unsigned long elapsed_secs;
        unsigned long played_lines;
        unsigned long file_line;
        unsigned long goto_line;
        unsigned int playing_lines;
        // last progress when playback finished or was interrupted (for status ? to keep showing |P:...)
        bool has_last_progress;
        unsigned long last_played_lines;
        unsigned int last_percent_complete;
        unsigned long last_elapsed_secs;
        uint8_t current_motion_mode;
        float saved_position[3]; // only saves XYZ
        float saved_spindle_rpm;
        bool saved_spindle_ccw;
        float last_spindle_rpm;
        float slope;
        bool saved_spindle_on;
        bool last_spindle_on;
        bool last_spindle_ccw;
#if !defined(STREAMED_JOB_PLAYBACK)
        bool skip_ocodes_prescan = false;
#endif

        struct {
            bool on_boot_gcode_enable:1;
            bool booted:1;
            bool home_on_boot:1;
            bool playing_file:1;
            bool leave_heaters_on:1;
            bool override_leave_heaters_on:1;
            bool inner_playing:1;
            bool laser_clustering:1;
            bool spindle_suspend_restore_enable:1;
        };
};
