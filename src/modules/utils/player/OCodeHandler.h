#pragma once

#include <stdio.h>
#include <string>
#include <map>

class StreamOutput;

// Maximum O-code nesting depth
#define OCODE_MAX_STACK_DEPTH 10

// Handles LinuxCNC-compatible O-code flow control:
//  - conditional: if / elseif / else / endif
//  - loops: while / endwhile, do / while, repeat / endrepeat
//  - control flow: break, continue
//  - subroutines: sub / endsub / return / call
//
// Integration with Player:
//   1. Call reset() + pre_scan() after opening the file (before playback starts)
//      or call prepare_jump() before resuming/jumping to a line.
//   2. For each fgets'd line, call process_line() before dispatching.
//      Returns true -> line was an O-code, do not dispatch it.
//      Returns false -> normal G-code line, call is_skipping() before dispatching
//   3. While is_skipping() is true, discard normal lines (stay in inner fgets loop).
//   4. Call reset() on abort.
class OCodeHandler {
    public:
        OCodeHandler();

        void reset();

        // One linear pass to build the subroutine-offset table. Preserves the
        // caller's file position. Run before playback (never mid-cut).
        void pre_scan(FILE* fh, StreamOutput* stream);

        // True when pre_scan() found duplicate subroutine definitions.
        bool pre_scan_failed() const { return pre_scan_failed_; }

        // Discard the block stack and ensure the subroutine table is built,
        // for use before a line jump / resume.
        void prepare_jump(FILE* fh, StreamOutput* stream);

        // Process one raw line from the G-code file.
        // Returns true if the line was an O-code and was consumed (do not dispatch).
        // Returns false if the line is a normal G-code line.
        bool process_line(const char* line, FILE* fh, StreamOutput* stream, unsigned long& file_line);

        // Returns true when inside a false branch or skipping a subroutine body.
        bool is_skipping() const;

    private:
        enum class BlockType { IF, WHILE, DO, REPEAT, SUB };

        struct Frame {
            int       num;
            BlockType type;
            long      loop_offset;    // fseek byte offset for loop-back (while/do/repeat)
            bool      executing;      // true -> this block is currently active
            bool      branch_taken;   // for if/elseif chains: has any branch fired yet?
            int       repeat_count;   // remaining iterations for repeat
            float     saved_params[30]; // saved #1-#30 on subroutine call
            long      return_offset;  // byte offset to return to after endsub/return
            int       jump_line;      // line to restore on loop-back or sub return (type-specific)
        };

        // Exposes only the subset of the std::vector API used below; depth is
        // bounded by OCODE_MAX_STACK_DEPTH (push sites guard against overflow).
        struct FrameStack {
            Frame* data  = nullptr;
            int    count = 0;
            int  size() const                 { return count; }
            void clear()                       { count = 0; }
            Frame&       operator[](int i)       { return data[i]; }
            const Frame& operator[](int i) const { return data[i]; }
            Frame* begin()                     { return data; }
            void push_back(const Frame& f)     { data[count++] = f; }
            void resize(int n)                 { count = n; } // only ever shrinks here
            void erase(Frame* it) {
                int i = (int)(it - data);
                for(int k = i; k + 1 < count; ++k) data[k] = data[k + 1];
                count--;
            }
        };

        struct SubEntry {
            long offset; // byte offset of the line after "Onnn sub"
            int  line;   // 1-based line of first sub body line
        };

        static Frame frame_storage_[OCODE_MAX_STACK_DEPTH];
        FrameStack stack_;
        std::map<int, SubEntry> sub_table_; // ocode_num -> sub body entry point
        bool pre_scanned_ = false;          // sub_table_ built lazily on first call
        bool pre_scan_failed_ = false;      // duplicate sub definitions in pre_scan
        bool tolerant_after_jump_ = false;   // warn instead of halt on label mismatch

        // Parse an O-code line; fills num, keyword (lower-case), and rest.
        // Returns false if the line does not begin with O followed by digits.
        static bool parse_ocode(const char* line, int& num, std::string& keyword, std::string& rest);

        float eval_expr(std::string& expr, StreamOutput* stream) const;

        // Scan forward in fh until a matching keyword is found at nesting depth 0.
        // open_kw increments depth; close_kw decrements / matches. Leaves the file
        // pointer just past the matched line. matched receives the keyword that hit.
        // When target_num >= 0, only the close_kw at depth 0 with that O-number matches.
        // When matched_rest is non-null, it receives the expression text after the keyword.
        bool skip_to(FILE* fh, const std::string& open_kw, const std::string& close_kw, std::string& matched, int& lines_read, int target_num = -1, std::string* matched_rest = NULL) const;

        void halt_error(StreamOutput* stream, const char* fmt, ...) const;
        void warn(StreamOutput* stream, const char* fmt, ...) const;
        void label_error(StreamOutput* stream, const char* fmt, ...) const;

        bool any_frame_skipping() const;
        int  find_loop_frame(int num) const;
};
