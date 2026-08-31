#include "OCodeHandler.h"

#include "libs/FirmwareFileSystem.h"
#include "libs/StreamOutput.h"
#include "libs/StreamOutputPool.h"
#include "libs/Kernel.h"
#include "libs/utils.h"
#include "modules/communication/utils/Gcode.h"

#include "mbed.h"

#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <math.h>

using std::string;
using std::map;

OCodeHandler::Frame OCodeHandler::frame_storage_[OCODE_MAX_STACK_DEPTH];

OCodeHandler::OCodeHandler()
{
    stack_.data  = frame_storage_;
    stack_.count = 0;
}

void OCodeHandler::reset()
{
    stack_.clear();
    sub_table_.clear();
    pre_scanned_ = false;
    pre_scan_failed_ = false;
    tolerant_after_jump_ = false;
}

// One linear pass to build sub_table_. Preserves the caller's file position so
// it can be run lazily (on the first subroutine call) rather than upfront.
void OCodeHandler::pre_scan(FILE* fh, StreamOutput* stream)
{
    pre_scanned_ = true;
    if(!fh) return;

    long saved = fwfs::ftell(fh);
    fwfs::fseek(fh, 0, SEEK_END);
    long file_size = fwfs::ftell(fh);
    fwfs::fseek(fh, 0, SEEK_SET);
    sub_table_.clear();

    if(file_size > 0) {
        THEKERNEL->streams->printf("O-code: pre-scan started...\r\n");
    }

    char buf[130];
    int line_num = 0;
    int last_pct = -1;
    uint32_t last_idle_us = us_ticker_read();
    while(fwfs::fgets(buf, sizeof(buf), fh) != NULL) {
        // Keep the system responsive (and the watchdog fed) while scanning a
        // large file. This runs before playback motion starts, so yielding here
        // is safe; it must never happen mid-cut.
        uint32_t now_us = us_ticker_read();
        if((now_us - last_idle_us) >= 200000) {
            THEKERNEL->call_event(ON_IDLE);
            if(THEKERNEL->is_halted()) {
                THEKERNEL->streams->printf("O-code: pre-scan aborted by halt\r\n");
                pre_scanned_     = false;
                pre_scan_failed_ = true;
                fwfs::fseek(fh, saved, SEEK_SET);
                return;
            }
            if(file_size > 0) {
                int pct = (int)((fwfs::ftell(fh) * 100L) / file_size);
                if(pct > last_pct) {
                    last_pct = pct;
                    THEKERNEL->streams->printf("O-code: pre-scan progress %d%%\r\n", pct);
                }
            }
            last_idle_us = now_us;
        }
        line_num++;

        int n;
        string kw, rest;
        if(parse_ocode(buf, n, kw, rest)) {
            if(kw == "sub") {
                if(sub_table_.count(n)) {
                    THEKERNEL->streams->printf("ERROR: O-code error: O%d sub defined more than once\n", n);
                    pre_scan_failed_ = true;
                } else {
                    // Store the offset of the line *after* "Onnn sub" so a call can
                    // jump straight into the body instead of re-reading the sub line
                    // (which the definition path would otherwise skip).
                    SubEntry entry;
                    entry.offset = fwfs::ftell(fh);
                    entry.line   = line_num + 1;
                    sub_table_[n] = entry;
                }
            }
        }
    }

    fwfs::fseek(fh, saved, SEEK_SET);

    if(file_size > 0 && last_pct < 100) {
        THEKERNEL->streams->printf("O-code: pre-scan complete\r\n");
    }

    if(!sub_table_.empty()) {
        stream->printf("O-code: pre-scan found %d subroutine(s)\n", (int)sub_table_.size());
    }
}

// Prepare for a line jump / resume. The block stack cannot be reconstructed from
// a bare line number, so discard it (stray closers afterwards warn, not halt).
// The subroutine table is kept if already built, or built now (upfront, before
// playback resumes) so a scan never has to run mid-cut.
void OCodeHandler::prepare_jump(FILE* fh, StreamOutput* stream)
{
    stack_.clear();
    tolerant_after_jump_ = true;
    if(!pre_scanned_) pre_scan(fh, stream);
}

// Extract O-code number, lower-cased keyword, and remaining text from a raw line.
// Returns false when the line does not start with O followed by digits.
bool OCodeHandler::parse_ocode(const char* line, int& num, string& keyword, string& rest)
{
    const char* p = ltrim_cstr(line);
    if(*p != 'O') return false;
    p++;

    char* end;
    long n = strtol(p, &end, 10);
    if(end == p) return false;
    num = (int)n;
    p = end;

    p = ltrim_cstr(p);
    if(!*p) return false;

    const char* kw_start = p;
    while(*p && !isspace((unsigned char)*p) && *p != '[') p++;
    keyword = lc(string(kw_start, p));

    p = ltrim_cstr(p);
    rest = p;
    while(!rest.empty() && (rest[rest.size()-1] == '\n' || rest[rest.size()-1] == '\r'))
        rest.resize(rest.size()-1);

    return !keyword.empty();
}

float OCodeHandler::eval_expr(string& expr, StreamOutput* stream) const
{
    char* buf = expr.empty() ? const_cast<char*>("") : &expr[0];
    char* endptr = NULL;
    return Gcode::evaluate_standalone_expression(buf, &endptr, stream);
}

void OCodeHandler::halt_error(StreamOutput* stream, const char* fmt, ...) const
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    // Always emit to the kernel streams so the message appears in the MDI
    // regardless of whether the player's current_stream is NullStream.
    THEKERNEL->streams->printf("ERROR: O-code error: %s\n", buf);
    THEKERNEL->set_halt_reason(MANUAL);
    THEKERNEL->call_event(ON_HALT, NULL);
}

// Non-fatal diagnostic. Used for stray/unmatched closers (e.g. an "endif" with no
// open "if"), which can legitimately occur when playback resumes in the middle of
// a block (goto/line-resume rewinds without rebuilding the block stack). Halting
// the machine in that case would be a regression, so we warn and continue instead.
void OCodeHandler::warn(StreamOutput* stream, const char* fmt, ...) const
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    stream->printf("O-code warning: %s\n", buf);
}

void OCodeHandler::label_error(StreamOutput* stream, const char* fmt, ...) const
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if(tolerant_after_jump_)
        warn(stream, "%s (ignored)", buf);
    else
        halt_error(stream, "%s", buf);
}

bool OCodeHandler::any_frame_skipping() const
{
    for(int i = 0; i < (int)stack_.size(); i++) {
        if(!stack_[i].executing) return true;
    }
    return false;
}

bool OCodeHandler::is_skipping() const
{
    return any_frame_skipping();
}

int OCodeHandler::find_loop_frame(int num) const
{
    for(int i = (int)stack_.size() - 1; i >= 0; i--) {
        BlockType t = stack_[i].type;
        if((t == BlockType::WHILE || t == BlockType::DO || t == BlockType::REPEAT) && stack_[i].num == num)
            return i;
    }
    return -1;
}

// Scan forward from the current file position until a matching O-code keyword
// is found at nesting depth 0. Handles depth via open_kw / close_kw.
// Leaves the file pointer just past the matched line.
bool OCodeHandler::skip_to(FILE* fh,
                            const string& open_kw,
                            const string& close_kw,
                            string& matched,
                            int& lines_read,
                            int target_num,
                            string* matched_rest) const
{
    int depth = 0;
    lines_read = 0;
    const bool track_do = (open_kw == "while");
    int do_nums[OCODE_MAX_STACK_DEPTH];
    int do_count = 0;
    char buf[130];
    uint32_t last_idle_us = us_ticker_read();
    while(fwfs::fgets(buf, sizeof(buf), fh) != NULL) {
        // Skipping a large block happens synchronously within a single main-loop
        // tick, so feed the watchdog (and let other ON_IDLE work run) periodically.
        lines_read++;
        uint32_t now_us = us_ticker_read();
        if((now_us - last_idle_us) >= 200000) {
            THEKERNEL->call_event(ON_IDLE);
            last_idle_us = now_us;
        }

        int n;
        string kw, rest;
        if(!parse_ocode(buf, n, kw, rest)) continue;

        if(track_do && kw == "do") {
            if(do_count < OCODE_MAX_STACK_DEPTH)
                do_nums[do_count++] = n;
            continue;
        }

        if(kw == open_kw) {
            bool is_do_close = false;
            if(track_do) {
                for(int i = do_count - 1; i >= 0; i--) {
                    if(do_nums[i] == n) {
                        do_nums[i] = do_nums[--do_count];
                        is_do_close = true;
                        break;
                    }
                }
            }
            if(!is_do_close)
                depth++;
            continue;
        }

        if(kw == close_kw) {
            if(depth == 0) {
                if(target_num < 0 || n == target_num) {
                    matched = kw;
                    if(matched_rest) *matched_rest = rest;
                    return true;
                }
                continue;
            }
            depth--;
        }
    }
    return false;
}

// Process one raw line from the G-code file.
// Returns true when the line was an O-code (consumed); false for normal G-code.
bool OCodeHandler::process_line(const char* line, FILE* fh, StreamOutput* stream, unsigned long& file_line)
{
    int num;
    string keyword, rest;

    if(!parse_ocode(line, num, keyword, rest)) return false;

    // SUB definition: skip the body when reached during normal execution flow.
    // The body only runs on an explicit call.
    if(keyword == "sub") {
        if(!any_frame_skipping()) {
            string matched;
            int lines_read = 0;
            if(!skip_to(fh, "sub", "endsub", matched, lines_read, num))
                halt_error(stream, "O%d sub has no matching endsub", num);
            file_line += lines_read;
        }
        return true;
    }

    // ENDSUB / RETURN: return from a subroutine call.
    if(keyword == "endsub" || keyword == "return") {
        // A return inside a false if-branch (or any skipped block) must be ignored,
        // just like break/continue. A natural terminal endsub is never skipping.
        if(any_frame_skipping()) return true;
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::SUB) {
                for(int p = 0; p < 30; p++)
                    THEKERNEL->local_params[p] = stack_[i].saved_params[p];
                file_line = (unsigned long)stack_[i].jump_line - 1;
                long ret = stack_[i].return_offset;
                stack_.resize(i);
                fwfs::fseek(fh, ret, SEEK_SET);
                return true;
            }
        }
        return true;
    }

    // CALL: invoke a numbered subroutine.
    if(keyword == "call") {
        if(any_frame_skipping()) return true;

        if((int)stack_.size() >= OCODE_MAX_STACK_DEPTH) {
            halt_error(stream, "O%d call exceeded max nesting depth %d", num, OCODE_MAX_STACK_DEPTH);
            return true;
        }

        if(!pre_scanned_) {
            stream->printf("Warning: O%d call encountered without pre-scan, scanning now (may delay)\r\n", num);
            pre_scan(fh, stream);
            if(pre_scan_failed_) {
                halt_error(stream, "O%d call: O-code pre-scan failed", num);
                return true;
            }
        }

        map<int,SubEntry>::iterator it = sub_table_.find(num);
        if(it == sub_table_.end()) {
            halt_error(stream, "O%d call: subroutine not found", num);
            return true;
        }

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::SUB;
        frame.loop_offset   = 0;
        frame.executing     = true;
        frame.branch_taken  = false;
        frame.repeat_count  = 0;
        frame.return_offset = fwfs::ftell(fh);
        frame.jump_line     = (int)file_line + 1;

        for(int p = 0; p < 30; p++)
            frame.saved_params[p] = THEKERNEL->local_params[p];

        // Parse bracketed arguments [a1] [a2] ... (pass full [expr] so the
        // expression parser consumes the closing bracket, same as if/while).
        char* rp = rest.empty() ? const_cast<char*>("") : &rest[0];
        for(int p = 0; p < 30; p++) {
            rp = const_cast<char*>(ltrim_cstr(rp));
            if(*rp != '[') break;
            char* endp = NULL;
            float val = Gcode::evaluate_standalone_expression(rp, &endp, stream);
            THEKERNEL->local_params[p] = val;
            rp = endp ? endp : rp + 1;
        }

        tolerant_after_jump_ = false;
        stack_.push_back(frame);
        file_line = (unsigned long)it->second.line - 1;
        fwfs::fseek(fh, it->second.offset, SEEK_SET);
        return true;
    }

    // IF
    if(keyword == "if") {
        if((int)stack_.size() >= OCODE_MAX_STACK_DEPTH) {
            halt_error(stream, "O%d if exceeded max nesting depth %d", num, OCODE_MAX_STACK_DEPTH);
            return true;
        }

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::IF;
        frame.loop_offset   = 0;
        frame.repeat_count  = 0;
        frame.return_offset = 0;
        frame.jump_line     = 0;

        if(!any_frame_skipping()) {
            float val = eval_expr(rest, stream);
            frame.executing    = (val != 0.0f && !isnan(val));
            frame.branch_taken = frame.executing;
        } else {
            frame.executing    = false;
            frame.branch_taken = true; // prevent elseif/else inside a false parent
        }
        tolerant_after_jump_ = false;
        stack_.push_back(frame);
        return true;
    }

    // ELSEIF
    if(keyword == "elseif") {
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::IF && stack_[i].num == num) {
                if(stack_[i].branch_taken) {
                    stack_[i].executing = false;
                } else {
                    float val = eval_expr(rest, stream);
                    bool cond = (val != 0.0f && !isnan(val));
                    stack_[i].executing    = cond;
                    stack_[i].branch_taken = cond;
                }
                return true;
            }
        }
        label_error(stream, "O%d elseif without matching if", num);
        return true;
    }

    // ELSE
    if(keyword == "else") {
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::IF && stack_[i].num == num) {
                stack_[i].executing    = !stack_[i].branch_taken;
                stack_[i].branch_taken = true;
                return true;
            }
        }
        label_error(stream, "O%d else without matching if", num);
        return true;
    }

    // ENDIF
    if(keyword == "endif") {
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::IF && stack_[i].num == num) {
                stack_.erase(stack_.begin() + i);
                return true;
            }
        }
        label_error(stream, "O%d endif without matching if", num);
        return true;
    }

    // WHILE (may also close a do-while if a matching DO frame exists)
    if(keyword == "while") {
        // Check for a matching DO frame first (closing "while" of a do-while)
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::DO && stack_[i].num == num) {
                if(!any_frame_skipping()) {
                    float val = eval_expr(rest, stream);
                    bool cond = (val != 0.0f && !isnan(val));
                    if(cond) {
                        file_line = (unsigned long)stack_[i].jump_line - 1;
                        fwfs::fseek(fh, stack_[i].loop_offset, SEEK_SET);
                    } else
                        stack_.erase(stack_.begin() + i);
                } else {
                    stack_.erase(stack_.begin() + i);
                }
                return true;
            }
        }

        // Standalone while
        if((int)stack_.size() >= OCODE_MAX_STACK_DEPTH) {
            halt_error(stream, "O%d while exceeded max nesting depth %d", num, OCODE_MAX_STACK_DEPTH);
            return true;
        }

        // Store the byte offset of this "while" line so we can re-evaluate on endwhile.
        long while_offset = fwfs::ftell(fh) - (long)strlen(line);

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::WHILE;
        frame.loop_offset   = while_offset;
        frame.repeat_count  = 0;
        frame.return_offset = 0;
        frame.branch_taken  = false;
        frame.jump_line     = (int)file_line;

        if(!any_frame_skipping()) {
            float val = eval_expr(rest, stream);
            bool cond = (val != 0.0f && !isnan(val));
            frame.executing = cond;
            if(!cond) {
                string matched;
                int lines_read = 0;
                if(!skip_to(fh, "while", "endwhile", matched, lines_read, num))
                    halt_error(stream, "O%d while has no matching endwhile", num);
                file_line += lines_read;
                return true; // don't push frame, block was skipped
            }
        } else {
            frame.executing = false;
        }
        tolerant_after_jump_ = false;
        stack_.push_back(frame);
        return true;
    }

    // ENDWHILE
    if(keyword == "endwhile") {
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::WHILE && stack_[i].num == num) {
                if(!any_frame_skipping()) {
                    // Seek back to the while line so it is re-evaluated next tick.
                    // Pop the frame; the while handler will re-push it when re-entered.
                    file_line = (unsigned long)stack_[i].jump_line - 1;
                    fwfs::fseek(fh, stack_[i].loop_offset, SEEK_SET);
                }
                stack_.erase(stack_.begin() + i);
                return true;
            }
        }
        label_error(stream, "O%d endwhile without matching while", num);
        return true;
    }

    // DO
    if(keyword == "do") {
        if((int)stack_.size() >= OCODE_MAX_STACK_DEPTH) {
            halt_error(stream, "O%d do exceeded max nesting depth %d", num, OCODE_MAX_STACK_DEPTH);
            return true;
        }

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::DO;
        frame.loop_offset   = fwfs::ftell(fh); // first line of body
        frame.executing     = !any_frame_skipping();
        frame.branch_taken  = false;
        frame.repeat_count  = 0;
        frame.return_offset = 0;
        frame.jump_line     = (int)file_line + 1;
        tolerant_after_jump_ = false;
        stack_.push_back(frame);
        return true;
    }

    // REPEAT
    if(keyword == "repeat") {
        if((int)stack_.size() >= OCODE_MAX_STACK_DEPTH) {
            halt_error(stream, "O%d repeat exceeded max nesting depth %d", num, OCODE_MAX_STACK_DEPTH);
            return true;
        }

        int count = 0;
        if(!any_frame_skipping()) {
            float val = eval_expr(rest, stream);
            count = (int)val;
        }

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::REPEAT;
        frame.loop_offset   = fwfs::ftell(fh); // first line of body
        frame.repeat_count  = count;
        frame.return_offset = 0;
        frame.branch_taken  = false;
        frame.jump_line     = (int)file_line + 1;

        if(!any_frame_skipping() && count > 0) {
            frame.executing = true;
            tolerant_after_jump_ = false;
            stack_.push_back(frame);
        } else {
            frame.executing = false;
            string matched;
            int lines_read = 0;
            if(!skip_to(fh, "repeat", "endrepeat", matched, lines_read, num))
                halt_error(stream, "O%d repeat has no matching endrepeat", num);
            file_line += lines_read;
        }
        return true;
    }

    // ENDREPEAT
    if(keyword == "endrepeat") {
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::REPEAT && stack_[i].num == num) {
                if(!any_frame_skipping()) {
                    stack_[i].repeat_count--;
                    if(stack_[i].repeat_count > 0) {
                        file_line = (unsigned long)stack_[i].jump_line - 1;
                        fwfs::fseek(fh, stack_[i].loop_offset, SEEK_SET);
                    } else {
                        stack_.erase(stack_.begin() + i);
                    }
                } else {
                    stack_.erase(stack_.begin() + i);
                }
                return true;
            }
        }
        label_error(stream, "O%d endrepeat without matching repeat", num);
        return true;
    }

    // BREAK
    if(keyword == "break") {
        if(any_frame_skipping()) return true;

        int idx = find_loop_frame(num);
        if(idx < 0) {
            label_error(stream, "O%d break does not point to a matching loop", num);
            return true;
        }

        BlockType loop_type = stack_[idx].type;
        stack_.resize(idx);

        string open_kw, close_kw;
        if(loop_type == BlockType::WHILE)       { open_kw = "while";  close_kw = "endwhile";  }
        else if(loop_type == BlockType::DO)     { open_kw = "do";     close_kw = "while";      }
        else                                    { open_kw = "repeat"; close_kw = "endrepeat";  }

        string matched;
        int lines_read = 0;
        if(!skip_to(fh, open_kw, close_kw, matched, lines_read, num))
            halt_error(stream, "O%d break: could not find end of loop", num);
        file_line += lines_read;
        return true;
    }

    // CONTINUE
    if(keyword == "continue") {
        if(any_frame_skipping()) return true;

        int idx = find_loop_frame(num);
        if(idx < 0) {
            label_error(stream, "O%d continue does not point to a matching loop", num);
            return true;
        }

        BlockType loop_type = stack_[idx].type;
        long loop_offset = stack_[idx].loop_offset;
        int jump_line = stack_[idx].jump_line;

        // Drop any block frames opened inside the loop body (e.g. an enclosing
        // if whose endif we are jumping over): continuing abandons them, so they
        // must be unwound or they leak one frame per iteration until the depth
        // limit is hit. This mirrors how break unwinds with resize().
        if(loop_type == BlockType::WHILE) {
            stack_.resize(idx); // also drop the while frame; it re-pushes on re-entry
            file_line = (unsigned long)jump_line - 1;
            fwfs::fseek(fh, loop_offset, SEEK_SET);
        } else if(loop_type == BlockType::DO) {
            stack_.resize(idx + 1); // keep the do frame, drop inner frames
            string matched;
            string condition;
            int lines_read = 0;
            if(!skip_to(fh, "do", "while", matched, lines_read, num, &condition))
                halt_error(stream, "O%d continue: could not find end of do-while", num);
            file_line += lines_read;
            float val = eval_expr(condition, stream);
            bool cond = (val != 0.0f && !isnan(val));
            if(cond) {
                file_line = (unsigned long)jump_line - 1;
                fwfs::fseek(fh, loop_offset, SEEK_SET);
            } else {
                stack_.erase(stack_.begin() + idx);
            }
        } else { // REPEAT
            stack_.resize(idx + 1); // keep the repeat frame, drop inner frames
            stack_[idx].repeat_count--;
            if(stack_[idx].repeat_count > 0) {
                file_line = (unsigned long)jump_line - 1;
                fwfs::fseek(fh, loop_offset, SEEK_SET);
            } else {
                string matched;
                int lines_read = 0;
                skip_to(fh, "repeat", "endrepeat", matched, lines_read, num);
                file_line += lines_read;
                stack_.erase(stack_.begin() + idx);
            }
        }
        return true;
    }

    halt_error(stream, "O%d unknown O-code keyword '%s'", num, keyword.c_str());
    return true;
}
