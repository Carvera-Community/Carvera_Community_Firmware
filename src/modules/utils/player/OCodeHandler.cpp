#include "OCodeHandler.h"

#include "libs/StreamOutput.h"
#include "libs/Kernel.h"
#include "libs/utils.h"
#include "modules/communication/utils/Gcode.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

using std::string;
using std::map;

// Block-stack storage placed in AHBSRAM to keep it off the tight main SRAM heap.
// Zeroed at startup by mbed_custom.cpp; frames are always written before read.
OCodeHandler::Frame OCodeHandler::frame_storage_[OCODE_MAX_STACK_DEPTH] __attribute__((section("AHBSRAM")));

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
}

// One linear pass to build sub_table_. Preserves the caller's file position so
// it can be run lazily (on the first subroutine call) rather than upfront.
void OCodeHandler::pre_scan(FILE* fh, StreamOutput* stream)
{
    pre_scanned_ = true;
    if(!fh) return;

    long saved = ftell(fh);
    fseek(fh, 0, SEEK_SET);
    sub_table_.clear();

    char buf[130];
    int line = 0;
    while(fgets(buf, sizeof(buf), fh) != NULL) {
        // Keep the system responsive (and the watchdog fed) while scanning a
        // large file. This runs before playback motion starts, so yielding here
        // is safe; it must never happen mid-cut.
        if((++line % 100) == 0) THEKERNEL->call_event(ON_IDLE);

        int n;
        string kw, rest;
        if(parse_ocode(buf, n, kw, rest)) {
            if(kw == "sub") {
                // Store the offset of the line *after* "Onnn sub" so a call can
                // jump straight into the body instead of re-reading the sub line
                // (which the definition path would otherwise skip).
                sub_table_[n] = ftell(fh);
            }
        }
    }

    fseek(fh, saved, SEEK_SET);

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
    if(!pre_scanned_) pre_scan(fh, stream);
}

// Extract O-code number, lower-cased keyword, and remaining text from a raw line.
// Returns false when the line does not start with O/o followed by digits.
bool OCodeHandler::parse_ocode(const char* line, int& num, string& keyword, string& rest)
{
    const char* p = ltrim_cstr(line);
    if(*p != 'O' && *p != 'o') return false;
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

float OCodeHandler::eval_expr(const char* expr, StreamOutput* stream) const
{
    char* endptr = NULL;
    return Gcode::evaluate_standalone_expression(expr, &endptr, stream);
}

void OCodeHandler::halt_error(StreamOutput* stream, const char* fmt, ...) const
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    stream->printf("O-code error: %s\n", buf);
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

int OCodeHandler::innermost_loop_frame() const
{
    for(int i = (int)stack_.size() - 1; i >= 0; i--) {
        BlockType t = stack_[i].type;
        if(t == BlockType::WHILE || t == BlockType::DO || t == BlockType::REPEAT)
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
                            string& matched) const
{
    int depth = 0;
    int line = 0;
    char buf[130];
    while(fgets(buf, sizeof(buf), fh) != NULL) {
        // Skipping a large block happens synchronously within a single main-loop
        // tick, so feed the watchdog (and let other ON_IDLE work run) periodically.
        if((++line % 100) == 0) THEKERNEL->call_event(ON_IDLE);

        int n;
        string kw, rest;
        if(!parse_ocode(buf, n, kw, rest)) continue;

        if(kw == open_kw) { depth++; continue; }

        if(kw == close_kw) {
            if(depth == 0) { matched = kw; return true; }
            depth--;
        }
    }
    return false;
}

// Process one raw line from the G-code file.
// Returns true when the line was an O-code (consumed); false for normal G-code.
bool OCodeHandler::process_line(const char* line, FILE* fh, StreamOutput* stream)
{
    int num;
    string keyword, rest;

    if(!parse_ocode(line, num, keyword, rest)) return false;

    // SUB definition: skip the body when reached during normal execution flow.
    // The body only runs on an explicit call.
    if(keyword == "sub") {
        if(!any_frame_skipping()) {
            string matched;
            if(!skip_to(fh, "sub", "endsub", matched))
                halt_error(stream, "O%d sub has no matching endsub", num);
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
                long ret = stack_[i].return_offset;
                stack_.resize(i);
                fseek(fh, ret, SEEK_SET);
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

        map<int,long>::iterator it = sub_table_.find(num);
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
        frame.return_offset = ftell(fh);

        for(int p = 0; p < 30; p++)
            frame.saved_params[p] = THEKERNEL->local_params[p];

        // Parse bracketed arguments [a1] [a2] ...
        const char* rp = rest.c_str();
        for(int p = 0; p < 30; p++) {
            rp = ltrim_cstr(rp);
            if(*rp != '[') break;
            char* endp;
            float val = Gcode::evaluate_standalone_expression(rp + 1, &endp, stream);
            THEKERNEL->local_params[p] = val;
            if(endp && *endp == ']') endp++;
            rp = endp ? endp : rp + 1;
        }

        stack_.push_back(frame);
        fseek(fh, it->second, SEEK_SET);
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

        if(!any_frame_skipping()) {
            float val = eval_expr(rest.c_str(), stream);
            frame.executing    = (val != 0.0f && !isnan(val));
            frame.branch_taken = frame.executing;
        } else {
            frame.executing    = false;
            frame.branch_taken = true; // prevent elseif/else inside a false parent
        }
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
                    float val = eval_expr(rest.c_str(), stream);
                    bool cond = (val != 0.0f && !isnan(val));
                    stack_[i].executing    = cond;
                    stack_[i].branch_taken = cond;
                }
                return true;
            }
        }
        warn(stream, "O%d elseif without matching if (ignored)", num);
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
        warn(stream, "O%d else without matching if (ignored)", num);
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
        warn(stream, "O%d endif without matching if (ignored)", num);
        return true;
    }

    // WHILE (may also close a do-while if a matching DO frame exists)
    if(keyword == "while") {
        // Check for a matching DO frame first (closing "while" of a do-while)
        for(int i = (int)stack_.size() - 1; i >= 0; i--) {
            if(stack_[i].type == BlockType::DO && stack_[i].num == num) {
                if(!any_frame_skipping()) {
                    float val = eval_expr(rest.c_str(), stream);
                    bool cond = (val != 0.0f && !isnan(val));
                    if(cond)
                        fseek(fh, stack_[i].loop_offset, SEEK_SET);
                    else
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
        long while_offset = ftell(fh) - (long)strlen(line);

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::WHILE;
        frame.loop_offset   = while_offset;
        frame.repeat_count  = 0;
        frame.return_offset = 0;
        frame.branch_taken  = false;

        if(!any_frame_skipping()) {
            float val = eval_expr(rest.c_str(), stream);
            bool cond = (val != 0.0f && !isnan(val));
            frame.executing = cond;
            if(!cond) {
                string matched;
                if(!skip_to(fh, "while", "endwhile", matched))
                    halt_error(stream, "O%d while has no matching endwhile", num);
                return true; // don't push frame, block was skipped
            }
        } else {
            frame.executing = false;
        }
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
                    fseek(fh, stack_[i].loop_offset, SEEK_SET);
                }
                stack_.erase(stack_.begin() + i);
                return true;
            }
        }
        warn(stream, "O%d endwhile without matching while (ignored)", num);
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
        frame.loop_offset   = ftell(fh); // first line of body
        frame.executing     = !any_frame_skipping();
        frame.branch_taken  = false;
        frame.repeat_count  = 0;
        frame.return_offset = 0;
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
            float val = eval_expr(rest.c_str(), stream);
            count = (int)val;
        }

        Frame frame;
        frame.num           = num;
        frame.type          = BlockType::REPEAT;
        frame.loop_offset   = ftell(fh); // first line of body
        frame.repeat_count  = count;
        frame.return_offset = 0;
        frame.branch_taken  = false;

        if(!any_frame_skipping() && count > 0) {
            frame.executing = true;
            stack_.push_back(frame);
        } else {
            frame.executing = false;
            string matched;
            if(!skip_to(fh, "repeat", "endrepeat", matched))
                halt_error(stream, "O%d repeat has no matching endrepeat", num);
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
                        fseek(fh, stack_[i].loop_offset, SEEK_SET);
                    } else {
                        stack_.erase(stack_.begin() + i);
                    }
                } else {
                    stack_.erase(stack_.begin() + i);
                }
                return true;
            }
        }
        warn(stream, "O%d endrepeat without matching repeat (ignored)", num);
        return true;
    }

    // BREAK
    if(keyword == "break") {
        if(any_frame_skipping()) return true;

        int idx = innermost_loop_frame();
        if(idx < 0) {
            warn(stream, "O%d break outside of a loop (ignored)", num);
            return true;
        }

        BlockType loop_type = stack_[idx].type;
        stack_.resize(idx);

        string open_kw, close_kw;
        if(loop_type == BlockType::WHILE)       { open_kw = "while";  close_kw = "endwhile";  }
        else if(loop_type == BlockType::DO)     { open_kw = "do";     close_kw = "while";      }
        else                                    { open_kw = "repeat"; close_kw = "endrepeat";  }

        string matched;
        if(!skip_to(fh, open_kw, close_kw, matched))
            halt_error(stream, "O%d break: could not find end of loop", num);
        return true;
    }

    // CONTINUE
    if(keyword == "continue") {
        if(any_frame_skipping()) return true;

        int idx = innermost_loop_frame();
        if(idx < 0) {
            warn(stream, "O%d continue outside of a loop (ignored)", num);
            return true;
        }

        BlockType loop_type = stack_[idx].type;
        long loop_offset = stack_[idx].loop_offset;

        // Drop any block frames opened inside the loop body (e.g. an enclosing
        // if whose endif we are jumping over): continuing abandons them, so they
        // must be unwound or they leak one frame per iteration until the depth
        // limit is hit. This mirrors how break unwinds with resize().
        if(loop_type == BlockType::WHILE) {
            stack_.resize(idx); // also drop the while frame; it re-pushes on re-entry
            fseek(fh, loop_offset, SEEK_SET);
        } else if(loop_type == BlockType::DO) {
            stack_.resize(idx + 1); // keep the do frame, drop inner frames
            fseek(fh, loop_offset, SEEK_SET);
        } else { // REPEAT
            stack_.resize(idx + 1); // keep the repeat frame, drop inner frames
            stack_[idx].repeat_count--;
            if(stack_[idx].repeat_count > 0) {
                fseek(fh, loop_offset, SEEK_SET);
            } else {
                string matched;
                skip_to(fh, "repeat", "endrepeat", matched);
                stack_.erase(stack_.begin() + idx);
            }
        }
        return true;
    }

    return true; // unknown keyword: consume silently
}
