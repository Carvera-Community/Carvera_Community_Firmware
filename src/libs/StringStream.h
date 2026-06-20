#ifndef _STRINGSTREAM_H_
#define _STRINGSTREAM_H_

#include "StreamOutput.h"

#include <string>

class StringStream : public StreamOutput {
    public:
        StringStream() {}
        int puts(const char *str, int size = 0) { size_t n = size > 0 ? size : strlen(str); output.append(str, n); return n; }
        void clear() { output.clear(); }
        std::string getOutput() const { return output; }

    private:
        std::string output;
};

#endif
