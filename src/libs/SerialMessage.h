#ifndef SERIALMESSAGE_H
#define SERIALMESSAGE_H

#include <string>

class StreamOutput;

struct SerialMessage {
        StreamOutput* stream;
        std::string message;
        unsigned int line;
};
#endif
