/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMOUTPUTPOOL_H
#define STREAMOUTPUTPOOL_H

using namespace std;
#include <set>
#include <string>
#include <cstdio>
#include <cstdarg>

#include "libs/StreamOutput.h"
#include "CommunicationProtocol.h"

class StreamOutputPool : public StreamOutput {

public:
    StreamOutputPool(){
    }

    int printf(const char *format, ...) __attribute__ ((format(printf, 2, 3)))
    {
        va_list args;
        va_start(args, format);
        comms::FormattedMessage message(format, args);
        va_end(args);
        if (!message.valid()) return message.size();

        for(set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++) {
            (*i)->protocol().send_message(*i, comms::MessageType::Text, message.data(), message.size());
        }
        return message.size();
    }

    int _putc(int c)
    {
        int r = 0;
        for(set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            int k = (*i)->_putc(c);
            if (k > r)
                r = k;
        }
        return r;
    }

    int puts(const char* s, int size)
    {
        int r = 0;
        for(set<StreamOutput*>::iterator i = this->streams.begin(); i != this->streams.end(); i++)
        {
            int k = (*i)->puts(s, size);
            if (k > r)
                r = k;
        }
        return r;
    }

    void append_stream(StreamOutput* stream)
    {
        this->streams.insert(stream);
    }

    void remove_stream(StreamOutput* stream)
    {
        this->streams.erase(stream);
    }

private:
    set<StreamOutput*> streams;
};

#endif
