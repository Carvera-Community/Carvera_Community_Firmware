#include "StreamOutput.h"
#include "CommunicationProtocol.h"

NullStreamOutput StreamOutput::NullStream;

const comms::ProtocolHandler& StreamOutput::protocol() const
{
    return comms::default_protocol();
}

int StreamOutput::printf(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    comms::FormattedMessage message(format, args);
    va_end(args);

    if (!message.valid()) return message.size();
    puts(message.data(), message.size());
    return message.size();
}
