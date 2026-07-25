#include "StreamOutput.h"

extern unsigned char fbuff[4096];
#define HEADER        0x8668
#define FOOTER        0x55AA
#define PTYPE_NORMAL_INFO	0x90

extern const unsigned short crc_table[256];
NullStreamOutput StreamOutput::NullStream;

ProtocolMode communication_protocol = PROTOCOL_MAKERA;

unsigned int StreamOutput::crc16_ccitt(unsigned char *data, unsigned int len)
{
	unsigned char tmp;
	unsigned short crc = 0;
	for (unsigned int i = 0; i < len; i ++) {
        tmp = ((crc >> 8) ^ data[i]) & 0xff;
        crc = ((crc << 8) ^ crc_table[tmp]) & 0xffff;
	}
	return crc & 0xffff;
}


void StreamOutput::PacketMessage(char cmd, const char* s, int size)
{
	int crc = 0;
    unsigned int len = 0;
	size_t total_length = size == 0 ? strlen(s) : size;
	fbuff[0] = (HEADER>>8)&0xFF;
	fbuff[1] = HEADER&0xFF;
	fbuff[4] = cmd;
	memcpy(&fbuff[5], s, total_length);
	len = total_length + 3;
	fbuff[2] = (len>>8)&0xFF;
	fbuff[3] = len&0xFF;
	crc = crc16_ccitt(&fbuff[2], len);
	fbuff[total_length+5] = (crc>>8)&0xFF;
	fbuff[total_length+6] = crc&0xFF;
	fbuff[total_length+7] = (FOOTER>>8)&0xFF;
	fbuff[total_length+8] = FOOTER&0xFF;
	puts((char *)fbuff, len+6);
}

int StreamOutput::printf(const char *format, ...)
{
    char b[256];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);
    va_list args_copy;
    va_copy(args_copy, args);

    int len = vsnprintf(b, sizeof(b), format, args);
    va_end(args);

    if (len < 0) {
        va_end(args_copy);
        return -1;
    } else if ((size_t)len < sizeof(b)) {
        va_end(args_copy);
        buffer = b;
    } else {
        buffer = new char[len + 1];
        vsnprintf(buffer, len + 1, format, args_copy);
        va_end(args_copy);
    }

    if (communication_protocol == PROTOCOL_SMOOTHIE || !frames_protocol_output()) {
        puts(buffer, strlen(buffer));
    } else {
        PacketMessage(PTYPE_NORMAL_INFO, buffer, strlen(buffer));
    }

    if (buffer != b)
        delete[] buffer;

    return len;
}

int StreamOutput::printfcmd(const char cmd, const char *format, ...)
{
    char b[256];
    char *buffer;
    va_list args;
    va_start(args, format);
    va_list args_copy;
    va_copy(args_copy, args);

    int len = vsnprintf(b, sizeof(b), format, args);
    va_end(args);

    if (len < 0) {
        va_end(args_copy);
        return -1;
    } else if ((size_t)len < sizeof(b)) {
        va_end(args_copy);
        buffer = b;
    } else {
        buffer = new char[len + 1];
        vsnprintf(buffer, len + 1, format, args_copy);
        va_end(args_copy);
    }

    if (communication_protocol == PROTOCOL_SMOOTHIE || !frames_protocol_output()) {
        puts(buffer, strlen(buffer));
    } else {
        PacketMessage(cmd, buffer, strlen(buffer));
    }

    if (buffer != b) delete[] buffer;
    return len;
}
