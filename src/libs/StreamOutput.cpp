#include "StreamOutput.h"


extern unsigned char fbuff[4096];
#define HEADER        0x8668
#define FOOTER        0x55AA
#define PTYPE_NORMAL_INFO	0x90

extern const unsigned short crc_table[256];
NullStreamOutput StreamOutput::NullStream;
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
    char b[64];
    char *buffer;
    // Make the message
    va_list args;
    va_start(args, format);

    int size = vsnprintf(b, 64, format, args) + 1; // we add one to take into account space for the terminating \0

    if (size < 64) {
        buffer = b;
    } else {
        buffer = new char[size];
        vsnprintf(buffer, size, format, args);
    }
    va_end(args);

    PacketMessage(PTYPE_NORMAL_INFO, buffer, strlen(buffer));

    if (buffer != b)
        delete[] buffer;

    return size - 1;
}
