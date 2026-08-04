#include "AppendFileStream.h"
#include "libs/FirmwareFileSystem.h"
#include <stdio.h>

int AppendFileStream::puts(const char *str, int size)
{
    FILE *fd= fwfs::fopen(this->fn, "a");
    if(fd == NULL) return 0;

    int n= fwfs::fwrite(str, 1, strlen(str), fd);
    fwfs::fclose(fd);
    return n;
}
