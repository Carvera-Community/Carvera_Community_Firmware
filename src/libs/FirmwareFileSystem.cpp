#include "FirmwareFileSystem.h"

#include <stdarg.h>

extern "C" int mkdir(const char *path, int mode);

namespace fwfs {

FILE *fopen(const char *path, const char *mode)
{
    return ::fopen(path, mode);
}

FILE *freopen(const char *path, const char *mode, FILE *stream)
{
    return ::freopen(path, mode, stream);
}

int fclose(FILE *stream)
{
    return ::fclose(stream);
}

size_t fread(void *ptr, size_t size, size_t count, FILE *stream)
{
    return ::fread(ptr, size, count, stream);
}

size_t fwrite(const void *ptr, size_t size, size_t count, FILE *stream)
{
    return ::fwrite(ptr, size, count, stream);
}

int fprintf(FILE *stream, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    int result = ::vfprintf(stream, format, args);
    va_end(args);
    return result;
}

char *fgets(char *str, int count, FILE *stream)
{
    return ::fgets(str, count, stream);
}

int fputs(const char *str, FILE *stream)
{
    return ::fputs(str, stream);
}

int fseek(FILE *stream, long offset, int origin)
{
    return ::fseek(stream, offset, origin);
}

long ftell(FILE *stream)
{
    return ::ftell(stream);
}

int fgetc(FILE *stream)
{
    return ::fgetc(stream);
}

int fputc(int ch, FILE *stream)
{
    return ::fputc(ch, stream);
}

int fgetpos(FILE *stream, fpos_t *pos)
{
    return ::fgetpos(stream, pos);
}

int fsetpos(FILE *stream, const fpos_t *pos)
{
    return ::fsetpos(stream, pos);
}

int feof(FILE *stream)
{
    return ::feof(stream);
}

int remove(const char *path)
{
    return ::remove(path);
}

int rename(const char *old_path, const char *new_path)
{
    return ::rename(old_path, new_path);
}

DIR *opendir(const char *path)
{
    return ::opendir(path);
}

int mkdir(const char *path, int mode)
{
    return ::mkdir(path, mode);
}

}
