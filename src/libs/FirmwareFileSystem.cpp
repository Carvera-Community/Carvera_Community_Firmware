#include "FirmwareFileSystem.h"

#include <stdarg.h>

#include "libs/utils.h"

extern "C" int mkdir(const char *path, int mode);

namespace fwfs {

// Every path entering the filesystem is length-gated first; see
// path_too_long_for_fatfs() in utils.cpp for why, and why the real fix
// belongs upstream in ChaNFS rather than here.
static inline bool path_too_long(const char *path)
{
    return path_too_long_for_fatfs(path);
}

FILE *fopen(const char *path, const char *mode)
{
    if (path_too_long(path)) return NULL;
    return ::fopen(path, mode);
}

FILE *freopen(const char *path, const char *mode, FILE *stream)
{
    if (path_too_long(path)) return NULL;
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
    if (path_too_long(path)) return -1;
    return ::remove(path);
}

int rename(const char *old_path, const char *new_path)
{
    if (path_too_long(old_path) || path_too_long(new_path)) return -1;
    return ::rename(old_path, new_path);
}

DIR *opendir(const char *path)
{
    if (path_too_long(path)) return NULL;
    return ::opendir(path);
}

int mkdir(const char *path, int mode)
{
    if (path_too_long(path)) return -1;
    return ::mkdir(path, mode);
}

}
