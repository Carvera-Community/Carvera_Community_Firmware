#ifndef FIRMWARE_FILE_SYSTEM_H
#define FIRMWARE_FILE_SYSTEM_H

#include <stdio.h>

#include "DirHandle.h"

namespace fwfs {

FILE *fopen(const char *path, const char *mode);
FILE *freopen(const char *path, const char *mode, FILE *stream);
int fclose(FILE *stream);
size_t fread(void *ptr, size_t size, size_t count, FILE *stream);
size_t fwrite(const void *ptr, size_t size, size_t count, FILE *stream);
int fprintf(FILE *stream, const char *format, ...);
char *fgets(char *str, int count, FILE *stream);
int fputs(const char *str, FILE *stream);
int fseek(FILE *stream, long offset, int origin);
long ftell(FILE *stream);
int fgetc(FILE *stream);
int fputc(int ch, FILE *stream);
int fgetpos(FILE *stream, fpos_t *pos);
int fsetpos(FILE *stream, const fpos_t *pos);
int feof(FILE *stream);
int remove(const char *path);
int rename(const char *old_path, const char *new_path);
DIR *opendir(const char *path);
int mkdir(const char *path, int mode);

}

#endif
