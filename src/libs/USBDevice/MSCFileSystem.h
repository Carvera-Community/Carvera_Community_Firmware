/* USB Mass Storage device file system
 * Copyrigh (c) 2010, Igor Skochinsky
 * based on SDFileStorage
 * Copyright (c) 2008-2009, sford
 */
 
#ifndef MSCFILESYSTEM_H
#define MSCFILESYSTEM_H

#include "mbed.h"
#include "usbhost_cpu.h"
#include "FATFileSystem.h"
#include "Module.h"

/* Class: MSCFileSystem
 *  Access the filesystem on an attached USB mass storage device (e.g. a memory stick)
 *
 * Example:
 * > MSCFileSystem msc("msc");
 * > 
 * > int main() {
 * >     FILE *fp = fopen("/msc/myfile.txt", "w");
 * >     fprintf(fp, "Hello World!\n");
 * >     fclose(fp);
 * > }
 */

class MSCFileSystem : public FATFileSystem, public Module {
public:

    /* Constructor: MSCFileSystem
     *  Create the File System for accessing a USB mass storage device
     *
     * Parameters:
     *  name - The name used to access the filesystem
     */
    MSCFileSystem(const char* name);
    virtual int disk_initialize();
    virtual int disk_write(const char *buffer, uint32_t block_number, uint32_t count);
    virtual int disk_read(char *buffer, uint32_t block_number, uint32_t count);    
    virtual int disk_status();
    virtual int disk_sync();
    virtual int disk_sectors();

    void on_module_loaded(void);
    void on_idle(void*);
    void on_second_tick(void *);
    void on_get_public_data(void* argument);
    void on_set_public_data(void* argument);

protected:

    int initialise_msc();
    USB_INT32U _numBlks;
    USB_INT32U _blkSize;
};

#endif
