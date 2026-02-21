/* USB Mass Storage device file system
 * Copyrigh (c) 2010, Igor Skochinsky
 * based on SDFileStorage
 * Copyright (c) 2008-2009, sford
 */
 
/* Introduction
 * ------------
 * TODO: write one
 * we're basically using NXP's USBHotLite sample code, just plugging in our own FAT library
 */
 
#include "MSCFileSystem.h"
#include "usbhost_inc.h"
#include "libs/Kernel.h"
#include "libs/StreamOutput.h"
#include "StreamOutputPool.h"
#include "libs/PublicData.h"
#include "Config.h"
#include "ConfigValue.h"
#include "PublicDataRequest.h"
#include "MSCFileSystemPublicAccess.h"
#include "libs/Pin.h"

#define usb_en_pin_checksum              			CHECKSUM("usb_en_pin")
#define usb_in_pin_checksum              			CHECKSUM("usb_in_pin")



MSCFileSystem::MSCFileSystem(const char* name) :
  FATFileSystem(name)
{
}

void print_inquiry(USB_INT08U *inqReply)
{
    // see USB Mass Storage Class – UFI Command Specification,
    // 4.2 INQUIRY Command
    printf("Inquiry reply:\n");
    uint8_t tmp = inqReply[0]&0x1F;
    printf("Peripheral device type: %02Xh\n", tmp);
    if ( tmp == 0 )
        printf("\t- Direct access (floppy)\n");
    else if ( tmp == 0x1F )
        printf("\t- none (no FDD connected)\n");
    else
        printf("\t- unknown type\n");
    tmp = inqReply[1] >> 7;
    printf("Removable Media Bit: %d\n", tmp);
    tmp = inqReply[2] & 3;
    printf("ANSI Version: %02Xh\n", tmp);
    if ( tmp != 0 )
        printf("\t- warning! must be 0\n");
    tmp = (inqReply[2]>>3) & 3;
    printf("ECMA Version: %02Xh\n", tmp);
    if ( tmp != 0 )
        printf("\t- warning! should be 0\n");
    tmp = inqReply[2]>>6;
    printf("ISO Version: %02Xh\n", tmp);
    if ( tmp != 0 )
        printf("\t- warning! should be 0\n");
    tmp = inqReply[3] & 0xF;
    printf("Response Data Format: %02Xh\n", tmp);
    if ( tmp != 1 )
        printf("\t- warning! should be 1\n");
    tmp = inqReply[4];
    printf("Additional length: %02Xh\n", tmp);
    if ( tmp != 0x1F )
        printf("\t- warning! should be 1Fh\n");
    printf("Vendor Information: '%.8s'\n", &inqReply[8]);
    printf("Product Identification: '%.16s'\n", &inqReply[16]);
    printf("Product Revision: '%.4s'\n", &inqReply[32]);        
}

int MSCFileSystem::initialise_msc()
{
    USB_INT32S  rc;
    USB_INT08U  inquiryResult[INQUIRY_LENGTH];
    
    //print_clock();
    Host_Init();               /* Initialize the  host controller                                    */

    rc = Host_EnumDev();       /* Enumerate the device connected                                            */
    if (rc != OK)
    {
        // fprintf(stderr, "Could not enumerate device: %d\n", rc) ;
    	THEKERNEL->streams->printf("Could not enumerate device: %d\n", rc);
        return rc;
    }
    
    /* Initialize the mass storage and scsi interfaces */
    rc = MS_Init( &_blkSize, &_numBlks, inquiryResult );
    if (rc != OK)
    {
        // fprintf(stderr, "Could not initialize mass storage interface: %d\n", rc);
        THEKERNEL->streams->printf("Could not initialize mass storage interface: %d\n", rc);
        return rc;
    }

    // printf("Successfully initialized mass storage interface; %d blocks of size %d\n", _numBlks, _blkSize);
    THEKERNEL->streams->printf("Successfully initialized mass storage interface; %d blocks of size %d\n", _numBlks, _blkSize);

    // print_inquiry(inquiryResult);

    // FATFileSystem supports only 512-byte blocks
    return _blkSize == 512 ? OK : 1;
}

int MSCFileSystem::disk_initialize()
{
    if ( initialise_msc() != OK )
        return 1;
        
    return 0;
}

int MSCFileSystem::disk_write(const char *buffer, uint32_t block_number, uint32_t count)
{
    if ( OK == MS_BulkSend(block_number, 1, (USB_INT08U *)buffer) )
        return 0;
    return 1;
}

int MSCFileSystem::disk_read(char *buffer, uint32_t block_number, uint32_t count)
{
    if ( OK == MS_BulkRecv(block_number, 1, (USB_INT08U *)buffer) )
        return 0;
    return 1;
}


int MSCFileSystem::disk_status() { return 0; }
int MSCFileSystem::disk_sync() { return 0; }
int MSCFileSystem::disk_sectors() { return _numBlks; }

void MSCFileSystem::on_module_loaded()
{
    Pin *usb_en_pin = new Pin();
    usb_en_pin->from_string(THEKERNEL->config->value( usb_en_pin_checksum )->by_default("1.19")->as_string());
    usb_en_pin->as_output();
    usb_en_pin->set(0);
    delete usb_en_pin;

//    Pin *usb_in_pin = new Pin();
//    usb_in_pin->from_string(THEKERNEL->config->value( usb_in_pin_checksum )->by_default("1.22")->as_string());
//    usb_in_pin->as_input();
//    delete usb_in_pin;

    register_for_event(ON_IDLE);
    this->register_for_event(ON_SECOND_TICK);
    this->register_for_event(ON_GET_PUBLIC_DATA);
    this->register_for_event(ON_SET_PUBLIC_DATA);
}

void MSCFileSystem::on_idle(void*)
{
	// pass
}

void MSCFileSystem::on_second_tick(void *)
{
}

void MSCFileSystem::on_get_public_data(void* argument)
{
    PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);

    if (pdr->starts_with(msc_file_system_checksum)) {
    	if (pdr->second_element_is(check_usb_host_checksum)) {
    	    disk_initialize();
			pdr->set_taken();
    	}
    }
}

void MSCFileSystem::on_set_public_data(void* argument)
{
    //PublicDataRequest* pdr = static_cast<PublicDataRequest*>(argument);
}
