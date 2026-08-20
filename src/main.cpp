/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "libs/Kernel.h"
#include "libs/FirmwareFileSystem.h"

#include "modules/tools/laser/Laser.h"
#include "modules/tools/spindle/SpindleMaker.h"
#include "modules/tools/temperaturecontrol/TemperatureControlPool.h"
#include "modules/tools/endstops/Endstops.h"
#include "modules/tools/zprobe/ZProbe.h"
#ifndef NO_TOOLS_SCARACAL
#include "modules/tools/scaracal/SCARAcal.h"
#endif
#ifndef NO_TOOLS_ROTARYDELTACALIBRATION
#include "RotaryDeltaCalibration.h"
#endif
#include "modules/tools/switch/SwitchPool.h"
#include "modules/tools/temperatureswitch/TemperatureSwitch.h"
#include "modules/tools/drillingcycles/Drillingcycles.h"
#include "modules/tools/atc/ATCHandler.h"
#include "modules/utils/wifi/WifiProvider.h"
#include "modules/robot/Conveyor.h"
#include "modules/utils/simpleshell/SimpleShell.h"
#include "modules/utils/configurator/Configurator.h"
#include "modules/utils/player/Player.h"
#include "modules/utils/mainbutton/MainButton.h"
#include "modules/communication/SerialConsole2.h"
#include "libs/USBDevice/MSCFileSystem.h"
#include "Config.h"
#include "checksumm.h"
#include "ConfigValue.h"
#include "StepTicker.h"
#include "SlowTicker.h"
#include "Robot.h"

// #include "libs/ChaNFSSD/SDFileSystem.h"
#include "libs/nuts_bolts.h"
#include "libs/utils.h"

// Debug
#include "libs/SerialMessage.h"

//#include "libs/USBDevice/SDCard/SDCard.h"
#include "libs/USBDevice/SDCard/SDFileSystem.h"
// #include "libs/USBDevice/USBSerial/USBSerial.h"
// #include "libs/USBDevice/DFU.h"
#include "libs/SDFAT.h"
#include "StreamOutputPool.h"

#include "libs/Watchdog.h"
#include "version.h"
#include "system_LPC17xx.h"

#include "mbed.h"

// disable MSD
#define DISABLEMSD
#define second_usb_serial_enable_checksum  CHECKSUM("second_usb_serial_enable")
// #define disable_msd_checksum  CHECKSUM("msd_disable")
// #define dfu_enable_checksum  CHECKSUM("dfu_enable")
#define usb_msc_checksum       CHECKSUM("usb_msc")
#define enable_checksum        CHECKSUM("enable")
#define watchdog_timeout_checksum  CHECKSUM("watchdog_timeout")

SDFileSystem sd(P0_18, P0_17, P0_15, P0_16, 12000000);

SDFAT mounter("sd", &sd);

GPIO leds[4] = {
    GPIO(P4_29),
    GPIO(P4_28),
	GPIO(P0_4),
    GPIO(P1_17)
};

void init() {

    SCB->VTOR = 0x4000;

    // Default pins to low status
    for (int i = 0; i < 4; i++){
        leds[i].output();
        leds[i]= 0;
    }


    GPIO beep = GPIO(P1_14);
    beep.output();
    beep = 0;
    GPIO extout = GPIO(P0_29);
    extout.output();
    extout = 0;
    extout = GPIO(P0_30);
    extout.output();
    extout = 0;
    extout = GPIO(P1_19);
    extout.output();
    extout = 0;

    // open 12V
    // GPIO v12 = GPIO(P0_11);
    /*
    GPIO v12 = GPIO(P0_9);
    v12.output();
    v12 = 1;

    // GPIO v24 = GPIO(P1_29);
    GPIO v24 = GPIO(P0_0);
    v24.output();
    v24 = 1;

    GPIO vCharge = GPIO(P0_23);
    vCharge.output();
    vCharge = 1;
    */

    Kernel* kernel = new Kernel();

    // kernel->streams->printf("Smoothie Running @%ldMHz\r\n", SystemCoreClock / 1000000);
    SimpleShell::version_command("", kernel->streams);

    bool sdok = (sd.disk_initialize() == 0);
    if(!sdok) kernel->streams->printf("SDCard failed to initialize\r\n");

    #ifdef NONETWORK
        kernel->streams->printf("NETWORK is disabled\r\n");
    #endif

#ifdef DISABLEMSD
	// msc = NULL;
	// kernel->streams->printf("MSD is disabled\r\n");

	/*
    // attempt to be able to disable msd in config
    if(sdok && !kernel->config->value( disable_msd_checksum )->as_bool(true)){
        // HACK to zero the memory USBMSD uses as it and its objects seem to not initialize properly in the ctor
        size_t n= sizeof(USBMSD);
        void *v = malloc(n);
        memset(v, 0, n); // clear the allocated memory
        msc= new(v) USBMSD(&u, &sd); // allocate object using zeroed memory
    }else{
        msc = NULL;
        kernel->streams->printf("MSD is disabled\r\n");
    }*/

#endif

    // Create and add main modules
    kernel->add_module( new Player() );

    // ATC Handler
    kernel->add_module( new ATCHandler() );

    // This module owns the USB host controller while enabled.
    if (kernel->config->value(usb_msc_checksum, enable_checksum)->as_bool(true)) {
        kernel->add_module( new MSCFileSystem("ud") );
    } else {
        kernel->streams->printf("NOTE: USB mass storage host is disabled\n");
    }

    // Serial Console handles IO with the wireless probe
    kernel->add_module( new SerialConsole2() );

    kernel->add_module( new MainButton() );

    // Wifi Provider
    kernel->add_module( new WifiProvider);

    // these modules can be completely disabled in the Makefile by adding to EXCLUDE_MODULES
    #ifndef NO_TOOLS_SWITCH
    SwitchPool *sp= new SwitchPool();
    sp->load_tools();
    delete sp;
    #endif

    // #ifndef NO_TOOLS_TEMPERATURECONTROL
    // Note order is important here must be after extruder so Tn as a parameter will get executed first
    TemperatureControlPool *tp= new TemperatureControlPool();
    tp->load_tools();
    delete tp;

    // #endif
    #ifndef NO_TOOLS_ENDSTOPS
    kernel->add_module( new Endstops() );
    #endif
    #ifndef NO_TOOLS_LASER
    kernel->add_module( new Laser() );
    #endif

    #ifndef NO_TOOLS_SPINDLE
    SpindleMaker *sm = new SpindleMaker();
    sm->load_spindle();
    delete sm;
    #endif
    #ifndef NO_TOOLS_ZPROBE
    kernel->add_module( new ZProbe() );
    #endif
    #ifndef NO_TOOLS_SCARACAL
    kernel->add_module( new SCARAcal() );
    #endif
    #ifndef NO_TOOLS_ROTARYDELTACALIBRATION
    kernel->add_module( new RotaryDeltaCalibration() );
    #endif
//    #ifndef NONETWORK
//    kernel->add_module( new Network() );
//    #endif
    #ifndef NO_TOOLS_TEMPERATURESWITCH
    // Must be loaded after TemperatureControl
    kernel->add_module( new TemperatureSwitch() );
    #endif
    #ifndef NO_TOOLS_DRILLINGCYCLES
    kernel->add_module( new Drillingcycles() );
    #endif
    // Create and initialize USB stuff
    // u.init();

/*
#ifdef DISABLEMSD
    if(sdok && msc != NULL){
        kernel->add_module( msc );
    }
#else
    if (!kernel->config->value( disable_msd_checksum )->as_bool(false)) {
        kernel->add_module( &msc );
    }
#endif
*/

    /* disable USB module
    kernel->add_module( &usbserial );
    if( kernel->config->value( second_usb_serial_enable_checksum )->as_bool(false) ){
        kernel->add_module( new USBSerial(&u) );
    }
    */

    // 10 second watchdog timeout (or config as seconds)

    // LUKE : DISABLED

    float t= kernel->config->value( watchdog_timeout_checksum )->as_number(10.0F);
    if(t > 0.1F) {
        // NOTE setting WDT_RESET with the current bootloader would leave it in DFU mode which would be suboptimal
        kernel->add_module( new Watchdog(t * 1000000, WDT_RESET )); // WDT_RESET));
        kernel->streams->printf("Watchdog enabled for %1.3f seconds\n", t);
    }else{
        kernel->streams->printf("WARNING Watchdog is disabled\n");
    }

    // kernel->add_module( &u );

    kernel->config->config_cache_clear();

    if(kernel->is_using_leds()) {
        // set some leds to indicate status... led0 init done, led1 mainloop running, led2 idle loop running, led3 sdcard ok
        leds[0]= 1; // indicate we are done with init
        leds[3]= sdok?1:0; // 4th led indicates sdcard is available (TODO maye should indicate config was found)
    }

    if(sdok) {
        // load config override file if present
        // NOTE only Mxxx commands that set values should be put in this file. The file is generated by M500
        FILE *fp= fwfs::fopen(kernel->config_override_filename(), "r");
        if(fp != NULL) {
            char buf[132];
            kernel->streams->printf("Loading config override file: %s...\n", kernel->config_override_filename());
            while(fwfs::fgets(buf, sizeof buf, fp) != NULL) {
                kernel->streams->printf("  %s", buf);
                if(buf[0] == ';') continue; // skip the comments
                struct SerialMessage message= {&(StreamOutput::NullStream), buf, 0};
                kernel->call_event(ON_CONSOLE_LINE_RECEIVED, &message);
            }
            kernel->streams->printf("config override file executed\n");
            fwfs::fclose(fp);
        }
    }

    // start the timers and interrupts
    THEKERNEL->conveyor->start(THEROBOT->get_number_registered_motors());
    THEKERNEL->step_ticker->start();
    THEKERNEL->slow_ticker->start();
}

int main()
{
    init();

    uint16_t cnt= 0;
    // Main loop
    while(1){
        if(THEKERNEL->is_using_leds()) {
            // flash led 2 to show we are alive
            leds[1]= (cnt++ & 0x1000) ? 1 : 0;
        }
        THEKERNEL->call_event(ON_MAIN_LOOP);
        THEKERNEL->call_event(ON_IDLE);
    }
}
