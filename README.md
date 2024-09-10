# Overview
This is a branch of the [Smoothieware](https://github.com/Smoothieware/Smoothieware) firmware for the Makera Carvera CNC Machine.Checkout the [Releases Page](https://github.com/MakeraInc/CarveraFirmware/releases)for downloads [Carvera](https://www.makera.com) Firmware.

Smoothie is a free, opensource, high performance G-code interpreter and CNC controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard, R2C2 or any other LPC17xx-based board. The motion control part is a port of the awesome grbl.

To let the Carvera work perfectly, please make sure both your controller software and firmware are up to date. For firmware, download the new firmware first, then use the 'update' function to upgrade the firmware, and reset the machine after the update is complete.

Documentation can be found here :
- https://wiki.makera.com/en/home
- https://wiki.makera.com/en/supported-codes
- https://smoothieware.github.io/Webif-pack/documentation/web/html/index.html

**NOTE** it is not necessary to build Smoothie yourself unless you want to. prebuilt binaries are available here: Releases Page
# Quick Start
These are the quick steps to get Smoothie dependencies installed on your computer:
- Pull down a clone of the Smoothie github project to your local machine.
- Download https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/gcc-arm-none-eabi-4_8-2014q1-20140314-win32.zip and place it in the root folder of this directory.
- right now I have to modify the win_install.cmd by removing lines 44-47 and manually downloading the .zip
  - echo Downloading GNU Tools for ARM Embedded Processors
  - echo %DATE% %TIME% Executing build\win32\curl -kL0 %GCC4ARM_URL%>>%LOGFILE%
  - build\win32\curl -kL0 %GCC4ARM_URL% >%GCC4ARM_TAR%
  - if errorlevel 1 goto ExitOnError
- In the root subdirectory of the cloned Smoothie project, there are install scripts for the supported platforms. Run the install script appropriate for your platform:
  - Windows: win_install.cmd
  - OS X: mac_installl
  - Linux: linux_installl
- You can then run the BuildShell script which will be created during the install to properly configure the PATH environment variable to point to the required version of GCC for ARM which was just installed on your machine. You may want to edit this script to further customize your development environment.
# Building Smoothie
Follow this guide... http://smoothieware.org/compiling-smoothie

In short... From a shell, switch into the root Smoothie project directory and run:
```bash
make clean
make all AXIS=5 PAXIS=3 CNC=1 
```
To upload you can either use the carvera controller upload feature under settings or Alternatively copy the file LPC1768/main.bin to the sdcard calling it firmware.bin and reset.

# Filing issues (for bugs ONLY)
Please follow this guide https://github.com/Smoothieware/Smoothieware/blob/edge/ISSUE_TEMPLATE.md
# Contributing
Please take a look at :

http://smoothieware.org/coding-standards
http://smoothieware.org/developers-guide
http://smoothieware.org/contribution-guidlines
Contributions very welcome !
# Donate
The Smoothie firmware is free software developed by volunteers. If you find this software useful, want to say thanks and encourage development, please consider a [Donation](https://paypal.me/smoothieware)
# License
Smoothieware is released under the GNU GPL v3, which you can find at http://www.gnu.org/licenses/gpl-3.0.en.html
