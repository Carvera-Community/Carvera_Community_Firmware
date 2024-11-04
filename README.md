# **Overview**

This is a community branch of the Smoothieware firmware for the Makera Carvera CNC Machine. It is also part of the Carvera Community Org, which includes these other projects:
[Machine Profiles](https://github.com/Carvera-Community/Carvera_Community_Profiles) - Post processors, 3d design files, and tool libraries for various CAM/CAD programs

Open Source Version of the Carvera [Controller](https://github.com/Carvera-Community/CarveraController) - includes builds for linux and macOS. There is a javascript based controller alternative in the works as well

[Trello](https://trello.com/b/qKxPlEbk/carvera-community-firmware-controller-and-other-tech) - for seeing progress on features and making recommendations

**NOTE** it is not necessary to build the firmware yourself unless you want to. prebuilt binaries are available [here](https://github.com/Carvera-Community/Carvera_Community_Firmware/releases). There will be periodic stable releases that line up with controller releases, and rolling beta versions to test new features.





Smoothie is a free, opensource, high performance G-code interpreter and CNC controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard, R2C2 or any other LPC17xx-based board. The motion control part is a port of the awesome grbl.

#  **Documentation**:
[makera website](https://wiki.makera.com/en/home)

[Community Spreadsheet](https://docs.google.com/spreadsheets/d/1i9jD0Tg6wzTpGYVqhLZMyLFN7pMlSfdfgpQIxDKbojc/edit?gid=1892564078#gid=1892564078) includes all the gcodes, mcodes, config variables, and console commands for the machine as well as some tested feeds and speeds and a page on accessories that have been used on the machine.

[Supported G codes](https://wiki.makera.com/en/supported-codes)

[Smoothieware documentation](https://smoothieware.github.io/Webif-pack/documentation/web/html/index.html)

[Carvera A to Z](https://carvera-a-to-z.gitbook.io/carvera-a-to-z) a work in progress wiki for all sorts of information on getting started with the Carvera CNC machine

# **Building Firmware Quick Start**

These are the quick steps to get Smoothie dependencies installed on your computer:

Pull down a clone of the this github project to your local machine.

Download the [launchpad prerequisites](https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/gcc-arm-none-eabi-4_8-2014q1-20140314-win32.zip) and place it in the root folder of this directory.

In the root subdirectory of the cloned project, there are install scripts for the supported platforms. Run the install script appropriate for your platform:
Windows: win_install.cmd
OS X: mac_install - I have not tested this yet, but you will probably have to edit the script to remove the command that downlads the gcc-arm-none file that is linked above
Linux: linux_install - I have not tested this yet, but you will probably have to edit the script to remove the command that downlads the gcc-arm-none file that is linked above
You can then run the BuildShell script which will be created during the install to properly configure the PATH environment variable to point to the required version of GCC for ARM which was just installed on your machine. You may want to edit this script to further customize your development environment.

note: in this fork, I modified the win_install.cmd by removing lines 44-47 and manually downloading the .zip

### **Building The Firmware**
Open the BuildShell.cmd and run:

make clean

make all AXIS=5 PAXIS=3 CNC=1

this will create a file in LPC1768/main.bin

rename it to firmware.bin or another name with firmware in it and either upload it using the carvera controller or copy the file to the sdcard and reset your machine.

Filing issues (for bugs ONLY)
Please follow this guide https://github.com/Smoothieware/Smoothieware/blob/edge/ISSUE_TEMPLATE.md

for more information on compiling smoothieware: Follow this guide... https://smoothieware.github.io/Webif-pack/documentation/web/html/compiling-smoothie.html

# **Contributing**
Open an [issue](https://github.com/Carvera-Community/Carvera_Community_Firmware/issues) either on github, trello, or message one of the admins. Issues can be for bugfixes or feature requests. 

Test beta versions of the firmware and give bugreports/feedback

Contribute pull requests to the project

contribute to the [A_To_Z wiki](https://github.com/SergeBakharev/carvera_a_to_z)

Please take a look at :

http://smoothieware.org/coding-standards

http://smoothieware.org/developers-guide

http://smoothieware.org/contribution-guidlines

Contributions very welcome! 

### **Donate**
This particular branch of the carvera firmware is maintained by [Fae Corrigan](https://www.patreon.com/propsmonster)
For smoothieware as a whole: the Smoothie firmware is free software developed by volunteers. If you find this software useful, want to say thanks and encourage development, please consider a [Donation](https://paypal.me/smoothieware)

### **License**
Smoothieware is released under the GNU GPL v3, which you can find at http://www.gnu.org/licenses/gpl-3.0.en.html


### **Other community resources: **

Open source controllers: 

https://cc.grid.space/ 
https://github.com/GridSpace/carve-control
https://github.com/AngryApostrophe/Clout

https://cnc.js.org/ 
https://github.com/cncjs/cncjs-pendant-boilerplate

community feeds, speeds and accessories: https://docs.google.com/spreadsheets/d/1i9jD0Tg6wzTpGYVqhLZMyLFN7pMlSfdfgpQIxDKbojc/edit

carvera website: https://www.makera.com/pages/community https://wiki.makera.com/en/home

work in progress wireless 3 axis touch probe: will be released open source and open hardware along with a purchasable version https://github.com/faecorrigan/Open-Source-3-axis-CNC-Touch-Probe
