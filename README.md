# **Overview**

This is a community branch of the Smoothieware firmware for the Makera Carvera
CNC Machine. It is also part of the Carvera Community Org, which includes these
other projects: [Machine Profiles](https://github.com/Carvera-Community/Carvera_Community_Profiles) -
Post processors, 3d design files, and tool libraries for various CAM/CAD
programs

Open Source Version of the Carvera
[Controller](https://github.com/Carvera-Community/CarveraController) - includes
builds for linux and macOS. There is a javascript based controller alternative
in the works as well

[Trello](https://trello.com/b/qKxPlEbk/carvera-community-firmware-controller-and-other-tech)
- for seeing progress on features and making recommendations

**NOTE** it is not necessary to build the firmware yourself unless you want to.
prebuilt binaries are available
[here](https://github.com/Carvera-Community/Carvera_Community_Firmware/releases).
There will be periodic stable releases that line up with controller releases,
and rolling beta versions to test new features.

Smoothie is a free, opensource, high performance G-code interpreter and CNC
controller written in Object-Oriented C++ for the LPC17xx micro-controller ( ARM
Cortex M3 architecture ). It will run on a mBed, a LPCXpresso, a SmoothieBoard,
R2C2 or any other LPC17xx-based board. The motion control part is a port of the
awesome grbl.

# Documentation and resources

- [Community feeds, speeds and
  accessories](https://docs.google.com/spreadsheets/d/1i9jD0Tg6wzTpGYVqhLZMyLFN7pMlSfdfgpQIxDKbojc/edit) Comprensive
  resource for G/M codes, config variables, console commands, as well as feeds & speeds recommendations.  
- [Makera website](https://wiki.makera.com/en/home), [supported codes](https://wiki.makera.com/en/supported-codes), [feeds & speeds](https://wiki.makera.com/en/speeds-and-feeds)
- [Smoothieware documentation](https://smoothieware.github.io/Webif-pack/documentation/web/html/index.html)
- [Carvera A to Z](https://carvera-a-to-z.gitbook.io/carvera-a-to-z) - A work in
  progress wiki for all sorts of information on getting started with the Carvera
  CNC machine

_**More from the Carvera Community**_

- [Carvera Controller](https://github.com/carvera-community/carvera_controller/)
  - community controller with extensive additional features and support for
  community firmware features.
- [Carvera Community Profiles](https://github.com/Carvera-Community/Carvera_Community_Profiles) - profiles and post-processor for various third party CAM software.
- [Carvera CLI](https://github.com/hagmonk/carvera-cli) - CLI interface to
  Carvera for scripting and device management.

_**Other open source tools**_

- https://cc.grid.space/ 
- https://github.com/GridSpace/carve-control
- https://github.com/AngryApostrophe/Clout
- https://cnc.js.org/ 
- https://github.com/cncjs/cncjs-pendant-boilerplate

Work in progress wireless 3 axis touch probe: will be released open source and
open hardware along with a purchasable version
https://github.com/faecorrigan/Open-Source-3-axis-CNC-Touch-Probe

# Filing issues and contributing 

Please follow [the Smoothieware issue template](https://github.com/Smoothieware/Smoothieware/blob/edge/ISSUE_TEMPLATE.md)
when filing bugs against this repo.

Contributions very welcome! 

- Open an
[issue](https://github.com/Carvera-Community/Carvera_Community_Firmware/issues)
either on github, trello, or message one of the admins. Issues can be for
bugfixes or feature requests. 
- Test beta versions of the firmware and give bugreports/feedback
- Contribute pull requests to the project
- Contribute to the [A_To_Z wiki](https://github.com/SergeBakharev/carvera_a_to_z)

Carvera Community Firmware uses the same guidelines as upstream Smoothieware
- http://smoothieware.org/coding-standards
- http://smoothieware.org/developers-guide
- http://smoothieware.org/contribution-guidlines

# Developers

See [DEVELOPER.md](./DEVELOPER.md) for instructions on building the firmware and
debugging.

See also [TODO](./TODO.md) for some ideas on things to work on :)

# Donate

This particular branch of the carvera firmware is maintained by [Fae
Corrigan](https://www.patreon.com/propsmonster) For smoothieware as a whole: the
Smoothie firmware is free software developed by volunteers. If you find this
software useful, want to say thanks and encourage development, please consider a
[Donation](https://paypal.me/smoothieware)

# License

Smoothieware is released under the GNU GPL v3, which you can find at
http://www.gnu.org/licenses/gpl-3.0.en.html MRI is released under Apache 2.0,
which you can find at https://www.apache.org/licenses/LICENSE-2.0

