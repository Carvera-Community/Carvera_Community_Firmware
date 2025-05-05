# TODO

## About this list

This firmware is a fork from Makera's official repository, and is based on
Smoothieware. This version of Smoothieware is deprecated. So we should keep the
following constraints in mind when choosing what tasks to work on:

- We still need to merge upstream changes from Makera. When we change code, the
  more we change it, the greater the likelihood of having to apply patches by
  hand. If you're making heavy changes to shared code, expect to be tapped to
  help in this process in the future :)
- Because Smoothieware is so old, it represents an all-you-can-eat buffet of
  refactoring and cleaning up. Resist this urge! We should limit ourselves to
  serious bugfixes, mechanical changes to fix linting errors and compiler
  support, or changes that unlock amazing new features for the community.
- As of this writing, there are no locally runnable unit tests, or any
  meaningful device side tests. This substantially increases the risk of all
  changes, but especially in core modules or device interfaces.
- [Research has
  shown](https://security.googleblog.com/2024/09/eliminating-memory-safety-vulnerabilities-Android.html)
  that risk is concentrated in new code. While there are flagrant memory safety
  violations throughout this codebase, the survival of those bugs after hundreds
  of thousands of hours of runtime should give us pause. In the absense of unit
  tests against host builds with tools like
  [ASan](https://github.com/google/sanitizers/wiki/addresssanitizer), we should
  err on the side of leaving them alone without a compelling justification (such
  as a crash affecting users)
- Familarize yourself with how the LPC1768 works in our developer docs.

## Stability 

- Ensure use of atomic data types and mask ISRs during shared read/writes
- Support time based debounce rather than counters dependent on frequency
- Remove use of dangerous `delete this` pattern which also fragments heap
- Audit modules for placement in AHB versus main memory
- Fix crashers when calling `config-get` with no arguments

## Debug

- Investigate support for GDB monitor to allow SimpleShell commands during GDB.
- Investigate max supported UART baud rate
- Ability to persist backtrace after hardware fault to SD card on reboot

## Performance

- Investigate basic primitives for hardware counters to measure CPU load

## Cool features

- Custom control of RGB LED
- Minimal USB HID stack for keyboard or controller
