/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BOOTCOUNTER_H
#define BOOTCOUNTER_H

#include <cstdint>

// Counts consecutive boot attempts that never completed init(), using the
// LPC17xx RTC battery-domain general purpose registers (GPREG0/1), which
// survive every reset (watchdog, NVIC_SystemReset, brown-out) and - with an
// RTC battery - power loss as well. Without a battery a power cycle clears
// them, which simply grants a fresh set of attempts.
//
// The point: any crash caused by data read during boot (a corrupt file on SD,
// a bad config value) recurs identically on every boot and therefore loops
// forever. After SAFE_BOOT_THRESHOLD consecutive failures, is_safe_boot()
// turns true and the boot-time SD autoloads are skipped so the machine can
// come up and report what happened.
namespace BootCounter {

    static const uint32_t SAFE_BOOT_THRESHOLD = 3;

    // Call once, as early as possible in init(). Returns the attempt number
    // for this boot (1 = first attempt since the counter was last cleared).
    uint32_t increment();

    // Call when init() has fully completed: clears the counter so only
    // consecutive failures count toward the threshold.
    void mark_boot_ok();

    // Attempt number of the current boot, as returned by increment().
    uint32_t attempts();

    // True when more than SAFE_BOOT_THRESHOLD consecutive attempts failed:
    // boot-time SD autoloads (flex compensation, custom tool slots, config
    // override) must be skipped this boot.
    bool is_safe_boot();
}

#endif
