/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

#include "BootCounter.h"

#include "LPC17xx.h"

// GPREG0 holds a magic tag so the very first boot after the RTC domain lost
// power (random register content) starts the count fresh. GPREG1 is the
// number of boot attempts since mark_boot_ok() last ran. Nothing else in the
// firmware uses the RTC general purpose registers.
#define BOOTCOUNTER_MAGIC 0xB007C0DEu
#define PCONP_PCRTC (1u << 9)

static uint32_t this_boot_attempts = 0;

uint32_t BootCounter::increment()
{
    // The RTC block must be powered for its registers to be accessible. PCRTC
    // is set at reset, but make sure rather than assume.
    LPC_SC->PCONP |= PCONP_PCRTC;

    if (LPC_RTC->GPREG0 != BOOTCOUNTER_MAGIC) {
        LPC_RTC->GPREG0 = BOOTCOUNTER_MAGIC;
        LPC_RTC->GPREG1 = 0;
    }

    LPC_RTC->GPREG1 = LPC_RTC->GPREG1 + 1;
    this_boot_attempts = LPC_RTC->GPREG1;
    return this_boot_attempts;
}

void BootCounter::mark_boot_ok()
{
    LPC_RTC->GPREG1 = 0;
}

uint32_t BootCounter::attempts()
{
    return this_boot_attempts;
}

bool BootCounter::is_safe_boot()
{
    return this_boot_attempts > SAFE_BOOT_THRESHOLD;
}
