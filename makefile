#!/usr/bin/make

DIRS = mbed mri src
DIRSCLEAN = $(addsuffix .clean,$(DIRS))

# Set to non zero value if you want checks to be enabled which reserve a
# specific amount of space for the stack.  The heap's growth will be
# constrained to reserve this much space for the stack and the stack won't be
# able to grow larger than this amount.
STACK_SIZE=4096
export STACK_SIZE

# --- Toolchain Definition ---
# Check the origin of CC and CXX. If they are from make's defaults,
# override them. Otherwise, respect environment/command line settings.
ifeq ($(origin CC), default)
  CC = arm-none-eabi-gcc
endif
ifeq ($(origin CXX), default)
  CXX = arm-none-eabi-g++
endif
ifeq ($(origin AS), default)
  AS = arm-none-eabi-gcc
endif
ifeq ($(origin AR), default)
  AR = arm-none-eabi-ar
endif
# LD depends on CXX, so we don't override it directly here.
ifeq ($(origin OBJCOPY), default)
  OBJCOPY = arm-none-eabi-objcopy
endif
ifeq ($(origin OBJDUMP), default)
  OBJDUMP = arm-none-eabi-objdump
endif
ifeq ($(origin SIZE), default)
  SIZE = arm-none-eabi-size
endif

# Additionally, check LD. If default, set it based on the potentially overridden CXX.
ifeq ($(origin LD), default)
  LD = $(CXX)
endif

# Use the potentially overridden values. LD uses the final CXX value.
CC      ?= arm-none-eabi-gcc
CXX     ?= arm-none-eabi-g++
AR      ?= arm-none-eabi-ar
LD      ?= $(CXX) # Linker uses C++ compiler
OBJCOPY ?= arm-none-eabi-objcopy
OBJDUMP ?= arm-none-eabi-objdump
SIZE    ?= arm-none-eabi-size

# Define internal variables used by sub-makefiles based on the standard ones.
# These are exported so sub-makes inherit them.
GCC     = $(CC)
GPP     = $(CXX)
# AS internal variable should now be correctly set to use the GCC driver.
# LD internal variable also correctly set above.
# OBJCOPY, OBJDUMP, SIZE internal variables correctly set above.
export GCC GPP AS LD OBJCOPY OBJDUMP SIZE AR # Include AR for mri/Makefile

# Determine GCC version based on the selected CXX compiler
# Ensure CXX is defined before this point
ifeq ($(strip $(CXX)),)
  $(error CXX is not defined. Cannot determine compiler version.)
endif
GCC_VERSION := $(shell $(CXX) -dumpversion)
# GCC Version detection - different approach for Windows vs Unix
ifeq "$(OS)" "Windows_NT"
# For Windows, check if version is 10.3+ or 11+ using findstr
IS_GCC_10_3_OR_LATER := $(shell echo $(GCC_VERSION) | findstr /b "10.[3-9] 10.[1-9][0-9] 11. 12. 13. 14." >nul && echo 1 || echo 0)
else
# Unix version detection
GCC_MAJOR := $(shell echo $(GCC_VERSION) | cut -f1 -d.)
GCC_MINOR := $(shell echo $(GCC_VERSION) | cut -f2 -d.)
IS_GCC_10_3_OR_LATER := $(shell echo $(GCC_VERSION) | awk -F. '{ if ($$1 > 10 || ($$1 == 10 && $$2 >= 3)) print 1; else print 0; }')
endif
export IS_GCC_10_3_OR_LATER
# --- End Toolchain Definition ---

all:
	@ $(MAKE) -C mbed
	@ $(MAKE) -C mri arm
	@echo Building Smoothie
	@ $(MAKE) -C src

clean: $(DIRSCLEAN)

$(DIRSCLEAN): %.clean:
	@echo Cleaning $*
	@ $(MAKE) -C $*  clean

debug-store:
	@ $(MAKE) -C src debug-store

flash:
	@ $(MAKE) -C src flash

dfu:
	@ $(MAKE) -C src dfu

upload:
	@ $(MAKE) -C src upload

debug:
	@ $(MAKE) -C src debug

console:
	@ $(MAKE) -C src console

.PHONY: all $(DIRS) $(DIRSCLEAN) debug-store flash upload debug console dfu

# --- Debugging Target --- 
# Prints the values of key toolchain variables as make sees them and exits.
.PHONY: print-vars
print-vars:
	@echo "--- Toolchain Variables ---"
	@echo "CC      = $(CC)"
	@echo "CXX     = $(CXX)"
	@echo "AS      = $(AS)"
	@echo "AR      = $(AR)"
	@echo "LD      = $(LD)"
	@echo "OBJCOPY = $(OBJCOPY)"
	@echo "OBJDUMP = $(OBJDUMP)"
	@echo "SIZE    = $(SIZE)"
	@echo "---------------------------"
	@echo "GCC     = $(GCC)"
	@echo "GPP     = $(GPP)"
	@echo "---------------------------"
	@echo "PATH    = $(PATH)"
	@echo "---------------------------"
	@echo "GCC_VERSION          = $(GCC_VERSION)"
	@echo "IS_GCC_10_3_OR_LATER = $(IS_GCC_10_3_OR_LATER)"
	@echo "---------------------------"
