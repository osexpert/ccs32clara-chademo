##
## This file is part of the libopenstm32 project.
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

OUT_DIR      = obj
PREFIX		?= arm-none-eabi
BINARY		= My407ccs2chademo
SIZE        = $(PREFIX)-size
CC		      = $(PREFIX)-gcc
CPP	      = $(PREFIX)-g++
LD		      = $(PREFIX)-gcc
OBJCOPY		= $(PREFIX)-objcopy
OBJDUMP		= $(PREFIX)-objdump
MKDIR_P     = mkdir -p
FPU_FLAGS = -mfloat-abi=hard -mfpu=fpv4-sp-d16
CPU_FLAGS = -DSTM32F4 -mcpu=cortex-m4 -mthumb
CFLAGS		= -Os -Isrc/ -Ilibopeninv/src -Ilibopencm3/include -Iexi -Iccs -Ichademo \
				 -fno-common -fno-builtin \
				 $(CPU_FLAGS) $(FPU_FLAGS) -std=gnu99 -ffunction-sections -fdata-sections
CPPFLAGS    = -Og -ggdb -Wall -Wextra -Isrc/ -Ilibopeninv/src -Ilibopencm3/include -Iexi -Iccs -Ichademo \
				-fno-common -std=c++11 -pedantic -DUSART_BAUDRATE=921600 \
				-ffunction-sections -fdata-sections -fno-builtin -fno-rtti -fno-exceptions -fno-unwind-tables $(CPU_FLAGS) $(FPU_FLAGS)
# Check if the variable GITHUB_RUN_NUMBER exists. When running on the github actions running, this
# variable is automatically available.
# Create a compiler define with the content of the variable. Or, if it does not exist, use replacement value 0.
EXTRACOMPILERFLAGS := $(shell \
  DATE=$$(date +%Y%m%d); \
  RUN=$${GITHUB_RUN_NUMBER:-0}; \
  SHA=$$(echo $${GITHUB_SHA:-0} | cut -c1-7); \
  echo "-DGITHUB_VERSION=\\\"$${DATE}-$${RUN}\\\" -DGITHUB_SHORT_SHA=\\\"$${SHA}\\\""; \
)

LDSCRIPT	  = linker.ld
# -march=armv7e-m did the trick here?? -mcpu=cortex-m4 did nothing. or...
LDFLAGS    = -Llibopencm3/lib -T$(LDSCRIPT) -march=armv7 -nostartfiles -nostdlib -Wl,--gc-sections,-Map,linker.map $(FPU_FLAGS)
OBJSL		  = main.o hwinit.o params.o \
				 my_string.o digio.o my_fp.o printf.o \
				 errormessage.o \
				 ipv6.o tcp.o \
				 connMgr.o modemFinder.o pevStateMachine.o \
				 hardwareInterface.o udpChecksum.o \
				 homeplug.o myHelpers.o qca7000.o \
				 appHandEXIDatatypesDecoder.o ByteStream.o EncoderChannel.o \
				 appHandEXIDatatypesEncoder.o DecoderChannel.o EXIHeaderDecoder.o \
				 appHandEXIDatatypes.o dinEXIDatatypesDecoder.o EXIHeaderEncoder.o \
				 BitInputStream.o dinEXIDatatypesEncoder.o MethodsBag.o \
				 BitOutputStream.o dinEXIDatatypes.o projectExiConnector.o \
				 chademoCharger.o chademoHardware.o led_blinker.o stm32scheduler.o


OBJS     = $(patsubst %.o,obj/%.o, $(OBJSL))
DEPENDS := $(patsubst %.o,obj/%.d, $(OBJSL))
vpath %.c src/ libopeninv/src exi/ ccs/ chademo/
vpath %.cpp src/ libopeninv/src exi/ ccs/ chademo/

OPENOCD_BASE	= /usr
OPENOCD		= $(OPENOCD_BASE)/bin/openocd
OPENOCD_SCRIPTS	= $(OPENOCD_BASE)/share/openocd/scripts
OPENOCD_FLASHER	= $(OPENOCD_SCRIPTS)/interface/parport.cfg
OPENOCD_BOARD	= $(OPENOCD_SCRIPTS)/board/olimex_stm32_h103.cfg

# Be silent per default, but 'make V=1' will show all compiler calls.
ifneq ($(V),1)
Q := @
NULL := 2>/dev/null
endif

# try-run
# Usage: option = $(call try-run, command,option-ok,otherwise)
# Exit code chooses option.
try-run = $(shell set -e;		\
	if ($(1)) >/dev/null 2>&1;	\
	then echo "$(2)";		\
	else echo "$(3)";		\
	fi)

# Test a linker (ld) option and return the gcc link command equivalent
comma := ,
link_command := -Wl$(comma)
ld-option = $(call try-run, $(PREFIX)-ld $(1) -v,$(link_command)$(1))

# Test whether we can suppress a safe warning about rwx segments
# only supported on binutils 2.39 or later
LDFLAGS	+= $(call ld-option,--no-warn-rwx-segments)

all: directories images
Debug:images
Release: images
cleanDebug:clean
images: $(BINARY)
	@printf "  OBJCOPY $(BINARY).bin\n"
	$(Q)$(OBJCOPY) -Obinary $(BINARY) $(BINARY).bin
	$(Q)$(SIZE) $(BINARY)

directories: ${OUT_DIR}

${OUT_DIR}:
	$(Q)${MKDIR_P} ${OUT_DIR}

$(BINARY): $(OBJS) $(LDSCRIPT)
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(LD) $(LDFLAGS) -o $(BINARY) $(OBJS) -lopencm3_stm32f4 -lm

-include $(DEPENDS)

$(OUT_DIR)/%.o: %.c Makefile
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(OUT_DIR)/%.o: %.cpp Makefile
	@printf "  CPP     $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CPP) $(CPPFLAGS) $(EXTRACOMPILERFLAGS) -MMD -MP -o $@ -c $<

clean:
	@printf "  CLEAN   ${OUT_DIR}\n"
	$(Q)rm -rf ${OUT_DIR}
	@printf "  CLEAN   $(BINARY)\n"
	$(Q)rm -f $(BINARY)
	@printf "  CLEAN   $(BINARY).bin\n"
	$(Q)rm -f $(BINARY).bin
	@printf "  CLEAN   $(BINARY).srec\n"
	$(Q)rm -f $(BINARY).srec
	@printf "  CLEAN   $(BINARY).list\n"
	$(Q)rm -f $(BINARY).list


.PHONY: directories images clean

get-deps:
	@printf "  GIT SUBMODULE\n"
	$(Q)git submodule update --init
	@printf "  MAKE libopencm3\n"
	$(Q)${MAKE} -C libopencm3 TARGETS=stm32/f4

Test:
	cd test && $(MAKE)
cleanTest:
	cd test && $(MAKE) clean
