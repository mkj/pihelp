# Name: Makefile
# Author: <insert your name here>
# Copyright: <insert your copyright message here>
# License: <insert your license reference here>

# This is a prototype Makefile. Modify it according to your needs.
# You should at least check the settings for
# DEVICE ....... The AVR device you compile for
# CLOCK ........ Target AVR clock rate in Hertz
# OBJECTS ...... The object files created from your source files. This list is
#                usually the same as the list of source files with suffix ".o".
# PROGRAMMER ... Options to avrdude which define the hardware you use for
#                uploading to the AVR and the interface where this hardware
#                is connected. We recommend that you leave it undefined and
#                add settings like this to your ~/.avrduderc file:
#                   default_programmer = "stk500v2"
#                   default_serial = "avrdoper"
# FUSES ........ Parameters for avrdude to flash the fuses appropriately.

DEVICE     = atmega328p
PROGDEVICE     = atmega328p
CLOCK      = 4915200
PROGRAMMER = #-c stk500v2 -P avrdoper
PROGRAMMER = -c stk500 -P ~/dev/stk500 -p $(PROGDEVICE)  -B 2
SOURCE_1WIRE = onewire.c simple_ds18b20.c crc8.c
SOURCE_CRYPTO = hmac-sha1.c sha1-asm.S aes.c
SOURCE_SD = byteordering.c fat.c  partition.c sd_raw.c 
SOURCE    = main.c
SOURCE += $(SOURCE_CRYPTO) $(SOURCE_SD)
LIBS       = 

OBJECTS := $(patsubst %.c,%.o,$(patsubst %.S,%.o,$(SOURCE)))

# default but 2mhz
FUSES      = -U hfuse:w:0xd9:m -U lfuse:w:0x77:m -U efuse:w:0xfd:m

# ATMega8 fuse bits used above (fuse bits for other devices are different!):
# Example for 8 MHz internal oscillator
# Fuse high byte:
# 0xd9 = 1 1 0 1   1 0 0 1 <-- BOOTRST (boot reset vector at 0x0000)
#        ^ ^ ^ ^   ^ ^ ^------ BOOTSZ0
#        | | | |   | +-------- BOOTSZ1
#        | | | |   +---------- EESAVE (set to 0 to preserve EEPROM over chip erase)
#        | | | +-------------- WDTON
#        | | +---------------- SPIEN (if set to 1, serial programming is disabled)
#        | +------------------ DWEN
#        +-------------------- RSTDISBL (if set to 0, RESET pin is disabled)
# Fuse low byte:
# 0x77   0 1 1 1   0 1 1 1  (low rising, full swing, crystal)
# 0x62 = 0 1 1 0   0 0 1 0
#        ^ ^ \ /   \--+--/
#        | |  |       +------- CKSEL 3..0 (8M internal RC)
#        | |  +--------------- SUT 1..0 (slowly rising power)
#        | +------------------ CKOUT
#        +-------------------- CLKDIV8 (initial)

# Extended fuse:
# 0xfd = 2.7V BOD
#   
#
# For computing fuse byte values for other devices and options see
# the fuse bit calculator at http://www.engbedded.com/fusecalc/


# Tune the lines below only if you know what you are doing:

SHELL := /bin/bash
export PATH := $(PATH):/usr/local/CrossPack-AVR/bin/

AVRDUDE = avrdude $(PROGRAMMER) 
#COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -g -std=c99 -mcall-prologues -fdata-sections -ffunction-sections  -Wl,--gc-sections -Wl,--relax -fwhole-program  -Wl,-u,vfprintf -lprintf_flt -lm
COMPILE = /usr/local/CrossPack-AVR/bin/avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) -g -std=c99  -mcall-prologues -fdata-sections -ffunction-sections  -Wl,--gc-sections -Wl,--relax -fwhole-program -flto

# symbolic targets:
all:	main.hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@
# "-x assembler-with-cpp" should not be necessary since this is the default
# file type for the .S (with capital S) extension. However, upper case
# characters are not always preserved on Windows. To ensure WinAVR
# compatibility define the file type manually.

.c.s:
	$(COMPILE) -S $< -o $@

flash:	all
	$(AVRDUDE) -U flash:w:main.hex:i

checkprog:	
	$(AVRDUDE) -v 

fuse:
	$(AVRDUDE) $(FUSES)

# Xcode uses the Makefile targets "", "clean" and "install"
install: flash 

# if you use a bootloader, change the command below appropriately:
load: all
	bootloadHID main.hex

clean:
	rm -f main.hex main.elf $(OBJECTS)

# file targets:
main.elf: $(SOURCE)
	$(COMPILE) -o main.elf $(SOURCE) $(LIBS)

main.hex: main.elf
	rm -f main.hex
	avr-objcopy -j .text -j .data -O ihex main.elf main.hex
	avr-size --format=avr --mcu=$(DEVICE) main.elf
# If you have an EEPROM section, you must also create a hex file for the
# EEPROM and add it to the "flash" target.

# Targets for code debugging and analysis:
disasm:	main.elf
	avr-objdump -d main.elf

cpp:
	$(COMPILE) -E main.c
