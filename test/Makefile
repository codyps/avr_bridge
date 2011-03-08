# Hey Emacs, this is a -*- makefile -*-
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging, 
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------

# MCU name
MCU = atmega168
.PREFIX:
F_CPU = 16000000

# Output format. (can be srec, ihex, binary)
FORMAT = ihex

# Target file name (without extension).
TARGET = main

# Object files directory
#     To put object files in current directory, use a dot (.), do NOT make
#     this an empty or blank macro!
OBJDIR = .

srcdir = .

VPATH = $(srcdir)
VERSION= arduino-ros-unknown
#VERSION = $(shell ./setlocalversion)

# List source files here.
SRC = $(wildcard *.c) $(wildcard *.cpp) $(wildcard avr_ros/*.cpp)

# List Assembler source files here.
#     Make them always end in a capital .S.  Files ending in a lowercase .s
#     will not be considered source files but generated files (assembler
#     output from the compiler), and will be deleted upon "make clean"!
#     Even though the DOS/Win* filesystem matches both .s and .S the same,
#     it will preserve the spelling of the filenames, and gcc itself does
#     care about how the name is spelled on its command-line.
ASRC =

# Optimization level, can be [0, 1, 2, 3, s]. 
#     0 = turn off optimization. s = optimize for size.
#     (Note: 3 is not always the best optimization level. See avr-libc FAQ.)
OPT = s

# Debugging format.
#     Native formats for AVR-GCC's -g are dwarf-2 [default] or stabs.
#     AVR Studio 4.10 requires dwarf-2.
#     AVR [Extended] COFF format requires stabs, plus an avr-objcopy run.
DEBUG = dwarf-2

# List any extra directories to look for include files here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRAINCDIRS =

# Compiler flag to set the C Standard level.
#     c89   = "ANSI" C
#     gnu89 = c89 plus GCC extensions
#     c99   = ISO C99 standard (not yet fully implemented)
#     gnu99 = c99 plus GCC extensions
CSTANDARD = -std=gnu99

# Place -D or -U options here for C sources
CDEFS = -DF_CPU=$(F_CPU)UL

# Place -D or -U options here for ASM sources
ADEFS = -DF_CPU=$(F_CPU)

# Place -D or -U options here for C++ sources
CXXDEFS = -DF_CPU=$(F_CPU)UL
#CXXDEFS += -D__STDC_LIMIT_MACROS
#CXXDEFS += -D__STDC_CONSTANT_MACROS



#---------------- Compiler Options C ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CFLAGS = -g$(DEBUG)
CFLAGS += $(CDEFS)
CFLAGS += -O$(OPT)
CFLAGS += -pipe
CFLAGS += -funsigned-char
CFLAGS += -funsigned-bitfields
CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
#CFLAGS += -fwrapv
CFLAGS += -Wall
CFLAGS += -Wextra
CFLAGS += -Wreturn-type
CFLAGS += -Wunused
CFLAGS += -Wstrict-prototypes
#CFLAGS += -mshort-calls
#CFLAGS += -fno-unit-at-a-time
CFLAGS += -Wundef
CFLAGS += -Wno-main
#CFLAGS += -Wunreachable-code
CFLAGS += -Wsign-compare
CFLAGS += -DVERSION='"$(VERSION)"'
#CFLAGS += -Wa,-adhlns=$(<:%.c=$(OBJDIR)/%.lst)
CFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))

CFLAGS += $(CSTANDARD)


#---------------- Compiler Options C++ ----------------
#  -g*:          generate debugging information
#  -O*:          optimization level
#  -f...:        tuning, see GCC manual and avr-libc documentation
#  -Wall...:     warning level
#  -Wa,...:      tell GCC to pass this to the assembler.
#    -adhlns...: create assembler listing
CXXFLAGS = -g$(DEBUG)
CXXFLAGS += $(CXXDEFS)
CXXFLAGS += -O$(OPT)
CXXFLAGS += -funsigned-char
CXXFLAGS += -funsigned-bitfields
CXXFLAGS += -fpack-struct
CXXFLAGS += -fshort-enums
CXXFLAGS += -fno-exceptions
CXXFLAGS += -Wall
CXXFLAGS += -Wundef
#CXXFLAGS += -mshort-calls
#CXXFLAGS += -fno-unit-at-a-time
#CXXFLAGS += -Wstrict-prototypes
CXXFLAGS += -Wunreachable-code
#CXXFLAGS += -Wsign-compare
#CXXFLAGS += -Wa,-adhlns=$(<:%.cpp=$(OBJDIR)/%.lst)
CXXFLAGS += $(patsubst %,-I%,$(EXTRAINCDIRS))
#CXXFLAGS += $(CSTANDARD)

#---------------- Assembler Options ----------------
#  -Wa,...:   tell GCC to pass this to the assembler.
#  -adhlns:   create listing
#  -gstabs:   have the assembler create line number information; note that
#             for use in COFF files, additional information about filenames
#             and function names needs to be present in the assembler source
#             files -- see avr-libc docs [FIXME: not yet described there]
#  -listing-cont-lines: Sets the maximum number of continuation lines of hex 
#       dump that will be displayed for a given single line of source input.
ASFLAGS = $(ADEFS) -Wa,-adhlns=$(<:%.S=$(OBJDIR)/%.lst),-gstabs,--listing-cont-lines=100

#---------------- Library Options ----------------
# Minimalistic printf version
PRINTF_LIB_MIN = -Wl,-u,vfprintf -lprintf_min

# Floating point printf version (requires MATH_LIB = -lm below)
PRINTF_LIB_FLOAT = -Wl,-u,vfprintf -lprintf_flt

# If this is left blank, then it will use the Standard printf version.
PRINTF_LIB = 
#PRINTF_LIB = $(PRINTF_LIB_MIN)
#PRINTF_LIB = $(PRINTF_LIB_FLOAT)

# Minimalistic scanf version
SCANF_LIB_MIN = -Wl,-u,vfscanf -lscanf_min

# Floating point + %[ scanf version (requires MATH_LIB = -lm below)
SCANF_LIB_FLOAT = -Wl,-u,vfscanf -lscanf_flt

# If this is left blank, then it will use the Standard scanf version.
SCANF_LIB = 
#SCANF_LIB = $(SCANF_LIB_MIN)
#SCANF_LIB = $(SCANF_LIB_FLOAT)

MATH_LIB = -lm

# List any extra directories to look for libraries here.
#     Each directory must be seperated by a space.
#     Use forward slashes for directory separators.
#     For a directory that has spaces, enclose it in quotes.
EXTRALIBDIRS = 

#---------------- External Memory Options ----------------

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# used for variables (.data/.bss) and heap (malloc()).
#EXTMEMOPTS = -Wl,-Tdata=0x801100,--defsym=__heap_end=0x80ffff

# 64 KB of external RAM, starting after internal RAM (ATmega128!),
# only used for heap (malloc()).
#EXTMEMOPTS = -Wl,--section-start,.data=0x801100,--defsym=__heap_end=0x80ffff
EXTMEMOPTS =

#---------------- Linker Options ----------------
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LDFLAGS = -Wl,-Map=$(TARGET).map,--cref
LDFLAGS += $(EXTMEMOPTS)
LDFLAGS += $(patsubst %,-L%,$(EXTRALIBDIRS))
LDFLAGS += $(PRINTF_LIB) $(SCANF_LIB) $(MATH_LIB)
#LDFLAGS += -T linker_script.x

#---------------- Programming Options (avrdude) ----------------

# Programming hardware
# Type: avrdude -c ?
# to get a full listing.
#
AVRDUDE_PROGRAMMER = stk500v2

# com1 = serial port. Use lpt1 to connect to parallel port.
AVRDUDE_PORT = com1    # programmer connected to serial device

AVRDUDE_WRITE_FLASH = -U flash:w:$(TARGET).hex
#AVRDUDE_WRITE_EEPROM = -U eeprom:w:$(TARGET).eep

# Uncomment the following if you want avrdude's erase cycle counter.
# Note that this counter needs to be initialized first using -Yn,
# see avrdude manual.
#AVRDUDE_ERASE_COUNTER = -y

# Uncomment the following if you do /not/ wish a verification to be
# performed after programming the device.
#AVRDUDE_NO_VERIFY = -V

# Increase verbosity level.  Please use this when submitting bug
# reports about avrdude. See <http://savannah.nongnu.org/projects/avrdude> 
# to submit bug reports.
#AVRDUDE_VERBOSE = -v -v

AVRDUDE_FLAGS = -p $(MCU) -P $(AVRDUDE_PORT) -c $(AVRDUDE_PROGRAMMER)
AVRDUDE_FLAGS += $(AVRDUDE_NO_VERIFY)
AVRDUDE_FLAGS += $(AVRDUDE_VERBOSE)
AVRDUDE_FLAGS += $(AVRDUDE_ERASE_COUNTER)

#---------------- Debugging Options ----------------

# For simulavr only - target MCU frequency.
DEBUG_MFREQ = $(F_CPU)

# Set the DEBUG_UI to either gdb or insight.
# DEBUG_UI = gdb
DEBUG_UI = insight

# Set the debugging back-end to either avarice, simulavr.
DEBUG_BACKEND = avarice
#DEBUG_BACKEND = simulavr

# GDB Init Filename.
GDBINIT_FILE = __avr_gdbinit

# When using avarice settings for the JTAG
JTAG_DEV = /dev/com1

# Debugging port used to communicate between GDB / avarice / simulavr.
DEBUG_PORT = 4242

# Debugging host used to communicate between GDB / avarice / simulavr, normally
#     just set to localhost unless doing some sort of crazy debugging when 
#     avarice is running on a different computer.
DEBUG_HOST = localhost

#===========================================================================

# Define programs and commands.
SHELL = sh
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
AR = avr-ar rcs
NM = avr-nm
AVRDUDE = avrdude
RM = rm -f
RMDIR = rm -rf
COPY = cp
WINSHELL = cmd

# Define all object files.
OBJ = $(SRC:=.o) 

# Define all listing files.
CCLST = $(CCSRC:=$(OBJDIR)/%.lst)
LST = $(SRC:=$(OBJDIR)/%.lst) $(CCLST) $(ASRC:=$(OBJDIR)/%.lst) 

# Compiler flags to generate dependency files.
GENDEPFLAGS = -MMD -MP -MF .dep/$(@F).d

# Combine all necessary flags and optional flags.
# Add target processor to flags.
ALL_CFLAGS   = -mmcu=$(MCU) -I. $(CFLAGS) $(GENDEPFLAGS) -pipe -D__STDC_LIMIT_MACROS
ALL_CXXFLAGS = -mmcu=$(MCU) -I. -x c++ $(CXXFLAGS) $(GENDEPFLAGS)
ALL_ASFLAGS  = -mmcu=$(MCU) -I. -x assembler-with-cpp $(ASFLAGS)

# Default target.
all: ros build

rebuild: | clean build

# Change the build target to build a HEX file or a library.
build: elf hex eep lss sym
#build: lib

.PHONY: ros
ros:
	rosrun avr_bridge gen_avr.py call_response.yaml .

elf: $(TARGET).elf
hex: $(TARGET).hex
eep: $(TARGET).eep
lss: $(TARGET).lss
sym: $(TARGET).sym
LIBNAME = lib$(TARGET).a
lib: $(LIBNAME)

# Display size of file.
HEXSIZE = $(SIZE) --target=$(FORMAT) $(TARGET).hex
ELFSIZE = $(SIZE) --mcu=$(MCU) --format=avr $(TARGET).elf

size : $(TARGET).size

%.size : %.elf
	$(SIZE) --mcu=$(MCU) $<

sizebefore:
	@if test -f $(TARGET).elf; then $(ELFSIZE); 2>/dev/null; fi

sizeafter:
	@if test -f $(TARGET).elf; then $(ELFSIZE); 2>/dev/null; fi

# Program the device.  
program: $(TARGET).hex $(TARGET).eep
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM)

TTY = /dev/ttyUSB0
fboot: $(TARGET).hex
	fboot -b 115200 -d $(TTY) -p $<
picocom:
	picocom -b 115200 -p o $(TTY)

# Generate avr-gdb config/init file which does the following:
#     define the reset signal, load the target file, connect to target, and set 
#     a breakpoint at main().
gdb-config: 
	@$(RM) $(GDBINIT_FILE)
	@echo define reset >> $(GDBINIT_FILE)
	@echo SIGNAL SIGHUP >> $(GDBINIT_FILE)
	@echo end >> $(GDBINIT_FILE)
	@echo file $(TARGET).elf >> $(GDBINIT_FILE)
	@echo target remote $(DEBUG_HOST):$(DEBUG_PORT)  >> $(GDBINIT_FILE)
ifeq ($(DEBUG_BACKEND),simulavr)
	@echo load  >> $(GDBINIT_FILE)
endif
	@echo break main >> $(GDBINIT_FILE)

debug: gdb-config $(TARGET).elf
ifeq ($(DEBUG_BACKEND), avarice)
	@echo Starting AVaRICE - Press enter when "waiting to connect" message displays.
	@$(WINSHELL) /c start avarice --jtag $(JTAG_DEV) --erase --program --file \
	$(TARGET).elf $(DEBUG_HOST):$(DEBUG_PORT)
	@$(WINSHELL) /c pause

else
	@$(WINSHELL) /c start simulavr --gdbserver --device $(MCU) --clock-freq \
	$(DEBUG_MFREQ) --port $(DEBUG_PORT)
endif
	@$(WINSHELL) /c start avr-$(DEBUG_UI) --command=$(GDBINIT_FILE)

# Convert ELF to COFF for use in debugging / simulating in AVR Studio or VMLAB.
COFFCONVERT = $(OBJCOPY) --debugging
COFFCONVERT += --change-section-address .data-0x800000
COFFCONVERT += --change-section-address .bss-0x800000
COFFCONVERT += --change-section-address .noinit-0x800000
COFFCONVERT += --change-section-address .eeprom-0x810000

coff: $(TARGET).elf
	@echo $(MSG_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-avr $< $(TARGET).cof

extcoff: $(TARGET).elf
	@echo $(MSG_EXTENDED_COFF) $(TARGET).cof
	$(COFFCONVERT) -O coff-ext-avr $< $(TARGET).cof

# Create final output files (.hex, .eep) from ELF output file.
%.hex: %.elf
	@echo OBJCOPY-HEX $@
	@$(OBJCOPY) -O $(FORMAT) -R .eeprom -R .fuse -R .lock $< $@

%.eep: %.elf
	@echo OBJCOPY-EEPROM $@
	@-$(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0 --no-change-warnings -O $(FORMAT) $< $@ || exit 0

# Create extended listing file from ELF output file.
%.lss: %.elf
	@echo OBJDUMP-LSS $@
	@$(OBJDUMP) -h -S -z $< > $@

# Create a symbol table from ELF output file.
%.sym: %.elf
	@echo NM-SYM $@
	@$(NM) -n $< > $@



# Create library from object files.
.SECONDARY : $(TARGET).a
.PRECIOUS : $(OBJ)
%.a: $(OBJ)
	@echo $(MSG_CREATING_LIBRARY) $@
	$(AR) $@ $(OBJ)

# Link: create ELF output file from object files.
.SECONDARY : $(TARGET).elf
.PRECIOUS : $(OBJ)
%.elf: $(OBJ)
	@echo LD $@
	@$(CC) $(ALL_CFLAGS) -o $@ $^ $(LDFLAGS)

# Compile: create object files from C source files.
%.c.o : %.c
	@echo CC $@
	@$(CC) $(ALL_CFLAGS) -c -o $@ $<


# Compile: create object files from C++ source files.
%.cpp.o : %.cpp
	@echo C++ $@
	@$(CC) $(ALL_CXXFLAGS) -c -o $@ $<  


# Compile: create assembler files from C source files.
%.c.s : %.c
	$(CC) -S $(ALL_CFLAGS) -o $@ $<


# Compile: create assembler files from C++ source files.
%.cpp.s : %.cpp
	$(CC) -S $(ALL_CXXFLAGS) -o $@ $<


# Assemble: create object files from assembler source files.
%.S.o : %.S
	@echo AS $@
	@$(CC) $(ALL_ASFLAGS) -c -o $@ $<


# Create preprocessed source for use in sending a bug report.
%.c.i : %.c
	$(CC) -E -mmcu=$(MCU) -I. $(CFLAGS) -o $@ $<

# Target: clean project.
.PHONY: clean_msg %.clean clean
clean_msg:
	@echo CLEAN
%.clean :
	@$(RM) $*.hex $*.eep $*.cof $*.elf $*.map $*.sym $*.lss
clean : | clean_msg $(TARGET).clean
	@$(RM) $(SRC:=$(OBJDIR)/%.o) \
	      $(SRC:=$(OBJDIR)/%.lst) \
	      $(CCSRC:=$(OBJDIR)/%.o) \
	      $(SRC:=.s) \
	      $(SRC:=.d) \
	      $(SRC:=.i)
	@$(RMDIR) .dep

# Create object files directory
$(shell mkdir $(OBJDIR) 2>/dev/null)

# Include the dependency files.
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# Listing of phony targets.
.PHONY : all begin finish end sizebefore sizeafter gccversion \
build elf hex eep lss sym coff extcoff \
clean program debug gdb-config %.clean clean_msg


