# Tool chain downloaded from: https://launchpad.net/gcc-arm-embedded
# unpacked to a directory: TPATH

TPATH = ../gcc-arm-none-eabi-4.9/bin
TCHAIN = arm-none-eabi

#-------------------------------------------------------------------------------

# list of optional features to be compiled in:
# i2c1 ... I2C-1 interface
# sdcard ... SD card interface with FAT filesystem
# bmp180 ... barometric pressure bmp180 sensor (selects i2c1 too)
# sdlog ... logging to sdcard (selects sdcard too)
# beeper ... beeper (vario etc)
# knob ... user knob to set volume and options

WITH_OPTS = beeper

MCU = STM32F103C8  # STM32F103C8 for no-name STM32F1 board, STM32F103CB for Maple mini

#-------------------------------------------------------------------------------

C_SRC += main.cpp

C_SRC += gps.cpp
C_SRC += rf.cpp
C_SRC += ctrl.cpp
C_SRC += sens.cpp
C_SRC += knob.cpp

C_SRC += uart1.cpp
C_SRC += uart2.cpp
C_SRC += spi1.cpp
C_SRC += spi2.cpp

C_SRC += format.cpp
C_SRC += ldpc.cpp
C_SRC += bitcount.cpp

C_SRC += adc.cpp
C_SRC += nmea.cpp

#-------------------------------------------------------------------------------

INCDIR += -I.

INCDIR += -Icmsis -Icmsis_boot
C_SRC  += $(wildcard cmsis_boot/*.c)
C_SRC  += $(wildcard cmsis_boot/startup/*.c)

INCDIR += -Istm_lib/inc
C_SRC  += $(wildcard stm_lib/src/*.c)

INCDIR += -IFreeRTOS_8.2.0/Source/include -IFreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3
C_SRC  += $(wildcard FreeRTOS_8.2.0/Source/*.c)
C_SRC  += $(wildcard FreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3/*.c)
C_SRC  += FreeRTOS_8.2.0/Source/portable/MemMang/heap_4.c

# ..............................................................................

WITH_OPTS ?=
WITH_DEFS =

ifneq ($(findstring bmp180,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_BMP180
  WITH_OPTS += i2c1
  C_SRC += atmosphere.cpp
  C_SRC += intmath.cpp
endif

ifneq ($(findstring i2c1,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_I2C1
  C_SRC  += i2c.cpp
endif

ifneq ($(findstring sdlog,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_SDLOG
  WITH_OPTS += sdcard
endif

ifneq ($(findstring sdcard,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_SDCARD
  INCDIR += -Ifatfs
  C_SRC  += sd.cpp
  C_SRC  += fatfs/ff.c
endif

ifneq ($(findstring beeper,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_BEEPER
  C_SRC += beep.cpp
endif

ifneq ($(findstring knob,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_KNOB
endif

ifneq ($(findstring config,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_CONFIG
endif

#-------------------------------------------------------------------------------

CC      = $(TPATH)/$(TCHAIN)-gcc
CPP     = $(TPATH)/$(TCHAIN)-g++
OBJCOPY = $(TPATH)/$(TCHAIN)-objcopy
OBJDUMP = $(TPATH)/$(TCHAIN)-objdump
ARCH    = $(TPATH)/$(TCHAIN)-ar
SIZE    = $(TPATH)/$(TCHAIN)-size
NM      = $(TPATH)/$(TCHAIN)-nm

MKDIR   = mkdir

OUTDIR  = build
OBJDIR  = $(OUTDIR)/obj

#-------------------------------------------------------------------------------

DEFS  = -D$(MCU) -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__
DEFS += -DSUPPORT_CPLUSPLUS -DGCC_ARMCM3 -DSPEEDUP_STM_LIB
DEFS += $(WITH_DEFS)

# generate dependency files
C_OPT_DEPS = -MD -MP -MF $(OBJDIR)/$(@F).d

Cx_OPT  = -mcpu=cortex-m3 -mthumb -Wall -ffunction-sections
Cx_OPT += -g -Os -fstack-usage
Cx_OPT += -fno-exceptions -fno-unwind-tables
Cx_OPT += $(C_OPT_DEPS)

CC_OPT  = $(Cx_OPT)
CPP_OPT = $(Cx_OPT) -fno-rtti

LDSCRIPT = link.ld

LNK_OPT  = -Wl,-Map=$(OUTDIR)/main.map -Wl,--cref -Wl,--emit-relocs -Wl,--gc-sections
LNK_OPT += -nostartfiles --specs=nosys.specs --specs=nano.specs
LNK_OPT += -lgcc -lc # -lstdc++
LNK_OPT += -T$(LDSCRIPT)

OBJECTS = $(patsubst %,$(OBJDIR)/%.o,$(basename $(notdir $(C_SRC))))
vpath %.c $(sort $(dir $(C_SRC)))
vpath %.cpp $(sort $(dir $(C_SRC)))

#-------------------------------------------------------------------------------

OTHER_DEPS = makefile

all: $(OUTDIR) $(OBJDIR) size-before $(OUTDIR)/main.elf $(OUTDIR)/main.hex $(OUTDIR)/main.bin $(OUTDIR)/main.dmp size-after

# display binary code size
size-before size-after:
ifneq (,$(wildcard $(OUTDIR)/main.elf))
	@echo "-"
	@echo "-$@:"
	@$(SIZE) --format=berkeley $(OUTDIR)/main.elf
endif

#-------------------------------------------------------------------------------

$(OBJDIR)/%.o: %.c $(OTHER_DEPS)
	@echo "+ compile C file         ... $(notdir $<)"
	@$(CC) -c $(CC_OPT)  $(INCDIR) $(DEFS) $< -o $@

$(OBJDIR)/%.o: %.cpp $(OTHER_DEPS)
	@echo "+ compile C++ file       ... $(notdir $<)"
	@$(CPP) -c $(CPP_OPT) $(INCDIR) $(DEFS) $< -o $@

$(OUTDIR)/main.elf: $(OBJECTS) $(OTHER_DEPS)
	@echo "> link executable        ... $(notdir $@)"
	@$(CPP) $(CC_OPT) $(LNK_OPT) -o $@ $(OBJECTS)
#	$(NM) -S --numeric-sort $@
#	$(NM) -S --size-sort $@
#	$(SIZE) -A -x $@

#-------------------------------------------------------------------------------

$(OUTDIR)/main.hex: $(OUTDIR)/main.elf
	@echo "> create hex image       ... $(notdir $@)"
	@$(OBJCOPY) -O ihex $< $@

$(OUTDIR)/main.bin: $(OUTDIR)/main.elf
	@echo "> create bin image       ... $(notdir $@)"
	@$(OBJCOPY) -O binary $< $@

$(OUTDIR)/main.dmp: $(OUTDIR)/main.elf
	@echo "> create exe dump        ... $(notdir $@)"
	@$(OBJDUMP) -d -S $< > $@

$(OUTDIR):
	$(MKDIR) -p $@

$(OBJDIR):
	$(MKDIR) -p $@

clean:
	-rm -fR $(OUTDIR)

arch:	clean
	tar cvzf diy-tracker.tgz makefile *.h *.c* *.ld *.py cmsis cmsis_boot stm_lib FreeRTOS_8.2.0 # free_rtos # FRT_Library

#-------------------------------------------------------------------------------

# Include the dependency files.
-include $(wildcard $(OBJDIR)/*.d) faked.include.file
