# Tool chain downloaded from: https://launchpad.net/gcc-arm-embedded
# unpacked to a directory: TPATH

# TPATH = ../../gcc-arm-none-eabi-4.9/bin
# TPATH = ../gcc-arm-none-eabi-5.4/bin
TPATH = /usr/bin
TCHAIN = arm-none-eabi

#-------------------------------------------------------------------------------

# list of optional features to be compiled in:
# i2c1          ... I2C-1 interface
# sdcard        ... SD card interface with FAT filesystem
# bmp180        ... barometric pressure bmp180 sensor
# bmp280        ... barometric pressure bmp280 sensor
# beeper        ... beeper
# vario         ... vario sound (selects beeper)
# sdlog         ... logging to sdcard (selects sdcard too)
# knob          ... user knob to set volume and options on PB0
# batt_sense    ... measure battery voltage with R-R divider to pin PB1
# config        ... setting the aircraft-type, address-type, address, etc. through the serial port

# rfm69
# rfm69w        ... for the lower tx power RF chip
# rfm95
# sx1272		... for sx1272

# relay         ... packet-relay code (conditional code not implemented yet)
# pps_irq       ... PPS signal makes an IRQ and the RTOS clock is adjusted in frequency to mathc the GPS (but not very precise)
# gps_pps       ... GPS does deliver PPS, otherwise we get the timing from when the GPS starts sending serial data
# gps_enable    ... GPS senses the "enable" line so it is possibly to shut it down
# gps_ubx_pass  ... pass UBX messages between the console and the GPS - for GPS configuration
# gps_nmea_pass ... pass (P-private) NMEA messages between the console and the GPS - for GPS configuration

# blue_pill     ... use Blue Pill STM32F103c8t6 board
# maple_mini    ... use Maple Mini STM32F103cbt6 board
# ogn_cube_1    ... Tracker hardware by Miroslav Cervenka
# swap_uarts    ... use UART1 for GPS and UART2 for console

# lookout       ...

# WITH_OPTS = blue_pill rfm69 beeper vario i2c1 bmp180 knob  relay config pps_irq # for regular tracker with a knob and BMP180 but no SD card
# WITH_OPTS = blue_pill rfm69 beeper vario i2c1 bmp180 sdlog relay config # for the test system (no knob but the SD card)
# WITH_OPTS = blue_pill rfm69 beeper vario i2c1 bmp180 lookout relay config gps_pps gps_enable gps_ubx_pass gps_nmea_pass pps_irq
# WITH_OPTS = maple_mini rfm69 i2c1 bmp180 relay config gps_pps pps_irq gps_enable
# WITH_OPTS = blue_pill rfm69 beeper i2c1 bmp180 relay lookout config gps_pps pps_irq gps_enable gps_ubx_pass gps_nmea_pass
# WITH_OPTS = blue_pill rfm69 beeper vario i2c1 bmp180 relay config gps_pps pps_irq gps_enable
# WITH_OPTS = blue_pill rfm69 beeper relay config
# WITH_OPTS = blue_pill rfm95 beeper vario i2c1 bmp280 relay config
WITH_OPTS = blue_pill beeper vario i2c1 bmp280 config gps_pps pps_irq batt_sense rf_irq sx1272 relay

# WITH_OPTS = rfm69 relay config swap_uarts i2c2 bmp280 ogn_cube_1 # for OGN-CUBE-1

# MCU = STM32F103C8  # STM32F103C8 for no-name STM32F1 board, STM32F103CB for Maple mini

RTOS = FreeRTOS_9.0.0

#-------------------------------------------------------------------------------

C_SRC += main.cpp
C_SRC += hal.cpp

C_SRC += gps.cpp
C_SRC += rf.cpp
C_SRC += ctrl.cpp
C_SRC += sens.cpp
C_SRC += knob.cpp

C_SRC += uart.cpp
C_SRC += uart1.cpp
C_SRC += uart2.cpp
C_SRC += spi1.cpp
C_SRC += spi2.cpp

C_SRC += format.cpp
C_SRC += ldpc.cpp
C_SRC += bitcount.cpp
C_SRC += intmath.cpp

C_SRC += adc.cpp
C_SRC += nmea.cpp

#-------------------------------------------------------------------------------

INCDIR += -I.

INCDIR += -Icmsis -Icmsis_boot
C_SRC  += $(wildcard cmsis_boot/*.c)
C_SRC  += $(wildcard cmsis_boot/startup/*.c)

INCDIR += -Istm_lib/inc
C_SRC  += $(wildcard stm_lib/src/*.c)

INCDIR += -I$(RTOS)/Source/include -I$(RTOS)/Source/portable/GCC/ARM_CM3
C_SRC  += $(wildcard $(RTOS)/Source/*.c)
C_SRC  += $(wildcard $(RTOS)/Source/portable/GCC/ARM_CM3/*.c)
C_SRC  += $(RTOS)/Source/portable/MemMang/heap_4.c

# ..............................................................................

WITH_OPTS ?=
WITH_DEFS =

ifneq ($(findstring bmp180,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_BMP180
  C_SRC += atmosphere.cpp
endif

ifneq ($(findstring bmp280,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_BMP280
  C_SRC += atmosphere.cpp
endif

ifneq ($(findstring i2c1,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_I2C1
  WITH_DEFS += -DBARO_I2C=I2C1
  C_SRC  += i2c.cpp
endif

ifneq ($(findstring i2c2,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_I2C2
  WITH_DEFS += -DBARO_I2C=I2C2
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

ifneq ($(findstring vario,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_VARIO
  WITH_OPTS += beeper
endif

ifneq ($(findstring beeper,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_BEEPER
  C_SRC += beep.cpp
endif

ifneq ($(findstring knob,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_KNOB
endif

ifneq ($(findstring batt_sense,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_BATT_SENSE
endif

ifneq ($(findstring config,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_CONFIG
endif

ifneq ($(findstring rfm69,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_RFM69
endif

ifneq ($(findstring rfm69w,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_RFM69W
endif

ifneq ($(findstring rfm95,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_RFM95
endif

ifneq ($(findstring sx1272,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_SX1272
endif

ifneq ($(findstring rf_irq,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_RF_IRQ
endif

ifneq ($(findstring relay,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_RELAY
endif

ifneq ($(findstring lookout,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_LOOKOUT
endif

ifneq ($(findstring gps_pps,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_GPS_PPS
ifneq ($(findstring pps_irq,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_PPS_IRQ
endif
endif

ifneq ($(findstring gps_enable,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_GPS_ENABLE
endif

ifneq ($(findstring gps_ubx_pass,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_GPS_UBX_PASS
endif

ifneq ($(findstring gps_nmea_pass,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_GPS_NMEA_PASS
endif

ifneq ($(findstring swap_uarts,$(WITH_OPTS)),)
  WITH_DEFS += -DWITH_SWAP_UARTS
  WITH_DEFS += -DUART1_RxFIFO_Size=32    # GPS
  WITH_DEFS += -DUART1_TxFIFO_Size=32
  WITH_DEFS += -DUART2_RxFIFO_Size=32    # console
  WITH_DEFS += -DUART2_TxFIFO_Size=512
else
  WITH_DEFS += -DUART1_RxFIFO_Size=32    # console
  WITH_DEFS += -DUART1_TxFIFO_Size=512
  WITH_DEFS += -DUART2_RxFIFO_Size=32    # GPS
  WITH_DEFS += -DUART2_TxFIFO_Size=32
endif

MCU = STM32F103C8

ifneq ($(findstring blue_pill,$(WITH_OPTS)),)
  MCU = STM32F103C8
  WITH_DEFS += -DWITH_BLUE_PILL
endif

ifneq ($(findstring maple_mini,$(WITH_OPTS)),)
  MCU = STM32F103CB
  WITH_DEFS += -DWITH_MAPLE_MINI
endif

ifneq ($(findstring ogn_cube_1,$(WITH_OPTS)),)
  MCU = STM32F103CB
  WITH_DEFS += -DWITH_OGN_CUBE_1
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

$(info DEFS = $(DEFS))
# $(info WITH_DEFS = $(WITH_DEFS))

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

# Flash using an st-link adapter
flash-stlink: $(OUTDIR)/main.hex
	st-flash --format ihex write $(OUTDIR)/main.hex

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
	tar cvzf diy-tracker.tgz makefile *.h *.cc *.cpp *.ld *.py cmsis cmsis_boot stm_lib FreeRTOS_8.2.0 FreeRTOS_9.0.0 # free_rtos # FRT_Library

#-------------------------------------------------------------------------------

# Include the dependency files.
-include $(wildcard $(OBJDIR)/*.d) faked.include.file
