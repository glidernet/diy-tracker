# Tool chain downloaded from: https://launchpad.net/gcc-arm-embedded
# unpacked to a directory: TPATH

TPATH = ../gcc-arm-none-eabi-4.9/bin
TCHAIN = arm-none-eabi

MCU = STM32F103C8  # STM32F103C8 for no-name STM32F1 board, STM32F103CB for Maple mini

CC      = $(TPATH)/$(TCHAIN)-gcc
CPP     = $(TPATH)/$(TCHAIN)-g++
OBJCOPY = $(TPATH)/$(TCHAIN)-objcopy
OBJDUMP = $(TPATH)/$(TCHAIN)-objdump
ARCH    = $(TPATH)/$(TCHAIN)-ar
SIZE    = $(TPATH)/$(TCHAIN)-size
NM      = $(TPATH)/$(TCHAIN)-nm

CPP_SRC += main.cpp

CPP_SRC += gps.cpp
CPP_SRC += rf.cpp
CPP_SRC += ctrl.cpp
CPP_SRC += sens.cpp

CPP_SRC += uart1.cpp
CPP_SRC += uart2.cpp
CPP_SRC += spi1.cpp
CPP_SRC += spi2.cpp

CPP_SRC += beep.cpp
CPP_SRC += format.cpp
CPP_SRC += ldpc.cpp
CPP_SRC += bitcount.cpp
CPP_SRC += intmath.cpp

CPP_SRC += atmosphere.cpp

INCDIR += -I.
H_SRC  += $(wildcard *.h)

INCDIR += -Icmsis -Icmsis_boot
H_SRC  += $(wildcard cmsis/*.h)
H_SRC  += $(wildcard cmsis_boot/*.h)
CC_SRC += $(wildcard cmsis_boot/*.c)
CC_SRC += $(wildcard cmsis_boot/startup/*.c)

INCDIR += -Istm_lib/inc
H_SRC  += $(wildcard stm_lib/inc/*.h)
CC_SRC += $(wildcard stm_lib/src/*.c)

INCDIR += -IFreeRTOS_8.2.0/Source/include -IFreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3
H_SRC  += $(wildcard FreeRTOS_8.2.0/Source/include/*.h)
H_SRC  += $(wildcard FreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3/*.h)
CC_SRC += $(wildcard FreeRTOS_8.2.0/Source/*.c)
CC_SRC += $(wildcard FreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3/*.c)
CC_SRC += FreeRTOS_8.2.0/Source/portable/MemMang/heap_4.c

INCDIR  += -Ifatfs
H_SRC   += $(wildcard fatfs/*.h)
CPP_SRC += sd.cpp
CC_SRC  += fatfs/ff.c

H_SRC   += i2c.h
CPP_SRC += i2c.cpp

H_SRC   += adc.h
CPP_SRC += adc.cpp

H_SRC   += bmp180.h
# CPP_SRC += bmp180.cpp

CPP_SRC += nmea.cpp

DEFS    = -D$(MCU) -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -DSUPPORT_CPLUSPLUS -DGCC_ARMCM3 -DSPEEDUP_STM_LIB

CC_OBJ     = $(CC_SRC:.c=.o)
CPP_OBJ    = $(CPP_SRC:.cpp=.o)

CC_OPT  = -mcpu=cortex-m3 -mthumb -Wall -ffunction-sections -g -Os -fstack-usage
CPP_OPT = -mcpu=cortex-m3 -mthumb -Wall -ffunction-sections -g -Os -fstack-usage

CC_OPT  += -fno-exceptions -fno-unwind-tables           # reduced the code size by 4kB
CPP_OPT += -fno-exceptions -fno-unwind-tables -fno-rtti

LDSCRIPT = link.ld

LNK_OPT  = -Wl,-Map=main.map -Wl,--cref -Wl,--emit-relocs -Wl,--gc-sections
LNK_OPT += -nostartfiles --specs=nosys.specs --specs=nano.specs
LNK_OPT += -lgcc -lc # -lstdc++
LNK_OPT += -T$(LDSCRIPT)

all:	main.elf main.hex main.bin main.dmp

$(CC_OBJ) : %.o : %.c makefile $(H_SRC)
	$(CC)  -c $(CC_OPT)  $(INCDIR) $(DEFS) $< -o $@

$(CPP_OBJ) : %.o : %.cpp makefile $(H_SRC)
	$(CPP) -c $(CPP_OPT) $(INCDIR) $(DEFS) $< -o $@

main.elf:       $(CC_OBJ) $(CPP_OBJ) $(H_SRC) makefile
	$(CPP) $(CC_OPT) $(LNK_OPT) -o $@ $(CC_OBJ) $(CPP_OBJ)
	$(NM) -S --numeric-sort $@
	$(NM) -S --size-sort $@
	$(SIZE) -A -x $@

main.hex:       main.elf
	$(OBJCOPY) -O ihex $< $@

main.bin:       main.elf
	$(OBJCOPY) -O binary $< $@

main.dmp:       main.elf
	$(OBJDUMP) -d -S $< > $@

clean:
	rm -f main.elf main.map main.hex main.bin main.dmp $(CC_OBJ) $(CPP_OBJ) *.o *.su */*.su */*/*.su */*/*/*.su */*/*/*/*.su */*/*/*/*/*.su

arch:	clean
	tar cvzf diy-tracker.tgz makefile *.h *.c* *.ld *.py cmsis cmsis_boot stm_lib FreeRTOS_8.2.0 # free_rtos # FRT_Library
