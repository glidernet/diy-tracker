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

CC_SRC += $(wildcard stm_lib/src/*.c)
CC_SRC += $(wildcard cmsis_boot/*.c)
CC_SRC += $(wildcard cmsis_boot/startup/*.c)
CC_SRC += $(wildcard FreeRTOS_8.2.0/Source/*.c)
CC_SRC += $(wildcard FreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3/*.c)
CC_SRC += FreeRTOS_8.2.0/Source/portable/MemMang/heap_4.c

H_SRC  += $(wildcard *.h)
H_SRC  += $(wildcard cmsis/*.h)
H_SRC  += $(wildcard cmsis_boot/*.h)
H_SRC  += $(wildcard stm_lib/inc/*.h)
H_SRC  += $(wildcard FreeRTOS_8.2.0/Source/include/*.h)
H_SRC  += $(wildcard FreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3/*.h)

INCDIR     = -I.
INCDIR    += -Icmsis -Icmsis_boot
INCDIR    += -Istm_lib/inc
INCDIR    += -IFreeRTOS_8.2.0/Source/include -IFreeRTOS_8.2.0/Source/portable/GCC/ARM_CM3

DEFS    = -D$(MCU) -DSTM32F10X_MD -DUSE_STDPERIPH_DRIVER -D__ASSEMBLY__ -DSUPPORT_CPLUSPLUS -DGCC_ARMCM3

CC_OBJ     = $(CC_SRC:.c=.o)
CPP_OBJ    = $(CPP_SRC:.cpp=.o)

CC_OPT  = -mcpu=cortex-m3 -mthumb -Wall -ffunction-sections -g -Os -fstack-usage
CPP_OPT = -mcpu=cortex-m3 -mthumb -Wall -ffunction-sections -g -Os -fstack-usage

LDSCRIPT = link.ld

LNK_OPT  = -Wl,-Map=main.map -Wl,--gc-sections
LNK_OPT += -nostartfiles --specs=nosys.specs --specs=nano.specs
LNK_OPT += -lgcc -lc -lstdc++
LNK_OPT += -T$(LDSCRIPT)

all:	main.elf main.hex main.bin main.dmp

$(CC_OBJ) : %.o : %.c makefile $(H_SRC)
	$(CC)  -c $(CC_OPT)  $(INCDIR) $(DEFS) $< -o $@

$(CPP_OBJ) : %.o : %.cpp makefile $(H_SRC)
	$(CPP) -c $(CPP_OPT) $(INCDIR) $(DEFS) $< -o $@

main.elf:       $(CC_OBJ) $(CPP_OBJ) $(H_SRC) makefile
	$(CPP) $(CC_OPT) $(LNK_OPT) -o $@ $(CC_OBJ) $(CPP_OBJ)
#	$(NM) -S --numeric-sort $@
#	$(NM) -S --size-sort $@
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
