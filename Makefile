TARGET = main

# Default target chip.
MCU ?= STM32F103C6

# Define the linker script location and chip architecture.
LD_SCRIPT = STM32F103x6_FLASH.ld

MCU_SPEC = cortex-m3

# Toolchain definitions (ARM bare metal defaults)
CC = arm-none-eabi-gcc
AS = arm-none-eabi-as
LD = arm-none-eabi-ld
OC = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
OS = arm-none-eabi-size

# Assembly directives.
ASFLAGS += -c
ASFLAGS += -O0
ASFLAGS += -mcpu=$(MCU_SPEC)
ASFLAGS += -mthumb
ASFLAGS += -Wall
# (Set error messages to appear on a single line.)
ASFLAGS += -fmessage-length=0

# C compilation directives
CFLAGS += -mcpu=$(MCU_SPEC)
CFLAGS += -mthumb
CFLAGS += -Wall
CFLAGS += -g
# (Set error messages to appear on a single line.)
CFLAGS += -fmessage-length=0
# (Set system to ignore semihosted junk)
CFLAGS += --specs=nosys.specs

# Linker directives.
LSCRIPT = ./ld/$(LD_SCRIPT)
LFLAGS += -mcpu=$(MCU_SPEC)
LFLAGS += -mthumb
LFLAGS += -Wall
LFLAGS += --specs=nano.specs
LFLAGS += --specs=nosys.specs
LFLAGS += -Os
LFLAGS += -Wl,--gc-sections,--relax
LFLAGS += -Wl,--start-group -lc -lgcc -lm -lstdc++ -Wl,--end-group
LFLAGS += -T$(LSCRIPT)

AS_SRC   =  ./boot_code/startup_stm32f103x6.s
C_SRC    =  ./boot_code/syscalls.c
C_SRC    += ./boot_code/system_stm32f1xx.c
# C_SRC    += ./src/nvic.c
C_SRC    += ./src/main.c

INCLUDE  =  -I./
INCLUDE  += -I./include

OBJS  = $(AS_SRC:.s=.o)
OBJS += $(C_SRC:.c=.o)

.PHONY: all
all: flash

%.o: %.s
	$(CC) -x assembler-with-cpp $(ASFLAGS) $< -o $@

%.o: %.c
	$(CC) -c $(CFLAGS) $(INCLUDE) -DSTM32F1 -DSTM32F103x6 $< -o $@

$(TARGET).elf: $(OBJS)
	$(CC) $^ $(LFLAGS) -o $@

$(TARGET).bin: $(TARGET).elf
	$(OC) -S -O binary $< $@
	$(OS) $<

flash: $(TARGET).bin
	st-flash --connect-under-reset write $(TARGET).bin 0x08000000

.PHONY: clean
clean:
	rm -f $(OBJS)
	rm -f $(TARGET).elf
	rm -f $(TARGET).bin