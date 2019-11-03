TARGET = main
.DEFAULT_GOAL = all
GDB = arm-none-eabi-gdb
CROSS_COMPILE ?= arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
CFLAGS = -O0 \
	 -std=c99 \
	 -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
	 -mcpu=cortex-m4 -mthumb \
	 -Wall -Werror -nostdlib\
	 -Tstm32_flash.ld \
	 -DUSER_NAME=\"$(USER)\"

ARCH = CM4
VENDOR = ST
PLAT = STM32F4xx

LIBDIR = .
CODEBASE = freertos
CMSIS_LIB = $(CODEBASE)/libraries/CMSIS/$(ARCH)
STM32_LIB = $(CODEBASE)/libraries/STM32F4xx_StdPeriph_Driver

CMSIS_PLAT_SRC = $(CMSIS_LIB)/DeviceSupport/$(VENDOR)/$(PLAT)

FREERTOS_SRC = $(CODEBASE)/libraries/FreeRTOS
FREERTOS_INC = $(FREERTOS_SRC)/include/                                       
FREERTOS_PORT_INC = $(FREERTOS_SRC)/portable/GCC/ARM_$(ARCH)F/

OUTDIR = build
SRCDIR = src\
         $(CMSIS_LIB)/CoreSupport \
         $(STM32_LIB)/src \
         $(CMSIS_PLAT_SRC) \
	 $(FREERTOS_SRC)
INCDIR = include \
         $(CMSIS_LIB)/CoreSupport \
         $(STM32_LIB)/inc \
         $(CMSIS_PLAT_SRC) \
	 $(FREERTOS_INC) \
	 $(FREERTOS_PORT_INC)
INCLUDES = $(addprefix -I,$(INCDIR))
DATDIR = data
TOOLDIR = tool
TMPDIR = output

HEAP_IMPL = heap_ww
SRC = $(wildcard $(addsuffix /*.c,$(SRCDIR))) \
      $(wildcard $(addsuffix /*.s,$(SRCDIR))) \
      $(FREERTOS_SRC)/portable/MemMang/$(HEAP_IMPL).c \
      $(FREERTOS_SRC)/portable/GCC/ARM_CM4F/port.c \
      $(CMSIS_PLAT_SRC)/startup/gcc_ride7/startup_stm32f40xx.s
OBJ := $(addprefix $(OUTDIR)/,$(patsubst %.s,%.o,$(SRC:.c=.o)))
DEP = $(OBJ:.o=.o.d)
DAT =
 
MAKDIR = mk
MAK = $(wildcard $(MAKDIR)/*.mk)

include $(MAK)


all: $(OUTDIR)/$(TARGET).bin $(OUTDIR)/$(TARGET).lst

$(OUTDIR)/$(TARGET).bin: $(OUTDIR)/$(TARGET).elf
	@echo "    OBJCOPY "$@
	@$(CROSS_COMPILE)objcopy -Obinary $< $@

$(OUTDIR)/$(TARGET).lst: $(OUTDIR)/$(TARGET).elf
	@echo "    LIST    "$@
	@$(CROSS_COMPILE)objdump -S $< > $@

$(OUTDIR)/$(TARGET).elf: $(OBJ) $(DAT)
	@echo "    LD      "$@
	@echo "    MAP     "$(OUTDIR)/$(TARGET).map
	@$(CROSS_COMPILE)gcc $(CFLAGS) -Wl,-Map=$(OUTDIR)/$(TARGET).map -o $@ $^

$(OUTDIR)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo "    CC      "$@
	@$(CROSS_COMPILE)gcc $(CFLAGS) -MMD -MF $@.d -o $@ -c $(INCLUDES) $<

$(OUTDIR)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo "    CC      "$@
	@$(CROSS_COMPILE)gcc $(CFLAGS) -MMD -MF $@.d -o $@ -c $(INCLUDES) $<

clean:
	rm -rf $(OUTDIR) $(TMPDIR)

flash: $(OUTDIR)/$(TARGET).bin
	st-flash write $(OUTDIR)/$(TARGET).bin 0x8000000

debug: $(OUTDIR)/$(TARGET).elf
	$(GDB) -tui $(OUTDIR)/$(TARGET).elf

-include $(DEP)



