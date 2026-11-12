.DEFAULT_GOAL := all
TARGET := m3508_can_test
BUILD_DIR ?= build/minimal

TOOLCHAIN_DIR ?=
CROSS_COMPILE ?= $(TOOLCHAIN_DIR)arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size

ifeq ($(OS),Windows_NT)
MKDIR_P = powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File tools/build_helper.ps1 mkdir "$(1)"
REMOVE_DIR = powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File tools/build_helper.ps1 clean "$(1)"
else
MKDIR_P = mkdir -p "$(1)"
REMOVE_DIR = rm -rf "$(1)"
endif

CPU_FLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFINES := -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY
INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32H7xx_HAL_Driver/Inc \
  -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
  -IDrivers/CMSIS/Core/Include

C_SOURCES := \
  Core/Src/main.c \
  Core/Src/fdcan.c \
  Core/Src/stm32h7xx_it.c \
  Core/Src/stm32h7xx_hal_msp.c \
  Core/Src/system_stm32h7xx.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c

ASM_SOURCES := Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h723xx.s
OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
DEPS := $(OBJECTS:.o=.d)
OBJECT_DIRS := $(sort $(dir $(OBJECTS)))

CFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES) -Og -g3 -DDEBUG -std=gnu11 \
          -ffunction-sections -fdata-sections -fstack-usage \
          -Wall -Wextra -Wshadow -MMD -MP
ASFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES) -x assembler-with-cpp -MMD -MP
LDSCRIPT := STM32H723VGTx_FLASH.ld
LDFLAGS := $(CPU_FLAGS) -specs=nano.specs -specs=nosys.specs \
           -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections -Wl,--print-memory-usage
LDLIBS := -Wl,--start-group -lc -lm -lnosys -Wl,--end-group

.PHONY: all clean size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/:
	@$(call MKDIR_P,$@)

$(OBJECT_DIRS):
	@$(call MKDIR_P,$@)

$(BUILD_DIR)/%.o: %.c | $(OBJECT_DIRS)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/%.o: %.s | $(OBJECT_DIRS)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) $<

clean:
	@$(call REMOVE_DIR,build)

-include $(DEPS)
