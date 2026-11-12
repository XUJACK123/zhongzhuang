# GNU Make + GNU Arm Embedded. Application/clock/CAN settings stay unchanged.
# Disable built-in .S -> .s transformations on case-insensitive Windows paths.
MAKEFLAGS += --no-builtin-rules
.SUFFIXES:
.DEFAULT_GOAL := all
TARGET := dm8009_step
BUILD_DIR ?= build
# Default to a debuggable build even if the parent environment sets DEBUG=release.
DEBUG = 1
OPT ?= -Og

# Same GCC_PATH override convention as chassis_of_hero.
# Default paths match the tools installed on this computer.
USER_ROOT := $(subst \,/,$(USERPROFILE))
GCC_PATH ?= $(USER_ROOT)/.eide/tools/gcc_arm/bin
OPENOCD_ROOT ?= $(USER_ROOT)/.eide/tools/openocd_7a1adfbec_mingw32
OPENOCD_SCRIPTS ?= $(OPENOCD_ROOT)/share/openocd/scripts
PREFIX := arm-none-eabi-
CC := "$(GCC_PATH)/$(PREFIX)gcc.exe"
CP := "$(GCC_PATH)/$(PREFIX)objcopy.exe"
SZ := "$(GCC_PATH)/$(PREFIX)size.exe"
OPENOCD := "$(OPENOCD_ROOT)/bin/openocd.exe"
POWERSHELL ?= powershell.exe

ifeq ($(DIRECT_TEST),1)
TARGET := dm8009_direct
BUILD_DIR := build_dm8009_direct
APP_DEFS := -DDM_DIRECT_TEST=1
ifeq ($(LOAD_TEST),1)
TARGET := dm8009_loaded_direct
BUILD_DIR := build_dm8009_loaded_direct
APP_DEFS += -DDM_STEP_TORQUE_LIMIT_NM=1.50f -DDM_STEP_TORQUE_SLEW_NM_S=0.20f -DDM_STEP_TIMEOUT_MS=20000U
else ifeq ($(LOAD_TEST),2)
TARGET := dm8009_loaded3_direct
BUILD_DIR := build_dm8009_loaded3_direct
APP_DEFS += -DDM_STEP_TORQUE_LIMIT_NM=3.00f -DDM_STEP_TORQUE_SLEW_NM_S=0.20f -DDM_STEP_SPEED_KP=0.20f -DDM_STEP_SPEED_KI=0.10f -DDM_STEP_TIMEOUT_MS=30000U
else ifeq ($(LOAD_TEST),3)
TARGET := dm8009_loaded5_direct
BUILD_DIR := build_dm8009_loaded5_direct
APP_DEFS += -DDM_STEP_TORQUE_LIMIT_NM=5.00f -DDM_STEP_TORQUE_SLEW_NM_S=0.20f -DDM_STEP_SPEED_DEG_S=2.0f -DDM_STEP_SPEED_KP=0.30f -DDM_STEP_SPEED_KI=0.10f -DDM_STEP_TIMEOUT_MS=60000U
else ifeq ($(LOAD_TEST),4)
TARGET := dm8009_loaded8_direct
BUILD_DIR := build_dm8009_loaded8_direct
APP_DEFS += -DDM_STEP_TORQUE_LIMIT_NM=8.00f -DDM_STEP_TORQUE_SLEW_NM_S=0.30f -DDM_STEP_SPEED_DEG_S=2.0f -DDM_STEP_SPEED_KP=0.30f -DDM_STEP_SPEED_KI=0.10f -DDM_STEP_TIMEOUT_MS=75000U
else ifeq ($(LOAD_TEST),5)
TARGET := dm8009_loaded12_direct
BUILD_DIR := build_dm8009_loaded12_direct
APP_DEFS += -DDM_STEP_TORQUE_LIMIT_NM=12.00f -DDM_STEP_TORQUE_SLEW_NM_S=0.40f -DDM_STEP_SPEED_DEG_S=2.0f -DDM_STEP_SPEED_KP=0.30f -DDM_STEP_SPEED_KI=0.20f -DDM_STEP_OVERSPEED_DEG_S=20.0f -DDM_STEP_TIMEOUT_MS=90000U
else ifeq ($(LOAD_TEST),6)
TARGET := dm8009_hold_direct
BUILD_DIR := build_dm8009_hold_direct
APP_DEFS += -DDM_STEP_HOLD_AT_TARGET=1 -DDM_STEP_TORQUE_LIMIT_NM=12.00f -DDM_STEP_TORQUE_SLEW_NM_S=0.40f -DDM_STEP_SPEED_DEG_S=2.0f -DDM_STEP_SPEED_KP=0.30f -DDM_STEP_SPEED_KI=0.20f -DDM_STEP_OVERSPEED_DEG_S=20.0f -DDM_STEP_TIMEOUT_MS=90000U
endif
ifneq ($(filter 1 2 3 4 5 6,$(LOAD_TEST)),)
APP_DEFS += -DDM_STEP_DEGREES=10.0f -DDM_STEP_TRAVEL_LIMIT_DEG=12.0f -DDM_DIRECT_START_SPEED_THRESHOLD_SCALE=1.0f
ifneq ($(filter 1 2,$(LOAD_TEST)),)
APP_DEFS += -DDM_STEP_SPEED_DEG_S=5.0f -DDM_STEP_OVERSPEED_DEG_S=10.0f
else ifneq ($(filter 3 4,$(LOAD_TEST)),)
APP_DEFS += -DDM_STEP_OVERSPEED_DEG_S=10.0f
endif
endif
C_SOURCES := Core/Src/main.c Core/Src/board.c Core/Src/board_led.c Core/Src/dbus_remote.c Core/Src/dm8009_app.c Core/Src/dm8009_step.c Core/Src/dm8009_direct.c Core/Src/system_stm32h7xx.c
else ifeq ($(ID_SCAN),1)
TARGET := dm8009_idscan
BUILD_DIR := build_dm8009_idscan
APP_DEFS := -DDM_ID_SCAN_ONLY=1 -DDM_FIXED_CAN_ID=0
C_SOURCES := Core/Src/main.c Core/Src/board.c Core/Src/board_led.c Core/Src/dbus_remote.c Core/Src/dm8009_app.c Core/Src/dm8009_step.c Core/Src/system_stm32h7xx.c
else ifeq ($(DIAGNOSTIC),1)
TARGET := can_diagnostic
BUILD_DIR := build_can_diagnostic
C_SOURCES := diagnostics/can_passive.c Core/Src/board.c Core/Src/system_stm32h7xx.c
else ifeq ($(DM8009),1)
TARGET := dm8009_diag
BUILD_DIR := build_dm8009_diag
C_SOURCES := diagnostics/dm8009_main.c Core/Src/board.c Core/Src/system_stm32h7xx.c
else
APP_DEFS := -DDM_STEP_HOLD_AT_TARGET=1 -DDM_STEP_FIXED_TARGET=1 -DDM_STEP_TORQUE_LIMIT_NM=28.00f -DDM_STEP_TORQUE_SLEW_NM_S=0.80f -DDM_STEP_SPEED_DEG_S=4.0f -DDM_STEP_SPEED_KP=0.30f -DDM_STEP_SPEED_KI=0.20f -DDM_STEP_OVERSPEED_DEG_S=40.0f -DDM_STEP_TIMEOUT_MS=90000U
C_SOURCES := Core/Src/main.c Core/Src/board.c Core/Src/board_led.c Core/Src/dbus_remote.c Core/Src/dm8009_app.c Core/Src/dm8009_step.c Core/Src/system_stm32h7xx.c
endif
HAL_MODULES := hal hal_cortex hal_fdcan hal_flash hal_flash_ex hal_gpio hal_pwr hal_pwr_ex hal_rcc hal_rcc_ex hal_spi hal_dma
C_SOURCES += $(addprefix Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_,$(addsuffix .c,$(HAL_MODULES)))
ASM_SOURCES := startup_stm32h723xx.s

MCU := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
C_DEFS := -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY -DHSE_VALUE=24000000UL $(APP_DEFS)
C_INCLUDES := -ICore/Inc -IDrivers/STM32H7xx_HAL_Driver/Inc -IDrivers/CMSIS/Include -IDrivers/CMSIS/Device/ST/STM32H7xx/Include
COMMON_FLAGS := $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -ffunction-sections -fdata-sections
ifeq ($(DEBUG),1)
COMMON_FLAGS += -g3
endif
CFLAGS := $(COMMON_FLAGS) -std=c11 -Wall -Wextra -MMD -MP
ASFLAGS := $(COMMON_FLAGS) -x assembler-with-cpp -MMD -MP
LDSCRIPT := STM32H723VG_FLASH.ld
LDFLAGS := $(MCU) -T$(LDSCRIPT) -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref -Wl,--print-memory-usage

OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

.PHONY: all clean download download_stlink download_dap download_jlink
all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	@echo CC $<
	@$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo AS $<
	@$(CC) $(ASFLAGS) -c $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(CP) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(CP) -O binary $< $@

$(BUILD_DIR):
	$(POWERSHELL) -NoProfile -Command "New-Item -ItemType Directory -Force -Path '$(BUILD_DIR)' | Out-Null"

# Guard the resolved path inside this project before deleting build outputs.
clean:
	$(POWERSHELL) -NoProfile -ExecutionPolicy Bypass -File tools/clean.ps1 -BuildDir "$(BUILD_DIR)"

# Each download waits for a successful build, programs/verifies Flash, then resets/runs.
# These targets operate hardware only when explicitly invoked by the user.
download: download_stlink

download_stlink: all
	$(OPENOCD) -s "$(OPENOCD_SCRIPTS)" -f openocd_stlink.cfg -c "program {$(BUILD_DIR)/$(TARGET).elf} verify reset exit"

download_dap: all
	$(OPENOCD) -s "$(OPENOCD_SCRIPTS)" -f openocd_dap.cfg -c "program {$(BUILD_DIR)/$(TARGET).elf} verify reset exit"

download_jlink: all
	$(OPENOCD) -s "$(OPENOCD_SCRIPTS)" -f openocd_jlink.cfg -c "program {$(BUILD_DIR)/$(TARGET).elf} verify reset exit"

-include $(OBJECTS:.o=.d)
