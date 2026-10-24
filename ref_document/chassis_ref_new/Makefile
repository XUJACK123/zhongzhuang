TARGET := chassis_torque_base
BUILD_DIR := build
TEST_BUILD_DIR := $(BUILD_DIR)/host_tests

TOOLCHAIN_DIR ?=
CROSS_COMPILE ?= $(TOOLCHAIN_DIR)arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
AS := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE := $(CROSS_COMPILE)size
HOST_CC ?= gcc

# GNU Make may be launched directly from PowerShell.  Keep directory handling
# independent of Unix utilities such as rm and mkdir -p.
ifeq ($(OS),Windows_NT)
HOST_EXEEXT := .exe
MKDIR_P = powershell.exe -NoProfile -NonInteractive -Command "[System.IO.Directory]::CreateDirectory('$(1)') | Out-Null"
REMOVE_DIR = powershell.exe -NoProfile -NonInteractive -Command "if (Test-Path -LiteralPath '$(1)') { Remove-Item -LiteralPath '$(1)' -Recurse -Force }"
else
HOST_EXEEXT :=
MKDIR_P = mkdir -p "$(1)"
REMOVE_DIR = rm -rf "$(1)"
endif

TEST_BINARY := $(TEST_BUILD_DIR)/test_chassis_logic$(HOST_EXEEXT)

DEBUG ?= 1
OPT ?= -Og
EXTRA_CFLAGS ?=

# 临时单电机测试：make TEST_MOTOR=1
# 启用后编译 tests/bench_motor_test.c，并在 freertos.c 中创建测试任务。
TEST_MOTOR ?= 0

CPU_FLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFINES := -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY \
           -DARM_MATH_CM7 -D__FPU_PRESENT=1U

INCLUDES := \
  -ICore/Inc \
  -ITask \
  -Iapplication \
  -Ialgorithm \
  -Iboards \
  -IDrivers/STM32H7xx_HAL_Driver/Inc \
  -IDrivers/STM32H7xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32H7xx/Include \
  -IDrivers/CMSIS/Core/Include \
  -IDrivers/CMSIS/DSP/Include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/include \
  -IMiddlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS \
  -IMiddlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1

C_SOURCES := \
  Core/Src/main.c \
  Core/Src/gpio.c \
  Core/Src/dma.c \
  Core/Src/fdcan.c \
  Core/Src/spi.c \
  Core/Src/tim.c \
  Core/Src/usart.c \
  Core/Src/freertos.c \
  Core/Src/stm32h7xx_it.c \
  Core/Src/stm32h7xx_hal_msp.c \
  Core/Src/stm32h7xx_hal_timebase_tim.c \
  Core/Src/system_stm32h7xx.c \
  Core/Src/syscalls.c \
  Task/chassis_task.c \
  Task/INS_task.c \
  Task/send_task.c \
  application/CAN_receive.c \
  application/remote_control.c \
  application/chassis_protocol.c \
  application/chassis_safety.c \
  application/USART7_Receive.c \
  algorithm/PID.c \
  algorithm/kalman_filter.c \
  algorithm/leg_position.c \
  boards/BMI088driver.c \
  boards/BMI088Middleware.c \
  boards/bsp_delay.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_spi_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_tim_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c \
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c \
  Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_init_f32.c \
  Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_add_f32.c \
  Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_sub_f32.c \
  Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_mult_f32.c \
  Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_trans_f32.c \
  Drivers/CMSIS/DSP/Source/MatrixFunctions/arm_mat_inverse_f32.c \
  Middlewares/Third_Party/FreeRTOS/Source/croutine.c \
  Middlewares/Third_Party/FreeRTOS/Source/event_groups.c \
  Middlewares/Third_Party/FreeRTOS/Source/list.c \
  Middlewares/Third_Party/FreeRTOS/Source/queue.c \
  Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c \
  Middlewares/Third_Party/FreeRTOS/Source/tasks.c \
  Middlewares/Third_Party/FreeRTOS/Source/timers.c \
  Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c \
  Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c

ASM_SOURCES := Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc/startup_stm32h723xx.s
ifeq ($(TEST_MOTOR),1)
C_SOURCES += tests/bench_motor_test.c
endif


OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
DEPS := $(OBJECTS:.o=.d)
OBJECT_DIRS := $(sort $(dir $(OBJECTS)))

CFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES) $(OPT) -std=gnu11 \
          -ffunction-sections -fdata-sections -fstack-usage \
          -Wall -Wextra -Wshadow -MMD -MP $(EXTRA_CFLAGS)
ifeq ($(DEBUG),1)
CFLAGS += -g3 -DDEBUG
endif
ifeq ($(TEST_MOTOR),1)
CFLAGS += -DTEST_MOTOR_ENABLE=1
endif

ASFLAGS := $(CPU_FLAGS) $(DEFINES) $(INCLUDES) -x assembler-with-cpp -MMD -MP
LDSCRIPT := STM32H723VGTx_FLASH.ld
LDFLAGS := $(CPU_FLAGS) -specs=nano.specs -specs=nosys.specs \
           -T$(LDSCRIPT) -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
           -Wl,--gc-sections -Wl,--print-memory-usage
LDLIBS := -Wl,--start-group -lc -lm -lnosys -Wl,--end-group

.PHONY: all clean size test

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

$(OBJECTS): | $(OBJECT_DIRS)

$(OBJECT_DIRS):
	@$(call MKDIR_P,$@)

$(BUILD_DIR)/%.o: %.c Makefile
	$(CC) -c $(CFLAGS) $< -o $@

# CMSIS-RTOS v1 为关闭的可选功能保留了空参数；只对这份第三方适配层抑制告警。
$(BUILD_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o: CFLAGS += -Wno-unused-parameter

$(BUILD_DIR)/%.o: %.s Makefile
	$(AS) -c $(ASFLAGS) $< -o $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) $<

test: $(TEST_BINARY)
	$(TEST_BINARY)

$(TEST_BINARY): tests/test_chassis_logic.c \
                application/chassis_protocol.c \
                application/chassis_safety.c | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Iapplication $^ -lm -o $@

$(TEST_BUILD_DIR)/:
	@$(call MKDIR_P,$@)

clean:
	@$(call REMOVE_DIR,$(BUILD_DIR))

-include $(DEPS)
