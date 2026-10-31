.DEFAULT_GOAL := all
TARGET := chassis_torque_base
TEST_MOTOR ?= 0
ifneq ($(filter $(TEST_MOTOR),0 1),$(TEST_MOTOR))
$(error TEST_MOTOR must be 0 or 1)
endif
PROFILE := $(if $(filter 1,$(TEST_MOTOR)),bench,default)
BUILD_DIR ?= build/$(PROFILE)
TEST_BUILD_DIR := build/host_tests
# 台架路由和角度统一在 application/bench_config.h 修改。
ifneq ($(strip $(BENCH_MOTOR)$(BENCH_CAN_BUS)$(BENCH_CAN_ID)$(BENCH_RX_ID)$(BENCH_TARGET_DEG)),)
$(error Edit application/bench_config.h instead of passing BENCH_* make variables)
endif

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
MKDIR_P = powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File tools/build_helper.ps1 mkdir "$(1)"
REMOVE_DIR = powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File tools/build_helper.ps1 clean "$(1)"
else
HOST_EXEEXT :=
MKDIR_P = mkdir -p "$(1)"
REMOVE_DIR = rm -rf "$(1)"
endif

TEST_BINARY := $(TEST_BUILD_DIR)/test_chassis_logic$(HOST_EXEEXT)

DEBUG = 1
OPT ?= -Og
EXTRA_CFLAGS ?=

CPU_FLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard
DEFINES := -DUSE_HAL_DRIVER -DSTM32H723xx -DUSE_PWR_LDO_SUPPLY \
           -DARM_MATH_CM7 -D__FPU_PRESENT=1U

INCLUDES := \
  -ICore/Inc \
  -ITask \
  -Iapplication \
  -Ialgorithm \
  -Iboards \
  -Itests \
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
  application/chassis_motor_config.c \
  application/chassis_protocol.c \
  application/chassis_safety.c \
  application/USART7_Receive.c \
  algorithm/PID.c \
  algorithm/kalman_filter.c \
  algorithm/leg_position.c \
  boards/BMI088driver.c \
  boards/BMI088Middleware.c \
  boards/bsp_delay.c \
  boards/board_indicator.c \
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
C_SOURCES += tests/bench_motor_test.c tests/bench_controller.c
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

ROUTE_TESTS := $(foreach n,0 1 2 3 4 5 joint wheel,$(TEST_BUILD_DIR)/route_$(n)$(HOST_EXEEXT))
.PHONY: all clean size test bench FORCE
CONFIG_STAMP := $(BUILD_DIR)/.build-config
CONFIG_TEXT = $(CC) $(CFLAGS) $(ASFLAGS) $(LDFLAGS) $(LDLIBS)

FORCE:

$(CONFIG_STAMP): FORCE | $(BUILD_DIR)/
ifeq ($(OS),Windows_NT)
	@powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File tools/build_helper.ps1 stamp "$@" "$(CONFIG_TEXT)"
else
	@printf '%s' '$(subst ','"'"',$(CONFIG_TEXT))' > "$@.tmp"
	@cmp -s "$@.tmp" "$@" && rm "$@.tmp" || mv "$@.tmp" "$@"
endif

$(BUILD_DIR)/:
	@$(call MKDIR_P,$@)

bench:
	$(MAKE) TEST_MOTOR=1 all

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) $(LDSCRIPT) Makefile $(CONFIG_STAMP)
	$(CC) $(OBJECTS) $(LDFLAGS) $(LDLIBS) -o $@

$(BUILD_DIR)/$(TARGET).hex: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/$(TARGET).bin: $(BUILD_DIR)/$(TARGET).elf
	$(OBJCOPY) -O binary -S $< $@

$(OBJECTS): | $(OBJECT_DIRS)

$(OBJECT_DIRS):
	@$(call MKDIR_P,$@)

$(BUILD_DIR)/%.o: %.c Makefile $(CONFIG_STAMP)
	$(CC) -c $(CFLAGS) $< -o $@

# CMSIS-RTOS v1 为关闭的可选功能保留了空参数；只对这份第三方适配层抑制告警。
$(BUILD_DIR)/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.o: CFLAGS += -Wno-unused-parameter

$(BUILD_DIR)/%.o: %.s Makefile $(CONFIG_STAMP)
	$(AS) -c $(ASFLAGS) $< -o $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) $<

test: $(TEST_BUILD_DIR)/output_bench$(HOST_EXEEXT) $(TEST_BUILD_DIR)/output_default$(HOST_EXEEXT) $(TEST_BUILD_DIR)/test_board_indicator$(HOST_EXEEXT) $(TEST_BINARY) $(TEST_BUILD_DIR)/test_bench_controller$(HOST_EXEEXT) $(ROUTE_TESTS) $(TEST_BUILD_DIR)/test_remote_receive$(HOST_EXEEXT)
	$(TEST_BINARY)
	$(TEST_BUILD_DIR)/output_bench$(HOST_EXEEXT)
	$(TEST_BUILD_DIR)/output_default$(HOST_EXEEXT)
	$(TEST_BUILD_DIR)/test_board_indicator$(HOST_EXEEXT)
	$(TEST_BUILD_DIR)/test_remote_receive$(HOST_EXEEXT)
	$(TEST_BUILD_DIR)/test_bench_controller$(HOST_EXEEXT)
	$(foreach test,$(ROUTE_TESTS),"$(test)" && ) echo motor routing tests passed

$(TEST_BINARY): tests/test_chassis_logic.c \
                application/chassis_protocol.c \
                application/chassis_safety.c application/chassis_motor_config.c | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Iapplication $(filter %.c,$^) -lm -o $@

$(TEST_BUILD_DIR)/:
	@$(call MKDIR_P,$@)

clean:
	@$(call REMOVE_DIR,build)

$(TEST_BINARY): $(wildcard application/*.h)

ROUTE_SOURCES := tests/test_motor_routing.c application/chassis_protocol.c application/chassis_motor_config.c
define ROUTE_TEST_RULE
$(TEST_BUILD_DIR)/route_$(1)$(HOST_EXEEXT): $(ROUTE_SOURCES) $(wildcard application/*.h) Makefile | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Iapplication -DTEST_MOTOR_ENABLE=1 -DBENCH_MOTOR_INDEX=$(2) $(3) $(ROUTE_SOURCES) -lm -o $$@
endef
$(foreach n,0 1 2 3 4 5,$(eval $(call ROUTE_TEST_RULE,$(n),$(n),-DBENCH_CAN_BUS=0 -DBENCH_CAN_ID=0 -DBENCH_RX_ID=-1)))
$(eval $(call ROUTE_TEST_RULE,joint,0,-DBENCH_CAN_BUS=2 -DBENCH_CAN_ID=5 -DBENCH_RX_ID=0x15 -DTEST_CUSTOM_JOINT))
$(eval $(call ROUTE_TEST_RULE,wheel,4,-DBENCH_CAN_BUS=2 -DBENCH_CAN_ID=0x200 -DBENCH_RX_ID=0x204 -DTEST_CUSTOM_WHEEL))

$(TEST_BUILD_DIR)/test_bench_controller$(HOST_EXEEXT): tests/test_bench_controller.c tests/bench_controller.c $(wildcard tests/*.h) $(wildcard application/*.h) Makefile | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Iapplication -Itests $(filter %.c,$^) -lm -o $@

$(TEST_BUILD_DIR)/test_remote_receive$(HOST_EXEEXT): tests/test_remote_receive.c application/remote_control.c application/chassis_protocol.c application/chassis_motor_config.c $(wildcard application/*.h) $(wildcard tests/remote_stubs/*.h) Makefile | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Itests/remote_stubs -Iapplication $(filter %.c,$^) -lm -o $@

$(TEST_BUILD_DIR)/test_board_indicator$(HOST_EXEEXT): tests/test_board_indicator.c boards/board_indicator.c boards/board_indicator.h $(wildcard application/*.h) tests/indicator_stubs/main.h Makefile | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Itests/indicator_stubs -Iboards -Iapplication $(filter %.c,$^) -o $@

-include $(DEPS)

OUTPUT_TEST_SOURCES := tests/test_chassis_output.c application/chassis_safety.c application/chassis_protocol.c application/chassis_motor_config.c
define OUTPUT_TEST_RULE
$(TEST_BUILD_DIR)/output_$(1)$(HOST_EXEEXT): $(OUTPUT_TEST_SOURCES) Task/chassis_task.c Task/chassis_task.h $(wildcard application/*.h) $(wildcard tests/chassis_stubs/*.h) Makefile | $(TEST_BUILD_DIR)/
	$(HOST_CC) -std=c11 -Wall -Wextra -Werror -Itests/chassis_stubs -Iapplication -DTEST_MOTOR_ENABLE=$(2) $(OUTPUT_TEST_SOURCES) -lm -o $$@
endef
$(eval $(call OUTPUT_TEST_RULE,bench,1))
$(eval $(call OUTPUT_TEST_RULE,default,0))
