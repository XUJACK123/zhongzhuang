# Read-only snapshot for the current ELF. No reset, halt, resume or programming.
init
echo ===CPU_STATE===
poll
echo ===RAM_SNAPSHOT_1===
echo g_can_bus_off
echo [stm32h723.cpu0 read_memory 0x24000018 32 1]
echo uwTick
echo [stm32h723.cpu0 read_memory 0x240000d4 32 1]
echo g_c620_current
echo [stm32h723.cpu0 read_memory 0x24000000 32 1]
echo g_can_tx_queued
echo [stm32h723.cpu0 read_memory 0x24000020 32 1]
echo g_can_tx_failed
echo [stm32h723.cpu0 read_memory 0x2400001c 32 1]
echo g_c620_feedback
echo [stm32h723.cpu0 read_memory 0x240000c4 32 4]
echo g_error
echo [stm32h723.cpu0 read_memory 0x24000014 32 1]
echo ===FDCAN2_CCCR_NBTP===
echo [stm32h723.cpu0 read_memory 0x4000a418 32 2]
echo ===FDCAN2_ECR_PSR===
echo [stm32h723.cpu0 read_memory 0x4000a440 32 2]
echo ===FDCAN2_TX_STATUS===
echo [stm32h723.cpu0 read_memory 0x4000a4c0 32 10]
echo ===GPIOB_MODE===
echo [stm32h723.cpu0 read_memory 0x58020400 32 1]
echo ===GPIOB_AFR===
echo [stm32h723.cpu0 read_memory 0x58020420 32 2]
echo ===SCB_FAULTS===
echo [stm32h723.cpu0 read_memory 0xe000ed28 32 7]
sleep 1000
echo ===RAM_SNAPSHOT_2===
echo g_can_bus_off
echo [stm32h723.cpu0 read_memory 0x24000018 32 1]
echo uwTick
echo [stm32h723.cpu0 read_memory 0x240000d4 32 1]
echo g_c620_current
echo [stm32h723.cpu0 read_memory 0x24000000 32 1]
echo g_can_tx_queued
echo [stm32h723.cpu0 read_memory 0x24000020 32 1]
echo g_can_tx_failed
echo [stm32h723.cpu0 read_memory 0x2400001c 32 1]
echo g_c620_feedback
echo [stm32h723.cpu0 read_memory 0x240000c4 32 4]
echo g_error
echo [stm32h723.cpu0 read_memory 0x24000014 32 1]
echo ===FLASH_COMPARE===
if {[catch {echo [verify_image build/c620_m3508.bin 0x08000000 bin]} err]} {echo $err}
shutdown
