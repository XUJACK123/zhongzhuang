# No halt, resume, reset or verify command: read live diagnostic RAM only.
init
poll
echo ===BEFORE===
echo DIAG_HEADER
echo [stm32h723.cpu0 read_memory 0x24000000 32 29]
echo TICK
echo [stm32h723.cpu0 read_memory 0x240005ac 32 1]
echo ERROR
echo [stm32h723.cpu0 read_memory 0x24000508 32 1]
sleep 5000
echo ===AFTER===
echo DIAG_HEADER
echo [stm32h723.cpu0 read_memory 0x24000000 32 29]
echo TICK
echo [stm32h723.cpu0 read_memory 0x240005ac 32 1]
echo ERROR
echo [stm32h723.cpu0 read_memory 0x24000508 32 1]
echo ===ALL_ID_BITMAP===
echo [stm32h723.cpu0 read_memory 0x24000074 32 64]
echo ===FRAME_RING===
echo [stm32h723.cpu0 read_memory 0x24000178 32 28]
shutdown
