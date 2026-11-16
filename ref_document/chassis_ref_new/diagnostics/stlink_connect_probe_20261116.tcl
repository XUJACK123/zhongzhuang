source [find interface/stlink-dap.cfg]
transport select dapdirect_swd
set CHIPNAME stm32h723
set DUAL_CORE 0
set DUAL_BANK 0
source [find target/stm32h7x.cfg]
reset_config none
adapter speed 100
# Prepare the same debug clocks used by the standard examine-end hook,
# before reading CPUID. No reset, halt, resume or flash commands.
stm32h723.cpu0 configure -event examine-start {
    stm32h7x_dbgmcu_mmw 0x004 0x00600007 0
}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
for {set n 1} {$n <= 2} {incr n} {
    echo "READ_SAMPLE_$n"
    echo [stm32h723.cpu0 curstate]
    if {[catch {mdw 0xE000ED00 1} value]} {echo "CPUID_READ_FAILED: $value"} else {echo $value}
    if {[catch {mdw 0xE000EDF0 1} value]} {echo "DHCSR_READ_FAILED: $value"} else {echo $value}
    if {[catch {mdw 0x08000000 8} value]} {echo "FLASH_READ_FAILED: $value"} else {echo $value}
    if {[catch {dump_image diagnostics/connection_stlink_stable_${n}_20261116.bin 0x08000000 256} value]} {echo "DUMP_FAILED: $value"} else {echo $value}
    sleep 200
}
targets stm32h723.ap2
mdw 0xE00E1000 2
shutdown
