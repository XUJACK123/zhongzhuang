source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
set r [read_memory 0x20000220 8 28]
set phase [lindex [read_memory 0x2001447c 32 1] 0]
if {[lindex $r 24] != 1 || [lindex $r 10] == 2 || ($phase != 0 && $phase != 4)} {error "Physical run permit/locked-state check failed"}
if {![string equal [stm32h723.cpu0 curstate] running]} {error "CPU is not running"}
mww 0x200144b0 0x26111602
shutdown
