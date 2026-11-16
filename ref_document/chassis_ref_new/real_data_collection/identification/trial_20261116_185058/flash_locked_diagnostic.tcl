source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
set r [read_memory 0x20000220 8 28]
set ch [read_memory 0x2001447c 32 10]
if {[lindex $r 10] != 2 || [lindex $r 24] != 1 || [lindex $ch 0] != 4} {error "DONE/S1-down precondition failed; no flash"}
reset halt
wait_halt 2000
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_185058/raw/preflash_previous.bin} 0x08000000 37816
program {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/build/identification/limited_current_identification.elf} verify reset
shutdown
