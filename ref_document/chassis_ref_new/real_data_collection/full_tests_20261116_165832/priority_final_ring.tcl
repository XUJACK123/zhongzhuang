source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
if {[string equal [stm32h723.cpu0 curstate] halted]} {resume}
sleep 2000
echo RING_HALT_BEGIN
set ring_error [catch { halt; wait_halt 1000; dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/priority_final_g_telemetry.bin} 0x20001084 20376 } ring_message]
echo RING_RESUME
if {[string equal [stm32h723.cpu0 curstate] halted]} {resume}
echo [stm32h723.cpu0 curstate]
if {$ring_error} {error $ring_message}
shutdown
