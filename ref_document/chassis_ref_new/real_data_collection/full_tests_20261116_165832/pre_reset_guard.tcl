source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/pre_remote_before_tick.bin} 0x20006064 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/pre_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/pre_remote_after_tick.bin} 0x20006064 4
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/pre_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/pre_g_can_monitor.bin} 0x20000000 160
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/pre_g_motor_snapshot.bin} 0x20000a48 248
echo [stm32h723.cpu0 curstate]
shutdown
