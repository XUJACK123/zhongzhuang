source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
reset run
sleep 3500
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/post_reset_g_chassis.bin} 0x20001028 40
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/post_reset_g_remote.bin} 0x20000220 28
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/post_reset_g_motor_diagnostics.bin} 0x2000601c 72
echo [stm32h723.cpu0 curstate]
shutdown
