source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/vectors_post_reset.bin} 0x08000000 256
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/full_tests_20261116_165832/raw/post_reset_check_g_chassis.bin} 0x20001028 40
echo [stm32h723.cpu0 curstate]
shutdown
