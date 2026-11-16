source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/vectors.bin} 0x8000000 256
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/code_mid.bin} 0x8004c04 256
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/field_measurements_20261116_205507/raw/code_end.bin} 0x8009708 256
echo CPU_STATE_[stm32h723.cpu0 curstate]
shutdown
