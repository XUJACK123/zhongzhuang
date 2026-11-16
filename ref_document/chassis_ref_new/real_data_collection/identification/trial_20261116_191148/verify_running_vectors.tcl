source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_191148/raw/diagnostic_vectors.bin} 0x08000000 256
echo [stm32h723.cpu0 curstate]
sleep 3500
shutdown
