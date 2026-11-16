source [find openocd_dap.cfg]
stm32h723.cpu0 configure -event examine-start {stm32h7x_dbgmcu_mmw 0x004 0x00600007 0}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
targets stm32h723.cpu0
mww 0x200144b4 0x26111601
sleep 50
dump_image {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/real_data_collection/identification/trial_20261116_185058/raw/armed_g_ident.bin} 0x2001447c 48
echo CPU_STATE_BEFORE_DISCONNECT
echo [stm32h723.cpu0 curstate]
shutdown
