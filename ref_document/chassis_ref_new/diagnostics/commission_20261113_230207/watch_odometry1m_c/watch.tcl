set dir {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/diagnostics/commission_20261113_230207/watch_odometry1m_c}
set n 120
for {set k 0} {$k < $n} {incr k} {
    if {$k > 0} { sleep 500 }
    dump_image [format "%s/%03d_g_imu.bin" $dir $k] 0x20000360 128
    dump_image [format "%s/%03d_g_measurement.bin" $dir $k] 0x20000638 92
    dump_image [format "%s/%03d_g_motor_snapshot.bin" $dir $k] 0x20000540 248
}
shutdown
