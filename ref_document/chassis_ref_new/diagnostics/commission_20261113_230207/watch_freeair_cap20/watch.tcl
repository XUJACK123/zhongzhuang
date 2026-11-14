set dir {D:/robomaster_material/electric_contol/zhongzhuang/ref_document/chassis_ref_new/diagnostics/commission_20261113_230207/watch_freeair_cap20}
set n 150
for {set k 0} {$k < $n} {incr k} {
    if {$k > 0} { sleep 200 }
    dump_image [format "%s/%03d_g_chassis.bin" $dir $k] 0x20000b20 40
    dump_image [format "%s/%03d_g_imu.bin" $dir $k] 0x20000360 128
    dump_image [format "%s/%03d_g_measurement.bin" $dir $k] 0x20000638 92
    dump_image [format "%s/%03d_g_motor_snapshot.bin" $dir $k] 0x20000540 248
    dump_image [format "%s/%03d_g_rl.bin" $dir $k] 0x20000694 1164
}
shutdown
