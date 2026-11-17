"""Start/read a zero-motion 60 s statistic run; never writes motor-run tokens."""
import argparse,json,math,struct
from pathlib import Path
import operate as o
KEYS=('request','ack','state','started','ended','count','invalid_count','inhibit_or','max_step_ms','step_gaps','max_motor_age_ms','max_imu_age_ms','max_remote_age_ms','tx_fail_start','tx_fail_end','rx_overflow_start','rx_overflow_end','drdy_start','drdy_end','imu_missed_start','imu_missed_end')
VALUES=[f'q{i}' for i in range(6)]+[f'qd{i}' for i in range(6)]+['gyro_x','gyro_y','gyro_z','gravity_x','gravity_y','gravity_z','yaw','accel_norm']
p=argparse.ArgumentParser();p.add_argument('operation',choices=['start','read']);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
out=a.out.resolve()
if out.exists():p.error('Fresh output directory required')
sy=o.m.elf_symbols(o.ELF,('g_static_measurement','g_remote','g_chassis','g_supported_session'))
o.m.dump_ram(out,sy,None,o.m.DEFAULT_OPENOCD,o.ELF)
def decode(raw):
    ints=struct.unpack_from('<21I',raw);d=dict(zip(KEYS,ints));floats=struct.unpack_from('<80f',raw,84)
    d['statistics']={name:{'mean':floats[i],'std':math.sqrt(max(0,floats[20+i]/max(1,d['count']-1))),'min':floats[40+i],'max':floats[60+i]} for i,name in enumerate(VALUES)}
    d['duration_ms']=(d['ended']-d['started'])&0xffffffff if d['state'] in (2,3) else None
    d['tx_fail_delta']=(d['tx_fail_end']-d['tx_fail_start'])&0xffffffff;d['rx_overflow_delta']=(d['rx_overflow_end']-d['rx_overflow_start'])&0xffffffff
    d['drdy_edges_delta']=(d['drdy_end']-d['drdy_start'])&0xffffffff;d['imu_missed_delta']=(d['imu_missed_end']-d['imu_missed_start'])&0xffffffff
    d['raw_frame_interval_ms']=120;d['statistics_expected_interval_ms']=2
    return d
d=decode((out/'a_g_static_measurement.bin').read_bytes())
if a.operation=='start':
    r=(out/'a_g_remote.bin').read_bytes();c=struct.unpack('<10I',(out/'a_g_chassis.bin').read_bytes());s=struct.unpack('<8I2f',(out/'a_g_supported_session.bin').read_bytes())
    assert r[10]==2 and r[24] and c[8]==0 and not s[4] and s[0]==s[1] and d['state']!=1
    token=(d['ack']+1)&0xffffffff;addr=sy['g_static_measurement'][0]
    o.run(out,[f'mww {addr:#x} {token}','sleep 30',f'dump_image {{{(out/"after_start.bin").as_posix()}}} {addr:#x} {sy["g_static_measurement"][1]}'],30)
    d=decode((out/'after_start.bin').read_bytes());assert d['ack']==token and d['state']==1
o.save(out/'result.json',d)
print(json.dumps({k:v for k,v in d.items() if k!='statistics'}))
