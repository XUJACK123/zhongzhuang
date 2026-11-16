"""Same supported pose, three MCU-only reset trials. Not power-cycle calibration.

Guard every reset with live STOP, fresh S1-down and zero output counters.
No calibration or motor parameter writes, flash, motor enable, or motion.
"""
from pathlib import Path
import importlib.util
import json
import struct
import math

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('collect',ROOT/'real_data_collection/collect_real_data.py')
c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)
c.OUT=Path((ROOT/'real_data_collection/latest_full_session.txt').read_text().strip())
c.RAW=c.OUT/'raw'
tick_addr=c.monitor.elf_symbols(c.ELF,('uwTick',))['uwTick'][0]


def main():
    trials=[]
    for i in range(1,4):
        tag=f'T2_soft_reset_{i}'
        if (c.OUT/(tag+'_snapshots.json')).exists():
            raise SystemExit('Refuse to overwrite a completed trial')
        commands=[c.dump(tag+'_guard','g_remote'),
                  f'dump_image {{{(c.RAW/(tag+"_guard_tick.bin")).as_posix()}}} {tick_addr:#x} 4',
                  c.dump(tag+'_guard','g_chassis'),c.dump(tag+'_guard','g_can_monitor')]
        c.run(tag+'_guard',commands)
        rc=(c.RAW/(tag+'_guard_g_remote.bin')).read_bytes()
        ch=(c.RAW/(tag+'_guard_g_chassis.bin')).read_bytes()
        cm=(c.RAW/(tag+'_guard_g_can_monitor.bin')).read_bytes()
        now=struct.unpack('<I',(c.RAW/(tag+'_guard_tick.bin')).read_bytes())[0]
        guard={'mode':struct.unpack_from('<I',ch,32)[0],'s1':rc[10],'remote_valid':bool(rc[24]),
               'age_upper_ms':(now-struct.unpack_from('<I',rc,12)[0])&0xffffffff,
               'nonzero':struct.unpack_from('<I',cm,28)[0],'enable':struct.unpack_from('<I',cm,32)[0]}
        c.save(c.OUT/(tag+'_guard.json'),guard)
        assert guard['mode']==0 and guard['s1']==2 and guard['remote_valid'] and guard['age_upper_ms']<=100
        assert guard['nonzero']==guard['enable']==0
        c.event('T2_MCU_software_reset',trial=i,mcu_tick=now,guard=guard,
                full_power_cycle=False,reference_calibration_changed=False)
        c.run(tag+'_reset',['reset run','sleep 3500','echo [stm32h723.cpu0 curstate]'])
        c.snapshots(tag,2,1)
        rows=json.loads((c.OUT/(tag+'_snapshots.json')).read_text())
        assert all(r['chassis']['mode']==0 and not r['nonzero_torque_frames'] and not r['enable_commands'] for r in rows)
        trials.append({'trial':i,'mcu_epoch':i+1,'snapshot_tags':[r['tag'] for r in rows],
                       'inhibit_values':[r['chassis']['inhibit'] for r in rows],
                       'raw_counts':[m['raw'] for m in rows[-1]['motor_bank']],
                       'arm_rad':[m['arm_rad'] for m in rows[-1]['motor_bank']],
                       'q_rad':rows[-1]['measurement']['joint_position_rad'],
                       'position_lost':[m['position_lost'] for m in rows[-1]['motor_bank']]})
        print('SOFT_RESET_TRIAL',i,trials[-1],flush=True)
    q_ranges=[max(t['q_rad'][i] for t in trials)-min(t['q_rad'][i] for t in trials) for i in range(6)]
    result={'kind':'same_pose_MCU_software_reset_repeatability_only','trials':trials,
            'q_range_rad':q_ranges,'leg_q_range_within_0p02rad':all(q_ranges[i]<=.02 for i in (0,1,3,4)),
            'physical_pose_fixture_used':False,'full_power_cycle_count':0,'encoder_zeros_modified':False,
            'full_T2_pass':False,'limitations':['No bidirectional physical approach repetitions.',
                                               'MCU reset is not full power cycling.',
                                               'Wheel absolute base can change by design after restart.']}
    c.save(c.OUT/'T2_soft_reset_result.json',result)


if __name__=='__main__':main()
