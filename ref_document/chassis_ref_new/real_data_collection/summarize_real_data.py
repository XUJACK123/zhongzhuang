"""Consolidate original and repaired-link T0 trials; never accesses hardware."""
import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import statistics as stats

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'real_data_collection'
OUT=Path((BASE/'latest_session.txt').read_text(encoding='utf-8-sig').strip())
ORDER=['right_hip_DM4','right_knee_DM3','right_wheel_M3508_3','left_hip_DM1','left_knee_DM2','left_wheel_M3508_2']
IDS=['0x304','0x303','0x203','0x301','0x302','0x202']


def read(name):
    return json.loads((OUT/name).read_text(encoding='utf-8'))


def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n',encoding='utf-8')


def q(values, p):
    values=sorted(values)
    if not values:return None
    i=(len(values)-1)*p; lo=int(i); hi=min(lo+1,len(values)-1)
    return values[lo]+(values[hi]-values[lo])*(i-lo)


def describe(values):
    return {'n':len(values),'min':min(values) if values else None,'max':max(values) if values else None,
            'mean':stats.mean(values) if values else None,'sample_std':stats.stdev(values) if len(values)>1 else None,
            'p50':q(values,.5),'p95':q(values,.95),'p99':q(values,.99)}


def static_summary(rows):
    span=(rows[-1]['chassis']['tick']-rows[0]['chassis']['tick'])&0xffffffff
    imu_time=[((r['imu']['tick']-rows[0]['imu']['tick'])&0xffffffff)/1000 for r in rows]
    yaw=[math.degrees(r['imu']['yaw_rad']) for r in rows]
    tmean=stats.mean(imu_time); ymean=stats.mean(yaw)
    denom=sum((t-tmean)**2 for t in imu_time)
    slope=sum((t-tmean)*(y-ymean) for t,y in zip(imu_time,yaw))/denom if denom else None
    motors=[]
    for i in range(6):
        valid=[r['motor_bank'][i] for r in rows if r['motor_bank'][i]['motor_received']>0]
        lost=any(m['position_lost'] for m in valid)
        motors.append({'label':ORDER[i],'id':IDS[i], 'received_count_start':rows[0]['nodes'][i]['received'],
                       'received_count_end':rows[-1]['nodes'][i]['received'],
                       'raw_counts':describe([m['raw'] for m in valid]),
                       'reported_arm_rad':describe([m['arm_rad'] for m in valid]),
                       'temperature_c':describe([m['temperature_c'] for m in valid]),
                       'position_lost_observed':lost,'unwrapped_angle_valid_for_analysis':bool(valid) and not lost,
                       'external_angle_reference_rad':None})
    errors={}
    for k in ('tx_error','rx_error','bus_off','fifo_overflow','tx_failed'):
        a,b=rows[0][k],rows[-1][k]
        errors[k]={'start':a,'end':b,'delta_u32':(b-a)&0xffffffff}
    return {'snapshot_count':len(rows),'mcu_span_s':span/1000,
            'sample_period_ms':describe([((b['chassis']['tick']-a['chassis']['tick'])&0xffffffff) for a,b in zip(rows,rows[1:])]),
            'read_bracket_ms':describe([r['mcu_read_bracket']['elapsed_ms_u32'] for r in rows]),
            'not_continuous_500hz':True,'motor_summary':motors,'errors':errors,
            'mode_values':sorted(set(r['chassis']['mode'] for r in rows)),
            'inhibit_values':sorted(set(r['chassis']['inhibit'] for r in rows)),
            's1_values':sorted(set(r['remote']['s1'] for r in rows)),
            'enable_commands_max':max(r['enable_commands'] for r in rows),
            'nonzero_torque_frames_max':max(r['nonzero_torque_frames'] for r in rows),
            'control_overruns_start_end':[rows[0]['chassis']['overruns'],rows[-1]['chassis']['overruns']],
            'stored_control_max_loop_us':max(r['chassis']['max_loop_us'] for r in rows),
            'active_inference_measured':False,'inference_duration_percentiles_us':None,
            'loop_duration_percentiles_us':None,
            'loop_timing_limit':'Firmware stores a running maximum only; STOP measurements cannot establish ACTIVE single/dual-actor worst case.',
            'joint_q_and_odometry_valid':False,
            'invalid_state_reason':'inhibit has geometry bit32 and motor bit4; copied q/xy zeros are not true physical zero or a drift measurement.',
            'imu_static':{'all_calibrated':all(r['imu']['calibrated'] for r in rows),
                          'samples_start_end':[rows[0]['imu']['samples'],rows[-1]['imu']['samples']],
                          'drdy_edges_start_end':[rows[0]['imu']['drdy_edges'],rows[-1]['imu']['drdy_edges']],
                          'gyro_corrected_rad_s':[describe([r['imu']['omega_body_rad_s'][i] for r in rows]) for i in range(3)],
                          'acceleration_m_s2':[describe([r['imu']['accel_body_m_s2'][i] for r in rows]) for i in range(3)],
                          'yaw_start_end_deg':[yaw[0],yaw[-1]], 'yaw_delta_deg':yaw[-1]-yaw[0],
                          'yaw_linear_slope_deg_s':slope,'drift_uncertainty_deg_s':None,
                          'interpretation':'Supported static onboard estimate; no external angle truth, temperature or vibration control; not an error bound or completed T5.'}}


def ring_summary(report):
    rows=report['samples']
    return {'frame_count':len(rows),'rejected_slots':report['rejected_slots'],
            'tick_first_last':[rows[0]['tick'],rows[-1]['tick']],
            'tick_span_ms':(rows[-1]['tick']-rows[0]['tick'])&0xffffffff,
            'tick_gap_ms':describe(report['tick_gaps_ms']),
            'all_gaps_2ms':report['continuous_2ms_window'],
            'motor_age_ms':[{'label':ORDER[i],'id':IDS[i],'age':describe([r['motor_age_ms'][i] for r in rows]),
                             'all_within_30ms':all(r['motor_age_ms'][i]<=30 for r in rows)} for i in range(6)],
            'imu_age_ms':describe([r['imu_age_ms'] for r in rows]),
            'remote_age_ms':describe([r['remote_age_ms'] for r in rows]),
            'imu_all_within_5ms':all(r['imu_age_ms']<=5 for r in rows),
            'remote_all_within_100ms':all(r['remote_age_ms']<=100 for r in rows),
            'mode_values':sorted(set(r['mode'] for r in rows)),
            'inhibit_values':sorted(set(r['inhibit'] for r in rows)),
            'fault_values':sorted(set(r['fault'] for r in rows)),
            'unwrap_ambiguous_last':rows[-1]['unwrap_ambiguous'],
            'imu_missed_edges_first_last':[rows[0]['imu_missed_edges'],rows[-1]['imu_missed_edges']],
            'missed_edges_definition':'Sum of max(DRDY edge increment minus one,0) between SPI reads; not automatically CAN/IMU packet loss rate.',
            'control_version':hex(rows[-1]['control_version']),'model_version':rows[-1]['model_version'],
            'getup_model_version':rows[-1]['getup_model_version'],
            'external_synchronization_error_ms':None,'uncertainty':'Millisecond MCU time quantization plus unknown sensor/internal delay.',
            'capture_interference':'CPU paused during RAM export after these frames, then resumed; no continuous 60s guarantee.'}


def main():
    stages={p.name.removesuffix('_snapshots.json'):json.loads(p.read_text(encoding='utf-8')) for p in OUT.glob('*_snapshots.json')}
    rings={p.name.removesuffix('_telemetry.json'):json.loads(p.read_text(encoding='utf-8')) for p in OUT.glob('*_telemetry.json')}
    for name in ('T0','T0_retry1','static','static_after_reconnect'):
        assert name in stages, name
    assert 'bounded_retry1' in rings
    latest=stages['static_after_reconnect'][-1]
    pairs=read('T0_retry1_result.json')['pairs']
    connections_pass=all(p['all_six_increasing'] for p in pairs) and len(pairs)==3
    position_lost=[ORDER[i] for i in range(6) if any(r['motor_bank'][i]['position_lost'] for r in stages['static_after_reconnect'])]
    stage_summary={k:static_summary(v) for k,v in stages.items() if k.startswith('static')}
    ring_quality={k:ring_summary(v) for k,v in rings.items()}
    events=[json.loads(s) for s in (OUT/'events.jsonl').read_text(encoding='utf-8').splitlines() if s.strip()]
    artifacts=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.name not in ('artifact_manifest.json','result.json'):
            artifacts.append({'path':p.relative_to(BASE).as_posix(),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
    write(OUT/'artifact_manifest.json',artifacts)
    report={
        'schema':'wheel_leg_real_data_collection_v1','generated_at':dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
        'summary_zh':[
            '已使用Horco DAP完成H板实测；用户确认可靠支撑、S1下档、静止。',
            '首次三组双快照右轮0x203无反馈；现场接线检查后另做三组，六路计数均持续增长。',
            '修复前与修复后各保存一段约64秒低频静止快照，以及各32帧、相邻tick为2ms的有界遥测。',
            '六电机通信子项通过，但左轮position_lost锁定、inhibit=36，完整T0未通过；T1-T8未执行。',
            '所有采样维持STOP，使能与非零力矩计数为0；本轮未烧录、未复位、未发送主动运动命令。',
            '两个遥测导出阶段仅为读取已录制窗口而暂停MCU，随后恢复运行；调试器会扰动系统，不能当作无干扰连续记录。',
            '当前q/xy等状态无效，零值不是实际零位或里程计无漂移；未知外部测量均为null。'],
        'session':read('session.json'),'identity':read('identity.json'),
        'motor_order':ORDER,'feedback_ids':IDS,
        'status':{'six_motor_reception_pass':connections_pass,'full_T0_pass':False,
                  'remaining_position_lost':position_lost,'latest_mode':latest['chassis']['mode'],
                  'latest_inhibit':latest['chassis']['inhibit'],'inhibit_bits':{'motor':4,'geometry':32},
                  'next_test_allowed':'supported zero-output diagnostics only; no getup or active torque tests',
                  'T1_T2':'not performed: no natural release, geometry truth, repeated approach or power cycles',
                  'T3_T4':'not performed: independent load/spring instruments and fixture absent',
                  'T5':'supported static descriptive observations only; external-angle/dynamic/cold-warm validation not performed',
                  'T6_T7_T8':'not performed'},
        'three_pair_trials':{'before_reconnection':read('T0_result.json'),'after_reconnection':read('T0_retry1_result.json')},
        'static_summaries':stage_summary,'bounded_window_summaries':ring_quality,
        'snapshot_sets':stages,'bounded_windows':rings,'events':events,
        'failed_attempts':[{'stage':'initial full image and RAM read','process_timeout_s':50,'original_stdout_log_available':False,
                            'raw_firmware_preserved':'raw/board_firmware_timeout_attempt1.bin',
                            'subsequent_validation':'complete ELF loading sections match; four padding bytes differ; RAM read retried separately'},
                           {'stage':'first T0','failure':'right wheel feedback absent; retained as genuine missing-node observation'},
                           {'stage':'intermediate final check','failure':'remote was stale; valid flag is historical; excluded from fresh-remote baseline'}],
        'uncertainties':{'external_voltage_temperature_load_floor_support_force':None,'external_angle_displacement_truth':None,
                         'full_60s_sensor_age_compliance':'not established by low-frequency asynchronous snapshots',
                         'active_single_dual_actor_loop_p50_p95_p99_max':'not measured; no ACTIVE inference or getup actor',
                         'physical_CAN_delivery_proven':False,'motor_Kt_measured':False,
                         'instrumented_readout_can_cause_overflow_or_unwrap_loss':True},
        'raw_artifact_manifest':artifacts,'session_directory':OUT.relative_to(BASE).as_posix()}
    assert report['status']['six_motor_reception_pass']
    assert all(r['nonzero_torque_frames']==0 and r['enable_commands']==0 and r['chassis']['mode']==0 for rows in stages.values() for r in rows)
    target=BASE/('real_data_'+OUT.name.removeprefix('T0_')+'.json')
    write(target,report)
    result={'primary_report':target.relative_to(BASE).as_posix(),'status':report['status'],
            'static_summaries':stage_summary,'bounded_window_summaries':ring_quality,
            'snapshot_count':sum(map(len,stages.values())),'bounded_frame_count':sum(len(x['samples']) for x in rings.values()),
            'artifact_count':len(artifacts)}
    write(OUT/'result.json',result)
    update_progress(result, OUT.relative_to(ROOT.parent.parent).as_posix(), target.relative_to(ROOT.parent.parent).as_posix())
    print(json.dumps({'report':str(target),'status':report['status'],'counts':{k:result[k] for k in ['snapshot_count','bounded_frame_count','artifact_count']}},ensure_ascii=False))


def update_progress(result, relative_session, relative_report):
    p=ROOT.parent.parent/'progress.md';s=p.read_text(encoding='utf-8')
    a='<!-- REAL_DATA_COLLECTION_20261116_BEGIN -->';b='<!-- REAL_DATA_COLLECTION_20261116_END -->'
    after=result['static_summaries']['static_after_reconnect'];win=result['bounded_window_summaries']['bounded_retry1']
    clock=dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y-%m-%d %H:%M:%S')
    text=f'''{a}
## 0. 2026-11-16 实车采集进度（{clock}，本次记录已保存）

- Horco DAP识别H723/Cortex-M7；板上106704字节范围内所有ELF Flash加载段均匹配当前构建。BIN仅0x080002cc的4字节对齐填充FF/00不同，原始BIN哈希不相同。本轮没有烧录、复位、主动使能或运动。
- **六电机通信子项已通过**：首次三组右轮0x203无帧，用户现场检查接线后，新增三组双快照，六路计数全部增长；两组试验分开保留。
- 修复前21个低频静止快照覆盖63.896秒；修复后21个覆盖{after['mcu_span_s']:.3f}秒。共保存{result['snapshot_count']}组异步RAM快照、{result['bounded_frame_count']}帧有界遥测；不是连续500Hz长记录。
- 修复后32帧窗口：相邻tick全部2ms；六路反馈最大年龄{[x['age']['max'] for x in win['motor_age_ms']]}ms；IMU最大{win['imu_age_ms']['max']}ms，遥控最大{win['remote_age_ms']['max']}ms。仅支持这个短窗的年龄结论。
- **完整T0未通过**：左轮position_lost锁定，mode=STOP、inhibit=36（电机4+几何32）。q/xy的零值无效，不能当作实际零位/无漂移；不能据此解锁或进入起身。
- 全部快照enable_commands=0、nonzero_torque_frames=0。两次遥测导出为稳定读取已记录窗口而暂停CPU，随后恢复；调试器扰动和FIFO累计溢出计数如实保留。
- 总汇文件：[真实数据JSON]({relative_report})；[会话结果]({relative_session}/result.json)、[原始文件哈希清单]({relative_session}/artifact_manifest.json)。未知电压、环境温度、载荷、地面、支撑力及量具信息为null。
- 下一步：核对左轮参考位并安排受控重启清除解缠丢失，再重做完整T0。当前没有自动改标定、清标志或重启。T1自然停位、T2夹具/重启重复性、T3力矩、T4弹簧、T5动态、T6推行、T7主动旋转、T8起身均未执行。

{b}

'''
    if a in s:s=s[:s.index(a)]+text+s[s.index(b)+len(b):].lstrip('\r\n')
    else:s=s.replace('## 1. 一句话状态',text+'## 1. 一句话状态',1)
    s=s.replace('实车DAP采集进行中；最新状态见§0','实车DAP采集已保存；最新状态见§0',1)
    p.write_text(s,encoding='utf-8')


if __name__=='__main__':main()
