"""Offline living report for the current on-site measurement session."""
from pathlib import Path
import datetime as dt
import hashlib
import importlib.util
import json
import math
import statistics

ROOT=Path(__file__).resolve().parents[1]
BASE=ROOT/'real_data_collection'
OUT=Path((BASE/'latest_full_session.txt').read_text().strip())
spec=importlib.util.spec_from_file_location('summary',BASE/'summarize_real_data.py')
summary=importlib.util.module_from_spec(spec);spec.loader.exec_module(summary)


def save(p,x):p.write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def analyze(rows):
    result=summary.static_summary(rows)
    valid=all(not (r['chassis']['inhibit']&(4|8|16|32|64|128)) and
              not any(m['position_lost'] for m in r['motor_bank']) for r in rows)
    result['joint_q_and_odometry_valid']=valid
    result['runtime_estimator_valid']=valid
    result['external_odometry_accuracy_verified']=False
    result['invalid_state_reason']=None if valid else 'runtime inhibits or position loss present'
    xy0=rows[0]['measurement']['root_xy_m']
    result['estimated_xy_start_end_m']=[xy0,rows[-1]['measurement']['root_xy_m']]
    result['estimated_xy_endpoint_change_m']=math.dist(xy0,rows[-1]['measurement']['root_xy_m']) if valid else None
    result['estimated_xy_max_sampled_excursion_m']=max(math.dist(xy0,r['measurement']['root_xy_m']) for r in rows) if valid else None
    result['joint_position_rad']=[summary.describe([r['measurement']['joint_position_rad'][i] for r in rows]) for i in range(6)]
    result['position_lost_any']=[any(r['motor_bank'][i]['position_lost'] for r in rows) for i in range(6)]
    result['imu_total_tilt_deg']=summary.describe([math.degrees(math.acos(max(-1,min(1,-r['imu']['gravity_body'][2])))) for r in rows])
    result['acceleration_norm_m_s2']=summary.describe([math.sqrt(sum(v*v for v in r['imu']['accel_body_m_s2'])) for r in rows])
    return result


def main():
    datasets={p.name.removesuffix('_snapshots.json'):json.loads(p.read_text(encoding='utf-8')) for p in OUT.glob('*_snapshots.json')}
    analyses={name:analyze(rows) for name,rows in datasets.items()}
    rings={p.name.removesuffix('_telemetry.json'):json.loads(p.read_text(encoding='utf-8')) for p in OUT.glob('*_telemetry.json')}
    ring_summaries={name:summary.ring_summary(data) for name,data in rings.items()}
    manual=json.loads((OUT/'manual_measurements.json').read_text(encoding='utf-8'))
    session=json.loads((OUT/'session.json').read_text(encoding='utf-8'))
    # MCU tick resets at each restart; use host capture-file order across epochs.
    latest_key=max(datasets,key=lambda k:(OUT/(k+'_snapshots.json')).stat().st_mtime_ns)
    last=datasets[latest_key][-1]
    imu=last['imu'];tilt=math.degrees(math.acos(max(-1,min(1,-imu['gravity_body'][2]))))
    fk=[]
    q=last['measurement']['joint_position_rad']
    for side,h,s in [('right',0,1),('left',3,-1)]:
        a=s*q[h];b=s*(q[h]+q[h+1])+math.pi/2
        x=.21*math.cos(a)+.25*math.cos(b);z=-.21*math.sin(a)-.25*math.sin(b)
        fk.append({'side':side,'firmware_fk_x_m':x,'firmware_fk_z_m':z,
                   'user_horizontal_x_m':manual['wheel_center_horizontal_from_hip_m'][side],
                   'user_wheel_relative_z_m':(.055-manual['hip_axis_height_m'][side]) if manual['hip_axis_height_m'][side] is not None else None,
                   'user_reference_is_motor_pair_midpoint':manual.get('hip_point_identification_pending',False),
                   'comparison_applicable':not manual.get('hip_point_identification_pending',False),
                   'interpretation':'User measured the midpoint between motor axes, not a confirmed single hip hinge. Do not compare these values directly or infer a calibration error.'})
    prerequisites={
      'T0':{'status':'communication_and_position_subchecks_pass; full_acceptance_incomplete',
            'three_pairs':json.loads((OUT/'T0_after_reset_result.json').read_text()),
            'missing':['continuous60s record with per-sample age and loop timing','ACTIVE single/dual-actor timing; getup actor missing']},
      'T1':{'status':'supported_pose_measured; natural_rest_trials_not_run',
            'missing':['anti-tip protection and controllable support unloading','10 symmetric and5+5 asymmetric natural rest trials']},
      'T2':{'status':'raw/arm/q captured after software reset; full_repeatability_not_run',
            'missing':['same fixture approach5 times in both directions','3 full power cycles','wheel marked one-turn repeats3 times']},
      'T3':{'status':'not_measurable_with_available_equipment',
            'missing':['independent force/torque sensor','rated fixture and measured force arm','bounded torque bench mode']},
      'T4':{'status':'installed_pin_distances_measured; force_curve_not_measured',
            'missing':['guarded compression fixture and force measurement or traceable supplier curve'],
            'manual_hand_compression_used':False},
      'T5':{'status':'static_data_and_level_discrepancy_recorded; not_passed',
            'user_body_level_observation':True,'board_parallel_confirmed':True,'latest_estimated_tilt_deg':tilt,
            'missing':['quantified external angle reference and uncertainty','cold/warm temperature measurement','repeated known angles/turns','continuous dynamic acceleration record']},
      'T6':{'status':'static_estimator_data_captured; displacement_scale_not_measured',
            'missing':['protected low-slip1m forward/back trials5 each','known arc and wheel-lift truth labels','continuous synchronous record']},
      'T7':{'status':'no_active_rotation_or_delay_identification',
            'missing':['guarded wheel fixture','continuous raw CAN arrival timestamp/rotational truth','bounded wheel bench mode']},
      'T8':{'status':'prerequisites_not_met; no_getup_attempt',
            'missing':['validated and registered getup actor','prior force/spring/dynamic gates','continuous trajectory and external assistance-force/contact truth']}}
    manifest=[]
    for p in sorted(OUT.rglob('*')):
        if p.is_file() and p.name not in ('result.json','artifact_manifest.json'):
            manifest.append({'path':p.relative_to(BASE).as_posix(),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
    save(OUT/'artifact_manifest.json',manifest)
    software_reset_result=json.loads((OUT/'T2_soft_reset_result.json').read_text()) if (OUT/'T2_soft_reset_result.json').exists() else None
    if software_reset_result:
        prerequisites['T2']['software_reset_repeatability']=software_reset_result
    report={'schema':'wheel_leg_full_measurement_session_v1','generated_at':dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).isoformat(),
            'user_prioritized_important_measurements':True,
            'important_measurements_saved':['six motor reception and recovered geometry','same-pose three MCU-only reset trials',
                                             'supported-pose manual dimensions and 24V','static IMU and estimated odometry'],
            'not_independent_measurements':['motor torque constant','spring force-displacement curve'],
            'session':session,'manual_measurements':manual,'tests':prerequisites,'snapshot_sets':datasets,
            'analyses':analyses,'bounded_windows':rings,'bounded_window_summaries':ring_summaries,
            'fk_comparison_unresolved':fk,'latest_dataset':latest_key,
            'all_tests_complete':False,'latest_inhibit':last['chassis']['inhibit'],
            'mcus_ticks_cross_reset_comparable':False,'software_reset_repeatability':software_reset_result,
            'latest_position_lost':[m['position_lost'] for m in last['motor_bank']],
            'raw_artifact_manifest':manifest,
            'events':[json.loads(s) for s in (OUT/'events.jsonl').read_text().splitlines() if s.strip()]}
    report_path=BASE/('real_data_'+OUT.name+'.json')
    save(report_path,report)
    save(OUT/'result.json',{'report':report_path.relative_to(BASE).as_posix(),'all_tests_complete':False,
                          'tests':prerequisites,'analyses':analyses,'manual_measurements':manual,
                          'bounded_window_summaries':ring_summaries})
    progress=ROOT.parent.parent/'progress.md';text=progress.read_text(encoding='utf-8')
    a='<!-- FULL_REAL_TESTS_20261116_BEGIN -->';b='<!-- FULL_REAL_TESTS_20261116_END -->'
    rel=report_path.relative_to(ROOT.parent.parent).as_posix()
    baseline=analyses.get('supported_pose_baseline',{})
    section=f'''{a}
### 0.1 全项实测继续（2026-11-16，最新状态优先于旧会话）

- 受控MCU重启后，三组双快照六路计数均增长、position_lost全false、inhibit=0、几何估计恢复，mode=STOP；没有改零位/烧录/主动使能。旧会话的左轮锁定已经解除，历史失败仍保留。
- 外部尺量：机身净空120mm，左右双电机轴心连线的中点高300mm，左右弹簧销心距240mm，左右轮心均在该中点前方304mm；电压24V。用户已澄清测量起点为双电机中点，不是已确认的单髋轴，撤回此前“髋轴高/距”的标签，不直接拿来和单髋轴FK比较。两轮接地、纸箱仍承重，不能算自然无辅助停位。
- 恢复后的基线{baseline.get('mcu_span_s')}秒，采样位置首尾变化{baseline.get('estimated_xy_endpoint_change_m')}m；只是机载静止估计变化，不是位移精度。最新inhibit={last['chassis']['inhibit']}，仍为零主动输出。
- 用户确认机身水平、H板固定且平行；后续静置机载倾角约1.8–1.9°，与早期14°分段保留；最新数据约{tilt:.2f}°。没有改标定或归零，外部角度/量尺精度未知，T5完整精度不标通过。
- 三次同姿态MCU软重启读数已完成；原始编码器计数与关节角在三次末帧一致，未改零位。此项不等于三次完全断电或正反逼近夹具姿态的完整T2。按用户新要求优先收尾重要测量，暂停扩展次要测试。
- 最后另存32帧遥测短窗；导出时暂停CPU读取后已恢复，原始数据和逐文件SHA-256均保留。短窗不是连续60秒记录。
- 当前数据总汇：[全项实测JSON]({rel})。T0长时连续记录/ACTIVE耗时仍缺；T1自然停位、T2完整夹具/重上电重复性、T6动态真值未完成。T3/T4缺力传感器及额定/防护夹具，不能徒手压弹簧代替。T7主动试验缺防护/连续通道，T8缺起身actor，均未执行。

{b}

'''
    if a in text:text=text[:text.index(a)]+section+text[text.index(b)+len(b):].lstrip('\r\n')
    else:text=text.replace('## 1. 一句话状态',section+'## 1. 一句话状态',1)
    progress.write_text(text,encoding='utf-8')
    print(json.dumps({'report':str(report_path),'datasets':list(datasets),'latest_inhibit':last['chassis']['inhibit'],
                      'latest_tilt_deg':tilt,'fk':fk,'artifact_count':len(manifest)}))


if __name__=='__main__':main()
