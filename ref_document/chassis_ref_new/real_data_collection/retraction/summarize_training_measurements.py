"""Offline evidence index for get-up identification; never accesses hardware."""
import hashlib
import json
import math
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'real_data_collection/training_measurements_20261117'
OUT.mkdir(exist_ok=True)

def read(path):
    return json.loads((ROOT / path).read_text(encoding='utf-8'))

def save(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')

def snapshot(path):
    value = read(path+'/result.json')['b']
    bank = (ROOT/path/'b_g_motor_snapshot.bin').read_bytes()
    assert len(bank) == 248
    value['raw_bank'] = []
    for i in range(6):
        arm, velocity, current = struct.unpack_from('<3f', bank, i*40+16)
        value['raw_bank'].append(dict(arm_rad=arm, velocity_rad_s=velocity,
            reported_current_a=current, position_lost=bool(bank[i*40+38]),
            raw=struct.unpack_from('<H', bank, i*40)[0],
            total_counts=struct.unpack_from('<q', bank, i*40+8)[0]))
    return value

stats_path='diagnostics/training_static_20261117_01_complete'
trace_path='diagnostics/training_static_20261117_01_trace'
s=read(stats_path+'/result.json'); t=read(trace_path+'/result.json')
rows=t['samples']; deg=180/math.pi
assert s['state']==2 and s['duration_ms']==60000
assert t['frozen'] and len(rows)==500 and not t['rejected_slots']
assert set(t['tick_gaps_ms'])=={120}
assert rows[0]['tick']==s['started'] and rows[-1]['tick']==s['ended']-120
assert all(r['mode']==0 and all(x==0 for x in r['torque']) for r in rows)
limits=read('user/algorithm/model/manifest.json')['physical_knee_limits_rad']
tilts=[math.acos(max(-1,min(1,-r['gravity'][2])))*deg for r in rows]
static=dict(duration_ms=s['duration_ms'], online_statistics_samples=s['count'],
    invalid_samples=s['invalid_count'], statistics_interval_ms=2,
    raw_frames=len(rows), raw_interval_ms=120, raw_span_ms=t['span_ms'],
    raw_stream_500_hz=False, max_control_step_ms=s['max_step_ms'],
    control_step_gaps=s['step_gaps'], max_motor_age_ms=s['max_motor_age_ms'],
    max_imu_age_ms=s['max_imu_age_ms'], max_remote_age_ms=s['max_remote_age_ms'],
    tx_fail_delta=s['tx_fail_delta'], rx_overflow_delta=s['rx_overflow_delta'],
    gyro_std_rad_s=[s['statistics']['gyro_'+axis]['std'] for axis in 'xyz'],
    yaw_peak_to_peak_deg=(s['statistics']['yaw']['max']-s['statistics']['yaw']['min'])*deg,
    yaw_net_drift_deg=None, tilt_range_deg=[min(tilts),max(tilts)],
    accel_norm_mean_m_s2=s['statistics']['accel_norm']['mean'],
    joint_position_peak_to_peak_deg=[(s['statistics'][f'q{i}']['max']-s['statistics'][f'q{i}']['min'])*deg for i in range(6)],
    knee_internal_angle_deg=[270+s['statistics']['q1']['mean']*deg,270-s['statistics']['q4']['mean']*deg],
    right_knee_below_configured_q_min_deg=(limits[0][0]-s['statistics']['q1']['mean'])*deg,
    physical_angle_truth=False, inhibit_or=s['inhibit_or'],
    imu_drdy_edges=s['drdy_edges_delta'], imu_unconsumed_edges=s['imu_missed_delta'],
    imu_note='DRDY about 2 kHz; imu_service permits at most one read per HAL millisecond. Unconsumed edge counter is not a count of stale control frames. Static run max IMU age 2 ms; no IMU freshness inhibit.',
    online_statistics_precision='float32 Welford; encoder quantization and adaptive gyro bias retained',
    source_statistics=stats_path, source_raw=trace_path)
save(OUT/'static_analysis.json',static)

before_path='diagnostics/training_side_identity_20261117_01_before'
after_path='diagnostics/training_side_identity_20261117_01_after'
b=snapshot(before_path); a=snapshot(after_path)
deltas=[y['total_counts']-x['total_counts'] for x,y in zip(b['raw_bank'],a['raw_bank'])]
assert b['nonzero_torque_frames']==a['nonzero_torque_frames']==0
assert b['enable_commands']==a['enable_commands']==0
identity=dict(operator_action='Robot-own RIGHT leg manually unfolded and held; left leg unchanged; S1 down.',
    raw_encoder_delta_counts=deltas, feedback_ids=[n['feedback_id'] for n in a['nodes']],
    right_leg_ids_supported=abs(deltas[0])>10 and abs(deltas[1])>10 and deltas[3:5]==[0,0],
    individual_hip_knee_role_and_absolute_zero_verified=False,
    wheel_side_verified=False, joint_delta_deg=[(y-x)*deg for x,y in zip(b['measurement']['joint_position_rad'],a['measurement']['joint_position_rad'])],
    after_knee_internal_angle_deg=[270+a['measurement']['joint_position_rad'][1]*deg,270-a['measurement']['joint_position_rad'][4]*deg],
    after_inhibit=a['chassis']['inhibit'], before=before_path, after=after_path,
    observation_scope='Endpoint comparison; no continuous manual-motion trajectory or external angle ground truth.')
save(OUT/'right_leg_identity.json',identity)

old_path='diagnostics/retraction_hold_after_20261117_02'
later_path='diagnostics/training_measurements_precheck_20261117_01'
old=snapshot(old_path); later=snapshot(later_path)
passive=dict(before=old_path,after=later_path,
    elapsed_between_chassis_snapshots_ms=later['chassis']['tick']-old['chassis']['tick'],
    nonzero_command_counter_before_after=[old['nonzero_torque_frames'],later['nonzero_torque_frames']],
    raw_encoder_delta_counts=[y['total_counts']-x['total_counts'] for x,y in zip(old['raw_bank'],later['raw_bank'])],
    calibrated_arm_delta_deg=[(y['arm_rad']-x['arm_rad'])*deg for x,y in zip(old['raw_bank'],later['raw_bank'])],
    position_lost_after=[x['position_lost'] for x in later['raw_bank']],
    interpretation='Right DM4/DM3 changed during an unrecorded zero-command interval. No conclusion about exact timing, spontaneous vs external motion, or force cause. Right wheel position_lost invalidates joint measurement cached then; fresh post-flash pose is separate evidence.')
save(OUT/'post_stop_endpoint_changes.json',passive)

wheel_checks=[]
for before,after,observation in [
    (after_path,'diagnostics/training_right_wheel_identity_20261117_01_after',
     'Operator confirmed whole leg moved; tyre was NOT rotated relative to shin.'),
    ('diagnostics/training_right_wheel_identity_20261117_01_after','diagnostics/training_right_wheel_identity_20261117_02_after',
     'Operator reported wheel can rotate backward but not forward; isolation of one wheel not established.')]:
    if not (ROOT/after/'result.json').exists(): continue
    first,last=snapshot(before),snapshot(after)
    wheel_checks.append(dict(before=before,after=after,operator_observation=observation,
        raw_encoder_delta_counts=[y['total_counts']-x['total_counts'] for x,y in zip(first['raw_bank'],last['raw_bank'])],
        calibrated_arm_delta_rad=[y['arm_rad']-x['arm_rad'] for x,y in zip(first['raw_bank'],last['raw_bank'])],
        position_lost=[x['position_lost'] for x in last['raw_bank']],
        reported_current_a=[x['reported_current_a'] for x in last['raw_bank']],
        nonzero_commands=last['nonzero_torque_frames'],enable_commands=last['enable_commands'],
        side_or_direction_verified=False))
save(OUT/'wheel_manual_attempts.json',wheel_checks)

isolated_wheels=[]
for side,index,before,after in [
    ('right',2,'diagnostics/training_right_wheel_identity_20261117_03_before','diagnostics/training_right_wheel_identity_20261117_03_after'),
    ('left',5,'diagnostics/training_right_wheel_identity_20261117_03_after','diagnostics/training_left_wheel_identity_20261117_01_after')]:
    if not (ROOT/after/'result.json').exists(): continue
    first,last=snapshot(before),snapshot(after)
    counts=[y['total_counts']-x['total_counts'] for x,y in zip(first['raw_bank'],last['raw_bank'])]
    angles=[y['arm_rad']-x['arm_rad'] for x,y in zip(first['raw_bank'],last['raw_bank'])]
    assert last['nonzero_torque_frames']==first['nonzero_torque_frames']==0
    assert last['enable_commands']==first['enable_commands']==0
    assert not any(x['position_lost'] for x in first['raw_bank']+last['raw_bank'])
    isolated_wheels.append(dict(side=side,index=index,feedback_id=last['nodes'][index]['feedback_id'],
        before=before,after=after,raw_encoder_delta_counts=counts,
        calibrated_wheel_angle_delta_rad=angles[index],
        calibrated_wheel_angle_delta_deg=angles[index]*deg,
        isolated_identity_supported=abs(counts[index])>100 and all(abs(x)<=16 for i,x in enumerate(counts) if i!=index),
        operator_action='forward manual tyre rotation; left trial explicitly specified top tread toward robot front',
        canonical_forward_sign_matches=angles[index]>0,
        final_direction_verified=False,
        conflict='Manual result conflicts with prior ground forward-push evidence in COMMISSIONING_LOG_20261113.md. Preserve both; no sign changes until ground-direction/frame cross-check.'))
save(OUT/'isolated_wheel_identity.json',isolated_wheels)

cad_path=ROOT/'../../sample_weight_file/artifacts/gas_spring_resolved.json'
cad=json.loads(cad_path.read_text(encoding='utf-8'))
stroke=cad['installation_measurements']['stroke_consistency']
spring_audit=dict(kind='offline provenance comparison; no new physical measurement',
    current_training_free_pin_mm=250,
    historical_cad_free_pin_mm=[x['modeled_extended_hole_distance_m']*1000 for x in stroke['per_leg_model']],
    historical_bare_spring_free_length_mm=stroke['bare_extended_length_m']*1000,
    historical_cad_geometry_confirmed=cad['geometry_confirmed'],
    historical_status=stroke['status'],
    source=str(cad_path.resolve()),source_sha256=hashlib.sha256(cad_path.read_bytes()).hexdigest(),
    conclusion='Neither 250 mm provisional assumption nor old about-228 mm CAD candidate is a new measurement of current hardware. Bare 205 mm excludes adapters. Need current installed pin distance paired with encoder/CAD pose and full-extension status; do not silently replace firmware limits or accepted force curve.')
save(OUT/'spring_length_provenance.json',spring_audit)

previous=read('real_data_collection/retraction/hold_session.json')
previous.update(state='Current measurement image 0xD2611174, STOP; collecting identification data',
    active_manifest='diagnostics/training_measurements_flash_20261117_01/session.json',
    trial_2=read('diagnostics/retraction_hold_trial_20261117_02/analysis.json'),
    trial_2_operator_report='Robot-own RIGHT leg visibly folded or bounced; confirmed by operator.',
    trial_2_complete_hold_passed=False,
    physical_side_recheck='training_measurements_20261117/right_leg_identity.json')
save(ROOT/'real_data_collection/retraction/hold_session.json',previous)

plan=read('real_data_collection/next_training_plan_20261117.json')
summary=dict(status='partial_measurements_in_progress', task='natural-rest get-up identification, plus standing takeover',
    fixture='Suspended, fixed body, wheels off ground; S1 down during this session',
    supply_static_v=24, supply_source='Operator reported static supply reading; instrument and precision unspecified',
    supply_under_load_v=None, independent_force_or_torque_instrument_available=False,
    accepted_inputs=plan['accepted_inputs'], assumptions=plan['retain_as_assumptions'],
    normal_rl_firmware_modified=False, getup_training_started=False,
    static_result='static_analysis.json', side_identity='right_leg_identity.json',
    post_stop_result='post_stop_endpoint_changes.json',
    wheel_manual_attempts='wheel_manual_attempts.json',
    wheel_one_direction_resistance='reported; power-off manual comparison pending; software nonzero/enables both 0',
    spring_length_audit='spring_length_provenance.json',
    stages={
        'T0': {'status':'partial','measured':'60 s / 30000 online samples, 500 raw frames, ages and timing, zero TX/RX failures', 'missing':'60 s full 500 Hz raw stream and actual bus timestamp/latency'},
        'T1': {'status':'needs_ground_fixture','measured':'suspended rest pose only','missing':'natural unassisted reset poses and which parts contact ground, repeatability'},
        'T2': {'status':'partial','measured':'right leg DM4/DM3 passive identity, historical soft reset repeats','missing':'individual hip/knee mechanical roles, matched CAD absolute pose, power-cycle and approach repeatability'},
        'T3': {'status':'partial_no_force_instrument','measured':'command/reported current/motion histories at 20 and 25 Nm command caps','missing':'independent torque gain, friction/efficiency and pose-dependent load separation'},
        'T4': {'status':'accepted_nominal_model','measured':'no new force measurement','missing':'installed free pin distance, hysteresis; nominal manufacturer curve retained'},
        'T5': {'status':'partial','measured':'60 s suspended static noise, gravity, yaw range','missing':'known-angle dynamic truth and cold/warm repeatability'},
        'T6': {'status':'partial','measured':'accepted wheel radius and symmetric left assumption','missing':'wheel physical side/sign confirmation, loaded rolling/slip with external displacement'},
        'T7': {'status':'partial','measured':'existing motion traces, fresh static feedback ages','missing':'wheel unwrap-loss cause under motion and hardware command-response delay'},
        'T8': {'status':'not_passed','measured':'bounded supported startup and suspended folding only','missing':'complete rise, support release, independent balance and load response'}},
    next_actions=['passive wheel identity/sign while STOP','CAD-matched leg pose and reset repeatability','pose-dependent bounded response after physical mapping is established','natural ground reset/contact distribution with changed fixture','supported get-up and load sag recordings'],
    training_use='Static noise/current-response evidence can constrain simulation. Unidentified physics remain labelled randomized ranges. Suspended poses must not be labelled natural ground resets or successful standing.',
    completion_claim=False)
summary['assumptions']['supply_v']=None
if (OUT/'wheel_repair.json').exists():
    summary['wheel_one_direction_resistance']='Operator reported mechanical repair; controller restarted, STOP and zero commands verified; specific mechanical fault unspecified.'
    summary['wheel_repair']='wheel_repair.json'
if isolated_wheels:
    summary['isolated_wheel_identity']='isolated_wheel_identity.json'
    summary['stages']['T6']['measured']+='; separately moved right ID3 and left ID2 confirmed; both reported-forward manual rotations give negative calibrated angles'
    summary['stages']['T6']['missing']='resolve conflict with historical ground forward-push direction; loaded rolling/slip with external displacement'
if (OUT/'knee_distance_check.json').exists():
    summary['knee_distance_check']='knee_distance_check.json'
    summary['stages']['T2']['measured']+='; operator confirmed 412 mm hip-to-wheel centre distance on both sides, about 126.97 deg included knee under 210/250 mm planar model, inconsistent with present calibration'
if (OUT/'passive_right_leg_analysis.json').exists():
    summary['passive_right_leg_analysis']='passive_right_leg_analysis.json'
    summary['stages']['T2']['measured']+='; 385 raw frames over 46.08 s fixed-knee manual right-leg movement: DM4/DM3 raw slope 1.00255; current knee angle range 32.01 deg, candidate knee sign -1 range 0.267 deg assuming unchanged hip sign. No calibration applied'
    summary['stages']['T2']['missing']='absolute hip direction/reference, full physical pose, applied sign/zero verification, power-cycle and approach repeatability'
    summary['next_actions']=['measure hip-to-knee vertical offsets and fore/aft placement in suspended pose','resolve wheel direction with grounded forward push','derive and validate calibration before active dynamics tests','natural ground reset/contact distribution','supported get-up and load sag recordings']
if (ROOT/'diagnostics/training_wheel_loss_20261117_01/result.json').exists():
    summary['stages']['T7']['measured']+='; right wheel ambiguity counter +2 during manual measurement, sticky position_lost; cached joint vector excluded, fresh raw leg feedback used offline'
if (OUT/'suspended_pose_measurements.json').exists():
    summary['suspended_pose_measurements']='suspended_pose_measurements.json'
    summary['pose_calibration_status']='not applied: operator confirms wheel axes 215 mm forward on both sides, but knee-behind/165 mm drop with 210/250 mm links is geometrically inconsistent under the planar model'
    summary['next_actions'][0]='reconcile hip/knee/wheel centre definitions and robot forward direction in one unchanged side-view pose before deriving calibration'
save(OUT/'summary.json',summary)

sources=[stats_path,trace_path,before_path,after_path,old_path,later_path,
    'diagnostics/retraction_hold_trial_20261117_02','diagnostics/training_measurements_flash_20261117_01']
sources += [x['after'] for x in wheel_checks]
sources += [x[key] for x in isolated_wheels for key in ('before','after')]
if (OUT/'wheel_repair.json').exists(): sources += ['diagnostics/training_wheel_repair_20261117_01_status']
sources += ['diagnostics/'+name for name in ('training_knee_distance_20261117_01_after','training_knee_invariance_20261117_01_after','training_passive_right_leg_20261117_01_start','training_passive_right_leg_20261117_01_trace','training_wheel_loss_20261117_01') if (ROOT/'diagnostics'/name).exists()]
sources += ['diagnostics/'+name for name in ('training_suspended_pose_20261117_01_after','training_suspended_pose_20261117_02_after') if (ROOT/'diagnostics'/name).exists()]
manifest={}
for directory in sources:
    for file in sorted((ROOT/directory).rglob('*')):
        if file.is_file(): manifest[file.relative_to(ROOT).as_posix()]=hashlib.sha256(file.read_bytes()).hexdigest()
save(OUT/'evidence_sha256.json',manifest)
print(json.dumps({'static':static,'right_leg_identity':identity},ensure_ascii=False))
