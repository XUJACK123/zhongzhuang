"""Create an evidence index for completed current probes; no hardware access."""
import json,hashlib,struct,subprocess,sys,math
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2];OUT=ROOT/'real_data_collection/force_identification_20261117';OUT.mkdir(exist_ok=True)
reports=[];hashes={}
for trace in sorted((ROOT/'diagnostics').glob('current_probe_*_trace')):
    if not (trace/'result.json').exists(): continue
    with (trace/'analysis.log').open('w',encoding='utf-8') as log:
        subprocess.run([sys.executable,str(Path(__file__).with_name('analyze.py')),str(trace)],stdout=log,check=True)
    d=json.loads((trace/'analysis.json').read_text());trigger=trace.with_name(trace.name.removesuffix('_trace')+'_trigger')
    d.pop('interpretation',None);reports.append(d)
    for directory in (trace,trigger):
        for file in directory.rglob('*'):
            if file.is_file(): hashes[file.relative_to(ROOT).as_posix()]=hashlib.sha256(file.read_bytes()).hexdigest()
summary=dict(kind='measured current and raw-motion response; no independent force ground truth',
    completed_trials=len(reports),trials=reports,
    geometry='User chooses selected STEP; repeated ruler survey paused; no new absolute calibration applied',
    unavailable=['independent output-shaft torque','spring force/hysteresis separation','friction/efficiency separation','ground reaction force'],
    source_limits='Free suspended legs; one motor can move the other through linkage. Nominal current-to-torque coefficient is assumed, not identified here.',
    measured_force_n=None,independent_torque_scale_identified=False)
if (OUT/'final_stop_verification.json').exists(): summary['final_stop_verification']=json.loads((OUT/'final_stop_verification.json').read_text())
(OUT/'summary.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n',encoding='utf-8')
for directory in (ROOT/'diagnostics').glob('current_probe_*'):
    if directory.is_dir():
        for file in directory.rglob('*'):
            if file.is_file():hashes[file.relative_to(ROOT).as_posix()]=hashlib.sha256(file.read_bytes()).hexdigest()
(OUT/'evidence_sha256.json').write_text(json.dumps(hashes,indent=2)+'\n',encoding='utf-8')
lines=['# 力相关数据：电流与原始电机运动响应','',
    '几何按用户选定 STEP 使用，暂停重复尺测。当前没有独立测力仪；以下 Nm 是名义电流换算值，不能作为实测轴端力矩或弹簧轴力。',
    '',f'已归档 {len(reports)} 轮实机记录。原始波形为10 ms采样，精确停止事件为2 ms控制时刻。',
    '', '| 电机 | 原始电流方向目标（名义 Nm） | 实际运行 ms | 本轮达到的名义 Nm | 反馈峰值 A | 活动帧最大电机轴位移° | 停止原因 |',
    '|---|---:|---:|---:|---:|---:|---|']
for d in reports:
    e=d.get('stop_event',{});nm=e.get('selected_raw_nominal_nm_before_stop')
    nm=math.copysign(max(abs(nm or 0),d['max_abs_command_a']*d['nominal_kt_nm_a']),d['raw_nominal_target_nm'])
    displacement=max(d['max_abs_raw_output_delta_deg'])
    lines.append(f"| DM{d['dm_id']} | {d['raw_nominal_target_nm']:+g} | {e.get('duration_ms','—')} | {nm:+.3f} | {d['max_abs_feedback_a']:.3f} | {displacement:.3f} | { {1:'按时停止',2:'位移边界',3:'速度边界'}.get(d['guard_reason'],str(d['guard_reason']))} |")
lines+=['','表中位移峰值取10 ms活动帧，2 ms停止事件和停机后运动可能更大；精确触发值见各 analysis.json 的 stop_event，含约200 ms停机后窗口的峰值另存 max_abs_delta_including_recorded_post_stop_deg。九轮记录中含停机后最大电机轴位移约1.76°，不是外部机械角度真值或全局运动上界。',
    '', '已观察到单个膝电机驱动时同侧髋轴也运动。可检测运动阈值仅用于比较当前姿态的响应，不能单独当作弹簧负载或静摩擦。各试验初始原始角度见对应 trigger 的 g_supported_trial.bin。',
    '', '先前20/25 Nm联合位置控制的电流跟随另见 archived_current_tracking.json；其控制器和姿态与本次不同，不能直接当作同条件力矩重复试验。',
    '', '正常 RL 模型、共享方向/零点和实机验证标志均未因本次更新。下一步若要独立轴端力矩，需要已知外力/力臂或测力传感器；若要关节静态抗力曲线，需要约束伴随髋运动并配对有效机械姿态。']
(OUT/'README.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(json.dumps({'completed_trials':len(reports),'motors':[d['dm_id'] for d in reports],'output':str(OUT)}))
