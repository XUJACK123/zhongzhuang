"""Offline consolidation of immutable trial records; never accesses the probe."""
from pathlib import Path
import datetime as dt,hashlib,json,math
ROOT=Path(__file__).resolve().parents[2];HERE=Path(__file__).resolve().parent
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def save(p,obj):p.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
def knee(f,s):
    hip=0 if s==1 else 3;a=f['arm_rad'][s]-f['arm_rad'][hip]
    return (1 if s==1 else -1)*(a/2+math.asin((113.4/135)*math.sin(a/2))-math.pi/2)
trials=[];plots=[]
for folder in sorted(HERE.glob('trial_*')):
    if not (folder/'result.json').exists() or not (folder/'session.json').exists():continue
    meta=read(folder/'session.json');record=read(folder/'result.json');fs=record['frames'];s=meta.get('selected_policy_index',1)
    peak=max(abs(f['protocol_feedback'][s]) for f in fs);nom=max(abs(f['nominal_requested_nm']) for f in fs)
    response=peak>max(.1,.1*nom)
    q=[knee(f,s) for f in fs];origin=fs[0]['arm_rad'][s]
    row={'trial':folder.name,'selected_policy_index':s,'selected':meta['selected'],
      'planned_levels_a':meta.get('current_levels_a',[meta['current_limit_a']]),
      'logged_peak_request_a':max(abs(f['requested_a']) for f in fs),'logged_peak_nominal_request_nm':nom,
      'protocol_reported_peak_nm':peak,'response_observed_in_protocol':response,
      'motor_shaft_excursion_deg':max(abs(f['arm_rad'][s]-origin) for f in fs)*180/math.pi,
      'derived_joint_knee_range_deg':(max(q)-min(q))*180/math.pi,
      'derived_joint_angle_uses_existing_calibration_not_external_measurement':True,
      'reason':record['summary']['reason'],'frame_count':len(fs),'loop_max_us':record['summary']['loop_max_us'],
      'max_temperature_c':max(max(f['temperature_c']) for f in fs),
      'can_error_counts_after':{key:record['post_trial_status']['can'][key] for key in ('tx_error','rx_error','bus_off')},
      'fifo_overflow_after':record['post_trial_status']['fifo_overflow'],
      'manual_disturbance':record.get('operator_observation',{}).get('wheel_touched_or_turned_during_previous_trial',False),
      'independent_torque_or_force_reference':False}
    save(folder/'offline_assessment.json',row);trials.append(row)
    if response and meta.get('current_limit_a',0)>=15:
        point_sets=[]
        for i in sorted(set(f['trial_index'] for f in fs if f['phase']=='RUNNING')):
            running=[f for f in fs if f['phase']=='RUNNING' and f['trial_index']==i]
            point_sets.append({'name':f"{row['planned_levels_a'][i]:g} A target",
                'points':[[abs(f['nominal_requested_nm']),abs(f['protocol_feedback'][s])] for f in running]})
        plots.append({'selected':s,'trial':folder.name,'torque':point_sets,
          'angle':[{'name':'Motor shaft','points':[[abs(f['requested_a']),(f['arm_rad'][s]-origin)*180/math.pi] for f in fs if f['phase']=='RUNNING']},
                   {'name':'Derived joint knee','points':[[abs(f['requested_a']),(knee(f,s)-q[0])*180/math.pi] for f in fs if f['phase']=='RUNNING']}]})
summary={'updated_utc':dt.datetime.now(dt.timezone.utc).isoformat(),'timezone':'Asia/Hong_Kong',
  'trial_count':len(trials),'logged_frame_count':sum(t['frame_count'] for t in trials),'trials':trials,
  'actual_scope':'Two knee-motor current-command/protocol-feedback and bounded motion diagnostics, not independent shaft torque calibration.',
  'T3_independent_torque_calibration_completed':False,'T4_independent_spring_force_curve_completed':False,'T8_getup_executed':False,
  'limitations':['DM feedback is protocol-reported Nm; current commands use the existing configured conversion, not an independent current sensor.',
     'Single knee-motor excitation also moves the unpowered hip through the linkage; this is not an isolated spring-force test.',
     'Encoder-derived virtual knee angle differs from the selected motor shaft angle.',
     'Some commanded stages produced near-zero feedback; these must not be used as evidence of large resisting load.',
     'No full-duration 500 Hz stream, external load reference, active battery-voltage trace, or independently measured spring axial force.',
     'Getup actor is not registered in the normal firmware.'],
  'spring_force_n':None,'calibration_constants_changed':False,'commissioning_flags_promoted':False}
save(HERE/'identification_summary.json',summary)
latest_plots=[]
for selected in (1,4):
    choices=[p for p in plots if p['selected']==selected]
    if choices:latest_plots.append(choices[-1])
save(HERE/'plot_data.json',latest_plots)
main_path=ROOT/'real_data_collection/real_data_full_tests_20261116_165832.json';main=read(main_path)
main['active_identification_summary']=summary
save(main_path,main)
rows=[]
for selected in (1,4):
    choices=[r for r in trials if r['selected_policy_index']==selected and r['response_observed_in_protocol']]
    if choices:
        best=max(choices,key=lambda r:r['protocol_reported_peak_nm'])
        rows.append(f"| {'右膝DM3' if selected==1 else '左膝DM2'} | {best['logged_peak_request_a']:.2f} | {best['protocol_reported_peak_nm']:.2f} | {best['motor_shaft_excursion_deg']:.2f} | {best['derived_joint_knee_range_deg']:.2f} | {best['reason']} |")
text='\n'.join(['- 已保存 '+str(len(trials))+' 个独立批次、'+str(summary['logged_frame_count'])+' 帧；失败/提前停止批次一并保留。',
 '| 被测轴 | 最大记录请求A | 协议反馈峰值Nm | 电机轴角最大变化° | 按现有标定反解膝关节角范围° | 退出原因 |',
 '|---|---:|---:|---:|---:|---|',*rows,
 '- 表内为有反馈的批次；同一指令也出现过反馈近零的批次，输出兑现仍不稳定，不能把这类数据当作弹簧负载过大。CAN错误/溢出以各批次记录为准。',
 '- 电机轴角与闭链反解的膝关节角不同，反解仍依赖现有标定；没有把单电机电流扫幅直接换成纯弹簧轴力。',
 '- 当前完成的是主动力矩请求/协议反馈与机构微动子项。T3独立力矩标定、T4纯弹簧力曲线未完成；T8缺已注册起身actor，未执行起身。',
 '- 总汇：[完整实测JSON](ref_document/chassis_ref_new/real_data_collection/real_data_full_tests_20261116_165832.json)；[短汇总](ref_document/chassis_ref_new/real_data_collection/identification/identification_summary.json)。原始BIN、完整OpenOCD日志、源码/镜像及SHA-256清单均保留。',
 '- 正常RL模型、标定及验证标志未修改；当前板上为独立诊断，结束态与后续控制链路检查见最新会话。'])
progress=ROOT.parents[1]/'progress.md';body=progress.read_text(encoding='utf-8')
a='<!-- LIMITED_CURRENT_20261116_BEGIN -->';b='<!-- LIMITED_CURRENT_20261116_END -->'
block=a+'\n### 0.2 主动采集与输出链路检查（'+dt.datetime.now(dt.timezone(dt.timedelta(hours=8))).strftime('%Y-%m-%d %H:%M:%S')+'）\n\n'+text+'\n\n'+b
body=body[:body.index(a)]+block+body[body.index(b)+len(b):];progress.write_text(body,encoding='utf-8')
for folder in HERE.glob('trial_*'):
    manifest=[{'path':str(p.relative_to(folder)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(folder.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json']
    save(folder/'artifact_manifest.json',manifest)
print(json.dumps({'trials':len(trials),'frames':summary['logged_frame_count'],'main_json_bytes':main_path.stat().st_size,'main_json_sha256':hashlib.sha256(main_path.read_bytes()).hexdigest(),'plot_trials':[p['trial'] for p in latest_plots]},ensure_ascii=False))
