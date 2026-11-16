#!/usr/bin/env python3
"""Reporting only: archive frozen results without changing models or evaluators."""
from datetime import datetime
import json
from pathlib import Path
import zipfile
from zoneinfo import ZoneInfo

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from realdata_train import PROJECT,digest


def main():
    root=PROJECT/'runs/realdata_20261117';workspace=PROJECT.parents[1]
    if (root/'final_summary.json').exists():
        raise ValueError('Final report already exists; preserve the completed round instead of appending it again.')
    audit=json.loads((root/'heldout_audit.json').read_text())
    if not audit['integrity_passed'] or not audit['all_planned_episodes_completed']:
        raise ValueError('final reporting requires a complete, internally consistent audit')
    frozen=json.loads((root/'frozen_candidates/manifest.json').read_text())
    selected=json.loads((root/'final_selection.json').read_text())
    status=json.loads((root/'latest_status.json').read_text())
    groups={g['label']:g for g in audit['groups']}
    now=datetime.now(ZoneInfo('Asia/Shanghai'));start=datetime.fromisoformat('2026-11-17T01:53:00+08:00')
    exports={}
    for name in ['final_export_near','final_export_full','final_export_balance_near','final_export_balance_full']:
        path=root/name/'manifest.json';m=json.loads(path.read_text())
        m['host_c_parity'].update(atol=2e-5,rtol=1e-4);path.write_text(json.dumps(m,indent=2))
        for file,sha in m['generated_sha256'].items():
            if digest(root/name/file)!=sha:raise ValueError('C export file changed: '+name+'/'+file)
        exports[name]=m['host_c_parity']
    summary={'schema':'realdata_training_final_report_20261117_v1',
        'started_at':start.isoformat(),'finished_at':now.isoformat(),'elapsed_minutes':(now-start).total_seconds()/60,
        'time_budget_minutes':300,'budget_exceeded':now>start.replace(hour=6,minute=53),
        'work_status':'Offline training, frozen evaluation and archive complete.',
        'model_status':'NOT QUALIFIED: incomplete nominal success and failed sensitivity evaluation.',
        'objective':'Supported rise, complete support removal, stable dwell, smooth handoff and frozen balance takeover.',
        'recorded_ppo_learner_transitions_sum':status['recorded_last_transition_counts_sum'],
        'ppo_count_note':'Sum of each retained experiment last counter; parallel experiments and different curricula remain separate. Synthetic supervised samples excluded.',
        'recorded_ppo_runs':len(status['runs']),'gpu_utilization_observed_percent':[94,98],
        'selected_development':selected['candidates'],'frozen_at':frozen['frozen_at'],
        'frozen_manifest_sha256':digest(root/'frozen_candidates/manifest.json'),
        'heldout_groups':audit['groups'],'heldout_episodes':audit['completed_episodes'],
        'heldout_sensitivity_successes':sum(g['passed'] for g in audit['groups'][6:]),
        'heldout_sensitivity_trials':sum(g['completed'] for g in audit['groups'][6:]),
        'trajectory_label_agreement':audit['integrity_passed'],'all_scopes_passed':audit['all_tested_scopes_passed'],
        'regression_tests':{'passed':51,'log':'regression_final_before_freeze.log','sha256':digest(root/'regression_final_before_freeze.log')},
        'c_exports':exports,'hardware_accessed':False,'flashed':False,'encoder_calibration_changed':False,
        'production_firmware_changed':False,
        'physical_inputs':{'mass_kg':21.9,'effective_wheel_radius_m':.7/(4*np.pi),
            'left_wheel_radius':'user-requested symmetry assumption','gas_spring_label':'BG450-080',
            'force_curve':'provisional 450 N / 80 mm / 1.7 progression / polytropic exponent1.2',
            'hysteresis':'unmeasured hypothesis: 15 N seal term + 20 Ns/m drag',
            'target_hip_height_m':.35,'motor_caps_nm':[20,20,2,20,20,2]},
        'limitations':['Actual pressure, spring force/hysteresis, rotor inertia and gear drag are unmeasured.',
            'Coarse surveyed pose conflicts with old encoder references; no real calibration changed.',
            '350 mm hip target and virtual fixture are development designs, not measured physical support.',
            'Serial equivalent geometry and body inertia shape inherited; total mass adjusted in base body.',
            'Ideal IMU/encoder inputs; real filtering, CAN transport and timing are not reproduced.',
            'GPU/CPU full physical parity remains outside its tight budget; CPU is evaluation authority.',
            'Nominal held-out success is limited; all 18 sensitivity cases failed. No real-robot acceptance.'],
        'next_work':['Identify independent spring loading/unloading force and actual installed geometry/pressure.',
            'Reconcile encoder references and measure disabled supported/natural initial states.',
            'Fit actuator/passive dynamics and sensor timing, then train a staged robust curriculum.',
            'Use new held-out seeds for a future model; keep this frozen evaluation unchanged.']}
    (root/'final_summary.json').write_text(json.dumps(summary,indent=2))
    figure,axes=plt.subplots(2,1,figsize=(10,9),gridspec_kw={'height_ratios':[1.2,1.]})
    x=np.arange(3);width=.35
    for i,(role,label) in enumerate([('near','Near-pose weights (u25)'),('full','Full-pose weights (u20)')]):
        rows=[groups[role+'_'+scope] for scope in ['near','mixed','full']]
        bars=axes[0].bar(x+(i-.5)*width,[100*r['passed']/r['completed'] for r in rows],width,label=label)
        for bar,row in zip(bars,rows):
            axes[0].text(bar.get_x()+bar.get_width()/2,bar.get_height()+2,f'{row["passed"]}/{row["completed"]}',ha='center')
    axes[0].set_xticks(x,['Near reset 0..0.2','Mixed reset 0..1','Full surveyed reset'])
    axes[0].set_ylim(0,110);axes[0].set_ylabel('Held-out complete success (%)');axes[0].legend()
    axes[0].grid(axis='y',alpha=.25)
    labels=['Near: randomized hypotheses','Full: randomized hypotheses','Full: spring 225 N','Full: spring 900 N',
            'Full: spring 1178 N (conditional)','Full: reflected inertia x0.5','Full: reflected inertia x2']
    sensitivity=audit['groups'][6:]
    table=axes[1].table(cellText=[[label,f'{r["passed"]}/{r["completed"]}'] for label,r in zip(labels,sensitivity)],
        colLabels=['Unmeasured-condition sensitivity','Pass / trials'],cellLoc='left',loc='center',colWidths=[.78,.2])
    table.auto_set_font_size(False);table.set_fontsize(10);table.scale(1,1.55);axes[1].axis('off')
    figure.suptitle('Frozen CPU evaluation: 66/66 audited, model NOT qualified')
    figure.text(.5,.01,'Each model retains its declared handoff configuration. Zero support and all 10 s balance gates required. No hardware validation.',ha='center',fontsize=8)
    figure.tight_layout(rect=(0,.025,1,.96))
    figure.savefig(root/'heldout_results.png',dpi=160);figure.savefig(root/'heldout_results.pdf');plt.close(figure)
    text=(root/'README.md').read_text()
    old='当前文件在训练过程中建立，最终结论以`final_summary.json`及冻结后的CPU报告为准。'
    text=text.replace(old,'本轮已结束，独立结果未通过整体验收；结论见`final_summary.json`、`heldout_audit.json`。')
    text+='''

## 最终结果（权重冻结后）

| 冻结候选 | 近站姿8种子 | 中间姿态8种子 | 完整粗测姿态8种子 |
|---|---:|---:|---:|
| 近站姿update25 | 7/8 | 1/8 | 0/8 |
| 完整姿态update20 | 0/8 | 2/8 | 2/8 |

18个假设参数敏感性回合全部失败。66/66回合均完成，来源/种子/轨迹逐帧复算一致；没有把失败或提前结束回合剔除。开发集完整姿态6/8下降到独立完整姿态2/8，说明泛化不足。**两套模型均未通过所声明范围的整体验收，不能当作可直接上车的起身模型。**

`frozen_candidates/near`和`frozen_candidates/full`各保存匹配的`getup.pt`、`balance.pt`、配置和物理快照。`heldout_cpu/`保存所有分组报告、逐回合增量结果与原始轨迹；`heldout_audit.json`复核全部66条。`final_export_*`为两套起身/平衡C推理副本，各1024组C/Torch对拍通过；没有安装到生产固件。

`media/near_u25_development.mp4`、`media/frozen_full_u20_development.mp4`是最终权重的成功开发回放，均有逐帧轨迹和审计；它们不是整体成功率的替代。`heldout_results.png/.pdf`及`training_progress.png`可单独查看。所有未选中的实验和失败日志仍保留在本目录。

结束后没有根据独立集继续训练。下一轮需优先缩小弹簧力/滞回、惯量、摩擦、参考位和真实时序的不确定性，再用新的独立种子验收。当前T3/T4/T8实车标志均未因此变为通过。
'''
    (root/'README.md').write_text(text)
    p=workspace/'progress.md';content=p.read_text()
    content=content.replace('## 2026-11-17 新一轮实测数据训练（进行中）','## 2026-11-17 新一轮实测数据训练（已结束；未通过独立验收）',1)
    headline='''
**最终状态：本轮离线训练、冻结评估和归档已完成，模型未通过整体验收。** 独立CPU：近站姿候选在近站姿7/8，完整姿态候选在完整粗测姿态2/8；18个参数敏感性回合0/18。全部66回合保存并复算，标签/哈希一致。没有访问硬件、改实车标定、注册或烧录模型。详细见[最终报告](ref_document/fifty_mujoco/runs/realdata_20261117/final_summary.json)、[独立审计](ref_document/fifty_mujoco/runs/realdata_20261117/heldout_audit.json)、[结果图](ref_document/fifty_mujoco/runs/realdata_20261117/heldout_results.png)。

'''
    heading='## 2026-11-17 新一轮实测数据训练（已结束；未通过独立验收）\n'
    content=content.replace(heading,heading+'\n'+headline,1)
    note=f'''
### {now:%H:%M} 最终独立结果与收尾

- 本轮01:53开始，{now:%H:%M}结束，约{summary['elapsed_minutes']:.1f}分钟，未用尽300分钟上限。最后PPO于05:57前后停止；05:58:01冻结两类权重及评估源码，此后未根据独立结果调参。记录的21组PPO末计数合计69,869,166个有效学习转移（不同课程/并行实验相加，监督样本另计）。
- 独立名义结果：近站姿候选在近/中间/完整姿态为7/8、1/8、0/8；完整候选为0/8、2/8、2/8。18个条件敏感性回合全部失败；每个失败和提前终止回合均保留。66/66回合完成，逐帧条件与标签一致，冻结权重/配置/种子/轨迹哈希检查无问题。
- 最终近站姿权重SHA `2b3d09178292e1ba7a624c1f2df2966c9cbef40ceac31cc742ded173a1864f7c`；完整姿态权重SHA `8c85f761099d1f2b3c6b521046a53cf05b2cb6fb857c6a02f73dcec12b5f4dac`。见`frozen_candidates/`及`final_selection.json`。开发6/8不是独立通过，单个视频成功也不能替代总体成功率。
- 51项回归检查通过。四套隔离起身/平衡C导出各1024组主机对拍通过；仅推理一致，不是硬件验证。最终完整姿态权重的连续成功开发回放为`media/frozen_full_u20_development.*`，3.69 s起身、0.3 s交接、10 s平衡，逐帧审计通过。
- 物理缺口仍在：真实BG450-080压力/轴力/滞回未独立测得，450 N/80 mm/250 mm自由销距及折算惯量、摩擦只是候选；粗测与编码器参考位矛盾未解决，真实IMU/CAN时序未覆盖。因此没有标记T3/T4/T8实车通过，也没有访问板卡或改正常固件。
- 冻结后不再用这些独立种子继续拟合。下一轮先补实际被动力学、参考位和时序，再逐级扩展鲁棒起身课程，并保留新的独立验收集。全部文件与复现说明见`runs/realdata_20261117/README.md`；结果包见同目录`review_bundle.zip`。
'''
    content=content.replace('<!-- REALDATA_TRAINING_20261117_END -->',note+'\n<!-- REALDATA_TRAINING_20261117_END -->',1);p.write_text(content)
    p=workspace/'progress_real_data.md';content=p.read_text()
    content=content.replace('## 2026-11-17 离线训练使用状态（04:51）',f'## 2026-11-17 离线训练使用状态（已结束，{now:%H:%M}）',1)
    old='CPU开发回放首次出现完整粗测姿态的有支撑起身、完全撤支撑、交接和10 s平衡成功回合（教师方案1/2）；标准网络还在训练，独立验收未执行。T8实车起身仍未执行。持续状态以`progress.md`本轮章节为准。'
    new='标准网络已出现完整粗测姿态起身、完全撤支撑及10 s平衡成功回合。冻结后独立CPU结果：近站姿候选近站姿7/8，完整姿态候选完整粗测姿态2/8；18个假设参数敏感性回合全部失败。66回合逐帧复算与标签一致，但整体未通过。训练与评估已结束，T3/T4仍缺独立真值，T8实车起身仍未执行。详见`progress.md`和`ref_document/fifty_mujoco/runs/realdata_20261117/final_summary.json`。'
    content=content.replace(old,new,1);p.write_text(content)
    important=['README.md','final_summary.json','latest_status.json','training_progress.png','heldout_results.png','heldout_results.pdf',
        'final_selection.json','acceptance_plan.json','heldout_spec.json','heldout_audit.json','input_manifest.json','step_audit.json',
        'evidence_and_spring_calculation.json','spring_candidate_curve.csv','spring_geometry_evidence.png','regression_final_before_freeze.log']
    directories=['frozen_candidates','heldout_cpu','final_export_near','final_export_full','final_export_balance_near','final_export_balance_full','media']
    bundle=root/'review_bundle.zip'
    with zipfile.ZipFile(bundle,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=5) as archive:
        for name in important:
            if (root/name).exists():archive.write(root/name,name)
        for directory in directories:
            for file in sorted((root/directory).rglob('*')):
                if file.is_file():archive.write(file,str(file.relative_to(root)))
        archive.write(workspace/'progress.md','progress.md');archive.write(workspace/'progress_real_data.md','progress_real_data.md')
    hashes={str(file.relative_to(root)):digest(file) for file in root.rglob('*') if file.is_file() and file.name not in ('final_artifact_manifest.json','finalize.log')}
    (root/'final_artifact_manifest.json').write_text(json.dumps({'created_at':now.isoformat(),'files':hashes,
        'project_progress_sha256':digest(workspace/'progress.md'),'real_data_progress_sha256':digest(workspace/'progress_real_data.md'),
        'review_bundle_sha256':digest(bundle),'all_training_stopped':True},indent=2))
    print(json.dumps({'finished_at':summary['finished_at'],'elapsed_minutes':summary['elapsed_minutes'],
        'qualified':False,'review_bundle_bytes':bundle.stat().st_size,'review_bundle_sha256':digest(bundle)}),flush=True)


if __name__=='__main__':main()
