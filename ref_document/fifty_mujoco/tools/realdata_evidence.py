#!/usr/bin/env python3
"""Reproducible data lineage and conditional spring/geometry calculations."""
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT/'src'))
from fifty_mujoco.envs.sample_env31_realdata import spring_force
from fifty_mujoco.mechanism.loader import load_mechanism
from fifty_mujoco.mechanism.spring import load_spring_link


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    out=PROJECT/'runs/realdata_20261117';data=PROJECT.parent/'chassis_ref_new/real_data_collection'
    cfg=json.loads((PROJECT/'configs/realdata_350_20261117.json').read_text())
    wheel=json.loads((data/'field_measurements_20261116_205507/marked_wheel_result.json').read_text())
    manual=json.loads((data/'field_measurements_20261116_205507/manual_angle_reference.json').read_text())
    spring_reference=json.loads((data/'identification/manufacturer_bg450/spring_curve_reference.json').read_text())
    link=load_spring_link(load_mechanism(PROJECT/'configs/mechanism.yaml'))
    step=PROJECT.parents[2]/'26英雄整车总装2.0.STEP'
    # The explicitly requested STEP lives next to the workspace directory.
    if not step.exists():step=PROJECT.parents[1].parent/'26英雄整车总装2.0.STEP'
    source_paths=[data/'physical_measurements_priority_20261116.json',
                  data/'field_measurements_20261116_205507/marked_wheel_result.json',
                  data/'field_measurements_20261116_205507/manual_angle_reference.json',
                  data/'cad_reference_for_field_measurements_20261116.json',
                  data/'identification/manufacturer_bg450/spring_curve_reference.json']
    pressure=[]
    for diameter in (.006,.010):
        area=math.pi*diameter**2/4
        pressure.append({'assumed_effective_diameter_m':diameter,'area_m2':area,
                         'force_n_at_9_12_15_MPa':[p*1e6*area for p in (9,12,15)],
                         'pressure_bar_for_450N':450/area/1e5})
    pin_from_coarse=link.pin_distance_mm(math.radians(180-manual['right_knee_included_deg_approx']))
    report={'schema':'realdata_training_evidence_v1','evidence_kind':'derived_and_provisional_not_new_measurement',
            'input_sha256':{str(p.relative_to(PROJECT.parent)):sha(p) for p in source_paths},
            'step':{'path':str(step),'bytes':step.stat().st_size,'sha256':sha(step),'evidence':'cad_extracted'},
            'accepted_mass_kg':cfg['mass_kg'],'mass_source':'operator accepted existing 21.9 kg record; no fresh weighing',
            'wheel':{'effective_radius_m':wheel['effective_radius_m_from_manual_travel_and_turns'],
                     'equation':'700 mm / (2 marked turns * 2*pi)','left_is_symmetry_assumption':True,
                     'geometric_outer_diameter_m':.115,'ratio_15_88_not_verified_by_lost_encoder_trial':True},
            'geometry':{'thigh_m':.21,'shin_m':.25,'pin_offset_m':.011,
                        'cad_pin_at_assembly_m':link.pin_distance_mm()/1000,
                        'pin_m_from_coarse_120deg_included':pin_from_coarse/1000,
                        'reported_pin_m':.24,'residual_m':(.24-pin_from_coarse/1000),
                        'included_knee_deg_implied_by_240mm_CAD_pins':180-math.degrees(link.delta_for_pin(240)),
                        'geometry_not_refit_to_inconsistent_coarse_readings':True},
            'pressure_calculation':{'equation':'F = gauge_pressure * pi * effective_diameter^2 / 4',
                                    'rows':pressure,'actual_pressure_MPa':None,
                                    'diameter_10mm_measured_part_unverified':True,
                                    'not_an_independent_spring_force_measurement':True},
            'spring_candidate':cfg['spring'],
            'spring_model_equation':'F0 * (D/(D-(Lfree-L)))^n - Fseal*tanh(Ldot/vsmooth) - c*Ldot; push force clipped at zero',
            'dead_length_equation':'D = stroke / (1 - progression^(-1/n))',
            'manufacturer_source':spring_reference['manufacturer_product_page'],
            'exact_BG450_to_BKB_equivalence_verified':False,
            'manufacturer_journey_units_explicit':False,
            'hysteresis_status':'15 N seal friction and 20 N s/m damping are engineering sensitivity hypotheses, not measured loops',
            'rotor_inertia':cfg['reflected_inertia'],
            'standing_target':{'default_rad':cfg['standing_default_rad'],'root_height_m':.35,
                               'clearance_m':cfg['standing_clearance_m'],'evidence':'simulation IK design, not encoder calibration'},
            'missing_measurements':['actual spring pressure and exact-model force-stroke loops','independent motor torque',
                                    'actual stroke endpoints and precision zero references','full mass inertia tensor',
                                    'unassisted zero-output rest','continuous active timing and IMU dynamic truth'],
            'hardware_changed_or_accessed':False,'T3_or_T4_marked_measured_pass':False}
    (out/'evidence_and_spring_calculation.json').write_text(json.dumps(report,indent=2,ensure_ascii=False))
    lengths=np.linspace(.17,.25,161);s=cfg['spring'];rows=[]
    for length in lengths:
        rows.append({'pin_distance_m':float(length),'quasistatic_proxy_n':spring_force(float(length),0,s),
                     'compress_10mm_s_proxy_n':spring_force(float(length),-.01,s),
                     'return_10mm_s_proxy_n':spring_force(float(length),.01,s)})
    with (out/'spring_candidate_curve.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({'font.family':'Noto Sans CJK SC','axes.unicode_minus':False})
    fig,axes=plt.subplots(1,2,figsize=(12,4.5),layout='constrained')
    for key,label in [('quasistatic_proxy_n','准静态目录代理'),('compress_10mm_s_proxy_n','假设压缩 10 mm/s'),('return_10mm_s_proxy_n','假设回伸 10 mm/s')]:
        axes[0].plot(lengths*1000,[r[key] for r in rows],label=label)
    axes[0].set(xlabel='销心距 / mm',ylabel='估算轴向力 / N',title='BG450-080 候选模型：非实测力曲线')
    axes[0].legend(fontsize=8);axes[0].grid(alpha=.3)
    included=np.linspace(30,180,200)
    axes[1].plot(included,[link.pin_distance_mm(math.radians(180-a)) for a in included],label='STEP 安装点几何')
    axes[1].scatter([120],[pin_from_coarse],label='粗测膝内角 120° 的几何预测',zorder=4)
    axes[1].axhline(240,color='tab:red',ls='--',label='手工记录销距 240 mm')
    axes[1].set(xlabel='膝内角 / °',ylabel='销心距 / mm',title='保留约 14.3 mm 数据矛盾，不强行拟合')
    axes[1].legend(fontsize=8);axes[1].grid(alpha=.3)
    fig.savefig(out/'spring_geometry_evidence.png',dpi=180);plt.close(fig)
    print(json.dumps({'step_sha256':report['step']['sha256'],'coarse_pin_residual_m':report['geometry']['residual_m'],
                      'pressure_calculation':pressure,'report':str(out/'evidence_and_spring_calculation.json')},ensure_ascii=False),flush=True)


if __name__=='__main__':main()
