"""Build an isolated diagnostic with the root authoritative Makefile; never flash."""
from pathlib import Path
import hashlib,json,re,shutil,subprocess
import math
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
BUILD=HERE/'build'
BUILD.mkdir(exist_ok=True)
ARM=ROOT/'build/identification'
sources=['real_data_collection/identification/limited_current.c','real_data_collection/identification/test_controller.c']
with (HERE/'host_test.log').open('w',encoding='utf-8') as f:
    subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror',*sources,'-lm','-o',str(BUILD/'test_controller.exe')],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
    subprocess.run([str(BUILD/'test_controller.exe')],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
    subprocess.run(['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','real_data_collection/identification/test_tx_age.c','-o',str(BUILD/'test_tx_age.exe')],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
    subprocess.run([str(BUILD/'test_tx_age.exe')],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
text=(ROOT/'Makefile').read_text()
block=re.search(r'^C_SOURCES := \\\n(.*?)\n\n',text,re.M|re.S).group(1)
src=block.replace('\\\n',' ').split()
src=[s for s in src if s not in ('Core/Src/main.c','Core/Src/fdcan.c','user/chassis.c')]
src+=['real_data_collection/identification/entry.c','real_data_collection/identification/limited_current.c','real_data_collection/identification/bench_fdcan.c']
make=shutil.which('mingw32-make') or shutil.which('make')
argv=[make,'-j4','TARGET=limited_current_identification','BUILD_DIR=build/identification','C_SOURCES='+' '.join(src)]
with (HERE/'arm_build.log').open('w',encoding='utf-8') as f:
    subprocess.run(argv,cwd=ROOT,stdout=f,stderr=subprocess.STDOUT,check=True)
elf=ARM/'limited_current_identification.elf'
binfile=elf.with_suffix('.bin')
header=(HERE/'limited_current.h').read_text()
config=(ROOT/'user/robot_config.h').read_text()
def number(name,text=header):
    value=re.search(r'^#define '+name+r'\s+([^\s/]+)',text,re.M).group(1)
    return float(value.strip('()').rstrip('fU'))
levels=[float(v.strip().rstrip('f')) for v in re.search(r'#define IDENT_CURRENT_LEVELS_A\s*\{([^}]+)\}',header).group(1).split(',')]
cap=number('IDENT_CURRENT_LIMIT_A');kt=number('ROBOT_DM_KT_NM_A',config)
assert len(levels)==int(number('IDENT_TRIAL_COUNT')) and all(math.isfinite(v) and 0<v<=cap for v in levels)
assert cap*kt<=number('ROBOT_LEG_TORQUE_CAP_NM',config)
selected=int(number('IDENT_SELECTED_INDEX'))
assert selected in (0,1,3,4), 'Diagnostic supports DM motors only'
labels={0:'right hip DM4',1:'right knee DM3',3:'left hip DM1',4:'left knee DM2'}
manifest={'image_kind':'isolated limited-current diagnostic; not RL policy','root_makefile_authoritative':True,
          'elf':str(elf),'elf_sha256':hashlib.sha256(elf.read_bytes()).hexdigest(),
          'bin_sha256':hashlib.sha256(binfile.read_bytes()).hexdigest(),
          'selected':labels[selected],'selected_policy_index':selected,'current_limit_a':cap,'current_levels_a':levels,'canonical_current_direction':number('IDENT_CURRENT_DIRECTION'),'nominal_torque_limit_nm':cap*kt,
          'active_duration_limit_ms':number('IDENT_RUN_MS'),'inter_trial_zero_ms':number('IDENT_COOLDOWN_MS'),'output_arm_delta_limit_deg':number('IDENT_MAX_DELTA_RAD')*180/math.pi,'velocity_limit_rad_s':number('IDENT_MAX_SPEED_RAD_S'),
          'passive_wheel_delta_limit_deg':number('IDENT_WHEEL_MAX_DELTA_RAD')*180/math.pi,'passive_wheel_speed_limit_rad_s':number('IDENT_WHEEL_MAX_SPEED_RAD_S'),
          'control_ms':2,'active_log_every_ms':6,'cooldown_log_first_ms':32,'cooldown_logged_at_boundaries_only':False,
          'cooldown_output_zero_keep_enabled':True,
          'hardware_tx_events_recorded':True,'hardware_timestamp_unit_us':16,'foreign_command_listen_only':True,
          'auto_retransmission_enabled':True,'abort_pending_nonzero_after_ms':4,
          'auto_start':False,'requires_software_trigger_and_S1_run':True,'software_trigger_delay_ms':2000,
          'flashed':False,'physical_response_verified':False,'independently_measured_shaft_torque':False,
          'source_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.suffix in ('.c','.h')}}
manifest['source_sha256']['can_tx_age.h']=hashlib.sha256((ROOT/'user/can_tx_age.h').read_bytes()).hexdigest()
(HERE/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
print(json.dumps(manifest,indent=2))
