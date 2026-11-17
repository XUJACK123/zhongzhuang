"""Root Makefile isolated raw-current probe build. Never accesses hardware."""
from pathlib import Path
import hashlib,json,re,shutil,subprocess
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
BUILD=ROOT/'build/current_probe'
BUILD.mkdir(parents=True,exist_ok=True)
base=['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','-Itests/fake','-Iuser','-Iuser/algorithm']
common=['user/algorithm/rl.c','user/algorithm/model/model.c','user/algorithm/model/getup_model.c',str(HERE/'controller.c'),str(HERE/'measurement.c')]
with (HERE/'host_test.log').open('w') as log:
    for test,src in [('controller',common+['user/motor_io.c']),('application',common+['user/motor_io.c','user/remote_control.c','user/imu_filter.c','user/stand_entry.c','user/supported_trial.c','user/telemetry.c','user/trial_trace.c',str(HERE/'application.c')])]:
        exe=BUILD/('test_'+test+'.exe')
        subprocess.run(base+src+[str(HERE/('test_'+test+'.c')),'-lm','-o',str(exe)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run([str(exe)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
block=re.search(r'^C_SOURCES := \\\n(.*?)\n\n',(ROOT/'Makefile').read_text(),re.M|re.S).group(1)
src=block.replace('\\\n',' ').split()
src=[s for s in src if s!='user/chassis.c']+['real_data_collection/current_probe/application.c','real_data_collection/current_probe/controller.c','real_data_collection/current_probe/measurement.c']
with (HERE/'arm_build.log').open('w') as log:
    subprocess.run([shutil.which('mingw32-make') or shutil.which('make'),'-j4','TARGET=current_probe','BUILD_DIR=build/current_probe','C_SOURCES='+' '.join(src)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
elf=BUILD/'current_probe.elf'
header=(HERE/'controller.h').read_text()
def number(name):return float(re.search(r'^#define '+name+r'\s+(\S+)',header,re.M).group(1).rstrip('Uf'))
report={'kind':'single DM raw-current ramp; no actor or joint-space feedback',
'control_version':'0xC2611171','elf':str(elf),'bin_sha256':hashlib.sha256(elf.with_suffix('.bin').read_bytes()).hexdigest(),
'ramp_ms':1000,'hold_ms':500,'release_ms':500,'total_active_ms':2000,'record_interval_ms':10,
'raw_nominal_cap_nm':3,'leg_travel_limit_deg':2,'leg_speed_limit_rad_s':0.15,
'wheel_torque_nm':0,'flashed':False,'physical_result':None,
'q_semantics':'raw-positive output shaft angle radians; NOT generalized joint angles',
'force_truth':'command and feedback current only; Nm uses nominal Kt, not measured shaft torque',
'sources':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.suffix in ('.c','.h','.py')}}
assert report['total_active_ms']+400<512*report['record_interval_ms']
(HERE/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
