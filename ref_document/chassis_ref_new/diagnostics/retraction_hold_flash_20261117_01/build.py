"""Root Makefile isolated suspended retraction build. Never accesses hardware."""
from pathlib import Path
import hashlib,json,re,shutil,subprocess
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
BUILD=ROOT/'build/retraction'
BUILD.mkdir(parents=True,exist_ok=True)
base=['gcc','-std=c11','-O2','-Wall','-Wextra','-Werror','-Itests/fake','-Iuser','-Iuser/algorithm']
common=['user/algorithm/rl.c','user/algorithm/model/model.c','user/algorithm/model/getup_model.c',str(HERE/'controller.c')]
with (HERE/'host_test.log').open('w') as log:
    for test,src in [('controller',common),('application',common+['user/motor_io.c','user/remote_control.c','user/imu_filter.c','user/stand_entry.c','user/supported_trial.c','user/telemetry.c','user/trial_trace.c',str(HERE/'application.c')])]:
        exe=BUILD/('test_'+test+'.exe')
        subprocess.run(base+src+[str(HERE/('test_'+test+'.c')),'-lm','-o',str(exe)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run([str(exe)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
block=re.search(r'^C_SOURCES := \\\n(.*?)\n\n',(ROOT/'Makefile').read_text(),re.M|re.S).group(1)
src=block.replace('\\\n',' ').split()
src=[s for s in src if s!='user/chassis.c']+['real_data_collection/retraction/application.c','real_data_collection/retraction/controller.c']
with (HERE/'arm_build.log').open('w') as log:
    subprocess.run([shutil.which('mingw32-make') or shutil.which('make'),'-j4','TARGET=suspended_retraction','BUILD_DIR=build/retraction','C_SOURCES='+' '.join(src)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
elf=BUILD/'suspended_retraction.elf'
header=(HERE/'controller.h').read_text()
def number(name):return float(re.search(r'^#define '+name+r'\s+(\S+)',header,re.M).group(1).rstrip('Uf'))
report={'kind':'independent suspended bilateral knee fold and hold; no actor','control_version':re.search(r'#define RETRACT_CONTROL_VERSION (\S+)',header).group(1).rstrip('U'),'elf':str(elf),'bin_sha256':hashlib.sha256(elf.with_suffix('.bin').read_bytes()).hexdigest(),'target_delta_deg':number('RETRACT_DELTA_RAD')*180/3.141592653589793,'active_legs':['right','left'],'ramp_ms':int(number('RETRACT_RAMP_MS')),'hold_ms':int(number('RETRACT_HOLD_MS')),'release_ms':int(number('RETRACT_RELEASE_MS')),'leg_motor_cap_nm':number('RETRACT_CAP_NM'),'wheel_torque_nm':0,'record_interval_ms':int(number('RETRACT_RECORD_MS')),'flashed':False,'physical_result':None,'sources':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.suffix in ('.c','.h','.py')}}
report['total_active_ms']=report['ramp_ms']+report['hold_ms']+report['release_ms']
assert report['total_active_ms']+400<512*report['record_interval_ms'],'Trace cannot cover full test'
(HERE/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
