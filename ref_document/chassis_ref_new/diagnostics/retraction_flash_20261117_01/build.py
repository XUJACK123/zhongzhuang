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
report={'kind':'independent suspended left-knee position test; no actor','control_version':'0xD2611171','elf':str(elf),'bin_sha256':hashlib.sha256(elf.with_suffix('.bin').read_bytes()).hexdigest(),'target_delta_deg':5,'ramp_ms':1500,'hold_ms':1000,'release_ms':1000,'leg_motor_cap_nm':20,'wheel_and_right_leg_torque_nm':0,'record_interval_ms':10,'flashed':False,'physical_result':None,'sources':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in HERE.iterdir() if p.suffix in ('.c','.h','.py')}}
(HERE/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(report,indent=2))
