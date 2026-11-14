#!/usr/bin/env python3
"""Real actor numeric parity, observation contract and portable control tests.

Does not use a debugger, access motors, or require Torch/MuJoCo/CUDA.
The numerical reference is scalar float64 math, not the original training runtime.
"""
from __future__ import annotations
import ctypes as C
import importlib.util
import json
import math
import os
from pathlib import Path
import random
import re
import shutil
import subprocess
import struct
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from export_policy import SAMPLE, load_actor, reference, sha
from calibration_values import measured_pose

ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT/"build/host"
class Network(C.Structure):
    _fields_ = [("h1",C.c_float*128),("h2",C.c_float*64),("h3",C.c_float*32)]
class Measurement(C.Structure):
    _fields_ = [("q",C.c_float*6),("qd",C.c_float*6),("omega",C.c_float*3),
                ("gravity",C.c_float*3),("xy",C.c_float*2),("yaw",C.c_float),("velocity",C.c_float*2)]
class Controller(C.Structure):
    _fields_ = [("network",Network),("obs",C.c_float*31),("action",C.c_float*6),
                ("command",C.c_float*3),("reference_xy",C.c_float*2),("reference_yaw",C.c_float),
                ("target_q",C.c_float*6),("target_qd",C.c_float*6),
                ("joint_torque",C.c_float*6),("motor_torque",C.c_float*6)]

def main():
    measured=measured_pose(150,100,150,100)
    assert max(abs(math.degrees(x)-y) for x,y in zip(measured,(150,-170,0,-150,170,0)))<1e-10
    try: measured_pose(150,0,150,100)
    except ValueError: pass
    else: raise AssertionError('Invalid included knee angle accepted')
    BUILD.mkdir(parents=True,exist_ok=True)
    cc=os.environ.get("HOST_CC") or shutil.which("gcc")
    if not cc: raise SystemExit("HOST_CC or native gcc is required")
    library=BUILD/("rl_host.dll" if os.name=="nt" else "rl_host.so")
    sources=["user/algorithm/rl.c","user/algorithm/model/model.c","user/control_state.c",
             "user/motor_io.c","user/remote_control.c","user/state_estimator.c","tests/test_logic.c"]
    cmd=[cc,"-std=c11","-O2","-Wall","-Wextra","-Wshadow","-Werror","-shared","-Iuser",
         "-Iuser/algorithm",*sources,"-o",str(library),"-lm"]
    if os.name!="nt": cmd.insert(1,"-fPIC")
    subprocess.run(cmd,cwd=ROOT,check=True)
    lib=C.CDLL(str(library))
    failed=lib.test_logic()
    if failed: raise AssertionError(f"C control/protocol test failed at tests/test_logic.c:{failed}")
    fake_sources=sources[:-1]+["user/chassis.c","tests/test_chassis.c"]
    integration=[]
    for active in (False,True):
        suffix="enabled_fake" if active else "default_fake"
        fake_library=BUILD/(suffix+(".dll" if os.name=="nt" else ".so"))
        # Both variants get their own fixture so the test never depends on the
        # production switch or on real calibration values.
        config=(ROOT/"user/robot_config.h").read_text()
        extra=[]
        if active:
            for flag in ("ENABLE_OUTPUT","MOTOR_CALIBRATED","IMU_FRAME_VERIFIED","POLICY_CONTRACT_VERIFIED","ODOMETRY_VERIFIED"):
                config=re.sub(rf"^#define ROBOT_{flag} 0$",f"#define ROBOT_{flag} 1",config,flags=re.M)
            default=json.loads((ROOT/"user/algorithm/model/manifest.json").read_text())["default_q"]
            arm=list(default)
            for h,sign in ((0,1),(3,-1)):
                beta=sign*default[h+1]+math.pi/2
                arm[h]=sign*default[h]; arm[h+1]=arm[h]+2*math.atan2(math.sin(beta),113.4/135+math.cos(beta))
            # Fake feedback reports raw zero, so map zero counts to the default pose
            # instead of inheriting whatever real calibration robot_config.h holds.
            config=re.sub(r"^#define ROBOT_ENCODER_SIGNS \{.*\}$","#define ROBOT_ENCODER_SIGNS {1, 1, 1, 1, 1, 1}",config,flags=re.M)
            config=re.sub(r"^#define ROBOT_ENCODER_REFERENCE_RAD \{.*\}$","#define ROBOT_ENCODER_REFERENCE_RAD {0, 0, 0, 0, 0, 0}",config,flags=re.M)
            replacement="#define ROBOT_ARM_REFERENCE_RAD {"+",".join(float(v).hex()+"f" for v in arm)+"}"
            config=re.sub(r"^#define ROBOT_ARM_REFERENCE_RAD \{.*\}$",replacement,config,flags=re.M)
        else:
            config=re.sub(r"^#define ROBOT_ENABLE_OUTPUT 1$","#define ROBOT_ENABLE_OUTPUT 0",config,flags=re.M)
        fixture=BUILD/("fake_calibration.h" if active else "fake_disabled.h"); fixture.write_text(config)
        extra=["-include",str(fixture)]
        fake_cmd=[cc,"-std=c11","-O2","-Wall","-Wextra","-Wshadow","-Werror","-shared",
                  "-Itests/fake","-Iuser","-Iuser/algorithm",*extra,*fake_sources,"-o",str(fake_library),"-lm"]
        if os.name!="nt": fake_cmd.insert(1,"-fPIC")
        subprocess.run(fake_cmd,cwd=ROOT,check=True)
        fake=C.CDLL(str(fake_library)); result=fake.test_chassis()
        if result: raise AssertionError(f"{suffix} integration failed at tests/test_chassis.c:{result}")
        integration.append(suffix+" passed")
    fp=C.POINTER(C.c_float)
    lib.rl_actor.argtypes=[C.POINTER(Network),fp,fp]; lib.rl_actor.restype=C.c_bool
    lib.rl_reset.argtypes=[C.POINTER(Controller),C.POINTER(Measurement)]
    lib.rl_policy_step.argtypes=[C.POINTER(Controller),C.POINTER(Measurement),fp,C.c_float]
    lib.rl_policy_step.restype=C.c_bool
    actor=load_actor(SAMPLE/"model_3050.pt")
    words=0
    for i,layer in enumerate(actor):
        for kind,values in zip(("w","b"),layer):
            compiled=(C.c_float*len(values)).in_dll(lib,f"rl_{kind}{i}")
            assert bytes(compiled)==struct.pack(f"<{len(values)}f",*values)
            words+=len(values)
    manifest=json.loads((ROOT/"user/algorithm/model/manifest.json").read_text())
    assert manifest["sha256"]==sha(SAMPLE/"model_3050.pt")
    for name,digest in manifest["generated_sha256"].items():
        assert sha(ROOT/"user/algorithm/model"/name)==digest
    rng=random.Random(20261113); net=Network(); worst=0
    vectors=[[0]*31]+[[rng.uniform(-3,3) for _ in range(31)] for _ in range(256)]
    # Include large but valid clipped observations as a numeric stress test.
    vectors += [[rng.uniform(-100,100) for _ in range(31)] for _ in range(16)]
    for values in vectors:
        obs=(C.c_float*31)(*values); out=(C.c_float*6)()
        assert lib.rl_actor(C.byref(net),obs,out)
        expected=reference(actor,list(obs))
        error=max(abs(a-b) for a,b in zip(out,expected)); worst=max(worst,error)
        for a,b in zip(out,expected): assert abs(a-b)<=0.001+0.00003*abs(b),(a,b)
    bad=(C.c_float*31)(*([0]*31)); bad[2]=float("nan"); out=(C.c_float*6)()
    assert not lib.rl_actor(C.byref(net),bad,out) and list(out)==[0]*6

    # Independent archived mathematical contract, compared on nonzero states.
    contract_path=ROOT.parent/"fifty_mujoco/src/fifty_mujoco/contracts/sample_actor31.py"
    spec=importlib.util.spec_from_file_location("contract",contract_path)
    contract=importlib.util.module_from_spec(spec); spec.loader.exec_module(contract)
    max_obs_error=0
    for _ in range(100):
        m=Measurement(); m.q[:]=[a+rng.uniform(-.1,.1) for a in contract.SOURCE_DEFAULT_ANGLES]
        m.qd[:]=[rng.uniform(-2,2) for _ in range(6)]; m.omega[:]=[.1,-.2,.3]; m.gravity[:]=[.02,-.03,-.999]
        m.xy[:]=[.3,-.2]; m.yaw=.4; m.velocity[:]=[.2,-.05]
        c=Controller(); lib.rl_reset(C.byref(c),C.byref(m))
        c.reference_xy[:]=[.4,-.1]; c.reference_yaw=.6
        c.action[:]=[.2,-.3,.8,-.4,.5,-.9]
        old=list(c.action); ref_xy=list(c.reference_xy); ref_yaw=c.reference_yaw
        request=[.3,1,.1]; dt=C.c_float(.01).value
        applied=contract.ramp_commands(list(c.command),request,dt)
        expected=contract.build_observation(angular_velocity_body=list(m.omega),projected_gravity_body=list(m.gravity),
            applied_commands=applied,joint_position=list(m.q),joint_velocity=list(m.qd),
            previous_clipped_action=old,position_xy=list(m.xy),reference_xy=ref_xy,yaw=m.yaw,
            reference_yaw=ref_yaw,body_velocity_xy=list(m.velocity))
        assert lib.rl_policy_step(C.byref(c),C.byref(m),(C.c_float*3)(*request),dt)
        error=max(abs(a-b) for a,b in zip(c.obs,expected)); max_obs_error=max(max_obs_error,error)
        assert error<3e-6,error
        ref_next,yaw_next=contract.advance_reference(ref_xy,ref_yaw,applied[0],applied[1],dt)
        assert max(abs(a-b) for a,b in zip(c.reference_xy,ref_next))<1e-6
        assert abs(c.reference_yaw-yaw_next)<1e-6
        targets=contract.action_targets(list(c.action),[[-4.296127953784042,-2.5592683579936732],[2.559268357993676,4.296127953784042]])
        assert max(abs(a-b) for a,b in zip(c.target_q,targets["position"]))<1e-6
    report={"status":"pass","weight_float32_words_bit_exact":words,
            "actor_vectors":len(vectors),"actor_max_absolute_error":worst,
            "observation_vectors":100,"observation_max_absolute_error":max_obs_error,
            "protocol_and_control_assertions":"passed", "physical_pose_conversion":"passed", "torch_forward_executed":False,
            "fake_hal_control_loop":integration,
            "arm_runtime_tested":False,"hardware_tested":False,"checkpoint_sha256":manifest["sha256"]}
    (BUILD/"test_report.json").write_text(json.dumps(report,indent=2)+"\n")
    print(json.dumps(report,indent=2))

if __name__=="__main__": main()
