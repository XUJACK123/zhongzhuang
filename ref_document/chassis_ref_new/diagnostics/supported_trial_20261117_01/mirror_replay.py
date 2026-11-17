import ctypes as C
import json
import math
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
lib = C.CDLL(str(ROOT/'build/host/rl_host.dll'))
A = C.c_float*6
P = C.POINTER(C.c_float)
lib.rl_joint_to_motor.argtypes = [P,P,P,P,P]
lib.rl_joint_to_motor.restype = C.c_bool
lib.motor_encode.argtypes = [P,C.POINTER(C.c_uint8),C.POINTER(C.c_uint8)]
lib.motor_encode.restype = C.c_bool
q=A(3.1,-3.4,0,-3.1,3.4,0)
tau=A(4,-6,1,-4,6,1)
qd=A(.1,-.2,.3,-.1,.2,.3)
motor,speed=A(),A()
assert lib.rl_joint_to_motor(q,qd,tau,motor,speed)
assert all(abs(motor[i]-motor[i+3])<1e-6 for i in range(3))
assert abs(sum(tau[i]*qd[i] for i in range(6))-sum(motor[i]*speed[i] for i in range(6)))<1e-5
DM,DJI=(C.c_uint8*8)(),(C.c_uint8*8)()
assert lib.motor_encode(motor,DM,DJI)
dm=bytes(DM); dji=bytes(DJI)
raw=[int.from_bytes(dm[i:i+2],'little',signed=True) for i in range(0,8,2)]
wheel=[int.from_bytes(dji[i:i+2],'big',signed=True) for i in range(0,8,2)]
assert raw[3]==-raw[0] and raw[2]==raw[1]
assert wheel[2]==-wheel[1]
result={'test':'mirrored physical generalized torque through actual C mapping and CAN packing','joint_torque':list(tau),'canonical_motor_torque':list(motor),'dm_slots_ID1_to_ID4':raw,'dji_slots_ID1_to_ID4':wheel,'power_consistency':'pass','mirror_mapping':'pass','physical_installation_reverified':False}
(Path(__file__).parent/'mirror_replay.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
