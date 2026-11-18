import json, math, pathlib, numpy as np, mujoco
from fifty_mujoco.envs.sample_env31_realdata import RealDataHardwareEnv31
r=pathlib.Path('runs/getup_seated_20261118_035341')
e=RealDataHardwareEnv31(config=json.loads((r/'full.json').read_text()))
m,d=e.model,e.data
wheel=[m.geom(n).id for n in ('right_wheel_collision_0','left_wheel_collision_0')]
base=m.geom('base_link_collision_0').id
floor=m.geom('floor').id
allowed=set(wheel+[base])
def geom_gap(g):
    rot=d.geom_xmat[g].reshape(3,3)
    size=m.geom_size[g]
    extent=(abs(rot[2])@size if g==base else size[0]*math.sqrt(max(0,1-rot[2,2]**2))+size[1]*abs(rot[2,2]))
    return float(d.geom_xpos[g,2]-extent)
def evaluate(h,k,p):
    mujoco.mj_resetData(m,d)
    d.qpos[3:7]=[math.cos(p/2),0,math.sin(p/2),0]
    d.qpos[e.source.qpos_ids]=[h,-k,0,-h,k,0]
    mujoco.mj_forward(m,d)
    gap=[geom_gap(g) for g in wheel+[base]]
    if max(gap)-min(gap)>.009: return None
    d.qpos[2]-=min(gap)
    mujoco.mj_forward(m,d)
    initial=d.qpos.copy()
    lowest=0.;max_vel=0.
    for tick in range(1000):
        if tick%2==0:
            q=d.qpos[e.source.qpos_ids]
            if not (np.all(q[[1,4]]>=e.limits[:,0]-1e-4) and np.all(q[[1,4]]<=e.limits[:,1]+1e-4)):
                return {'failure':'knee_limit','time_s':float(d.time),'initial':initial.tolist(),'qpos':d.qpos.tolist()}
            try: t,_=e._spring_torque(q,d.qvel[e.source.qvel_ids])
            except ValueError: return {'failure':'spring_stroke','time_s':float(d.time),'initial':initial.tolist()}
            d.qfrc_applied[e.source.qvel_ids]=t
        mujoco.mj_step(m,d,nstep=2)
        for c in d.contact:
            a,b=c.geom1,c.geom2
            lowest=min(lowest,float(c.dist))
            if not (floor in (a,b) and (b if a==floor else a) in allowed):
                return {'failure':'forbidden_contact','geoms':[m.geom(a).name,m.geom(b).name],'time_s':float(d.time),'initial':initial.tolist()}
        max_vel=max(max_vel,float(np.max(np.abs(d.qvel))))
    mujoco.mj_forward(m,d)
    loads={m.geom(g).name:0. for g in allowed}
    for i,c in enumerate(d.contact):
        if floor in (c.geom1,c.geom2):
            g=c.geom2 if c.geom1==floor else c.geom1
            if g in allowed:
                f=np.zeros(6);mujoco.mj_contactForce(m,d,i,f);loads[m.geom(g).name]+=float(f[0])
    x=(d.geom_xpos[wheel,0]-d.qpos[0]).tolist()
    return {'failure':None if min(loads.values())>=4.3 and max(x)<-.01 and np.max(np.abs(d.qvel))<.2 else 'target_shape_or_rest_lost','initial':initial.tolist(),'qpos':d.qpos.tolist(),'qvel':d.qvel.tolist(),'loads_n':loads,'wheel_offset_x_m':x,'lowest_contact_distance_m':lowest,'peak_speed':max_vel}
rows=[]
for p in np.radians([0,5,10,15,20,-5,-10,-15,-20]):
    for k in np.linspace(2.60,3.68,19):
        for h in np.linspace(2.85,4.1,63):
            row=evaluate(h,k,p)
            if row is not None: rows.append(row)
(r/'seated_scan.json').write_text(json.dumps(rows,indent=2))
from collections import Counter
print(Counter(row['failure'] for row in rows))
print([r for r in rows if r['failure'] is None][:2])
