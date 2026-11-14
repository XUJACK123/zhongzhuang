#!/usr/bin/env python3
"""Step-by-step encoder commissioning: capture raw counts, diff them, write macros.

Never flashes, enables motors or changes commissioning flags. `capture` reads RAM
read-only; `apply` rewrites only the three calibration arrays in
user/robot_config.h. Counts are raw 0..8191 in policy order: right hip DM4,
right knee DM3, right wheel DJI3, left hip DM1, left knee DM2, left wheel DJI2.
"""
import argparse
import datetime
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
from struct import unpack_from
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
import monitor_can
from calibration_values import macros, measured_pose, define_line, arm_from_joint
ROOT=Path(__file__).resolve().parents[1]
CONFIG=ROOT/'user/robot_config.h'
NAMES=("ROBOT_ENCODER_SIGNS","ROBOT_ENCODER_REFERENCE_RAD","ROBOT_ARM_REFERENCE_RAD")

def now():
    return datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).isoformat()

def new_session():
    stamp=datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    path=ROOT/'diagnostics'/('commission_'+stamp)
    path.mkdir(parents=True,exist_ok=True)
    return path

def load_session(path=None):
    if path: return Path(path)
    found=sorted((ROOT/'diagnostics').glob('commission_*'),key=lambda p:p.name)
    return found[-1] if found else new_session()

def read_index(session):
    path=session/'commission.json'
    return json.loads(path.read_text(encoding='utf-8')) if path.exists() else {'created_at':now(),'snapshots':[]}

def save_index(session,index):
    (session/'commission.json').write_text(json.dumps(index,indent=2)+'\n',encoding='utf-8')

def print_counts(motors):
    print(f"{'idx':>3} {'motor':<20} {'raw':>6} {'total':>10} {'deg':>9} {'temp':>4} status")
    for i,m in enumerate(motors):
        deg=m['total']*360/8192
        print(f"{i:>3} {m['label']:<20} {m['raw']:>6} {m['total']:>10} {deg:>9.3f} {m['temp']:>4} 0x{m['status']:02X}")

def capture(a):
    session=load_session(a.session); index=read_index(session)
    target=session/'snapshots'/re.sub(r'[^0-9A-Za-z_.-]','_',a.label)
    report=monitor_can.snapshot(target,interval=None,openocd=a.openocd)
    snap=report['a']
    motors=[{'label':n['label'],'raw':n['encoder_counts'],'total':n['encoder_total_counts'],
             'temp':n['temperature_c'],'status':n['status_byte']} for n in snap['nodes']]
    index['snapshots']=[s for s in index['snapshots'] if s['label']!=a.label]
    index['snapshots'].append({'label':a.label,'at':now(),'dir':target.relative_to(ROOT).as_posix(),
        'motors':motors,'imu':{'yaw_rad':snap['imu']['yaw_rad'],'gravity_body':snap['imu']['gravity_body'],
        'calibrated':snap['imu']['calibrated']},'chassis':snap['chassis'],'remote':snap['remote']})
    save_index(session,index)
    print(f"Session: {session.relative_to(ROOT).as_posix()}")
    print(f"Label:   {a.label}")
    print_counts(motors)
    print(f"IMU yaw {snap['imu']['yaw_rad']:+.6f} rad gravity ({', '.join(f'{v:+.4f}' for v in snap['imu']['gravity_body'])})")
    return 0

def find(index,label):
    for s in index['snapshots']:
        if s['label']==label: return s
    raise SystemExit(f"no snapshot labelled {label!r}")

def watch(a):
    session=load_session(a.session)
    out=session/('watch_'+re.sub(r'[^0-9A-Za-z_.-]','_',a.label))
    out.mkdir(parents=True,exist_ok=True)
    elf=ROOT/'build/rl/wheel_leg_rl.elf'
    symbols=monitor_can.elf_symbols(elf,('g_motor_snapshot','g_imu','g_measurement','g_chassis','g_rl'))
    samples=max(1,int(a.seconds/a.interval))
    script=out/'watch.tcl'
    lines=['set dir {%s}'%out.as_posix(),'set n %d'%samples,
           'for {set k 0} {$k < $n} {incr k} {',
           '    if {$k > 0} { sleep %d }'%int(a.interval*1000)]
    for name,(address,size) in symbols.items():
        lines.append(f'    dump_image [format "%s/%03d_{name}.bin" $dir $k] {address:#x} {size}')
    lines+=['    catch {resume}','}','catch {resume}','shutdown']
    script.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    argv=monitor_can.openocd_argv(a.openocd,['init','targets stm32h723.cpu0'])+['-f',str(script)]
    proc=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,timeout=a.seconds+180)
    (out/'openocd.log').write_text(proc.stdout+proc.stderr,encoding='utf-8')
    missing=[k for k in range(samples) if not (out/f'{k:03d}_g_motor_snapshot.bin').exists()]
    if proc.returncode or missing:
        print((proc.stdout+proc.stderr)[-4000:])
        raise SystemExit(f'watch incomplete: {len(missing)} of {samples} samples missing')
    series=[]
    for k in range(samples):
        bank=(out/f'{k:03d}_g_motor_snapshot.bin').read_bytes()
        imu=(out/f'{k:03d}_g_imu.bin').read_bytes()
        series.append({'t':k*a.interval,
            'raw':[int.from_bytes(bank[i*40:i*40+2],'little') for i in range(6)],
            'total':[int.from_bytes(bank[i*40+8:i*40+16],'little',signed=True) for i in range(6)],
            'q':list(unpack_from('<6f',(out/f'{k:03d}_g_measurement.bin').read_bytes(),0)),
            'xy':list(unpack_from('<2f',(out/f'{k:03d}_g_measurement.bin').read_bytes(),72)),
            'vel':list(unpack_from('<2f',(out/f'{k:03d}_g_measurement.bin').read_bytes(),84)),
            'mode':int.from_bytes((out/f'{k:03d}_g_chassis.bin').read_bytes()[32:36],'little'),
            'inhibit':int.from_bytes((out/f'{k:03d}_g_chassis.bin').read_bytes()[4:8],'little'),
            'fault':int.from_bytes((out/f'{k:03d}_g_chassis.bin').read_bytes()[8:12],'little'),
            'torque':list(unpack_from('<6f',(out/f'{k:03d}_g_rl.bin').read_bytes(),1140)),
            'yaw':unpack_from('<f',imu,52)[0]})
    series=sanitize(series)
    report={'label':a.label,'interval_s':a.interval,'samples':series}
    report['segments']=[segment(series,i) for i in range(6)]
    report['excursions']=[excursion(series,i) for i in range(6)]
    (out/'watch.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(f"Watch {samples} samples over {samples*a.interval:.0f}s -> {(out/'watch.json').relative_to(ROOT).as_posix()}")
    print(f"q start {[round(v,3) for v in series[0]['q']]}")
    print(f"q end   {[round(v,3) for v in series[-1]['q']]}")
    print(f"yaw unwrapped {math.degrees(unwrap_yaw(series)):+.2f} deg "
          f"({series[0]['yaw']:+.4f} -> {series[-1]['yaw']:+.4f} rad)")
    print(f"root xy {[round(v,4) for v in series[0]['xy']]} -> {[round(v,4) for v in series[-1]['xy']]} m")
    modes=sorted({s['mode'] for s in series}); inhib=sorted({s['inhibit'] for s in series})
    peak=max((max(abs(v) for v in s['torque']) for s in series),default=0.0)
    print(f"modes {modes} inhibit {inhib} max|motor_torque| {peak:.2f} Nm")
    for i,peaks in enumerate(report['excursions']):
        detail=[(p['start_s'],p['peak_delta_counts'],round(p['peak_delta_deg'],2)) for p in peaks]
        print(f"{i} {monitor_can.NODE_LABELS[i]:<20} excursions {detail}")
    return 0

def segment(series,index,threshold=8):
    """Net movement runs, split by at least three unchanged samples."""
    runs=[]; start=None; last=None
    for k,s in enumerate(series):
        if k and s['total'][index]!=series[k-1]['total'][index]:
            if start is None: start=k-1
            last=k
        elif start is not None and k-last>=3:
            runs.append((start,last)); start=None
    if start is not None: runs.append((start,last))
    result=[]
    for i,j in runs:
        moved=series[j]['total'][index]-series[i]['total'][index]
        if abs(moved)>=threshold:
            result.append({'start_s':series[i]['t'],'end_s':series[j]['t'],'delta_counts':moved,
                           'delta_deg':moved*360/8192})
    return result

def excursion(series,index,threshold=20):
    """Peak deviation from the median. A spring-loaded leg returns, so the held
    net change understates a hand movement that was performed and released."""
    values=[s['total'][index] for s in series]
    base=statistics.median(values)
    result=[]; k=0
    while k<len(values):
        if abs(values[k]-base)>threshold:
            j=k
            while j+1<len(values) and abs(values[j+1]-base)>threshold: j+=1
            peak=max(values[k:j+1],key=lambda v:abs(v-base))
            result.append({'start_s':series[k]['t'],'end_s':series[j]['t'],'start_counts':values[k],
                           'end_counts':values[j],'peak_counts':peak,'peak_delta_counts':peak-base,
                           'peak_delta_deg':(peak-base)*360/8192})
            k=j+1
        else: k+=1
    return result

def unwrap_yaw(series):
    """Total signed yaw about the vertical axis, unwrapped between samples."""
    total=0.0; prev=series[0]['yaw']
    for s in series[1:]:
        d=s['yaw']-prev
        while d>math.pi: d-=2*math.pi
        while d<-math.pi: d+=2*math.pi
        total+=d; prev=s['yaw']
    return total

def sanitize(series):
    """Drop torn 64-bit RAM reads: the running CPU can rewrite one under the debugger."""
    last=None
    for s in series:
        if last is not None and any(abs(v)>2**31 for v in s['total']): s['total']=list(last)
        last=s['total']
    return series

def reset(a):
    """SYSRESETREQ reset: clears the latched rx_overflow and starts the firmware."""
    elf=ROOT/'build/rl/wheel_leg_rl.elf'
    argv=monitor_can.openocd_argv(a.openocd,['gdb_port disabled','tcl_port disabled','telnet_port disabled',
                                             'init','targets stm32h723.cpu0','reset run','shutdown'])
    proc=subprocess.run(argv,cwd=ROOT,capture_output=True,text=True,timeout=60)
    print((proc.stdout+proc.stderr).strip().splitlines()[-3:] if proc.stdout else '')
    return proc.returncode

def report(a):
    data=json.loads(a.watch.read_text(encoding='utf-8'))
    series=sanitize(data['samples'])
    data['excursions']=[excursion(series,i) for i in range(6)]
    a.watch.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')
    for i,peaks in enumerate(data['excursions']):
        detail=[(p['start_s'],p['peak_delta_counts'],round(p['peak_delta_deg'],2)) for p in peaks]
        print(f"{i} {monitor_can.NODE_LABELS[i]:<20} excursions {detail}")
    return 0

def delta(a):
    session=load_session(a.session); index=read_index(session)
    before,after=find(index,a.before),find(index,a.after)
    print(f"{'idx':>3} {'motor':<20} {'raw from':>8} {'raw to':>7} {'wrapped':>8} {'total delta':>12} sign")
    for i,(p,c) in enumerate(zip(before['motors'],after['motors'])):
        wrapped=((c['raw']-p['raw']+4096)%8192)-4096
        total=c['total']-p['total']
        sign='+1' if total>0 else ('-1' if total<0 else '?')
        print(f"{i:>3} {p['label']:<20} {p['raw']:>8} {c['raw']:>7} {wrapped:>8} {total:>12} {sign}")
    print(f"between {before['at']} and {after['at']}")
    return 0

def pose_from_args(a):
    if a.angles:
        try: return measured_pose(*a.angles)
        except ValueError as e: raise SystemExit(str(e))
    if a.at_training_default:
        return json.loads((ROOT/'user/algorithm/model/manifest.json').read_text())['default_q']
    return [math.radians(x) for x in a.joint_deg]

def ik_leg(x,z,side):
    """Wheel center (x forward, z up) relative to the hip to joint angles.

    Same FK as user/state_estimator.c: X=0.21cos(a)+0.25cos(b), Z=-0.21sin(a)-0.25sin(b).
    Returns the solution on the archived assembly branch, or None.
    """
    u,v=x,-z
    l1,l2=0.21,0.25
    r=math.hypot(u,v)
    if not abs(l1-l2)<r<l1+l2: return None
    cos_delta=(r*r-l1*l1-l2*l2)/(2*l1*l2)
    for delta in (math.acos(max(-1.0,min(1.0,cos_delta))),-math.acos(max(-1.0,min(1.0,cos_delta)))):
        a=math.atan2(v,u)-math.atan2(l2*math.sin(delta),l1+l2*math.cos(delta))
        b=a+delta
        q=[a,b-a-math.pi/2]
        if side<0: q=[-a,math.pi/2-b+a]
        beta=side*q[1]+math.pi/2
        alpha=2*math.atan2(math.sin(beta),113.4/135+math.cos(beta))
        if -2*math.pi+0.001<alpha<-0.001: return q
    return None

def pose(a):
    if a.counts: counts=list(a.counts); gravity=None
    else:
        session=load_session(a.session)
        target=session/'snapshots'/'pose'
        report=monitor_can.snapshot(target,interval=None,openocd=a.openocd)
        snap=report['a']
        counts=[n['encoder_counts'] for n in snap['nodes']]
        gravity=snap['imu']['gravity_body']
        print_counts([{'label':n['label'],'raw':n['encoder_counts'],'total':n['encoder_total_counts'],
                       'temp':n['temperature_c'],'status':n['status_byte']} for n in snap['nodes']])
    if gravity:
        tilt=math.degrees(math.acos(max(-1.0,min(1.0,-gravity[2]))))
        print(f"IMU gravity ({', '.join(f'{v:+.4f}' for v in gravity)}) -> chassis tilt {tilt:.2f} deg")
        if tilt>2.0: print('WARNING: level the chassis before using this pose measurement')
    if a.hip_wheel_distance:
        drop=(a.hip_height-a.wheel_radius)/1000.0; reach=a.hip_wheel_distance/1000.0
        if reach<abs(drop): raise SystemExit('hip-to-wheel distance is shorter than the vertical drop')
        a.wheel_x=[math.sqrt(max(0.0,reach*reach-drop*drop))*(1000.0 if a.front else -1000.0)]
    x_right=(a.wheel_x[0] if len(a.wheel_x)==1 else a.wheel_x[0])/1000.0
    x_left=(a.wheel_x[0] if len(a.wheel_x)==1 else a.wheel_x[1])/1000.0
    z=a.wheel_radius/1000.0-a.hip_height/1000.0
    q=[0]*6
    for side,x in ((1,x_right),(-1,x_left)):
        solution=ik_leg(x,z,side)
        if solution is None:
            raise SystemExit(f'no assembly-branch solution for side {"right" if side>0 else "left"}: '
                             f'check hip height {a.hip_height} mm and wheel offset {x*1000:.1f} mm')
        h=0 if side>0 else 3
        q[h],q[h+1]=solution
    print(f"q = [{', '.join(f'{v:+.4f}' for v in q)}] rad")
    try: values=macros(counts,a.signs,q)
    except ValueError as e: raise SystemExit(str(e))
    for name in NAMES: print(define_line(name,values[name]))
    if not a.write: return 0
    apply_values(values,{'counts':counts,'signs':a.signs,'joint_rad':q,'source':'hip-height pose solve'})
    print(f'Wrote {CONFIG.relative_to(ROOT).as_posix()}; commissioning flags unchanged.')
    return 0

def apply_values(values,record=None,dry_run=False,session=None):
    """Rewrite only the three calibration arrays; never touches the flags."""
    text=CONFIG.read_text(encoding='utf-8')
    updated=text
    for name in NAMES:
        line=define_line(name,values[name])
        pattern=re.compile(r'^#define '+name+r' \{.*\}$',re.M)
        if not pattern.search(updated): raise SystemExit(f'{name} line not found in robot_config.h')
        updated=pattern.sub(lambda m,line=line:line,updated,count=1)
    for name in NAMES:
        old=re.search(r'^#define '+name+r' \{.*\}$',text,re.M).group(0)
        new=re.search(r'^#define '+name+r' \{.*\}$',updated,re.M).group(0)
        print('-'+old+'\n+'+new)
    if dry_run:
        print('/* dry run: robot_config.h not modified */')
        return
    CONFIG.write_text(updated,encoding='utf-8',newline='')
    if record is not None:
        index=read_index(load_session(session))
        index['applied']=index.get('applied',[])+[{'at':now(),**record,**values}]
        save_index(load_session(session),index)

def apply(a):
    q=pose_from_args(a)
    try: values=macros(a.counts,a.signs,q)
    except ValueError as e: raise SystemExit(str(e))
    apply_values(values,{'counts':a.counts,'signs':a.signs,'joint_rad':q},a.dry_run,a.session)
    if not a.dry_run: print(f'Wrote {CONFIG.relative_to(ROOT).as_posix()}; commissioning flags unchanged.')
    return 0

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--session',type=Path,help='commission session directory')
    sub=p.add_subparsers(dest='command',required=True)
    c=sub.add_parser('capture',help='read one RAM snapshot and store it in the session')
    c.add_argument('--label',required=True)
    c.add_argument('--openocd',type=Path,default=monitor_can.DEFAULT_OPENOCD)
    c.set_defaults(run=capture)
    d=sub.add_parser('delta',help='compare two stored snapshots')
    d.add_argument('--before',required=True); d.add_argument('--after',required=True)
    d.set_defaults(run=delta)
    s=sub.add_parser('watch',help='record raw counts continuously and segment the movements')
    s.add_argument('--label',required=True)
    s.add_argument('--seconds',type=float,default=60)
    s.add_argument('--interval',type=float,default=1)
    s.add_argument('--openocd',type=Path,default=monitor_can.DEFAULT_OPENOCD)
    s.set_defaults(run=watch)
    r=sub.add_parser('report',help='re-analyze a stored watch.json without hardware')
    r.add_argument('--watch',type=Path,required=True)
    r.set_defaults(run=report)
    z=sub.add_parser('reset',help='reset the board (clears latched CAN rx overflow)')
    z.add_argument('--openocd',type=Path,default=monitor_can.DEFAULT_OPENOCD)
    z.set_defaults(run=reset)
    w=sub.add_parser('apply',help='write the three calibration macros into robot_config.h')
    w.add_argument('--counts',type=int,nargs=6,required=True)
    w.add_argument('--signs',type=int,nargs=6,required=True)
    w.add_argument('--angles',type=float,nargs=4,metavar=('RIGHT_THIGH','RIGHT_INCLUDED','LEFT_THIGH','LEFT_INCLUDED'))
    w.add_argument('--joint-deg',type=float,nargs=6)
    w.add_argument('--at-training-default',action='store_true')
    w.add_argument('--dry-run',action='store_true')
    w.set_defaults(run=apply)
    q=sub.add_parser('pose',help='reference pose from measured hip height and wheel offset')
    q.add_argument('--signs',type=int,nargs=6,required=True)
    q.add_argument('--hip-height',type=float,required=True,help='hip axis height above the floor, mm')
    q.add_argument('--wheel-x',type=float,nargs='+',default=[0.0],
                   help='wheel center fore-aft offset from the hip axis, mm, positive forward; one value or right/left')
    q.add_argument('--hip-wheel-distance',type=float,
                   help='straight-line hip axis to wheel axle distance, mm; overrides --wheel-x')
    q.add_argument('--front',action='store_true',help='wheel center is in front of the hip axis')
    q.add_argument('--wheel-radius',type=float,default=55.0,help='wheel radius, mm')
    q.add_argument('--counts',type=int,nargs=6,help='use these counts instead of a fresh board read')
    q.add_argument('--openocd',type=Path,default=monitor_can.DEFAULT_OPENOCD)
    q.add_argument('--write',action='store_true')
    q.set_defaults(run=pose)
    a=p.parse_args()
    if a.command=='apply':
        if sum(map(bool,(a.angles,a.joint_deg,a.at_training_default)))!=1:
            p.error('choose exactly one of --angles, --joint-deg, --at-training-default')
        if any(n<0 or n>=8192 for n in a.counts) or any(s not in (-1,1) for s in a.signs):
            p.error('counts must be 0..8191; signs must be -1 or +1')
    return a.run(a)

if __name__=='__main__': sys.exit(main())
