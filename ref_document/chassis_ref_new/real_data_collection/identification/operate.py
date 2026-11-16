"""Explicit hardware operations for the isolated one-shot diagnostic.

No debugger sampling while an armed trial may run. The operator keeps S1 down
until arm has completed and OpenOCD has exited. collect is only used afterwards.
"""
import argparse, datetime as dt, hashlib, json, math, shutil, struct, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT/'real_data_collection'))
import collect_real_data as c
sys.path.insert(0,str(ROOT/'tools'))
import monitor_can as m
ELF=ROOT/'build/identification/limited_current_identification.elf'
NAMES=('g_ident','g_ident_request_token','g_ident_log','g_ident_tick','g_ident_healthy',
       'g_motor_snapshot','g_remote','g_imu','g_can_monitor','g_can_tx_fail','g_can_rx_overflow','g_diag_can')
S=m.elf_symbols(ELF,NAMES)
PHASES=('IDLE','ARMED','ENABLING','RUNNING','DONE','COOLDOWN')
REASONS=('NONE','COMPLETE','REMOTE_STOP','FEEDBACK_STOP','MOVEMENT_STOP','BUS_STOP','TIME_STOP','INVALID_INPUT')

def save(path,obj): c.save(path,obj)
def setup(new=False):
    global S
    pointer=HERE/'latest_trial.txt'
    if new:
        stamp=dt.datetime.now().strftime('%Y%m%d_%H%M%S')
        out=HERE/('trial_'+stamp);out.mkdir();(out/'raw').mkdir()
        pointer.write_text(str(out),encoding='utf-8')
    else:
        out=Path(pointer.read_text().strip())
        S=m.elf_symbols(out/'diagnostic.elf',NAMES)
    c.OUT=out;c.RAW=out/'raw'
    return out
def dump(tag,name,size=None):
    addr,n=S[name]
    return f'dump_image {{{(c.RAW/(tag+"_"+name+".bin")).as_posix()}}} {addr:#x} {n if size is None else size}'
def rd(tag,name):return (c.RAW/(tag+'_'+name+'.bin')).read_bytes()
def u(tag,name):return struct.unpack('<I',rd(tag,name))[0]
def progress(text):
    p=ROOT.parents[1]/'progress.md';s=p.read_text(encoding='utf-8')
    a='<!-- LIMITED_CURRENT_20261116_BEGIN -->';b='<!-- LIMITED_CURRENT_20261116_END -->'
    block=a+'\n### 0.2 独立限流辨识（实时更新）\n\n'+text+'\n\n'+b
    if a in s:s=s[:s.index(a)]+block+s[s.index(b)+len(b):]
    else:s=s.replace('## 1. 一句话状态',block+'\n\n## 1. 一句话状态',1)
    p.write_text(s,encoding='utf-8')
def sync_main(stage):
    path=ROOT/'real_data_collection/real_data_full_tests_20261116_165832.json'
    main=json.loads(path.read_text(encoding='utf-8'));trials=[]
    for folder in sorted(HERE.glob('trial_*')):
        if not (folder/'session.json').exists():continue
        row={'directory':str(folder),'image':json.loads((folder/'session.json').read_text())}
        if (folder/'result.json').exists():row['record']=json.loads((folder/'result.json').read_text())
        trials.append(row)
    main['active_identification']={'stage':stage,'updated_at_utc':c.utc(),'trials':trials,
        'independent_shaft_torque_measured':False,'spring_force_curve_measured':False,'getup_test_executed':False,
        'previous_zero_output_snapshots_are_separate_epoch':True}
    save(path,main)
def status(tag,require_done=False):
    reject='$phase != 4' if require_done else '$phase != 0 && $phase != 1 && $phase != 4'
    commands=[f'set phase [lindex [read_memory {S["g_ident"][0]:#x} 32 1] 0]',
              'echo PHASE_BEFORE_SWD_DUMPS_$phase',
              f'if {{{reject}}} {{error "Diagnostic may be active; refuse long SWD sampling"}}']
    for n in NAMES:
        commands.append(dump(tag,n,28 if n=='g_ident_log' else None))
    c.run(tag,commands,timeout=45)
    ident=struct.unpack_from('<4I',rd(tag,'g_ident'))
    bank=rd(tag,'g_motor_snapshot');rm=rd(tag,'g_remote');imu=rd(tag,'g_imu')
    result={'host_utc':c.utc(),'phase':PHASES[ident[0]],'reason':REASONS[ident[1]],
            'last_token':ident[3],'request_token':u(tag,'g_ident_request_token'),
            'tick':u(tag,'g_ident_tick'),'healthy_at_control_step':bool(u(tag,'g_ident_healthy')),
            's1':rm[10],'remote_valid':bool(rm[24]),'remote_tick':struct.unpack_from('<I',rm,12)[0],
            'imu_calibrated':bool(imu[109]),'gravity_body':struct.unpack_from('<3f',imu,40),
            'can':dict(zip(('received','other','tx_error','rx_error','bus_off','last_error','tx_queued','nonzero','enable','status_samples'),struct.unpack_from('<10I',rd(tag,'g_can_monitor')))),
            'tx_failed':u(tag,'g_can_tx_fail'),'fifo_overflow':u(tag,'g_can_rx_overflow'),
            'log_header':struct.unpack('<7I',rd(tag,'g_ident_log')),
            'motors':[],'asynchronous_swd_reads':True}
    wire=rd(tag,'g_diag_can')
    wire_names=('groups_tx','fc_tx','fd_tx','active_groups_tx','active_fc_tx','active_gap_max_us','event_loss_flags_seen','foreign_group_rx','foreign_dji_rx')
    if len(wire)>=60:wire_names+=('all_gap_max_us','expired_torque_abort_requests')
    offset=4*len(wire_names)
    result['wire']=dict(zip(wire_names,struct.unpack_from('<'+str(len(wire_names))+'I',wire)))
    result['wire']['last_foreign_group']=wire[offset:offset+8].hex(' ')
    result['wire']['last_foreign_dji']=wire[offset+8:offset+16].hex(' ')
    for i,label in enumerate(m.NODE_LABELS):
        q=i*40
        result['motors'].append({'label':label,'raw':struct.unpack_from('<H',bank,q)[0],
            'arm_rad':struct.unpack_from('<f',bank,q+16)[0],'velocity_rad_s':struct.unpack_from('<f',bank,q+20)[0],
            'feedback':struct.unpack_from('<f',bank,q+24)[0],'tick':struct.unpack_from('<I',bank,q+28)[0],
            'received':struct.unpack_from('<I',bank,q+32)[0],'temperature_c':bank[q+36],
            'state':bank[q+37],'position_lost':bool(bank[q+38])})
    save(c.OUT/(tag+'.json'),result)
    print(json.dumps(result,ensure_ascii=False))
    return result
def flash():
    previous=Path((HERE/'latest_trial.txt').read_text().strip()) if (HERE/'latest_trial.txt').exists() else None
    out=setup(True)
    manifest=json.loads((HERE/'manifest.json').read_text())
    assert manifest['elf_sha256']==hashlib.sha256(ELF.read_bytes()).hexdigest()
    for name in ('limited_current.h','limited_current.c','entry.c','test_controller.c','test_tx_age.c','bench_fdcan.c','diag_can.h','manifest.json','host_test.log','arm_build.log'):
        shutil.copy2(HERE/name,out/name)
    shutil.copy2(ROOT/'user/can_tx_age.h',out/'can_tx_age.h')
    for ext in ('.elf','.bin'):shutil.copy2(ELF.with_suffix(ext),out/('diagnostic'+ext))
    original=ROOT/'build/rl/wheel_leg_rl.elf'
    for ext in ('.elf','.bin'):shutil.copy2(original.with_suffix(ext),out/('restore_rl'+ext))
    diagnostic_running=previous and (previous/'session.json').exists() and json.loads((previous/'session.json').read_text()).get('flashed')
    if diagnostic_running:
        old=m.elf_symbols(previous/'diagnostic.elf',('g_ident','g_remote'))
        cha=old['g_ident'][0];rem=old['g_remote'][0]
        guard='if {[lindex $r 24] != 1 || [lindex $ch 0] != 4} {error "Previous one-shot diagnostic must be DONE; no flash"}'
        backup_bytes=(previous/'diagnostic.bin').stat().st_size
    else:
        old=m.elf_symbols(original,('g_chassis','g_remote'))
        cha=old['g_chassis'][0];rem=old['g_remote'][0]
        guard='if {[lindex $r 10] != 2 || [lindex $r 24] != 1 || [lindex $ch 8] != 0} {error "STOP/S1-down precondition failed; no flash"}'
        backup_bytes=original.with_suffix('.bin').stat().st_size
    commands=[f'set r [read_memory {rem:#x} 8 28]',f'set ch [read_memory {cha:#x} 32 10]',guard,
              'reset halt','wait_halt 2000',
              f'dump_image {{{(out/"raw/preflash_previous.bin").as_posix()}}} 0x08000000 {backup_bytes}',
              f'program {{{ELF.as_posix()}}} verify reset']
    progress('- 已保存前序实测，前一诊断为DONE；正在备份并换入新版独立诊断，上电锁定、不会自动出力。\n- 用户授权由主机自动烧录、触发、记录；刚性台架/限位已确认，S1中档提供运行许可，下档仍停车。\n- 正常RL源码/模型/标定未修改；电流换算不作为独立实测轴力矩。')
    c.run('flash_locked_diagnostic',commands,timeout=90)
    manifest['flashed']=True;manifest['flash_verified_by_openocd']=True
    manifest['user_fixture_confirmed']=True;manifest['independent_mechanical_limits_confirmed']=True
    manifest['restored_rl']=False
    save(out/'session.json',manifest)
    c.run('verify_running_vectors',[f'dump_image {{{(out/"raw/diagnostic_vectors.bin").as_posix()}}} 0x08000000 256','echo [stm32h723.cpu0 curstate]','sleep 3500'],timeout=30)
    assert (out/'raw/diagnostic_vectors.bin').read_bytes()==ELF.with_suffix('.bin').read_bytes()[:256]
    s=status('idle_after_flash')
    assert s['phase']=='IDLE' and s['request_token']==0 and s['can']['nonzero']==0 and s['can']['enable']==0
    levels='/'.join(f'{v:g}' for v in manifest.get('current_levels_a',[manifest['current_limit_a']]))+'A'
    progress(f'- 前序实测完整保留；已烧录校验{levels}主机触发诊断，当前IDLE/零输出，尚未发触发令牌。\n- S1中档为实体许可，主机发单次令牌后延迟2000ms，再运行每档最多500ms，中间零输出1000ms；腿位移上限{manifest["output_arm_delta_limit_deg"]:.1f}°、速度上限{manifest["velocity_limit_rad_s"]}rad/s；被动轮{manifest["passive_wheel_delta_limit_deg"]:.1f}°/{manifest["passive_wheel_speed_limit_rad_s"]}rad/s；S1下档/失联/反馈异常停止并取消待执行请求。\n- 完整日志/原程序备份：`{out.relative_to(ROOT).as_posix()}`；正常RL构建/模型/标定未改，当前板上为临时诊断。\n- 缺独立力参考，未完成T3独立轴力矩、T4纯弹簧轴力曲线或T8起身。')
    sync_main(levels+' diagnostic flashed and locked, waiting for arm')
def trigger():
    setup();s=status('pre_trigger')
    manifest=json.loads((c.OUT/'session.json').read_text())
    assert manifest.get('requires_software_trigger_and_S1_run'),'Wrong image protocol'
    assert s['phase'] in ('IDLE','DONE')
    assert s['healthy_at_control_step'] and s['s1'] in (1,3) and s['remote_valid'],'Physical S1 run permit required'
    assert all(not a['position_lost'] and a['temperature_c']<45 and abs(a['velocity_rad_s'])<.05 for a in s['motors'])
    if s['phase']=='DONE':
        previous=c.OUT
        assert (previous/'result.json').exists(),'Export the previous frozen record before reusing the image'
        out=setup(True)
        for name in ('limited_current.h','limited_current.c','entry.c','test_controller.c','test_tx_age.c','bench_fdcan.c','diag_can.h','can_tx_age.h','manifest.json','host_test.log','arm_build.log','diagnostic.elf','diagnostic.bin','restore_rl.elf','restore_rl.bin'):
            shutil.copy2(previous/name,out/name)
        manifest['image_reused_from']=str(previous);manifest['flashed_in_this_trial']=False
        save(out/'inherited_pre_trigger_status.json',s)
    token=(s['last_token']+1)&0xffffffff if s['last_token'] else 0x26111601
    assert token
    commands=[f'set r [read_memory {S["g_remote"][0]:#x} 8 28]',
        f'set phase [lindex [read_memory {S["g_ident"][0]:#x} 32 1] 0]',
        'if {[lindex $r 24] != 1 || [lindex $r 10] == 2 || ($phase != 0 && $phase != 4)} {error "Physical run permit/locked-state check failed"}',
        'if {![string equal [stm32h723.cpu0 curstate] running]} {error "CPU is not running"}',
        f'mww {S["g_ident_request_token"][0]:#x} {token:#x}']
    c.run('trigger_then_disconnect',commands,timeout=30) # shutdown immediately after token; no post-trigger reads
    manifest['trigger']={'token':token,'host_utc_after_debugger_exit':c.utc(),'delay_ms':2000,'no_debugger_sampling_until_done':True}
    save(c.OUT/'session.json',manifest)
    levels='/'.join(f'{v:g}' for v in manifest.get('current_levels_a',[manifest['current_limit_a']]))+'A'
    progress(f'- {levels}批次主机触发指令已发出，OpenOCD已退出；2000ms后由板上自主执行并记录，运行期间不轮询SWD。\n- 每档最多500ms，间隔零输出1000ms；腿保护{manifest["output_arm_delta_limit_deg"]:.1f}°/{manifest["velocity_limit_rad_s"]}rad/s，被动轮{manifest["passive_wheel_delta_limit_deg"]:.1f}°/{manifest["passive_wheel_speed_limit_rad_s"]}rad/s，S1下档/失联/失鲜停车并取消请求。\n- 完成或保护退出后再读取冻结记录；板上为独立诊断，RL模型/标定未改。')
    sync_main(levels+' host trigger sent; debugger exited; await autonomous completion before reading')
    print('TRIGGER_SENT_AND_DEBUGGER_EXITED; will read only after automatic stop')
def collect():
    setup()
    manifest=json.loads((c.OUT/'session.json').read_text())
    selected=manifest.get('selected_policy_index',1)
    time.sleep(8)  # Includes 2s host-trigger delay and the entire bounded three-stage sequence.
    s=status('post_trial',require_done=True)
    assert s['phase']=='DONE','No completed trial; do not halt or dump the large buffer'
    assert s['log_header'][5]==1,'Buffer is not frozen'
    header=s['log_header']
    used_bytes=28+header[2]*header[4]
    c.run('frozen_trial_log',[dump('frozen','g_ident_log',used_bytes)],timeout=90)
    data=rd('frozen','g_ident_log');magic,schema,size,cap,n,frozen,loopmax=struct.unpack_from('<7I',data)
    assert magic==0x1d261116 and ((schema==1 and size==208) or (schema==2 and size==212)) and cap==384 and n<=cap
    frames=[]
    for j in range(n):
        b=data[28+j*size:28+(j+1)*size]
        begin,tick,phase,reason,s1,remoteage,imuage,submitted=struct.unpack_from('<8I',b)
        end=struct.unpack_from('<I',b,size-4)[0]
        assert begin==end and begin%2==0 and begin>0
        frames.append({'tick':tick,'phase':PHASES[phase],'reason':REASONS[reason],'s1':s1,
            'remote_age_ms':remoteage,'imu_age_ms':imuage,'submitted':submitted,
            'requested_a':struct.unpack_from('<f',b,32)[0],'nominal_requested_nm':struct.unpack_from('<f',b,36)[0],
            'arm_rad':struct.unpack_from('<6f',b,40),'velocity_rad_s':struct.unpack_from('<6f',b,64),
            'protocol_feedback':struct.unpack_from('<6f',b,88),'motor_tick':struct.unpack_from('<6I',b,112),
            'motor_received':struct.unpack_from('<6I',b,136),'raw':struct.unpack_from('<6H',b,160),
            'temperature_c':list(b[172:178]),'state':list(b[178:184]),'dm_bytes':b[184:192].hex(' '),
            'dji_bytes':b[192:200].hex(' '),'trial_index':struct.unpack_from('<I',b,200)[0] if schema==2 else 0,
            'loop_us':struct.unpack_from('<I',b,size-8)[0]})
    assert frames
    nonzero=[f for f in frames if any(bytes.fromhex(f['dm_bytes']))]
    for f in frames:
        dm=bytes.fromhex(f['dm_bytes'])
        slot=2*((4,3,3,1,2,2)[selected]-1)
        assert dm[:slot]==bytes(slot) and dm[slot+2:]==bytes(6-slot) and bytes.fromhex(f['dji_bytes'])==bytes(8)
    first=frames[0]
    summary={'selected':manifest['selected'],'selected_policy_index':selected,'reason':s['reason'],'frames':n,'frozen':bool(frozen),'loop_max_us':loopmax,
       'nonzero_encoded_frames':len(nonzero),'max_requested_abs_a':max(abs(f['requested_a']) for f in frames),
       'max_nominal_requested_abs_nm':max(abs(f['nominal_requested_nm']) for f in frames),
       'max_abs_arm_delta_deg':[max(abs(f['arm_rad'][i]-first['arm_rad'][i]) for f in frames)*180/math.pi for i in range(6)],
       'max_abs_protocol_feedback':[max(abs(f['protocol_feedback'][i]) for f in frames) for i in range(6)],
       'feedback_units':['Nm','Nm','A','Nm','Nm','A'],
       'all_frame_gaps_2ms':all((y['tick']-x['tick'])&0xffffffff==2 for x,y in zip(frames,frames[1:])),
       'only_selected_motor_encoded_nonzero':True,'independent_shaft_torque_measured':False,'spring_force_curve_measured':False}
    summary['per_level']=[{'trial_index':i,'frames':len(fs),
       'max_requested_abs_a':max(abs(f['requested_a']) for f in fs),
       'planned_peak_a':manifest.get('current_levels_a',[manifest['current_limit_a']])[i],
       'max_abs_selected_reported_nm':max(abs(f['protocol_feedback'][selected]) for f in fs),
       'arm_range_selected_deg':(max(f['arm_rad'][selected] for f in fs)-min(f['arm_rad'][selected] for f in fs))*180/math.pi}
       for i in sorted(set(f['trial_index'] for f in frames if f['phase']=='RUNNING'))
       if (fs:=[f for f in frames if f['trial_index']==i and f['phase']=='RUNNING'])]
    save(c.OUT/'result.json',{'summary':summary,'frames':frames,'post_trial_status':s})
    save(c.OUT/'artifact_manifest.json',[{'path':str(p.relative_to(c.OUT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()} for p in sorted(c.OUT.rglob('*')) if p.is_file() and p.name!='artifact_manifest.json'])
    print('TRIAL_SUMMARY',json.dumps(summary,ensure_ascii=False))
    progress('- '+manifest['selected']+' 限流批次已执行并导出冻结记录：`'+str(c.OUT.relative_to(ROOT)).replace('\\','/')+'/result.json`。\n- 结果：'+json.dumps(summary,ensure_ascii=False)+'。\n- DM上报值为协议Nm，不是独立测得的轴力矩。板上仍为临时诊断，未宣称弹簧力曲线或起身完成。')
    sync_main('Current trial completed, frozen log exported; diagnostic remains on board')
if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('action',choices=['flash','status','arm','trigger','collect']);a=p.parse_args()
    if a.action=='flash':flash()
    elif a.action in ('arm','trigger'):trigger()
    elif a.action=='collect':collect()
    else:setup();status('status_'+dt.datetime.now().strftime('%H%M%S'))
