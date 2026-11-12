"""Independent seeded per-episode force sampling; no dependence on actor IDs/COM."""
import math

def validate(settings):
    if settings is None:return None
    keys={'force_range_n','yaw_torque_range_nm','duration_s','gap_s','first_delay_s','force_free_fraction','seed'}
    if not isinstance(settings,dict) or set(settings)!=keys:raise ValueError('Complete random-pulse settings required')
    for key in keys-{'force_free_fraction','seed'}:
        pair=settings[key]
        if not isinstance(pair,(list,tuple)) or len(pair)!=2 or any(type(v) not in (int,float) or not math.isfinite(v) for v in pair) or pair[0]>pair[1]:
            raise ValueError('Invalid pulse range: '+key)
        if key not in ('yaw_torque_range_nm',) and pair[0]<0:raise ValueError('Negative time/force range')
        if key in ('duration_s','gap_s','first_delay_s') and pair[0]<=0:raise ValueError('Positive pulse timing required')
    fraction=settings['force_free_fraction']
    if type(fraction) not in (int,float) or not math.isfinite(fraction) or not 0<=fraction<1:raise ValueError('Force-free fraction must be in [0,1)')
    if type(settings['seed']) is not int or not 0<=settings['seed']<2**63:raise ValueError('Invalid pulse seed')
    return dict(settings)

class RandomPulses:
    def __init__(self,ops,num_envs,device,dtype,dt,settings):
        self.ops=ops;self.n=num_envs;self.device=device;self.dtype=dtype;self.dt=dt;self.settings=validate(settings)
        if self.settings is None or not math.isfinite(dt) or dt<=0:raise ValueError('Enabled settings and positive dt required')
        self.generator=ops.Generator(device=device).manual_seed(settings['seed'])
        self.force_free=ops.zeros(num_envs,device=device,dtype=ops.bool)
        self.next_start=ops.zeros(num_envs,device=device,dtype=ops.long)
        self.end_step=ops.zeros_like(self.next_start)
        self.elapsed=ops.zeros_like(self.next_start)
        self.current=ops.zeros((num_envs,6),device=device,dtype=dtype)
        self.output=ops.zeros_like(self.current)
        self.pulse_count=ops.zeros_like(self.next_start)
        self.sign_count=ops.zeros((num_envs,6),device=device,dtype=ops.long)
        self.reset_count=ops.zeros_like(self.next_start)
        self.last_tick=None
        self.reset(ops.arange(num_envs,device=device))

    def uniform(self,shape):
        return self.ops.rand(shape,device=self.device,dtype=self.dtype,generator=self.generator)

    def span(self,values,key):
        low,high=self.settings[key];return low+(high-low)*values

    def reset(self,ids):
        if len(ids)==0:return
        draw=self.uniform((len(ids),2))
        self.force_free[ids]=draw[:,0]<self.settings['force_free_fraction']
        self.next_start[ids]=self.ops.ceil(self.span(draw[:,1],'first_delay_s')/self.dt).long()
        self.end_step[ids]=0;self.elapsed[ids]=0;self.current[ids]=0;self.output[ids]=0
        self.reset_count[ids]+=1;self.last_tick=None

    def at(self,tick):
        # Physics decimation calls the force hook repeatedly without advancing
        # control time. Draw once and hold the exact same wrench in all substeps.
        if tick==self.last_tick:return self.output
        self.last_tick=tick;ops=self.ops
        # PPO randomizes episode_length_buf at startup. Measure real physical
        # time since reset independently so that first pulses cannot jump ahead.
        episode_steps=self.elapsed
        draw=self.uniform((self.n,5))
        due=(episode_steps>=self.next_start)&~self.force_free
        duration=ops.ceil(self.span(draw[:,0],'duration_s')/self.dt).long()
        gap=ops.ceil(self.span(draw[:,1],'gap_s')/self.dt).long()
        magnitude=self.span(draw[:,2],'force_range_n');angle=(draw[:,3]*2-1)*math.pi
        yaw=self.span(draw[:,4],'yaw_torque_range_nm');zero=ops.zeros_like(yaw)
        candidate=ops.stack((magnitude*ops.cos(angle),magnitude*ops.sin(angle),zero,zero,zero,yaw),dim=-1)
        self.current=ops.where(due[:,None],candidate,self.current)
        self.end_step=ops.where(due,episode_steps+duration,self.end_step)
        self.next_start=ops.where(due,episode_steps+duration+gap,self.next_start)
        self.pulse_count+=due.long()
        signs=ops.stack(tuple(due&(candidate[:,i]*sign>0) for i in (0,1,5) for sign in (1,-1)),dim=-1)
        self.sign_count+=signs.long()
        active=(episode_steps<self.end_step)&~self.force_free
        self.output=ops.where(active[:,None],self.current,ops.zeros_like(self.current))
        self.elapsed=self.elapsed+1
        return self.output

    def report(self,nominal_mask):
        def group(mask):
            return dict(environments=int(mask.sum()),force_free_current=int(self.force_free[mask].sum()),
                pulses=int(self.pulse_count[mask].sum()),signed_event_counts=self.sign_count[mask].sum(0).cpu().tolist())
        return dict(settings=self.settings,dt=self.dt,nominal_com=group(nominal_mask),other_com=group(~nominal_mask),
            sign_order=['+X','-X','+Y','-Y','+yaw','-yaw'],
            force_free_current=self.force_free.cpu().tolist(),next_start_steps=self.next_start.cpu().tolist(),
            note='Force-free membership resampled independently at each episode reset;own RNG never consumes the global training RNG. Pulse lengths quantized up to one control tick. Counts cumulative across resets.')
