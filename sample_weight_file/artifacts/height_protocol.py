"""Command-only bottom-clearance evaluation; no pose resets implement height.

This helper does not enable a curriculum. Both evaluators opt into its explicit
command schedule and record metadata when a height test is requested.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class HeightProtocol:
    kind: str
    bounds_m: tuple
    knots: tuple
    minimum_duration_s: float
    slew_m_s: float

    @classmethod
    def from_config(cls, cfg, target_m=None, cycle=False, slew_m_s=.02):
        if not cfg['env'].get('clearance_control', False):
            raise ValueError('This archive does not command chassis-bottom clearance')
        if cycle and target_m is not None:
            raise ValueError('Choose a fixed clearance or a cycle, not both')
        bounds=tuple(float(v) for v in cfg['commands']['ranges']['height'])
        nominal=float(cfg['rewards']['base_height_target'])
        if len(bounds)!=2 or not all(math.isfinite(v) for v in (*bounds,nominal)):
            raise ValueError('Finite archived clearance range and nominal required')
        low,high=bounds
        if not 0<low<=nominal<=high:
            raise ValueError('Nominal clearance must lie in the positive archived range')
        if not math.isfinite(slew_m_s) or slew_m_s<=0:
            raise ValueError('Positive finite height slew required')
        if not cycle:
            target=nominal if target_m is None else float(target_m)
            if not math.isfinite(target) or not low<=target<=high:
                raise ValueError('Requested clearance is outside the archived training range')
            return cls('fixed',bounds,((0.,target),),30.,float(slew_m_s))
        if low==high:
            raise ValueError('This archive was trained at one height; no height cycle is supported')
        # Eight-second holds separate each ramp. The same 20 mm/s rate is used
        # by the interactive controls. Nothing depends on measured robot motion.
        knots=[(0.,nominal),(8.,nominal)]
        time=8.;previous=nominal
        for target in (low,high,nominal):
            time+=abs(target-previous)/slew_m_s
            if not math.isfinite(time):raise ValueError('Height cycle duration overflow')
            if time>knots[-1][0]:knots.append((time,target))
            time+=8.
            knots.append((time,target));previous=target
        duration=max(40.,time)
        if duration>time:knots.append((duration,nominal))
        return cls('cycle',bounds,tuple(knots),duration,float(slew_m_s))

    def command(self, time_s):
        if not math.isfinite(time_s) or time_s<0:
            raise ValueError('Nonnegative finite elapsed time required')
        for (start,a),(end,b) in zip(self.knots,self.knots[1:]):
            if time_s<=end:
                return a+(b-a)*(time_s-start)/(end-start)
        return self.knots[-1][1]

    def validate_duration(self, seconds):
        if not math.isfinite(seconds) or seconds+1e-9<self.minimum_duration_s:
            raise ValueError(f'Height protocol needs at least {self.minimum_duration_s:g} seconds')

    def metadata(self):
        return dict(protocol='bottom-clearance-slew-v1',kind=self.kind,
                    bounds_m=list(self.bounds_m),knots=[list(k) for k in self.knots],
                    minimum_duration_s=self.minimum_duration_s,slew_m_s=self.slew_m_s,
                    actuation='archived policy and bounded motor controller; command changes only')

    def matches_trace(self, time_s, commands, failed, dt):
        """Check action-start targets through each trial's first terminal step."""
        import numpy as np
        commands=np.asarray(commands);failed=np.asarray(failed,dtype=bool)
        if not math.isfinite(dt) or dt<=0:raise ValueError('Positive finite sample interval required')
        if commands.ndim!=2 or commands.shape!=failed.shape or len(time_s)!=len(commands):
            return False
        expected=np.array([self.command(max(0.,float(t)-dt)) for t in time_s])[:,None]
        expected=np.broadcast_to(expected,commands.shape)
        active=np.concatenate((np.ones((1,commands.shape[1]),dtype=bool),~failed[:-1]),axis=0)
        return bool(np.allclose(commands[active],expected[active],rtol=0.,atol=1e-6))
