"""Opt-in fixed actor motor errors and effective robot/ground friction errors."""
import json
import math


def validate(settings):
    if settings is None:return None
    for key,bounds in (('motor_gain_range',(.9,1.1)),('ground_friction_scale_range',(.8,1.2))):
        values=settings.get(key)
        if (not isinstance(values,list) or len(values)!=2 or
            any(type(v) not in (int,float) or not math.isfinite(v) for v in values) or
            not bounds[0]<=values[0]<=1<=values[1]<=bounds[1]):
            raise ValueError('Invalid confirmed uncertainty bounds: '+key)
    for key,lo,hi in (('nominal_stride',2,100),('friction_buckets',2,64)):
        if type(settings.get(key)) is not int or not lo<=settings[key]<=hi:
            raise ValueError('Invalid '+key)
    return dict(settings)


def configure(cfg,settings):
    cfg.env.parameter_randomization=validate(settings)
    if settings is not None:
        if cfg.domain_rand.randomize_friction:raise ValueError('Two friction randomizers enabled')
        if cfg.terrain.static_friction!=cfg.terrain.dynamic_friction:
            raise ValueError('Effective friction scaling requires equal plane static/dynamic coefficients')
        if cfg.domain_rand.randomize_base_com and cfg.domain_rand.com_nominal_stride!=settings['nominal_stride']:
            raise ValueError('COM and parameter nominal subsets must coincide')


def sample(torch,count,settings):
    gains=torch.ones(count,6);scales=torch.ones(count)
    if settings is not None:
        validate(settings)
        lo,hi=settings['motor_gain_range'];gains.uniform_(lo,hi)
        lo,hi=settings['ground_friction_scale_range']
        buckets=torch.linspace(lo,hi,settings['friction_buckets'])
        scales=buckets[torch.randint(len(buckets),(count,))]
        gains[::settings['nominal_stride']]=1;scales[::settings['nominal_stride']]=1
    return gains,scales


def shape_friction(nominal,ground,scale):
    # Measured on the installed PhysX: mu_contact=(mu_shape+mu_plane)/2.
    value=scale*(nominal+ground)-ground
    if not math.isfinite(value) or value<0:raise ValueError('Requested effective friction cannot be represented')
    return value


def apply_shapes(props,nominal,ground,scale):
    if len(props)!=len(nominal):raise ValueError('Imported shape count changed')
    for prop,base in zip(props,nominal):
        prop.friction=base if scale==1 else shape_friction(base,ground,scale)
    return props


def audit(env):
    ground=env.cfg.terrain.static_friction
    actual=[];max_error=0.
    for i,(e,actor) in enumerate(zip(env.envs,env.actor_handles)):
        values=[float(p.friction) for p in env.gym.get_actor_rigid_shape_properties(e,actor)]
        expected=[shape_friction(base,ground,float(env.parameter_friction_scales[i])) for base in env.nominal_shape_friction]
        max_error=max(max_error,max(abs(a-b) for a,b in zip(values,expected)))
        actual.append(values)
    if max_error>1e-6:raise RuntimeError('Imported actor friction differs from requested distribution')
    cfg=env.cfg.env.parameter_randomization
    return dict(enabled=cfg is not None,settings=cfg,nominal_indices=list(range(0,env.num_envs,cfg['nominal_stride'] if cfg else 1)),
        motor_order=list(env.dof_names),motor_gains=env.parameter_motor_gains.tolist(),
        effective_ground_friction_scales=env.parameter_friction_scales.tolist(),
        nominal_shape_friction=env.nominal_shape_friction,ground_friction=ground,
        imported_shape_friction=actual,max_import_error=max_error,
        semantics='Fixed per actor; gain after nominal motor envelope then reclip. Friction scaling calibrated for ground contacts; self-contact friction is not a uniform scaling.')


def save(env,path):
    (path/'parameter_randomization.json').write_text(json.dumps(env.parameter_randomization_report,indent=2)+'\n')
