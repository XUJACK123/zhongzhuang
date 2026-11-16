"""Bounded chassis-COM training distribution and imported-property audit."""
import math

def configure(cfg, settings):
    cfg.domain_rand.randomize_base_com=False
    cfg.domain_rand.com_nominal_stride=0
    if settings is None:return
    if set(settings)!={'half_range_m','nominal_stride'}:raise ValueError('Unknown COM randomization settings')
    bounds=settings['half_range_m'];stride=settings['nominal_stride']
    if len(bounds)!=3 or any(not math.isfinite(v) or not 0<v<=.02 for v in bounds):
        raise ValueError('COM half ranges must be three positive values <=20 mm')
    if type(stride) is not int or stride<2:raise ValueError('Nominal stride must be integer >=2')
    cfg.domain_rand.randomize_base_com=True
    cfg.domain_rand.rand_com_vec=list(bounds)
    cfg.domain_rand.com_nominal_stride=stride

def signature(prop):
    return dict(mass=float(prop.mass),com=[float(getattr(prop.com,a)) for a in 'xyz'],
        inertia=[float(getattr(getattr(prop.inertia,a),b)) for a in 'xyz' for b in 'xyz'])

def reserve_nominal(props,offsets,stride):
    # Parent initializes all offsets when creating environment zero, and already
    # applied its first offset. Undo only that one; future nominal rows are zero.
    for axis,value in zip('xyz',offsets[0].tolist()):
        setattr(props[0].com,axis,getattr(props[0].com,axis)-value)
    offsets[::stride]=0

def audit(env,nominal):
    expected=env.base_com.detach().cpu().tolist()
    actual=[];max_com_error=0.;max_mass_error=0.;max_inertia_error=0.
    for index,(handle,actor) in enumerate(zip(env.envs,env.actor_handles)):
        properties=env.gym.get_actor_rigid_body_properties(handle,actor)
        for body,(prop,baseline) in enumerate(zip(properties,nominal)):
            got=signature(prop)
            offset=expected[index] if body==0 else [0.]*3
            max_com_error=max(max_com_error,max(abs(a-b-c) for a,b,c in zip(got['com'],baseline['com'],offset)))
            max_mass_error=max(max_mass_error,abs(got['mass']-baseline['mass']))
            max_inertia_error=max(max_inertia_error,max(abs(a-b) for a,b in zip(got['inertia'],baseline['inertia'])))
            if body==0:actual.append([a-b for a,b in zip(got['com'],baseline['com'])])
    if max_com_error>1e-6 or max_mass_error>1e-6 or max_inertia_error>1e-6:
        raise RuntimeError('Imported COM distribution, mass or inertia differs from requested values')
    enabled=bool(env.cfg.domain_rand.randomize_base_com)
    stride=getattr(env.cfg.domain_rand,'com_nominal_stride',0)
    if enabled:
        bounds=env.cfg.domain_rand.rand_com_vec
        if any(abs(x)>b+1e-6 for row in actual for x,b in zip(row,bounds)):
            raise RuntimeError('Imported COM outside configured bounds')
        if any(any(abs(x)>1e-6 for x in actual[i]) for i in range(0,len(actual),stride)):
            raise RuntimeError('Nominal COM subset changed')
    return dict(enabled=enabled,environments=len(actual),frame='base_link local axes',
        nominal_chassis_com_m=nominal[0]['com'],nominal_stride=stride,
        nominal_count=sum(all(abs(x)<1e-7 for x in row) for row in actual),
        offset_min_m=[min(row[i] for row in actual) for i in range(3)],
        offset_max_m=[max(row[i] for row in actual) for i in range(3)],
        maximum_com_readback_error_m=max_com_error,maximum_mass_error_kg=max_mass_error,
        maximum_inertia_error_kg_m2=max_inertia_error,first_offsets_m=actual[:8],
        distribution='Uniform independent axes, fixed per environment, nominal rows every stride; not calibrated hardware')
