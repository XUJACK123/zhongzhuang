"""Small independently testable control/contact/termination helpers."""
import math
import xml.etree.ElementTree as ET


def knee_branch_violation(q, minimum_included_angle_deg=0., stop_tolerance_rad=1e-4):
    """Policy-order tensors: physical bend is +/-q_knee + pi/2, NOT q_knee.

    The user-confirmed branch has -pi < bend < 0. Both collinear
    singularities and the reversed branch are invalid, on either leg.
    """
    right, left = q[..., 1] + math.pi/2, -q[..., 4] + math.pi/2
    invalid=(right <= -math.pi) | (right >= 0) | (left <= -math.pi) | (left >= 0)
    if minimum_included_angle_deg:
        minimum=math.radians(minimum_included_angle_deg)-stop_tolerance_rad
        invalid=invalid | (right+math.pi<minimum) | (left+math.pi<minimum)
    return invalid


def intersect_mechanical_knee_limits(limits, mechanical):
    """Intersect a measured interior-angle stop with existing spring limits."""
    minimum=mechanical.get('minimum_knee_included_angle_deg',0.)
    if not math.isfinite(minimum) or not 0<=minimum<180:
        raise ValueError('Minimum knee interior angle must be in [0,180) degrees')
    result={name:None if values is None else list(values) for name,values in limits.items()}
    stop=math.radians(minimum)-1.5*math.pi
    for name,sign in (('rf1_Joint',1),('lf1_Joint',-1)):
        if name not in result:continue
        lo,hi=result[name]
        if sign==1:lo=max(lo,stop)
        else:hi=min(hi,-stop)
        if not lo<hi:raise ValueError('Mechanical and spring knee ranges do not intersect: '+name)
        result[name]=[lo,hi]
    return result


def validate_knee_branch(params, urdf=None):
    """Reject unsafe ranges even when gas springs are disabled. No collision claim."""
    sources = {'parameters': params['joint_limits_rad']}
    if urdf is not None:
        root = ET.parse(urdf).getroot()
        sources['urdf'] = {name: [float(root.find(f"joint[@name='{name}']/limit").get(k))
                                   for k in ('lower','upper')]
                           for name in ('rf1_Joint','lf1_Joint')}
    report = {}
    minimum=params.get('mechanical_limits',{}).get('minimum_knee_included_angle_deg',0.)
    intersect_mechanical_knee_limits(params['joint_limits_rad'],params.get('mechanical_limits',{}))
    for name,sign in (('rf1_Joint',1),('lf1_Joint',-1)):
        default = sign*params['default_joint_angles_rad'][name] + math.pi/2
        if not -math.pi < default < 0:
            raise ValueError(f'{name}: default pose is outside the permitted knee bending branch')
        if default+math.pi<math.radians(minimum)-1e-9:
            raise ValueError(f'{name}: default pose violates measured minimum knee angle')
        for source,limits in sources.items():
            lo,hi = limits[name]
            bends = sorted((sign*lo+math.pi/2,sign*hi+math.pi/2))
            if not lo < hi or not (-math.pi < bends[0] <= bends[1] < 0):
                raise ValueError(f'{name}: {source} limits permit a straight/folded singularity or knee reverse bend')
            if bends[0]+math.pi<math.radians(minimum)-1e-9:
                raise ValueError(f'{name}: {source} limits violate measured minimum knee angle')
        report[name] = dict(default_bend_deg=math.degrees(default),
                           minimum_included_angle_deg=minimum,
                           bend_range_deg=[math.degrees(x) for x in bends],
                           straight_joint_angle_rad=-sign*math.pi/2,
                           reversed_default_joint_angle_rad=sign*(-default-math.pi/2))
    return report


def adjacent_collision_masks(body_names, joints):
    """One shared bit per adjacent pair; all non-adjacent pairs remain enabled."""
    masks = {name: 0 for name in body_names}
    for bit, (parent, child) in enumerate(joints):
        masks[parent] |= 1 << bit
        masks[child] |= 1 << bit
    return masks


def failures(torch, gravity, base_height, contacts, root, joints, cfg):
    tilted = gravity[:, 2] > -math.cos(math.radians(cfg.max_tilt_deg))
    low = base_height < cfg.min_base_height_m
    contact = contacts > cfg.contact_force_n
    invalid = ~torch.isfinite(root).all(dim=1) | ~torch.isfinite(joints).all(dim=1)
    return tilted | low | contact, invalid


def position_targets(torch, actions, default, limits, scale):
    target = default + actions * scale
    return torch.maximum(torch.minimum(target, limits[:, 1]), limits[:, 0])


def periodic_hip_error(torch, target, position):
    error=target-position
    error=error.clone()
    error[..., [0,3]]=torch.atan2(torch.sin(error[..., [0,3]]),torch.cos(error[..., [0,3]]))
    return error
