"""Passive push gas strut across an adjacent thigh/shin revolute joint.

Anchors are in URDF link frames (metres), never SolidWorks global frames.
A sealed gas volume reduced by rod insertion is assumed; no hydraulic end damping,
seal friction, thermal dynamics, strut mass or end-stop contact is added.
The calibrated polytropic model requires a valid stroke interval. The explicit
nominal_constant approximation requires no unmeasured volume/adapter dimensions
and can use installation-derived stroke limits. When configured, physical stops
are represented by URDF hinge limits; a small numerical penetration tolerance
does not enlarge the specified stroke. Force progression remains uncalibrated.
"""
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load_config(path, urdf, enable=None):
    cfg = json.loads(Path(path).read_text(encoding="utf-8"))
    if enable is not None:
        cfg["enabled"] = enable
    if not isinstance(cfg.get("enabled"), bool):
        raise ValueError("gas_spring.enabled must be true or false")
    if not cfg["enabled"]:
        return cfg
    errors = []
    if cfg.get("geometry_confirmed") is not True and not cfg.get("geometry_approximation"):
        errors.append("geometry: confirm anchors or explicitly describe the approximation")
    springs = cfg.get("springs", [])
    if not springs:
        errors.append("springs: at least one strut is required")
    root = ET.parse(urdf).getroot()
    joints = {j.attrib["name"]: j for j in root.findall("joint")}
    for s in springs:
        name = s.get("joint", "<missing joint>")
        j = joints.get(name)
        if j is None or name not in ("rf1_Joint", "lf1_Joint"):
            errors.append(f"{name}: only the adjacent thigh/shin knee joint is supported")
            continue
        for field in ("parent_anchor_m", "child_anchor_m"):
            v = s.get(field)
            if not isinstance(v, list) or len(v) != 3 or not all(_number(x) for x in v):
                errors.append(f"{name}.{field}: supply three finite coordinates in metres")
        model = s.setdefault("force_model", "polytropic")
        if model not in ("polytropic", "nominal_constant"):
            errors.append(f"{name}: unsupported force_model {model}")
        fields = ["reference_force_n"]
        if model == "polytropic":
            fields += ["min_length_m", "max_length_m", "reference_length_m",
                       "rod_diameter_m", "reference_gas_volume_m3",
                       "polytropic_exponent", "ambient_pressure_pa"]
        elif s.get("min_length_m") is not None or s.get("max_length_m") is not None:
            fields += ["min_length_m", "max_length_m"]
        for field in fields:
            if not _number(s.get(field)) or s[field] <= 0:
                errors.append(f"{name}.{field}: supply a positive measured/confirmed value")
        damping = s.get("viscous_damping_ns_m")
        if not _number(damping) or damping < 0:
            errors.append(f"{name}.viscous_damping_ns_m: must be nonnegative")
        tolerance = s.get('length_limit_tolerance_m', 0.)
        if not _number(tolerance) or tolerance < 0 or tolerance > 0.002:
            errors.append(f'{name}.length_limit_tolerance_m: expected 0..0.002 m numerical solver tolerance')
        stops = s.get('joint_stop_limits_rad')
        if stops is not None:
            limit = j.find('limit')
            if not isinstance(stops,list) or len(stops) != 2 or not all(_number(v) for v in stops) or not stops[0] < stops[1]:
                errors.append(f'{name}: invalid gas-spring joint stops')
            elif limit is None or any(limit.get(k) is None or abs(float(limit.get(k))-v) > 1e-9 for k,v in zip(('lower','upper'),stops)):
                errors.append(f'{name}: URDF limits do not enforce the configured gas-spring stroke; rebuild/update the asset')
        if s.get('stroke_m') is not None:
            if not _number(s.get('stroke_m')) or s['stroke_m'] <= 0:
                errors.append(f'{name}.stroke_m must be positive')
            elif not all(_number(s.get(k)) for k in ('min_length_m','max_length_m')):
                errors.append(f'{name}: installed length endpoints required with stroke_m')
            elif abs(s['max_length_m']-s['min_length_m']-s['stroke_m']) > 1e-9:
                errors.append(f'{name}: installed length interval must equal specified stroke')
        # Transform parent anchor into the zero-angle joint frame. The current
        # training URDF has a Z rotation only; reject unsupported transforms.
        origin = j.find("origin")
        xyz = [float(v) for v in origin.attrib.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in origin.attrib.get("rpy", "0 0 0").split()]
        axis = [float(v) for v in j.find("axis").attrib["xyz"].split()]
        if abs(rpy[0]) > 1e-9 or abs(rpy[1]) > 1e-9 or axis != [0., 0., 1.]:
            errors.append(f"{name}: expected Z hinge and zero roll/pitch in URDF")
        s["joint_origin_m"] = xyz
        s["joint_yaw_rad"] = rpy[2]
    if errors:
        raise ValueError("Gas spring configuration incomplete:\n- " + "\n- ".join(errors))
    for s in springs:
        if s["force_model"] == "nominal_constant":
            if s.get("min_length_m") is not None and not s["min_length_m"] < s["max_length_m"]:
                raise ValueError(f"{s['joint']}: invalid length interval")
            continue
        lo, hi, ref = (s[k] for k in ("min_length_m", "max_length_m", "reference_length_m"))
        area = math.pi * s["rod_diameter_m"] ** 2 / 4
        if not 0 < lo < hi or not lo <= ref <= hi:
            raise ValueError(f"{s['joint']}: invalid length interval/reference")
        if s["reference_gas_volume_m3"] - area * (ref - lo) <= 0:
            raise ValueError(f"{s['joint']}: gas volume becomes nonpositive at compression")
        if not 1 <= s["polytropic_exponent"] <= 1.4:
            raise ValueError("Choose n in [1, 1.4]: isothermal through adiabatic nitrogen")
    return cfg


def geometry(q, s, ops=math):
    """Return length and analytic d(length)/dq; works with floats or tensors."""
    ox, oy, oz = s["joint_origin_m"]
    ax, ay, az = s["parent_anchor_m"]
    yaw = s["joint_yaw_rad"]
    c, sn = math.cos(yaw), math.sin(yaw)
    ax, ay, az = c*(ax-ox)+sn*(ay-oy), -sn*(ax-ox)+c*(ay-oy), az-oz
    bx, by, bz = s["child_anchor_m"]
    x = ops.cos(q)*bx - ops.sin(q)*by
    y = ops.sin(q)*bx + ops.cos(q)*by
    dx, dy, dz = x-ax, y-ay, bz-az
    length = ops.sqrt(dx*dx + dy*dy + dz*dz)
    # Reject coincident anchors in the caller; epsilon prevents NaNs first.
    derivative = (-dx*y + dy*x) / (length + 1e-12)
    return length, derivative


def stroke_joint_interval(s, reference_q, sign, branch_margin_rad=1e-3):
    """Connected feasible interval containing the extended reference pose.

    Solve L(q)^2 = A+B*cos(q)+C*sin(q) analytically. The red branch is a
    topological constraint, not a collision constraint. A compression endpoint
    below the geometric minimum cannot create a physical compression stop.
    """
    f=lambda q: geometry(q,s)[0]**2
    a=(f(0)+f(math.pi))/2; b=(f(0)-f(math.pi))/2; c=f(math.pi/2)-a
    amplitude=math.hypot(b,c); phase=math.atan2(c,b)
    if amplitude<1e-14:
        raise ValueError('Spring mounting does not constrain knee angle')
    low,high=sorted(sign*(beta-math.pi/2) for beta in (-math.pi+branch_margin_rad,-branch_margin_rad))
    cuts=[low,high]
    for length in (s['min_length_m'],s['max_length_m']):
        ratio=(length*length-a)/amplitude
        if -1<=ratio<=1:
            angle=math.acos(ratio)
            for root in (phase-angle,phase+angle):
                for winding in range(-3,4):
                    q=root+2*math.pi*winding
                    if low<q<high: cuts.append(q)
    cuts=sorted(set(cuts)); feasible=[]
    for lo,hi in zip(cuts,cuts[1:]):
        length=geometry((lo+hi)/2,s)[0]
        if s['min_length_m']-1e-12<=length<=s['max_length_m']+1e-12:
            if feasible and abs(feasible[-1][1]-lo)<1e-10: feasible[-1][1]=hi
            else: feasible.append([lo,hi])
    chosen=next((interval for interval in feasible if interval[0]-1e-9<=reference_q<=interval[1]+1e-9),None)
    if chosen is None: raise ValueError('Reference pose has no feasible gas-spring interval on red branch')
    return chosen,dict(geometric_min_length_m=math.sqrt(max(0,a-amplitude)),
        geometric_max_length_m=math.sqrt(a+amplitude),
        compression_stop_reachable=math.sqrt(max(0,a-amplitude))-1e-10<=s['min_length_m']<=math.sqrt(a+amplitude)+1e-10,
        branch_margin_rad=branch_margin_rad,feasible_intervals_rad=feasible,
        endpoint_lengths_m=[geometry(q,s)[0] for q in chosen],
        note='No linkage collision restriction. Unreachable compression endpoint does not impose a knee stop.')


def axial_force(length, length_rate, s):
    if s.get("force_model", "polytropic") == "nominal_constant":
        return s["reference_force_n"] - s["viscous_damping_ns_m"] * length_rate
    area = math.pi * s["rod_diameter_m"]**2 / 4
    v0 = s["reference_gas_volume_m3"]
    volume = v0 + area * (length - s["reference_length_m"])
    p0 = s["reference_force_n"] / area + s["ambient_pressure_pa"]
    pressure = p0 * (v0 / volume)**s["polytropic_exponent"]
    return (pressure - s["ambient_pressure_pa"])*area - s["viscous_damping_ns_m"]*length_rate


def evaluate(q, qd, s, ops=math):
    length, jac = geometry(q, s, ops)
    # Explicit interval guards supplement the simulator's hinge-stop constraints.
    if s.get("min_length_m") is not None:
        tolerance = s.get('length_limit_tolerance_m', 0.) if s.get('force_model') == 'nominal_constant' else 0.
        invalid = (length < s["min_length_m"]-tolerance) | (length > s["max_length_m"]+tolerance) | (length < 1e-8)
        outside = bool(invalid.any()) if hasattr(invalid, "any") else bool(invalid)
        if outside:
            raise ValueError(f"{s['joint']}: spring outside calibrated stroke; check anchors/limits. "
                             "Check configured joint stops; refusing silent extrapolation.")
    rate = jac * qd
    force = axial_force(length, rate, s)
    # Virtual work: positive extension force does work F*d(length).
    return force * jac, length, force


class GasSpringBank:
    def __init__(self, cfg, dof_names):
        self.springs = cfg["springs"] if cfg["enabled"] else []
        self.indices = [list(dof_names).index(s["joint"]) for s in self.springs]

    def compute(self, q, qd):
        import torch
        torque = torch.zeros_like(q)
        for idx, s in zip(self.indices, self.springs):
            value, _, _ = evaluate(q[:, idx], qd[:, idx], s, torch)
            torque[:, idx] += value
        return torque


def summarize(cfg, parameters):
    """Dependency-free nominal posture and joint-range diagnostics."""
    rows = []
    for s in cfg["springs"] if cfg["enabled"] else []:
        name = s["joint"]
        q = parameters["default_joint_angles_rad"][name]
        torque, length, force = evaluate(q, 0., s)
        lo, hi = parameters["joint_limits_rad"][name]
        scan = [evaluate(lo + (hi-lo)*i/200, 0., s) for i in range(201)]
        rows.append(dict(joint=name, force_model=s["force_model"],
                         default_length_m=length, default_force_n=force,
                         default_passive_torque_nm=torque,
                         joint_range_length_m=[min(x[1] for x in scan), max(x[1] for x in scan)],
                         joint_range_torque_nm=[min(x[0] for x in scan), max(x[0] for x in scan)],
                         stroke_enforced=s.get("min_length_m") is not None,
                         installed_length_limits_m=[s.get('min_length_m'),s.get('max_length_m')],
                         joint_stop_limits_rad=s.get('joint_stop_limits_rad'),
                         numerical_length_tolerance_m=s.get('length_limit_tolerance_m',0.)))
    return dict(enabled=cfg["enabled"], geometry_confirmed=cfg.get("geometry_confirmed", False),
                geometry_approximation=cfg.get("geometry_approximation"), springs=rows)
