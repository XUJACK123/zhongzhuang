"""Shared observation geometry for native and MuJoCo reference tracking.

Position/velocity are simulator estimates here. Hardware deployment needs a
validated odometry/state estimator; these are not claimed as direct IMU signals.
"""


def yaw_xyzw(quat, ops):
    x,y,z,w = [quat[...,i] for i in range(4)]
    return ops.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def reversal_command(time_s):
    """40-second start/stop and sign-reversal test, straight heading reference."""
    return [0.,.15,0.,-.15,0.,.3,-.3,0.][min(7,int((time_s+1e-6)//5))]


def turning_command(time_s):
    """40-second left/right/stop/arc transition protocol, (m/s, rad/s)."""
    return [(0.,0.),(0.,.3),(0.,0.),(0.,-.3),(.15,.3),(.15,-.3),(-.15,.3),(0.,0.)][min(7,int((time_s+1e-6)//5))]


def wrap_angle(angle, ops):
    return ops.arctan2(ops.sin(angle),ops.cos(angle))


def bottom_clearance(root_height, projected_gravity, box_center, box_halfsize, ops):
    """Lowest point of an axis-aligned chassis-local box above a horizontal floor.

    Projected gravity is R.T @ (0,0,-1), so its negative is the world-Z row
    of the chassis rotation. Include tilt and box offset; root height alone is
    not the requested bottom clearance.
    """
    up=-projected_gravity
    return root_height+(up*box_center).sum(-1)-(ops.abs(up)*box_halfsize).sum(-1)


def root_velocity_command(forward, yaw_rate, offset_xy, ops):
    """Root-frame velocity when commands address a fixed chassis-local point.

    If C = P + R d and C rolls forward, Pdot = R ([v, 0] - omega J d).
    The offset is fixed configuration, never inferred from measured drift.
    """
    return ops.stack((forward+yaw_rate*offset_xy[1],-yaw_rate*offset_xy[0]),-1)


def advance_reference(position_xy, heading, forward, yaw_rate, dt, ops, offset_xy=None):
    """Exact constant-twist SE(2) integration, independent of measured motion.

    NumPy and torch sinc are normalized; this form also has a finite derivative
    at zero yaw rate. Return new values without mutating caller-owned state.
    """
    half=yaw_rate*dt/2
    distance=forward*dt*ops.sinc(half/3.141592653589793)
    direction=heading+half
    delta=ops.stack((distance*ops.cos(direction),distance*ops.sin(direction)),-1)
    new_heading=wrap_angle(heading+2*half,ops)
    if offset_xy is not None:
        # Integrate the commanded point, then transform back to the root. Both
        # rotations use commanded headings, so actual drift cannot move the goal.
        dc=ops.cos(heading)-ops.cos(new_heading)
        ds=ops.sin(heading)-ops.sin(new_heading)
        delta=delta+ops.stack((dc*offset_xy[0]-ds*offset_xy[1],
                             ds*offset_xy[0]+dc*offset_xy[1]),-1)
    return position_xy+delta,new_heading


def reference_features(position_xy, reference_xy, quat_xyzw, body_velocity_xy, ops, reference_yaw=None):
    yaw = yaw_xyzw(quat_xyzw, ops)
    c,s = ops.cos(yaw), ops.sin(yaw)
    error = reference_xy-position_xy
    angle=-yaw if reference_yaw is None else reference_yaw-yaw
    return ops.stack((ops.clip(5*(c*error[...,0]+s*error[...,1]),-2,2),
                      ops.clip(5*(-s*error[...,0]+c*error[...,1]),-2,2),
                      ops.sin(angle), ops.cos(angle)-1,
                      2*body_velocity_xy[...,0],2*body_velocity_xy[...,1]),-1)
