"""Versioned, optional training curriculum; hardware and failure checks stay intact."""
import json
import math
import xml.etree.ElementTree as ET
from command_sampling import endpoint_probability
from action_limits import validate_wheel_limit,validate_hip_limit
from pose_feedback_reward import validate as validate_pose_feedback
from random_pulses import validate as validate_random_pulses


def apply_profile(path, cfg, ppo):
    if not path.exists():
        return None
    p = json.loads(path.read_text(encoding='utf-8'))
    if p['task'] not in ('standing','hold_and_track','ground_getup'):
        raise ValueError('Unsupported training profile')
    wheel_limit=validate_wheel_limit(p.get('wheel_action_limit',1.))
    hip_limit=validate_hip_limit(p.get('hip_action_limit',1.))
    cfg.control.wheel_action_limit=wheel_limit
    cfg.control.hip_action_limit=hip_limit
    cfg.normalization.clip_actions=max(wheel_limit,hip_limit)
    deferred=p.get('defer_loss_statistics',False)
    if type(deferred) is not bool:
        raise ValueError('Deferred loss statistics must be boolean')
    ppo.algorithm.defer_loss_statistics=deferred
    if 'checkpoint_interval_updates' in p:
        interval=p['checkpoint_interval_updates']
        if type(interval) is not int or interval<1:raise ValueError('Checkpoint interval must be a positive integer')
        ppo.runner.save_interval=interval
    if 'entropy_coef' in p:
        entropy = p['entropy_coef']
        if type(entropy) not in (int, float) or not math.isfinite(entropy) or entropy < 0:
            raise ValueError('Entropy coefficient must be finite and nonnegative')
        ppo.algorithm.entropy_coef = float(entropy)
    sides=p.get('native_cylinder_sides')
    if sides is not None and (type(sides) is not int or not 3<=sides<=32):
        raise ValueError('Native cylinder sides must be an integer from 3 to 32 for GPU-compatible hulls')
    # Omitted on historical profiles: retain the installed importer's default.
    cfg.asset.slices_per_cylinder=sides
    cfg.init_state.pos = [0., 0., p['height_m']]
    cfg.init_state.default_joint_angles = p['joint_angles_rad']
    cfg.env.initial_velocity_noise = p['initial_velocity_noise']
    if 'episode_length_s' in p:
        duration=p['episode_length_s']
        if type(duration) not in (int,float) or not math.isfinite(duration) or duration<=0:
            raise ValueError('Episode duration must be positive and finite')
        cfg.env.episode_length_s=float(duration)
    hold=p.get('motion_hold_resamples',1)
    if type(hold) is not int or hold<1:
        raise ValueError('Motion hold must be a positive integer resample count')
    if hold>1:
        if p['task']!='hold_and_track' or not p.get('se2_reference'):
            raise ValueError('Long motion holds require SE2 hold-and-track training')
        if cfg.env.episode_length_s<hold*cfg.commands.resampling_time:
            raise ValueError('Episode must contain a full motion hold')
    cfg.env.motion_hold_resamples=hold
    mixed=p.get('mixed_motion_holds',False)
    if type(mixed) is not bool or (mixed and hold<=1):
        raise ValueError('Mixed motion holds require a boolean option and a long hold')
    cfg.env.mixed_motion_holds=mixed
    cfg.env.command_endpoint_probability = endpoint_probability(p.get('command_endpoint_probability', 0.))
    cfg.env.recovery_task=p.get('recovery_task')
    cfg.env.wheel_alignment=p.get('wheel_alignment')
    if p['task']=='ground_getup' and not cfg.env.recovery_task:raise ValueError('Ground get-up criteria required')
    if cfg.env.recovery_task:
        from recovery_task import validate
        validate(cfg.env.recovery_task)
        if p['task']!='ground_getup' or not p.get('clearance_control'):
            raise ValueError('Recovery requires the explicit ground get-up task')
        cfg.env.episode_length_s=p.get('episode_length_s',5.)
        if cfg.env.episode_length_s<=cfg.env.recovery_task['deadline_s']+.5:
            raise ValueError('Get-up episodes need a post-rise stability horizon')
    if cfg.env.wheel_alignment:
        if not 0<cfg.env.wheel_alignment.get('scale_m',.02)<=.1 or cfg.env.wheel_alignment.get('settle_s',.5)<0:
            raise ValueError('Invalid wheel alignment scale or settling time')
    cfg.commands.ranges.lin_vel_x = p.get('forward_command_range',[0., 0.])
    cfg.commands.heading_command = False
    cfg.commands.ranges.ang_vel_yaw = [0., 0.]
    cfg.env.se2_reference=p.get('se2_reference',False)
    cfg.env.pose_feedback_reward=validate_pose_feedback(p.get('pose_feedback_reward'))
    if cfg.env.pose_feedback_reward:
        feedback=cfg.env.pose_feedback_reward
        if not cfg.env.se2_reference:raise ValueError('Pose-feedback rewards require SE2 references')
        if feedback['speed_limit_m_s']<max(abs(v) for v in cfg.commands.ranges.lin_vel_x):
            raise ValueError('Feedback target limit must cover user speed commands')
        if feedback['yaw_limit_rad_s']<max(abs(v) for v in p.get('yaw_command_range',[0.,0.])):
            raise ValueError('Feedback target limit must cover user yaw commands')
    elif p['reward_scales'].get('pose_yaw_rate',0):
        raise ValueError('Pose yaw reward requires explicit feedback settings')
    offset=p.get('motion_reference_offset_xy_m')
    if offset is not None:
        if not cfg.env.se2_reference or len(offset)!=2 or not all(math.isfinite(x) for x in offset):
            raise ValueError('A finite 2D command-point offset requires SE2 tracking')
    cfg.env.motion_reference_offset_xy_m=offset
    if cfg.env.se2_reference:
        if not p.get('reference_observations',False):raise ValueError('SE2 tracking requires reference observations')
        cfg.commands.ranges.ang_vel_yaw=p.get('yaw_command_range',[0.,0.])
        fractions=[p.get(name,0.) for name in ('standing_fraction','straight_fraction','turn_fraction')]
        if any(not 0<=x<=1 for x in fractions) or sum(fractions)>1:
            raise ValueError('Command-mode fractions must be nonnegative and sum to at most one')
    cfg.env.straight_fraction=p.get('straight_fraction',0.)
    cfg.env.turn_fraction=p.get('turn_fraction',0.)
    rates=p.get('command_slew_rates')
    if rates is not None:
        if len(rates)!=3 or any(not math.isfinite(v) or v<=0 for v in rates):
            raise ValueError('Three positive finite command slew rates required')
        cfg.env.command_slew_rates=list(rates)
    training_slew=p.get('training_command_slew',False)
    if type(training_slew) is not bool or (training_slew and
            (rates is None or p['task']!='hold_and_track' or not cfg.env.se2_reference)):
        raise ValueError('Training command slew requires explicit rates and SE2 hold-and-track')
    cfg.env.training_command_slew=training_slew
    cfg.commands.ranges.height = [p['height_m'], p['height_m']]
    cfg.rewards.base_height_target = p['height_m']
    cfg.env.clearance_control=p.get('clearance_control',False)
    if cfg.env.clearance_control:
        if not p.get('reference_observations',False):raise ValueError('Clearance control requires the archived tracking-state implementation')
        base=ET.parse(path.parent.parent/'urdf/fifty_training.urdf').find("link[@name='base_link']")
        collisions=base.findall('collision')
        if len(collisions)!=1 or collisions[0].find('geometry/box') is None:
            raise ValueError('Clearance control currently requires the audited single chassis box')
        origin=collisions[0].find('origin')
        if any(abs(float(x))>1e-12 for x in origin.get('rpy','0 0 0').split()):
            raise ValueError('Chassis-local collision box must be axis aligned')
        cfg.env.clearance_box_center=[float(x) for x in origin.get('xyz','0 0 0').split()]
        cfg.env.clearance_box_halfsize=[float(x)/2 for x in collisions[0].find('geometry/box').get('size').split()]
        cfg.commands.ranges.height=p.get('clearance_range_m',[p['clearance_m']]*2)
        if not 0<cfg.commands.ranges.height[0]<=p['clearance_m']<=cfg.commands.ranges.height[1]:
            raise ValueError('Invalid bottom-clearance target/range')
        cfg.rewards.base_height_target=p['clearance_m']
    cfg.terrain.mesh_type = 'plane'
    cfg.terrain.curriculum = False
    cfg.noise.add_noise = False
    for name in dir(cfg.domain_rand):
        if name.startswith('randomize_') or name == 'push_robots':
            if isinstance(getattr(cfg.domain_rand, name), bool):
                setattr(cfg.domain_rand, name, False)
    from training_com import configure as configure_com
    configure_com(cfg,p.get('com_randomization'))
    from training_parameters import configure as configure_parameters
    configure_parameters(cfg,p.get('parameter_randomization'))
    for name in dir(cfg.rewards.scales):
        if not name.startswith('_'):
            setattr(cfg.rewards.scales, name, 0.)
    for name, value in p['reward_scales'].items():
        setattr(cfg.rewards.scales, name, value)
    cfg.rewards.only_positive_rewards = False
    moving_weight=p.get('moving_yaw_tracking_weight',1.)
    if type(moving_weight) not in (int,float) or not math.isfinite(moving_weight) or not 0<moving_weight<=1:
        raise ValueError('Moving-yaw tracking weight must be finite in (0,1]')
    if moving_weight!=1 and not cfg.env.se2_reference:
        raise ValueError('Moving-yaw priority requires SE2 commands')
    cfg.rewards.moving_yaw_tracking_weight=moving_weight
    cfg.rewards.clip_single_reward = p.get('clip_single_reward',1.)
    cfg.rewards.upright_sharpness = p.get('upright_sharpness',10.)
    distance_scale=p.get('reference_position_error_scale_m',.1)
    if type(distance_scale) not in (int,float) or not math.isfinite(distance_scale) or distance_scale<=0:
        raise ValueError('Reference position error scale must be positive and finite')
    cfg.rewards.reference_position_error_scale_m=distance_scale
    if 'height_tracking_sigma_m' in p:
        if not math.isfinite(p['height_tracking_sigma_m']) or p['height_tracking_sigma_m']<=0:
            raise ValueError('Positive finite height tracking scale required')
        cfg.rewards.height_tracking_variance_m2=p['height_tracking_sigma_m']**2
    dense=p.get('height_tracking_dense_weight_m_inv',0.)
    if type(dense) not in (int,float) or not math.isfinite(dense) or dense<0:
        raise ValueError('Dense clearance weight must be finite and nonnegative')
    if dense and (not cfg.env.clearance_control or p['reward_scales'].get('base_height',0.)<=0):
        raise ValueError('Dense clearance shaping requires clearance control and positive base-height scale')
    cfg.rewards.height_tracking_dense_weight_m_inv=float(dense)
    cfg.env.reference_observations = p.get('reference_observations',False)
    if cfg.env.reference_observations:
        cfg.env.num_observations = 31
        cfg.env.num_privileged_obs = 147
    cfg.env.push_force_n = p.get('push_force_n',0.)
    cfg.env.push_yaw_torque_nm = p.get('push_yaw_torque_nm',0.)
    cfg.env.push_interval_s = p.get('push_interval_s',5.)
    cfg.env.push_duration_s = p.get('push_duration_s',.2)
    cfg.env.random_pulses=validate_random_pulses(p.get('random_pulses'))
    cfg.env.standing_fraction = p.get('standing_fraction',0.)
    cfg.control.damping = dict(cfg.control.damping, wheel=p['wheel_velocity_kp'])
    cfg.control.vel_action_scale = p['wheel_velocity_action_scale']
    ppo.runner.policy_class_name = 'ActorCritic'
    ppo.policy.init_noise_std = p['policy_initial_std']
    return p
