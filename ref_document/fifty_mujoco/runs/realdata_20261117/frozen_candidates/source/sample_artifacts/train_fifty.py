#!/usr/bin/env python3
"""Standalone Isaac Gym task adapter for the generated `fifty_training` model.

Modes
-----
--validate-only   Dependency-free structural / kinematic check.  Does not
                  import Isaac Gym or torch.
--smoke-test      Load one environment with all randomization disabled and run
                  100 zero-action steps.  Prints DOF order, body count, masses,
                  contact indices, torque limits and a finiteness verdict.
--joint-test      Fixed base, one environment.  Sends a small command to every
                  joint in turn and records the resulting motion, so that the
                  index -> joint -> physical direction mapping is verified
                  before any training run.
--audit-test      Compare PD gains with gas springs in a suspended model,
                  check wheel motion and read back the actual collision masks.
(default)         Train with the Fudan PPO implementation. The task is registered as
                  ``fifty_wheel_legged``. The shared simulator has an optional
                  passive-force hook, inactive for other tasks.

The model, configuration and this script are copied into the run directory so
that a checkpoint can always be traced back to the exact URDF that produced it.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from gas_spring import GasSpringBank, load_config, summarize
from actuator_model import CoupledActuators, load_hardware, validate_hardware_asset
from training_checks import adjacent_collision_masks, failures, position_targets, validate_knee_branch, knee_branch_violation, periodic_hip_error
from recovery_task import initial_state as recovery_initial_state, advance as recovery_advance
from recovery_geometry import RecoveryGeometry
from wheel_alignment import foreaft_offset, eligible as alignment_eligible, penalty as alignment_penalty
from training_com import signature as body_signature, reserve_nominal, audit as audit_com
import training_parameters
from command_sampling import apply_endpoints
from moving_yaw_rewards import tracking_weight
from action_limits import clip as clip_motor_actions
from random_pulses import RandomPulses
from training_commands import TrainingCommands

PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = json.loads((PACKAGE / "config" / "model_parameters.json").read_text(encoding="utf-8"))
URDF = PACKAGE / "urdf" / "fifty_training.urdf"
TASK = "fifty_wheel_legged"

# Provisional: both leg motors sit in the chassis (five-bar parallel leg), so
# the virtual knee has almost no rotor inertia in this equivalent model.  Fill
# in the reflected inertia once the motor / gearbox data is available.
DOF_ARMATURE = {"f0": 0.0, "f1": 0.0, "wheel": 0.0}


def main():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--fudan-root", type=Path,
        default=PACKAGE.parent.parent / "ref_document/fudan_rl_wheel_leg-main")
    parser.add_argument("--validate-only", action="store_true")
    parser.add_argument("--smoke-test", action="store_true")
    parser.add_argument("--joint-test", action="store_true")
    parser.add_argument("--audit-test", action="store_true", help="Fixed-base collision and PD diagnostic")
    parser.add_argument('--com-audit-output',type=Path,help='Read back COM distribution and unchanged mass/inertia without PPO')
    parser.add_argument('--benchmark-output',type=Path,help='Measure inference/physics throughput without PPO updates')
    parser.add_argument('--random-pulse-audit-output',type=Path,help='Frozen-policy training-force audit without PPO')
    parser.add_argument('--command-ramp-audit-output',type=Path,help='Frozen-policy training command lifecycle audit without PPO')
    parser.add_argument('--resume-checkpoint',type=Path,help='Restore policy and both optimizers from an explicit checkpoint')
    parser.add_argument('--torch-threads',type=int)
    parser.add_argument('--ppo-mini-batches',type=int)
    parser.add_argument('--evaluate-output', type=Path)
    parser.add_argument('--evaluate-seconds', type=float, default=20.)
    parser.add_argument('--evaluate-zero', action='store_true')
    parser.add_argument('--evaluation-noise', type=float, default=0.05)
    parser.add_argument('--evaluate-tracking', action='store_true')
    parser.add_argument('--evaluation-command', type=float, default=0.)
    parser.add_argument('--evaluation-push-force', type=float, default=0.)
    parser.add_argument('--evaluation-yaw-torque', type=float, default=0.)
    parser.add_argument('--evaluation-reversals',action='store_true')
    parser.add_argument('--evaluation-yaw-command',type=float,default=0.)
    parser.add_argument('--evaluation-turn-transitions',action='store_true')
    parser.add_argument('--evaluation-clearance',type=float,help='Fixed bottom-clearance target within the archived range')
    parser.add_argument('--evaluation-height-cycle',action='store_true',help='Slew through the archived clearance range')
    spring_switch = parser.add_mutually_exclusive_group()
    spring_switch.add_argument("--gas-spring", dest="spring_enabled", action="store_true",
                               help="Enable gas springs (default follows config)")
    spring_switch.add_argument("--no-gas-spring", dest="spring_enabled", action="store_false",
                               help="Disable gas springs for an A/B comparison")
    parser.set_defaults(spring_enabled=None)
    parser.add_argument("--gas-spring-config", type=Path,
                        default=PACKAGE / "config/gas_spring.json")
    parser.add_argument("--check-gas-spring", action="store_true",
                        help="Validate enabled spring parameters without Isaac Gym")
    local, remaining = parser.parse_known_args()
    height_requested=local.evaluation_clearance is not None or local.evaluation_height_cycle
    if height_requested and (not local.evaluate_tracking or not local.evaluate_output):
        raise ValueError('Height evaluation requires --evaluate-tracking and --evaluate-output')
    if local.evaluation_reversals and (not local.evaluate_tracking or local.evaluate_seconds<40):
        raise ValueError('Reversal evaluation needs --evaluate-tracking and at least 40 seconds')
    if local.evaluation_turn_transitions and (local.evaluation_reversals or not local.evaluate_tracking or local.evaluate_seconds<40):
        raise ValueError('Turning transitions need separate tracking evaluation of at least 40 seconds')

    validate_knee_branch(PARAMS, URDF)
    if not PARAMS.get('chassis_mounting',{}).get('applied_to_training_asset'):
        raise ValueError('Rebuild the training asset with the confirmed 180-degree chassis mounting before training')
    hardware_cfg=load_hardware()
    validate_hardware_asset(PARAMS,URDF,hardware_cfg)
    spring_cfg = load_config(local.gas_spring_config, URDF, enable=local.spring_enabled)
    spring_report = summarize(spring_cfg, PARAMS)
    if local.check_gas_spring:
        print(json.dumps(spring_report, indent=2, ensure_ascii=False))
        return 0

    if local.validate_only:
        sys.path.insert(0, str(PACKAGE / "integration"))
        from validate_model import main as validate_main
        return validate_main()

    plane = local.fudan_root.resolve() / "plane"
    if not (plane / "wheel_legged_gym/envs/base/legged_robot.py").is_file():
        raise SystemExit("Pass --fudan-root pointing at fudan_rl_wheel_leg-main.")
    sys.path.insert(0, str(plane))
    sys.path.insert(0, str(plane / "rsl_rl"))
    sys.argv = [sys.argv[0]] + remaining

    import isaacgym  # noqa: F401  (must precede torch)
    from isaacgym import gymtorch, gymapi
    from isaacgym.torch_utils import quat_rotate_inverse
    import torch
    if local.torch_threads is not None:
        if local.torch_threads<1:raise ValueError('torch-threads must be positive')
        torch.set_num_threads(local.torch_threads)
    from wheel_legged_gym.envs.base.legged_robot import LeggedRobot
    from wheel_legged_gym.envs.wheel_legged.wheel_legged_config import (
        WheelLeggedCfg, WheelLeggedCfgPPO)
    from wheel_legged_gym.utils import get_args, task_registry
    from wheel_legged_gym.utils.helpers import class_to_dict
    from tracking_state import reference_features, yaw_xyzw, wrap_angle, advance_reference, bottom_clearance, root_velocity_command
    from pose_feedback_reward import targets as pose_feedback_targets

    stance = PARAMS["spawn_height_m"] - 0.010

    class FiftyCfg(WheelLeggedCfg):
        class env(WheelLeggedCfg.env):
            num_envs = 4096
            fail_to_terminal_time_s = 0.10

        class termination:
            max_tilt_deg = 60.0
            min_base_height_m = 0.15
            contact_force_n = 10.0

        class terrain(WheelLeggedCfg.terrain):
            curriculum = False
            restitution = 0.0

        class domain_rand(WheelLeggedCfg.domain_rand):
            restitution_range = [0.0, 0.1]

        class normalization(WheelLeggedCfg.normalization):
            clip_actions = 1.0

        class sim(WheelLeggedCfg.sim):
            substeps = 2

            class physx(WheelLeggedCfg.sim.physx):
                contact_offset = 0.002
                num_position_iterations = 8
                num_velocity_iterations = 1

        class init_state(WheelLeggedCfg.init_state):
            pos = [0.0, 0.0, PARAMS["spawn_height_m"]]
            rot = [0.0, 0.0, 0.0, 1.0]
            default_joint_angles = dict(PARAMS["default_joint_angles_rad"])

        class control(WheelLeggedCfg.control):
            pos_action_scale = 0.5
            vel_action_scale = 10.0
            stiffness = {"f0": 40.0, "f1": 60.0, "wheel": 0.0}
            damping = {"f0": 2.0, "f1": 2.0, "wheel": 0.05}

        class asset(WheelLeggedCfg.asset):
            file = str(URDF)
            name = "fifty"
            foot_name = "wheel"
            l1 = PARAMS["leg_dimensions"]["l"]["thigh_planar_m"]
            l2 = PARAMS["leg_dimensions"]["l"]["shin_planar_m"]
            offset = 0.0
            dof_armature = dict(DOF_ARMATURE)
            replace_cylinder_with_capsule = False
            # STL origins already transform SW frames into our Z-up frames.
            # The importer flip converts mesh conventions; it is not a
            # self-collision switch and must not be enabled for these meshes.
            flip_visual_attachments = False
            fix_base_link = False
            self_collisions = 0
            recompute_inertia = False
            thickness = 0.001
            penalize_contacts_on = ["base_link", "0_link", "1_link"]
            terminate_after_contacts_on = ["base_link"]
            collapse_fixed_joints = False

        class commands(WheelLeggedCfg.commands):
            curriculum = False

            class ranges(WheelLeggedCfg.commands.ranges):
                lin_vel_x = [-0.3, 0.3]
                ang_vel_yaw = [-0.4, 0.4]
                height = [stance - 0.03, stance + 0.01]

        class rewards(WheelLeggedCfg.rewards):
            base_height_target = stance

    class FiftyCfgPPO(WheelLeggedCfgPPO):
        class runner(WheelLeggedCfgPPO.runner):
            experiment_name = "fifty_wheel_legged"
            run_name = "six_dof"
            resume = False

    class FiftyRobot(LeggedRobot):
        def step(self,actions):
            wheel_limit=getattr(self.cfg.control,'wheel_action_limit',1.)
            hip_limit=getattr(self.cfg.control,'hip_action_limit',1.)
            if wheel_limit!=1. or hip_limit!=1.:
                actions=clip_motor_actions(torch,actions,wheel_limit,hip_limit)
            return super().step(actions)

        def reset_idx(self,env_ids):
            if len(env_ids)==0:return super().reset_idx(env_ids)
            if getattr(self,'random_pulses',None) is not None:self.random_pulses.reset(env_ids)
            alignment=getattr(self.cfg.env,'wheel_alignment',None)
            recovery=getattr(self.cfg.env,'recovery_task',None)
            if recovery and hasattr(self,'recovery_state'):
                first=self.recovery_state['first_stable_s'][env_ids]
                success=(first>=0)&(first<=recovery['deadline_s']+1e-6)
                success_rate=success.float().mean()
                success_time=torch.where(success,first,torch.zeros_like(first)).sum()/success.sum().clamp_min(1)
                compressed=self.recovery_state['compressed'][env_ids].float().mean()
            if alignment and hasattr(self,'alignment_count'):
                count=self.alignment_count[env_ids].sum().clamp_min(1)
                rms=(self.alignment_squared[env_ids].sum()/count).sqrt()*1000
                peak=self.alignment_peak[env_ids].max()*1000
                fraction=self.alignment_count[env_ids].sum()/self.episode_length_buf[env_ids].sum().clamp_min(1)
            was_resetting=getattr(self,'_resetting_commands',False)
            self._resetting_commands=True
            try:
                super().reset_idx(env_ids)
            finally:
                self._resetting_commands=was_resetting
            if recovery and hasattr(self,'recovery_state'):
                self.extras['episode'].update(getup_within_deadline_rate=success_rate,getup_success_time_s=success_time)
                if recovery.get('compression_before_hip'):self.extras['episode']['getup_compressed_rate']=compressed
            if alignment and hasattr(self,'alignment_count'):
                self.extras['episode'].update(wheel_alignment_rms_mm=rms,wheel_alignment_peak_mm=peak,
                    wheel_alignment_eligible_fraction=fraction)
                for value in (self.alignment_count,self.alignment_squared,self.alignment_peak):value[env_ids]=0

        def _reset_root_states(self, env_ids):
            noise = getattr(self.cfg.env, 'initial_velocity_noise', None)
            if noise is None:
                return super()._reset_root_states(env_ids)
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
            self.root_states[env_ids, 7:13] = 0.
            # Only horizontal/rotational perturbations; wheels start on the plane.
            perturb = (torch.rand((len(env_ids), 5), device=self.device)*2-1)*noise
            self.root_states[env_ids, 7:9] = perturb[:, :2]
            self.root_states[env_ids, 10:13] = perturb[:, 2:]
            ids = env_ids.to(torch.int32)
            self.gym.set_actor_root_state_tensor_indexed(self.sim,
                gymtorch.unwrap_tensor(self.root_states), gymtorch.unwrap_tensor(ids), len(ids))
            self.base_quat[env_ids] = self.root_states[env_ids, 3:7]
            self.base_lin_vel[env_ids] = quat_rotate_inverse(self.base_quat[env_ids], self.root_states[env_ids, 7:10])
            self.base_ang_vel[env_ids] = quat_rotate_inverse(self.base_quat[env_ids], self.root_states[env_ids, 10:13])
            self.projected_gravity[env_ids] = quat_rotate_inverse(self.base_quat[env_ids], self.gravity_vec[env_ids])
            self.base_height[env_ids] = self.cfg.init_state.pos[2]
            self.actions[env_ids] = 0.
            if hasattr(self,'command_unchanged_s'):self.command_unchanged_s[env_ids]=0
            if getattr(self.cfg.env,'recovery_task',None):
                self.recovery_state['released'][env_ids]=False
                self.recovery_state['hold_s'][env_ids]=0
                self.recovery_state['first_stable_s'][env_ids]=-1
                self.recovery_state['compressed'][env_ids]=False
                self.recovery_state['compression_hold_s'][env_ids]=0
                self.recovery_state['first_compressed_s'][env_ids]=-1
            if hasattr(self,'position_reference'):
                self.position_reference[env_ids] = self.root_states[env_ids,:2]
                self.heading_reference[env_ids] = yaw_xyzw(self.root_states[env_ids,3:7],torch)

        def _post_physics_step_callback(self):
            if getattr(self.cfg.env,'reference_observations',False):
                if getattr(self.cfg.env,'se2_reference',False):
                    self.position_reference,self.heading_reference=advance_reference(
                        self.position_reference,self.heading_reference,self.commands[:,0],self.commands[:,1],self.dt,torch,
                        offset_xy=getattr(self.cfg.env,'motion_reference_offset_xy_m',None))
                else:
                    self.position_reference[:,0] += self.commands[:,0]*self.dt
            super()._post_physics_step_callback()
            if self.training_commands is not None:
                self.commands[:,:3].copy_(self.training_commands.advance(self.dt))
            changed=(torch.abs(self.commands[:,:3]-self.previous_commands)>1e-5).any(dim=1)
            self.command_unchanged_s=torch.where(changed,torch.zeros_like(self.command_unchanged_s),self.command_unchanged_s+self.dt)
            self.previous_commands.copy_(self.commands[:,:3])

        def _resample_commands(self, env_ids):
            command_ramp=getattr(self,'training_commands',None)
            preserve=None
            hold=getattr(self.cfg.env,'motion_hold_resamples',1)
            if hold>1 and len(env_ids) and not getattr(self,'_resetting_commands',False):
                interval=int(self.cfg.commands.resampling_time/self.dt)*hold
                preserve=(self.episode_length_buf[env_ids]%interval)!=0
                if getattr(self.cfg.env,'mixed_motion_holds',False):
                    # Alternate groups of four: both cohorts retain the nominal COM quarter.
                    preserve &= ((env_ids//4)%2)==0
                source=command_ramp.requested if command_ramp is not None else self.commands
                old_motion=source[env_ids,:2].clone()
            super()._resample_commands(env_ids)
            apply_endpoints(self.commands, env_ids, self.command_ranges,
                            getattr(self.cfg.env, 'command_endpoint_probability', 0.), torch)
            fraction = getattr(self.cfg.env,'standing_fraction',0.)
            if getattr(self.cfg.env,'se2_reference',False) and len(env_ids):
                mode=torch.rand(len(env_ids),device=self.device)
                straight=getattr(self.cfg.env,'straight_fraction',0.)
                turn=getattr(self.cfg.env,'turn_fraction',0.)
                self.commands[env_ids[mode<fraction],:2]=0.
                self.commands[env_ids[(mode>=fraction)&(mode<fraction+straight)],1]=0.
                self.commands[env_ids[(mode>=fraction+straight)&(mode<fraction+straight+turn)],0]=0.
            elif fraction and len(env_ids):
                stopped = env_ids[torch.rand(len(env_ids),device=self.device)<fraction]
                self.commands[stopped,0] = 0.
            if preserve is not None:
                self.commands[env_ids[preserve],:2]=old_motion[preserve]
            if command_ramp is not None:
                command_ramp.request(env_ids,self.commands[env_ids,:3],
                                     reset=getattr(self,'_resetting_commands',False))
                self.commands[env_ids,:3]=command_ramp.applied[env_ids]

        def _reward_reference_position(self):
            error = (self.base_position[:,:2]-self.position_reference).square().sum(1)
            reward = torch.exp(-error/.01)
            factor = getattr(getattr(self.cfg,'rewards',None),'moving_yaw_tracking_weight',1.)
            if factor != 1.: reward = reward * tracking_weight(self.commands,factor,torch)
            return reward

        def _reward_reference_position_error(self):
            error = (self.base_position[:,:2]-self.position_reference).square().sum(1)
            reward = error/self.cfg.rewards.reference_position_error_scale_m**2
            factor = getattr(self.cfg.rewards,'moving_yaw_tracking_weight',1.)
            if factor != 1.: reward = reward * tracking_weight(self.commands,factor,torch)
            return reward

        def _reward_heading(self):
            yaw = yaw_xyzw(self.base_quat,torch)
            if getattr(self.cfg.env,'se2_reference',False):
                yaw=wrap_angle(yaw-self.heading_reference,torch)
            return torch.exp(-yaw.square()/.015)

        def _reward_reference_velocity(self):
            offset=getattr(self.cfg.env,'motion_reference_offset_xy_m',None)
            if getattr(self.cfg.env,'pose_feedback_reward',None):
                forward,yaw_rate=self._pose_velocity_target()
                desired=root_velocity_command(forward,yaw_rate,offset or [0.,0.],torch)
                error = (self.base_lin_vel[:,:2]-desired).square().sum(1)
            elif offset is None:
                error = (self.base_lin_vel[:,0]-self.commands[:,0]).square()+self.base_lin_vel[:,1].square()
            else:
                desired=root_velocity_command(self.commands[:,0],self.commands[:,1],offset,torch)
                error=(self.base_lin_vel[:,:2]-desired).square().sum(1)
            reward = torch.exp(-error/.01)
            factor = getattr(getattr(self.cfg,'rewards',None),'moving_yaw_tracking_weight',1.)
            if factor != 1.: reward = reward * tracking_weight(self.commands,factor,torch)
            return reward

        def _pose_velocity_target(self):
            return pose_feedback_targets(self.commands[:,0],self.commands[:,1],
                self.base_position[:,:2],self.position_reference,yaw_xyzw(self.base_quat,torch),
                self.heading_reference,self.cfg.env.pose_feedback_reward,torch)

        def _reward_pose_yaw_rate(self):
            _,desired=self._pose_velocity_target()
            sigma=self.cfg.env.pose_feedback_reward['yaw_sigma_rad_s']
            return torch.exp(-((self.base_ang_vel[:,2]-desired)/sigma).square())

        def _reward_yaw_rate(self):
            error=self.base_ang_vel[:,2]
            if getattr(self.cfg.env,'se2_reference',False):error=error-self.commands[:,1]
            return error.square()

        def _reward_wheel_action_smooth(self):
            # Optional wheel-target curvature cost; constant ramps remain free.
            change=self.actions[:,[2,5]]-2*self.last_actions[:,[2,5],0]+self.last_actions[:,[2,5],1]
            return change.square().sum(dim=1)

        def _bottom_clearance(self):
            return bottom_clearance(self.base_height,self.projected_gravity,self.clearance_box_center,self.clearance_box_halfsize,torch)

        def _reward_base_height(self):
            if not getattr(self.cfg.env,'clearance_control',False):return super()._reward_base_height()
            error=self._bottom_clearance()-self.commands[:,2]
            if self.reward_scales['base_height']<0:return error.abs()
            reward=torch.exp(-error.square()/getattr(self.cfg.rewards,'height_tracking_variance_m2',.001))
            dense=getattr(self.cfg.rewards,'height_tracking_dense_weight_m_inv',0.)
            # Optional training shaping: preserve a signal outside the narrow tracking band.
            # Clearance still comes from the existing tilted chassis box, not root height.
            return reward-dense*error.abs() if dense else reward

        def _reward_recovery_progress(self):
            return self.recovery_signals['progress']

        def _reward_recovery_stable(self):
            return self.recovery_signals['stable'].float()

        def _reward_recovery_compress(self):
            return self.recovery_signals['compression_progress']

        def _reward_recovery_axle(self):
            error=self.recovery_geometry_state['axle_com_error']
            return self.recovery_state['compressed'].float()*torch.exp(-error.square()/.03**2)

        def _reward_recovery_support(self):
            load=self.contact_forces[:,self.feet_indices,2].sum(1)/(self.fifty_mass*9.81)
            aligned=self.recovery_geometry_state['axle_com_error'].abs()<.04
            return self.recovery_state['compressed'].float()*aligned.float()*load.clamp(0,1)

        def _reward_recovery_early_hip(self):
            # Soft phase preference, not a joint range/torque limit; modest
            # contact-balancing hip corrections remain available while folding.
            excess=(self.dof_vel[:,[0,3]].abs()-1.5).clamp_min(0).square().mean(1)
            return (~self.recovery_state['compressed']).float()*excess

        def _reward_wheel_alignment(self):
            cfg=self.cfg.env.wheel_alignment
            offset=foreaft_offset(self.dof_pos,self.cfg.asset.l1,self.cfg.asset.l2,torch)
            mask=alignment_eligible(self.commands,self.command_unchanged_s,cfg,torch)
            if getattr(self.cfg.env,'recovery_task',None):mask &= self.recovery_state['first_stable_s']>=0
            self.alignment_count+=mask.float()
            self.alignment_squared+=torch.where(mask,offset.square(),torch.zeros_like(offset))
            self.alignment_peak=torch.maximum(self.alignment_peak,torch.where(mask,offset.abs(),torch.zeros_like(offset)))
            return alignment_penalty(offset,mask,cfg,torch)

        def _reward_alive(self):
            return torch.ones(self.num_envs, device=self.device)

        def _reward_upright(self):
            return torch.exp(-getattr(self.cfg.rewards,'upright_sharpness',10.)*(1.+self.projected_gravity[:, 2]))

        def _reward_stand_position(self):
            delta = self.base_position[:, :2]-self.env_origins[:, :2]
            return torch.exp(-delta.square().sum(1)/0.04)

        def _reward_stand_velocity(self):
            return torch.exp(-self.base_lin_vel[:, :2].square().sum(1)/0.04)

        def _process_dof_props(self, props, env_id):
            if env_id == 0:
                self.imported_dof_properties = {n: {key: props[key][i].item() for key in props.dtype.names}
                                                for i, n in enumerate(self.dof_names)}
            # The importer returned FLT_MAX stiffness for continuous wheels,
            # effectively locking them despite effort mode. Our PD is external.
            for i, name in enumerate(self.dof_names):
                props["stiffness"][i] = 0.0
                kind = "wheel" if "wheel" in name else ("hip" if "f0" in name else "knee")
                props["damping"][i] = PARAMS["joint_damping"][kind]
                if "wheel" in name or "f0" in name:
                    props["hasLimits"][i] = False
            return super()._process_dof_props(props, env_id)

        def compute_proprioception_observations(self):
            obs=super().compute_proprioception_observations()
            offset=periodic_hip_error(torch,self.dof_pos,self.default_dof_pos)
            obs[:,[9,11]]=offset[:,[0,3]]*self.obs_scales.dof_pos
            if getattr(self.cfg.env,'reference_observations',False):
                reference=self.heading_reference if getattr(self.cfg.env,'se2_reference',False) else None
                extra=reference_features(self.base_position[:,:2],self.position_reference,
                                         self.base_quat,self.base_lin_vel[:,:2],torch,reference_yaw=reference)
                obs=torch.cat((obs,extra),dim=1)
            return obs

        def _compute_torques(self, actions):
            target = position_targets(torch, actions, self.default_dof_pos,
                                      self.dof_pos_limits, self.cfg.control.pos_action_scale)
            velocity = actions * self.cfg.control.vel_action_scale
            velocity[:, [0, 1, 3, 4]] = 0.0
            raw = self.p_gains * periodic_hip_error(torch,target,self.dof_pos) + self.d_gains * (velocity-self.dof_vel)
            self.motor_torque_unclipped = raw * self.torques_scale
            return self.actuators.apply(self.motor_torque_unclipped,self.dof_pos,self.dof_vel,self.motor_output_gain)

        def check_termination(self):
            contact = torch.linalg.vector_norm(
                self.contact_forces[:, self.termination_contact_indices], dim=-1).amax(dim=1)
            fail, invalid = failures(torch, self.projected_gravity, self.base_height,
                                     contact, self.root_states, self.dof_state.flatten(1),
                                     self.cfg.termination)
            if bool(invalid.any()):
                raise FloatingPointError("Non-finite physics state; stop before invalid observations reach PPO.")
            recovery=getattr(self.cfg.env,'recovery_task',None)
            if recovery:
                self.recovery_signals=recovery_advance(self.recovery_state,self.episode_length_buf*self.dt,
                    self._bottom_clearance(),self.projected_gravity,self.base_lin_vel,self.base_ang_vel,
                    self.contact_forces[:,self.feet_indices,2],contact,
                    (self.base_position[:,:2]-self.position_reference).norm(dim=1),self.fifty_mass*9.81,
                    recovery,self.dt,torch,q=self.dof_pos)
                if recovery.get('compression_before_hip'):
                    self.recovery_geometry_state=self.recovery_geometry.evaluate(self.dof_pos,self.projected_gravity)
                fail=self.recovery_signals['failed']
            self.fail_buf = torch.where(fail, self.fail_buf+1, torch.zeros_like(self.fail_buf))
            self.time_out_buf = self.episode_length_buf >= self.max_episode_length
            self.edge_reset_buf[:] = False
            if self.cfg.terrain.mesh_type in ["heightfield", "trimesh"]:
                self.edge_reset_buf = ((self.base_position[:, 0] > self.terrain_x_max-1)
                                      | (self.base_position[:, 0] < self.terrain_x_min+1)
                                      | (self.base_position[:, 1] > self.terrain_y_max-1)
                                      | (self.base_position[:, 1] < self.terrain_y_min+1))
            self.reset_buf = ((self.fail_buf >= math.ceil(self.cfg.env.fail_to_terminal_time_s/self.dt))
                              | invalid | self.time_out_buf | self.edge_reset_buf
                              | knee_branch_violation(self.dof_state[..., 0],
                                  PARAMS.get('mechanical_limits',{}).get('minimum_knee_included_angle_deg',0.)))
            if recovery:self.reset_buf |= fail
            if getattr(self.cfg.env, 'capture_evaluation', False):
                self.evaluation_state = {
                    'position': self.root_states[:, :3].clone(),
                    'gravity': self.projected_gravity.clone(),
                    'wheel_contacts': self.contact_forces[:, self.feet_indices].norm(dim=-1).clone(),
                    'base_contact': contact.clone(),
                    'failed': (self.reset_buf & ~self.time_out_buf).clone(),
                    'branch_invalid': knee_branch_violation(self.dof_pos,
                        PARAMS.get('mechanical_limits',{}).get('minimum_knee_included_angle_deg',0.)).clone(),
                    'quaternion': self.base_quat.clone(),
                    'velocity': self.base_lin_vel.clone(),
                    'yaw_rate': self.base_ang_vel[:,2].clone(),
                    'reference': self.position_reference.clone(),
                    'reference_yaw': self.heading_reference.clone(),
                }
                if getattr(self.cfg.env,'clearance_control',False):
                    self.evaluation_state.update(clearance=self._bottom_clearance().clone(),height_command=self.commands[:,2].clone())
                if recovery:
                    self.evaluation_state.update(recovery_stable=self.recovery_signals['stable'].clone(),
                        recovery_first_stable_s=self.recovery_state['first_stable_s'].clone(),
                        recovery_released=self.recovery_state['released'].clone(),
                        wheel_vertical=self.contact_forces[:,self.feet_indices,2].clone(),
                        angular_velocity=self.base_ang_vel.clone(),dof_position=self.dof_pos.clone())
                    if recovery.get('compression_before_hip'):
                        self.evaluation_state.update(compressed=self.recovery_state['compressed'].clone(),
                            first_compressed_s=self.recovery_state['first_compressed_s'].clone(),
                            axle_com_error=self.recovery_geometry_state['axle_com_error'].clone())

        def _compute_passive_torques(self):
            force = getattr(self.cfg.env,'push_force_n',0.)
            torque = getattr(self.cfg.env,'push_yaw_torque_nm',0.)
            if self.random_pulses is not None:
                wrench=self.random_pulses.at(self.common_step_counter)
                self.test_forces.zero_();self.test_torques.zero_()
                self.test_forces[:,self.base_body_index]=wrench[:,:3]
                self.test_torques[:,self.base_body_index]=wrench[:,3:]
                if not self.gym.apply_rigid_body_force_tensors(self.sim,gymtorch.unwrap_tensor(self.test_forces),
                        gymtorch.unwrap_tensor(self.test_torques),gymapi.ENV_SPACE):
                    raise RuntimeError('Random force application failed')
                self.random_pulse_applied=wrench.clone();self.random_pulse_apply_calls+=1
            elif force or torque:
                interval=max(1,round(self.cfg.env.push_interval_s/self.dt))
                duration=max(1,round(self.cfg.env.push_duration_s/self.dt))
                active=(self.episode_length_buf>=interval)&(self.episode_length_buf%interval<duration)
                self.test_forces.zero_();self.test_torques.zero_()
                group=torch.arange(self.num_envs,device=self.device)%8
                for axis,positive,negative in ((0,0,1),(1,2,3)):
                    self.test_forces[:,self.base_body_index,axis]=active*force*((group==positive).float()-(group==negative).float())
                self.test_torques[:,self.base_body_index,2]=active*torque*((group==4).float()-(group==5).float())
                self.gym.apply_rigid_body_force_tensors(self.sim,gymtorch.unwrap_tensor(self.test_forces),
                    gymtorch.unwrap_tensor(self.test_torques),gymapi.ENV_SPACE)
            return self.gas_springs.compute(self.dof_pos, self.dof_vel)

        def _to_sim_dof_torques(self, torque):
            return torque.index_select(1, self.sim_from_policy)

        def _init_buffers(self):
            super()._init_buffers()
            # Preserve the native simulator tensor; all policy-side buffers
            # use the documented right-first order, even if PhysX sorts links.
            self.sim_dof_state = self.dof_state
            self.dof_state = self.sim_dof_state.view(self.num_envs, self.num_dof, 2).index_select(
                1, self.policy_from_sim).contiguous()
            self.dof_pos = self.dof_state[..., 0]
            self.dof_vel = self.dof_state[..., 1]
            self.position_reference=self.root_states[:,:2].clone()
            self.heading_reference=yaw_xyzw(self.root_states[:,3:7],torch).clone()
            self.training_commands=None
            if getattr(self.cfg.env,'training_command_slew',False):
                self.training_commands=TrainingCommands(self.commands[:,:3],self.cfg.env.command_slew_rates,
                    [0.,0.,self.cfg.rewards.base_height_target],torch)
                self.commands[:,:3].copy_(self.training_commands.applied)
            self.previous_commands=self.commands[:,:3].clone()
            self.command_unchanged_s=torch.zeros(self.num_envs,device=self.device)
            self.alignment_count=torch.zeros_like(self.command_unchanged_s)
            self.alignment_squared=torch.zeros_like(self.command_unchanged_s)
            self.alignment_peak=torch.zeros_like(self.command_unchanged_s)
            self.recovery_state=recovery_initial_state(self.command_unchanged_s)
            recovery=getattr(self.cfg.env,'recovery_task',None)
            if recovery and recovery.get('compression_before_hip'):
                self.recovery_geometry=RecoveryGeometry(URDF,PARAMS['expected_runtime_dof_order'],torch,self.device,self.dof_pos.dtype)
            if getattr(self.cfg.env,'clearance_control',False):
                self.clearance_box_center=torch.tensor(self.cfg.env.clearance_box_center,device=self.device)
                self.clearance_box_halfsize=torch.tensor(self.cfg.env.clearance_box_halfsize,device=self.device)
            self.test_forces=torch.zeros((self.num_envs,self.num_bodies,3),device=self.device)
            self.test_torques=torch.zeros_like(self.test_forces)
            settings=getattr(self.cfg.env,'random_pulses',None)
            self.random_pulses=RandomPulses(torch,self.num_envs,self.device,self.test_forces.dtype,self.dt,settings) if settings else None
            self.random_pulse_apply_calls=0
            self.random_pulse_applied=torch.zeros((self.num_envs,6),device=self.device)

        def compute_dof_vel(self):
            self.dof_state.copy_(self.sim_dof_state.view(self.num_envs, self.num_dof, 2).index_select(
                1, self.policy_from_sim))
            super().compute_dof_vel()

        def _reset_dofs(self, env_ids):
            recovery=getattr(self.cfg.env,'recovery_task',None)
            if recovery:self.dof_pos[env_ids]=torch.tensor(recovery['reset_joint_angles_rad'],device=self.device)
            else:self.dof_pos[env_ids] = self.default_dof_pos[env_ids]
            self.dof_state[env_ids, :, 1] = 0.0
            self.dof_vel[env_ids] = 0.0
            self.sim_dof_state.view(self.num_envs, self.num_dof, 2)[env_ids] = (
                self.dof_state[env_ids].index_select(1, self.sim_from_policy))
            ids = env_ids.to(dtype=torch.int32)
            self.gym.set_dof_state_tensor_indexed(
                self.sim, gymtorch.unwrap_tensor(self.sim_dof_state),
                gymtorch.unwrap_tensor(ids), len(ids))

        def _process_rigid_body_props(self, props, env_id):
            # Validate CAD mass before deliberate mass/inertia randomization.
            if env_id == 0:
                raw_mass = sum(p.mass for p in props)
                if abs(raw_mass - PARAMS["source_mass_kg"]) > 1e-3:
                    raise RuntimeError(f"URDF mass mismatch before randomization: {raw_mass}")
                self.nominal_body_properties=[body_signature(p) for p in props]
            props=super()._process_rigid_body_props(props, env_id)
            if env_id==0 and self.cfg.domain_rand.randomize_base_com:
                reserve_nominal(props,self.base_com,self.cfg.domain_rand.com_nominal_stride)
            return props

        def _process_rigid_shape_props(self,props,env_id):
            if env_id==0:
                self.nominal_shape_friction=[float(p.friction) for p in props]
                self.parameter_motor_gains,self.parameter_friction_scales=training_parameters.sample(
                    torch,self.num_envs,self.cfg.env.parameter_randomization)
                self.motor_output_gain=(self.parameter_motor_gains.to(self.device)
                    if self.cfg.env.parameter_randomization is not None else None)
            props=super()._process_rigid_shape_props(props,env_id)
            # The importer reuses this list; always derive from original values.
            return training_parameters.apply_shapes(props,self.nominal_shape_friction,
                self.cfg.terrain.static_friction,float(self.parameter_friction_scales[env_id]))

        def _create_envs(self):
            super()._create_envs()
            expected = PARAMS["expected_runtime_dof_order"]
            self.sim_dof_names = list(self.dof_names)
            if len(self.sim_dof_names) != len(expected) or set(self.sim_dof_names) != set(expected):
                raise RuntimeError(f"Unexpected imported joints: {self.sim_dof_names}")
            self.policy_from_sim = torch.tensor([self.sim_dof_names.index(n) for n in expected],
                                                dtype=torch.long, device=self.device)
            self.sim_from_policy = torch.argsort(self.policy_from_sim)
            for name in ("dof_pos_limits", "dof_vel_limits", "torque_limits"):
                setattr(self, name, getattr(self, name).index_select(0, self.policy_from_sim))
            self.dof_names = list(expected)
            self.dof_pos_limits[[0,3],0]=-float('inf')
            self.dof_pos_limits[[0,3],1]=float('inf')
            if self.num_bodies != 7:
                raise RuntimeError(f"Expected 7 rigid bodies, got {self.num_bodies}.")
            if int(self.feet_indices.numel()) != 2:
                raise RuntimeError("Both wheel bodies must be identified by foot_name='wheel'.")
            self.gas_springs = GasSpringBank(spring_cfg, self.dof_names)
            self.actuators = CoupledActuators(torch, hardware_cfg, self.dof_names)
            props = self.gym.get_actor_rigid_body_properties(self.envs[0], self.actor_handles[0])
            mass = sum(p.mass for p in props)
            self.fifty_mass = mass
            self.fifty_body_names = list(self.gym.get_actor_rigid_body_names(self.envs[0], self.actor_handles[0]))
            self.base_body_index=self.fifty_body_names.index('base_link')
            if self.base_body_index!=0:raise RuntimeError('COM distribution expects base_link at body zero')
            self.com_randomization_report=audit_com(self,self.nominal_body_properties)
            # Adjacent CAD pieces share shaft regions; filter only these pairs,
            # leaving base-shin, base-wheel, opposite legs, etc. collidable.
            joint_pairs = [(j.find("parent").get("link"), j.find("child").get("link"))
                           for j in ET.parse(URDF).getroot().findall("joint")]
            self.collision_masks = adjacent_collision_masks(self.fifty_body_names, joint_pairs)
            for e, actor in zip(self.envs, self.actor_handles):
                props = self.gym.get_actor_rigid_shape_properties(e, actor)
                shape_ranges = self.gym.get_actor_rigid_body_shape_indices(e, actor)
                for name, shape_range in zip(self.fifty_body_names, shape_ranges):
                    for i in range(shape_range.start, shape_range.start+shape_range.count):
                        props[i].filter = self.collision_masks[name]
                        props[i].contact_offset = self.cfg.sim.physx.contact_offset
                        props[i].rest_offset = 0.0
                self.gym.set_actor_rigid_shape_properties(e, actor, props)
            self.parameter_randomization_report=training_parameters.audit(self)
            if len(set(self.nominal_shape_friction))!=1:
                raise RuntimeError('Exporting mixed nominal shape materials requires per-shape correspondence')
            self.cfg.env.native_contact_materials=dict(version=1,combine='average',
                robot_sliding_friction=self.nominal_shape_friction[0],
                ground_sliding_friction=self.cfg.terrain.dynamic_friction,
                source='Imported nominal PhysX shape properties and configured plane; solver sliding test verified average')

    cfg, train_cfg = FiftyCfg(), FiftyCfgPPO()
    from training_profile import apply_profile
    profile_path = PACKAGE/'config/training_profile.json'
    profile = apply_profile(profile_path, cfg, train_cfg)
    if profile:
        stance = profile['height_m']
    local.height_protocol=None
    if local.random_pulse_audit_output:
        cfg.env.capture_evaluation=True
        cfg.commands.ranges.lin_vel_x=[0.,0.];cfg.commands.ranges.ang_vel_yaw=[0.,0.]
        cfg.commands.ranges.height=[cfg.rewards.base_height_target]*2
        cfg.commands.resampling_time=30.;cfg.env.episode_length_s=30.
        cfg.env.command_endpoint_probability=0.
    if height_requested:
        from height_protocol import HeightProtocol
        local.height_protocol=HeightProtocol.from_config(class_to_dict(cfg),local.evaluation_clearance,local.evaluation_height_cycle)
        local.height_protocol.validate_duration(local.evaluate_seconds)
    if local.evaluate_output:
        # Evaluation supplies already-ramped applied commands from its protocol.
        cfg.env.training_command_slew=False
        # Frozen nominal acceptance must not silently inherit the training COM
        # mixture. Explicit uncertainty tests are separate archived scenarios.
        cfg.domain_rand.randomize_base_com=False
        cfg.env.parameter_randomization=None
        cfg.env.random_pulses=None
        cfg.env.command_endpoint_probability=0.
        cfg.env.motion_hold_resamples=1
        cfg.env.mixed_motion_holds=False
        cfg.env.capture_evaluation = True
        cfg.env.initial_velocity_noise = local.evaluation_noise
        cfg.env.episode_length_s = local.evaluate_seconds + 1.
        if local.evaluate_tracking:
            # Evaluation follows its explicit schedule, not random training
            # command resampling partway through a reversal or height ramp.
            cfg.commands.resampling_time=local.evaluate_seconds+1.
            if getattr(cfg.env,'clearance_control',False):
                initial_clearance=local.height_protocol.command(0.) if local.height_protocol else cfg.rewards.base_height_target
                cfg.commands.ranges.height=[initial_clearance]*2
            if (local.evaluation_yaw_command or local.evaluation_turn_transitions) and not cfg.env.se2_reference:
                raise ValueError('This profile has no commanded heading reference; turning is not supported')
            cfg.commands.ranges.lin_vel_x=[local.evaluation_command]*2
            cfg.commands.ranges.ang_vel_yaw=[local.evaluation_yaw_command]*2
            cfg.env.standing_fraction=0.
            cfg.env.straight_fraction=0.
            cfg.env.turn_fraction=0.
            cfg.env.push_force_n=local.evaluation_push_force
            cfg.env.push_yaw_torque_nm=local.evaluation_yaw_torque
            cfg.env.push_interval_s=10.
    if local.ppo_mini_batches is not None:
        if local.ppo_mini_batches<1:raise ValueError('ppo-mini-batches must be positive')
        train_cfg.algorithm.num_mini_batches=local.ppo_mini_batches
    args = get_args()
    args.task = TASK

    if local.smoke_test or local.joint_test or local.audit_test:
        cfg.env.parameter_randomization=None
        cfg.env.random_pulses=None
        args.num_envs = 1
        cfg.noise.add_noise = False
        for name in vars(type(cfg.domain_rand)):
            if name.startswith("randomize_"):
                setattr(cfg.domain_rand, name, False)
        for name in ["randomize_friction", "randomize_restitution", "randomize_base_mass",
                     "randomize_inertia", "randomize_base_com", "randomize_Kp",
                     "randomize_Kd", "randomize_motor_torque", "randomize_default_dof_pos",
                     "randomize_action_delay", "push_robots"]:
            setattr(cfg.domain_rand, name, False)
        cfg.commands.ranges.lin_vel_x = [0.0, 0.0]
        cfg.commands.ranges.ang_vel_yaw = [0.0, 0.0]
        cfg.commands.ranges.height = [cfg.rewards.base_height_target] * 2
        if local.joint_test or local.audit_test:
            cfg.asset.fix_base_link = True
            cfg.init_state.pos[2] = 1.2

    task_registry.register(TASK, FiftyRobot, cfg, train_cfg)
    env, env_cfg = task_registry.make_env(name=TASK, args=args)
    print("DOF order        :", list(env.dof_names))
    print("Simulator order  :", env.sim_dof_names)
    print("Body names       :", env.fifty_body_names)
    print("Loaded mass [kg] :", round(env.fifty_mass, 9))
    print("Torque limits    :", env.torque_limits.tolist())
    print("Kp / Kd          :", env.p_gains[0].tolist(), env.d_gains[0].tolist())
    print("Feet indices     :", env.feet_indices.tolist())
    print("Gas springs      :", json.dumps(spring_report, ensure_ascii=False))
    print("Native asset import:", json.dumps(env.imported_asset_options))
    print("Self-collision masks:", env.collision_masks)
    print('Imported COM distribution:',json.dumps(env.com_randomization_report),flush=True)
    print('Imported motor/friction distribution:',json.dumps({k:env.parameter_randomization_report[k]
        for k in ('enabled','settings','max_import_error')}),flush=True)
    if local.evaluate_output:
        local.evaluate_output.mkdir(parents=True,exist_ok=True)
        (local.evaluate_output/'com_randomization.json').write_text(json.dumps(env.com_randomization_report,indent=2)+'\n')
        training_parameters.save(env,local.evaluate_output)
    if local.com_audit_output:
        local.com_audit_output.mkdir(parents=True,exist_ok=False)
        (local.com_audit_output/'imported-properties.json').write_text(json.dumps(env.com_randomization_report,indent=2)+'\n')
        (local.com_audit_output/'env_cfg.json').write_text(json.dumps(class_to_dict(env_cfg),indent=2,default=str)+'\n')
        training_parameters.save(env,local.com_audit_output)
        # Exercise the actual adapter's PD -> coupled motor -> delivered torque
        # path without stepping the physical model or updating the policy.
        env.dof_pos.copy_(env.default_dof_pos);env.dof_vel.zero_()
        actions=torch.arange(1,7,device=env.device).float().repeat(env.num_envs,1)*.001
        env._compute_torques(actions)
        delivered=env.actuators.motor_torque_applied.clone()
        env.actuators.apply(env.motor_torque_unclipped,env.dof_pos,env.dof_vel)
        expected=env.actuators.motor_torque_applied*(env.motor_output_gain if env.motor_output_gain is not None else 1.)
        if not torch.allclose(delivered,expected,atol=1e-6,rtol=1e-5):raise RuntimeError('Motor gain not applied in adapter')
        (local.com_audit_output/'motor-path-audit.json').write_text(json.dumps(dict(passed=True,
            actors=env.num_envs,max_error=float((delivered-expected).abs().max()),
            probe='Small unsaturated actions; all six actual motor gains verified'))+'\n')
        env.gym.destroy_sim(env.sim)
        return 0
    if local.audit_test:
        print("Imported DOF properties:", json.dumps(env.imported_dof_properties))

    if local.audit_test:
        from runtime_audit import run
        run(env, PACKAGE / "validation/training_audit")
        env.gym.destroy_sim(env.sim)
        return 0

    if local.smoke_test:
        env.reset()
        zeros = torch.zeros((env.num_envs, 6), device=env.device)
        initial_z = float(env.root_states[0, 2])
        resets = 0
        peak_passive_torque = 0.0
        for _ in range(100):
            obs, _, rew, dones, _, _ = env.step(zeros)
            if not torch.isfinite(env.root_states).all() or not torch.isfinite(env.dof_state).all():
                raise RuntimeError("Non-finite physics state during the smoke test.")
            if not torch.isfinite(obs).all() or not torch.isfinite(rew).all():
                raise RuntimeError("Non-finite observation/reward during the smoke test.")
            resets += int(dones.sum().item())
            if not torch.isfinite(env.passive_torques).all():
                raise RuntimeError("Non-finite passive spring torque.")
            peak_passive_torque = max(peak_passive_torque, float(env.passive_torques.abs().max()))
        contact = env.contact_forces[0, env.feet_indices, :]
        print(json.dumps({
            "smoke_steps": 100,
            "resets": resets,
            "finite_states": True,
            "initial_base_height_m": initial_z,
            "final_base_height_m": float(env.root_states[0, 2]),
            "wheel_contact_force_norm": [round(float(x), 2) for x in contact.norm(dim=-1)],
            "expected_weight_n": round(env.fifty_mass * 9.81, 2),
            "stable_standing_verified": False,
            "gas_spring_enabled": spring_cfg["enabled"],
            "peak_passive_torque_nm": peak_passive_torque,
        }, indent=2))
        env.gym.destroy_sim(env.sim)
        return 0

    if local.joint_test:
        zeros = torch.zeros((env.num_envs, 6), device=env.device)
        rows = []
        for idx, name in enumerate(env.dof_names):
            env.reset_idx(torch.arange(env.num_envs, device=env.device))
            for _ in range(20):                    # settle back at the default pose
                env.step(zeros)
            q0 = env.dof_pos[0].clone()
            qd0 = env.dof_vel[0].clone()
            action = zeros.clone()
            action[0, idx] = 0.2 if "wheel" not in name else 1.0
            for _ in range(50):                    # 0.5 s
                env.step(action)
            dq = float(env.dof_pos[0, idx] - q0[idx])
            dqd = float(env.dof_vel[0, idx] - qd0[idx])
            rows.append({"index": idx, "joint": name,
                         "delta_q": round(dq, 4), "delta_qd": round(dqd, 4),
                         "sign_ok": (dq > 0) if "wheel" not in name else (dqd > 0)})
        print(json.dumps({"joint_test": rows,
                          "note": "wheel rows compare delta_qd, leg rows compare delta_q"},
                         indent=2))
        for idx, name in enumerate(env.dof_names):
            env.reset_idx(torch.arange(env.num_envs, device=env.device))
            for _ in range(20):
                env.step(zeros)
            action = zeros.clone()
            action[0, idx] = -0.2 if "wheel" not in name else -1.0
            for _ in range(50):
                env.step(action)
            print(f"negative command {name:14s} -> dof_pos={env.dof_pos[0, idx].item():+.4f} "
                  f"dof_vel={env.dof_vel[0, idx].item():+.4f}")
        env.gym.destroy_sim(env.sim)
        return 0

    runner, train_cfg = task_registry.make_alg_runner(env=env, name=TASK, args=args,
                            log_root=None if (local.benchmark_output or local.evaluate_output or local.random_pulse_audit_output or local.command_ramp_audit_output) else 'default')
    if local.resume_checkpoint:
        runner.load(str(local.resume_checkpoint.resolve()))
        print('Restored completed updates:',runner.current_learning_iteration,flush=True)
    if local.command_ramp_audit_output:
        if not local.resume_checkpoint:raise ValueError('Frozen checkpoint required for command audit')
        from training_commands import audit
        audit(env,runner,local.command_ramp_audit_output)
        env.gym.destroy_sim(env.sim)
        return 0
    if local.random_pulse_audit_output:
        if not local.resume_checkpoint:raise ValueError('Frozen checkpoint required for pulse audit')
        from audit_random_pulses import run
        run(env,runner,local.random_pulse_audit_output)
        env.gym.destroy_sim(env.sim)
        return 0
    if local.benchmark_output:
        from performance_benchmark import run
        run(env,runner,local.benchmark_output)
        env.gym.destroy_sim(env.sim)
        return 0
    if local.evaluate_output:
        if local.evaluate_tracking:
            from evaluate_tracking import evaluate
        else:
            from evaluate_standing import evaluate
        evaluate(env, runner, local)
        env.gym.destroy_sim(env.sim)
        return 0
    log_dir = Path(task_registry.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "gas_spring_resolved.json").write_text(
        json.dumps(spring_cfg, indent=2, ensure_ascii=False), encoding="utf-8")
    shutil.copy2(PACKAGE / "integration/gas_spring.py", log_dir / "gas_spring.py")
    shutil.copy2(PACKAGE / "integration/training_checks.py", log_dir / "training_checks.py")
    shutil.copy2(PACKAGE / "integration/actuator_model.py", log_dir / "actuator_model.py")
    shutil.copy2(PACKAGE / "config/actuator_hardware.json", log_dir / "actuator_hardware.json")
    if (PACKAGE/'config/mechanical_limits.json').exists():
        shutil.copy2(PACKAGE/'config/mechanical_limits.json',log_dir/'mechanical_limits.json')
    for name in ('recovery_task.py','recovery_geometry.py','wheel_alignment.py','training_checks.py','training_com.py','command_sampling.py','training_parameters.py','action_limits.py','pose_feedback_reward.py','random_pulses.py','training_commands.py'):
        shutil.copy2(PACKAGE/'integration'/name,log_dir/name)
    (log_dir/'com_randomization.json').write_text(json.dumps(env.com_randomization_report,indent=2)+'\n')
    training_parameters.save(env,log_dir)
    (log_dir/'requested_com_offsets_m.json').write_text(json.dumps(env.base_com.detach().cpu().tolist())+'\n')
    if env.random_pulses is not None:
        (log_dir/'random_pulses.json').write_text(json.dumps(env.random_pulses.report(env.base_com.abs().amax(dim=1)<1e-7),indent=2)+'\n')
    if profile:
        shutil.copy2(profile_path, log_dir/'training_profile.json')
        shutil.copy2(PACKAGE/'integration/training_profile.py', log_dir/'training_profile.py')
        shutil.copy2(PACKAGE/'integration/tracking_state.py', log_dir/'tracking_state.py')
        shutil.copy2(PACKAGE/'integration/height_protocol.py', log_dir/'height_protocol.py')
    for name, obj in [("env_cfg.json", env_cfg), ("train_cfg.json", train_cfg)]:
        (log_dir / name).write_text(json.dumps(class_to_dict(obj), indent=2, default=str),
                                    encoding="utf-8")
    for path in [Path(__file__), URDF, PACKAGE / "config/model_parameters.json",
                 PACKAGE / "config/build_report.json",
                 PACKAGE / "docs/CONVERSION_NOTES.md"]:
        shutil.copy2(path, log_dir / path.name)
    if getattr(cfg.env,'recovery_task',None):
        # Runner construction calls reset(), which performs a dummy zero-action
        # step. Restart the physical episode so the deadline starts at PPO's
        # first action, not that initialization step.
        env.reset_idx(torch.arange(env.num_envs,device=env.device))
        env.compute_observations()
    runner.learn(num_learning_iterations=train_cfg.runner.max_iterations,
                 init_at_random_ep_len=not bool(getattr(cfg.env,'recovery_task',None)))
    if env.random_pulses is not None:
        (log_dir/'random_pulses_final.json').write_text(json.dumps(env.random_pulses.report(env.base_com.abs().amax(dim=1)<1e-7),indent=2)+'\n')
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
