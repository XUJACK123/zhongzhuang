"""Command line entry point.

Usage::

    python -m fifty_mujoco.cli <subcommand>

Implemented in this phase (P0-P2): ``audit-assets``, ``build-assets``,
``validate-mjcf``, ``inspect``, ``spawn-audit``, ``doctor``.
Everything else is a gated stub that fails loudly instead of pretending to work.
"""

from __future__ import annotations

import argparse
import json
import platform
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np

from . import __version__
from .config import Config, load_configs
from .paths import PROJECT_ROOT, assets_dir, reports_dir


# --------------------------------------------------------------------------- utils
def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _write_report(payload: dict[str, Any], name: str) -> Path:
    path = reports_dir() / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _print_checks(title: str, checks: dict[str, Any]) -> None:
    print(f"\n[{title}]")
    for name, check in checks.items():
        if not isinstance(check, dict) or "passed" not in check:
            continue
        print(f"  {'PASS' if check['passed'] else 'FAIL'}  {name}")


def _model_path(config: Config, variant: str) -> Path:
    return assets_dir() / "mjcf" / f"fifty_{variant}.xml"


def _env(config: Config, variant: str):
    from .envs.cpu_env import FiftyCpuEnv

    return FiftyCpuEnv(_model_path(config, variant), config)


# --------------------------------------------------------------------------- commands
def cmd_audit_assets(args: argparse.Namespace, config: Config) -> int:
    from .assets.audit import audit_assets

    report = audit_assets(config)
    path = _write_report(report, "assets_audit.json")
    _print_checks("audit-assets", report["checks"])
    print(f"\n  source urdf sha256 : {report['sources']['source_urdf']['sha256']}")
    print(f"  defects reproduced : {len(report['defects'])}")
    for defect in report["defects"]:
        if "offset_norm_m" in defect:
            print(
                f"    - {defect['link']:<12} visual offset {defect['offset_norm_m']:.6f} m "
                f"(training urdf vs recomputed)"
            )
        else:
            print(f"    - {defect.get('link')}: {defect.get('issue')}")
    print("  findings (non-gating)")
    print(f"    - visual offsets correct : {report['findings']['visual_offsets']['correct']}")
    print(
        f"    - inertial rpy identity  : {report['findings']['inertial_origin_rotation']['identity']}"
    )
    print(f"  missing inputs     : {[k for k, v in report['missing_inputs'].items() if v]}")
    print(f"  report             : {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_build_assets(args: argparse.Namespace, config: Config) -> int:
    from .assets.build_corrected import build_corrected_assets
    from .assets.build_mjcf import write_mjcf_variants
    from .assets.validate_mjcf import validate, write_validation_report

    corrected = build_corrected_assets(config)
    paths = write_mjcf_variants(config, corrected)
    validation = validate(config, corrected, paths)
    report_path = write_validation_report(validation, reports_dir() / "model_validation.json")

    _print_checks("build-assets / corrected assets", corrected.report["checks"])
    _print_checks("build-assets / model validation", validation["checks"])
    corrected_ok = corrected.report["status"] == "passed"
    passed = corrected_ok and validation["status"] == "passed"
    print(f"\n  corrected urdf : {corrected.report['corrected_urdf']}")
    print(f"  visual xml     : {paths.visual}")
    print(f"  train xml      : {paths.train}")
    print(f"  spawn z        : {paths.spawn_z:.9f} m")
    print(f"  total mass     : {validation['checks']['total_mass']['compiled_kg']:.9f} kg")
    print(f"  report         : {report_path}")

    manifest = {
        "run_id": f"build-assets-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}",
        "date": _now(),
        "package_version": __version__,
        "source_urdf_hash": corrected.report["source_urdf_sha256"],
        "corrected_asset_hash": corrected.report["corrected_urdf_sha256"],
        "model_xml_hash": {
            "visual": _sha256(paths.visual),
            "train": _sha256(paths.train),
        },
        "spawn_spec_hash": _sha256(PROJECT_ROOT / "configs" / "spawn.yaml"),
        "policy_contract_hash": _sha256(PROJECT_ROOT / "configs" / "policy_contract.yaml"),
        "physics_dt_s": float(config.robot["simulation"]["timestep_s"]),
        "policy_dt_s": float(config.robot["control"]["policy_dt_s"]),
        "spring_backend": config.gas_springs_raw["backend"],
        "spring_count": config.spring_count,
        "spring_parameter_source": "missing (configs/gas_spring.yaml has null anchors)",
        "compensation_mode": str(config.gas_springs_raw["compensation_mode"]),
        "actor_obs_dim": 25,
        "history_dim": 125,
        "action_dim": 6,
        "privileged_schema": 141 if config.spring_count == 0 else 149,
        "critic_input_dim": 144 if config.spring_count == 0 else 152,
        "validation_level": "static checks + CPU geometry/spawn only",
        "passed_gates": ["P1"] if passed else ["none"],
        "unresolved_items": [
            "gas-spring anchors, stroke and force unknown (P3/P4 unpassed)",
            "wheel radius provisional",
            "rotor armature unknown",
            "motor torque limit provisional",
            "no observation/reward/PPO/teacher code yet (P4-P10)",
        ],
    }
    _write_report(manifest, "run_manifest.json")
    return 0 if passed else 1


def cmd_validate_mjcf(args: argparse.Namespace, config: Config) -> int:
    from .assets.build_corrected import build_corrected_assets
    from .assets.build_mjcf import MjcfPaths, write_mjcf_variants
    from .assets.validate_mjcf import validate, write_validation_report

    corrected = build_corrected_assets(config)
    paths = write_mjcf_variants(config, corrected)
    report = validate(config, corrected, paths)
    path = write_validation_report(report, reports_dir() / "model_validation.json")
    _print_checks("validate-mjcf", report["checks"])
    print(f"\n  report: {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_inspect(args: argparse.Namespace, config: Config) -> int:
    env = _env(config, args.variant)
    summary = env.model_summary()
    print(json.dumps(summary, indent=2, ensure_ascii=False))

    from .control.spawn import load_spawn_spec, resolve_spawn_height

    spec = load_spawn_spec(config)
    root_z, bottoms = resolve_spawn_height(env, spec)
    print(f"\nresolved spawn z = {root_z:.9f} m, wheel bottoms = {bottoms}")

    if args.viewer:
        try:
            import time

            import mujoco.viewer

            env.reset(root_position=(spec.root_xy_m[0], spec.root_xy_m[1], root_z))
            if args.animate:
                print("\nopening viewer (simulating); close the window to exit")
                mujoco.viewer.launch(env.model, env.data)
            else:
                # Static display by default: with no controller the robot would
                # fall over within a second and look "inverted".  Press the
                # viewer's Play button to simulate.
                print("\nopening viewer (static spawn pose); close the window to exit")
                with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
                    while viewer.is_running():
                        viewer.sync()
                        time.sleep(0.02)
        except Exception as error:  # pragma: no cover - depends on the desktop session
            print(
                f"\nviewer unavailable ({type(error).__name__}: {error}).\n"
                "This is expected on a headless session; run `inspect` without --viewer "
                "or launch it from a desktop terminal."
            )
            return 3
    return 0


def cmd_spawn_audit(args: argparse.Namespace, config: Config) -> int:
    from .control.spawn import audit_spawn, load_spawn_spec

    env = _env(config, args.variant)
    spec = load_spawn_spec(config)
    report = audit_spawn(env, config, spec)
    path = _write_report(report, "spawn_report.json")
    _print_checks("spawn-audit", report["checks"])
    print(f"\n  spawn z            : {report['spawn_z_m']:.9f} m")
    print(f"  wheel axles (world): {report['frames']['wheel_axles_world_m']}")
    print(f"  top marker (world) : {report['frames']['top_marker_world_m']}")
    print(f"  lowest robot point : {report['frames']['lowest_robot_point_m']}")
    print(f"  report             : {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_spring_audit(args: argparse.Namespace, config: Config) -> int:
    from .control.spring_audit import audit_springs

    env = _env(config, "train")
    report = audit_springs(config, env)
    path = _write_report(report, "spring_report.json")
    _print_checks("spring-audit", report["checks"])
    if report["status"] == "not_configured":
        print("\n  gas springs are not configured; nothing to audit")
        return 2
    print("\n  profile            :", report["profile"], "(calibrated:", report["calibrated"], ")")
    for name, spring in report["springs"].items():
        print(
            f"  {name:<10} length {spring['length_at_spawn_m']:.5f} m  "
            f"force {spring['force_at_spawn_n']:.1f} N  tendon {spring['tendon_id']}  "
            f"actuator {spring['actuator_id']}  status {spring['parameter_status']}"
        )
    support = report["support_test"]
    print(
        f"  支持测试（无控制器）: 起始 {support['start_height_m']:.4f} m  "
        f"最终 {support['final_height_m']:.4f} m  最低 {support['min_height_m']:.4f} m  "
        f"up·Z {support['final_up_dot']:.3f}"
    )
    print(f"  report             : {path}")
    print("  note: P3 仍未关闭 —— 锚点来自复旦模型，行程未知")
    return 0 if report["status"] == "passed" else 1


def cmd_render(args: argparse.Namespace, config: Config) -> int:
    from .control.spawn import load_spawn_spec, resolve_spawn_height
    from .evaluation.render import render_views

    env = _env(config, args.variant)
    spec = load_spawn_spec(config)
    root_z, bottoms = resolve_spawn_height(env, spec)
    env.data.qvel[:] = 0.0
    env.forward()

    out_dir = PROJECT_ROOT / "reports" / "screenshots"
    images = render_views(env, out_dir, prefix=args.prefix, show_collision=args.show_collision)

    report = {
        "stage": "visual",
        "date": _now(),
        "variant": args.variant,
        "xml_path": str(env.xml_path.resolve()),
        "xml_sha256": env.model_summary()["xml_sha256"],
        "spawn_z_m": root_z,
        "wheel_bottoms_m": bottoms,
        "root_quat_wxyz": np.asarray(env.data.qpos[3:7], dtype=float).tolist(),
        "springs": [spring.name for spring in env.springs],
        "images": images,
    }
    path = _write_report(report, "render_report.json")
    for image in images:
        print(f"  {image['view']:<12} {image['path']}")
    print(f"\n  report: {path}")
    return 0


def cmd_doctor(args: argparse.Namespace, config: Config) -> int:
    report: dict[str, Any] = {
        "stage": "environment",
        "date": _now(),
        "python": sys.version.split()[0],
        "platform": platform.platform(),
        "packages": {},
        "gpu": {},
        "checks": {},
    }

    def version(name: str) -> str | None:
        """Distribution version without importing the (sometimes heavy) module."""
        from importlib.metadata import PackageNotFoundError, version as distribution_version

        distributions = {"warp": "warp-lang", "mjlab": "mjlab", "onnxruntime": "onnxruntime"}
        try:
            return distribution_version(distributions.get(name, name))
        except PackageNotFoundError:
            return None

    for name in ("numpy", "mujoco", "torch", "warp", "mjlab", "onnxruntime"):
        report["packages"][name] = version(name)

    report["checks"]["mujoco_cpu"] = {"passed": report["packages"]["mujoco"] is not None}
    if report["packages"]["mujoco"] is not None:
        import mujoco

        xml = _model_path(config, "train")
        report["checks"]["mujoco_can_load_model"] = {
            "passed": xml.is_file(),
            "path": str(xml),
        }

    device = args.device
    torch_version = report["packages"]["torch"]
    if torch_version is None:
        report["checks"]["torch_gpu"] = {"passed": False, "note": "torch not installed (gpu extra)"}
    else:  # pragma: no cover - depends on the optional GPU stack
        import torch

        available = bool(torch.cuda.is_available())
        report["gpu"]["torch_cuda_available"] = available
        report["gpu"]["device_count"] = int(torch.cuda.device_count()) if available else 0
        if available:
            index = int(device.split(":")[-1]) if ":" in device else 0
            report["gpu"]["name"] = torch.cuda.get_device_name(index)
            report["gpu"]["memory_total_mib"] = round(
                torch.cuda.get_device_properties(index).total_memory / 2**20, 1
            )
            tensor = torch.rand(1024, device=device)
            report["gpu"]["tensor_sum"] = float((tensor * 2).sum().item())
            report["checks"]["torch_gpu"] = {"passed": bool(report["gpu"]["tensor_sum"] > 0)}
        else:
            report["checks"]["torch_gpu"] = {"passed": False, "note": "cuda not available"}

    warp_version = report["packages"]["warp"]
    if warp_version is None:
        report["checks"]["warp"] = {"passed": False, "note": "warp-lang not installed (gpu extra)"}
    else:  # pragma: no cover
        try:
            import warp as wp

            wp.init()
            report["checks"]["warp"] = {
                "passed": True,
                "version": warp_version,
                "devices": [str(device) for device in wp.get_devices()],
            }
        except Exception as error:
            report["checks"]["warp"] = {"passed": False, "note": str(error)}

    if report["packages"]["mjlab"] is None:
        report["checks"]["mjlab"] = {"passed": False, "note": "mjlab not installed (gpu extra)"}
    else:  # pragma: no cover
        try:
            import mjlab  # noqa: F401

            report["checks"]["mjlab"] = {"passed": True, "version": report["packages"]["mjlab"]}
        except Exception as error:
            report["checks"]["mjlab"] = {"passed": False, "note": str(error)}

    path = _write_report(report, "doctor.json")
    _print_checks("doctor", report["checks"])
    print(f"\n  report: {path}")
    return 0 if all(check.get("passed") for check in report["checks"].values()) else 1


GATED_COMMANDS: dict[str, str] = {}

TEACHER_ONNX = (
    "ref_document/fudan_rl_wheel_leg-main/mujoco/actor/yuntai/上台阶3_angz+.onnx"
)


def cmd_joint_test(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.channel_suite import run_joint_test

    env = _env(config, "train")
    report = run_joint_test(config, env)
    path = _write_report(report, "joint_test.json")
    for name, entry in report["channels"].items():
        print(
            f"  {'PASS' if entry['sign_ok'] else 'FAIL'}  {name:<18} "
            f"cmd {entry['command']:+.3f} -> {entry['measured']:+.5f} {entry['unit']} "
            f"(tau_peak {entry['peak_torque_nm']:.2f} N·m)"
        )
    print(f"\n  peak torque: {report['peak_torque_nm']:.2f} N·m\n  report: {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_spring_sweep(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.spring_suite import run_spring_sweep

    env = _env(config, "train")
    report = run_spring_sweep(config, env, duration_s=args.duration)
    path = _write_report(report, "spring_sweep.json")
    for name, entry in report["combinations"].items():
        print(
            f"  {name:<28} h {entry['start_height_m']:.3f} -> {entry['final_height_m']:.3f} m  "
            f"up·Z {entry['up_dot']:+.3f}  tau_rms {entry['torque_rms_nm']:.2f}  "
            f"sat {entry['saturation_fraction']:.2%}  gas {entry['max_spring_force_n']:.0f} N"
        )
    _print_checks("spring-sweep", report["checks"])
    print(f"\n  report: {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_contact_test(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.contact_suite import run_contact_test

    env = _env(config, "train")
    report = run_contact_test(config, env)
    path = _write_report(report, "contact_test.json")
    _print_checks("contact-test", report["checks"])
    load = report["checks"]["dual_wheel_load"]
    print(
        f"\n  wheels {np.round(load['wheel_normal_force_n'], 1)} N  "
        f"total {load['total_vertical_n']:.1f} N vs weight {load['weight_n']:.1f} N"
    )
    print(f"  report: {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_contract_test(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.contract_suite import run_contract_test

    report = run_contract_test(config, num_envs=args.num_envs)
    path = _write_report(report, "contract_test.json")
    _print_checks("contract-test", report["checks"])
    print(f"\n  report: {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_prepare_teacher(args: argparse.Namespace, config: Config) -> int:
    from .learning.actor_critic import NetworkDims
    from .transfer.teacher import restore_teacher, write_teacher_manifest

    path = Path(args.onnx) if args.onnx else config.workspace_path(TEACHER_ONNX)
    dims = NetworkDims(num_privileged=141 + 4 * config.spring_count)
    teacher = restore_teacher(path, dims)
    manifest = write_teacher_manifest(teacher, PROJECT_ROOT / "configs" / "teacher_manifest.yaml")
    report = {"stage": "P6", "status": "passed" if teacher.report["passed"] else "failed", **teacher.report}
    report_path = _write_report(report, "teacher_report.json")
    print(f"  inputs        : {teacher.report['inputs']}")
    print(f"  matched tensors: {teacher.report['matched_tensors']}")
    print(f"  max abs error  : {teacher.report['max_abs_error']:.3e}")
    print(f"  max rel error  : {teacher.report['max_rel_error']:.3e}")
    print(f"  manifest       : {manifest}")
    print(f"  report         : {report_path}")
    print("  note: 通道顺序仍未证明；evaluate-teacher 才是物理语义的检查")
    return 0 if report["status"] == "passed" else 1


def cmd_evaluate_teacher(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.teacher_eval import evaluate_teacher

    path = Path(args.onnx) if args.onnx else config.workspace_path(TEACHER_ONNX)
    report = evaluate_teacher(config, path, episodes_per_command=args.episodes, seconds=args.seconds)
    report_path = _write_report(report, "teacher_eval.json")
    for name, entry in report["commands"].items():
        print(
            f"  {name:<10} vx {entry['command'][0]:+.2f} wz {entry['command'][1]:+.2f} -> "
            f"survival {entry['mean_survival_s']:.2f} s  "
            f"vx_err {entry['tracking_vx_rmse']:.3f}  up·Z {entry['mean_up_dot']:+.3f}"
        )
    print(f"\n  status: {report['status']}\n  report: {report_path}")
    print("  note: 目标模型的站立高度与教师训练时不同，迁移表现差不等于权重恢复失败")
    return 0 if report["status"] == "passed" else 1


def cmd_smoke(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.gpu_smoke import run_smoke

    report = run_smoke(config, num_envs=args.num_envs)
    path = _write_report(report, "gpu_smoke.json")
    _print_checks("smoke", report.get("checks", {}))
    print(f"\n  status: {report['status']}")
    if report.get("note"):
        print(f"  note  : {report['note']}")
    print(f"  report: {path}")
    return 0 if report["status"] == "passed" else 1


def cmd_benchmark(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.benchmark import run_benchmark

    report = run_benchmark(config, scales=args.scales)
    path = _write_report(report, "benchmark.json")
    for entry in report["results"]:
        print(
            f"  envs {entry['num_envs']:>3} : {entry['steps_per_s']:.0f} policy steps/s, "
            f"{entry['env_steps_per_s']:.0f} env-steps/s, {entry['wall_s']:.1f} s"
        )
    print(f"\n  report: {path}")
    return 0


def cmd_train(args: argparse.Namespace, config: Config) -> int:
    from .learning.runner import OnPolicyRunner, RunnerConfig

    run_dir = PROJECT_ROOT / "runs" / (args.name or datetime.now(timezone.utc).strftime("run_%Y%m%d_%H%M%S"))
    runner_cfg = RunnerConfig(
        num_envs=args.num_envs,
        num_steps_per_env=args.steps_per_env,
        max_updates=args.smoke_updates or args.updates,
        max_wall_time_s=None if args.smoke_updates else args.max_wall_time_s,
        save_interval=args.save_interval,
        critic_warmup_updates=0 if args.smoke_updates else args.critic_warmup,
        stage_updates_per_level=args.stage_updates,
        seed=args.seed,
        log_dir=str(run_dir),
    )
    runner = OnPolicyRunner(config, runner_config=runner_cfg)
    teacher_section = (config.policy_contract or {}).get("teacher", {}) or {}
    distillation_enabled = bool(teacher_section.get("distillation_enabled", False))
    if args.teacher and not args.no_teacher:
        if not distillation_enabled:
            print(
                "  teacher : requested but DISABLED by policy_contract.teacher."
                "distillation_enabled=false"
            )
            print(
                "            (the Fudan ONNX restores exactly but does not"
                " transfer to this chassis; see docs/MIGRATION_STATUS.md)"
            )
        else:
            runner.attach_teacher(
                config.workspace_path(args.teacher)
                if args.teacher != "default"
                else config.workspace_path(TEACHER_ONNX)
            )
    result = runner.train()
    print(f"\n  updates      : {result['updates']} ({result['stop_reason']})")
    print(f"  final stage  : {result['final_stage']}")
    print(f"  elapsed      : {result['elapsed_s']:.1f} s")
    print(f"  run dir      : {run_dir}")
    return 0


def cmd_train_gpu(args: argparse.Namespace, config: Config) -> int:
    """PPO on the Warp + CUDA-graph environment, with live progress."""
    from .learning.gpu_ppo import GpuPpoConfig, GpuPpoTrainer, plot_progress

    run_dir = PROJECT_ROOT / "runs" / (args.name or datetime.now(timezone.utc).strftime("gpu_%Y%m%d_%H%M%S"))
    run_dir.mkdir(parents=True, exist_ok=True)

    cfg = GpuPpoConfig(
        num_envs=args.num_envs,
        num_steps=args.steps,
        updates=args.updates,
        epochs=args.epochs,
        minibatches=args.minibatches,
        lr=args.lr,
        seed=args.seed,
        device=args.device,
        log_every=args.log_every,
        leg_kp=args.leg_kp,
        leg_kd=args.leg_kd,
        eval_every=args.eval_every,
        eval_steps=args.eval_steps,
        save_every=args.save_every,
        entropy_coef=args.entropy_coef,
        max_std=args.max_std,
        adaptive_lr=not args.no_adaptive_lr,
        stop_on_survive=args.stop_on_survive,
        progress_path=run_dir / "progress.log",
        save_path=run_dir / "model_last.pt",
    )
    (run_dir / "progress.log").write_text("", encoding="utf-8")
    trainer = GpuPpoTrainer(config, cfg)
    # Start the clock *after* environment construction + CUDA-graph capture,
    # otherwise ~15 s of setup is silently taken out of the requested budget.
    deadline = time.perf_counter() + args.minutes * 60.0
    print(f"  envs {args.num_envs} x {args.steps} steps, device {args.device}, "
          f"leg Kp {trainer.env.gains.leg_kp:g} / Kd {trainer.env.gains.leg_kd:g}, "
          f"cuda graph {'ON' if trainer.graph_ok else 'OFF (' + str(trainer.env.capture_error) + ')'}")
    print(f"  budget {args.minutes:.1f} min, run dir {run_dir}", flush=True)

    result = trainer.train(verbose=True, deadline=deadline)
    records = trainer.history
    trainer.save(run_dir / "model_last.pt")
    final_eval = trainer.evaluate(steps=max(args.eval_steps, 600)) if trainer.eval_env else {}
    if final_eval:
        print(
            f"  final eval (deterministic, {final_eval['eval_steps']} steps): "
            f"survival {final_eval['eval_survival_s']:.2f}s "
            f"alive {final_eval['eval_alive_frac']*100:.0f}% "
            f"height {final_eval['eval_height_final']:.3f} m "
            f"up {final_eval['eval_up_min']:+.3f} "
            f"vx±{final_eval['eval_vx_mae']:.3f}",
            flush=True,
        )
    report = {
        "stage": "P9-gpu",
        "status": "completed",
        "num_envs": args.num_envs,
        "num_steps": args.steps,
        "updates": result["updates"],
        "env_steps": result["env_steps"],
        "elapsed_s": result["elapsed_s"],
        "sps": result["env_steps"] / max(result["elapsed_s"], 1e-9),
        "cuda_graph": trainer.graph_ok,
        "capture_error": getattr(trainer.env, "capture_error", None),
        "leg_kp": trainer.env.gains.leg_kp,
        "leg_kd": trainer.env.gains.leg_kd,
        "wheel_torque_limit_nm": trainer.env.gains.wheel_torque_limit_nm,
        "leg_torque_limit_nm": trainer.env.gains.torque_limit_nm,
        "final_eval": final_eval,
        "history": records,
    }
    path = _write_report(report, "gpu_train.json")
    if not args.no_plot:
        plot_path = Path(args.plot) if args.plot else run_dir / "progress.png"
        plot_progress(records, plot_path, title=f"GPU PPO  {args.num_envs} 环境 × {args.steps} 步")
        print(f"  plot : {plot_path}")
    print(f"\n  updates   : {result['updates']}")
    print(f"  env steps : {result['env_steps']:,}")
    print(f"  elapsed   : {result['elapsed_s']:.1f} s ({report['sps']:,.0f} env-steps/s)")
    print(f"  checkpoint: {run_dir / 'model_last.pt'}")
    print(f"  report    : {path}")
    return 0


def cmd_export(args: argparse.Namespace, config: Config) -> int:
    import torch

    from .learning.runner import OnPolicyRunner, RunnerConfig
    from .transfer.export import export_onnx, verify_onnx, write_export_manifest

    checkpoint = Path(args.checkpoint)
    # Two checkpoint flavours exist: the CPU ``OnPolicyRunner`` writes ``ppo``
    # (actor + critic + optimiser), the GPU trainer writes ``policy`` (actor +
    # encoder only).  Export only ever needs the actor, so accept both.
    payload = torch.load(checkpoint, map_location="cpu", weights_only=False)
    if "ppo" in payload:
        runner_cfg = RunnerConfig(num_envs=1, log_dir=None)
        runner = OnPolicyRunner(config, runner_config=runner_cfg, checkpoint=checkpoint)
        policy = runner.policy
        extra_meta = {
            "update_count": runner.update_count,
            "stage": runner.stage,
            "pd_gains": runner.env.gains.__dict__,
            "spawn_z_m": runner.env.spawn_z,
        }
        source = "cpu-runner"
    elif "policy" in payload:
        from .learning.actor_critic import ActorCriticSequence, NetworkDims

        dims = payload.get("dims") or {}
        policy = ActorCriticSequence(
            NetworkDims(**{k: v for k, v in dims.items()})
        )
        policy.load_state_dict(payload["policy"])
        policy.eval()
        extra_meta = {
            "update_count": payload.get("updates"),
            "stage": payload.get("stage", "gpu-ppo"),
            "env_steps": payload.get("env_steps"),
            "trainer_cfg": payload.get("cfg"),
            "pd_gains": payload.get("pd_gains"),
            "spawn_z_m": payload.get("spawn_z_m"),
        }
        source = "gpu-trainer"
    else:
        print(f"  unrecognised checkpoint keys: {sorted(payload)}")
        return 1

    out = PROJECT_ROOT / "runs" / "export" / f"{checkpoint.stem}.onnx"
    export_onnx(policy, out)
    verification = verify_onnx(policy, out)
    manifest = write_export_manifest(
        policy,
        out,
        extra={
            "checkpoint": str(checkpoint),
            "checkpoint_source": source,
            "verification": verification,
            "default_joint_angles_rad": config.default_angles,
            "spring_profile": config.gas_spring_profile,
            "spring_count": config.spring_count,
            **extra_meta,
        },
    )
    print(f"  onnx        : {out}")
    print(f"  source      : {source}")
    print(f"  max abs err : {verification['max_abs_error']:.3e}")
    print(f"  manifest    : {manifest}")
    return 0 if verification["passed"] else 1


def cmd_play(args: argparse.Namespace, config: Config) -> int:
    from .evaluation.playback import play_onnx

    return play_onnx(config, args.onnx, viewer=args.viewer, seconds=args.seconds,
                     command=(args.vx, args.wz), variant=args.variant)



def cmd_gated(args: argparse.Namespace, config: Config) -> int:
    gate = GATED_COMMANDS.get(args.command, "a later phase")
    print(
        f"'{args.command}' is not implemented yet: it belongs to gate {gate}.\n"
        "This build only covers P0-P2 (assets, MJCF, CPU spawn audit).\n"
        "See docs/PHASES.md for the gate list and the data each gate needs.",
        file=sys.stderr,
    )
    return 2


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


# --------------------------------------------------------------------------- parser
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="fifty-mujoco",
        description="fifty wheel-legged robot MuJoCo migration (phase P0-P2)",
    )
    parser.add_argument("--version", action="version", version=f"fifty_mujoco {__version__}")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("audit-assets", help="P0: audit source assets and known defects")
    subparsers.add_parser("build-assets", help="P1: corrected URDF + two MJCFs + validation")
    subparsers.add_parser("validate-mjcf", help="P1: re-run the release checks")

    inspect = subparsers.add_parser("inspect", help="P2: dump model summary (optional viewer)")
    inspect.add_argument("--backend", default="cpu", choices=["cpu"])
    inspect.add_argument("--variant", default="visual", choices=["visual", "train"])
    inspect.add_argument("--viewer", action="store_true")
    inspect.add_argument(
        "--animate",
        action="store_true",
        help="step physics in the viewer (the robot falls: there is no controller yet)",
    )

    spawn = subparsers.add_parser("spawn-audit", help="P2: upright spawn audit")
    spawn.add_argument("--backend", default="cpu", choices=["cpu"])
    spawn.add_argument("--profile", default="geometry_only", choices=["geometry_only"])
    spawn.add_argument("--variant", default="train", choices=["visual", "train"])

    spring = subparsers.add_parser("spring-audit", help="P3: passive gas-spring interface audit")
    spring.add_argument("--backend", default="cpu", choices=["cpu"])
    spring.add_argument("--profile", default="reference_300n", choices=["geometry_only", "reference_300n"])

    render = subparsers.add_parser("render", help="fixed-camera snapshots of the model")
    render.add_argument("--backend", default="cpu", choices=["cpu"])
    render.add_argument("--variant", default="visual", choices=["visual", "train"])
    render.add_argument("--prefix", default="model")
    render.add_argument("--show-collision", action="store_true")

    doctor = subparsers.add_parser("doctor", help="environment / GPU / Torch / Warp check")
    doctor.add_argument("--device", default="cuda:0")

    joint = subparsers.add_parser("joint-test", help="P4: per-channel sign/unit test")
    joint.add_argument("--backend", default="cpu", choices=["cpu"])

    sweep = subparsers.add_parser("spring-sweep", help="P4: gas-spring / motor combinations")
    sweep.add_argument("--backend", default="cpu", choices=["cpu"])
    sweep.add_argument("--duration", type=float, default=1.0)

    contact = subparsers.add_parser("contact-test", help="P5: drop / roll / load / fallen")
    contact.add_argument("--backend", default="cpu", choices=["cpu"])

    contract = subparsers.add_parser("contract-test", help="P5: observation / action / history contract")
    contract.add_argument("--backend", default="cpu", choices=["cpu"])
    contract.add_argument("--num-envs", type=int, default=2)

    prepare = subparsers.add_parser("prepare-teacher", help="P6: restore the teacher ONNX")
    prepare.add_argument("--onnx", default=None)

    evaluate_teacher = subparsers.add_parser("evaluate-teacher", help="P6: teacher on the target model")
    evaluate_teacher.add_argument("--onnx", default=None)
    evaluate_teacher.add_argument("--episodes", type=int, default=2)
    evaluate_teacher.add_argument("--seconds", type=float, default=10.0)

    smoke = subparsers.add_parser("smoke", help="P7: batch backend smoke (mjlab when available)")
    smoke.add_argument("--backend", default="mjlab", choices=["mjlab", "cpu"])
    smoke.add_argument("--num-envs", type=int, default=16)

    benchmark = subparsers.add_parser("benchmark", help="P8: parallel scale measurement")
    benchmark.add_argument("--scales", type=int, nargs="+", default=[1, 16, 64])

    train = subparsers.add_parser("train", help="P8/P9: PPO training")
    train.add_argument("--num-envs", type=int, default=64)
    train.add_argument("--steps-per-env", type=int, default=48)
    train.add_argument("--updates", type=int, default=1000)
    train.add_argument("--smoke-updates", type=int, default=None)
    train.add_argument("--max-wall-time-s", type=float, default=10800.0)
    train.add_argument("--save-interval", type=int, default=50)
    train.add_argument("--critic-warmup", type=int, default=20)
    train.add_argument("--stage-updates", type=int, default=30)
    train.add_argument("--seed", type=int, default=0)
    train.add_argument("--name", default=None)
    train.add_argument("--teacher", default="default")
    train.add_argument("--no-teacher", action="store_true")

    gpu = subparsers.add_parser(
        "train-gpu",
        help="P9: PPO on MuJoCo Warp + CUDA graphs (batched control/obs/reward)",
    )
    gpu.add_argument("--num-envs", type=int, default=2048)
    gpu.add_argument(
        "--steps",
        type=int,
        default=24,
        help="rollout length per update; larger = bigger PPO batches = better GPU use",
    )
    gpu.add_argument("--updates", type=int, default=100000)
    gpu.add_argument("--minutes", type=float, default=10.0)
    gpu.add_argument("--epochs", type=int, default=5)
    gpu.add_argument("--minibatches", type=int, default=4)
    gpu.add_argument("--lr", type=float, default=1e-3)
    gpu.add_argument("--seed", type=int, default=0)
    gpu.add_argument("--device", default="cuda:0")
    gpu.add_argument("--log-every", type=int, default=10)
    gpu.add_argument(
        "--leg-kp",
        type=float,
        default=None,
        help="override the leg Kp (default: the config contract, 20 N.m/rad)",
    )
    gpu.add_argument("--leg-kd", type=float, default=None, help="override the leg Kd")
    gpu.add_argument(
        "--eval-every",
        type=int,
        default=50,
        help="deterministic evaluation every N updates (0 disables)",
    )
    gpu.add_argument("--eval-steps", type=int, default=300, help="eval horizon in policy steps")
    gpu.add_argument(
        "--save-every",
        type=int,
        default=100,
        help="checkpoint + history.json every N updates (0 = only at the end)",
    )
    gpu.add_argument("--entropy-coef", type=float, default=0.005, help="entropy bonus")
    gpu.add_argument("--max-std", type=float, default=1.0, help="clamp on the action std")
    gpu.add_argument(
        "--no-adaptive-lr",
        action="store_true",
        help="keep --lr fixed (recommended for large batches: KL is tiny and the "
        "adaptive rule otherwise drives lr to its floor)",
    )
    gpu.add_argument(
        "--stop-on-survive",
        type=int,
        default=0,
        help="stop after N consecutive evals that survive the whole window (0 = off)",
    )
    gpu.add_argument("--name", default=None)
    gpu.add_argument("--plot", default=None)
    gpu.add_argument("--no-plot", action="store_true")

    export = subparsers.add_parser("export", help="P10: export the actor+encoder to ONNX")
    export.add_argument("--checkpoint", required=True)

    play = subparsers.add_parser("play", help="P10: ONNX playback on the CPU model")
    play.add_argument("--onnx", required=True)
    play.add_argument("--viewer", action="store_true")
    play.add_argument("--seconds", type=float, default=10.0)
    play.add_argument("--vx", type=float, default=0.0)
    play.add_argument("--wz", type=float, default=0.0)
    play.add_argument("--variant", default="visual", choices=["visual", "train"])

    for name in GATED_COMMANDS:
        subparsers.add_parser(name, help=f"gated stub ({GATED_COMMANDS[name]})")

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    config = load_configs()

    handlers = {
        "audit-assets": cmd_audit_assets,
        "build-assets": cmd_build_assets,
        "validate-mjcf": cmd_validate_mjcf,
        "inspect": cmd_inspect,
        "spawn-audit": cmd_spawn_audit,
        "spring-audit": cmd_spring_audit,
        "joint-test": cmd_joint_test,
        "spring-sweep": cmd_spring_sweep,
        "contact-test": cmd_contact_test,
        "contract-test": cmd_contract_test,
        "prepare-teacher": cmd_prepare_teacher,
        "evaluate-teacher": cmd_evaluate_teacher,
        "smoke": cmd_smoke,
        "benchmark": cmd_benchmark,
        "train": cmd_train,
        "train-gpu": cmd_train_gpu,        "export": cmd_export,
        "play": cmd_play,
        "render": cmd_render,
        "doctor": cmd_doctor,
    }
    handler = handlers.get(args.command, cmd_gated)
    return handler(args, config)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
