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
from .paths import DEFAULT_WORKSPACE_ROOT, PROJECT_ROOT, assets_dir, reports_dir


# --------------------------------------------------------------------------- utils
def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _write_report(payload: dict[str, Any], name: str) -> Path:
    path = reports_dir() / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _write_json(payload: dict[str, Any], path: str | Path) -> Path:
    """Write an explicitly requested report path (``--json``).

    ``--json`` used to only *print* the path and never write the file, so an
    evaluation could look finished while leaving no evidence behind.
    """
    target = Path(path)
    if not target.is_absolute():
        target = Path.cwd() / target
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return target


def _report_path(payload: dict[str, Any], explicit: str | None, default_name: str) -> Path:
    return _write_json(payload, explicit) if explicit else _write_report(payload, default_name)


def _print_checks(title: str, checks: dict[str, Any]) -> None:
    print(f"\n[{title}]")
    for name, check in checks.items():
        if not isinstance(check, dict) or "passed" not in check:
            continue
        print(f"  {'PASS' if check['passed'] else 'FAIL'}  {name}")


def _model_path(config: Config, variant: str) -> Path:
    """Model file for an entry point.

    ``train`` is the model selected in ``configs/models.yaml`` (or
    ``FIFTY_MODEL``); ``visual`` names the serial-only mesh variant and is
    therefore rejected for any other selected model instead of silently loading
    the serial bodies under the closed model's name map.
    """
    if variant == "train":
        return config.model_xml()
    if config.model_name != "serial":
        raise SystemExit(
            f"variant {variant!r} only exists for the serial model; "
            f"model {config.model_name!r} has no visual MJCF (use --variant train)"
        )
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
    path_check = report["checks"]["wheel_load_path"]
    print(
        f"\n  wheels {np.round(path_check['wheel_normal_force_n'], 1)} N  "
        f"contacts {path_check['contact_count']}  "
        f"trunk {path_check['trunk_height_m']:.4f} m  "
        f"other {path_check['other_contacts']}"
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


def _parse_init_parts(text: str | None) -> tuple[str, ...] | None:
    """``--init-parts`` -> sub-module tuple (``None`` = the whole network)."""
    if text is None:
        return None
    cleaned = text.strip().lower()
    if cleaned in ("", "all"):
        return None
    known = {"actor", "encoder", "critic", "std"}
    parts = tuple(part.strip() for part in cleaned.split(",") if part.strip())
    unknown = [part for part in parts if part not in known]
    if unknown or not parts:
        raise SystemExit(
            f"--init-parts: unknown {', '.join(unknown) or text!r}; "
            f"choose from {', '.join(sorted(known))} or 'all'"
        )
    return parts


def cmd_train_gpu(args: argparse.Namespace, config: Config) -> int:
    """PPO on the Warp + CUDA-graph environment, with live progress."""
    from .learning.gpu_ppo import GpuPpoConfig, GpuPpoTrainer, plot_progress

    if args.resume and args.init_from:
        print("  --resume and --init-from are different experiments; pick one")
        return 2
    if args.resume and _parse_init_parts(args.init_parts) is not None:
        print("  --init-parts only applies to --init-from")
        return 2

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
        stand_command=args.stand_command,
        checkpoint_keep=args.checkpoint_keep,
        progress_path=run_dir / "progress.log",
        save_path=run_dir / "model_last.pt",
    )
    (run_dir / "progress.log").write_text("", encoding="utf-8")
    trainer = GpuPpoTrainer(config, cfg)
    if args.resume:
        loaded = trainer.load(Path(args.resume), resume=True)
        print(f"  resume from {args.resume}: update {loaded['update']}, "
              f"fingerprint match {loaded['fingerprint_match']}")
        if loaded["fingerprint_match"] is False:
            print("  WARNING: the model or config hash changed since that checkpoint; "
                  "this is a different experiment, not a continuation")
    elif args.init_from:
        parts = _parse_init_parts(args.init_parts)
        loaded = trainer.load(Path(args.init_from), resume=False, parts=parts)
        description = "all weights" if parts is None else " + ".join(parts)
        print(f"  initialised from {args.init_from} ({description}, fresh optimiser/std/critic "
              f"unless listed)")
        print(f"  next update index: {trainer.update_count + 1}")
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
            f"p10 {final_eval['eval_t_fall_p10']:.2f}s "
            f"alive {final_eval['eval_alive_frac']*100:.0f}% "
            f"h±{final_eval['eval_height_mae']:.3f} m "
            f"tilt {final_eval['eval_tilt_max_deg']:.1f} deg "
            f"vx±{final_eval['eval_vx_mae']:.3f}",
            flush=True,
        )
        print(
            "  note: this short deterministic window is a training signal only; "
            "acceptance is `evaluate-stand` (64 seeds x 60 s, CPU model)",
            flush=True,
        )
    report = {
        "stage": "P9-gpu",
        "status": "completed",
        "num_envs": args.num_envs,
        "num_steps": args.steps,
        "updates": result["updates"],
        "total_updates": result["total_updates"],
        "stop_reason": result["stop_reason"],
        "stand_command": args.stand_command,
        "init_from": trainer.init_from,
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
    print(f"\n  updates   : {result['updates']} this run / {result['total_updates']} total"
          f"  (stop: {result['stop_reason']})")
    print(f"  env steps : {result['env_steps']:,}")
    print(f"  elapsed   : {result['elapsed_s']:.1f} s ({report['sps']:,.0f} env-steps/s)")
    print(f"  checkpoint: {run_dir / 'model_last.pt'}")
    print(f"  report    : {path}")
    return 0


def cmd_evaluate_stand(args: argparse.Namespace, config: Config) -> int:
    """Independent acceptance evaluation (doc 14.2).

    ``--backend cpu`` is the reference (independent physics, per-env reasons);
    ``--backend warp`` runs the same window on the training backend ~200x faster.
    """
    from .evaluation.stand_eval import (
        OnnxPolicy,
        TorchPolicy,
        make_scenarios,
        run_stand_eval,
        zero_policy,
    )

    if args.zero_policy:
        policy = zero_policy
        source = "zero-policy"
    elif args.onnx:
        policy = OnnxPolicy(args.onnx)
        source = f"onnx:{args.onnx}"
    elif args.checkpoint:
        policy = TorchPolicy.from_checkpoint(args.checkpoint, device=args.device)
        source = f"checkpoint:{args.checkpoint}"
    else:
        print("  need --checkpoint, --onnx or --zero-policy")
        return 2

    scenarios = make_scenarios(base_seed=args.seed_base, count=args.episodes)
    report = run_stand_eval(
        config,
        policy,
        steps=args.steps,
        scenarios=scenarios,
        backend=args.backend,
        device=args.device,
    )
    report["policy_source"] = source
    report["generated_at"] = _now()
    report["seeds"] = [scenario.seed for scenario in scenarios]
    # Never overwrite another run's evidence (progress.md §16.3): the default
    # name carries the policy, the window and the timestamp.
    if args.onnx or args.checkpoint:
        stem = Path(args.onnx or args.checkpoint).stem
    else:
        stem = "zero_policy"
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    default_name = f"stand_eval_{stem}_{args.steps}steps_{stamp}.json"
    path = _report_path(report, args.json, default_name)

    criteria = report["criteria"]["checks"]
    print(f"  policy   : {source}")
    print(f"  backend  : {report.get('backend', 'cpu')}")
    print(f"  window   : {report['steps']} steps = {report['window_s']:.1f} s per episode, "
          f"episode limit {report['episode_limit_s']:.1f} s")
    print(f"  episodes : {report['episodes']} (seeds {args.seed_base}..{args.seed_base + args.episodes - 1})")
    print(f"  success  : {report['successes']}/{report['episodes']} = {report['success_rate']*100:.1f}%")
    print(f"  survival : min {report['survival_min_s']:.2f}s  p10 {report['survival_p10_s']:.2f}s  "
          f"median {report['survival_median_s']:.2f}s")
    print(f"  reasons  : {report['reasons']}")
    for name, check in criteria.items():
        flag = "PASS" if check["passed"] else "FAIL"
        print(f"  {flag} {name:20s} {check['value']:.4f}  (limit {check['threshold']})")
    print(f"  status   : {report['status']}")
    print(f"  report   : {path}")
    return 0 if report["criteria"]["passed"] else 1


def cmd_evaluate_lqr(args: argparse.Namespace, config: Config) -> int:
    """Section 11.6 gate: the teacher is judged like a policy, and cannot pass silently."""
    import yaml

    from .control.lqr_teacher import LqrTeacher
    from .evaluation.lqr_suite import evaluate_teacher

    gains: dict[str, float] = {}
    if args.config_file:
        payload = yaml.safe_load(Path(args.config_file).read_text(encoding="utf-8")) or {}
        gains = {
            name: float(entry["value"])
            for name, entry in (payload.get("gains") or {}).items()
            if isinstance(entry, dict) and "value" in entry
        }
    for item in args.set or []:
        name, _, value = item.partition("=")
        gains[name.strip()] = float(value)

    teacher = LqrTeacher(config, gains)
    report = evaluate_teacher(
        config,
        teacher,
        steps=args.steps,
        episodes=args.episodes,
        seed_base=args.seed_base,
    )
    report["generated_at"] = _now()
    report["gain_overrides"] = gains
    # Canonical gate file on purpose: this path always holds the *latest*
    # teacher verdict; every historical verdict is recorded in progress.md.
    path = _report_path(report, args.json, "lqr_teacher_eval.json")
    print(f"  window   : {report['steps']} steps = {report['window_s']:.1f} s")
    print(f"  episodes : {report['episodes']}")
    print(f"  success  : {report['successes']}/{report['episodes']} = {report['success_rate']*100:.1f}%")
    print(f"  survival : min {report['survival_min_s']:.2f}s  median {report['survival_median_s']:.2f}s")
    print(f"  reasons  : {report['reasons']}")
    params = report["teacher"]["parameters"] or {}
    print(f"  params   : mass {params.get('total_mass_kg')} kg, com above axle "
          f"{params.get('com_above_axle_m')} m, wheel r {params.get('wheel_radius_m')} m")
    print(f"  gain K   : {report['teacher']['wheel_gain']}")
    print(f"  status   : {report['status']}  usable_for_demonstrations="
          f"{report['teacher']['usable_for_demonstrations']}")
    print(f"  report   : {path}")
    return 0 if report["criteria"]["passed"] else 1


def _step_candidates() -> list[Path]:
    """The two documented locations: inside the workspace and the external CAD drop."""
    names = ("26英雄整车总装2.0.STEP", "26英雄整车总装.STEP")
    roots = [
        DEFAULT_WORKSPACE_ROOT / "solidwork_graph",
        DEFAULT_WORKSPACE_ROOT.parents[1] / "solidwork_graph",
    ]
    return [root / name for root in roots for name in names]


def cmd_step_audit(args: argparse.Namespace, config: Config) -> int:
    """S2: STEP assembly structure and cylinder-axis candidates (stdlib only)."""
    from .assets.step_audit import audit_step, summarize

    path = Path(args.step) if args.step else next(
        (candidate for candidate in _step_candidates() if candidate.is_file()), None
    )
    if path is None or not path.is_file():
        print("  STEP not found; pass --step explicitly. Candidates:")
        for candidate in _step_candidates():
            print(f"    {candidate}")
        return 2
    report = audit_step(path, with_geometry=not args.no_geometry)
    report["generated_at"] = _now()
    summary = summarize(report, needles=args.match or ("髋", "hip", "spring", "弹簧", "DM8009"))
    report["summary"] = summary
    out = _report_path(report, args.json, "step_geometry_audit.json")
    counts = report["counts"]
    print(f"  file      : {path.name} ({report['source']['bytes']:,} bytes)")
    print(f"  products  : {counts.get('PRODUCT')}")
    print(f"  instances : {counts.get('NEXT_ASSEMBLY_USAGE_OCCURRENCE')} "
          f"(transforms parsed: {sum(1 for i in report['instances'] if 'transform_mm' in i)})")
    print(f"  roots     : {report['assembly']['roots']}")
    geometry = report.get("geometry") or {}
    if geometry:
        diagnostics = geometry["diagnostics"]
        print(f"  cylinders : {diagnostics['cylinders']} "
              f"(attributed to a part: {diagnostics['attributed_cylinders']})")
        print("  axis pairs:")
        for entry in geometry["measurements"]:
            distance = entry.get("distance_mm")
            value = f"{distance:10.3f} mm" if distance is not None else "  unmeasured"
            parts = ", ".join(entry.get("candidate_parts") or []) or "-"
            print(f"      {value}  {entry['label']}  [{entry['status']}]  {parts}")
        for entry in geometry.get("cross_part", []):
            print(f"      {'unmeasured':>13}  {entry['label']}  [{entry['status']}]")
        drive = report.get("drive_train") or {}
        if drive:
            print(f"  drive     : {drive['motor_count']} motor(s) inside a "
                  f"{drive['module_assembly']} module "
                  f"({len(drive['excluded_motors'])} other name match(es) rejected)")
            for motor in drive["motors"]:
                print(f"      {motor['count']:2d} x {motor['product']}")
            spring = drive["gas_spring"]
            pins = spring.get("pin_to_pin") or {}
            if pins:
                print(f"  spring    : pin-to-pin {pins['center_distance_mm']:.3f} mm "
                      f"(along {pins['along_spring_axis_mm']:.3f}, "
                      f"off axis {pins['perpendicular_mm']:.3f}) [{spring['status']}]")
            else:
                print(f"  spring    : {spring.get('status')} (no pin pair measured)")
            gearbox = drive.get("gearbox") or {}
            ratio = gearbox.get("ratio") or {}
            arrangements = ratio.get("arrangements") or []
            if arrangements:
                best = arrangements[0]
                print(f"  gearbox   : Z_sun {ratio['sun_teeth']} (derived), planets "
                      f"{gearbox['planet_teeth']}, ring {gearbox['ring_teeth']} -> "
                      f"{best['ring_fixed_carrier_output']:.2f}:1 ring fixed / "
                      f"{best['carrier_fixed_ring_output']:.2f}:1 carrier fixed "
                      f"[{ratio['status']}]")
    for needle, matches in summary["matches"].items():
        print(f"  match {needle!r}: {len(matches)} instance(s)")
        for entry in matches[:3]:
            print(f"      {entry['parent']} -> {entry['child']}")
    print(f"  status    : {report['status']}")
    print(f"  report    : {out}")
    return 0


def cmd_gpu_parity(args: argparse.Namespace, config: Config) -> int:
    from datetime import datetime, timezone

    from .evaluation.parity_suite import run_parity_suite, summarize as summarize_parity

    report = run_parity_suite(
        config,
        num_envs=args.num_envs,
        steps=args.steps,
        seed=args.seed,
        device=args.device,
        check_graph=not args.no_graph,
    )
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out = _report_path(report, args.json, f"gpu_parity_{stamp}.json")
    print("CPU/GPU contract parity (field by field):")
    for line in summarize_parity(report).splitlines():
        print(f"  {line}")
    print(f"  report : {out}")
    return 0 if report["status"] == "passed" else 1


def cmd_stage_plan(args: argparse.Namespace, config: Config) -> int:
    from datetime import datetime, timezone

    from .learning.stages import describe, load_training_stages, planned_commands

    stages = load_training_stages()
    seeds = list(args.seeds) if getattr(args, "seeds", None) else None
    selected = [
        stage
        for stage in stages.stages
        if args.stage is None or stage.id == args.stage
    ]
    commands = planned_commands(stages, seeds)
    report = {
        "tool": "stage-plan",
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "meta": dict(stages.meta),
        "baseline": {
            key: {
                "value": entry.value,
                "default": entry.default,
                "implemented_by": entry.implemented_by,
                "note": entry.note,
            }
            for key, entry in stages.baseline.items()
        },
        "stages": [
            {
                "id": stage.id,
                "name": stage.name,
                "task": dict(stage.task),
                "keep_standing_fraction": stage.keep_standing_fraction,
                "prerequisite": stage.prerequisite,
                "gate": stage.gate,
                "blocked_by": stage.blocked_by,
            }
            for stage in selected
        ],
        "randomization": {
            key: {
                "implemented": entry.implemented,
                "where": entry.where,
                "cli_exposed": entry.cli_exposed,
                "blocked_by": entry.blocked_by,
            }
            for key, entry in stages.randomization.items()
        },
        "gates": {key: dict(value) for key, value in stages.gates.items()},
        "train_gpu_commands": commands,
        "evidence": "not_run: this file describes a plan, not a training result",
    }
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out = _report_path(report, args.json, f"training_stages_plan_{stamp}.json")
    print(describe(stages))
    print("train-gpu commands (baseline, one per seed):")
    for command in commands:
        print(f"  {command}")
    print(f"report         : {out}")
    return 0


def cmd_joint_conventions(args: argparse.Namespace, config: Config) -> int:
    """Print and verify the joint sign/zero/axis table and the reference pose."""
    from datetime import datetime, timezone

    import yaml

    from .mechanism import load_mechanism
    from .mechanism.conventions import load_conventions, mjcf_joint_axes, reference_pose
    from .paths import configs_dir

    conventions = load_conventions()
    axes = mjcf_joint_axes()
    with (configs_dir() / "robot.yaml").open("r", encoding="utf-8") as handle:
        robot = yaml.safe_load(handle)
    with (configs_dir() / "policy_contract.yaml").open("r", encoding="utf-8") as handle:
        contract = yaml.safe_load(handle)
    verification = conventions.verify(axes, robot, contract)

    mechanism = load_mechanism()
    pose = reference_pose(conventions, mechanism)
    solved = conventions.reference_channel_values()
    drift = {
        "thigh_rad": pose.thigh_angle - solved["thigh_rad"],
        "shin_rad": pose.shin_angle - solved["shin_rad"],
        "crank_rad": pose.crank_angle - solved["crank_rad"],
    }
    target = conventions.reference.get("target") or {}
    chord_target = float(target.get("chord_m", 0.0))

    report = {
        "tool": "joint-conventions",
        "generated_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "version": (conventions.meta or {}).get("version"),
        "frame": (conventions.meta or {}).get("frame"),
        "verification": verification,
        "channels": [
            {
                "name": entry.name,
                "role": entry.role,
                "side": entry.side,
                "axis_base": list(entry.axis_base),
                "sign": entry.sign,
                "zero_rad": entry.zero_rad,
                "limit_rad": list(entry.limit_rad) if entry.limit_rad else None,
                "drives": entry.drives,
            }
            for entry in conventions.channels
        ],
        "reference_pose": {
            "stored": solved,
            "recomputed": pose.as_dict(),
            "drift_rad": drift,
            "chord_target_m": chord_target,
            "chord_recomputed_m": pose.chord_m,
            "singular": abs(pose.jacobian_det) < 1e-6,
        },
        "status": (
            "passed"
            if verification["status"] == "passed"
            and max(abs(value) for value in drift.values()) <= 1e-9
            and abs(pose.chord_m - chord_target) <= 1e-9
            else "failed"
        ),
        "note": (
            "signs and zeros are the simulation/asset convention; the real encoder"
            " zeroes and CAN directions remain a deployment calibration"
        ),
    }
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out = _report_path(report, args.json, f"joint_conventions_{stamp}.json")

    print(f"version : {report['version']}   frame : {report['frame']}")
    print("channel        side   axis            sign   zero(rad)   drives")
    for entry in conventions.channels:
        axis = ", ".join(f"{value:+.0f}" for value in entry.axis_base)
        print(
            f"  {entry.name:<13} {entry.side:<6} [{axis}]  {entry.sign:+.0f}"
            f"   {entry.zero_rad:+.6f}   {entry.drives}"
        )
    print(f"axes verified against the MJCF : {verification['status']}")
    for problem in verification["problems"]:
        print(f"  PROBLEM: {problem}")
    print(
        f"reference pose : thigh {pose.thigh_angle:+.6f}  shin {pose.shin_angle:+.6f}"
        f"  crank {pose.crank_angle:+.6f} rad"
    )
    print(
        f"  chord {pose.chord_m:.9f} m (target {chord_target:.9f})"
        f"  det J {pose.jacobian_det:+.6f}"
        f"  {'SINGULAR' if report['reference_pose']['singular'] else 'non-singular'}"
    )
    worst = max(abs(value) for value in drift.values())
    print(f"  stored-vs-recomputed drift : {worst:.3e} rad")
    print(f"status : {report['status']}")
    print(f"report : {out}")
    return 0 if report["status"] == "passed" else 1


def cmd_mechanism_check(args: argparse.Namespace, config: Config) -> int:
    from datetime import datetime, timezone

    from .mechanism.check import mechanism_report, summarize as summarize_mechanism

    report = mechanism_report(
        allow_inferred=bool(getattr(args, "allow_inferred_topology", False))
    )
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out = _report_path(report, args.json, f"mechanism_check_{stamp}.json")
    print("mechanism geometry (cad_extracted, not measured):")
    for line in summarize_mechanism(report).splitlines():
        print(f"  {line}")
    print(f"  report : {out}")
    return 0 if report["passed"] else 1


def cmd_closed_mjcf(args: argparse.Namespace, config: Config) -> int:
    from datetime import datetime, timezone

    from .assets.build_closed_mjcf import main as build_and_verify

    status = build_and_verify()
    report = _closed_mjcf_report()
    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    out = _report_path(report, args.json, f"closed_chain_{stamp}.json")
    print(f"  report : {out}")
    return status


def _closed_mjcf_report() -> dict:
    from .assets.build_closed_mjcf import verify

    report = verify()
    return {
        "tool": "closed-mjcf",
        "schema": 1,
        "claim": (
            "the closed chain compiles, both loops close at the assembly pose, and the"
            " total mass equals the serial asset; it is a kinematic skeleton, not the"
            " training model"
        ),
        **report,
    }


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
            "update_count": payload.get("update", payload.get("updates")),
            "stage": payload.get("stage", "gpu-ppo"),
            "env_steps": payload.get("env_steps"),
            "trainer_cfg": payload.get("cfg"),
            "pd_gains": payload.get("pd_gains"),
            "spawn_z_m": payload.get("spawn_z_m"),
            "fingerprint": payload.get("fingerprint"),
            "eval_version": payload.get("eval_version"),
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
        default=25,
        help="checkpoint + history.json every N updates (0 = only at the end); "
        "default matches the experiment baseline in progress.md",
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
        help="stop after N consecutive evals that survive the whole window (0 = off); "
        "a window shorter than 60 s never counts as solved",
    )
    gpu.add_argument(
        "--stand-command",
        action="store_true",
        help="hold vx=0, wz=0 and h=h0 for every env and episode (curriculum stage 1); "
        "without it the environment resamples the motion commands",
    )
    gpu.add_argument("--name", default=None)
    gpu.add_argument("--plot", default=None)
    gpu.add_argument("--no-plot", action="store_true")
    gpu.add_argument(
        "--resume",
        default=None,
        help="continue the same experiment (optimiser, lr, update counter, RNG)",
    )
    gpu.add_argument(
        "--init-from",
        default=None,
        help="load only the network weights into a new experiment (fresh optimiser)",
    )
    gpu.add_argument(
        "--init-parts",
        default="all",
        help="sub-modules taken by --init-from: all (default), or a comma-separated "
        "subset of actor,encoder,critic,std. The H0 warm start is actor,encoder - "
        "the critic, the exploration std and the optimiser stay freshly initialised",
    )
    gpu.add_argument(
        "--checkpoint-keep",
        type=int,
        default=5,
        help="numbered model_update_*.pt snapshots to retain",
    )

    stand = subparsers.add_parser(
        "evaluate-stand",
        help="independent 60 s standing evaluation (per-episode first failure)",
    )
    stand.add_argument("--checkpoint", default=None, help="GPU/CPU trainer checkpoint (.pt)")
    stand.add_argument("--onnx", default=None, help="exported ONNX actor")
    stand.add_argument("--steps", type=int, default=6000, help="policy steps per episode (6000 = 60 s)")
    stand.add_argument("--episodes", type=int, default=64, help="number of independent scenarios")
    stand.add_argument("--seed-base", type=int, default=1000, help="first evaluation seed")
    stand.add_argument("--device", default="cpu", help="policy device; with --backend warp this is also the physics device")
    stand.add_argument(
        "--backend",
        default="cpu",
        choices=["cpu", "warp"],
        help=(
            "physics backend: cpu is the independent reference (~25 min for 64x60 s);"
            " warp runs on WarpVecEnv (~6.6 s) and must give the same verdict"
        ),
    )
    stand.add_argument("--json", default=None, help="write the full report to this path")
    stand.add_argument(
        "--zero-policy",
        action="store_true",
        help="control run: hold the spawn pose (no policy at all)",
    )

    lqr = subparsers.add_parser(
        "evaluate-lqr",
        help="LQR teacher standalone gate (section 11.6)",
    )
    lqr.add_argument("--config-file", default=str(PROJECT_ROOT / "configs" / "lqr_teacher.yaml"))
    lqr.add_argument("--set", action="append", help="gain override, e.g. --set q_pitch=60")
    lqr.add_argument("--steps", type=int, default=6000)
    lqr.add_argument("--episodes", type=int, default=64)
    lqr.add_argument("--seed-base", type=int, default=1000)
    lqr.add_argument("--json", default=None)

    step = subparsers.add_parser(
        "step-audit",
        help="S2: STEP assembly structure (products, instances, transforms)",
    )
    step.add_argument(
        "--step",
        default=None,
        help="STEP path (default: search the workspace and the external CAD drop)",
    )
    step.add_argument("--json", default=None)
    step.add_argument("--match", action="append", help="extra name substring to look for")
    step.add_argument(
        "--no-geometry",
        action="store_true",
        help="skip cylinder-axis extraction (structure only, much faster on huge files)",
    )

    mechanism = subparsers.add_parser(
        "mechanism-check",
        help="S2: planar mechanism geometry, FK/IK and loop closure",
    )
    mechanism.add_argument("--json", default=None)
    mechanism.add_argument(
        "--allow-inferred-topology",
        action="store_true",
        help=(
            "evaluate the candidate body assignments (requires reading the CAD mates"
            " before anything downstream uses the result)"
        ),
    )

    closed = subparsers.add_parser(
        "closed-mjcf",
        help="S2c: emit and verify the closed-chain MJCF skeleton (kinematics only)",
    )
    closed.add_argument("--json", default=None)

    conventions_cmd = subparsers.add_parser(
        "joint-conventions",
        help="S3: joint signs/zeros/axes and the reference standing pose",
    )
    conventions_cmd.add_argument("--json", default=None)

    stage_plan = subparsers.add_parser(
        "stage-plan",
        help="S5: print the curriculum/randomisation plan and the baseline train command",
    )
    stage_plan.add_argument("--stage", type=int, default=None, help="only report this stage id")
    stage_plan.add_argument(
        "--seeds",
        type=int,
        nargs="+",
        default=None,
        help="seeds to emit commands for (default: the baseline seed list)",
    )
    stage_plan.add_argument("--json", default=None)

    parity = subparsers.add_parser(
        "gpu-parity", help="S1: CPU/GPU contract parity, field by field"
    )
    parity.add_argument("--num-envs", type=int, default=4)
    parity.add_argument("--steps", type=int, default=24)
    parity.add_argument("--seed", type=int, default=7)
    parity.add_argument("--device", default="cuda:0")
    parity.add_argument("--no-graph", action="store_true", help="skip the CUDA-graph replay check")
    parity.add_argument("--json", default=None)

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
        "train-gpu": cmd_train_gpu,
        "evaluate-stand": cmd_evaluate_stand,
        "evaluate-lqr": cmd_evaluate_lqr,
        "step-audit": cmd_step_audit,
        "mechanism-check": cmd_mechanism_check,
        "closed-mjcf": cmd_closed_mjcf,
        "joint-conventions": cmd_joint_conventions,
        "stage-plan": cmd_stage_plan,
        "gpu-parity": cmd_gpu_parity,
        "export": cmd_export,
        "play": cmd_play,
        "render": cmd_render,
        "doctor": cmd_doctor,
    }
    handler = handlers.get(args.command, cmd_gated)
    return handler(args, config)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
