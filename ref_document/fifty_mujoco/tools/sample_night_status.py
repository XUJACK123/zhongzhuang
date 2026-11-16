#!/usr/bin/env python3
"""Refresh the user-requested Chinese progress section from this night's files."""
import json
from datetime import datetime
from pathlib import Path
from zoneinfo import ZoneInfo

PROJECT = Path(__file__).resolve().parents[1]
WORKSPACE = PROJECT.parents[1]


def lines(path):
    result = []
    if path.exists():
        for line in path.read_text().splitlines():
            try:
                result.append(json.loads(line))
            except json.JSONDecodeError:
                pass  # A concurrently written last line is read next time.
    return result


def main():
    final_path = PROJECT/"runs/night_20261116/final_summary.json"
    if final_path.exists():
        final = json.loads(final_path.read_text())
        print(json.dumps({"status": "completed", "finished_at": final["finished_at"],
                          "summary": str(final_path.relative_to(PROJECT))}, ensure_ascii=False))
        return  # Preserve the final Chinese record instead of restoring live text.
    run = PROJECT/"runs/qualified_combined_v1_20261116"
    rows, evaluations = lines(run/"rows.jsonl"), lines(run/"evaluations.jsonl")
    last = rows[-1] if rows else {}
    complete = json.loads((run/"training_report.json").read_text()) if (run/"training_report.json").exists() else None
    best = min(evaluations, key=lambda x: x["score"]) if evaluations else None
    corners = lines(PROJECT/"reports/sample_weight/qualified_u200_corners_20261116.rows.jsonl")
    expanded = lines(PROJECT/"reports/sample_weight/qualified_u400_development_20261116.rows.jsonl")
    current_corners = lines(PROJECT/"reports/sample_weight/qualified_u400_corners_20261116.rows.jsonl")
    heldout_nominal = lines(PROJECT/"reports/sample_weight/qualified_final_nominal_20261116.rows.jsonl")
    heldout_combined = lines(PROJECT/"reports/sample_weight/qualified_final_combined_20261116.rows.jsonl")
    stamp = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y-%m-%d %H:%M")
    status = {"time": stamp, "run": str(run.relative_to(PROJECT)), "update": last.get("update", 0),
              "training_elapsed_s": complete["wall_seconds"] if complete else last.get("elapsed_s", 0), "episodes": last.get("episodes_ended", 0),
              "training_failures": sum(r["window_failures"] for r in rows),
              "best_development": {"update": best["update"], "score": best["score"], "all_passed": best["all_classes_threshold_met"]} if best else None,
              "stop_reason": complete["stop_reason"] if complete else None,
              "corners_completed": len(corners), "corners_passed": sum(r["thresholds_met"] for r in corners),
              "corners_physical_failures": sum(not r["completed"] for r in corners),
              "u400_expanded_completed": len(expanded), "u400_expanded_passed": sum(r["thresholds_met"] for r in expanded),
              "u400_corners_completed": len(current_corners), "u400_corners_passed": sum(r["thresholds_met"] for r in current_corners),
              "heldout_nominal_completed": len(heldout_nominal), "heldout_nominal_passed": sum(r["thresholds_met"] for r in heldout_nominal),
              "heldout_combined_completed": len(heldout_combined), "heldout_combined_passed": sum(r["thresholds_met"] for r in heldout_combined)}
    state = "已停止（开发指标进入平台期）" if complete and complete["stop_reason"] == "development_plateau" else f"已停止（{complete['stop_reason']}）" if complete else "日志持续更新，尚无结束报告"
    development = f"当前最佳update {best['update']}，综合误差{best['score']:.4f}（旧权重同集0.5339）；五类{'全部达标' if best['all_classes_threshold_met'] else '未全部达标'}" if best else "尚未完成"
    block = f'''### 本轮最新进展（{stamp}，Asia/Shanghai）

- **工作预算**：01:35开始，最多约6小时，最迟07:35结束。用户已授权自主处理。
- **起身暂不训练**：16次原协议动态探针全部失败；近站姿约68 ms、支撑测量姿态约24 ms触膝限位，失能静置无法生成合法reset。324组对称有界动作扫描未找到通过现有启动条件的方案；1/0.5/0.25 ms物理步长下零力矩触限时间一致约66/24 ms。没有提升电机上限或修改生产限位。
- **瓶颈溯源**：当前伸展端限位继承自旧约228 mm安装长度弹簧，当前目标弹簧自由长为250 mm。单独的弹簧/分支几何候选在零力矩下仍约33.75/73.25 ms到达边界，未提升为实际机械限位；当前所有关节armature为0，未补入电机转子折算惯量。这不是实车起身不可能的证明；需要核实失能停位、机械干涉、惯量/摩擦、弹簧及带负载使能过程。
- **本轮继续的训练**：已站稳初态的平衡与联合随机化。新`QualifiedHardwareEnv31`直接调用当前固件C里程计质量与参考管理；理想IMU/编码器输入，不包含真实IMU滤波、CAN延迟、硬件启动或起身。
- **训练最新状态**：`runs/qualified_combined_v1_20261116`，{state}。已完成{status['update']}次PPO更新、{status['update']*1280:,}个转移，训练器已用{status['training_elapsed_s']/60:.1f}分钟，已结束{status['episodes']}回合、失败{status['training_failures']}次。四个腿部电机gain0.8–1.0、摩擦0.6–0.9、弹簧倍率0.9–1.1，20%名义样本；20环境×64步，单段≤1小时，开发平台期早停。
- **开发评估**：{development}。小样本开发指标仅用于选型。旧权重在新闭环上的基线：名义10/10×10 s、联合100/100×40 s。
- **联合边界检查**：update200候选已完成{len(corners)}/320场，达标{status['corners_passed']}场，物理失败{status['corners_physical_failures']}场；四个腿部电机各取0.8/1.0，摩擦0.6/0.9，弹簧0.9/1.1，每组合五类动作20 s。这是开发角点测试。
- **扩大开发验证**：update400已完成{len(expanded)}/100场，每场40 s，达标{status['u400_expanded_passed']}场；种子从23000000起，独立于当前训练日志。
- **当前候选边界验证**：update400已完成{len(current_corners)}/320场，达标{status['u400_corners_passed']}场；不继承update200的通过结论。
- **独立验收进行中**：02:24冻结update400（SHA-256前缀`1e7041f60fc411e4`）、评估代码与阈值。名义已完成{len(heldout_nominal)}/200场、达标{status['heldout_nominal_passed']}场；联合已完成{len(heldout_combined)}/500场、达标{status['heldout_combined_passed']}场，均为40 s。训练/开发/保留种子交集为空，不按保留结果调参。
- **验证与追溯**：针对性60项检查及9项验收判定测试通过，含300步C/Python观测/actor对拍、质量降级/恢复、跨回合清理及错误验收拒绝；PPO冒烟3次更新/96转移、恢复后2次更新/64转移及独立导出工具512组C/Torch对拍通过。源码快照与种子约定保存在`runs/night_20261116/manifest.json`。原冻结u100和原始归档未覆盖。
- **证据缺口**：旧离线汇总依赖的OpenOCD日志在本机缺失；保留原报告，不将缺失记录当作新增实测。宿主GPU可用，当前小网络/CPU物理训练使用CPU。
- **下一步**：完成独立验收并独立重算各类通过率/指标，整理候选包与结果。名义种子30000000起、联合40000000起；站立、转向、换向三段连续40 s视频/轨迹已完成。起身需先修复/验证被动模型和启动条件；不自动烧录或宣称实车通过。

报告：[起身动态探针](ref_document/fifty_mujoco/reports/sample_weight/getup_probe_initial_20261116.json)、[启动与步长扫描](ref_document/fifty_mujoco/reports/sample_weight/getup_feasibility_20261116.json)、[限位来源](ref_document/fifty_mujoco/reports/sample_weight/getup_limit_audit_20261116.json)、[起身诊断图](ref_document/fifty_mujoco/reports/sample_weight/figures/getup_feasibility_20261116.png)、[联合基线](ref_document/fifty_mujoco/reports/sample_weight/qualified_baseline_combined_20261116.json)。

'''
    path = WORKSPACE/"progress.md"
    text = path.read_text()
    start, end = text.index("### 本轮最新进展"), text.index("**本轮实机参数迁移")
    text = text[:start]+block+text[end:]
    pieces = text.splitlines()
    pieces[2] = f"> 更新：{stamp}（Asia/Shanghai，本轮夜间训练与验证）"
    path.write_text("\n".join(pieces)+"\n")
    (PROJECT/"runs/night_20261116/latest_status.json").write_text(json.dumps(status, indent=2)+"\n")
    print(json.dumps(status, ensure_ascii=False))


if __name__ == "__main__":
    main()
