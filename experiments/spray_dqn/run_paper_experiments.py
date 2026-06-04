from __future__ import annotations

import argparse
import json
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from orchard_world import default_output_dir


REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class ExperimentConfig:
    name: str
    description: str
    algorithms: str
    timesteps: int
    flags: tuple[str, ...]


def base_enhanced_flags(goal_coverage: float) -> list[str]:
    return [
        "--goal-coverage",
        str(goal_coverage),
        "--goal-metric",
        "demand",
        "--dynamic-obstacles",
        "2",
        "--dynamic-obstacle-mode",
        "corridor",
        "--intelligent-irrigation",
    ]


def experiment_configs(args) -> list[ExperimentConfig]:
    base = tuple(base_enhanced_flags(args.goal_coverage))
    return [
        ExperimentConfig(
            name="hierarchical_5seeds",
            description="Main enhanced task: 5 seeds, 4 movement actions, auto-spray, safety override.",
            algorithms=args.main_algorithms,
            timesteps=args.main_timesteps,
            flags=base + ("--auto-spray-control", "--safety-controller"),
        ),
        ExperimentConfig(
            name="ablation_always_spray_no_safety",
            description="Ablation: 4 actions, always spray, no safety override.",
            algorithms=args.ablation_algorithms,
            timesteps=args.ablation_timesteps,
            flags=base,
        ),
        ExperimentConfig(
            name="ablation_autospray_no_safety",
            description="Ablation: 4 actions, auto-spray enabled, no safety override.",
            algorithms=args.ablation_algorithms,
            timesteps=args.ablation_timesteps,
            flags=base + ("--auto-spray-control",),
        ),
        ExperimentConfig(
            name="ablation_always_spray_safety",
            description="Ablation: 4 actions, always spray, safety override enabled.",
            algorithms=args.ablation_algorithms,
            timesteps=args.ablation_timesteps,
            flags=base + ("--safety-controller",),
        ),
        ExperimentConfig(
            name="ablation_autospray_safety",
            description="Ablation: 4 actions, auto-spray and safety override enabled.",
            algorithms=args.ablation_algorithms,
            timesteps=args.ablation_timesteps,
            flags=base + ("--auto-spray-control", "--safety-controller"),
        ),
        ExperimentConfig(
            name="ablation_explicit_spray_no_safety",
            description="Ablation: 8 actions, learned spray on/off, no safety override.",
            algorithms=args.ablation_algorithms,
            timesteps=args.ablation_timesteps,
            flags=base + ("--spray-control",),
        ),
        ExperimentConfig(
            name="ablation_explicit_spray_safety",
            description="Ablation: 8 actions, learned spray on/off, safety override enabled.",
            algorithms=args.ablation_algorithms,
            timesteps=args.ablation_timesteps,
            flags=base + ("--spray-control", "--safety-controller"),
        ),
    ]


def run(cmd: list[str], dry_run: bool) -> None:
    print("+ " + " ".join(str(part) for part in cmd), flush=True)
    if not dry_run:
        subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def run_config(args, config: ExperimentConfig) -> Path:
    output_dir = Path(args.output_root) / config.name
    cmd = [
        sys.executable,
        "experiments/spray_dqn/run_top5_multiseed.py",
        "--algorithms",
        config.algorithms,
        "--seeds",
        args.seeds,
        "--timesteps",
        str(config.timesteps),
        "--max-steps",
        str(args.max_steps),
        "--device",
        args.device,
        "--output-dir",
        str(output_dir),
        "--learning-curves",
        "--eval-freq",
        str(args.eval_freq),
    ]
    if args.aggregate_only:
        cmd.append("--aggregate-only")
    if args.force:
        cmd.append("--force")
    cmd.extend(config.flags)
    run(cmd, args.dry_run)
    if not args.dry_run and not args.skip_curve_plots:
        run(
            [
                sys.executable,
                "experiments/spray_dqn/plot_learning_curves.py",
                "--input-dir",
                str(output_dir),
                "--output-dir",
                str(output_dir / "summary"),
            ],
            args.dry_run,
        )
    return output_dir


def load_top_summary(output_dir: Path) -> dict[str, Any] | None:
    path = output_dir / "summary" / "multiseed_summary.json"
    if not path.exists():
        return None
    with path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    summary = payload.get("summary", [])
    return summary[0] if summary else None


def write_index(args, configs: list[ExperimentConfig], output_dirs: list[Path]) -> None:
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    path = output_root / "paper_experiment_index.md"
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# Paper Experiment Index\n\n")
        handle.write("## Shared Settings\n\n")
        handle.write(f"- Seeds: `{args.seeds}`\n")
        handle.write(f"- Main timesteps: `{args.main_timesteps}`\n")
        handle.write(f"- Ablation timesteps: `{args.ablation_timesteps}`\n")
        handle.write(f"- Max episode steps: `{args.max_steps}`\n")
        handle.write(f"- Success threshold: goal progress >= `{args.goal_coverage:.2f}` and zero planning collisions\n")
        handle.write("- Goal metric: demand satisfaction for intelligent irrigation runs\n")
        handle.write("- Reported statistics: mean +/- sample standard deviation over completed seeds\n\n")
        handle.write("## Experiments\n\n")
        handle.write("| Experiment | Algorithms | Description | Summary |\n")
        handle.write("|---|---|---|---|\n")
        for config, output_dir in zip(configs, output_dirs):
            summary_path = output_dir / "summary" / "multiseed_summary.md"
            handle.write(
                f"| `{config.name}` | `{config.algorithms}` | {config.description} | "
                f"`{summary_path}` |\n"
            )
        handle.write("\n## Current Top-Line Results\n\n")
        handle.write("| Experiment | Best algorithm | Success rate (%) | Goal progress (%) | Coverage (%) |\n")
        handle.write("|---|---|---:|---:|---:|\n")
        for output_dir in output_dirs:
            top = load_top_summary(output_dir)
            if not top:
                handle.write(f"| `{output_dir.name}` | pending | - | - | - |\n")
                continue
            handle.write(
                f"| `{output_dir.name}` | {top.get('label', top.get('algorithm'))} | "
                f"{top.get('success_rate_percent', 0.0):.1f} | "
                f"{top.get('goal_progress_percent_mean', 0.0):.1f} +/- {top.get('goal_progress_percent_std', 0.0):.1f} | "
                f"{top.get('coverage_percent_mean', 0.0):.1f} +/- {top.get('coverage_percent_std', 0.0):.1f} |\n"
            )
    print(f"Wrote {path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Run the paper-grade 5-seed and ablation experiment suite.")
    parser.add_argument("--output-root", default=str(default_output_dir() / "paper_required_experiments"))
    parser.add_argument("--seeds", default="7,11,19,23,31")
    parser.add_argument("--main-algorithms", default="dqn,drqn,dueling-dqn,rainbow-dqn-lite")
    parser.add_argument("--ablation-algorithms", default="dqn,drqn")
    parser.add_argument("--main-timesteps", type=int, default=15000)
    parser.add_argument("--ablation-timesteps", type=int, default=10000)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--goal-coverage", type=float, default=0.90)
    parser.add_argument("--eval-freq", type=int, default=5000)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--aggregate-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--skip-main", action="store_true")
    parser.add_argument("--skip-ablations", action="store_true")
    parser.add_argument("--skip-curve-plots", action="store_true")
    args = parser.parse_args()

    configs = experiment_configs(args)
    selected = []
    for config in configs:
        if args.skip_main and config.name == "hierarchical_5seeds":
            continue
        if args.skip_ablations and config.name != "hierarchical_5seeds":
            continue
        selected.append(config)
    output_dirs = [run_config(args, config) for config in selected]
    if not args.dry_run:
        write_index(args, selected, output_dirs)


if __name__ == "__main__":
    main()
