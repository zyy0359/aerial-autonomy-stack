from __future__ import annotations

import argparse
import csv
import json
import statistics
import subprocess
import sys
from pathlib import Path
from typing import Any

from orchard_world import DEFAULT_WORLD, OrchardWorldGrid, default_output_dir, evaluate_path


REPO_ROOT = Path(__file__).resolve().parents[2]
LABELS = {
    "dqn": "DQN",
    "ppo": "PPO",
    "double-dqn": "Double DQN",
    "dueling-dqn": "Dueling DQN",
    "rainbow-dqn-lite": "Rainbow DQN lite",
    "drqn": "DRQN",
}


def run(cmd: list[str]) -> None:
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=REPO_ROOT, check=True)


def parse_seeds(value: str) -> list[int]:
    return [int(item.strip()) for item in value.split(",") if item.strip()]


def maybe_run(cmd: list[str], expected_summary: Path, force: bool) -> None:
    if expected_summary.exists() and not force:
        print(f"skip existing {expected_summary}", flush=True)
        return
    run(cmd)


def enhanced_flags(args) -> list[str]:
    flags = [
        "--goal-metric",
        args.goal_metric,
        "--dynamic-obstacles",
        str(args.dynamic_obstacles),
        "--dynamic-obstacle-span",
        str(args.dynamic_obstacle_span),
        "--dynamic-safety-radius",
        str(args.dynamic_safety_radius),
        "--dynamic-obstacle-mode",
        args.dynamic_obstacle_mode,
    ]
    if args.intelligent_irrigation:
        flags.append("--intelligent-irrigation")
    if args.spray_control:
        flags.append("--spray-control")
    return flags


def train_seed(args, seed: int) -> None:
    seed_dir = Path(args.output_dir) / f"seed_{seed}"
    model_dir = seed_dir / "models"
    metrics_dir = seed_dir / "metrics"
    model_dir.mkdir(parents=True, exist_ok=True)
    metrics_dir.mkdir(parents=True, exist_ok=True)

    algorithms = set(args.algorithms)
    if "dqn" in algorithms:
        maybe_run(
            [
                sys.executable,
                "experiments/spray_dqn/train_dqn.py",
                "--world",
                args.world,
                "--cell-size",
                str(args.cell_size),
                "--goal-coverage",
                str(args.goal_coverage),
                "--timesteps",
                str(args.timesteps),
                "--seed",
                str(seed),
                "--device",
                args.device,
                "--model-out",
                str(model_dir / "dqn"),
                "--summary-out",
                str(metrics_dir / "dqn_summary.json"),
            ]
            + enhanced_flags(args),
            metrics_dir / "dqn_summary.json",
            args.force,
        )

    sb3_algorithms = [name for name in args.algorithms if name == "ppo"]
    if sb3_algorithms:
        expected = [metrics_dir / f"{name}_summary.json" for name in sb3_algorithms]
        if args.force or any(not path.exists() for path in expected):
            run(
                [
                    sys.executable,
                    "experiments/spray_dqn/train_sb3_algorithms.py",
                    "--world",
                    args.world,
                    "--cell-size",
                    str(args.cell_size),
                    "--goal-coverage",
                    str(args.goal_coverage),
                    "--timesteps",
                    str(args.timesteps),
                    "--seed",
                    str(seed),
                    "--device",
                    args.device,
                    "--algorithms",
                    ",".join(sb3_algorithms),
                    "--model-dir",
                    str(model_dir),
                    "--metrics-dir",
                    str(metrics_dir),
                ]
                + enhanced_flags(args)
            )
        else:
            print(f"skip existing {', '.join(str(path) for path in expected)}", flush=True)

    variant_algorithms = [
        name for name in args.algorithms
        if name in {"double-dqn", "dueling-dqn", "rainbow-dqn-lite"}
    ]
    if variant_algorithms:
        expected = [metrics_dir / f"{name}_summary.json" for name in variant_algorithms]
        if args.force or any(not path.exists() for path in expected):
            run(
                [
                    sys.executable,
                    "experiments/spray_dqn/train_dqn_variants.py",
                    "--world",
                    args.world,
                    "--cell-size",
                    str(args.cell_size),
                    "--goal-coverage",
                    str(args.goal_coverage),
                    "--timesteps",
                    str(args.timesteps),
                    "--seed",
                    str(seed),
                    "--device",
                    args.device,
                    "--algorithms",
                    ",".join(variant_algorithms),
                    "--model-dir",
                    str(model_dir),
                    "--metrics-dir",
                    str(metrics_dir),
                ]
                + enhanced_flags(args)
            )
        else:
            print(f"skip existing {', '.join(str(path) for path in expected)}", flush=True)

    if "drqn" in algorithms:
        maybe_run(
            [
                sys.executable,
                "experiments/spray_dqn/train_drqn.py",
                "--world",
                args.world,
                "--cell-size",
                str(args.cell_size),
                "--goal-coverage",
                str(args.goal_coverage),
                "--timesteps",
                str(args.timesteps),
                "--seed",
                str(seed),
                "--device",
                args.device,
                "--model-out",
                str(model_dir / "drqn.pt"),
                "--summary-out",
                str(metrics_dir / "drqn_summary.json"),
            ]
            + enhanced_flags(args),
            metrics_dir / "drqn_summary.json",
            args.force,
        )


def metric_goal_value(metrics: dict[str, Any], goal_metric: str) -> float:
    if goal_metric == "demand":
        return float(metrics.get("demand_satisfaction", metrics.get("coverage", 0.0)))
    return float(metrics.get("coverage", 0.0))


def load_seed_rows(
    output_dir: Path,
    seeds: list[int],
    algorithms: list[str],
    grid: OrchardWorldGrid,
    goal_metric: str,
    goal_coverage: float,
) -> list[dict[str, Any]]:
    rows = []
    for seed in seeds:
        metrics_dir = output_dir / f"seed_{seed}" / "metrics"
        for algorithm in algorithms:
            summary_path = metrics_dir / f"{algorithm}_summary.json"
            if not summary_path.exists():
                rows.append({"seed": seed, "algorithm": algorithm, "missing": str(summary_path)})
                continue
            with summary_path.open("r", encoding="utf-8") as handle:
                payload = json.load(handle)
            if "path" not in payload:
                rows.append({"seed": seed, "algorithm": algorithm, "missing": "summary has no path"})
                continue
            path = [tuple(cell) for cell in payload["path"]]
            metrics = evaluate_path(grid, path)
            metrics.update(payload.get("final_metrics", {}))
            goal_value = metric_goal_value(metrics, goal_metric)
            rows.append(
                {
                    "seed": seed,
                    "algorithm": algorithm,
                    "label": LABELS.get(algorithm, algorithm),
                    "goal_progress_percent": goal_value * 100.0,
                    "coverage_percent": float(metrics.get("coverage", 0.0)) * 100.0,
                    "demand_satisfaction_percent": float(
                        metrics.get("demand_satisfaction", metrics.get("coverage", 0.0))
                    )
                    * 100.0,
                    "path_length_m": float(metrics.get("path_length_m", 0.0)),
                    "repeat_spray_percent": float(metrics.get("repeat_spray_ratio", 0.0)) * 100.0,
                    "collisions": float(metrics.get("collisions", 0.0)),
                    "dynamic_collisions": float(metrics.get("dynamic_collisions", 0.0)),
                    "min_safety_value": float(metrics.get("min_safety_value", 1.0)),
                    "dose_rmse": float(metrics.get("dose_rmse", 0.0)),
                    "over_spray_percent": float(metrics.get("over_spray_ratio", 0.0)) * 100.0,
                    "waypoint_count": float(metrics.get("waypoint_count", len(path))),
                    "success": int(goal_value >= goal_coverage and float(metrics.get("collisions", 0.0)) == 0.0),
                    "summary": str(summary_path),
                }
            )
    return rows


def mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return 0.0, 0.0
    if len(values) == 1:
        return float(values[0]), 0.0
    return float(statistics.mean(values)), float(statistics.stdev(values))


def aggregate(rows: list[dict[str, Any]], algorithms: list[str]) -> list[dict[str, Any]]:
    summary_rows = []
    for algorithm in algorithms:
        completed = [row for row in rows if row.get("algorithm") == algorithm and "missing" not in row]
        successful = [row for row in completed if row.get("success") == 1]
        success_path = [row["path_length_m"] for row in successful]
        success_repeat = [row["repeat_spray_percent"] for row in successful]
        success_waypoints = [row["waypoint_count"] for row in successful]
        summary = {
            "algorithm": algorithm,
            "label": LABELS.get(algorithm, algorithm),
            "n": len(completed),
            "success_rate_percent": 0.0 if not completed else 100.0 * statistics.mean(row["success"] for row in completed),
            "success_n": len(successful),
        }
        for name in (
            "goal_progress_percent",
            "coverage_percent",
            "demand_satisfaction_percent",
            "path_length_m",
            "repeat_spray_percent",
            "collisions",
            "dynamic_collisions",
            "min_safety_value",
            "dose_rmse",
            "over_spray_percent",
            "waypoint_count",
        ):
            mean, std = mean_std([row[name] for row in completed])
            summary[f"{name}_mean"] = mean
            summary[f"{name}_std"] = std
        summary["success_path_length_mean"], summary["success_path_length_std"] = mean_std(success_path)
        summary["success_repeat_spray_mean"], summary["success_repeat_spray_std"] = mean_std(success_repeat)
        summary["success_waypoint_mean"], summary["success_waypoint_std"] = mean_std(success_waypoints)
        summary_rows.append(summary)
    return sorted(
        summary_rows,
        key=lambda row: (
            -row["success_rate_percent"],
            -row["goal_progress_percent_mean"],
            row["collisions_mean"],
            row["dynamic_collisions_mean"],
            row["path_length_m_mean"],
        ),
    )


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = sorted({key for row in rows for key in row.keys()})
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def fmt(row: dict[str, Any], key: str, digits: int = 1) -> str:
    return f"{row[f'{key}_mean']:.{digits}f} +/- {row[f'{key}_std']:.{digits}f}"


def write_markdown(path: Path, summary_rows: list[dict[str, Any]], seed_rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write(
            "| Rank | Algorithm | Success rate (%) | Goal progress (%) | Coverage (%) | "
            "Demand satisfaction (%) | Path length (m) | Repeat spray (%) | Collisions | "
            "Dynamic collisions | Min S_v | Waypoints |\n"
        )
        handle.write("|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for index, row in enumerate(summary_rows, start=1):
            handle.write(
                f"| {index} | {row['label']} | {row['success_rate_percent']:.1f} | "
                f"{fmt(row, 'goal_progress_percent')} | "
                f"{fmt(row, 'coverage_percent')} | "
                f"{fmt(row, 'demand_satisfaction_percent')} | "
                f"{fmt(row, 'path_length_m')} | "
                f"{fmt(row, 'repeat_spray_percent')} | "
                f"{fmt(row, 'collisions')} | "
                f"{fmt(row, 'dynamic_collisions')} | "
                f"{fmt(row, 'min_safety_value', 2)} | "
                f"{fmt(row, 'waypoint_count')} |\n"
            )
        handle.write("\nSuccessful trials only:\n\n")
        handle.write("| Algorithm | Successful seeds | Path length (m) | Repeat spray (%) | Waypoints |\n")
        handle.write("|---|---:|---:|---:|---:|\n")
        for row in summary_rows:
            handle.write(
                f"| {row['label']} | {row['success_n']} | "
                f"{row['success_path_length_mean']:.1f} +/- {row['success_path_length_std']:.1f} | "
                f"{row['success_repeat_spray_mean']:.1f} +/- {row['success_repeat_spray_std']:.1f} | "
                f"{row['success_waypoint_mean']:.1f} +/- {row['success_waypoint_std']:.1f} |\n"
            )
        handle.write("\nPer-seed results:\n\n")
        handle.write(
            "| Seed | Algorithm | Goal progress (%) | Coverage (%) | Demand satisfaction (%) | "
            "Path length (m) | Repeat spray (%) | Collisions | Dynamic collisions | Min S_v | Waypoints |\n"
        )
        handle.write("|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for row in sorted(
            [item for item in seed_rows if "missing" not in item],
            key=lambda item: (item["seed"], item["algorithm"]),
        ):
            handle.write(
                f"| {row['seed']} | {row['label']} | {row['goal_progress_percent']:.1f} | "
                f"{row['coverage_percent']:.1f} | {row['demand_satisfaction_percent']:.1f} | "
                f"{row['path_length_m']:.1f} | {row['repeat_spray_percent']:.1f} | "
                f"{row['collisions']:.0f} | {row['dynamic_collisions']:.0f} | "
                f"{row['min_safety_value']:.2f} | {row['waypoint_count']:.0f} |\n"
            )


def plot_summary(path: Path, summary_rows: list[dict[str, Any]]) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        return

    labels = [row["label"] for row in summary_rows]
    metrics = [
        ("goal_progress_percent", "Goal progress (%)"),
        ("path_length_m", "Path length (m)"),
        ("min_safety_value", "Min S_v"),
        ("success_rate_percent", "Success rate (%)"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8))
    for ax, (key, title) in zip(axes.flatten(), metrics):
        if key == "success_rate_percent":
            means = [row[key] for row in summary_rows]
            errors = None
        else:
            means = [row[f"{key}_mean"] for row in summary_rows]
            errors = [row[f"{key}_std"] for row in summary_rows]
        ax.bar(labels, means, yerr=errors, capsize=4)
        ax.set_title(title)
        if "percent" in key:
            ax.set_ylim(0, 105)
        if key == "min_safety_value":
            ax.set_ylim(0, 1.05)
        ax.grid(axis="y", alpha=0.25)
        ax.tick_params(axis="x", rotation=20)
    fig.suptitle("RL algorithms across random seeds")
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run and aggregate RL algorithms over multiple random seeds.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--goal-metric", choices=["coverage", "demand"], default="coverage")
    parser.add_argument("--timesteps", type=int, default=20000)
    parser.add_argument("--seeds", default="7,11,19")
    parser.add_argument("--algorithms", default="dqn,ppo,double-dqn,dueling-dqn,rainbow-dqn-lite")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--dynamic-obstacles", type=int, default=0)
    parser.add_argument("--dynamic-obstacle-span", type=int, default=4)
    parser.add_argument("--dynamic-safety-radius", type=int, default=3)
    parser.add_argument("--dynamic-obstacle-mode", choices=["random", "corridor"], default="random")
    parser.add_argument("--intelligent-irrigation", action="store_true")
    parser.add_argument("--spray-control", action="store_true")
    parser.add_argument("--output-dir", default=str(default_output_dir() / "multiseed_20260513_top5"))
    parser.add_argument("--aggregate-only", action="store_true")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    args.algorithms = [item.strip() for item in args.algorithms.split(",") if item.strip()]
    seeds = parse_seeds(args.seeds)

    if not args.aggregate_only:
        for seed in seeds:
            print(f"=== seed {seed} ===", flush=True)
            train_seed(args, seed)

    output_dir = Path(args.output_dir)
    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size)
    seed_rows = load_seed_rows(output_dir, seeds, args.algorithms, grid, args.goal_metric, args.goal_coverage)
    summary_rows = aggregate(seed_rows, args.algorithms)

    result_dir = output_dir / "summary"
    result_dir.mkdir(parents=True, exist_ok=True)
    with (result_dir / "multiseed_summary.json").open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(
            {
                "seeds": seeds,
                "goal_coverage": args.goal_coverage,
                "goal_metric": args.goal_metric,
                "dynamic_obstacles": args.dynamic_obstacles,
                "dynamic_obstacle_mode": args.dynamic_obstacle_mode,
                "intelligent_irrigation": args.intelligent_irrigation,
                "spray_control": args.spray_control,
                "summary": summary_rows,
                "per_seed": seed_rows,
            },
            handle,
            indent=2,
        )
    write_csv(result_dir / "multiseed_per_seed.csv", seed_rows)
    write_csv(result_dir / "multiseed_summary.csv", summary_rows)
    write_markdown(result_dir / "multiseed_summary.md", summary_rows, seed_rows)
    plot_summary(result_dir / "multiseed_summary.png", summary_rows)
    print(json.dumps(summary_rows, indent=2))


if __name__ == "__main__":
    main()
