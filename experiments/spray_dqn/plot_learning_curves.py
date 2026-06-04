from __future__ import annotations

import argparse
import csv
import statistics
from pathlib import Path
from typing import Any

from orchard_world import default_output_dir


LABELS = {
    "dqn": "DQN",
    "dueling-dqn": "Dueling DQN",
    "rainbow-dqn-lite": "Rainbow DQN lite",
    "drqn": "DRQN",
    "double-dqn": "Double DQN",
}


def read_curve(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            typed = dict(row)
            for key, value in row.items():
                if key in {"algorithm"}:
                    continue
                try:
                    typed[key] = float(value)
                except (TypeError, ValueError):
                    typed[key] = value
            rows.append(typed)
    return rows


def load_curves(input_dir: Path, algorithms: set[str] | None = None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for path in sorted(input_dir.glob("seed_*/metrics/*_learning_curve.csv")):
        for row in read_curve(path):
            algorithm = str(row.get("algorithm", "")).lower().replace("_", "-")
            if algorithms and algorithm not in algorithms:
                continue
            row["algorithm"] = algorithm
            row["source"] = str(path)
            rows.append(row)
    return rows


def mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return 0.0, 0.0
    if len(values) == 1:
        return float(values[0]), 0.0
    return float(statistics.mean(values)), float(statistics.stdev(values))


def aggregate(rows: list[dict[str, Any]], metrics: list[str]) -> list[dict[str, Any]]:
    grouped: dict[tuple[str, int], list[dict[str, Any]]] = {}
    for row in rows:
        key = (str(row["algorithm"]), int(row["timesteps"]))
        grouped.setdefault(key, []).append(row)
    summary_rows: list[dict[str, Any]] = []
    for (algorithm, timesteps), items in sorted(grouped.items(), key=lambda item: (item[0][0], item[0][1])):
        summary: dict[str, Any] = {
            "algorithm": algorithm,
            "label": LABELS.get(algorithm, algorithm),
            "timesteps": timesteps,
            "n": len(items),
        }
        for metric in metrics:
            mean, std = mean_std([float(item.get(metric, 0.0)) for item in items])
            summary[f"{metric}_mean"] = mean
            summary[f"{metric}_std"] = std
        summary_rows.append(summary)
    return summary_rows


def write_csv(rows: list[dict[str, Any]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def plot(rows: list[dict[str, Any]], metrics: list[str], output: Path) -> None:
    import matplotlib.pyplot as plt

    if not rows:
        raise SystemExit("No learning-curve rows found.")
    metric_titles = {
        "episode_reward": "Evaluation return",
        "coverage_percent": "Coverage (%)",
        "demand_satisfaction_percent": "Demand satisfaction (%)",
        "goal_progress_percent": "Goal progress (%)",
    }
    algorithms = sorted({str(row["algorithm"]) for row in rows})
    fig, axes = plt.subplots(len(metrics), 1, figsize=(10, 3.4 * len(metrics)), squeeze=False)
    for ax, metric in zip(axes.flatten(), metrics):
        for algorithm in algorithms:
            algo_rows = [row for row in rows if row["algorithm"] == algorithm]
            xs = [row["timesteps"] for row in algo_rows]
            ys = [row[f"{metric}_mean"] for row in algo_rows]
            yerr = [row[f"{metric}_std"] for row in algo_rows]
            line = ax.plot(xs, ys, marker="o", linewidth=2, label=LABELS.get(algorithm, algorithm))[0]
            lower = [y - e for y, e in zip(ys, yerr)]
            upper = [y + e for y, e in zip(ys, yerr)]
            ax.fill_between(xs, lower, upper, color=line.get_color(), alpha=0.16)
        ax.set_title(metric_titles.get(metric, metric))
        ax.set_xlabel("Training timesteps")
        ax.grid(alpha=0.25)
        if metric.endswith("_percent"):
            ax.set_ylim(0, 105)
    axes.flatten()[0].legend(ncol=2)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)


def main() -> None:
    parser = argparse.ArgumentParser(description="Aggregate and plot periodic evaluation learning curves.")
    parser.add_argument("--input-dir", required=True, help="Run directory containing seed_*/metrics/*_learning_curve.csv files.")
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--algorithms", default=None, help="Comma-separated algorithm filter.")
    parser.add_argument(
        "--metrics",
        default="episode_reward,coverage_percent,demand_satisfaction_percent",
        help="Comma-separated curve metrics.",
    )
    args = parser.parse_args()

    input_dir = Path(args.input_dir)
    output_dir = Path(args.output_dir) if args.output_dir else input_dir / "summary"
    algorithms = None
    if args.algorithms:
        algorithms = {item.strip().lower().replace("_", "-") for item in args.algorithms.split(",") if item.strip()}
    metrics = [item.strip() for item in args.metrics.split(",") if item.strip()]
    rows = load_curves(input_dir, algorithms=algorithms)
    summary_rows = aggregate(rows, metrics)
    write_csv(summary_rows, output_dir / "learning_curve_summary.csv")
    plot(summary_rows, metrics, output_dir / "learning_curves.png")
    print(f"Wrote {output_dir / 'learning_curve_summary.csv'}")
    print(f"Wrote {output_dir / 'learning_curves.png'}")


if __name__ == "__main__":
    main()
