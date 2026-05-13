from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

from orchard_world import (
    DEFAULT_WORLD,
    OrchardWorldGrid,
    default_output_dir,
    evaluate_path,
    nearest_target_path,
    orchard_row_path,
)


LABELS = {
    "orchard-row": "Orchard row",
    "nearest-target": "Nearest target",
    "dqn": "DQN",
    "ppo": "PPO",
    "a2c": "A2C",
    "sac": "SAC",
    "td3": "TD3",
    "maskable-ppo": "Maskable PPO",
    "double-dqn": "Double DQN",
    "dueling-dqn": "Dueling DQN",
    "rainbow-dqn-lite": "Rainbow DQN lite",
    "drqn": "DRQN",
}

COLORS = {
    "orchard-row": "#1971c2",
    "nearest-target": "#2f9e44",
    "dqn": "#e03131",
    "ppo": "#9c36b5",
    "a2c": "#f08c00",
    "sac": "#0b7285",
    "td3": "#5f3dc4",
    "maskable-ppo": "#c2255c",
    "double-dqn": "#1864ab",
    "dueling-dqn": "#2b8a3e",
    "rainbow-dqn-lite": "#e67700",
    "drqn": "#087f5b",
}


def coverage_trace(grid: OrchardWorldGrid, path: list[tuple[int, int]]) -> list[float]:
    sprayed: set[tuple[int, int]] = set()
    trace = []
    for cell in path:
        if grid.in_bounds(cell) and cell not in grid.obstacle_cells:
            for spray_cell in grid.spray_cells(cell):
                if spray_cell in grid.target_cells:
                    sprayed.add(spray_cell)
        trace.append(0.0 if not grid.target_cells else len(sprayed) / len(grid.target_cells))
    return trace


def add_result(results: dict[str, Any], name: str, metrics: dict[str, Any], path: list[Any], source: str) -> None:
    typed_path = [tuple(cell) for cell in path]
    results[name] = {
        "label": LABELS.get(name, name),
        "source": source,
        "metrics": metrics,
        "path": typed_path,
    }


def load_existing_comparison(path: Path, results: dict[str, Any]) -> None:
    if not path.exists():
        return
    with path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    for name, payload in data.get("algorithms", {}).items():
        if "metrics" in payload and "path" in payload:
            add_result(results, name, payload["metrics"], payload["path"], str(path))


def load_summaries(metrics_dir: Path, results: dict[str, Any], grid: OrchardWorldGrid) -> dict[str, str]:
    skipped: dict[str, str] = {}
    for path in sorted(metrics_dir.glob("*_summary.json")):
        with path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        algorithm = payload.get("algorithm")
        if not algorithm:
            continue
        algorithm = str(algorithm).lower().replace("_", "-")
        if "skipped" in payload:
            skipped[algorithm] = str(payload["skipped"])
            continue
        if "path" not in payload:
            continue
        typed_path = [tuple(cell) for cell in payload["path"]]
        metrics = evaluate_path(grid, typed_path)
        metrics.update(payload.get("final_metrics", {}))
        add_result(results, algorithm, metrics, typed_path, str(path))
    return skipped


def add_planners(grid: OrchardWorldGrid, results: dict[str, Any]) -> None:
    for name, path in {
        "orchard-row": orchard_row_path(grid),
        "nearest-target": nearest_target_path(grid),
    }.items():
        add_result(results, name, evaluate_path(grid, path), path, "computed baseline")


def ranked_rows(results: dict[str, Any]) -> list[dict[str, Any]]:
    rows = []
    for name, result in results.items():
        metrics = result["metrics"]
        rows.append(
            {
                "algorithm": name,
                "label": LABELS.get(name, name),
                "coverage_percent": metrics["coverage"] * 100.0,
                "path_length_m": metrics["path_length_m"],
                "repeat_spray_percent": metrics["repeat_spray_ratio"] * 100.0,
                "collisions": metrics["collisions"],
                "waypoint_count": metrics["waypoint_count"],
                "source": result["source"],
            }
        )
    return sorted(
        rows,
        key=lambda row: (
            -row["coverage_percent"],
            row["collisions"],
            row["path_length_m"],
            row["repeat_spray_percent"],
            row["waypoint_count"],
        ),
    )


def write_csv(rows: list[dict[str, Any]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_markdown(rows: list[dict[str, Any]], skipped: dict[str, str], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("| Rank | Algorithm | Coverage (%) | Path length (m) | Repeat spray (%) | Collisions | Waypoints |\n")
        handle.write("|---:|---|---:|---:|---:|---:|---:|\n")
        for index, row in enumerate(rows, start=1):
            handle.write(
                f"| {index} | {row['label']} | {row['coverage_percent']:.1f} | "
                f"{row['path_length_m']:.1f} | {row['repeat_spray_percent']:.1f} | "
                f"{row['collisions']} | {row['waypoint_count']} |\n"
            )
        if skipped:
            handle.write("\nSkipped algorithms:\n\n")
            for name, reason in skipped.items():
                handle.write(f"- {LABELS.get(name, name)}: {reason}\n")


def plot_metric_bars(rows: list[dict[str, Any]], output: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for comparison plots.") from exc

    metrics = [
        ("coverage_percent", "Coverage (%)"),
        ("path_length_m", "Path length (m)"),
        ("repeat_spray_percent", "Repeat spray (%)"),
        ("collisions", "Collisions"),
    ]
    labels = [row["label"] for row in rows]
    colors = [COLORS.get(row["algorithm"], "#495057") for row in rows]
    fig, axes = plt.subplots(2, 2, figsize=(max(12, 1.5 * len(rows)), 8))
    for ax, (key, title) in zip(axes.flatten(), metrics):
        values = [row[key] for row in rows]
        ax.bar(labels, values, color=colors)
        ax.set_title(title)
        if key in {"coverage_percent", "repeat_spray_percent"}:
            ax.set_ylim(0, 105)
        elif key == "collisions":
            ax.set_ylim(0, max(1.0, max(values) + 1.0))
        ax.grid(axis="y", alpha=0.25)
        ax.tick_params(axis="x", rotation=25)
        for index, value in enumerate(values):
            text = f"{value:.1f}" if key != "collisions" else f"{value:.0f}"
            ax.text(index, value, text, ha="center", va="bottom", fontsize=8)
    fig.suptitle("RL and baseline spray path planning comparison")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def plot_coverage(results: dict[str, Any], rows: list[dict[str, Any]], grid: OrchardWorldGrid, output: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for comparison plots.") from exc

    fig, ax = plt.subplots(figsize=(11, 6))
    for row in rows:
        name = row["algorithm"]
        trace = coverage_trace(grid, results[name]["path"])
        ax.plot(
            range(len(trace)),
            [value * 100.0 for value in trace],
            label=row["label"],
            color=COLORS.get(name),
            linewidth=2,
        )
    ax.set_title("Coverage over waypoints")
    ax.set_xlabel("Waypoint index")
    ax.set_ylabel("Coverage (%)")
    ax.set_ylim(0, 105)
    ax.grid(alpha=0.25)
    ax.legend(ncol=2)
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def plot_paths(results: dict[str, Any], rows: list[dict[str, Any]], grid: OrchardWorldGrid, output: Path) -> None:
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError as exc:
        raise SystemExit("matplotlib and numpy are required for path plots.") from exc

    cols = min(3, len(rows))
    plot_rows = math.ceil(len(rows) / cols)
    fig, axes = plt.subplots(plot_rows, cols, figsize=(5 * cols, 5 * plot_rows), squeeze=False)
    cmap = ListedColormap(["#f8f9fa", "#8fd694", "#2f9e44", "#495057"])
    for ax, row in zip(axes.flatten(), rows):
        name = row["algorithm"]
        path = results[name]["path"]
        sprayed: set[tuple[int, int]] = set()
        for cell in path:
            if grid.in_bounds(cell) and cell not in grid.obstacle_cells:
                for spray_cell in grid.spray_cells(cell):
                    if spray_cell in grid.target_cells:
                        sprayed.add(spray_cell)
        layers = np.zeros((grid.rows, grid.cols), dtype=int)
        for cell in grid.target_cells:
            layers[cell] = 1
        for cell in sprayed & grid.target_cells:
            layers[cell] = 2
        for cell in grid.obstacle_cells:
            layers[cell] = 3

        ax.imshow(layers, cmap=cmap, origin="upper")
        if path:
            ys = [cell[0] for cell in path]
            xs = [cell[1] for cell in path]
            ax.plot(xs, ys, color=COLORS.get(name, "#212529"), linewidth=1.4, marker="o", markersize=2)
            ax.scatter([xs[0]], [ys[0]], color="#212529", s=45)
            ax.scatter([xs[-1]], [ys[-1]], color="#f08c00", s=45)
        ax.set_title(f"{row['label']}: cov={row['coverage_percent']:.1f}%, len={row['path_length_m']:.0f}m")
        ax.set_xticks([])
        ax.set_yticks([])
    for ax in axes.flatten()[len(rows):]:
        ax.axis("off")
    fig.suptitle("Path overlays on the SDF-derived orchard grid")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def main() -> None:
    parser = argparse.ArgumentParser(description="Build paper-ready tables and plots for RL algorithm comparison.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--metrics-dir", default=str(default_output_dir() / "metrics"))
    parser.add_argument("--comparison-json", default=str(default_output_dir() / "comparison" / "comparison.json"))
    parser.add_argument("--output-dir", default=str(default_output_dir() / "paper_comparison"))
    args = parser.parse_args()

    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size)
    results: dict[str, Any] = {}
    add_planners(grid, results)
    load_existing_comparison(Path(args.comparison_json), results)
    skipped = load_summaries(Path(args.metrics_dir), results, grid)
    rows = ranked_rows(results)

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "world_summary": grid.active_target_summary(),
        "ranking": rows,
        "skipped": skipped,
        "algorithms": {
            name: {
                "metrics": result["metrics"],
                "path": result["path"],
                "source": result["source"],
            }
            for name, result in results.items()
        },
    }
    with (output_dir / "rl_benchmark.json").open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(payload, handle, indent=2)
    write_csv(rows, output_dir / "rl_benchmark_table.csv")
    write_markdown(rows, skipped, output_dir / "rl_benchmark_table.md")
    plot_metric_bars(rows, output_dir / "rl_benchmark_metrics.png")
    plot_coverage(results, rows, grid, output_dir / "rl_coverage_over_waypoints.png")
    plot_paths(results, rows, grid, output_dir / "rl_path_overlays.png")

    print(json.dumps({"ranking": rows, "skipped": skipped}, indent=2))


if __name__ == "__main__":
    main()
