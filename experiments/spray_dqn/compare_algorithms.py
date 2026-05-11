from __future__ import annotations

import argparse
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


ALGORITHM_LABELS = {
    "orchard-row": "Orchard row",
    "nearest-target": "Nearest target",
    "dqn": "DQN",
}
COLORS = {
    "orchard-row": "#1971c2",
    "nearest-target": "#2f9e44",
    "dqn": "#e03131",
}


def dqn_path(world: str, model_path: str, cell_size: float, goal_coverage: float) -> list[tuple[int, int]]:
    try:
        from stable_baselines3 import DQN
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required to compare DQN.") from exc
    from orchard_dqn_env import OrchardDQNEnv

    env = OrchardDQNEnv(world_path=world, cell_size_m=cell_size, goal_coverage=goal_coverage)
    model = DQN.load(model_path, env=env)
    obs, _ = env.reset()
    terminated = False
    truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(int(action))
    return env.path


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


def get_algorithm_path(
    algorithm: str,
    grid: OrchardWorldGrid,
    world: str,
    model: str,
    cell_size: float,
    goal_coverage: float,
) -> list[tuple[int, int]] | None:
    if algorithm == "orchard-row":
        return orchard_row_path(grid)
    if algorithm == "nearest-target":
        return nearest_target_path(grid)
    if algorithm == "dqn":
        model_path = Path(model)
        if not model_path.exists():
            return None
        return dqn_path(world, str(model_path), cell_size, goal_coverage)
    raise ValueError(f"Unknown algorithm: {algorithm}")


def plot_metric_bars(results: dict[str, Any], output: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for comparison plots.") from exc

    algorithms = [
        name for name, result in results["algorithms"].items()
        if "metrics" in result
    ]
    metrics = [
        ("coverage", "Coverage (%)", lambda value: value * 100.0),
        ("path_length_m", "Path length (m)", float),
        ("repeat_spray_ratio", "Repeat spray (%)", lambda value: value * 100.0),
        ("collisions", "Collisions", float),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes_flat = axes.flatten()
    for ax, (metric_key, title, transform) in zip(axes_flat, metrics):
        values = [
            transform(results["algorithms"][name]["metrics"][metric_key])
            for name in algorithms
        ]
        labels = [ALGORITHM_LABELS.get(name, name) for name in algorithms]
        colors = [COLORS.get(name, "#495057") for name in algorithms]
        ax.bar(labels, values, color=colors)
        ax.set_title(title)
        if metric_key in {"coverage", "repeat_spray_ratio"}:
            ax.set_ylim(0, 105)
        elif metric_key == "collisions":
            ax.set_ylim(0, max(1.0, max(values) + 1.0))
        ax.grid(axis="y", alpha=0.25)
        ax.tick_params(axis="x", rotation=15)
        for index, value in enumerate(values):
            text = f"{value:.1f}" if metric_key != "collisions" else f"{value:.0f}"
            ax.text(index, value, text, ha="center", va="bottom", fontsize=9)
    fig.suptitle("Spray path planning comparison on apple_orchard.sdf")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def plot_coverage_traces(results: dict[str, Any], output: Path) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise SystemExit("matplotlib is required for comparison plots.") from exc

    fig, ax = plt.subplots(figsize=(10, 5))
    for name, result in results["algorithms"].items():
        trace = result.get("coverage_trace")
        if not trace:
            continue
        xs = list(range(len(trace)))
        ys = [value * 100.0 for value in trace]
        ax.plot(xs, ys, label=ALGORITHM_LABELS.get(name, name), color=COLORS.get(name), linewidth=2)
    ax.set_title("Coverage over waypoints")
    ax.set_xlabel("Waypoint index")
    ax.set_ylabel("Coverage (%)")
    ax.set_ylim(0, 105)
    ax.grid(alpha=0.25)
    ax.legend()
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def plot_paths(grid: OrchardWorldGrid, results: dict[str, Any], output: Path) -> None:
    try:
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError as exc:
        raise SystemExit("matplotlib and numpy are required for comparison plots.") from exc

    algorithms = [
        name for name, result in results["algorithms"].items()
        if "path" in result
    ]
    if not algorithms:
        return
    cols = min(3, len(algorithms))
    rows = math.ceil(len(algorithms) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 5 * rows), squeeze=False)
    cmap = ListedColormap(["#f8f9fa", "#8fd694", "#2f9e44", "#495057"])

    for ax, algorithm in zip(axes.flatten(), algorithms):
        path = [tuple(cell) for cell in results["algorithms"][algorithm]["path"]]
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
            ys = [row for row, _ in path]
            xs = [col for _, col in path]
            ax.plot(xs, ys, color=COLORS.get(algorithm, "#212529"), linewidth=1.5, marker="o", markersize=2)
            ax.scatter([xs[0]], [ys[0]], color="#212529", s=45)
            ax.scatter([xs[-1]], [ys[-1]], color="#f08c00", s=45)
        metrics = results["algorithms"][algorithm]["metrics"]
        ax.set_title(
            f"{ALGORITHM_LABELS.get(algorithm, algorithm)}: "
            f"cov={metrics['coverage']:.1%}, "
            f"len={metrics['path_length_m']:.0f}m, "
            f"coll={metrics['collisions']}"
        )
        ax.set_xticks([])
        ax.set_yticks([])
        ax.grid(color="white", linewidth=0.4)

    for ax in axes.flatten()[len(algorithms):]:
        ax.axis("off")
    fig.suptitle("Path overlays on the SDF-derived orchard grid")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare spray planning algorithms on the original apple_orchard map.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--model", default=str(default_output_dir() / "models" / "dqn_apple_orchard.zip"))
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--algorithms", default="orchard-row,nearest-target,dqn")
    parser.add_argument("--output-dir", default=str(default_output_dir() / "comparison"))
    args = parser.parse_args()

    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size)
    output_dir = Path(args.output_dir)
    algorithms = [item.strip() for item in args.algorithms.split(",") if item.strip()]
    results: dict[str, Any] = {
        "world_summary": grid.active_target_summary(),
        "algorithms": {},
    }

    for algorithm in algorithms:
        path = get_algorithm_path(algorithm, grid, args.world, args.model, args.cell_size, args.goal_coverage)
        if path is None:
            results["algorithms"][algorithm] = {"skipped": f"model not found: {args.model}"}
            continue
        results["algorithms"][algorithm] = {
            "metrics": evaluate_path(grid, path),
            "coverage_trace": coverage_trace(grid, path),
            "path": path,
        }

    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "comparison.json"
    with json_path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(results, handle, indent=2)

    plot_metric_bars(results, output_dir / "comparison_metrics.png")
    plot_coverage_traces(results, output_dir / "coverage_over_waypoints.png")
    plot_paths(grid, results, output_dir / "path_overlays.png")
    print(f"Wrote {json_path}")
    print(f"Wrote {output_dir / 'comparison_metrics.png'}")
    print(f"Wrote {output_dir / 'coverage_over_waypoints.png'}")
    print(f"Wrote {output_dir / 'path_overlays.png'}")


if __name__ == "__main__":
    main()
