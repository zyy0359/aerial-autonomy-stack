from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD, default_output_dir


def load_summary(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def plot_result(summary: dict[str, Any], output: Path, show: bool = False) -> None:
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D

    env = OrchardDQNEnv(
        world_path=summary.get("world", DEFAULT_WORLD),
        cell_size_m=float(summary.get("cell_size_m", 5.0)),
        goal_coverage=float(summary.get("goal_coverage", 0.97)),
        dynamic_obstacle_count=int(summary.get("dynamic_obstacles", 0)),
        dynamic_obstacle_seed=int(summary.get("seed", 0)),
        dynamic_obstacle_mode=str(summary.get("dynamic_obstacle_mode", "random")),
        intelligent_irrigation=bool(summary.get("intelligent_irrigation", False)),
        irrigation_seed=int(summary.get("seed", 0)),
        goal_metric=summary.get("goal_metric"),
        spray_control=bool(summary.get("spray_control", False)),
        auto_spray_control=bool(summary.get("auto_spray_control", False)),
        safety_controller=bool(summary.get("safety_controller", False)),
    )
    path = [tuple(cell) for cell in summary["path"]]
    rows = [cell[0] for cell in path]
    cols = [cell[1] for cell in path]

    demand = np.where(env.target_mask, env.demand_map, np.nan)
    fig, ax = plt.subplots(figsize=(9, 9))
    heatmap = ax.imshow(demand, cmap="YlGnBu", interpolation="nearest")
    fig.colorbar(heatmap, ax=ax, shrink=0.72, label="Demand")

    obstacle_rows, obstacle_cols = np.where(env.obstacle_mask)
    ax.scatter(obstacle_cols, obstacle_rows, marker="s", s=28, color="#212529", label="Static obstacle")

    target_rows, target_cols = np.where(env.target_mask)
    ax.scatter(target_cols, target_rows, marker="o", s=42, facecolors="none", edgecolors="#2f9e44", label="Target")

    for moving_path in env.dynamic_paths:
        moving_rows = [cell[0] for cell in moving_path.cells]
        moving_cols = [cell[1] for cell in moving_path.cells]
        ax.plot(moving_cols, moving_rows, "--", color="#e03131", linewidth=1.5, alpha=0.75)
        ax.scatter(moving_cols, moving_rows, marker="x", color="#e03131", s=28)

    ax.plot(cols, rows, color="#1864ab", linewidth=2.0, label="UAV path")
    ax.scatter(cols[0], rows[0], marker="*", s=160, color="#f08c00", label="Start")
    ax.scatter(cols[-1], rows[-1], marker="P", s=120, color="#7048e8", label="End")
    ax.set_title(
        f"{summary.get('algorithm', 'algorithm')} seed={summary.get('seed')} "
        f"goal={summary.get('final_metrics', {}).get('goal_progress', 0.0):.1%}"
    )
    ax.set_xlabel("Grid column")
    ax.set_ylabel("Grid row")
    ax.set_xlim(-0.5, env.grid.cols - 0.5)
    ax.set_ylim(env.grid.rows - 0.5, -0.5)
    ax.grid(color="white", alpha=0.35, linewidth=0.8)
    dynamic_handle = Line2D([0], [0], color="#e03131", linestyle="--", label="Dynamic obstacle route")
    handles, labels = ax.get_legend_handles_labels()
    if env.dynamic_paths:
        handles.append(dynamic_handle)
    ax.legend(handles=handles, loc="upper right")
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    if show:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot an enhanced dynamic/irrigation spray result.")
    parser.add_argument("--summary", required=True, help="Path to an algorithm *_summary.json file.")
    parser.add_argument("--output", default=str(default_output_dir() / "enhanced_result.png"))
    parser.add_argument("--show", action="store_true", help="Show an interactive matplotlib window if a display is available.")
    args = parser.parse_args()
    plot_result(load_summary(Path(args.summary)), Path(args.output), show=args.show)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()
