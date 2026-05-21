from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD, OrchardWorldGrid, evaluate_path, nearest_target_path, orchard_row_path


LABELS = {
    "orchard-row": "Orchard row",
    "nearest-target": "Nearest target",
    "dqn": "DQN",
    "double-dqn": "Double DQN",
    "dueling-dqn": "Dueling DQN",
    "rainbow-dqn-lite": "Rainbow DQN lite",
    "drqn": "DRQN",
    "ppo": "PPO",
    "a2c": "A2C",
    "sac": "SAC",
    "td3": "TD3",
    "maskable-ppo": "Maskable PPO",
}

COLORS = {
    "orchard-row": "#1971c2",
    "nearest-target": "#2f9e44",
    "dqn": "#e03131",
    "double-dqn": "#1864ab",
    "dueling-dqn": "#2b8a3e",
    "rainbow-dqn-lite": "#e67700",
    "drqn": "#087f5b",
    "ppo": "#9c36b5",
    "a2c": "#f08c00",
    "sac": "#0b7285",
    "td3": "#5f3dc4",
    "maskable-ppo": "#c2255c",
}

BACKGROUND_COLORS = {
    "non_spray": "#c9ced6",
    "target": "#f8f9fa",
    "sprayed": "#111111",
    "obstacle": "#ff922b",
    "transit": "#ffe066",
}


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def normalize_algorithm(value: str) -> str:
    return value.lower().replace("_", "-")


def summary_score(payload: dict[str, Any]) -> tuple[float, float, float, float]:
    metrics = payload.get("final_metrics", {})
    progress = float(metrics.get("goal_progress", metrics.get("demand_satisfaction", metrics.get("coverage", 0.0))))
    collisions = float(metrics.get("collisions", 0.0)) + float(metrics.get("dynamic_collisions", 0.0))
    path_length = float(metrics.get("path_length_m", metrics.get("path_length", 1e9)))
    repeat = float(metrics.get("repeat_spray_ratio", metrics.get("over_spray_ratio", 1.0)))
    return progress, -collisions, -path_length, -repeat


def collect_best_summaries(input_dir: Path, algorithms: list[str] | None) -> dict[str, dict[str, Any]]:
    wanted = {normalize_algorithm(item) for item in algorithms} if algorithms else None
    best: dict[str, dict[str, Any]] = {}
    for path in sorted(input_dir.rglob("*_summary.json")):
        payload = load_json(path)
        if "skipped" in payload or "path" not in payload:
            continue
        algorithm = normalize_algorithm(str(payload.get("algorithm", path.stem.replace("_summary", ""))))
        if wanted is not None and algorithm not in wanted:
            continue
        payload["_summary_path"] = str(path)
        if algorithm not in best or summary_score(payload) > summary_score(best[algorithm]):
            best[algorithm] = payload
    return best


def env_from_payload(payload: dict[str, Any]) -> OrchardDQNEnv:
    return OrchardDQNEnv(
        world_path=payload.get("world", DEFAULT_WORLD),
        cell_size_m=float(payload.get("cell_size_m", 5.0)),
        goal_coverage=float(payload.get("goal_coverage", 0.97)),
        dynamic_obstacle_count=int(payload.get("dynamic_obstacles", 0)),
        dynamic_obstacle_seed=int(payload.get("seed", 0)),
        dynamic_obstacle_mode=str(payload.get("dynamic_obstacle_mode", "random")),
        intelligent_irrigation=bool(payload.get("intelligent_irrigation", False)),
        irrigation_seed=int(payload.get("seed", 0)),
        goal_metric=payload.get("goal_metric"),
        spray_control=bool(payload.get("spray_control", False)),
        auto_spray_control=bool(payload.get("auto_spray_control", False)),
        safety_controller=bool(payload.get("safety_controller", False)),
        target_mode=str(payload.get("target_mode", "trees")),
        field_bounds=payload.get("field_bounds"),
        field_blocks=payload.get("field_blocks"),
        field_spacing_m=payload.get("field_spacing_m"),
    )


def grid_from_args(args: argparse.Namespace) -> OrchardWorldGrid:
    return OrchardWorldGrid(
        world_path=args.world,
        cell_size_m=args.cell_size,
        target_mode=args.target_mode,
        field_bounds=args.field_bounds,
        field_blocks=args.field_blocks,
        field_spacing_m=args.field_spacing,
    )


def add_rule_baselines(results: dict[str, dict[str, Any]], args: argparse.Namespace) -> None:
    grid = grid_from_args(args)
    for name, path in {
        "orchard-row": orchard_row_path(grid),
        "nearest-target": nearest_target_path(grid),
    }.items():
        if args.algorithms and name not in args.algorithms:
            continue
        results[name] = {
            "algorithm": name,
            "world": args.world,
            "cell_size_m": args.cell_size,
            "target_mode": args.target_mode,
            "field_bounds": args.field_bounds,
            "field_blocks": args.field_blocks,
            "field_spacing_m": args.field_spacing,
            "final_metrics": evaluate_path(grid, path),
            "path": [list(cell) for cell in path],
            "_summary_path": "computed baseline",
        }


def path_cells(payload: dict[str, Any]) -> list[tuple[int, int]]:
    return [tuple(cell) for cell in payload.get("path", [])]


def action_counts(path: list[tuple[int, int]]) -> dict[str, int]:
    counts = {"up": 0, "down": 0, "left": 0, "right": 0, "stay_or_other": 0}
    for current, nxt in zip(path, path[1:]):
        dr = nxt[0] - current[0]
        dc = nxt[1] - current[1]
        if dr < 0 and dc == 0:
            counts["up"] += abs(dr)
        elif dr > 0 and dc == 0:
            counts["down"] += abs(dr)
        elif dc < 0 and dr == 0:
            counts["left"] += abs(dc)
        elif dc > 0 and dr == 0:
            counts["right"] += abs(dc)
        else:
            counts["stay_or_other"] += 1
    return counts


def sprayed_cells(env: OrchardDQNEnv, path: list[tuple[int, int]]) -> set[tuple[int, int]]:
    sprayed: set[tuple[int, int]] = set()
    for cell in path:
        if env.grid.in_bounds(cell) and cell not in env.grid.obstacle_cells:
            for spray_cell in env.grid.spray_cells(cell):
                if spray_cell in env.grid.target_cells:
                    sprayed.add(spray_cell)
    return sprayed


def off_target_ratio(env: OrchardDQNEnv, path: list[tuple[int, int]]) -> float:
    if not path:
        return 0.0
    off = sum(1 for cell in path if cell not in env.grid.target_cells)
    return off / len(path)


def write_tables(rows: list[dict[str, Any]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "action_matrix_summary.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    md_path = output_dir / "action_matrix_summary.md"
    with md_path.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("| Algorithm | Seed | Coverage (%) | Path length (m) | Up | Down | Left | Right | Off-target steps (%) |\n")
        handle.write("|---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
        for row in rows:
            handle.write(
                f"| {row['algorithm']} | {row['seed']} | {row['coverage_percent']:.1f} | "
                f"{row['path_length_m']:.1f} | {row['up']} | {row['down']} | {row['left']} | "
                f"{row['right']} | {row['off_target_percent']:.1f} |\n"
            )


def result_table_row(name: str, payload: dict[str, Any], env: OrchardDQNEnv, path: list[tuple[int, int]]) -> dict[str, Any]:
    counts = action_counts(path)
    metrics = payload.get("final_metrics", {})
    coverage = float(metrics.get("coverage", 0.0)) * 100.0
    path_length = float(metrics.get("path_length_m", metrics.get("path_length", 0.0)))
    off_ratio = off_target_ratio(env, path) * 100.0
    return {
        "algorithm": LABELS.get(name, name),
        "seed": payload.get("seed", "-"),
        "coverage_percent": coverage,
        "path_length_m": path_length,
        "up": counts["up"],
        "down": counts["down"],
        "left": counts["left"],
        "right": counts["right"],
        "stay_or_other": counts["stay_or_other"],
        "off_target_percent": off_ratio,
        "source": payload.get("_summary_path", ""),
    }


def pad_to_square(layers: np.ndarray, pad_value: int = 0) -> tuple[np.ndarray, int, int]:
    rows, cols = layers.shape
    side = max(rows, cols)
    row_offset = (side - rows) // 2
    col_offset = (side - cols) // 2
    square = np.full((side, side), pad_value, dtype=layers.dtype)
    square[row_offset:row_offset + rows, col_offset:col_offset + cols] = layers
    return square, row_offset, col_offset


def plot_square_overlay(results: dict[str, dict[str, Any]], output_dir: Path, max_arrows: int) -> list[dict[str, Any]]:
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    ordered = sorted(results.items(), key=lambda item: summary_score(item[1]), reverse=True)
    first_env = env_from_payload(ordered[0][1])
    layers = np.zeros((first_env.grid.rows, first_env.grid.cols), dtype=int)
    aggregate_sprayed: set[tuple[int, int]] = set()
    for _, payload in ordered:
        env = env_from_payload(payload)
        aggregate_sprayed.update(sprayed_cells(env, path_cells(payload)))
    for cell in first_env.grid.target_cells:
        layers[cell] = 1
    for cell in aggregate_sprayed & first_env.grid.target_cells:
        layers[cell] = 2
    for cell in first_env.grid.obstacle_cells:
        layers[cell] = 3

    square, row_offset, col_offset = pad_to_square(layers)
    cmap = ListedColormap(
        [
            BACKGROUND_COLORS["non_spray"],
            BACKGROUND_COLORS["target"],
            BACKGROUND_COLORS["sprayed"],
            BACKGROUND_COLORS["obstacle"],
        ]
    )
    fig, ax = plt.subplots(figsize=(9.5, 9.5))
    ax.imshow(square, cmap=cmap, origin="upper", interpolation="nearest", vmin=0, vmax=3)
    side = square.shape[0]
    ax.set_xticks(np.arange(-0.5, side, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, side, 1), minor=True)
    ax.grid(which="minor", color="#868e96", linewidth=0.28, alpha=0.55)
    ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    ax.set_aspect("equal")

    table_rows: list[dict[str, Any]] = []
    for index, (name, payload) in enumerate(ordered):
        env = env_from_payload(payload)
        path = path_cells(payload)
        table_rows.append(result_table_row(name, payload, env, path))
        if len(path) < 2:
            continue
        color = COLORS.get(name, f"C{index}")
        y = np.array([cell[0] + row_offset for cell in path], dtype=float)
        x = np.array([cell[1] + col_offset for cell in path], dtype=float)
        ax.plot(x, y, color=color, linewidth=1.8, alpha=0.95, label=LABELS.get(name, name))
        stride = max(1, len(path) // max_arrows)
        starts = path[:-1:stride]
        ends = path[1::stride]
        qx = np.array([cell[1] + col_offset for cell in starts], dtype=float)
        qy = np.array([cell[0] + row_offset for cell in starts], dtype=float)
        qu = np.array([nxt[1] - cur[1] for cur, nxt in zip(starts, ends)], dtype=float)
        qv = np.array([nxt[0] - cur[0] for cur, nxt in zip(starts, ends)], dtype=float)
        ax.quiver(qx, qy, qu, qv, angles="xy", scale_units="xy", scale=1, color=color, width=0.0035, alpha=0.9)
        ax.scatter([x[0]], [y[0]], marker="*", s=90, color="#212529", zorder=5)
        ax.scatter([x[-1]], [y[-1]], marker="P", s=80, color=color, edgecolors="#212529", linewidths=0.4, zorder=5)

    legend_items = [
        Patch(facecolor=BACKGROUND_COLORS["non_spray"], label="Non-spray / road / other"),
        Patch(facecolor=BACKGROUND_COLORS["target"], label="Target field cell"),
        Patch(facecolor=BACKGROUND_COLORS["sprayed"], label="Sprayed target cell"),
        Patch(facecolor=BACKGROUND_COLORS["obstacle"], label="Static obstacle"),
        Line2D([0], [0], color="#212529", marker="*", linestyle="", label="Start"),
    ]
    path_items = [
        Line2D([0], [0], color=COLORS.get(name, f"C{index}"), linewidth=2.0, label=LABELS.get(name, name))
        for index, (name, _) in enumerate(ordered)
    ]
    ax.legend(handles=legend_items + path_items, loc="lower right", fontsize=8, framealpha=0.88)
    ax.set_title("Square UAV Action Matrix: Multi-Algorithm Coverage Paths", fontsize=14)
    output_dir.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(output_dir / "action_matrix_square_overlay.png", dpi=220)
    return table_rows


def plot_matrix(results: dict[str, dict[str, Any]], output_dir: Path, max_arrows: int) -> list[dict[str, Any]]:
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch

    ordered = sorted(results.items(), key=lambda item: summary_score(item[1]), reverse=True)
    cols = min(3, max(1, len(ordered)))
    rows_n = math.ceil(len(ordered) / cols)
    fig, axes = plt.subplots(rows_n, cols, figsize=(6.2 * cols, 6.2 * rows_n), squeeze=False)
    cmap = ListedColormap(
        [
            BACKGROUND_COLORS["non_spray"],
            BACKGROUND_COLORS["target"],
            BACKGROUND_COLORS["sprayed"],
            BACKGROUND_COLORS["obstacle"],
            BACKGROUND_COLORS["transit"],
        ]
    )
    table_rows: list[dict[str, Any]] = []

    for ax, (name, payload) in zip(axes.flatten(), ordered):
        env = env_from_payload(payload)
        path = path_cells(payload)
        sprayed = sprayed_cells(env, path)
        layers = np.zeros((env.grid.rows, env.grid.cols), dtype=int)
        for cell in env.grid.target_cells:
            layers[cell] = 1
        for cell in sprayed:
            layers[cell] = 2
        for cell in env.grid.obstacle_cells:
            layers[cell] = 3
        for cell in path:
            if env.grid.in_bounds(cell) and cell not in env.grid.target_cells and cell not in env.grid.obstacle_cells:
                layers[cell] = 4

        ax.imshow(layers, cmap=cmap, origin="upper", interpolation="nearest", vmin=0, vmax=4)
        ax.set_xticks(np.arange(-0.5, env.grid.cols, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, env.grid.rows, 1), minor=True)
        ax.grid(which="minor", color="#868e96", linewidth=0.25, alpha=0.45)
        ax.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)

        if len(path) >= 2:
            y = np.array([cell[0] for cell in path], dtype=float)
            x = np.array([cell[1] for cell in path], dtype=float)
            color = COLORS.get(name, "#212529")
            ax.plot(x, y, color=color, linewidth=1.3, alpha=0.9)
            stride = max(1, len(path) // max_arrows)
            starts = path[:-1:stride]
            ends = path[1::stride]
            qx = np.array([cell[1] for cell in starts], dtype=float)
            qy = np.array([cell[0] for cell in starts], dtype=float)
            qu = np.array([nxt[1] - cur[1] for cur, nxt in zip(starts, ends)], dtype=float)
            qv = np.array([nxt[0] - cur[0] for cur, nxt in zip(starts, ends)], dtype=float)
            ax.quiver(qx, qy, qu, qv, angles="xy", scale_units="xy", scale=1, color=color, width=0.004, alpha=0.95)
            ax.scatter([x[0]], [y[0]], marker="*", s=110, color="#212529", zorder=4)
            ax.scatter([x[-1]], [y[-1]], marker="P", s=90, color="#f08c00", zorder=4)

        table_row = result_table_row(name, payload, env, path)
        coverage = table_row["coverage_percent"]
        path_length = table_row["path_length_m"]
        seed = payload.get("seed", "-")
        ax.set_title(
            f"{LABELS.get(name, name)} seed={seed}\n"
            f"cov={coverage:.1f}% len={path_length:.0f}m | U/D/L/R="
            f"{table_row['up']}/{table_row['down']}/{table_row['left']}/{table_row['right']}"
        )
        table_rows.append(table_row)

    for ax in axes.flatten()[len(ordered):]:
        ax.axis("off")

    legend_items = [
        Patch(facecolor=BACKGROUND_COLORS["non_spray"], label="Non-spray / road / other"),
        Patch(facecolor=BACKGROUND_COLORS["target"], label="Target field cell"),
        Patch(facecolor=BACKGROUND_COLORS["sprayed"], label="Sprayed target cell"),
        Patch(facecolor=BACKGROUND_COLORS["obstacle"], label="Static obstacle"),
        Patch(facecolor=BACKGROUND_COLORS["transit"], label="Transit outside target"),
        Line2D([0], [0], color="#212529", marker="*", linestyle="", label="Start"),
        Line2D([0], [0], color="#f08c00", marker="P", linestyle="", label="End"),
    ]
    fig.legend(handles=legend_items, loc="lower center", ncol=4, frameon=False)
    fig.suptitle("UAV action matrix: grid coverage and up/down/left/right movement", fontsize=15)
    fig.tight_layout(rect=(0, 0.055, 1, 0.96))
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / "action_matrix_paths.png"
    fig.savefig(output, dpi=180)
    return table_rows


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot grid action matrices for RL spray path summaries.")
    parser.add_argument("--input-dir", default="experiments/spray_dqn/outputs/hierarchical_extra_rl_3seeds_10k")
    parser.add_argument("--output-dir", default="experiments/spray_dqn/outputs/action_matrix")
    parser.add_argument("--algorithms", default=None, help="Comma-separated algorithms. Empty means all found.")
    parser.add_argument("--include-baselines", action="store_true")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--target-mode", choices=["trees", "field", "blocks"], default="trees")
    parser.add_argument("--field-bounds", default=None)
    parser.add_argument("--field-blocks", default=None)
    parser.add_argument("--field-spacing", type=float, default=None)
    parser.add_argument("--max-arrows", type=int, default=180)
    parser.add_argument("--style", choices=["panels", "square-overlay", "both"], default="panels")
    args = parser.parse_args()

    algorithms = [normalize_algorithm(item.strip()) for item in args.algorithms.split(",") if item.strip()] if args.algorithms else None
    results = collect_best_summaries(Path(args.input_dir), algorithms)
    args.algorithms = algorithms
    if args.include_baselines:
        add_rule_baselines(results, args)
    if not results:
        raise SystemExit(f"No summary paths found in {args.input_dir}.")
    output_dir = Path(args.output_dir)
    table_rows: list[dict[str, Any]] = []
    if args.style in {"panels", "both"}:
        table_rows = plot_matrix(results, output_dir=output_dir, max_arrows=max(20, args.max_arrows))
        print(f"Wrote {output_dir / 'action_matrix_paths.png'}")
    if args.style in {"square-overlay", "both"}:
        table_rows = plot_square_overlay(results, output_dir=output_dir, max_arrows=max(20, args.max_arrows))
        print(f"Wrote {output_dir / 'action_matrix_square_overlay.png'}")
    write_tables(table_rows, output_dir)
    print(f"Wrote {output_dir / 'action_matrix_summary.csv'}")
    print(f"Wrote {output_dir / 'action_matrix_summary.md'}")


if __name__ == "__main__":
    main()
