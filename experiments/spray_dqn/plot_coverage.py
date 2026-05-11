from __future__ import annotations

import argparse
from pathlib import Path

from orchard_world import (
    DEFAULT_WORLD,
    OrchardWorldGrid,
    default_output_dir,
    evaluate_path,
    nearest_target_path,
    orchard_row_path,
)


def get_path(
    policy: str,
    grid: OrchardWorldGrid,
    world: str,
    model_path: str,
    cell_size: float,
    goal_coverage: float,
) -> list[tuple[int, int]]:
    if policy == "orchard-row":
        return orchard_row_path(grid)
    if policy == "nearest-target":
        return nearest_target_path(grid)
    if policy == "dqn":
        try:
            from stable_baselines3 import DQN
        except ImportError as exc:
            raise SystemExit("stable-baselines3 is required for --policy dqn.") from exc
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
    raise ValueError(f"Unknown policy: {policy}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot a spray path over entities parsed from apple_orchard.sdf.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--policy", choices=["orchard-row", "nearest-target", "dqn"], default="orchard-row")
    parser.add_argument("--model", default=str(default_output_dir() / "models" / "dqn_apple_orchard.zip"))
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--output", default=None)
    args = parser.parse_args()

    try:
        import numpy as np
        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap
    except ImportError as exc:
        raise SystemExit("matplotlib is required for plotting.") from exc

    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size)
    path = get_path(args.policy, grid, args.world, args.model, args.cell_size, args.goal_coverage)
    sprayed: set[tuple[int, int]] = set()
    for cell in path:
        for spray_cell in grid.spray_cells(cell):
            if spray_cell in grid.target_cells:
                sprayed.add(spray_cell)
    metrics = evaluate_path(grid, path)

    layers = np.zeros((grid.rows, grid.cols), dtype=int)
    for cell in grid.target_cells:
        layers[cell] = 1
    for cell in sprayed & grid.target_cells:
        layers[cell] = 2
    for cell in grid.obstacle_cells:
        layers[cell] = 3
    cmap = ListedColormap(["#f8f9fa", "#8fd694", "#2f9e44", "#495057"])
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(layers, cmap=cmap, origin="upper")
    if path:
        ys = [row for row, _ in path]
        xs = [col for _, col in path]
        ax.plot(xs, ys, color="#e03131", linewidth=1.5, marker="o", markersize=2)
        ax.scatter([xs[0]], [ys[0]], color="#212529", s=50, label="start")
        ax.scatter([xs[-1]], [ys[-1]], color="#f08c00", s=50, label="end")
    ax.set_title(
        f"{args.policy} on apple_orchard.sdf: "
        f"coverage={metrics['coverage']:.1%}, collisions={metrics['collisions']}"
    )
    ax.grid(color="white", linewidth=0.5)
    ax.legend(loc="upper right")
    fig.tight_layout()

    output = Path(args.output) if args.output else default_output_dir() / "plots" / f"{args.policy}_orchard.png"
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    print(output)


if __name__ == "__main__":
    main()
