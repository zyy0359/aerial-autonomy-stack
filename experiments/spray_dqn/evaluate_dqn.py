from __future__ import annotations

import argparse
import json
from pathlib import Path

from orchard_world import DEFAULT_WORLD, OrchardWorldGrid, default_output_dir, evaluate_path, orchard_row_path


def dqn_path(world: str, model_path: str, cell_size: float) -> list[tuple[int, int]]:
    try:
        from stable_baselines3 import DQN
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required to evaluate DQN.") from exc
    from orchard_dqn_env import OrchardDQNEnv
    env = OrchardDQNEnv(world_path=world, cell_size_m=cell_size)
    model = DQN.load(model_path, env=env)
    obs, _ = env.reset()
    terminated = False
    truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(int(action))
    return env.path


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate paths on the original apple_orchard-derived map.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--model", default=str(default_output_dir() / "models" / "dqn_apple_orchard.zip"))
    parser.add_argument("--include-dqn", choices=["true", "false"], default="true")
    parser.add_argument("--output", default=str(default_output_dir() / "metrics" / "evaluation.json"))
    args = parser.parse_args()

    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size)
    results = {
        "world_summary": grid.active_target_summary(),
        "orchard_row": evaluate_path(grid, orchard_row_path(grid)),
    }
    model_path = Path(args.model)
    if args.include_dqn == "true" and model_path.exists():
        results["dqn"] = evaluate_path(grid, dqn_path(args.world, str(model_path), args.cell_size))
    elif args.include_dqn == "true":
        results["dqn"] = {"skipped": f"model not found: {model_path}"}

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(results, handle, indent=2)
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
