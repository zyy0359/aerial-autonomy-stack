from __future__ import annotations

import argparse
import json
from pathlib import Path

from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD, default_output_dir


def main() -> None:
    parser = argparse.ArgumentParser(description="Train DQN on a map derived from the existing apple_orchard SDF.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--timesteps", type=int, default=50000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--model-out", default=str(default_output_dir() / "models" / "dqn_apple_orchard"))
    parser.add_argument("--summary-out", default=str(default_output_dir() / "metrics" / "train_summary.json"))
    args = parser.parse_args()

    try:
        from stable_baselines3 import DQN
        from stable_baselines3.common.monitor import Monitor
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required for DQN training.") from exc

    env = Monitor(OrchardDQNEnv(world_path=args.world, cell_size_m=args.cell_size, goal_coverage=args.goal_coverage))
    model = DQN(
        "MlpPolicy",
        env,
        learning_rate=1e-3,
        buffer_size=50000,
        learning_starts=200,
        batch_size=64,
        gamma=0.98,
        train_freq=4,
        target_update_interval=1000,
        exploration_fraction=0.5,
        exploration_final_eps=0.05,
        verbose=1,
        seed=args.seed,
        device=args.device,
    )
    model.learn(total_timesteps=args.timesteps)

    model_out = Path(args.model_out)
    model_out.parent.mkdir(parents=True, exist_ok=True)
    model.save(model_out)

    eval_env = OrchardDQNEnv(world_path=args.world, cell_size_m=args.cell_size, goal_coverage=args.goal_coverage)
    obs, _ = eval_env.reset(seed=args.seed)
    terminated = False
    truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, info = eval_env.step(int(action))

    summary = {
        "algorithm": "dqn",
        "family": "sb3",
        "implementation": "Stable-Baselines3 DQN on the discrete orchard grid.",
        "world": args.world,
        "model": str(model_out.with_suffix(".zip")),
        "timesteps": args.timesteps,
        "seed": args.seed,
        "goal_coverage": args.goal_coverage,
        "grid": eval_env.grid.active_target_summary(),
        "final_metrics": info,
        "path": eval_env.path,
    }
    summary_out = Path(args.summary_out)
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    with summary_out.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary["final_metrics"], indent=2))


if __name__ == "__main__":
    main()
