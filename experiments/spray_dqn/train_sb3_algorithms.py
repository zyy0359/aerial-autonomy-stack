from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from orchard_dqn_env import OrchardDQNEnv
from orchard_rl_envs import OrchardContinuousActionEnv
from orchard_world import DEFAULT_WORLD, default_output_dir, evaluate_path


DISCRETE_ALGORITHMS = {"dqn", "ppo", "a2c", "maskable-ppo"}
CONTINUOUS_ALGORITHMS = {"sac", "td3"}


def safe_name(name: str) -> str:
    return name.lower().replace("_", "-")


def make_env(
    algorithm: str,
    world: str,
    cell_size: float,
    goal_coverage: float,
):
    if algorithm in CONTINUOUS_ALGORITHMS:
        return OrchardContinuousActionEnv(
            world_path=world,
            cell_size_m=cell_size,
            goal_coverage=goal_coverage,
        )
    return OrchardDQNEnv(
        world_path=world,
        cell_size_m=cell_size,
        goal_coverage=goal_coverage,
    )


def make_model(
    algorithm: str,
    env,
    seed: int,
    device: str,
):
    try:
        from stable_baselines3 import A2C, DQN, PPO, SAC, TD3
        from stable_baselines3.common.noise import NormalActionNoise
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required for SB3 baseline training.") from exc

    if algorithm == "dqn":
        return DQN(
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
            seed=seed,
            device=device,
        )
    if algorithm == "ppo":
        return PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=128,
            batch_size=64,
            gamma=0.98,
            gae_lambda=0.95,
            ent_coef=0.01,
            verbose=1,
            seed=seed,
            device=device,
        )
    if algorithm == "a2c":
        return A2C(
            "MlpPolicy",
            env,
            learning_rate=7e-4,
            n_steps=32,
            gamma=0.98,
            ent_coef=0.01,
            verbose=1,
            seed=seed,
            device=device,
        )
    if algorithm == "sac":
        return SAC(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=50000,
            learning_starts=200,
            batch_size=64,
            gamma=0.98,
            train_freq=4,
            gradient_steps=1,
            verbose=1,
            seed=seed,
            device=device,
        )
    if algorithm == "td3":
        import numpy as np

        action_noise = NormalActionNoise(mean=np.zeros(2), sigma=0.2 * np.ones(2))
        return TD3(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            buffer_size=50000,
            learning_starts=200,
            batch_size=64,
            gamma=0.98,
            train_freq=4,
            gradient_steps=1,
            action_noise=action_noise,
            verbose=1,
            seed=seed,
            device=device,
        )
    if algorithm == "maskable-ppo":
        try:
            from sb3_contrib import MaskablePPO
        except ImportError as exc:
            raise RuntimeError("sb3-contrib is required for Maskable PPO.") from exc
        return MaskablePPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=128,
            batch_size=64,
            gamma=0.98,
            gae_lambda=0.95,
            ent_coef=0.01,
            verbose=1,
            seed=seed,
            device=device,
        )
    raise ValueError(f"Unknown SB3 algorithm: {algorithm}")


def evaluate_model(model, algorithm: str, world: str, cell_size: float, goal_coverage: float, seed: int) -> dict[str, Any]:
    env = make_env(algorithm, world, cell_size, goal_coverage)
    obs, _ = env.reset(seed=seed)
    terminated = False
    truncated = False
    while not (terminated or truncated):
        if algorithm == "maskable-ppo":
            action, _ = model.predict(obs, deterministic=True, action_masks=env.action_masks())
        else:
            action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(action)
    return {
        "metrics": evaluate_path(env.grid, env.path),
        "path": env.path,
    }


def write_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(payload, handle, indent=2)


def train_one(args, algorithm: str) -> dict[str, Any]:
    try:
        from stable_baselines3.common.monitor import Monitor
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required for SB3 baseline training.") from exc

    algorithm = safe_name(algorithm)
    if algorithm not in DISCRETE_ALGORITHMS | CONTINUOUS_ALGORITHMS:
        raise ValueError(f"Unsupported SB3 algorithm: {algorithm}")

    model_dir = Path(args.model_dir)
    metrics_dir = Path(args.metrics_dir)
    model_path = model_dir / algorithm
    summary_path = metrics_dir / f"{algorithm}_summary.json"

    try:
        env = Monitor(make_env(algorithm, args.world, args.cell_size, args.goal_coverage))
        model = make_model(algorithm, env, args.seed, args.device)
        model.learn(total_timesteps=args.timesteps)
        model_dir.mkdir(parents=True, exist_ok=True)
        model.save(model_path)
        eval_result = evaluate_model(model, algorithm, args.world, args.cell_size, args.goal_coverage, args.seed)
        summary = {
            "algorithm": algorithm,
            "family": "sb3",
            "implementation": implementation_note(algorithm),
            "world": args.world,
            "cell_size_m": args.cell_size,
            "timesteps": args.timesteps,
            "seed": args.seed,
            "goal_coverage": args.goal_coverage,
            "model": str(model_path.with_suffix(".zip")),
            "final_metrics": eval_result["metrics"],
            "path": eval_result["path"],
        }
    except RuntimeError as exc:
        summary = {
            "algorithm": algorithm,
            "family": "sb3",
            "implementation": implementation_note(algorithm),
            "skipped": str(exc),
        }

    write_summary(summary_path, summary)
    return summary


def implementation_note(algorithm: str) -> str:
    notes = {
        "dqn": "Stable-Baselines3 DQN on the discrete orchard grid.",
        "ppo": "Stable-Baselines3 PPO on the discrete orchard grid.",
        "a2c": "Stable-Baselines3 A2C on the discrete orchard grid.",
        "sac": "Stable-Baselines3 SAC on a 2D continuous-action wrapper mapped to grid moves.",
        "td3": "Stable-Baselines3 TD3 on a 2D continuous-action wrapper mapped to grid moves.",
        "maskable-ppo": "sb3-contrib MaskablePPO using OrchardDQNEnv.action_masks().",
    }
    return notes.get(algorithm, algorithm)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train SB3 RL baselines on the original apple_orchard-derived task.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--timesteps", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--algorithms", default="ppo,a2c,sac,td3,maskable-ppo")
    parser.add_argument("--model-dir", default=str(default_output_dir() / "models"))
    parser.add_argument("--metrics-dir", default=str(default_output_dir() / "metrics"))
    args = parser.parse_args()

    summaries = []
    for algorithm in [item.strip() for item in args.algorithms.split(",") if item.strip()]:
        print(f"=== Training {algorithm} ===")
        summaries.append(train_one(args, algorithm))
    print(json.dumps({summary["algorithm"]: summary.get("final_metrics", summary.get("skipped")) for summary in summaries}, indent=2))


if __name__ == "__main__":
    main()
