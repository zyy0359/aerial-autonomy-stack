from __future__ import annotations

import argparse
import json
from pathlib import Path

from learning_curve_utils import curve_row, write_learning_curve
from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD, default_output_dir, evaluate_path


def env_kwargs(args) -> dict:
    return {
        "world_path": args.world,
        "cell_size_m": args.cell_size,
        "max_steps": args.max_steps,
        "goal_coverage": args.goal_coverage,
        "dynamic_obstacle_count": args.dynamic_obstacles,
        "dynamic_obstacle_span": args.dynamic_obstacle_span,
        "dynamic_safety_radius_cells": args.dynamic_safety_radius,
        "dynamic_obstacle_seed": args.seed,
        "dynamic_obstacle_mode": args.dynamic_obstacle_mode,
        "intelligent_irrigation": args.intelligent_irrigation,
        "irrigation_seed": args.seed,
        "goal_metric": args.goal_metric,
        "spray_control": args.spray_control,
        "auto_spray_control": args.auto_spray_control,
        "safety_controller": args.safety_controller,
        "target_mode": args.target_mode,
        "field_bounds": args.field_bounds,
        "field_blocks": args.field_blocks,
        "field_spacing_m": args.field_spacing,
    }


def evaluate_model(model, args, seed: int) -> dict:
    eval_env = OrchardDQNEnv(**env_kwargs(args))
    obs, _ = eval_env.reset(seed=seed)
    terminated = False
    truncated = False
    episode_reward = 0.0
    episode_steps = 0
    info = {}
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = eval_env.step(int(action))
        episode_reward += float(reward)
        episode_steps += 1
    metrics = evaluate_path(eval_env.grid, eval_env.path)
    metrics.update(info)
    return {
        "metrics": metrics,
        "path": eval_env.path,
        "episode_reward": episode_reward,
        "episode_steps": episode_steps,
        "grid": eval_env.grid.active_target_summary(),
    }


class LearningCurveCallback:
    def __init__(self, args):
        from stable_baselines3.common.callbacks import BaseCallback

        class Callback(BaseCallback):
            def __init__(self, outer_args):
                super().__init__()
                self.outer_args = outer_args
                self.rows = []

            def _on_step(self) -> bool:
                if self.outer_args.eval_freq <= 0 or not self.outer_args.curve_out:
                    return True
                if self.num_timesteps % self.outer_args.eval_freq != 0:
                    return True
                self._append_row(self.num_timesteps)
                return True

            def _on_training_end(self) -> None:
                if not self.outer_args.curve_out:
                    return
                if self.outer_args.eval_freq > 0 and (
                    not self.rows or self.rows[-1]["timesteps"] != self.num_timesteps
                ):
                    self._append_row(self.num_timesteps)
                write_learning_curve(self.rows, self.outer_args.curve_out)

            def _append_row(self, timesteps: int) -> None:
                result = evaluate_model(self.model, self.outer_args, seed=self.outer_args.seed)
                self.rows.append(
                    curve_row(
                        timesteps=timesteps,
                        algorithm="dqn",
                        seed=self.outer_args.seed,
                        metrics=result["metrics"],
                        goal_metric=self.outer_args.goal_metric
                        or ("demand" if self.outer_args.intelligent_irrigation else "coverage"),
                        goal_coverage=self.outer_args.goal_coverage,
                        episode_reward=result["episode_reward"],
                        episode_steps=result["episode_steps"],
                    )
                )

        self.callback = Callback(args)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train DQN on a map derived from the existing apple_orchard SDF.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--max-steps", type=int, default=500)
    parser.add_argument("--timesteps", type=int, default=50000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--goal-metric", choices=["coverage", "demand"], default=None)
    parser.add_argument("--dynamic-obstacles", type=int, default=0)
    parser.add_argument("--dynamic-obstacle-span", type=int, default=4)
    parser.add_argument("--dynamic-safety-radius", type=int, default=3)
    parser.add_argument("--dynamic-obstacle-mode", choices=["random", "corridor"], default="random")
    parser.add_argument("--intelligent-irrigation", action="store_true")
    parser.add_argument("--spray-control", action="store_true")
    parser.add_argument("--auto-spray-control", action="store_true")
    parser.add_argument("--safety-controller", action="store_true")
    parser.add_argument("--target-mode", choices=["trees", "field", "blocks"], default="trees")
    parser.add_argument("--field-bounds", default=None, help="Field mode bounds: min_x,min_y,max_x,max_y")
    parser.add_argument("--field-blocks", default=None, help="Blocks mode rectangles: name:min_x,min_y,max_x,max_y;...")
    parser.add_argument("--field-spacing", type=float, default=None)
    parser.add_argument("--guided-exploration", type=float, default=0.0, help="Accepted for CLI compatibility; SB3 DQN does not use expert guided exploration.")
    parser.add_argument("--guided-exploration-fraction", type=float, default=0.5, help="Accepted for CLI compatibility.")
    parser.add_argument("--eval-freq", type=int, default=0, help="Evaluate every N training timesteps and save a learning curve when --curve-out is set.")
    parser.add_argument("--curve-out", default=None, help="Optional CSV/JSON learning curve output path.")
    parser.add_argument("--model-out", default=str(default_output_dir() / "models" / "dqn_apple_orchard"))
    parser.add_argument("--summary-out", default=str(default_output_dir() / "metrics" / "train_summary.json"))
    args = parser.parse_args()

    try:
        from stable_baselines3 import DQN
        from stable_baselines3.common.monitor import Monitor
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required for DQN training.") from exc

    env = Monitor(OrchardDQNEnv(**env_kwargs(args)))
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
    curve_callback = LearningCurveCallback(args).callback if args.curve_out else None
    model.learn(total_timesteps=args.timesteps, callback=curve_callback)

    model_out = Path(args.model_out)
    model_out.parent.mkdir(parents=True, exist_ok=True)
    model.save(model_out)

    eval_result = evaluate_model(model, args, seed=args.seed)
    info = eval_result["metrics"]

    summary = {
        "algorithm": "dqn",
        "family": "sb3",
        "implementation": "Stable-Baselines3 DQN on the discrete orchard grid.",
        "world": args.world,
        "model": str(model_out.with_suffix(".zip")),
        "cell_size_m": args.cell_size,
        "max_steps": args.max_steps,
        "timesteps": args.timesteps,
        "seed": args.seed,
        "goal_coverage": args.goal_coverage,
        "goal_metric": args.goal_metric or ("demand" if args.intelligent_irrigation else "coverage"),
        "dynamic_obstacles": args.dynamic_obstacles,
        "dynamic_obstacle_mode": args.dynamic_obstacle_mode,
        "intelligent_irrigation": args.intelligent_irrigation,
        "spray_control": args.spray_control,
        "auto_spray_control": args.auto_spray_control,
        "safety_controller": args.safety_controller,
        "target_mode": args.target_mode,
        "field_bounds": args.field_bounds,
        "field_blocks": args.field_blocks,
        "field_spacing_m": args.field_spacing,
        "guided_exploration": args.guided_exploration,
        "guided_exploration_fraction": args.guided_exploration_fraction,
        "grid": eval_result["grid"],
        "final_metrics": info,
        "path": eval_result["path"],
        "episode_reward": eval_result["episode_reward"],
    }
    summary_out = Path(args.summary_out)
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    with summary_out.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary["final_metrics"], indent=2))


if __name__ == "__main__":
    main()
