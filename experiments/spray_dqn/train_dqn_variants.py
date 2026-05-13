from __future__ import annotations

import argparse
import json
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD, default_output_dir, evaluate_path


@dataclass(frozen=True)
class VariantConfig:
    name: str
    double_target: bool
    dueling: bool
    prioritized_replay: bool
    implementation: str


VARIANTS = {
    "double-dqn": VariantConfig(
        name="double-dqn",
        double_target=True,
        dueling=False,
        prioritized_replay=False,
        implementation="Custom PyTorch Double DQN with online action selection and target-network evaluation.",
    ),
    "dueling-dqn": VariantConfig(
        name="dueling-dqn",
        double_target=True,
        dueling=True,
        prioritized_replay=False,
        implementation="Custom PyTorch Dueling Double DQN.",
    ),
    "rainbow-dqn-lite": VariantConfig(
        name="rainbow-dqn-lite",
        double_target=True,
        dueling=True,
        prioritized_replay=True,
        implementation=(
            "Rainbow-style DQN lite: Double DQN + Dueling network + prioritized replay. "
            "Distributional C51 and NoisyNet exploration are not included."
        ),
    ),
}


def env_kwargs(args) -> dict[str, Any]:
    return {
        "world_path": args.world,
        "cell_size_m": args.cell_size,
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
    }


class QNetwork:
    def __init__(self, obs_dim: int, action_dim: int, dueling: bool, device: str):
        import torch
        from torch import nn

        class Net(nn.Module):
            def __init__(self):
                super().__init__()
                self.feature = nn.Sequential(
                    nn.Linear(obs_dim, 128),
                    nn.ReLU(),
                    nn.Linear(128, 128),
                    nn.ReLU(),
                )
                self.dueling = dueling
                if dueling:
                    self.value = nn.Linear(128, 1)
                    self.advantage = nn.Linear(128, action_dim)
                else:
                    self.head = nn.Linear(128, action_dim)

            def forward(self, obs):
                features = self.feature(obs)
                if not self.dueling:
                    return self.head(features)
                value = self.value(features)
                advantage = self.advantage(features)
                return value + advantage - advantage.mean(dim=1, keepdim=True)

        self.model = Net().to(device)
        self.device = torch.device(device)

    def __call__(self, obs):
        return self.model(obs)

    def state_dict(self):
        return self.model.state_dict()

    def load_state_dict(self, state_dict):
        return self.model.load_state_dict(state_dict)

    def parameters(self):
        return self.model.parameters()

    def train(self):
        self.model.train()

    def eval(self):
        self.model.eval()


class ReplayBuffer:
    def __init__(self, capacity: int, obs_dim: int, prioritized: bool):
        self.capacity = int(capacity)
        self.prioritized = prioritized
        self.obs = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.next_obs = np.zeros((capacity, obs_dim), dtype=np.float32)
        self.actions = np.zeros(capacity, dtype=np.int64)
        self.rewards = np.zeros(capacity, dtype=np.float32)
        self.dones = np.zeros(capacity, dtype=np.float32)
        self.priorities = np.ones(capacity, dtype=np.float32)
        self.size = 0
        self.pos = 0

    def add(self, obs, action: int, reward: float, next_obs, done: bool) -> None:
        self.obs[self.pos] = obs
        self.actions[self.pos] = int(action)
        self.rewards[self.pos] = float(reward)
        self.next_obs[self.pos] = next_obs
        self.dones[self.pos] = float(done)
        max_priority = self.priorities[: self.size].max(initial=1.0)
        self.priorities[self.pos] = max_priority
        self.pos = (self.pos + 1) % self.capacity
        self.size = min(self.size + 1, self.capacity)

    def sample(self, batch_size: int, beta: float):
        if self.prioritized:
            scaled = self.priorities[: self.size] ** 0.6
            probs = scaled / scaled.sum()
            indexes = np.random.choice(self.size, size=batch_size, p=probs)
            weights = (self.size * probs[indexes]) ** (-beta)
            weights = weights / weights.max()
        else:
            indexes = np.random.randint(0, self.size, size=batch_size)
            weights = np.ones(batch_size, dtype=np.float32)
        return indexes, weights.astype(np.float32)

    def update_priorities(self, indexes, priorities) -> None:
        if not self.prioritized:
            return
        self.priorities[indexes] = np.asarray(priorities, dtype=np.float32) + 1e-5


def linear_schedule(start: float, end: float, progress: float) -> float:
    clipped = min(max(progress, 0.0), 1.0)
    return start + clipped * (end - start)


def evaluate_policy(q_net: QNetwork, env: OrchardDQNEnv, seed: int) -> dict[str, Any]:
    import torch

    q_net.eval()
    obs, _ = env.reset(seed=seed)
    terminated = False
    truncated = False
    info: dict[str, Any] = {}
    while not (terminated or truncated):
        with torch.no_grad():
            tensor_obs = torch.as_tensor(obs, dtype=torch.float32, device=q_net.device).unsqueeze(0)
            q_values = q_net(tensor_obs).squeeze(0)
            mask = torch.as_tensor(env.action_masks(), dtype=torch.bool, device=q_net.device)
            q_values = q_values.masked_fill(~mask, -1e9)
            action = int(q_values.argmax(dim=0).item())
        obs, _, terminated, truncated, info = env.step(action)
    metrics = evaluate_path(env.grid, env.path)
    metrics.update(info)
    return {
        "metrics": metrics,
        "path": env.path,
    }


def train_variant(args, config: VariantConfig) -> dict[str, Any]:
    import torch
    import torch.nn.functional as F
    from torch import optim

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    device = torch.device(args.device)
    env = OrchardDQNEnv(**env_kwargs(args))
    obs_dim = int(env.observation_space.shape[0])
    action_dim = int(env.action_space.n)
    q_net = QNetwork(obs_dim, action_dim, dueling=config.dueling, device=str(device))
    target_net = QNetwork(obs_dim, action_dim, dueling=config.dueling, device=str(device))
    target_net.load_state_dict(q_net.state_dict())
    optimizer = optim.Adam(q_net.parameters(), lr=args.learning_rate)
    replay = ReplayBuffer(args.buffer_size, obs_dim, prioritized=config.prioritized_replay)

    obs, _ = env.reset(seed=args.seed)
    episode_rewards = []
    episode_reward = 0.0

    for step in range(1, args.timesteps + 1):
        epsilon = linear_schedule(
            args.exploration_initial_eps,
            args.exploration_final_eps,
            step / max(1, int(args.timesteps * args.exploration_fraction)),
        )
        if random.random() < epsilon:
            valid_actions = np.flatnonzero(env.action_masks())
            action = int(np.random.choice(valid_actions)) if valid_actions.size else int(env.action_space.sample())
        else:
            with torch.no_grad():
                tensor_obs = torch.as_tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
                q_values = q_net(tensor_obs).squeeze(0)
                mask = torch.as_tensor(env.action_masks(), dtype=torch.bool, device=device)
                q_values = q_values.masked_fill(~mask, -1e9)
                action = int(q_values.argmax(dim=0).item())

        next_obs, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        replay.add(obs, action, reward, next_obs, done)
        episode_reward += float(reward)
        obs = next_obs

        if done:
            episode_rewards.append(episode_reward)
            episode_reward = 0.0
            obs, _ = env.reset()

        if replay.size >= args.learning_starts and step % args.train_freq == 0:
            q_net.train()
            beta = linear_schedule(args.per_beta_start, 1.0, step / max(1, args.timesteps))
            for _ in range(args.gradient_steps):
                indexes, weights = replay.sample(args.batch_size, beta=beta)
                obs_batch = torch.as_tensor(replay.obs[indexes], dtype=torch.float32, device=device)
                next_obs_batch = torch.as_tensor(replay.next_obs[indexes], dtype=torch.float32, device=device)
                action_batch = torch.as_tensor(replay.actions[indexes], dtype=torch.long, device=device).unsqueeze(1)
                reward_batch = torch.as_tensor(replay.rewards[indexes], dtype=torch.float32, device=device)
                done_batch = torch.as_tensor(replay.dones[indexes], dtype=torch.float32, device=device)
                weight_batch = torch.as_tensor(weights, dtype=torch.float32, device=device)

                current_q = q_net(obs_batch).gather(1, action_batch).squeeze(1)
                with torch.no_grad():
                    if config.double_target:
                        next_actions = q_net(next_obs_batch).argmax(dim=1, keepdim=True)
                        next_q = target_net(next_obs_batch).gather(1, next_actions).squeeze(1)
                    else:
                        next_q = target_net(next_obs_batch).max(dim=1).values
                    target_q = reward_batch + args.gamma * (1.0 - done_batch) * next_q
                td_error = target_q - current_q
                loss = (F.smooth_l1_loss(current_q, target_q, reduction="none") * weight_batch).mean()

                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(q_net.parameters(), args.max_grad_norm)
                optimizer.step()
                replay.update_priorities(indexes, np.abs(td_error.detach().cpu().numpy()))

        if step % args.target_update_interval == 0:
            target_net.load_state_dict(q_net.state_dict())

    eval_env = OrchardDQNEnv(**env_kwargs(args))
    eval_result = evaluate_policy(q_net, eval_env, args.seed)
    model_dir = Path(args.model_dir)
    model_dir.mkdir(parents=True, exist_ok=True)
    model_path = model_dir / f"{config.name}.pt"
    torch.save(
        {
            "algorithm": config.name,
            "obs_dim": obs_dim,
            "action_dim": action_dim,
            "dueling": config.dueling,
            "state_dict": q_net.state_dict(),
        },
        model_path,
    )

    return {
        "algorithm": config.name,
        "family": "custom-dqn",
        "implementation": config.implementation,
        "world": args.world,
        "cell_size_m": args.cell_size,
        "timesteps": args.timesteps,
        "seed": args.seed,
        "goal_coverage": args.goal_coverage,
        "goal_metric": args.goal_metric or ("demand" if args.intelligent_irrigation else "coverage"),
        "dynamic_obstacles": args.dynamic_obstacles,
        "dynamic_obstacle_mode": args.dynamic_obstacle_mode,
        "intelligent_irrigation": args.intelligent_irrigation,
        "spray_control": args.spray_control,
        "model": str(model_path),
        "final_metrics": eval_result["metrics"],
        "path": eval_result["path"],
        "episode_rewards_tail": episode_rewards[-10:],
    }


def write_summary(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(payload, handle, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser(description="Train custom DQN variants on the original apple_orchard-derived task.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--goal-coverage", type=float, default=1.0)
    parser.add_argument("--timesteps", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--algorithms", default="double-dqn,dueling-dqn,rainbow-dqn-lite")
    parser.add_argument("--goal-metric", choices=["coverage", "demand"], default=None)
    parser.add_argument("--dynamic-obstacles", type=int, default=0)
    parser.add_argument("--dynamic-obstacle-span", type=int, default=4)
    parser.add_argument("--dynamic-safety-radius", type=int, default=3)
    parser.add_argument("--dynamic-obstacle-mode", choices=["random", "corridor"], default="random")
    parser.add_argument("--intelligent-irrigation", action="store_true")
    parser.add_argument("--spray-control", action="store_true")
    parser.add_argument("--model-dir", default=str(default_output_dir() / "models"))
    parser.add_argument("--metrics-dir", default=str(default_output_dir() / "metrics"))
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--buffer-size", type=int, default=50000)
    parser.add_argument("--learning-starts", type=int, default=200)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--gamma", type=float, default=0.98)
    parser.add_argument("--train-freq", type=int, default=4)
    parser.add_argument("--gradient-steps", type=int, default=1)
    parser.add_argument("--target-update-interval", type=int, default=1000)
    parser.add_argument("--exploration-fraction", type=float, default=0.5)
    parser.add_argument("--exploration-initial-eps", type=float, default=1.0)
    parser.add_argument("--exploration-final-eps", type=float, default=0.05)
    parser.add_argument("--per-beta-start", type=float, default=0.4)
    parser.add_argument("--max-grad-norm", type=float, default=10.0)
    args = parser.parse_args()

    summaries = []
    for name in [item.strip() for item in args.algorithms.split(",") if item.strip()]:
        key = name.lower().replace("_", "-")
        if key == "rainbow-dqn":
            key = "rainbow-dqn-lite"
        if key not in VARIANTS:
            raise ValueError(f"Unsupported DQN variant: {name}. Choices: {', '.join(VARIANTS)}")
        print(f"=== Training {key} ===")
        summary = train_variant(args, VARIANTS[key])
        write_summary(Path(args.metrics_dir) / f"{key}_summary.json", summary)
        summaries.append(summary)

    print(json.dumps({summary["algorithm"]: summary["final_metrics"] for summary in summaries}, indent=2))


if __name__ == "__main__":
    main()
