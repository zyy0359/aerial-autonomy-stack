from __future__ import annotations

import argparse
import json
import random
from pathlib import Path
from typing import Any

import numpy as np

from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD, default_output_dir, evaluate_path


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


class EpisodeReplayBuffer:
    def __init__(self, capacity: int, sequence_length: int, obs_dim: int):
        self.capacity = int(capacity)
        self.sequence_length = int(sequence_length)
        self.obs_dim = int(obs_dim)
        self.episodes: list[list[tuple[np.ndarray, int, float, np.ndarray, bool]]] = []

    def add_episode(self, episode: list[tuple[np.ndarray, int, float, np.ndarray, bool]]) -> None:
        if not episode:
            return
        self.episodes.append(episode)
        if len(self.episodes) > self.capacity:
            self.episodes.pop(0)

    @property
    def size(self) -> int:
        return sum(len(episode) for episode in self.episodes)

    def sample(self, batch_size: int) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        candidates = [episode for episode in self.episodes if len(episode) >= self.sequence_length]
        if not candidates:
            candidates = [episode for episode in self.episodes if episode]
        obs_batch = np.zeros((batch_size, self.sequence_length, self.obs_dim), dtype=np.float32)
        next_obs_batch = np.zeros_like(obs_batch)
        action_batch = np.zeros((batch_size, self.sequence_length), dtype=np.int64)
        reward_batch = np.zeros((batch_size, self.sequence_length), dtype=np.float32)
        done_batch = np.ones((batch_size, self.sequence_length), dtype=np.float32)
        for batch_index in range(batch_size):
            episode = random.choice(candidates)
            if len(episode) >= self.sequence_length:
                start = random.randint(0, len(episode) - self.sequence_length)
                chunk = episode[start : start + self.sequence_length]
            else:
                chunk = episode
            for step_index, (obs, action, reward, next_obs, done) in enumerate(chunk):
                obs_batch[batch_index, step_index] = obs
                next_obs_batch[batch_index, step_index] = next_obs
                action_batch[batch_index, step_index] = action
                reward_batch[batch_index, step_index] = reward
                done_batch[batch_index, step_index] = float(done)
        return obs_batch, action_batch, reward_batch, next_obs_batch, done_batch


def linear_schedule(start: float, end: float, progress: float) -> float:
    clipped = min(max(progress, 0.0), 1.0)
    return start + clipped * (end - start)


def evaluate_policy(model, env: OrchardDQNEnv, seed: int, device) -> dict[str, Any]:
    import torch

    model.eval()
    obs, _ = env.reset(seed=seed)
    hidden = None
    terminated = False
    truncated = False
    info: dict[str, Any] = {}
    while not (terminated or truncated):
        with torch.no_grad():
            obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=device).view(1, 1, -1)
            q_values, hidden = model(obs_tensor, hidden)
            q_values = q_values[:, -1, :].squeeze(0)
            mask = torch.as_tensor(env.action_masks(), dtype=torch.bool, device=device)
            q_values = q_values.masked_fill(~mask, -1e9)
            action = int(q_values.argmax(dim=0).item())
        obs, _, terminated, truncated, info = env.step(action)
    metrics = evaluate_path(env.grid, env.path)
    metrics.update(info)
    return {"metrics": metrics, "path": env.path}


def train(args) -> dict[str, Any]:
    import torch
    import torch.nn.functional as F
    from torch import nn, optim

    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)

    device = torch.device(args.device)
    env = OrchardDQNEnv(**env_kwargs(args))
    obs_dim = int(env.observation_space.shape[0])
    action_dim = int(env.action_space.n)

    class DRQN(nn.Module):
        def __init__(self):
            super().__init__()
            self.encoder = nn.Sequential(
                nn.Linear(obs_dim, args.hidden_size),
                nn.ReLU(),
            )
            self.gru = nn.GRU(args.hidden_size, args.hidden_size, batch_first=True)
            self.head = nn.Sequential(
                nn.Linear(args.hidden_size, args.hidden_size),
                nn.ReLU(),
                nn.Linear(args.hidden_size, action_dim),
            )

        def forward(self, obs_seq, hidden=None):
            batch, steps, _ = obs_seq.shape
            features = self.encoder(obs_seq.reshape(batch * steps, obs_dim))
            features = features.reshape(batch, steps, args.hidden_size)
            recurrent, hidden = self.gru(features, hidden)
            return self.head(recurrent), hidden

    q_net = DRQN().to(device)
    target_net = DRQN().to(device)
    target_net.load_state_dict(q_net.state_dict())
    optimizer = optim.Adam(q_net.parameters(), lr=args.learning_rate)
    replay = EpisodeReplayBuffer(args.episode_buffer_size, args.sequence_length, obs_dim)

    obs, _ = env.reset(seed=args.seed)
    hidden = None
    episode: list[tuple[np.ndarray, int, float, np.ndarray, bool]] = []
    episode_rewards: list[float] = []
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
                obs_tensor = torch.as_tensor(obs, dtype=torch.float32, device=device).view(1, 1, -1)
                q_values, hidden = q_net(obs_tensor, hidden)
                q_values = q_values[:, -1, :].squeeze(0)
                mask = torch.as_tensor(env.action_masks(), dtype=torch.bool, device=device)
                q_values = q_values.masked_fill(~mask, -1e9)
                action = int(q_values.argmax(dim=0).item())

        next_obs, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        episode.append((obs.copy(), action, float(reward), next_obs.copy(), done))
        episode_reward += float(reward)
        obs = next_obs

        if done:
            replay.add_episode(episode)
            episode_rewards.append(episode_reward)
            episode = []
            episode_reward = 0.0
            obs, _ = env.reset()
            hidden = None

        if replay.size >= args.learning_starts and step % args.train_freq == 0:
            q_net.train()
            for _ in range(args.gradient_steps):
                obs_b, action_b, reward_b, next_obs_b, done_b = replay.sample(args.batch_size)
                obs_batch = torch.as_tensor(obs_b, dtype=torch.float32, device=device)
                next_obs_batch = torch.as_tensor(next_obs_b, dtype=torch.float32, device=device)
                action_batch = torch.as_tensor(action_b, dtype=torch.long, device=device).unsqueeze(-1)
                reward_batch = torch.as_tensor(reward_b, dtype=torch.float32, device=device)
                done_batch = torch.as_tensor(done_b, dtype=torch.float32, device=device)

                q_values, _ = q_net(obs_batch)
                current_q = q_values.gather(2, action_batch).squeeze(2)
                with torch.no_grad():
                    next_actions = q_net(next_obs_batch)[0].argmax(dim=2, keepdim=True)
                    next_q = target_net(next_obs_batch)[0].gather(2, next_actions).squeeze(2)
                    target_q = reward_batch + args.gamma * (1.0 - done_batch) * next_q
                loss = F.smooth_l1_loss(current_q, target_q)

                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(q_net.parameters(), args.max_grad_norm)
                optimizer.step()

        if step % args.target_update_interval == 0:
            target_net.load_state_dict(q_net.state_dict())

    eval_env = OrchardDQNEnv(**env_kwargs(args))
    eval_result = evaluate_policy(q_net, eval_env, args.seed, device)

    model_out = Path(args.model_out)
    model_out.parent.mkdir(parents=True, exist_ok=True)
    torch.save(
        {
            "algorithm": "drqn",
            "obs_dim": obs_dim,
            "action_dim": action_dim,
            "hidden_size": args.hidden_size,
            "state_dict": q_net.state_dict(),
        },
        model_out,
    )
    return {
        "algorithm": "drqn",
        "family": "custom-recurrent-dqn",
        "implementation": "Custom PyTorch DRQN with a GRU memory layer for partially observable dynamic orchard planning.",
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
        "model": str(model_out),
        "final_metrics": eval_result["metrics"],
        "path": eval_result["path"],
        "episode_rewards_tail": episode_rewards[-10:],
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Train DRQN on the enhanced apple_orchard spray planning task.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--goal-coverage", type=float, default=0.97)
    parser.add_argument("--goal-metric", choices=["coverage", "demand"], default=None)
    parser.add_argument("--timesteps", type=int, default=30000)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--dynamic-obstacles", type=int, default=2)
    parser.add_argument("--dynamic-obstacle-span", type=int, default=4)
    parser.add_argument("--dynamic-safety-radius", type=int, default=3)
    parser.add_argument("--dynamic-obstacle-mode", choices=["random", "corridor"], default="random")
    parser.add_argument("--intelligent-irrigation", action="store_true")
    parser.add_argument("--spray-control", action="store_true")
    parser.add_argument("--model-out", default=str(default_output_dir() / "models" / "drqn.pt"))
    parser.add_argument("--summary-out", default=str(default_output_dir() / "metrics" / "drqn_summary.json"))
    parser.add_argument("--hidden-size", type=int, default=128)
    parser.add_argument("--episode-buffer-size", type=int, default=200)
    parser.add_argument("--sequence-length", type=int, default=16)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--learning-starts", type=int, default=500)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--gamma", type=float, default=0.98)
    parser.add_argument("--train-freq", type=int, default=4)
    parser.add_argument("--gradient-steps", type=int, default=1)
    parser.add_argument("--target-update-interval", type=int, default=1000)
    parser.add_argument("--exploration-fraction", type=float, default=0.6)
    parser.add_argument("--exploration-initial-eps", type=float, default=1.0)
    parser.add_argument("--exploration-final-eps", type=float, default=0.05)
    parser.add_argument("--max-grad-norm", type=float, default=10.0)
    args = parser.parse_args()

    summary = train(args)
    summary_out = Path(args.summary_out)
    summary_out.parent.mkdir(parents=True, exist_ok=True)
    with summary_out.open("w", encoding="utf-8", newline="\n") as handle:
        json.dump(summary, handle, indent=2)
    print(json.dumps(summary["final_metrics"], indent=2))


if __name__ == "__main__":
    main()
