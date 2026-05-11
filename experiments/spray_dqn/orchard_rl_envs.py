from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from orchard_dqn_env import OrchardDQNEnv
from orchard_world import DEFAULT_WORLD

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    gym = None
    spaces = None


class OrchardContinuousActionEnv(gym.Env if gym is not None else object):
    """Continuous-action proxy for SAC/TD3 on the same orchard planning task.

    The base orchard planner is grid-based. SAC and TD3 require continuous action
    spaces, so this wrapper maps a 2D continuous direction vector to the closest
    cardinal grid action before stepping the original environment.
    """

    metadata = {"render_modes": ["ansi"]}

    def __init__(
        self,
        world_path: str | Path = DEFAULT_WORLD,
        cell_size_m: float = 5.0,
        max_steps: int = 500,
        goal_coverage: float = 1.0,
        render_mode: str | None = None,
    ):
        if gym is None or spaces is None:
            raise ImportError("gymnasium is required for OrchardContinuousActionEnv.")
        super().__init__()
        self.base_env = OrchardDQNEnv(
            world_path=world_path,
            cell_size_m=cell_size_m,
            max_steps=max_steps,
            goal_coverage=goal_coverage,
            render_mode=render_mode,
        )
        self.observation_space = self.base_env.observation_space
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

    @property
    def grid(self):
        return self.base_env.grid

    @property
    def path(self):
        return self.base_env.path

    @property
    def coverage(self):
        return self.base_env.coverage

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        return self.base_env.reset(seed=seed, options=options)

    def step(self, action):
        vector = np.asarray(action, dtype=np.float32).reshape(-1)
        if vector.size < 2:
            vector = np.pad(vector, (0, 2 - vector.size))
        row_axis = float(vector[0])
        col_axis = float(vector[1])
        if abs(row_axis) >= abs(col_axis):
            discrete_action = 1 if row_axis >= 0.0 else 0
        else:
            discrete_action = 3 if col_axis >= 0.0 else 2
        return self.base_env.step(discrete_action)

    def render(self):
        return self.base_env.render()

    def close(self):
        return self.base_env.close()
