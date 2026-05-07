from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from orchard_world import DEFAULT_WORLD, OrchardWorldGrid

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover
    gym = None
    spaces = None


ACTION_DELTAS = {
    0: (-1, 0),
    1: (1, 0),
    2: (0, -1),
    3: (0, 1),
}


class OrchardDQNEnv(gym.Env if gym is not None else object):
    """DQN wrapper whose map is generated from the existing apple_orchard SDF."""

    metadata = {"render_modes": ["ansi"]}

    def __init__(
        self,
        world_path: str | Path = DEFAULT_WORLD,
        cell_size_m: float = 5.0,
        max_steps: int = 500,
        goal_coverage: float = 0.95,
        render_mode: str | None = None,
    ):
        if gym is None or spaces is None:
            raise ImportError("gymnasium is required for OrchardDQNEnv.")
        super().__init__()
        self.grid = OrchardWorldGrid(world_path=world_path, cell_size_m=cell_size_m)
        self.target_mask = np.zeros((self.grid.rows, self.grid.cols), dtype=bool)
        self.obstacle_mask = np.zeros((self.grid.rows, self.grid.cols), dtype=bool)
        for cell in self.grid.target_cells:
            self.target_mask[cell] = True
        for cell in self.grid.obstacle_cells:
            self.obstacle_mask[cell] = True
        self.max_steps = int(max_steps)
        self.goal_coverage = float(goal_coverage)
        self.render_mode = render_mode
        self.action_space = spaces.Discrete(4)
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(11,),
            dtype=np.float32,
        )
        self.reset()

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        self.pos = self.grid.start
        self.path = [self.pos]
        self.sprayed = np.zeros_like(self.target_mask, dtype=bool)
        self.step_count = 0
        self.collision_count = 0
        self.new_spray_count = 0
        self.repeat_spray_count = 0
        self._spray(self.pos)
        return self._obs(), self._info()

    def step(self, action: int):
        action = int(action)
        row_delta, col_delta = ACTION_DELTAS[action]
        next_pos = (self.pos[0] + row_delta, self.pos[1] + col_delta)
        previous_distance = self._nearest_uncovered_distance()
        self.step_count += 1
        reward = -0.01
        terminated = False
        truncated = False
        if not self.grid.in_bounds(next_pos) or self.obstacle_mask[next_pos]:
            self.collision_count += 1
            reward -= 2.0
        else:
            self.pos = next_pos
            self.path.append(self.pos)
            new_cells, repeat_cells = self._spray(self.pos)
            reward += 5.0 * new_cells
            reward -= 0.15 * repeat_cells
            reward += 0.3 * (previous_distance - self._nearest_uncovered_distance())
            if self.coverage >= self.goal_coverage:
                reward += 50.0
                terminated = True
        if self.step_count >= self.max_steps and not terminated:
            truncated = True
        return self._obs(), reward, terminated, truncated, self._info()

    @property
    def coverage(self) -> float:
        target_count = int(self.target_mask.sum())
        if target_count == 0:
            return 0.0
        return float((self.sprayed & self.target_mask).sum() / target_count)

    @property
    def repeat_spray_ratio(self) -> float:
        total = self.new_spray_count + self.repeat_spray_count
        return 0.0 if total == 0 else float(self.repeat_spray_count / total)

    def _spray(self, cell: tuple[int, int]) -> tuple[int, int]:
        new_cells = 0
        repeat_cells = 0
        for spray_cell in self.grid.spray_cells(cell):
            if not self.target_mask[spray_cell]:
                continue
            if self.sprayed[spray_cell]:
                repeat_cells += 1
            else:
                self.sprayed[spray_cell] = True
                new_cells += 1
        self.new_spray_count += new_cells
        self.repeat_spray_count += repeat_cells
        return new_cells, repeat_cells

    def _nearest_uncovered_distance(self) -> float:
        nearest = self._nearest_uncovered_cell()
        if nearest is None:
            return 0.0
        row, col = self.pos
        return float(abs(nearest[0] - row) + abs(nearest[1] - col))

    def _nearest_uncovered_cell(self) -> tuple[int, int] | None:
        uncovered = np.argwhere(self.target_mask & ~self.sprayed)
        if uncovered.size == 0:
            return None
        row, col = self.pos
        distances = np.abs(uncovered[:, 0] - row) + np.abs(uncovered[:, 1] - col)
        nearest = uncovered[int(distances.argmin())]
        return int(nearest[0]), int(nearest[1])

    def _obs(self) -> np.ndarray:
        row, col = self.pos
        nearest = self._nearest_uncovered_cell() or self.pos
        target_row, target_col = nearest
        neighbors = []
        for row_delta, col_delta in ACTION_DELTAS.values():
            candidate = (row + row_delta, col + col_delta)
            blocked = (not self.grid.in_bounds(candidate)) or self.obstacle_mask[candidate]
            neighbors.append(1.0 if blocked else 0.0)
        return np.array(
            [
                row / max(1, self.grid.rows - 1),
                col / max(1, self.grid.cols - 1),
                target_row / max(1, self.grid.rows - 1),
                target_col / max(1, self.grid.cols - 1),
                (target_row - row + self.grid.rows) / max(1, 2 * self.grid.rows),
                (target_col - col + self.grid.cols) / max(1, 2 * self.grid.cols),
                self.coverage,
                *neighbors,
            ],
            dtype=np.float32,
        )

    def _info(self) -> dict[str, Any]:
        return {
            "coverage": self.coverage,
            "repeat_spray_ratio": self.repeat_spray_ratio,
            "path_length": max(0, len(self.path) - 1),
            "collisions": self.collision_count,
            "new_spray_cells": self.new_spray_count,
            "repeat_spray_cells": self.repeat_spray_count,
            "step_count": self.step_count,
            "position": self.pos,
        }

    def render(self):
        chars = np.full((self.grid.rows, self.grid.cols), ".", dtype="<U1")
        chars[self.target_mask] = "T"
        chars[self.sprayed & self.target_mask] = "S"
        chars[self.obstacle_mask] = "X"
        chars[self.pos] = "A"
        return "\n".join("".join(row) for row in chars)
