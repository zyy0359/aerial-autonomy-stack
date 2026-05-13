from __future__ import annotations

from dataclasses import dataclass
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


@dataclass(frozen=True)
class MovingObstaclePath:
    cells: tuple[tuple[int, int], ...]
    phase: int = 0


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
        dynamic_obstacle_count: int = 0,
        dynamic_obstacle_span: int = 4,
        dynamic_safety_radius_cells: int = 3,
        dynamic_obstacle_seed: int = 0,
        dynamic_obstacle_mode: str = "random",
        intelligent_irrigation: bool = False,
        irrigation_seed: int = 0,
        goal_metric: str | None = None,
        spray_control: bool = False,
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
        self.dynamic_obstacle_count = max(0, int(dynamic_obstacle_count))
        self.dynamic_obstacle_span = max(1, int(dynamic_obstacle_span))
        self.dynamic_safety_radius_cells = max(1, int(dynamic_safety_radius_cells))
        self.dynamic_obstacle_seed = int(dynamic_obstacle_seed)
        self.dynamic_obstacle_mode = dynamic_obstacle_mode
        if self.dynamic_obstacle_mode not in {"random", "corridor"}:
            raise ValueError("dynamic_obstacle_mode must be 'random' or 'corridor'.")
        self.intelligent_irrigation = bool(intelligent_irrigation)
        self.spray_control = bool(spray_control)
        self.goal_metric = goal_metric or ("demand" if self.intelligent_irrigation else "coverage")
        if self.goal_metric not in {"coverage", "demand"}:
            raise ValueError("goal_metric must be 'coverage' or 'demand'.")
        self.dynamic_paths = self._build_dynamic_obstacle_paths()
        self.demand_map = self._build_demand_map(int(irrigation_seed))
        self.action_space = spaces.Discrete(8 if self.spray_control else 4)
        obs_dim = 11
        if self.dynamic_obstacle_count:
            obs_dim += 4
        if self.intelligent_irrigation:
            obs_dim += 6
        if self.spray_control:
            obs_dim += 1
        self.observation_space = spaces.Box(
            low=0.0,
            high=1.0,
            shape=(obs_dim,),
            dtype=np.float32,
        )
        self.reset()

    def reset(self, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        self.pos = self.grid.start
        self.path = [self.pos]
        self.sprayed = np.zeros_like(self.target_mask, dtype=bool)
        self.delivered_dose = np.zeros_like(self.demand_map, dtype=np.float32)
        self.step_count = 0
        self.collision_count = 0
        self.dynamic_collision_count = 0
        self.safety_violation_count = 0
        self.safety_values: list[float] = []
        self.new_spray_count = 0
        self.repeat_spray_count = 0
        self.spray_action_count = 0
        self.no_spray_action_count = 0
        self.over_spray_amount = 0.0
        self.total_dose_progress = 0.0
        self._spray(self.pos)
        return self._obs(), self._info()

    def step(self, action: int):
        action = int(action)
        move_action = action % 4
        spray_on = (not self.spray_control) or action < 4
        row_delta, col_delta = ACTION_DELTAS[move_action]
        next_pos = (self.pos[0] + row_delta, self.pos[1] + col_delta)
        previous_distance = self._nearest_uncovered_distance()
        previous_goal_progress = self.goal_progress
        self.step_count += 1
        reward = -0.01
        terminated = False
        truncated = False
        current_dynamic = self.dynamic_obstacle_cells(self.step_count - 1)
        next_dynamic = self.dynamic_obstacle_cells(self.step_count)
        dynamic_collision = self.dynamic_obstacle_count > 0 and (
            next_pos in current_dynamic or next_pos in next_dynamic
        )
        if not self.grid.in_bounds(next_pos) or self.obstacle_mask[next_pos]:
            self.collision_count += 1
            reward -= 2.0
        elif dynamic_collision:
            self.collision_count += 1
            self.dynamic_collision_count += 1
            reward -= 5.0
        else:
            self.pos = next_pos
            self.path.append(self.pos)
            if spray_on:
                self.spray_action_count += 1
                new_cells, repeat_cells, dose_progress, over_spray = self._spray(self.pos)
                reward += 5.0 * dose_progress
                reward -= 0.15 * repeat_cells
                reward -= 0.35 * over_spray
            else:
                self.no_spray_action_count += 1
                new_cells, repeat_cells, dose_progress, over_spray = 0, 0, 0.0, 0.0
                reward -= 0.005
            reward += 0.3 * (previous_distance - self._nearest_uncovered_distance())
            if self.intelligent_irrigation:
                reward += 15.0 * max(0.0, self.goal_progress - previous_goal_progress)
                if dose_progress <= 0.0 and previous_distance <= self._nearest_uncovered_distance():
                    reward -= 0.05
            if self.goal_progress >= self.goal_coverage:
                reward += 50.0
                terminated = True
        safety_value = self.dynamic_safety_value(self.pos, time_step=self.step_count)
        self.safety_values.append(safety_value)
        if self.dynamic_obstacle_count:
            reward -= 0.5 * (1.0 - safety_value)
            if safety_value < 0.5:
                self.safety_violation_count += 1
        if self.step_count >= self.max_steps and not terminated:
            truncated = True
        return self._obs(), reward, terminated, truncated, self._info()

    def action_masks(self) -> np.ndarray:
        row, col = self.pos
        move_masks = []
        next_dynamic = self.dynamic_obstacle_cells(self.step_count + 1)
        for row_delta, col_delta in ACTION_DELTAS.values():
            candidate = (row + row_delta, col + col_delta)
            valid = self.grid.in_bounds(candidate) and not self.obstacle_mask[candidate]
            if self.dynamic_obstacle_count and candidate in next_dynamic:
                valid = False
            move_masks.append(valid)
        mask_array = np.array(move_masks, dtype=bool)
        if self.dynamic_obstacle_count and not mask_array.any():
            mask_array = np.array(
                [
                    self.grid.in_bounds((row + row_delta, col + col_delta))
                    and not self.obstacle_mask[row + row_delta, col + col_delta]
                    for row_delta, col_delta in ACTION_DELTAS.values()
                ],
                dtype=bool,
            )
        if self.spray_control:
            mask_array = np.concatenate([mask_array, mask_array])
        return mask_array

    def valid_action_mask(self) -> np.ndarray:
        return self.action_masks()

    @property
    def coverage(self) -> float:
        target_count = int(self.target_mask.sum())
        if target_count == 0:
            return 0.0
        return float((self.sprayed & self.target_mask).sum() / target_count)

    @property
    def demand_satisfaction(self) -> float:
        demand_total = float((self.demand_map * self.target_mask).sum())
        if demand_total <= 0.0:
            return self.coverage
        satisfied = np.minimum(self.delivered_dose, self.demand_map) * self.target_mask
        return float(satisfied.sum() / demand_total)

    @property
    def high_need_coverage(self) -> float:
        high_need = self.target_mask & (self.demand_map >= 1.25)
        count = int(high_need.sum())
        if count == 0:
            return self.coverage
        satisfied = (self.delivered_dose >= self.demand_map) & high_need
        return float(satisfied.sum() / count)

    @property
    def dose_rmse(self) -> float:
        targets = self.target_mask
        if int(targets.sum()) == 0:
            return 0.0
        error = (self.demand_map[targets] - self.delivered_dose[targets]).astype(np.float32)
        return float(np.sqrt(np.mean(error * error)))

    @property
    def over_spray_ratio(self) -> float:
        delivered_total = float((self.delivered_dose * self.target_mask).sum())
        if delivered_total <= 0.0:
            return 0.0
        over = np.maximum(self.delivered_dose - self.demand_map, 0.0) * self.target_mask
        return float(over.sum() / delivered_total)

    @property
    def goal_progress(self) -> float:
        return self.demand_satisfaction if self.goal_metric == "demand" else self.coverage

    @property
    def repeat_spray_ratio(self) -> float:
        total = self.new_spray_count + self.repeat_spray_count
        return 0.0 if total == 0 else float(self.repeat_spray_count / total)

    def _spray(self, cell: tuple[int, int]) -> tuple[int, int, float, float]:
        new_cells = 0
        repeat_cells = 0
        dose_progress = 0.0
        over_spray = 0.0
        for spray_cell in self.grid.spray_cells(cell):
            if not self.target_mask[spray_cell]:
                continue
            previous_dose = float(self.delivered_dose[spray_cell])
            demand = float(self.demand_map[spray_cell])
            previous_satisfied = min(previous_dose, demand)
            next_dose = previous_dose + 1.0
            next_satisfied = min(next_dose, demand)
            self.delivered_dose[spray_cell] = next_dose
            dose_progress += max(0.0, next_satisfied - previous_satisfied)
            over_spray += max(0.0, next_dose - demand) - max(0.0, previous_dose - demand)
            if previous_dose > 0.0:
                repeat_cells += 1
            else:
                self.sprayed[spray_cell] = True
                new_cells += 1
        self.new_spray_count += new_cells
        self.repeat_spray_count += repeat_cells
        self.total_dose_progress += dose_progress
        self.over_spray_amount += over_spray
        return new_cells, repeat_cells, dose_progress, over_spray

    def _nearest_uncovered_distance(self) -> float:
        nearest = self._nearest_uncovered_cell()
        if nearest is None:
            return 0.0
        row, col = self.pos
        return float(abs(nearest[0] - row) + abs(nearest[1] - col))

    def _nearest_uncovered_cell(self) -> tuple[int, int] | None:
        if self.intelligent_irrigation:
            uncovered = np.argwhere(self.target_mask & (self.delivered_dose < self.demand_map))
        else:
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
        values = [
            row / max(1, self.grid.rows - 1),
            col / max(1, self.grid.cols - 1),
            target_row / max(1, self.grid.rows - 1),
            target_col / max(1, self.grid.cols - 1),
            (target_row - row + self.grid.rows) / max(1, 2 * self.grid.rows),
            (target_col - col + self.grid.cols) / max(1, 2 * self.grid.cols),
            self.goal_progress,
            *neighbors,
        ]
        if self.dynamic_obstacle_count:
            dynamic = self.nearest_dynamic_obstacle()
            if dynamic is None:
                dynamic_row, dynamic_col = row, col
                dynamic_distance = 1.0
            else:
                dynamic_row, dynamic_col = dynamic
                dynamic_distance = min(
                    1.0,
                    (abs(dynamic_row - row) + abs(dynamic_col - col))
                    / max(1, self.grid.rows + self.grid.cols),
                )
            values.extend(
                [
                    dynamic_row / max(1, self.grid.rows - 1),
                    dynamic_col / max(1, self.grid.cols - 1),
                    dynamic_distance,
                    self.dynamic_safety_value(self.pos, time_step=self.step_count),
                ]
            )
        if self.intelligent_irrigation:
            current_demand = float(self.demand_map[self.pos]) if self.target_mask[self.pos] else 0.0
            nearest_demand = float(self.demand_map[nearest]) if self.target_mask[nearest] else 0.0
            current_delivered = float(self.delivered_dose[self.pos]) if self.target_mask[self.pos] else 0.0
            nearest_delivered = float(self.delivered_dose[nearest]) if self.target_mask[nearest] else 0.0
            current_deficit = max(0.0, current_demand - current_delivered)
            nearest_deficit = max(0.0, nearest_demand - nearest_delivered)
            values.extend(
                [
                    min(1.0, current_demand / 2.0),
                    min(1.0, nearest_demand / 2.0),
                    min(1.0, current_deficit / 2.0),
                    min(1.0, nearest_deficit / 2.0),
                    self.demand_satisfaction,
                    min(1.0, self.dose_rmse / 2.0),
                ]
            )
        if self.spray_control:
            values.append(1.0 if self.target_mask[self.pos] and self.delivered_dose[self.pos] < self.demand_map[self.pos] else 0.0)
        return np.array(values, dtype=np.float32)

    def _build_demand_map(self, seed: int) -> np.ndarray:
        demand = np.zeros((self.grid.rows, self.grid.cols), dtype=np.float32)
        if not self.intelligent_irrigation:
            demand[self.target_mask] = 1.0
            return demand
        rng = np.random.default_rng(seed)
        targets = np.argwhere(self.target_mask)
        if targets.size == 0:
            return demand
        centers = np.array(
            [
                [targets[:, 0].min(), targets[:, 1].mean()],
                [targets[:, 0].max(), targets[:, 1].mean()],
                [targets[:, 0].mean(), targets[:, 1].max()],
            ],
            dtype=np.float32,
        )
        scale = max(1.0, float(max(self.grid.rows, self.grid.cols)) / 4.0)
        for target in targets:
            distances = np.linalg.norm(centers - target.astype(np.float32), axis=1)
            stress = float(np.exp(-distances.min() / scale))
            noise = float(rng.uniform(-0.12, 0.12))
            demand_value = 0.65 + 1.05 * stress + noise
            demand[tuple(target)] = float(np.clip(demand_value, 0.5, 2.0))
        return demand

    def _build_dynamic_obstacle_paths(self) -> list[MovingObstaclePath]:
        if self.dynamic_obstacle_count <= 0:
            return []
        rng = np.random.default_rng(self.dynamic_obstacle_seed)
        free_cells = [
            (row, col)
            for row in range(self.grid.rows)
            for col in range(self.grid.cols)
            if not self.obstacle_mask[row, col]
            and not self.target_mask[row, col]
            and abs(row - self.grid.start[0]) + abs(col - self.grid.start[1]) > 3
        ]
        if self.dynamic_obstacle_mode == "corridor":
            target_cells = list(self.grid.target_cells)
            free_cells = sorted(
                free_cells,
                key=lambda cell: (
                    min(abs(cell[0] - target[0]) + abs(cell[1] - target[1]) for target in target_cells),
                    abs(cell[0] - self.grid.start[0]) + abs(cell[1] - self.grid.start[1]),
                ),
            )
        else:
            rng.shuffle(free_cells)
        paths: list[MovingObstaclePath] = []
        used: set[tuple[int, int]] = set()
        directions = [(0, 1), (1, 0)]
        for cell in free_cells:
            if len(paths) >= self.dynamic_obstacle_count:
                break
            if cell in used:
                continue
            for row_delta, col_delta in directions:
                segment = self._dynamic_segment(cell, row_delta, col_delta)
                if len(segment) >= 3 and not any(item in used for item in segment):
                    phase = int(rng.integers(0, max(1, len(segment))))
                    paths.append(MovingObstaclePath(tuple(segment), phase=phase))
                    used.update(segment)
                    break
        return paths

    def _dynamic_segment(self, center: tuple[int, int], row_delta: int, col_delta: int) -> list[tuple[int, int]]:
        cells: list[tuple[int, int]] = []
        for offset in range(-self.dynamic_obstacle_span, self.dynamic_obstacle_span + 1):
            candidate = (center[0] + row_delta * offset, center[1] + col_delta * offset)
            if (
                self.grid.in_bounds(candidate)
                and not self.obstacle_mask[candidate]
                and not self.target_mask[candidate]
            ):
                cells.append(candidate)
        return cells

    def dynamic_obstacle_cells(self, time_step: int | None = None) -> set[tuple[int, int]]:
        if not self.dynamic_paths:
            return set()
        if time_step is None:
            time_step = self.step_count
        cells = set()
        for path in self.dynamic_paths:
            route = list(path.cells)
            if len(route) > 2:
                route = route + route[-2:0:-1]
            index = (int(time_step) + path.phase) % len(route)
            cells.add(route[index])
        return cells

    def nearest_dynamic_obstacle(self) -> tuple[int, int] | None:
        cells = self.dynamic_obstacle_cells(self.step_count)
        if not cells:
            return None
        row, col = self.pos
        return min(cells, key=lambda cell: abs(cell[0] - row) + abs(cell[1] - col))

    def dynamic_safety_value(self, cell: tuple[int, int], time_step: int | None = None) -> float:
        cells = self.dynamic_obstacle_cells(time_step)
        if not cells:
            return 1.0
        distance = min(abs(item[0] - cell[0]) + abs(item[1] - cell[1]) for item in cells)
        return float(np.clip(distance / self.dynamic_safety_radius_cells, 0.0, 1.0))

    def _info(self) -> dict[str, Any]:
        mean_safety = float(np.mean(self.safety_values)) if self.safety_values else 1.0
        min_safety = float(np.min(self.safety_values)) if self.safety_values else 1.0
        return {
            "coverage": self.coverage,
            "goal_progress": self.goal_progress,
            "demand_satisfaction": self.demand_satisfaction,
            "high_need_coverage": self.high_need_coverage,
            "dose_rmse": self.dose_rmse,
            "over_spray_ratio": self.over_spray_ratio,
            "repeat_spray_ratio": self.repeat_spray_ratio,
            "path_length": max(0, len(self.path) - 1),
            "path_length_m": max(0, len(self.path) - 1) * self.grid.cell_size_m,
            "collisions": self.collision_count,
            "dynamic_collisions": self.dynamic_collision_count,
            "mean_safety_value": mean_safety,
            "min_safety_value": min_safety,
            "safety_violation_count": self.safety_violation_count,
            "new_spray_cells": self.new_spray_count,
            "repeat_spray_cells": self.repeat_spray_count,
            "spray_actions": self.spray_action_count,
            "no_spray_actions": self.no_spray_action_count,
            "over_spray_amount": self.over_spray_amount,
            "dose_progress": self.total_dose_progress,
            "step_count": self.step_count,
            "position": self.pos,
        }

    def render(self):
        chars = np.full((self.grid.rows, self.grid.cols), ".", dtype="<U1")
        chars[self.target_mask] = "T"
        chars[self.sprayed & self.target_mask] = "S"
        chars[self.obstacle_mask] = "X"
        for cell in self.dynamic_obstacle_cells(self.step_count):
            chars[cell] = "M"
        chars[self.pos] = "A"
        return "\n".join("".join(row) for row in chars)
