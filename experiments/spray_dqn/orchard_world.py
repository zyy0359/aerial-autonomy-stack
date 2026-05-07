from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable


REPO_ROOT = Path(__file__).resolve().parents[2]
SIM_WORLD_DIR = REPO_ROOT / "simulation" / "simulation_resources" / "simulation_worlds"
DEFAULT_WORLD = SIM_WORLD_DIR / "apple_orchard.sdf"
TREE_MODELS = {"apple", "birch"}
CONTAINER_MODELS = {"apple_grid", "birch_row"}
OBSTACLE_MODELS = {"jeep", "subaru", "airstream", "gazebo", "person_standing"}


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0


@dataclass(frozen=True)
class OrchardEntity:
    name: str
    model: str
    x: float
    y: float
    z: float
    yaw: float
    role: str


def parse_pose(pose_el: ET.Element | None) -> Pose2D:
    if pose_el is None or not pose_el.text:
        return Pose2D(0.0, 0.0, 0.0, 0.0)
    values = [float(value) for value in pose_el.text.split()]
    values += [0.0] * (6 - len(values))
    yaw = values[5]
    if pose_el.attrib.get("degrees", "false").lower() == "true":
        yaw = math.radians(yaw)
    return Pose2D(values[0], values[1], values[2], yaw)


def compose_pose(parent: Pose2D, child: Pose2D) -> Pose2D:
    cos_yaw = math.cos(parent.yaw)
    sin_yaw = math.sin(parent.yaw)
    x = parent.x + child.x * cos_yaw - child.y * sin_yaw
    y = parent.y + child.x * sin_yaw + child.y * cos_yaw
    return Pose2D(x=x, y=y, z=parent.z + child.z, yaw=parent.yaw + child.yaw)


def model_name_from_uri(uri: str) -> str:
    if not uri.startswith("model://"):
        return uri
    return uri.replace("model://", "", 1).strip("/").split("/")[0]


def model_sdf_path(model: str) -> Path | None:
    model_dir = SIM_WORLD_DIR / model
    for filename in ("model.sdf", "model.sdf.erb"):
        path = model_dir / filename
        if path.exists():
            return path
    return None


def role_for_model(model: str) -> str:
    if model in TREE_MODELS:
        return "target"
    if model in OBSTACLE_MODELS:
        return "obstacle"
    return "reference"


def parse_include(include_el: ET.Element, parent_pose: Pose2D, prefix: str = "") -> tuple[str, str, Pose2D]:
    uri = include_el.findtext("uri", default="").strip()
    model = model_name_from_uri(uri)
    name = include_el.findtext("name", default=model).strip()
    if prefix:
        name = f"{prefix}/{name}"
    pose = compose_pose(parent_pose, parse_pose(include_el.find("pose")))
    return name, model, pose


def collect_model_entities(model: str, parent_pose: Pose2D, prefix: str, stack: tuple[str, ...] = ()) -> list[OrchardEntity]:
    if model in stack:
        return []
    path = model_sdf_path(model)
    if path is None:
        return []
    root = ET.parse(path).getroot()
    entities: list[OrchardEntity] = []
    for include_el in root.findall(".//include"):
        name, child_model, pose = parse_include(include_el, parent_pose, prefix=prefix)
        if child_model in CONTAINER_MODELS:
            entities.extend(collect_model_entities(child_model, pose, name, stack + (model,)))
            continue
        nested = collect_model_entities(child_model, pose, name, stack + (model,))
        if nested:
            entities.extend(nested)
            continue
        entities.append(
            OrchardEntity(
                name=name,
                model=child_model,
                x=pose.x,
                y=pose.y,
                z=pose.z,
                yaw=pose.yaw,
                role=role_for_model(child_model),
            )
        )
    return entities


def collect_orchard_entities(world_path: str | Path = DEFAULT_WORLD) -> list[OrchardEntity]:
    world_path = Path(world_path)
    root = ET.parse(world_path).getroot()
    entities: list[OrchardEntity] = []
    for include_el in root.findall(".//world//include"):
        name, model, pose = parse_include(include_el, Pose2D(0.0, 0.0, 0.0, 0.0))
        if model in CONTAINER_MODELS:
            entities.extend(collect_model_entities(model, pose, name))
            continue
        nested = collect_model_entities(model, pose, name)
        if nested:
            entities.extend(nested)
            continue
        entities.append(
            OrchardEntity(
                name=name,
                model=model,
                x=pose.x,
                y=pose.y,
                z=pose.z,
                yaw=pose.yaw,
                role=role_for_model(model),
            )
        )
    return entities


class OrchardWorldGrid:
    """Planning grid derived from the existing apple_orchard SDF world."""

    def __init__(
        self,
        world_path: str | Path = DEFAULT_WORLD,
        cell_size_m: float = 5.0,
        margin_m: float = 15.0,
        spray_radius_cells: int = 1,
        obstacle_radius_m: float = 6.0,
        start_xy: tuple[float, float] = (0.0, 0.0),
        altitude_m: float = 22.0,
    ):
        self.world_path = Path(world_path)
        self.cell_size_m = float(cell_size_m)
        self.margin_m = float(margin_m)
        self.spray_radius_cells = int(spray_radius_cells)
        self.obstacle_radius_m = float(obstacle_radius_m)
        self.start_xy = start_xy
        self.altitude_m = float(altitude_m)
        self.entities = collect_orchard_entities(self.world_path)
        self.targets = [entity for entity in self.entities if entity.role == "target"]
        self.obstacles = [entity for entity in self.entities if entity.role == "obstacle"]
        if not self.targets:
            raise ValueError(f"No active apple/birch target trees found in {self.world_path}.")
        self._build_grid()

    def _build_grid(self) -> None:
        xs = [entity.x for entity in self.targets + self.obstacles] + [self.start_xy[0]]
        ys = [entity.y for entity in self.targets + self.obstacles] + [self.start_xy[1]]
        self.min_x = math.floor((min(xs) - self.margin_m) / self.cell_size_m) * self.cell_size_m
        self.max_x = math.ceil((max(xs) + self.margin_m) / self.cell_size_m) * self.cell_size_m
        self.min_y = math.floor((min(ys) - self.margin_m) / self.cell_size_m) * self.cell_size_m
        self.max_y = math.ceil((max(ys) + self.margin_m) / self.cell_size_m) * self.cell_size_m
        self.cols = int(round((self.max_x - self.min_x) / self.cell_size_m)) + 1
        self.rows = int(round((self.max_y - self.min_y) / self.cell_size_m)) + 1
        self.target_cells: set[tuple[int, int]] = set()
        self.obstacle_cells: set[tuple[int, int]] = set()
        for entity in self.targets:
            self.target_cells.add(self.world_to_grid(entity.x, entity.y))
        obstacle_radius_cells = max(1, int(math.ceil(self.obstacle_radius_m / self.cell_size_m)))
        for entity in self.obstacles:
            center = self.world_to_grid(entity.x, entity.y)
            for row, col in self.cells_within(center, obstacle_radius_cells):
                self.obstacle_cells.add((row, col))
        self.start = self.nearest_free_cell(self.world_to_grid(*self.start_xy))
        self.obstacle_cells.discard(self.start)

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        col = int(round((x - self.min_x) / self.cell_size_m))
        row = int(round((self.max_y - y) / self.cell_size_m))
        return self.clamp_cell((row, col))

    def grid_to_world(self, cell: tuple[int, int]) -> tuple[float, float, float]:
        row, col = cell
        x = self.min_x + col * self.cell_size_m
        y = self.max_y - row * self.cell_size_m
        return x, y, self.altitude_m

    def clamp_cell(self, cell: tuple[int, int]) -> tuple[int, int]:
        row, col = cell
        return min(max(row, 0), self.rows - 1), min(max(col, 0), self.cols - 1)

    def in_bounds(self, cell: tuple[int, int]) -> bool:
        row, col = cell
        return 0 <= row < self.rows and 0 <= col < self.cols

    def cells_within(self, center: tuple[int, int], radius: int) -> Iterable[tuple[int, int]]:
        row0, col0 = center
        for row in range(row0 - radius, row0 + radius + 1):
            for col in range(col0 - radius, col0 + radius + 1):
                if self.in_bounds((row, col)) and abs(row - row0) + abs(col - col0) <= radius:
                    yield row, col

    def spray_cells(self, cell: tuple[int, int]) -> list[tuple[int, int]]:
        return list(self.cells_within(cell, self.spray_radius_cells))

    def nearest_free_cell(self, cell: tuple[int, int]) -> tuple[int, int]:
        if self.in_bounds(cell) and cell not in self.obstacle_cells:
            return cell
        for radius in range(1, max(self.rows, self.cols)):
            for candidate in self.cells_within(cell, radius):
                if candidate not in self.obstacle_cells:
                    return candidate
        raise ValueError("No free cell available in orchard grid.")

    def active_target_summary(self) -> dict[str, Any]:
        return {
            "world": str(self.world_path),
            "target_count": len(self.targets),
            "target_cell_count": len(self.target_cells),
            "obstacle_count": len(self.obstacles),
            "rows": self.rows,
            "cols": self.cols,
            "cell_size_m": self.cell_size_m,
            "start": self.start,
        }


def evaluate_path(grid: OrchardWorldGrid, path: list[tuple[int, int]]) -> dict[str, Any]:
    sprayed: set[tuple[int, int]] = set()
    collisions = 0
    new_cells = 0
    repeat_cells = 0
    distance_m = 0.0
    previous: tuple[int, int] | None = None
    for cell in path:
        if previous is not None:
            distance_m += math.dist(grid.grid_to_world(previous)[:2], grid.grid_to_world(cell)[:2])
        previous = cell
        if not grid.in_bounds(cell) or cell in grid.obstacle_cells:
            collisions += 1
            continue
        for spray_cell in grid.spray_cells(cell):
            if spray_cell not in grid.target_cells:
                continue
            if spray_cell in sprayed:
                repeat_cells += 1
            else:
                sprayed.add(spray_cell)
                new_cells += 1
    target_count = len(grid.target_cells)
    total_sprays = new_cells + repeat_cells
    return {
        "coverage": 0.0 if target_count == 0 else float(len(sprayed & grid.target_cells) / target_count),
        "repeat_spray_ratio": 0.0 if total_sprays == 0 else float(repeat_cells / total_sprays),
        "path_length_m": distance_m,
        "collisions": collisions,
        "new_spray_cells": new_cells,
        "repeat_spray_cells": repeat_cells,
        "waypoint_count": len(path),
    }


def orchard_row_path(grid: OrchardWorldGrid) -> list[tuple[int, int]]:
    target_cells = sorted(
        {grid.world_to_grid(entity.x, entity.y) for entity in grid.targets},
        key=lambda cell: (cell[1], cell[0]),
    )
    path = [grid.start]
    current = grid.start
    by_col: dict[int, list[tuple[int, int]]] = {}
    for cell in target_cells:
        by_col.setdefault(cell[1], []).append(cell)
    reverse = False
    for col in sorted(by_col):
        cells = sorted(by_col[col], reverse=reverse)
        for cell in cells:
            if cell != current:
                path.extend(shortest_grid_path(grid, current, cell)[1:])
                current = path[-1]
        reverse = not reverse
    return path


def shortest_grid_path(grid: OrchardWorldGrid, start: tuple[int, int], goal: tuple[int, int]) -> list[tuple[int, int]]:
    if start == goal:
        return [start]
    frontier: deque[tuple[int, int]] = deque([start])
    came_from: dict[tuple[int, int], tuple[int, int] | None] = {start: None}
    while frontier:
        current = frontier.popleft()
        if current == goal:
            break
        row, col = current
        for candidate in ((row - 1, col), (row + 1, col), (row, col - 1), (row, col + 1)):
            if not grid.in_bounds(candidate):
                continue
            if candidate in grid.obstacle_cells:
                continue
            if candidate in came_from:
                continue
            came_from[candidate] = current
            frontier.append(candidate)
    if goal not in came_from:
        return [start, goal]
    path = [goal]
    current = goal
    while came_from[current] is not None:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def default_output_dir() -> Path:
    return Path(__file__).resolve().parent / "outputs"
