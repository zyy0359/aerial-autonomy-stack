from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any


def goal_value(metrics: dict[str, Any], goal_metric: str) -> float:
    if goal_metric == "demand":
        return float(metrics.get("demand_satisfaction", metrics.get("coverage", 0.0)))
    return float(metrics.get("coverage", 0.0))


def curve_row(
    *,
    timesteps: int,
    algorithm: str,
    seed: int,
    metrics: dict[str, Any],
    goal_metric: str,
    goal_coverage: float,
    episode_reward: float | None = None,
    episode_steps: int | None = None,
) -> dict[str, Any]:
    progress = goal_value(metrics, goal_metric)
    collisions = float(metrics.get("collisions", 0.0))
    return {
        "timesteps": int(timesteps),
        "algorithm": algorithm,
        "seed": int(seed),
        "episode_reward": 0.0 if episode_reward is None else float(episode_reward),
        "episode_steps": 0 if episode_steps is None else int(episode_steps),
        "goal_progress": progress,
        "goal_progress_percent": progress * 100.0,
        "coverage": float(metrics.get("coverage", 0.0)),
        "coverage_percent": float(metrics.get("coverage", 0.0)) * 100.0,
        "demand_satisfaction": float(metrics.get("demand_satisfaction", metrics.get("coverage", 0.0))),
        "demand_satisfaction_percent": float(metrics.get("demand_satisfaction", metrics.get("coverage", 0.0))) * 100.0,
        "high_need_coverage": float(metrics.get("high_need_coverage", metrics.get("coverage", 0.0))),
        "dose_rmse": float(metrics.get("dose_rmse", 0.0)),
        "over_spray_ratio": float(metrics.get("over_spray_ratio", 0.0)),
        "repeat_spray_ratio": float(metrics.get("repeat_spray_ratio", 0.0)),
        "path_length_m": float(metrics.get("path_length_m", 0.0)),
        "collisions": collisions,
        "dynamic_collisions": float(metrics.get("dynamic_collisions", 0.0)),
        "mean_safety_value": float(metrics.get("mean_safety_value", 1.0)),
        "min_safety_value": float(metrics.get("min_safety_value", 1.0)),
        "safety_violation_count": float(metrics.get("safety_violation_count", 0.0)),
        "safety_overrides": float(metrics.get("safety_overrides", 0.0)),
        "success": int(progress >= float(goal_coverage) and collisions == 0.0),
    }


def write_learning_curve(rows: list[dict[str, Any]], output: str | Path | None) -> None:
    if not output or not rows:
        return
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.suffix.lower() == ".json":
        with output.open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(rows, handle, indent=2)
        csv_output = output.with_suffix(".csv")
    else:
        csv_output = output
        with output.with_suffix(".json").open("w", encoding="utf-8", newline="\n") as handle:
            json.dump(rows, handle, indent=2)
    fieldnames = list(rows[0].keys())
    with csv_output.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
