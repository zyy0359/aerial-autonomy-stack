from __future__ import annotations

import argparse
import json
from pathlib import Path

from generate_spray_mission import mission_from_path, write_mission_yaml
from orchard_world import DEFAULT_WORLD, OrchardWorldGrid, evaluate_path


def load_path_from_summary(summary_path: Path) -> tuple[str, list[tuple[int, int]]]:
    with summary_path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    if "path" not in payload:
        raise ValueError(f"No path found in {summary_path}.")
    algorithm = payload.get("algorithm", summary_path.stem.replace("_summary", ""))
    return str(algorithm), [tuple(cell) for cell in payload["path"]]


def load_path_from_benchmark(benchmark_path: Path, algorithm: str) -> tuple[str, list[tuple[int, int]]]:
    with benchmark_path.open("r", encoding="utf-8") as handle:
        payload = json.load(handle)
    algorithms = payload.get("algorithms", {})
    if algorithm not in algorithms:
        raise ValueError(f"Algorithm '{algorithm}' not found in {benchmark_path}.")
    return algorithm, [tuple(cell) for cell in algorithms[algorithm]["path"]]


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate an AAS mission YAML from a saved RL result path.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--summary", default=None)
    parser.add_argument("--benchmark-json", default=None)
    parser.add_argument("--algorithm", default=None)
    parser.add_argument("--output", required=True)
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--takeoff-altitude", type=float, default=22.0)
    parser.add_argument("--landing-altitude", type=float, default=18.0)
    parser.add_argument("--waypoint-wait", type=float, default=1.0)
    args = parser.parse_args()

    if args.summary:
        algorithm, path = load_path_from_summary(Path(args.summary))
    elif args.benchmark_json and args.algorithm:
        algorithm, path = load_path_from_benchmark(Path(args.benchmark_json), args.algorithm)
    else:
        raise SystemExit("Use either --summary or --benchmark-json with --algorithm.")

    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size, altitude_m=args.takeoff_altitude)
    mission = mission_from_path(
        grid,
        path,
        speed_m_s=args.speed,
        takeoff_altitude_m=args.takeoff_altitude,
        landing_altitude_m=args.landing_altitude,
        waypoint_wait_s=args.waypoint_wait,
        max_waypoints=len(path),
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_mission_yaml(mission, output)
    metrics = evaluate_path(grid, path)
    print(f"Wrote {output}")
    print(f"algorithm={algorithm}")
    print(
        f"coverage={metrics['coverage']:.3f} "
        f"path_length_m={metrics['path_length_m']:.1f} "
        f"collisions={metrics['collisions']} "
        f"waypoints={metrics['waypoint_count']}"
    )


if __name__ == "__main__":
    main()
