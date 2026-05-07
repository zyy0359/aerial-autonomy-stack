from __future__ import annotations

import argparse
from pathlib import Path

from orchard_world import DEFAULT_WORLD, OrchardWorldGrid, default_output_dir, evaluate_path, orchard_row_path


DEFAULT_MISSION_OUT = (
    Path(__file__).resolve().parents[2] / "aircraft" / "aircraft_resources" / "missions" / "spray_dqn.yaml"
)


def dqn_path(world: str, model_path: str, cell_size: float) -> list[tuple[int, int]]:
    try:
        from stable_baselines3 import DQN
    except ImportError as exc:
        raise SystemExit("stable-baselines3 is required for --policy dqn.") from exc
    from orchard_dqn_env import OrchardDQNEnv
    env = OrchardDQNEnv(world_path=world, cell_size_m=cell_size)
    model = DQN.load(model_path, env=env)
    obs, _ = env.reset()
    terminated = False
    truncated = False
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=True)
        obs, _, terminated, truncated, _ = env.step(int(action))
    return env.path


def compress_path(path: list[tuple[int, int]], max_waypoints: int) -> list[tuple[int, int]]:
    if len(path) <= max_waypoints:
        return path
    if max_waypoints < 2:
        raise ValueError("max_waypoints must be at least 2 to preserve start and end points.")
    last_index = len(path) - 1
    indexes = [
        round(i * last_index / (max_waypoints - 1))
        for i in range(max_waypoints)
    ]
    compressed: list[tuple[int, int]] = []
    seen_indexes: set[int] = set()
    for index in indexes:
        if index in seen_indexes:
            continue
        compressed.append(path[index])
        seen_indexes.add(index)
    if compressed[0] != path[0]:
        compressed.insert(0, path[0])
    if compressed[-1] != path[-1]:
        compressed.append(path[-1])
    return compressed[:max_waypoints]


def mission_from_path(
    grid: OrchardWorldGrid,
    path: list[tuple[int, int]],
    speed_m_s: float,
    takeoff_altitude_m: float,
    landing_altitude_m: float,
    waypoint_wait_s: float,
    max_waypoints: int,
) -> dict:
    waypoints = compress_path(path, max_waypoints=max_waypoints)
    steps = [
        {
            "action": "takeoff",
            "params": {
                "takeoff_altitude": float(takeoff_altitude_m),
                "vtol_transition_heading": 0.0,
                "vtol_loiter_nord": 100.0,
                "vtol_loiter_east": 100.0,
                "vtol_loiter_alt": float(takeoff_altitude_m),
            },
        },
        {"action": "wait", "params": {"duration": 3.0}},
        {"action": "speed", "params": {"speed": float(speed_m_s)}},
    ]
    for cell in waypoints:
        east, north, altitude = grid.grid_to_world(cell)
        steps.append(
            {
                "action": "reposition",
                "params": {
                    "east": round(east, 3),
                    "north": round(north, 3),
                    "altitude": round(altitude, 3),
                },
            }
        )
        if waypoint_wait_s > 0.0:
            steps.append({"action": "wait", "params": {"duration": float(waypoint_wait_s)}})
    steps.append(
        {
            "action": "land",
            "params": {
                "landing_altitude": float(landing_altitude_m),
                "vtol_transition_heading": 0.0,
            },
        }
    )
    return {"steps": steps}


def write_mission_yaml(mission: dict, output: Path) -> None:
    with output.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("steps:\n")
        for step in mission["steps"]:
            handle.write(f"- action: {step['action']}\n")
            params = step.get("params", {})
            if params:
                handle.write("  params:\n")
                for key, value in params.items():
                    handle.write(f"    {key}: {float(value):.3f}\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate AAS mission YAML using the original apple_orchard world.")
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--policy", choices=["orchard-row", "dqn"], default="orchard-row")
    parser.add_argument("--model", default=str(default_output_dir() / "models" / "dqn_apple_orchard.zip"))
    parser.add_argument("--output", default=str(DEFAULT_MISSION_OUT))
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--takeoff-altitude", type=float, default=22.0)
    parser.add_argument("--landing-altitude", type=float, default=18.0)
    parser.add_argument("--waypoint-wait", type=float, default=1.0)
    parser.add_argument("--max-waypoints", type=int, default=80)
    args = parser.parse_args()

    grid = OrchardWorldGrid(world_path=args.world, cell_size_m=args.cell_size, altitude_m=args.takeoff_altitude)
    path = orchard_row_path(grid) if args.policy == "orchard-row" else dqn_path(args.world, args.model, args.cell_size)
    mission = mission_from_path(
        grid,
        path,
        speed_m_s=args.speed,
        takeoff_altitude_m=args.takeoff_altitude,
        landing_altitude_m=args.landing_altitude,
        waypoint_wait_s=args.waypoint_wait,
        max_waypoints=args.max_waypoints,
    )
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_mission_yaml(mission, output)
    metrics = evaluate_path(grid, path)
    print(f"Wrote {output}")
    print(grid.active_target_summary())
    print(
        f"coverage={metrics['coverage']:.3f} "
        f"path_length_m={metrics['path_length_m']:.1f} "
        f"collisions={metrics['collisions']}"
    )


if __name__ == "__main__":
    main()
