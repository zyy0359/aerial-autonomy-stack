from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from generate_mission_from_result import load_path_from_summary
from generate_spray_mission import mission_from_path, write_mission_yaml
from orchard_world import DEFAULT_WORLD, OrchardWorldGrid, evaluate_path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MISSION = REPO_ROOT / "aircraft" / "aircraft_resources" / "missions" / "spray_paper_replay.yaml"
DEFAULT_REPORT = REPO_ROOT / "experiments" / "spray_dqn" / "outputs" / "gazebo_replay" / "gazebo_replay_report.md"


def final_metrics(payload: dict[str, Any]) -> dict[str, Any]:
    metrics = dict(payload.get("final_metrics", {}))
    if "coverage" not in metrics and "metrics" in payload:
        metrics.update(payload["metrics"])
    return metrics


def write_report(
    *,
    report: Path,
    summary: Path,
    mission: Path,
    algorithm: str,
    metrics: dict[str, Any],
    planning_metrics: dict[str, Any],
    payload: dict[str, Any],
) -> None:
    report.parent.mkdir(parents=True, exist_ok=True)
    rel_mission = mission.relative_to(REPO_ROOT) if mission.is_relative_to(REPO_ROOT) else mission
    rel_summary = summary.relative_to(REPO_ROOT) if summary.is_relative_to(REPO_ROOT) else summary
    with report.open("w", encoding="utf-8", newline="\n") as handle:
        handle.write("# Gazebo Replay Verification\n\n")
        handle.write("## Replay Artifact\n\n")
        handle.write(f"- Source summary: `{rel_summary}`\n")
        handle.write(f"- Mission YAML: `{rel_mission}`\n")
        handle.write(f"- Algorithm: `{algorithm}`\n")
        handle.write(f"- Seed: `{payload.get('seed', 'n/a')}`\n")
        handle.write(f"- Target mode: `{payload.get('target_mode', 'trees')}`\n")
        handle.write(f"- Goal metric: `{payload.get('goal_metric', 'coverage')}`\n")
        handle.write(f"- Goal threshold: `{payload.get('goal_coverage', 'n/a')}`\n\n")
        handle.write("## Planning-Layer Metrics\n\n")
        handle.write("| Metric | Value |\n")
        handle.write("|---|---:|\n")
        handle.write(f"| Coverage | {planning_metrics.get('coverage', 0.0) * 100.0:.1f}% |\n")
        handle.write(f"| Demand satisfaction | {metrics.get('demand_satisfaction', planning_metrics.get('coverage', 0.0)) * 100.0:.1f}% |\n")
        handle.write(f"| Path length | {planning_metrics.get('path_length_m', 0.0):.1f} m |\n")
        handle.write(f"| Planning collisions | {planning_metrics.get('collisions', 0)} |\n")
        handle.write(f"| Dynamic collisions | {metrics.get('dynamic_collisions', 0)} |\n")
        handle.write(f"| Min S_v | {metrics.get('min_safety_value', 1.0):.2f} |\n")
        handle.write(f"| Mission waypoints | {planning_metrics.get('waypoint_count', 0)} |\n\n")
        handle.write("## Replay Commands\n\n")
        handle.write("Start AAS/Gazebo in dev mode:\n\n")
        handle.write("```bash\n")
        handle.write("cd scripts\n")
        handle.write("DEV=true AUTOPILOT=px4 NUM_QUADS=1 NUM_VTOLS=0 WORLD=apple_orchard HEADLESS=false CAMERA=false LIDAR=false GND_CONTAINER=false RTF=1.0 ./sim_run.sh\n")
        handle.write("```\n\n")
        handle.write("Execute the generated mission in the aircraft container:\n\n")
        handle.write("```bash\n")
        handle.write(
            "docker exec -d aircraft-container-inst0_1 bash -c "
            f"\"source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && "
            f"source /aas/aircraft_ws/install/setup.bash && ros2 run mission mission --conops {rel_mission.name} "
            "--ros-args -r __ns:=/Drone1 -p use_sim_time:=true\"\n"
        )
        handle.write("```\n\n")
        handle.write("Record live position evidence:\n\n")
        handle.write("```bash\n")
        handle.write(
            "docker exec -it aircraft-container-inst0_1 bash -c "
            "\"source /opt/ros/humble/setup.bash && source /aas/github_ws/install/setup.bash && "
            "source /aas/aircraft_ws/install/setup.bash && ros2 topic echo /Drone1/fmu/out/vehicle_local_position\"\n"
        )
        handle.write("```\n\n")
        handle.write("## Verification Checklist\n\n")
        handle.write("| Item | Result | Evidence |\n")
        handle.write("|---|---|---|\n")
        handle.write("| Gazebo apple_orchard world launched | pending | screenshot/log |\n")
        handle.write("| Drone took off successfully | pending | mission log/QGC |\n")
        handle.write("| Reposition waypoint sequence executed | pending | ROS 2 position topic |\n")
        handle.write("| Mission completed or landed | pending | mission log/QGC |\n")
        handle.write("| No visible crash during replay | pending | Gazebo/QGC |\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate an AAS mission and replay report from a saved RL summary.")
    parser.add_argument("--summary", required=True)
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--mission-out", default=str(DEFAULT_MISSION))
    parser.add_argument("--report-out", default=str(DEFAULT_REPORT))
    parser.add_argument("--speed", type=float, default=5.0)
    parser.add_argument("--takeoff-altitude", type=float, default=22.0)
    parser.add_argument("--landing-altitude", type=float, default=18.0)
    parser.add_argument("--waypoint-wait", type=float, default=1.0)
    args = parser.parse_args()

    summary = Path(args.summary)
    algorithm, path, payload = load_path_from_summary(summary)
    grid = OrchardWorldGrid(
        world_path=args.world,
        cell_size_m=args.cell_size,
        altitude_m=args.takeoff_altitude,
        target_mode=payload.get("target_mode", "trees"),
        field_bounds=payload.get("field_bounds"),
        field_blocks=payload.get("field_blocks"),
        field_spacing_m=payload.get("field_spacing_m"),
    )
    mission = mission_from_path(
        grid,
        path,
        speed_m_s=args.speed,
        takeoff_altitude_m=args.takeoff_altitude,
        landing_altitude_m=args.landing_altitude,
        waypoint_wait_s=args.waypoint_wait,
        max_waypoints=len(path),
    )
    mission_out = Path(args.mission_out)
    mission_out.parent.mkdir(parents=True, exist_ok=True)
    write_mission_yaml(mission, mission_out)
    planning_metrics = evaluate_path(grid, path)
    metrics = final_metrics(payload)
    report_out = Path(args.report_out)
    write_report(
        report=report_out,
        summary=summary,
        mission=mission_out,
        algorithm=algorithm,
        metrics=metrics,
        planning_metrics=planning_metrics,
        payload=payload,
    )
    print(f"Wrote {mission_out}")
    print(f"Wrote {report_out}")


if __name__ == "__main__":
    main()
