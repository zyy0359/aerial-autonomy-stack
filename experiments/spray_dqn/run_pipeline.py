from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

from orchard_world import DEFAULT_WORLD, default_output_dir


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = default_output_dir() / "models" / "dqn_apple_orchard.zip"
DEFAULT_MISSION = REPO_ROOT / "aircraft" / "aircraft_resources" / "missions" / "spray_dqn.yaml"
DEFAULT_EVAL = default_output_dir() / "metrics" / "evaluation.json"
DEFAULT_PLOT = default_output_dir() / "plots" / "dqn_coverage.png"


def run(cmd: list[str]) -> None:
    print("+ " + " ".join(str(part) for part in cmd))
    subprocess.run(cmd, check=True, cwd=REPO_ROOT)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Train/evaluate DQN, generate AAS mission YAML, and plot coverage for apple_orchard."
    )
    parser.add_argument("--world", default=str(DEFAULT_WORLD))
    parser.add_argument("--cell-size", type=float, default=5.0)
    parser.add_argument("--timesteps", type=int, default=20000)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--mission-out", default=str(DEFAULT_MISSION))
    parser.add_argument("--eval-out", default=str(DEFAULT_EVAL))
    parser.add_argument("--plot-out", default=str(DEFAULT_PLOT))
    parser.add_argument("--skip-train", action="store_true")
    args = parser.parse_args()

    model = Path(args.model)
    model_for_train = str(model.with_suffix("")) if model.suffix == ".zip" else str(model)
    if not args.skip_train:
        run(
            [
                sys.executable,
                "experiments/spray_dqn/train_dqn.py",
                "--world",
                args.world,
                "--cell-size",
                str(args.cell_size),
                "--timesteps",
                str(args.timesteps),
                "--device",
                args.device,
                "--model-out",
                model_for_train,
                "--summary-out",
                str(default_output_dir() / "metrics" / "train_summary.json"),
            ]
        )

    if not model.exists():
        raise SystemExit(f"Model not found: {model}. Run without --skip-train first.")

    run(
        [
            sys.executable,
            "experiments/spray_dqn/evaluate_dqn.py",
            "--world",
            args.world,
            "--cell-size",
            str(args.cell_size),
            "--model",
            str(model),
            "--output",
            args.eval_out,
        ]
    )
    run(
        [
            sys.executable,
            "experiments/spray_dqn/generate_spray_mission.py",
            "--world",
            args.world,
            "--cell-size",
            str(args.cell_size),
            "--policy",
            "dqn",
            "--model",
            str(model),
            "--output",
            args.mission_out,
        ]
    )
    run(
        [
            sys.executable,
            "experiments/spray_dqn/plot_coverage.py",
            "--world",
            args.world,
            "--cell-size",
            str(args.cell_size),
            "--policy",
            "dqn",
            "--model",
            str(model),
            "--output",
            args.plot_out,
        ]
    )


if __name__ == "__main__":
    main()

