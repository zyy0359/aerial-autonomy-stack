#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../.."

.venv/bin/python experiments/spray_dqn/run_top5_multiseed.py \
  --algorithms dqn,drqn,dueling-dqn,rainbow-dqn-lite \
  --seeds 7,11,19,23,31 \
  --timesteps 15000 \
  --max-steps 500 \
  --goal-coverage 0.90 \
  --goal-metric demand \
  --dynamic-obstacles 2 \
  --dynamic-obstacle-mode corridor \
  --auto-spray-control \
  --safety-controller \
  --intelligent-irrigation \
  --learning-curves \
  --eval-freq 5000 \
  --output-dir experiments/spray_dqn/outputs/hierarchical_autospray_5seeds_15k_curves \
  --force
