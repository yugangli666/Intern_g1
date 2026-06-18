#!/usr/bin/env bash
set -euo pipefail

cd /home/ubuntu/InternNav
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
conda activate Intern

python scripts/realworld/http_internvla_server.py \
  --device cuda:0 \
  --model_path checkpoints/InternVLA-N1-DualVLN \
  --resize_w 384 \
  --resize_h 384 \
  --num_history 8 \
  --plan_step_gap 8 \
  --server_log_dir g1_server_logs \
  --skip_warmup
