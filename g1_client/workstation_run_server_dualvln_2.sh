#!/usr/bin/env bash
set -euo pipefail

cd /home/ubuntu/InternNav
source /home/ubuntu/miniconda3/etc/profile.d/conda.sh
conda activate Intern

G1_LOG_SYNC="${G1_LOG_SYNC:-1}"
G1_LOG_REMOTE="${G1_LOG_REMOTE:-unitree@192.168.0.225}"
G1_LOG_SOURCE="${G1_LOG_SOURCE:-/home/unitree/Intern_g1/g1_client/logs/}"
G1_LOG_DEST="${G1_LOG_DEST:-/home/ubuntu/InternNav/g1_logs_from_g1}"
G1_LOG_SSH_OPTS="${G1_LOG_SSH_OPTS:--o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new}"

sync_g1_logs() {
  if [ "$G1_LOG_SYNC" = "0" ]; then
    echo "[G1 LOG] Sync disabled by G1_LOG_SYNC=0."
    return 0
  fi

  mkdir -p "$G1_LOG_DEST"
  echo "[G1 LOG] Syncing G1 logs from ${G1_LOG_REMOTE}:${G1_LOG_SOURCE} to ${G1_LOG_DEST}/"

  if command -v rsync >/dev/null 2>&1; then
    if rsync -avz -e "ssh ${G1_LOG_SSH_OPTS}" "${G1_LOG_REMOTE}:${G1_LOG_SOURCE}" "${G1_LOG_DEST}/"; then
      echo "[G1 LOG] rsync complete: ${G1_LOG_DEST}/"
      return 0
    fi
    echo "[G1 LOG][WARN] rsync failed, trying scp fallback."
  fi

  if scp -r ${G1_LOG_SSH_OPTS} "${G1_LOG_REMOTE}:${G1_LOG_SOURCE%/}" "${G1_LOG_DEST}/"; then
    echo "[G1 LOG] scp complete: ${G1_LOG_DEST}/"
    return 0
  fi

  echo "[G1 LOG][WARN] Failed to sync G1 logs. Check G1_LOG_REMOTE, network, and SSH key/password."
  return 0
}

sync_g1_logs_on_exit() {
  status=$?
  sync_g1_logs || true
  exit "$status"
}
trap sync_g1_logs_on_exit EXIT

python scripts/realworld/http_internvla_server.py \
  --device cuda:0 \
  --model_path checkpoints/InternVLA-N1-DualVLN \
  --resize_w 384 \
  --resize_h 384 \
  --num_history 8 \
  --plan_step_gap 8 \
  --skip_warmup
