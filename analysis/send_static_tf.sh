#!/usr/bin/env bash
set -euo pipefail

############################
# 配置区：允许留空
############################
HW_CONFIG="/home/wr/localization_alg-stable-end/src/localization_ndt/analysis/hw_config.json"        # 空 -> 用 python 默认 /opt/factory/hw_config.json
BASE_FRAME="base_footprint"       # 空 -> 用 python 默认 base_footprint
PY_SCRIPT="/home/wr/localization_alg-stable-end/src/localization_ndt/analysis/send_static_tf.py"
############################

if [[ ! -f "$PY_SCRIPT" ]]; then
  echo "[ERROR] PY_SCRIPT not found: $PY_SCRIPT" >&2
  exit 2
fi

cmd=(python3 "$PY_SCRIPT")

if [[ -n "$HW_CONFIG" ]]; then
  if [[ ! -f "$HW_CONFIG" ]]; then
    echo "[ERROR] HW_CONFIG not found: $HW_CONFIG" >&2
    exit 2
  fi
  cmd+=(--config "$HW_CONFIG")
else
  echo "[INFO] HW_CONFIG empty -> use default /opt/factory/hw_config.json"
fi

if [[ -n "$BASE_FRAME" ]]; then
  cmd+=(--base_frame "$BASE_FRAME")
else
  echo "[INFO] BASE_FRAME empty -> use default base_footprint"
fi

echo "[INFO] Run: ${cmd[*]}"
exec "${cmd[@]}"
