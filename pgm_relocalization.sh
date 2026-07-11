#!/usr/bin/env bash
set -euo pipefail

###########################
# 配置区：只改这里就行
###########################
MAP_NAME="abtr251229_9.yaml"   # 支持 .pcd/.pgm/.yaml/.yml/不写扩展名

# PCD 模式参数
VOXEL_LEAF_SIZE=0.05
MAP_PUBLISH_DURATION=3
PCD_PUBLISH_INTERVAL=1.0

# PGM(map_server) 模式参数
SLEEP_BEFORE_KILL=5
KILL_MAP_SERVER="true"       # true: rviz 接收后 kill map_server

# RViz 配置文件名（都位于 map/ 目录，和原 relocalization.rviz 同目录）
RVIZ_PCD="pcd.rviz"
RVIZ_PGM="pgm.rviz"

###########################
# 自动推导目录
###########################
BASE_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MAP_DIR="$BASE_DIR/map"

RVIZ_PCD_PATH="$MAP_DIR/$RVIZ_PCD"
RVIZ_PGM_PATH="$MAP_DIR/$RVIZ_PGM"

PCD_PID=""
MAP_SERVER_PID=""

cleanup() {
  [[ -n "${PCD_PID}" ]] && kill "${PCD_PID}" 2>/dev/null || true
  [[ -n "${MAP_SERVER_PID}" ]] && kill "${MAP_SERVER_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

###########################
# 工具函数：解析 MAP_NAME
###########################
resolve_map_file() {
  local name="$1"
  local path=""

  if [[ "$name" == *.* ]]; then
    path="$MAP_DIR/$name"
    [[ -f "$path" ]] && { echo "$path"; return 0; }
    return 1
  fi

  for ext in pcd pgm yaml yml; do
    path="$MAP_DIR/${name}.${ext}"
    [[ -f "$path" ]] && { echo "$path"; return 0; }
  done
  return 1
}

###########################
# PCD 模式：发布 /cloud_pcd，并用 pcd.rviz 打开
###########################
run_pcd_mode() {
  local pcd_path="$1"

  [[ -f "$RVIZ_PCD_PATH" ]] || { echo "RViz 配置不存在: $RVIZ_PCD_PATH"; exit 1; }

  command -v pcl_voxel_grid >/dev/null 2>&1 || {
    echo "未找到 pcl_voxel_grid：sudo apt-get install pcl-tools"
    exit 1
  }

  echo "[PCD] 原始地图: $pcd_path"
  echo "[PCD] 体素尺寸: ${VOXEL_LEAF_SIZE} m"

  local pcd_name down_path
  pcd_name="$(basename "$pcd_path")"
  down_path="$MAP_DIR/${pcd_name%.*}_down.pcd"

  if awk -v v="$VOXEL_LEAF_SIZE" 'BEGIN{exit !(v<=0)}'; then
    echo "[PCD] VOXEL_LEAF_SIZE<=0，跳过下采样"
    down_path="$pcd_path"
  else
    echo "[PCD] 下采样输出: $down_path"
    pcl_voxel_grid "$pcd_path" "$down_path" -leaf \
      ${VOXEL_LEAF_SIZE},${VOXEL_LEAF_SIZE},${VOXEL_LEAF_SIZE}
  fi

  echo "[PCD] 发布 /cloud_pcd，interval=${PCD_PUBLISH_INTERVAL}s，持续约 ${MAP_PUBLISH_DURATION}s"
  rosrun pcl_ros pcd_to_pointcloud "$down_path" "$PCD_PUBLISH_INTERVAL" _frame_id:=map &
  PCD_PID=$!

  # 等订阅者出现后计时关闭（可保留你原逻辑）
  (
    local topic="/cloud_pcd"
    for _ in $(seq 1 100); do
      if rostopic info "$topic" 2>/dev/null | grep -q "Subscribers: None"; then
        sleep 0.1
        continue
      fi
      if rostopic info "$topic" 2>/dev/null | grep -q "Subscribers:"; then
        echo "[PCD] 检测到 $topic 有订阅者，${MAP_PUBLISH_DURATION}s 后自动关闭发布..."
        break
      fi
      sleep 0.1
    done

    sleep "$MAP_PUBLISH_DURATION"
    if ps -p "$PCD_PID" >/dev/null 2>&1; then
      echo "[PCD] 时间到，关闭 pcd_to_pointcloud (PID=$PCD_PID)"
      kill "$PCD_PID" 2>/dev/null || true
    fi
  ) &

  echo "[PCD] 启动 RViz: $RVIZ_PCD_PATH"
  rviz -d "$RVIZ_PCD_PATH"
}

###########################
# PGM/YAML 模式：map_server 发布 /map，并用 pgm.rviz 打开
###########################
run_map_server_mode() {
  local yaml_path="$1"

  [[ -f "$RVIZ_PGM_PATH" ]] || { echo "RViz 配置不存在: $RVIZ_PGM_PATH"; exit 1; }

  rospack find map_server >/dev/null 2>&1 || {
    echo "未找到 map_server：sudo apt-get install ros-\$ROS_DISTRO-map-server"
    exit 1
  }

  echo "[PGM] 启动 map_server: $yaml_path"
  rosrun map_server map_server "$yaml_path" __name:=map_server_rviz _frame_id:=map &
  MAP_SERVER_PID=$!

  echo "[PGM] 等待 /map..."
  for _ in $(seq 1 50); do
    rostopic echo -n 1 /map >/dev/null 2>&1 && break || true
    sleep 0.1
  done

  echo "[PGM] 启动 RViz: $RVIZ_PGM_PATH"
  rviz -d "$RVIZ_PGM_PATH" &
  RVIZ_PID=$!

  echo "[PGM] 给 RViz ${SLEEP_BEFORE_KILL}s 接收地图..."
  sleep "$SLEEP_BEFORE_KILL"

  if [[ "$KILL_MAP_SERVER" == "true" ]]; then
    echo "[PGM] kill map_server (pid=$MAP_SERVER_PID)，RViz 会保留最后一帧地图"
    kill "$MAP_SERVER_PID" 2>/dev/null || true
    MAP_SERVER_PID=""
  fi

  wait "$RVIZ_PID" || true
}

###########################
# 主流程
###########################
MAP_PATH="$(resolve_map_file "$MAP_NAME" || true)"
if [[ -z "$MAP_PATH" ]]; then
  echo "未找到地图文件：$MAP_DIR/$MAP_NAME (或同名 .pcd/.pgm/.yaml/.yml)"
  exit 1
fi

EXT="${MAP_PATH##*.}"
EXT="$(echo "$EXT" | tr '[:upper:]' '[:lower:]')"
echo "[INFO] MAP_PATH=$MAP_PATH (.$EXT)"

case "$EXT" in
  pcd)
    run_pcd_mode "$MAP_PATH"
    ;;
  yaml|yml)
    run_map_server_mode "$MAP_PATH"
    ;;
  pgm)
    base="${MAP_PATH%.*}"
    yaml_path="${base}.yaml"
    [[ -f "$yaml_path" ]] || yaml_path="${base}.yml"
    [[ -f "$yaml_path" ]] || { echo "[PGM] 未找到同名 yaml/yml：${base}.yaml 或 ${base}.yml"; exit 1; }
    run_map_server_mode "$yaml_path"
    ;;
  *)
    echo "不支持的地图类型: .$EXT"
    exit 1
    ;;
esac

