#!/usr/bin/env bash
set -euo pipefail

###########################
# 配置区：只改这里就行
###########################

# 支持三种写法：
#   1) MAP_NAME="abtr_ref.pcd"
#   2) MAP_NAME="abtr_ref.pgm"   # 会自动找 abtr_ref.yaml
#   3) MAP_NAME="abtr_ref.yaml"
MAP_NAME="abtr_f.pgm"

# 仅 PCD 模式有效：体素下采样尺寸（米）；<=0 则跳过下采样
VOXEL_LEAF_SIZE=0.05

# 是否启用反光板可视化/编辑
ENABLE_REFLECTOR_EDITOR="true"

# Publish Point 点击模式：
#   0 -> 新增反光板
#   1 -> 删除最近反光板
CLICK_MODE=1

# 当 CLICK_MODE=1 时，删除最近点的最大允许距离（米）
CLICK_DELETE_RADIUS=0.20

# 是否自动保存 JSON
REFLECTOR_AUTO_SAVE="true"

# 反光板显示大小
REFLECTOR_MARKER_SCALE=0.18
REFLECTOR_TEXT_SCALE=0.12

# 仅 PCD 模式有效：地图点云发布持续时间（秒），到时间后自动关闭 pcd_to_pointcloud
MAP_PUBLISH_DURATION=3

# 仅 PCD 模式有效：每次发布的时间间隔（秒）
PCD_PUBLISH_INTERVAL=1.0

# 地图坐标系
MAP_FRAME_ID="map"

# 自己启动 map_server 后，等待 /map 消息的超时时间（秒）
MAP_WAIT_TIMEOUT=10

###########################
# 自动推导目录
###########################
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAP_DIR="${BASE_DIR}/map"

RVIZ_CONFIG="${MAP_DIR}/relocalization.rviz"
REFLECTOR_EDITOR_SCRIPT="${BASE_DIR}/reflector_editor.py"

MAP_TOPIC="/map"
MAP_PUBLISH_PID=""
EDITOR_PID=""
MAP_SERVER_LOG="/tmp/relocalization_map_server_$$.log"

cleanup() {
  set +e

  if [[ -n "${EDITOR_PID}" ]] && ps -p "${EDITOR_PID}" >/dev/null 2>&1; then
    echo "[INFO] 关闭 reflector_editor (PID=${EDITOR_PID})"
    kill "${EDITOR_PID}" 2>/dev/null || true
  fi

  if [[ -n "${MAP_PUBLISH_PID}" ]] && ps -p "${MAP_PUBLISH_PID}" >/dev/null 2>&1; then
    echo "[INFO] 关闭地图发布节点 (PID=${MAP_PUBLISH_PID})"
    kill "${MAP_PUBLISH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# 如果想在脚本里顺便 source 自己的 workspace，可以打开这一行：
# source "${BASE_DIR}/devel/setup.bash"

###########################
# 工具函数
###########################
wait_for_topic_message() {
  local topic_name="$1"
  local timeout_sec="${2:-10}"
  local end_time=$((SECONDS + timeout_sec))

  echo "[INFO] 等待 ${topic_name} 有消息..."
  while (( SECONDS < end_time )); do
    if rostopic echo -n 1 "${topic_name}" >/dev/null 2>&1; then
      echo "[INFO] ${topic_name} 已有消息"
      return 0
    fi
    sleep 0.2
  done

  echo "[ERROR] 等待 ${topic_name} 消息超时（${timeout_sec}s）"
  return 1
}

find_real_map_server_exec() {
  local candidates=()
  local p=""

  # 优先标准 ROS 安装路径
  if [[ -n "${ROS_DISTRO:-}" ]]; then
    candidates+=("/opt/ros/${ROS_DISTRO}/lib/map_server/map_server")
  fi

  # 再尝试 /opt/ros 下其他版本
  for p in /opt/ros/*/lib/map_server/map_server; do
    candidates+=("${p}")
  done

  # 再尝试 CMAKE_PREFIX_PATH
  IFS=':' read -r -a _prefixes <<< "${CMAKE_PREFIX_PATH:-}"
  for p in "${_prefixes[@]}"; do
    [[ -z "${p}" ]] && continue
    candidates+=("${p}/lib/map_server/map_server")
  done

  local cand=""
  for cand in "${candidates[@]}"; do
    [[ -n "${cand}" ]] || continue
    [[ -x "${cand}" ]] || continue
    echo "${cand}"
    return 0
  done

  return 1
}

start_system_map_server() {
  local map_server_bin="$1"
  local yaml_path="$2"
  local frame_id="$3"

  echo "[INFO] 使用系统 map_server 可执行文件: ${map_server_bin}"
  echo "[INFO] map_server 日志: ${MAP_SERVER_LOG}"

  : > "${MAP_SERVER_LOG}"
  "${map_server_bin}" "${yaml_path}" _frame_id:="${frame_id}" >"${MAP_SERVER_LOG}" 2>&1 &
  MAP_PUBLISH_PID=$!

  sleep 0.5
  if ! ps -p "${MAP_PUBLISH_PID}" >/dev/null 2>&1; then
    echo "[ERROR] map_server 进程启动失败"
    echo "[ERROR] 请查看日志: ${MAP_SERVER_LOG}"
    [[ -f "${MAP_SERVER_LOG}" ]] && tail -n 50 "${MAP_SERVER_LOG}" || true
    return 1
  fi

  wait_for_topic_message "${MAP_TOPIC}" "${MAP_WAIT_TIMEOUT}" || {
    echo "[ERROR] map_server 已启动但 ${MAP_TOPIC} 未就绪"
    echo "[ERROR] 请查看日志: ${MAP_SERVER_LOG}"
    [[ -f "${MAP_SERVER_LOG}" ]] && tail -n 50 "${MAP_SERVER_LOG}" || true
    return 1
  }

  return 0
}

###########################
# 解析地图类型
###########################
MAP_EXT="${MAP_NAME##*.}"
MAP_EXT_LOWER="$(echo "${MAP_EXT}" | tr '[:upper:]' '[:lower:]')"
MAP_STEM="${MAP_NAME%.*}"

MAP_TYPE=""
PCD_PATH=""
PGM_PATH=""
YAML_PATH=""
DOWNSAMPLED_PATH=""
REFLECTOR_JSON_PATH="${MAP_DIR}/${MAP_STEM}.json"

case "${MAP_EXT_LOWER}" in
  pcd)
    MAP_TYPE="pcd"
    PCD_PATH="${MAP_DIR}/${MAP_NAME}"
    DOWNSAMPLED_PATH="${MAP_DIR}/${MAP_STEM}_down.pcd"
    ;;
  pgm)
    MAP_TYPE="pgm"
    PGM_PATH="${MAP_DIR}/${MAP_NAME}"
    YAML_PATH="${MAP_DIR}/${MAP_STEM}.yaml"
    ;;
  yaml|yml)
    MAP_TYPE="pgm"
    YAML_PATH="${MAP_DIR}/${MAP_NAME}"
    ;;
  *)
    echo "[ERROR] 不支持的地图格式: ${MAP_NAME}"
    echo "        当前仅支持 .pcd / .pgm / .yaml / .yml"
    exit 1
    ;;
esac

###########################
# 简单检查
###########################
if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "[ERROR] RViz 配置文件不存在: ${RVIZ_CONFIG}"
  exit 1
fi

if [[ "${MAP_TYPE}" == "pcd" ]]; then
  if [[ ! -f "${PCD_PATH}" ]]; then
    echo "[ERROR] 原始 PCD 地图不存在: ${PCD_PATH}"
    exit 1
  fi

  if awk -v v="${VOXEL_LEAF_SIZE}" 'BEGIN{exit !(v>0)}'; then
    if ! command -v pcl_voxel_grid >/dev/null 2>&1; then
      echo "[ERROR] 未找到 pcl_voxel_grid，请先安装 pcl-tools，例如："
      echo "        sudo apt-get install pcl-tools"
      exit 1
    fi
  fi
else
  if [[ -z "${YAML_PATH}" ]]; then
    echo "[ERROR] YAML 路径解析失败"
    exit 1
  fi

  if [[ ! -f "${YAML_PATH}" ]]; then
    echo "[ERROR] 地图 YAML 不存在: ${YAML_PATH}"
    echo "        注意：PGM 在 ROS 中需要配套同名 YAML 才能被 map_server 加载。"
    exit 1
  fi
fi

if [[ "${ENABLE_REFLECTOR_EDITOR}" == "true" ]]; then
  if [[ ! -f "${REFLECTOR_JSON_PATH}" ]]; then
    echo "[ERROR] 反光板 JSON 不存在: ${REFLECTOR_JSON_PATH}"
    exit 1
  fi

  if [[ ! -f "${REFLECTOR_EDITOR_SCRIPT}" ]]; then
    echo "[ERROR] 反光板编辑脚本不存在: ${REFLECTOR_EDITOR_SCRIPT}"
    exit 1
  fi

  if ! command -v python3 >/dev/null 2>&1; then
    echo "[ERROR] 未找到 python3"
    exit 1
  fi
fi

###########################
# 1. 启动地图发布
###########################
USE_SIM_TIME="$(rosparam get /use_sim_time 2>/dev/null || echo "false")"
if [[ "${USE_SIM_TIME}" == "true" ]]; then
  echo "[INFO] 检测到 use_sim_time=true，等待 /clock..."
  for i in $(seq 1 50); do
    if rostopic echo -n 1 /clock >/dev/null 2>&1; then
      echo "[INFO] /clock 已就绪"
      break
    fi
    sleep 0.1
  done
fi

if [[ "${MAP_TYPE}" == "pcd" ]]; then
  echo "[INFO] 地图类型: PCD"
  echo "[INFO] 原始地图: ${PCD_PATH}"
  echo "[INFO] 对应 JSON: ${REFLECTOR_JSON_PATH}"
  echo "[INFO] 体素尺寸: ${VOXEL_LEAF_SIZE} m"

  if awk -v v="${VOXEL_LEAF_SIZE}" 'BEGIN{exit !(v<=0)}'; then
    echo "[INFO] VOXEL_LEAF_SIZE<=0，跳过下采样，直接使用原始地图发布。"
    DOWNSAMPLED_PATH="${PCD_PATH}"
  else
    echo "[INFO] 下采样后地图: ${DOWNSAMPLED_PATH}"
    if [[ -f "${DOWNSAMPLED_PATH}" ]]; then
      echo "[INFO] 检测到已存在下采样地图，将覆盖重新生成..."
    fi

    pcl_voxel_grid "${PCD_PATH}" "${DOWNSAMPLED_PATH}" -leaf \
      "${VOXEL_LEAF_SIZE},${VOXEL_LEAF_SIZE},${VOXEL_LEAF_SIZE}"

    echo "[INFO] 下采样完成。"
  fi

  echo "[INFO] 使用地图发布: ${DOWNSAMPLED_PATH}"
  echo "[INFO] 地图发布间隔: ${PCD_PUBLISH_INTERVAL}s, 持续约 ${MAP_PUBLISH_DURATION}s"

  rosrun pcl_ros pcd_to_pointcloud "${DOWNSAMPLED_PATH}" "${PCD_PUBLISH_INTERVAL}" _frame_id:="${MAP_FRAME_ID}" &
  MAP_PUBLISH_PID=$!

  TOPIC="/cloud_pcd"
  (
    for i in $(seq 1 100); do
      if rostopic info "${TOPIC}" 2>/dev/null | grep -q "Subscribers: None"; then
        sleep 0.1
        continue
      fi
      if rostopic info "${TOPIC}" 2>/dev/null | grep -q "Subscribers:"; then
        echo "[INFO] 检测到 ${TOPIC} 有订阅者，开始计时 ${MAP_PUBLISH_DURATION}s 后关闭地图发布..."
        break
      fi
      sleep 0.1
    done

    sleep "${MAP_PUBLISH_DURATION}"
    if ps -p "${MAP_PUBLISH_PID}" >/dev/null 2>&1; then
      echo
      echo "[INFO] 已到地图发布持续时间 ${MAP_PUBLISH_DURATION}s，自动关闭 pcd_to_pointcloud (PID=${MAP_PUBLISH_PID})..."
      kill "${MAP_PUBLISH_PID}" 2>/dev/null || true
    fi
  ) &

else
  echo "[INFO] 地图类型: PGM/YAML"
  echo "[INFO] 地图 YAML: ${YAML_PATH}"
  if [[ -n "${PGM_PATH}" ]]; then
    echo "[INFO] 地图 PGM : ${PGM_PATH}"
  fi

  MAP_SERVER_BIN="$(find_real_map_server_exec || true)"
  if [[ -z "${MAP_SERVER_BIN}" ]]; then
    echo "[ERROR] 未找到真正可执行的系统 map_server"
    echo "[ERROR] 很可能是工作空间内存在同名包 map_server，导致 rosrun/rospack 命中源码包。"
    if [[ -n "${ROS_DISTRO:-}" ]]; then
      echo "[ERROR] 期望路径例如：/opt/ros/${ROS_DISTRO}/lib/map_server/map_server"
    fi
    exit 1
  fi

  start_system_map_server "${MAP_SERVER_BIN}" "${YAML_PATH}" "${MAP_FRAME_ID}" || exit 1
fi

###########################
# 2. 启动反光板编辑节点
###########################
if [[ "${ENABLE_REFLECTOR_EDITOR}" == "true" ]]; then
  echo "[INFO] 启动反光板编辑器..."
  echo "[INFO] JSON: ${REFLECTOR_JSON_PATH}"
  echo "[INFO] CLICK_MODE: ${CLICK_MODE}"
  echo "[INFO] CLICK_DELETE_RADIUS: ${CLICK_DELETE_RADIUS}"

  python3 "${REFLECTOR_EDITOR_SCRIPT}" \
    _json_path:="${REFLECTOR_JSON_PATH}" \
    _frame_id:="${MAP_FRAME_ID}" \
    _marker_scale:="${REFLECTOR_MARKER_SCALE}" \
    _text_scale:="${REFLECTOR_TEXT_SCALE}" \
    _auto_save:="${REFLECTOR_AUTO_SAVE}" \
    _click_mode:="${CLICK_MODE}" \
    _click_delete_radius:="${CLICK_DELETE_RADIUS}" &
  EDITOR_PID=$!

  sleep 0.5
fi

###########################
# 3. 启动 RViz
###########################
echo "[INFO] 启动 RViz（关闭 RViz 后脚本会自动退出）..."
echo "[INFO] 使用 RViz 配置: ${RVIZ_CONFIG}"

if [[ "${MAP_TYPE}" == "pcd" ]]; then
  echo "[INFO] RViz 中请查看 PointCloud2 话题: /cloud_pcd"
else
  echo "[INFO] RViz 中请查看 Map 话题: ${MAP_TOPIC}"
fi

rosrun rviz rviz -d "${RVIZ_CONFIG}"

echo "[INFO] RViz 已退出，脚本结束。"
