#include <Eigen/Dense>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <condition_variable>
#include <ctime>
#include <fstream>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <string>
#include <thread>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_set>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "localization_ndt/config/yaml_utils.hpp"
#include "localization_ndt/initial_pose_manager.hpp"
#include "localization_ndt/local_relocalizer_2d.hpp"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_amcl/map/map.hpp"
#include "nav2_amcl/motion_model/differential_motion_model.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_amcl/pf/pf_vector.hpp"
#include "nav2_amcl/portable_utils.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"

namespace localization_ndt {
namespace {

constexpr double kPi = 3.14159265358979323846;

inline double NormalizeAngle(double a) {
  while (a > kPi) {
    a -= 2.0 * kPi;
  }
  while (a < -kPi) {
    a += 2.0 * kPi;
  }
  return a;
}

inline double Deg2Rad(double deg) { return deg * kPi / 180.0; }

inline double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

inline double HuberUnit(double x) {
  const double ax = std::fabs(x);
  if (ax <= 1.0) {
    return 0.5 * ax * ax;
  }
  return ax - 0.5;
}

inline bool FileExists(const std::string &path) {
  std::ifstream ifs(path.c_str(), std::ios::binary);
  return ifs.good();
}

inline std::string Dirname(const std::string &path) {
  const std::size_t pos = path.find_last_of("/\\");
  if (pos == std::string::npos) {
    return ".";
  }
  if (pos == 0) {
    return "/";
  }
  return path.substr(0, pos);
}

inline std::string JoinPath(const std::string &dir, const std::string &name) {
  if (dir.empty()) {
    return name;
  }
  if (!name.empty() && name[0] == '/') {
    return name;
  }
  if (dir.back() == '/') {
    return dir + name;
  }
  return dir + "/" + name;
}

inline std::string MakeDefaultInterfaceYamlText() {
  return std::string("scan_topic: /scan\n"
                     "odom_topic: /odom/wheel\n"
                     "imu_topic: /imu\n"
                     "base_frame_id: base_footprint\n");
}

inline std::string MakeDefaultAmclParamsYamlText(const std::string &map_name,
                                                 bool publish_map_to_odom_tf) {
  std::ostringstream oss;
  oss << "# ============================================================\n"
      << "# AMCL 定位参数（自动生成模板）\n"
      << "# - 对应地图: " << map_name << "\n"
      << "# - 这里只放 AMCL 算法参数\n"
      << "# - 话题 / frame 配置见 config/interface/interface.yaml\n"
      << "# ============================================================\n"
      << "\n"
      << "# ---------- 1. 发布相关 ----------\n"
      << "publish_tf: true\n"
      << "publish_map: true\n"
      << "publish_particlecloud: false\n"
      << "publish_map_to_odom_tf: "
      << (publish_map_to_odom_tf ? "true" : "false") << "\n"
      << "particlecloud_max_samples: 200\n"
      << "use_yaml_frame_id: false\n"
      << "\n"
      << "# ---------- 2. 启动与初始化 ----------\n"
      << "# true: 启动后直接做全局撒点；false: 依赖 shutdown/last/fixed/use_initial_pose\n"
      << "# 大场景建议默认关闭，只作为最后兜底选项\n"
      << "global_init_on_startup: false\n"
      << "# true: 使用下面的 initial_pose_* 作为第一个初始化种子\n"
      << "use_initial_pose: false\n"
      << "initial_pose_x: 0.0\n"
      << "initial_pose_y: 0.0\n"
      << "initial_pose_yaw_deg: 0.0\n"
      << "\n"
      << "# 初始化种子验证：连续多少帧判定成功 / 失败\n"
      << "init_validate_good_frames: 2\n"
      << "init_validate_bad_frames: 3\n"
      << "\n"
      << "# ---------- 3. 扫描预处理 ----------\n"
      << "scan_min_range: 0.5\n"
      << "scan_max_range: 30.0\n"
      << "scan_voxel_m: 0.03\n"
      << "max_scan_points: 1500\n"
      << "\n"
      << "# ---------- 4. 激光模型 ----------\n"
      << "# 可选: likelihood_field / likelihood_field_prob\n"
      << "laser_model_type: likelihood_field\n"
      << "max_beams: 40\n"
      << "laser_min_range: 0.0\n"
      << "laser_max_range: 35.0\n"
      << "sigma_hit: 0.20\n"
      << "laser_likelihood_max_dist: 2.0\n"
      << "z_hit: 0.95\n"
      << "z_rand: 0.05\n"
      << "do_beamskip: false\n"
      << "beam_skip_distance: 0.5\n"
      << "beam_skip_threshold: 0.3\n"
      << "beam_skip_error_threshold: 0.9\n"
      << "\n"
      << "# ---------- 5. 运动模型 / 粒子滤波 ----------\n"
      << "alpha1: 0.2\n"
      << "alpha2: 0.2\n"
      << "alpha3: 0.2\n"
      << "alpha4: 0.2\n"
      << "alpha5: 0.2\n"
      << "min_particles: 200\n"
      << "max_particles: 800\n"
      << "pf_err: 0.05\n"
      << "pf_z: 0.99\n"
      << "recovery_alpha_fast: 0.0\n"
      << "recovery_alpha_slow: 0.0\n"
      << "resample_interval: 1\n"
      << "update_min_d: 0.05\n"
      << "update_min_a_deg: 2.0\n"
      << "\n"
      << "# ---------- 6. 初始协方差 ----------\n"
      << "init_cov_xx: 0.25\n"
      << "init_cov_yy: 0.25\n"
      << "init_cov_aa: 0.068538919452\n"
      << "\n"
      << "# ---------- 7. 局部重定位公共参数 ----------\n"
      << "# 以下参数由 manual / startup / lost_local / lost_fixed 共用\n"
      << "local_reloc_min_score: 0.20\n"
      << "local_reloc_score_margin: 0.00\n"
      << "local_reloc_min_valid_fraction: 0.30\n"
      << "local_reloc_bnb_max_level: 3\n"
      << "local_reloc_pyr_max_level: 6\n"
      << "local_reloc_max_scan_points: 1200\n"
      << "local_reloc_scan_voxel_m: 0.02\n"
      << "local_reloc_hit_sigma_m: 0.20\n"
      << "local_reloc_max_dist_m: 1.00\n"
      << "local_reloc_min_range: 0.0\n"
      << "local_reloc_max_range: 0.0\n"
      << "local_reloc_yaw_step_deg: 1.0\n"
      << "# 统一搜索窗口：manual / startup / lost 都使用这一组\n"
      << "# local_reloc_yaw_search_deg 为“半窗口角度”，180 表示全周搜索\n"
      << "local_reloc_xy_range_m: 5.0\n"
      << "local_reloc_yaw_search_deg: 180.0\n"
      << "# local_reloc 成功后，额外强制做多少帧真实 scan 的 sensor update（即使车没动）\n"
      << "local_reloc_settle_frames: 3\n"
      << "\n"
      << "# ---------- 8. 手动局部重定位 ----------\n"
      << "manual_local_reloc_enable: true\n"
      << "# 先用固定大窗 coarse 搜索，再用下面的协方差窗 fine 精修\n"
      << "manual_local_reloc_coarse_xy_range_m: 5.0\n"
      << "manual_local_reloc_coarse_yaw_search_deg: 180.0\n"
      << "# 手动拖拽保留旧版语义：根据 initialpose 协方差自适应搜索窗口\n"
      << "manual_local_reloc_xy_min_m: 0.30\n"
      << "manual_local_reloc_xy_sigma_k: 3.0\n"
      << "manual_local_reloc_xy_max_m: 5.0\n"
      << "manual_local_reloc_yaw_min_deg: 6.0\n"
      << "manual_local_reloc_yaw_sigma_k: 3.0\n"
      << "# coarse/fine 成功后，再用更小窗口做一次 final refine\n"
      << "manual_local_reloc_final_xy_range_m: 0.50\n"
      << "manual_local_reloc_final_yaw_search_deg: 10.0\n"
      << "manual_local_reloc_final_yaw_step_deg: 0.25\n"
      << "# 手动局部重定位成功后，用较紧的协方差重置 PF，避免结果正确但输出仍较散\n"
      << "manual_local_reloc_reseed_sigma_scale: 0.05\n"
      << "manual_local_reloc_reseed_xy_min_m: 0.05\n"
      << "manual_local_reloc_reseed_yaw_min_deg: 2.0\n"
      << "\n"
      << "# ---------- 9. 启动自动局部重定位 ----------\n"
      << "# 对 shutdown / last / fixed 这类启动 seed 先在局部窗口内精修一次\n"
      << "startup_local_reloc_enable: true\n"
      << "# fixed 候选批量尝试：每帧最多尝试多少个 fixed 候选\n"
      << "startup_max_candidates_per_scan: 3\n"
      << "# fixed 全量遍历失败后，停稳多久并每隔多久重新开启一次 fixed 遍历周期\n"
      << "startup_fixed_retry_interval_sec: 2.0\n"
      << "startup_fixed_retry_stationary_min_duration: 2.0\n"
      << "\n"
      << "# ---------- 10. LOST 自动局部重定位 ----------\n"
      << "# 围绕 last_good + odom 推进后的 DR 中心做局部重定位\n"
      << "lost_local_reloc_enable: true\n"
      << "lost_local_reloc_min_interval_sec: 2.0\n"
      << "lost_local_reloc_expand_factor: 1.5\n"
      << "lost_local_reloc_max_expansions: 1\n"
      << "\n"
      << "# ---------- 11. LOST_FIXED 批量恢复 ----------\n"
      << "# LOST_LOCAL 扩窗失败后，停稳再按 fixed 列表做批量恢复\n"
      << "lost_fixed_reloc_enable: true\n"
      << "lost_fixed_min_interval_sec: 2.0\n"
      << "lost_fixed_max_candidates_per_scan: 3\n"
      << "lost_fixed_stationary_min_duration: 2.0\n"
      << "\n"
      << "# ---------- 12. 自动局部重定位成功后，用于收紧 PF 的重置协方差 ----------\n"
      << "auto_local_reloc_reseed_sigma_scale: 0.25\n"
      << "auto_local_reloc_reseed_xy_min_m: 0.05\n"
      << "auto_local_reloc_reseed_yaw_min_deg: 2.0\n"
      << "\n"
      << "# ---------- 13. 状态机：GOOD 判定 ----------\n"
      << "state_good_best_min: 0.98\n"
      << "state_good_ambiguity_ratio_max: 0.10\n"
      << "state_good_max_clusters: 1\n"
      << "state_good_overall_sigma_xy_max: 0.60\n"
      << "state_good_best_sigma_yaw_deg_max: 15.0\n"
      << "state_good_jump_trans_max: 0.10\n"
      << "state_good_jump_yaw_deg_max: 1.0\n"
      << "\n"
      << "# ---------- 14. 状态机：AMBIGUOUS / LOST 判定 ----------\n"
      << "state_ambiguous_min_clusters: 2\n"
      << "state_ambiguous_best_min: 0.20\n"
      << "state_ambiguous_ratio_min: 0.35\n"
      << "state_lost_best_max: 0.20\n"
      << "state_lost_cluster_min: 20\n"
      << "state_lost_overall_sigma_xy_min: 2.0\n"
      << "\n"
      << "# ---------- 15. 状态机滞回 ----------\n"
      << "state_good_enter_frames: 2\n"
      << "state_weak_enter_frames: 2\n"
      << "state_ambiguous_enter_frames: 2\n"
      << "state_lost_enter_frames: 2\n"
      << "\n"
      << "# ---------- 16. last 位姿写回策略 ----------\n"
      << "# 仅在 GOOD + 静止持续一段时间后写回 last.yaml\n"
      << "last_pose_stationary_linear_speed_max: 0.05\n"
      << "last_pose_stationary_angular_speed_deg_max: 3.0\n"
      << "last_pose_stationary_min_duration: 2.0\n"
      << "\n"
      << "# ---------- 17. 运行辅助 ----------\n"
      << "odom_history_max_age_sec: 2.0\n"
      << "odom_history_max_size: 400\n"
      << "tf_lookup_timeout_sec: 0.02\n";
  return oss.str();
}

inline YAML::Node LoadYamlFileOrEmpty(const std::string &path, const char *tag) {
  try {
    return YAML::LoadFile(path);
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(tag << " load failed: " << path << " err=" << e.what());
    return YAML::Node();
  }
}

inline bool LoadOrCreateInterfaceYaml(const std::string &path, YAML::Node *out) {
  if (!out) {
    return false;
  }
  if (!FileExists(path)) {
    if (!localization_ndt::config::SaveTextToFile(path, MakeDefaultInterfaceYamlText())) {
      ROS_WARN_STREAM("[AMCL] failed to create interface yaml: " << path);
    } else {
      ROS_WARN_STREAM("[AMCL] created interface yaml template: " << path);
    }
  }
  *out = LoadYamlFileOrEmpty(path, "[AMCL] interface yaml");
  return true;
}

inline bool LoadOrCreateAmclParamsYaml(const std::string &path,
                                       const std::string &map_name,
                                       bool publish_map_to_odom_tf_default,
                                       YAML::Node *out) {
  if (!out) {
    return false;
  }
  if (!FileExists(path)) {
    const std::string text =
        MakeDefaultAmclParamsYamlText(map_name, publish_map_to_odom_tf_default);
    if (!localization_ndt::config::SaveTextToFile(path, text)) {
      ROS_WARN_STREAM("[AMCL] failed to create params yaml: " << path);
    } else {
      ROS_WARN_STREAM("[AMCL] created params yaml template: " << path);
    }
  }
  *out = LoadYamlFileOrEmpty(path, "[AMCL] params yaml");
  return true;
}

inline bool ResolveAmclRunMapSelection(const std::string &run_yaml_path,
                                       std::string *out_map,
                                       std::string *out_err) {
  if (out_map) {
    out_map->clear();
  }
  if (out_err) {
    out_err->clear();
  }

  if (!FileExists(run_yaml_path)) {
    if (out_err) {
      *out_err = "run.yaml not found: " + run_yaml_path;
    }
    return false;
  }

  YAML::Node root = LoadYamlFileOrEmpty(run_yaml_path, "[AMCL] run.yaml");
  if (!root || !root.IsMap()) {
    if (out_err) {
      *out_err = "run.yaml invalid or empty: " + run_yaml_path;
    }
    return false;
  }

  int run_target = 0;
  localization_ndt::config::LoadFieldFromYaml(root, "run_target", run_target);

  std::string pending_map;
  localization_ndt::config::LoadFieldFromYaml(root, "map", pending_map);
  pending_map = localization_ndt::config::TrimCopy(pending_map);

  const YAML::Node list = root["--启动列表--"];
  if (run_target > 0 && list && list.IsSequence()) {
    for (std::size_t i = 0; i < list.size(); ++i) {
      const YAML::Node item = list[i];
      if (!item || !item.IsMap()) {
        continue;
      }
      int id = 0;
      std::string map_name;
      localization_ndt::config::LoadFieldFromYaml(item, "id", id);
      localization_ndt::config::LoadFieldFromYaml(item, "map", map_name);
      map_name = localization_ndt::config::TrimCopy(map_name);
      if (id == run_target && !map_name.empty()) {
        if (out_map) {
          *out_map = map_name;
        }
        return true;
      }
    }

    if (out_err) {
      std::ostringstream oss;
      oss << "run.yaml: run_target=" << run_target
          << " not found or missing map.";
      *out_err = oss.str();
    }
    return false;
  }

  if (!pending_map.empty()) {
    if (out_map) {
      *out_map = pending_map;
    }
    return true;
  }

  if (out_err) {
    *out_err = "run.yaml has no usable map selection.";
  }
  return false;
}

inline geometry_msgs::Quaternion YawToQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

inline double QuaternionToYaw(const geometry_msgs::Quaternion &q_msg) {
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  q.normalize();
  return NormalizeAngle(std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                   1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())));
}

inline double SafeSqrt(double v) { return std::sqrt(std::max(0.0, v)); }

inline std::string FmtD(double v, int prec = 3) {
  if (!std::isfinite(v)) {
    return "nan";
  }
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss.precision(prec);
  oss << v;
  return oss.str();
}

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct LaserMount {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct PgmImage {
  int width = 0;
  int height = 0;
  int maxval = 255;
  std::vector<uint8_t> data;
};

bool ReadToken(std::istream &is, std::string *out) {
  if (!out) {
    return false;
  }
  out->clear();

  while (true) {
    const int c = is.peek();
    if (c == EOF) {
      return false;
    }
    if (std::isspace(c)) {
      is.get();
      continue;
    }
    if (c == '#') {
      std::string line;
      std::getline(is, line);
      continue;
    }
    break;
  }

  while (true) {
    const int c = is.peek();
    if (c == EOF || std::isspace(c) || c == '#') {
      break;
    }
    out->push_back(static_cast<char>(is.get()));
  }
  return !out->empty();
}

bool LoadPgm(const std::string &path, PgmImage *img, std::string *err) {
  if (!img) {
    return false;
  }

  std::ifstream ifs(path.c_str(), std::ios::binary);
  if (!ifs.is_open()) {
    if (err) {
      *err = "open pgm failed: " + path;
    }
    return false;
  }

  std::string magic;
  if (!ReadToken(ifs, &magic)) {
    if (err) {
      *err = "read pgm magic failed";
    }
    return false;
  }
  if (magic != "P5" && magic != "P2") {
    if (err) {
      *err = "unsupported pgm magic: " + magic;
    }
    return false;
  }

  std::string tok;
  if (!ReadToken(ifs, &tok)) {
    return false;
  }
  img->width = std::stoi(tok);
  if (!ReadToken(ifs, &tok)) {
    return false;
  }
  img->height = std::stoi(tok);
  if (!ReadToken(ifs, &tok)) {
    return false;
  }
  img->maxval = std::stoi(tok);

  if (img->width <= 0 || img->height <= 0 || img->maxval <= 0) {
    if (err) {
      *err = "invalid pgm header";
    }
    return false;
  }

  img->data.assign(static_cast<std::size_t>(img->width) *
                       static_cast<std::size_t>(img->height),
                   0);

  if (magic == "P5") {
    ifs.get();
    if (img->maxval > 255) {
      if (err) {
        *err = "16-bit pgm not supported";
      }
      return false;
    }
    ifs.read(reinterpret_cast<char *>(img->data.data()),
             static_cast<std::streamsize>(img->data.size()));
    if (!ifs.good()) {
      if (err) {
        *err = "pgm payload shorter than expected";
      }
      return false;
    }
  } else {
    for (int i = 0; i < img->width * img->height; ++i) {
      if (!ReadToken(ifs, &tok)) {
        if (err) {
          *err = "pgm ascii payload shorter than expected";
        }
        return false;
      }
      int v = std::stoi(tok);
      v = std::max(0, std::min(v, img->maxval));
      img->data[static_cast<std::size_t>(i)] =
          static_cast<uint8_t>(std::lround(255.0 * double(v) / img->maxval));
    }
  }

  return true;
}

class OccupancyMap {
public:
  bool loadFromYaml(const std::string &map_yaml, std::string *err) {
    ready_ = false;
    yaml_path_ = map_yaml;
    known_.clear();
    occ_.clear();

    YAML::Node y;
    try {
      y = YAML::LoadFile(map_yaml);
    } catch (const std::exception &e) {
      if (err) {
        *err = std::string("yaml load failed: ") + e.what();
      }
      return false;
    }

    if (!y["image"] || !y["resolution"] || !y["origin"] ||
        !y["origin"].IsSequence() || y["origin"].size() < 3) {
      if (err) {
        *err = "yaml missing image/resolution/origin";
      }
      return false;
    }

    const std::string image = y["image"].as<std::string>();
    res_ = y["resolution"].as<double>();
    if (!(res_ > 0.0)) {
      if (err) {
        *err = "resolution must be positive";
      }
      return false;
    }

    ox_ = y["origin"][0].as<double>();
    oy_ = y["origin"][1].as<double>();
    oyaw_ = NormalizeAngle(y["origin"][2].as<double>());

    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    if (y["negate"]) {
      negate = y["negate"].as<int>();
    }
    if (y["occupied_thresh"]) {
      occupied_thresh = y["occupied_thresh"].as<double>();
    }
    if (y["free_thresh"]) {
      free_thresh = y["free_thresh"].as<double>();
    }

    yaml_frame_id_ = "map";
    if (y["frame_id"]) {
      try {
        yaml_frame_id_ = y["frame_id"].as<std::string>();
      } catch (...) {
        yaml_frame_id_ = "map";
      }
      if (yaml_frame_id_.empty()) {
        yaml_frame_id_ = "map";
      }
    }

    image_path_ = resolveImagePath(map_yaml, image);
    if (image_path_.empty()) {
      if (err) {
        *err = "failed to resolve image path";
      }
      return false;
    }

    PgmImage pgm;
    std::string pgm_err;
    if (!LoadPgm(image_path_, &pgm, &pgm_err)) {
      if (err) {
        *err = pgm_err;
      }
      return false;
    }

    w_ = pgm.width;
    h_ = pgm.height;
    const std::size_t sz =
        static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_);
    known_.assign(sz, 0);
    occ_.assign(sz, 0);

    occupied_cells_ = 0;
    free_cells_ = 0;
    unknown_cells_ = 0;

    for (int row = 0; row < h_; ++row) {
      const int y_flip = h_ - 1 - row;
      for (int x = 0; x < w_; ++x) {
        const std::size_t id = idx(x, y_flip);
        const uint8_t v = pgm.data[static_cast<std::size_t>(row) *
                                       static_cast<std::size_t>(w_) +
                                   static_cast<std::size_t>(x)];
        const double occ_prob =
            negate ? (double(v) / 255.0) : ((255.0 - double(v)) / 255.0);
        if (occ_prob > occupied_thresh) {
          known_[id] = 1;
          occ_[id] = 1;
          ++occupied_cells_;
        } else if (occ_prob < free_thresh) {
          known_[id] = 1;
          occ_[id] = 0;
          ++free_cells_;
        } else {
          ++unknown_cells_;
        }
      }
    }

    ready_ = true;
    return true;
  }

  bool ready() const { return ready_; }
  const std::string &yamlFrameId() const { return yaml_frame_id_; }
  const std::string &yamlPath() const { return yaml_path_; }
  const std::string &imagePath() const { return image_path_; }
  int width() const { return w_; }
  int height() const { return h_; }
  double resolution() const { return res_; }
  std::size_t occupiedCellCount() const { return occupied_cells_; }
  std::size_t freeCellCount() const { return free_cells_; }
  std::size_t unknownCellCount() const { return unknown_cells_; }

  nav_msgs::OccupancyGrid makeOccupancyGrid(const std::string &frame_id,
                                            const ros::Time &stamp) const {
    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.info.map_load_time = stamp;
    msg.info.resolution = static_cast<float>(res_);
    msg.info.width = static_cast<uint32_t>(w_);
    msg.info.height = static_cast<uint32_t>(h_);
    msg.info.origin.position.x = ox_;
    msg.info.origin.position.y = oy_;
    msg.info.origin.position.z = 0.0;
    msg.info.origin.orientation = YawToQuaternion(oyaw_);
    msg.data.resize(static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_),
                    -1);
    for (int y = 0; y < h_; ++y) {
      for (int x = 0; x < w_; ++x) {
        const std::size_t id = idx(x, y);
        if (!known_[id]) {
          msg.data[id] = -1;
        } else {
          msg.data[id] = occ_[id] ? 100 : 0;
        }
      }
    }
    return msg;
  }

private:
  std::size_t idx(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) +
           static_cast<std::size_t>(x);
  }

  std::string resolveImagePath(const std::string &yaml_path,
                               const std::string &image_in) const {
    if (image_in.empty()) {
      return std::string();
    }
    if (!image_in.empty() && image_in.front() == '/') {
      return FileExists(image_in) ? image_in : std::string();
    }
    const std::string p = JoinPath(Dirname(yaml_path), image_in);
    return FileExists(p) ? p : std::string();
  }

  bool ready_ = false;
  std::string yaml_path_;
  std::string image_path_;
  std::string yaml_frame_id_ = "map";
  int w_ = 0;
  int h_ = 0;
  double res_ = 0.05;
  double ox_ = 0.0;
  double oy_ = 0.0;
  double oyaw_ = 0.0;
  std::vector<uint8_t> known_;
  std::vector<uint8_t> occ_;
  std::size_t occupied_cells_ = 0;
  std::size_t free_cells_ = 0;
  std::size_t unknown_cells_ = 0;
};

Pose2D InversePose(const Pose2D &p) {
  const double c = std::cos(p.yaw);
  const double s = std::sin(p.yaw);
  Pose2D out;
  out.x = -(c * p.x + s * p.y);
  out.y = -(-s * p.x + c * p.y);
  out.yaw = NormalizeAngle(-p.yaw);
  return out;
}

Pose2D ComposePose(const Pose2D &a, const Pose2D &b) {
  const double c = std::cos(a.yaw);
  const double s = std::sin(a.yaw);
  Pose2D out;
  out.x = a.x + c * b.x - s * b.y;
  out.y = a.y + s * b.x + c * b.y;
  out.yaw = NormalizeAngle(a.yaw + b.yaw);
  return out;
}

enum class AmclLocState {
  UNINITIALIZED = 0,
  GOOD = 1,
  WEAK = 2,
  AMBIGUOUS = 3,
  LOST = 4,
};

inline const char *AmclLocStateStr(AmclLocState s) {
  switch (s) {
  case AmclLocState::UNINITIALIZED:
    return "UNINITIALIZED";
  case AmclLocState::GOOD:
    return "GOOD";
  case AmclLocState::WEAK:
    return "WEAK";
  case AmclLocState::AMBIGUOUS:
    return "AMBIGUOUS";
  case AmclLocState::LOST:
    return "LOST";
  default:
    return "UNKNOWN";
  }
}

class AmclLocalizerNode {
public:
  AmclLocalizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_) {
    loadParams();
    loadMapOrThrow();
    initParticleFilterOrThrow();
    initRos();
    publishMap();
    maybeInitializeOnStartup();
    logConfig();
    startWorker_();
  }

  ~AmclLocalizerNode() {
    stopWorker_();
    if (has_last_estimate_) {
      (void)initial_pose_mgr_.updateShutdownPose(
          toInitialPoseMgrPose_(last_estimate_pose_));
    }
    if (pf_) {
      pf_free(pf_);
      pf_ = nullptr;
    }
    if (amcl_map_) {
      map_free(amcl_map_);
      amcl_map_ = nullptr;
    }
  }

  void spin() { ros::spin(); }

private:
  struct FreeCellIndex {
    int x = 0;
    int y = 0;
  };

  struct AmclHypothesis {
    double weight = 0.0;
    pf_vector_t mean = pf_vector_zero();
    pf_matrix_t cov = pf_matrix_zero();
  };

  struct PendingInitialPose {
    bool valid = false;
    Pose2D pose;
    double cov_xx = 0.0;
    double cov_yy = 0.0;
    double cov_aa = 0.0;
    double sigma_x = 0.0;
    double sigma_y = 0.0;
    double sigma_yaw = 0.0;
    ros::Time stamp;
  };

  struct AmclGuardMetrics {
    bool core_ready = false;
    bool initialized = false;
    bool odom_ok = false;
    bool sensor_update_ok = false;
    int scan_points_count = 0;
    bool frame_valid = false;
  };

  struct AmclPfHealthMetrics {
    bool valid = false;
    bool pf_converged = false;
    int sample_count = 0;
    int cluster_count = 0;
    double best_cluster_weight = std::numeric_limits<double>::quiet_NaN();
    double second_cluster_weight = std::numeric_limits<double>::quiet_NaN();
    double ambiguity_ratio = std::numeric_limits<double>::quiet_NaN();
    double best_sigma_xy_major = std::numeric_limits<double>::quiet_NaN();
    double best_sigma_xy_minor = std::numeric_limits<double>::quiet_NaN();
    double best_sigma_yaw = std::numeric_limits<double>::quiet_NaN();
    double overall_sigma_xy_major = std::numeric_limits<double>::quiet_NaN();
    double overall_sigma_yaw = std::numeric_limits<double>::quiet_NaN();
  };

  struct AmclOdomContinuityMetrics {
    bool valid = false;
    double jump_trans = std::numeric_limits<double>::quiet_NaN();
    double jump_yaw = std::numeric_limits<double>::quiet_NaN();
    bool twist_valid = false;
    double linear_speed = std::numeric_limits<double>::quiet_NaN();
    double angular_speed = std::numeric_limits<double>::quiet_NaN();
    Pose2D predicted_pose;
  };

  struct AmclQualitySnapshot {
    ros::Time stamp;
    AmclGuardMetrics guard;
    AmclPfHealthMetrics pf;
    AmclOdomContinuityMetrics odom;
  };

  struct AmclStateConfig {
    double good_best_min = 0.98;
    double good_ambiguity_ratio_max = 0.10;
    int good_max_clusters = 1;
    double good_overall_sigma_xy_max = 0.60;
    double good_best_sigma_yaw_deg_max = 15.0;
    double good_jump_trans_max = 0.10;
    double good_jump_yaw_deg_max = 1.0;

    int ambiguous_min_clusters = 2;
    double ambiguous_best_min = 0.20;
    double ambiguous_ratio_min = 0.35;

    double lost_best_max = 0.20;
    int lost_cluster_min = 20;
    double lost_overall_sigma_xy_min = 2.0;

    int good_enter_frames = 2;
    int weak_enter_frames = 2;
    int ambiguous_enter_frames = 2;
    int lost_enter_frames = 2;

    int init_validate_good_frames = 2;
    int init_validate_bad_frames = 3;
    double init_accept_best_min = 0.70;
    double init_accept_ambiguity_ratio_max = 0.20;
    int init_accept_max_clusters = 80;
    double init_accept_overall_sigma_xy_max = 2.0;
    double init_reject_best_max = 0.20;
    int init_reject_cluster_min = 200;
    double init_reject_overall_sigma_xy_min = 5.0;

    double last_pose_stationary_linear_speed_max = 0.05;
    double last_pose_stationary_angular_speed_deg_max = 3.0;
    double last_pose_stationary_min_duration = 2.0;
  };

  struct InitSeedCandidate {
    enum class Source {
      SHUTDOWN = 0,
      LAST = 1,
      FIXED = 2,
      GLOBAL = 3,
      EXPLICIT = 4,
    };

    enum class Type {
      POSE = 0,
      GLOBAL = 1,
    };

    Type type = Type::POSE;
    Source source = Source::FIXED;
    Pose2D pose;
    std::string reason;
    int fixed_index = -1;
  };

  static pf_vector_t UniformPoseGenerator(void *arg) {
    AmclLocalizerNode *self = reinterpret_cast<AmclLocalizerNode *>(arg);
    return self ? self->uniformPoseGenerator_() : pf_vector_zero();
  }

  static Pose2D odomMsgToPose2D(const nav_msgs::Odometry &msg) {
    Pose2D p;
    p.x = msg.pose.pose.position.x;
    p.y = msg.pose.pose.position.y;
    p.yaw = QuaternionToYaw(msg.pose.pose.orientation);
    return p;
  }

  static pf_vector_t poseToPfVector(const Pose2D &pose) {
    pf_vector_t v = pf_vector_zero();
    v.v[0] = pose.x;
    v.v[1] = pose.y;
    v.v[2] = pose.yaw;
    return v;
  }

  static Pose2D pfVectorToPose(const pf_vector_t &v) {
    Pose2D pose;
    pose.x = v.v[0];
    pose.y = v.v[1];
    pose.yaw = NormalizeAngle(v.v[2]);
    return pose;
  }

  static localization_ndt::InitialPoseManager::Pose2D
  toInitialPoseMgrPose_(const Pose2D &pose) {
    localization_ndt::InitialPoseManager::Pose2D out;
    out.x = pose.x;
    out.y = pose.y;
    out.yaw = pose.yaw;
    return out;
  }

  void initConfigDirs_() {
    pkg_path_ = ros::package::getPath("localization_ndt");
    if (pkg_path_.empty()) {
      throw std::runtime_error("ros::package::getPath(localization_ndt) failed");
    }

    run_config_dir_ = JoinPath(pkg_path_, "config/run");
    interface_config_dir_ = JoinPath(pkg_path_, "config/interface");
    params_dir_ = JoinPath(pkg_path_, "config/amcl");
    pose_base_dir_ = JoinPath(pkg_path_, "config/initial_pose");
    pose_fixed_dir_ = JoinPath(pose_base_dir_, "fixed");
    pose_last_dir_ = JoinPath(pose_base_dir_, "last");

    pnh_.param<std::string>("run_config_dir", run_config_dir_, run_config_dir_);
    pnh_.param<std::string>("interface_config_dir", interface_config_dir_,
                            interface_config_dir_);
    pnh_.param<std::string>("params_dir", params_dir_, params_dir_);
    pnh_.param<std::string>("pose_base_dir", pose_base_dir_, pose_base_dir_);

    pose_fixed_dir_ = JoinPath(pose_base_dir_, "fixed");
    pose_last_dir_ = JoinPath(pose_base_dir_, "last");
    interface_yaml_path_ = JoinPath(interface_config_dir_, "interface.yaml");

    localization_ndt::config::EnsureDir(interface_config_dir_);
    localization_ndt::config::EnsureDir(params_dir_);
    localization_ndt::config::EnsureDir(pose_base_dir_);
    localization_ndt::config::EnsureDir(pose_fixed_dir_);
    localization_ndt::config::EnsureDir(pose_last_dir_);
  }

  void resolveMapSelection_() {
    const std::string run_yaml_path = JoinPath(run_config_dir_, "run.yaml");
    std::string run_err;
    if (!ResolveAmclRunMapSelection(run_yaml_path, &map_name_, &run_err)) {
      throw std::runtime_error(run_err);
    }
    map_name_ = localization_ndt::config::ExtractFileStem(map_name_);

    if (map_name_.empty()) {
      throw std::runtime_error("resolved map_name is empty");
    }

    map_yaml_path_ = JoinPath(JoinPath(pkg_path_, "map"), map_name_ + ".yaml");
  }

  void loadInterfaceConfig_() {
    YAML::Node y;
    (void)LoadOrCreateInterfaceYaml(interface_yaml_path_, &y);
    using localization_ndt::config::LoadFieldFromYaml;
    LoadFieldFromYaml(y, "scan_topic", scan_topic_);
    LoadFieldFromYaml(y, "odom_topic", odom_topic_);
    LoadFieldFromYaml(y, "imu_topic", imu_topic_);
    LoadFieldFromYaml(y, "base_frame_id", base_frame_id_);
  }

  void loadMapParamsYaml_() {
    params_yaml_path_ = JoinPath(params_dir_, map_name_ + ".yaml");
    YAML::Node y;
    (void)LoadOrCreateAmclParamsYaml(params_yaml_path_, map_name_,
                                     publish_map_to_odom_tf_, &y);
    if (!y || !y.IsMap()) {
      return;
    }

    using localization_ndt::config::LoadFieldFromYaml;
    LoadFieldFromYaml(y, "publish_tf", publish_tf_);
    LoadFieldFromYaml(y, "publish_map", publish_map_);
    LoadFieldFromYaml(y, "publish_particlecloud", publish_particlecloud_);
    LoadFieldFromYaml(y, "publish_map_to_odom_tf", publish_map_to_odom_tf_);
    LoadFieldFromYaml(y, "particlecloud_max_samples", particlecloud_max_samples_);
    LoadFieldFromYaml(y, "use_yaml_frame_id", use_yaml_frame_id_);
    LoadFieldFromYaml(y, "global_init_on_startup", global_init_on_startup_);
    LoadFieldFromYaml(y, "use_initial_pose", use_initial_pose_);

    LoadFieldFromYaml(y, "scan_min_range", scan_min_range_);
    LoadFieldFromYaml(y, "scan_max_range", scan_max_range_);
    LoadFieldFromYaml(y, "scan_voxel_m", scan_voxel_m_);
    LoadFieldFromYaml(y, "max_scan_points", max_scan_points_);

    LoadFieldFromYaml(y, "laser_min_range", laser_min_range_);
    LoadFieldFromYaml(y, "laser_max_range", laser_max_range_);
    LoadFieldFromYaml(y, "laser_model_type", laser_model_type_);
    LoadFieldFromYaml(y, "max_beams", max_beams_);
    LoadFieldFromYaml(y, "sigma_hit", sigma_hit_);
    LoadFieldFromYaml(y, "laser_likelihood_max_dist", laser_likelihood_max_dist_);
    LoadFieldFromYaml(y, "z_hit", z_hit_);
    LoadFieldFromYaml(y, "z_rand", z_rand_);
    LoadFieldFromYaml(y, "do_beamskip", do_beamskip_);
    LoadFieldFromYaml(y, "beam_skip_distance", beam_skip_distance_);
    LoadFieldFromYaml(y, "beam_skip_threshold", beam_skip_threshold_);
    LoadFieldFromYaml(y, "beam_skip_error_threshold", beam_skip_error_threshold_);

    LoadFieldFromYaml(y, "alpha1", alpha1_);
    LoadFieldFromYaml(y, "alpha2", alpha2_);
    LoadFieldFromYaml(y, "alpha3", alpha3_);
    LoadFieldFromYaml(y, "alpha4", alpha4_);
    LoadFieldFromYaml(y, "alpha5", alpha5_);
    LoadFieldFromYaml(y, "min_particles", min_particles_);
    LoadFieldFromYaml(y, "max_particles", max_particles_);
    LoadFieldFromYaml(y, "pf_err", pf_err_);
    LoadFieldFromYaml(y, "pf_z", pf_z_);
    LoadFieldFromYaml(y, "recovery_alpha_fast", recovery_alpha_fast_);
    LoadFieldFromYaml(y, "recovery_alpha_slow", recovery_alpha_slow_);
    LoadFieldFromYaml(y, "resample_interval", resample_interval_);
    LoadFieldFromYaml(y, "update_min_d", update_min_d_);

    double update_min_a_deg = update_min_a_ * 180.0 / kPi;
    LoadFieldFromYaml(y, "update_min_a_deg", update_min_a_deg);
    update_min_a_ = Deg2Rad(update_min_a_deg);

    LoadFieldFromYaml(y, "init_cov_xx", init_cov_xx_);
    LoadFieldFromYaml(y, "init_cov_yy", init_cov_yy_);
    LoadFieldFromYaml(y, "init_cov_aa", init_cov_aa_);
    LoadFieldFromYaml(y, "initial_pose_x", startup_initial_pose_.x);
    LoadFieldFromYaml(y, "initial_pose_y", startup_initial_pose_.y);
    double startup_yaw_deg = startup_initial_pose_.yaw * 180.0 / kPi;
    LoadFieldFromYaml(y, "initial_pose_yaw_deg", startup_yaw_deg);
    startup_initial_pose_.yaw = Deg2Rad(startup_yaw_deg);

    LoadFieldFromYaml(y, "manual_local_reloc_enable", manual_local_reloc_enable_);
    LoadFieldFromYaml(y, "manual_local_reloc_allow_external_map_paths",
                      manual_local_reloc_allow_external_map_paths_);
    LoadFieldFromYaml(y, "manual_local_reloc_coarse_xy_range_m",
                      manual_local_reloc_coarse_xy_range_m_);
    LoadFieldFromYaml(y, "manual_local_reloc_coarse_yaw_search_deg",
                      manual_local_reloc_coarse_yaw_search_deg_);
    LoadFieldFromYaml(y, "manual_local_reloc_xy_min_m",
                      manual_local_reloc_xy_min_m_);
    LoadFieldFromYaml(y, "manual_local_reloc_xy_sigma_k",
                      manual_local_reloc_xy_sigma_k_);
    LoadFieldFromYaml(y, "manual_local_reloc_xy_max_m",
                      manual_local_reloc_xy_max_m_);
    double manual_local_reloc_yaw_min_deg =
        manual_local_reloc_yaw_min_rad_ * 180.0 / kPi;
    LoadFieldFromYaml(y, "manual_local_reloc_yaw_min_deg",
                      manual_local_reloc_yaw_min_deg);
    manual_local_reloc_yaw_min_rad_ = Deg2Rad(manual_local_reloc_yaw_min_deg);
    LoadFieldFromYaml(y, "manual_local_reloc_yaw_sigma_k",
                      manual_local_reloc_yaw_sigma_k_);
    LoadFieldFromYaml(y, "manual_local_reloc_final_xy_range_m",
                      manual_local_reloc_final_xy_range_m_);
    LoadFieldFromYaml(y, "manual_local_reloc_final_yaw_search_deg",
                      manual_local_reloc_final_yaw_search_deg_);
    double manual_local_reloc_final_yaw_step_deg =
        manual_local_reloc_final_yaw_step_rad_ * 180.0 / kPi;
    LoadFieldFromYaml(y, "manual_local_reloc_final_yaw_step_deg",
                      manual_local_reloc_final_yaw_step_deg);
    manual_local_reloc_final_yaw_step_rad_ =
        Deg2Rad(manual_local_reloc_final_yaw_step_deg);
    LoadFieldFromYaml(y, "manual_local_reloc_reseed_sigma_scale",
                      manual_local_reloc_reseed_sigma_scale_);
    LoadFieldFromYaml(y, "manual_local_reloc_reseed_xy_min_m",
                      manual_local_reloc_reseed_xy_min_m_);
    double manual_local_reloc_reseed_yaw_min_deg =
        manual_local_reloc_reseed_yaw_min_rad_ * 180.0 / kPi;
    LoadFieldFromYaml(y, "manual_local_reloc_reseed_yaw_min_deg",
                      manual_local_reloc_reseed_yaw_min_deg);
    manual_local_reloc_reseed_yaw_min_rad_ =
        Deg2Rad(manual_local_reloc_reseed_yaw_min_deg);

    LoadFieldFromYaml(y, "local_reloc_min_score", local_reloc_common_opt_.min_score);
    LoadFieldFromYaml(y, "local_reloc_score_margin",
                      local_reloc_common_opt_.score_margin);
    LoadFieldFromYaml(y, "local_reloc_min_valid_fraction",
                      local_reloc_common_opt_.min_valid_fraction);
    LoadFieldFromYaml(y, "local_reloc_bnb_max_level",
                      local_reloc_common_opt_.bnb_max_level);
    LoadFieldFromYaml(y, "local_reloc_pyr_max_level",
                      local_reloc_common_opt_.pyr_max_level);
    LoadFieldFromYaml(y, "local_reloc_max_scan_points",
                      local_reloc_common_opt_.max_scan_points);
    LoadFieldFromYaml(y, "local_reloc_scan_voxel_m",
                      local_reloc_common_opt_.scan_voxel_m);
    LoadFieldFromYaml(y, "local_reloc_hit_sigma_m",
                      local_reloc_common_opt_.hit_sigma_m);
    LoadFieldFromYaml(y, "local_reloc_max_dist_m",
                      local_reloc_common_opt_.max_dist_m);
    LoadFieldFromYaml(y, "local_reloc_min_range",
                      local_reloc_common_opt_.min_range);
    LoadFieldFromYaml(y, "local_reloc_max_range",
                      local_reloc_common_opt_.max_range);
    double local_reloc_yaw_step_deg =
        local_reloc_common_opt_.yaw_step_rad * 180.0 / kPi;
    LoadFieldFromYaml(y, "local_reloc_yaw_step_deg",
                      local_reloc_yaw_step_deg);
    local_reloc_common_opt_.yaw_step_rad = Deg2Rad(local_reloc_yaw_step_deg);
    LoadFieldFromYaml(y, "local_reloc_xy_range_m", local_reloc_xy_range_m_);
    LoadFieldFromYaml(y, "local_reloc_yaw_search_deg", local_reloc_yaw_search_deg_);
    LoadFieldFromYaml(y, "local_reloc_settle_frames", local_reloc_settle_frames_);

    LoadFieldFromYaml(y, "startup_local_reloc_enable", startup_local_reloc_enable_);
    LoadFieldFromYaml(y, "startup_max_candidates_per_scan",
                      startup_max_candidates_per_scan_);
    LoadFieldFromYaml(y, "startup_fixed_retry_interval_sec",
                      startup_fixed_retry_interval_sec_);
    LoadFieldFromYaml(y, "startup_fixed_retry_stationary_min_duration",
                      startup_fixed_retry_stationary_min_duration_);

    LoadFieldFromYaml(y, "lost_local_reloc_enable", lost_local_reloc_enable_);
    LoadFieldFromYaml(y, "lost_local_reloc_min_interval_sec",
                      lost_local_reloc_min_interval_sec_);
    LoadFieldFromYaml(y, "lost_local_reloc_expand_factor",
                      lost_local_reloc_expand_factor_);
    LoadFieldFromYaml(y, "lost_local_reloc_max_expansions",
                      lost_local_reloc_max_expansions_);
    LoadFieldFromYaml(y, "lost_fixed_reloc_enable", lost_fixed_reloc_enable_);
    LoadFieldFromYaml(y, "lost_fixed_min_interval_sec",
                      lost_fixed_min_interval_sec_);
    LoadFieldFromYaml(y, "lost_fixed_max_candidates_per_scan",
                      lost_fixed_max_candidates_per_scan_);
    LoadFieldFromYaml(y, "lost_fixed_stationary_min_duration",
                      lost_fixed_stationary_min_duration_);

    LoadFieldFromYaml(y, "auto_local_reloc_reseed_sigma_scale",
                      auto_local_reloc_reseed_sigma_scale_);
    LoadFieldFromYaml(y, "auto_local_reloc_reseed_xy_min_m",
                      auto_local_reloc_reseed_xy_min_m_);
    double auto_local_reloc_reseed_yaw_min_deg =
        auto_local_reloc_reseed_yaw_min_rad_ * 180.0 / kPi;
    LoadFieldFromYaml(y, "auto_local_reloc_reseed_yaw_min_deg",
                      auto_local_reloc_reseed_yaw_min_deg);
    auto_local_reloc_reseed_yaw_min_rad_ =
        Deg2Rad(auto_local_reloc_reseed_yaw_min_deg);

    LoadFieldFromYaml(y, "random_seed", random_seed_);
    LoadFieldFromYaml(y, "odom_history_max_age_sec", odom_history_max_age_sec_);
    LoadFieldFromYaml(y, "odom_history_max_size", odom_history_max_size_);
    LoadFieldFromYaml(y, "tf_lookup_timeout_sec", tf_lookup_timeout_sec_);

    LoadFieldFromYaml(y, "state_good_best_min", state_cfg_.good_best_min);
    LoadFieldFromYaml(y, "state_good_ambiguity_ratio_max",
                      state_cfg_.good_ambiguity_ratio_max);
    LoadFieldFromYaml(y, "state_good_max_clusters", state_cfg_.good_max_clusters);
    LoadFieldFromYaml(y, "state_good_overall_sigma_xy_max",
                      state_cfg_.good_overall_sigma_xy_max);
    LoadFieldFromYaml(y, "state_good_best_sigma_yaw_deg_max",
                      state_cfg_.good_best_sigma_yaw_deg_max);
    LoadFieldFromYaml(y, "state_good_jump_trans_max",
                      state_cfg_.good_jump_trans_max);
    LoadFieldFromYaml(y, "state_good_jump_yaw_deg_max",
                      state_cfg_.good_jump_yaw_deg_max);
    LoadFieldFromYaml(y, "state_ambiguous_min_clusters",
                      state_cfg_.ambiguous_min_clusters);
    LoadFieldFromYaml(y, "state_ambiguous_best_min",
                      state_cfg_.ambiguous_best_min);
    LoadFieldFromYaml(y, "state_ambiguous_ratio_min",
                      state_cfg_.ambiguous_ratio_min);
    LoadFieldFromYaml(y, "state_lost_best_max", state_cfg_.lost_best_max);
    LoadFieldFromYaml(y, "state_lost_cluster_min", state_cfg_.lost_cluster_min);
    LoadFieldFromYaml(y, "state_lost_overall_sigma_xy_min",
                      state_cfg_.lost_overall_sigma_xy_min);
    LoadFieldFromYaml(y, "state_good_enter_frames", state_cfg_.good_enter_frames);
    LoadFieldFromYaml(y, "state_weak_enter_frames", state_cfg_.weak_enter_frames);
    LoadFieldFromYaml(y, "state_ambiguous_enter_frames",
                      state_cfg_.ambiguous_enter_frames);
    LoadFieldFromYaml(y, "state_lost_enter_frames", state_cfg_.lost_enter_frames);
    LoadFieldFromYaml(y, "init_validate_good_frames",
                      state_cfg_.init_validate_good_frames);
    LoadFieldFromYaml(y, "init_validate_bad_frames",
                      state_cfg_.init_validate_bad_frames);
    LoadFieldFromYaml(y, "init_accept_best_min",
                      state_cfg_.init_accept_best_min);
    LoadFieldFromYaml(y, "init_accept_ambiguity_ratio_max",
                      state_cfg_.init_accept_ambiguity_ratio_max);
    LoadFieldFromYaml(y, "init_accept_max_clusters",
                      state_cfg_.init_accept_max_clusters);
    LoadFieldFromYaml(y, "init_accept_overall_sigma_xy_max",
                      state_cfg_.init_accept_overall_sigma_xy_max);
    LoadFieldFromYaml(y, "init_reject_best_max",
                      state_cfg_.init_reject_best_max);
    LoadFieldFromYaml(y, "init_reject_cluster_min",
                      state_cfg_.init_reject_cluster_min);
    LoadFieldFromYaml(y, "init_reject_overall_sigma_xy_min",
                      state_cfg_.init_reject_overall_sigma_xy_min);
    LoadFieldFromYaml(y, "last_pose_stationary_linear_speed_max",
                      state_cfg_.last_pose_stationary_linear_speed_max);
    LoadFieldFromYaml(y, "last_pose_stationary_angular_speed_deg_max",
                      state_cfg_.last_pose_stationary_angular_speed_deg_max);
    LoadFieldFromYaml(y, "last_pose_stationary_min_duration",
                      state_cfg_.last_pose_stationary_min_duration);
  }

  void initializeInitialPoseManager_() {
    initial_pose_mgr_.setFixedDir(pose_fixed_dir_);
    initial_pose_mgr_.setLastDir(pose_last_dir_);
    if (!initial_pose_mgr_.initializeWithMapId(map_yaml_path_, map_name_, std::string())) {
      ROS_WARN_STREAM("[AMCL] InitialPoseManager init failed for map=" << map_name_);
    }
  }

  void maybeWriteLastPose_(const ros::Time &stamp) {
    if (!has_last_quality_snapshot_ || !has_last_estimate_) {
      return;
    }

    if (loc_state_ != AmclLocState::GOOD) {
      clearStationaryLastPoseTracking_();
      return;
    }

    const auto &odom = last_quality_snapshot_.odom;
    if (!odom.twist_valid) {
      return;
    }

    const double yaw_rate_deg =
        std::fabs(odom.angular_speed) * 180.0 / kPi;
    const bool now_stationary =
        (std::fabs(odom.linear_speed) <=
         state_cfg_.last_pose_stationary_linear_speed_max) &&
        (yaw_rate_deg <= state_cfg_.last_pose_stationary_angular_speed_deg_max);

    if (!now_stationary) {
      clearStationaryLastPoseTracking_();
      return;
    }

    if (!stationary_phase_) {
      stationary_phase_ = true;
      stationary_pose_written_ = false;
      stationary_start_time_ = stamp;
      return;
    }

    if (stationary_pose_written_) {
      return;
    }

    if ((stamp - stationary_start_time_).toSec() <
        state_cfg_.last_pose_stationary_min_duration) {
      return;
    }

    if (initial_pose_mgr_.updateLastPose(toInitialPoseMgrPose_(last_estimate_pose_))) {
      stationary_pose_written_ = true;
    }
  }

  void maybeUpdateLastGoodReference_() {
    if (loc_state_ != AmclLocState::GOOD) {
      return;
    }
    if (!has_last_estimate_ || !has_last_map_to_odom_) {
      return;
    }
    last_good_pose_ = last_estimate_pose_;
    last_good_map_to_odom_ = last_map_to_odom_;
    has_last_good_pose_ = true;
    has_last_good_map_to_odom_ = true;
  }

  void handleStableStateTransition_(const ros::Time &stamp) {
    if (loc_state_ == prev_loc_state_) {
      return;
    }

    ROS_INFO_STREAM("[AMCL] state edge: " << AmclLocStateStr(prev_loc_state_)
                    << " -> " << AmclLocStateStr(loc_state_)
                    << " t=" << FmtD(stamp.toSec(), 3));

    if (loc_state_ == AmclLocState::LOST) {
      last_lost_local_reloc_attempt_ = ros::Time(0);
      lost_local_reloc_fail_count_ = 0;
      clearLostFixedRecoveryTracking_();
    } else if (prev_loc_state_ == AmclLocState::LOST) {
      last_lost_local_reloc_attempt_ = ros::Time(0);
      lost_local_reloc_fail_count_ = 0;
      clearLostFixedRecoveryTracking_();
    }

    prev_loc_state_ = loc_state_;
  }

  static int stateEnterFrames_(AmclLocState state, const AmclStateConfig &cfg) {
    switch (state) {
    case AmclLocState::GOOD:
      return std::max(1, cfg.good_enter_frames);
    case AmclLocState::WEAK:
      return std::max(1, cfg.weak_enter_frames);
    case AmclLocState::AMBIGUOUS:
      return std::max(1, cfg.ambiguous_enter_frames);
    case AmclLocState::LOST:
      return std::max(1, cfg.lost_enter_frames);
    case AmclLocState::UNINITIALIZED:
    default:
      return 1;
    }
  }

  enum class InitValidationEval {
    CONTINUE = 0,
    ACCEPT = 1,
    REJECT = 2,
  };

  InitValidationEval assessInitValidation_(const AmclQualitySnapshot &snap) const {
    if (!snap.guard.frame_valid || !snap.pf.valid) {
      return InitValidationEval::CONTINUE;
    }

    const auto &pf = snap.pf;
    if (std::isfinite(pf.best_cluster_weight) &&
        pf.best_cluster_weight <= state_cfg_.init_reject_best_max) {
      return InitValidationEval::REJECT;
    }
    if (pf.cluster_count >= state_cfg_.init_reject_cluster_min) {
      return InitValidationEval::REJECT;
    }
    if (std::isfinite(pf.overall_sigma_xy_major) &&
        pf.overall_sigma_xy_major >= state_cfg_.init_reject_overall_sigma_xy_min) {
      return InitValidationEval::REJECT;
    }

    const bool accept =
        std::isfinite(pf.best_cluster_weight) &&
        (pf.best_cluster_weight >= state_cfg_.init_accept_best_min) &&
        std::isfinite(pf.ambiguity_ratio) &&
        (pf.ambiguity_ratio <= state_cfg_.init_accept_ambiguity_ratio_max) &&
        (pf.cluster_count <= state_cfg_.init_accept_max_clusters) &&
        std::isfinite(pf.overall_sigma_xy_major) &&
        (pf.overall_sigma_xy_major <= state_cfg_.init_accept_overall_sigma_xy_max);
    return accept ? InitValidationEval::ACCEPT : InitValidationEval::CONTINUE;
  }

  void clearStationaryLastPoseTracking_() {
    stationary_phase_ = false;
    stationary_pose_written_ = false;
    stationary_start_time_ = ros::Time(0);
  }

  enum class FrontendMode {
    NORMAL = 0,
    RELOC_REFINE = 1,
  };

  struct RelocRefineBasin {
    Pose2D representative_pose;
    Pose2D best_pose;
    int hits = 0;
    double best_raw_score = -std::numeric_limits<double>::infinity();
    double best_combined_score = -std::numeric_limits<double>::infinity();
    double sum_combined_score = 0.0;
    double best_valid_fraction = 0.0;
  };

  struct RelocRefineState {
    bool active = false;
    bool begin_init_validation_after_finish = false;
    Pose2D pose_center;
    Pose2D last_refined_pose;
    Pose2D last_refine_odom;
    Pose2D best_refined_pose;
    Pose2D best_refine_odom;
    bool has_best_pose = false;
    double xy_range_m = 0.3;
    double yaw_range_rad = Deg2Rad(8.0);
    int frames_left = 0;
    int stable_hits = 0;
    double last_score = 0.0;
    double best_score = -std::numeric_limits<double>::infinity();
    double best_valid_fraction = 0.0;
    std::vector<RelocRefineBasin> basins;
    std::string reason;
  };

  LocalRelocalizer2D *relocRefineMatcher_() const {
    if (manual_local_reloc_ && manual_local_reloc_->isReady()) {
      return manual_local_reloc_.get();
    }
    if (auto_local_reloc_ && auto_local_reloc_->isReady()) {
      return auto_local_reloc_.get();
    }
    return nullptr;
  }

  void clearRelocRefine_() {
    frontend_mode_ = FrontendMode::NORMAL;
    reloc_refine_ = RelocRefineState{};
  }

  void beginRelocRefine_(const Pose2D &seed_pose, const Pose2D &odom_pose,
                         const std::string &reason,
                         bool begin_init_validation_after_finish) {
    clearLocalRelocSettling_();
    frontend_mode_ = FrontendMode::RELOC_REFINE;
    reloc_refine_.active = true;
    reloc_refine_.begin_init_validation_after_finish =
        begin_init_validation_after_finish;
    reloc_refine_.pose_center = seed_pose;
    reloc_refine_.last_refined_pose = seed_pose;
    reloc_refine_.last_refine_odom = odom_pose;
    reloc_refine_.best_refined_pose = seed_pose;
    reloc_refine_.best_refine_odom = odom_pose;
    reloc_refine_.has_best_pose = true;
    reloc_refine_.xy_range_m = 0.6;
    reloc_refine_.yaw_range_rad = Deg2Rad(12.0);
    reloc_refine_.frames_left = 8;
    reloc_refine_.stable_hits = 0;
    reloc_refine_.last_score = 0.0;
    reloc_refine_.best_score = -std::numeric_limits<double>::infinity();
    reloc_refine_.best_valid_fraction = 0.0;
    reloc_refine_.reason = reason;
    ROS_INFO_STREAM("[AMCL] reloc_refine begin: reason=" << reason
                    << " win=(" << reloc_refine_.xy_range_m << "m,"
                    << reloc_refine_.yaw_range_rad * 180.0 / kPi << "deg)"
                    << " frames=" << reloc_refine_.frames_left);
  }

  void finishRelocRefine_(bool success) {
    const bool need_init_validation =
        reloc_refine_.begin_init_validation_after_finish;
    const std::string reason = reloc_refine_.reason;
    bool has_best_pose = reloc_refine_.has_best_pose;
    Pose2D best_pose = reloc_refine_.best_refined_pose;
    double best_score = reloc_refine_.best_score;
    double best_vf = reloc_refine_.best_valid_fraction;
    double best_combined = -std::numeric_limits<double>::infinity();
    int best_hits = 0;
    if (!reloc_refine_.basins.empty()) {
      double best_basin_score = -std::numeric_limits<double>::infinity();
      for (const auto &b : reloc_refine_.basins) {
        const double basin_score = relocRefineBasinScore_(b);
        if (basin_score > best_basin_score) {
          best_basin_score = basin_score;
          best_pose = b.best_pose;
          best_score = b.best_raw_score;
          best_vf = b.best_valid_fraction;
          best_combined = b.best_combined_score;
          best_hits = b.hits;
          has_best_pose = true;
        }
      }
    }
    clearRelocRefine_();
    if (success) {
      if (has_best_pose) {
        initializeFilterAtPose_(best_pose, 0.03 * 0.03, 0.03 * 0.03,
                                Deg2Rad(1.0) * Deg2Rad(1.0),
                                reason + "_refine_best");
      }
      force_update_ = true;
      ROS_INFO_STREAM("[AMCL] reloc_refine finish: ok=1 reason=" << reason
                      << " best_score=" << FmtD(best_score, 4)
                      << " best_vf=" << FmtD(best_vf, 3)
                      << " best_combined=" << FmtD(best_combined, 4)
                      << " best_hits=" << best_hits);
    } else {
      if (need_init_validation) {
        init_validation_pending_ = false;
        const AmclLocState failed_raw_state = AmclLocState::LOST;
        if (activateNextInitSeed_()) {
          ROS_WARN_STREAM("[AMCL] init seed rejected after reloc_refine fail: reason="
                          << reason << " raw=" << AmclLocStateStr(failed_raw_state)
                          << " -> try next seed");
          return;
        }
        initialized_ = false;
        initial_pose_is_known_ = false;
        has_last_map_to_odom_ = false;
        startup_local_reloc_pending_ = false;
        has_active_init_seed_ = false;
        if (!initial_pose_mgr_.fixedPoses().empty()) {
          startup_fixed_retry_waiting_ = true;
          startup_fixed_retry_stationary_start_ = ros::Time(0);
          ROS_WARN_STREAM("[AMCL] init exhausted after reloc_refine fail: reason="
                          << reason << " raw="
                          << AmclLocStateStr(failed_raw_state));
        } else {
          loc_state_ = failed_raw_state;
          ROS_WARN_STREAM("[AMCL] init failed after reloc_refine fail with no more seeds: reason="
                          << reason << " raw="
                          << AmclLocStateStr(failed_raw_state));
        }
        return;
      }
      if (has_best_pose) {
        initializeFilterAtPose_(best_pose, 0.03 * 0.03, 0.03 * 0.03,
                                Deg2Rad(1.0) * Deg2Rad(1.0),
                                reason + "_refine_best");
      }
      force_update_ = true;
      ROS_WARN_STREAM("[AMCL] reloc_refine finish: ok=0 reason=" << reason
                      << " keep_best=1 best_score=" << FmtD(best_score, 4)
                      << " best_vf=" << FmtD(best_vf, 3)
                      << " best_combined=" << FmtD(best_combined, 4)
                      << " best_hits=" << best_hits);
      return;
    }
    if (need_init_validation) {
      beginInitValidation_(reason + "_refine_done");
    }
  }

  bool stepRelocRefine_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                        const Pose2D &odom_pose, const ros::Time &stamp) {
    (void)stamp;
    if (frontend_mode_ != FrontendMode::RELOC_REFINE || !reloc_refine_.active) {
      return false;
    }

    LocalRelocalizer2D *matcher = relocRefineMatcher_();
    if (!matcher) {
      finishRelocRefine_(false);
      return true;
    }

    const Pose2D odom_delta =
        ComposePose(InversePose(reloc_refine_.last_refine_odom), odom_pose);
    Pose2D center = ComposePose(reloc_refine_.last_refined_pose, odom_delta);

    LocalRelocOptions opt = local_reloc_common_opt_;
    opt.xy_range_m = reloc_refine_.xy_range_m;
    opt.yaw_range_rad = reloc_refine_.yaw_range_rad;
    opt.yaw_step_rad = Deg2Rad(0.5);
    opt.scan_voxel_m = std::min(local_reloc_common_opt_.scan_voxel_m, 0.015);
    opt.min_score = std::max(local_reloc_common_opt_.min_score, 0.35);
    opt.score_margin = std::max(local_reloc_common_opt_.score_margin, 0.02);

    RelocPose2D center_pose;
    center_pose.x = center.x;
    center_pose.y = center.y;
    center_pose.yaw = center.yaw;

    const LocalRelocResult r = matcher->match(scan_xy_base, center_pose, opt);
    reloc_refine_.frames_left--;
    if (!r.ok) {
      if (reloc_refine_.frames_left <= 0) {
        finishRelocRefine_(false);
      }
      return true;
    }

    Pose2D refined;
    refined.x = r.best_pose.x;
    refined.y = r.best_pose.y;
    refined.yaw = r.best_pose.yaw;
    const double combined_score = refineCombinedScore_(r, center);

    const double dtrans =
        std::hypot(refined.x - reloc_refine_.last_refined_pose.x,
                   refined.y - reloc_refine_.last_refined_pose.y);
    const double dyaw =
        std::fabs(NormalizeAngle(refined.yaw - reloc_refine_.last_refined_pose.yaw));

    initializeFilterAtPose_(refined, 0.03 * 0.03, 0.03 * 0.03,
                            Deg2Rad(1.0) * Deg2Rad(1.0),
                            reloc_refine_.reason + "_refine");

    reloc_refine_.last_refined_pose = refined;
    reloc_refine_.last_refine_odom = odom_pose;
    reloc_refine_.last_score = r.best_score;
    const int basin_idx = findRelocRefineBasin_(refined);
    if (basin_idx >= 0) {
      auto &b = reloc_refine_.basins[static_cast<std::size_t>(basin_idx)];
      b.representative_pose = refined;
      b.hits++;
      b.sum_combined_score += combined_score;
      if (combined_score > b.best_combined_score) {
        b.best_combined_score = combined_score;
        b.best_raw_score = r.best_score;
        b.best_valid_fraction = r.valid_fraction;
        b.best_pose = refined;
      }
    } else {
      RelocRefineBasin b;
      b.representative_pose = refined;
      b.best_pose = refined;
      b.hits = 1;
      b.best_raw_score = r.best_score;
      b.best_combined_score = combined_score;
      b.sum_combined_score = combined_score;
      b.best_valid_fraction = r.valid_fraction;
      reloc_refine_.basins.push_back(b);
    }
    if (!reloc_refine_.has_best_pose || r.best_score > reloc_refine_.best_score + 1e-9 ||
        (std::fabs(r.best_score - reloc_refine_.best_score) <= 1e-9 &&
         r.valid_fraction > reloc_refine_.best_valid_fraction + 1e-9)) {
      reloc_refine_.best_refined_pose = refined;
      reloc_refine_.best_refine_odom = odom_pose;
      reloc_refine_.best_score = r.best_score;
      reloc_refine_.best_valid_fraction = r.valid_fraction;
      reloc_refine_.has_best_pose = true;
    }
    reloc_refine_.xy_range_m = std::max(0.12, reloc_refine_.xy_range_m * 0.6);
    reloc_refine_.yaw_range_rad =
        std::max(Deg2Rad(2.0), reloc_refine_.yaw_range_rad * 0.6);

    if (dtrans < 0.01 && dyaw < Deg2Rad(0.3)) {
      reloc_refine_.stable_hits++;
    } else {
      reloc_refine_.stable_hits = 0;
    }

    if (reloc_refine_.stable_hits >= 3 || reloc_refine_.frames_left <= 0) {
      finishRelocRefine_(true);
    }
    return true;
  }

  bool localRelocSettlingPending_() const {
    return forced_sensor_updates_remaining_ > 0;
  }

  void clearLocalRelocSettling_() {
    forced_sensor_updates_remaining_ = 0;
    local_reloc_settling_reason_.clear();
  }

  void armLocalRelocSettling_(const std::string &reason) {
    clearLocalRelocSettling_();
    if (local_reloc_settle_frames_ <= 0) {
      return;
    }
    forced_sensor_updates_remaining_ = local_reloc_settle_frames_;
    local_reloc_settling_reason_ = reason;
    ROS_INFO_STREAM("[AMCL] local_reloc settling armed: reason=" << reason
                    << " frames=" << forced_sensor_updates_remaining_);
  }

  void consumeLocalRelocSettlingFrame_() {
    if (forced_sensor_updates_remaining_ <= 0) {
      return;
    }
    forced_sensor_updates_remaining_--;
    if (forced_sensor_updates_remaining_ == 0) {
      ROS_INFO_STREAM("[AMCL] local_reloc settling complete: reason="
                      << local_reloc_settling_reason_);
      local_reloc_settling_reason_.clear();
    }
  }

  void clearLostFixedRecoveryTracking_() {
    lost_fixed_stationary_start_ = ros::Time(0);
    last_lost_fixed_cycle_start_ = ros::Time(0);
    lost_fixed_candidates_.clear();
    lost_fixed_cursor_ = 0;
    lost_fixed_cycle_active_ = false;
  }

  bool isCurrentTwistStationary_() const {
    if (!has_last_quality_snapshot_ || !last_quality_snapshot_.odom.twist_valid) {
      return false;
    }
    const auto &odom = last_quality_snapshot_.odom;
    const double yaw_rate_deg =
        std::fabs(odom.angular_speed) * 180.0 / kPi;
    return (std::fabs(odom.linear_speed) <=
            state_cfg_.last_pose_stationary_linear_speed_max) &&
           (yaw_rate_deg <= state_cfg_.last_pose_stationary_angular_speed_deg_max);
  }

  bool lostLocalRelocExhausted_() const {
    return lost_local_reloc_fail_count_ > lost_local_reloc_max_expansions_;
  }

  double commonLocalRelocYawRangeRad_() const {
    return std::min(kPi, Deg2Rad(local_reloc_yaw_search_deg_));
  }

  double manualLocalRelocCoarseYawRangeRad_() const {
    return std::min(kPi, Deg2Rad(manual_local_reloc_coarse_yaw_search_deg_));
  }

  double manualLocalRelocFinalYawRangeRad_() const {
    return std::min(kPi, Deg2Rad(manual_local_reloc_final_yaw_search_deg_));
  }

  double localRelocBaseScore_(double best_score, double second_score,
                              double valid_fraction,
                              double margin_weight,
                              double vf_weight) const {
    const double margin = Clamp(best_score - second_score, 0.0, 0.2);
    return best_score + margin_weight * margin +
           vf_weight * (valid_fraction - 0.5);
  }

  double manualRelocCombinedScore_(const LocalRelocResult &r) const {
    const double dxy = std::hypot(r.best_pose.x - manual_local_reloc_center_.x,
                                  r.best_pose.y - manual_local_reloc_center_.y);
    const double dyaw =
        std::fabs(NormalizeAngle(r.best_pose.yaw - manual_local_reloc_center_.yaw));
    const double xy_ref = std::max(0.05, manual_local_reloc_xy_prior_ref_m_);
    const double yaw_ref = std::max(Deg2Rad(1.0), manual_local_reloc_yaw_prior_ref_rad_);
    return localRelocBaseScore_(r.best_score, r.second_score, r.valid_fraction,
                                0.4, 0.2) -
           0.35 * HuberUnit(dxy / xy_ref) -
           0.01 * HuberUnit(dyaw / yaw_ref);
  }

  double refineCombinedScore_(const LocalRelocResult &r,
                              const Pose2D &pred_center) const {
    const double dxy =
        std::hypot(r.best_pose.x - pred_center.x, r.best_pose.y - pred_center.y);
    const double dyaw =
        std::fabs(NormalizeAngle(r.best_pose.yaw - pred_center.yaw));
    return localRelocBaseScore_(r.best_score, r.second_score, r.valid_fraction,
                                0.4, 0.2) -
           0.25 * HuberUnit(dxy / 0.12) -
           0.05 * HuberUnit(dyaw / Deg2Rad(2.0));
  }

  int findRelocRefineBasin_(const Pose2D &pose) const {
    for (std::size_t i = 0; i < reloc_refine_.basins.size(); ++i) {
      const auto &b = reloc_refine_.basins[i];
      const double dxy = std::hypot(pose.x - b.representative_pose.x,
                                    pose.y - b.representative_pose.y);
      const double dyaw = std::fabs(
          NormalizeAngle(pose.yaw - b.representative_pose.yaw));
      if (dxy < 0.12 && dyaw < Deg2Rad(2.0)) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  double relocRefineBasinScore_(const RelocRefineBasin &b) const {
    const double avg_combined =
        (b.hits > 0) ? (b.sum_combined_score / static_cast<double>(b.hits)) : -1e9;
    return 0.6 * b.best_combined_score + 0.4 * avg_combined +
           0.01 * static_cast<double>(b.hits);
  }

  void rebuildLostFixedCandidates_() {
    lost_fixed_candidates_.clear();
    lost_fixed_cursor_ = 0;
    const auto &fixed = initial_pose_mgr_.fixedPoses();
    if (fixed.empty()) {
      return;
    }

    lost_fixed_candidates_.reserve(fixed.size());
    for (std::size_t i = 0; i < fixed.size(); ++i) {
      lost_fixed_candidates_.push_back(i);
    }
    std::sort(lost_fixed_candidates_.begin(), lost_fixed_candidates_.end(),
              [&](std::size_t a, std::size_t b) {
                return fixed[a].priority > fixed[b].priority;
              });
  }

  void buildFixedOnlyInitSeedCandidates_() {
    init_seed_candidates_.clear();
    init_seed_cursor_ = 0;

    const auto &fixed = initial_pose_mgr_.fixedPoses();
    if (fixed.empty()) {
      return;
    }

    std::vector<std::size_t> order(fixed.size());
    for (std::size_t i = 0; i < fixed.size(); ++i) {
      order[i] = i;
    }
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
      return fixed[a].priority > fixed[b].priority;
    });

    for (std::size_t rank = 0; rank < order.size(); ++rank) {
      const std::size_t idx = order[rank];
      InitSeedCandidate cand;
      cand.type = InitSeedCandidate::Type::POSE;
      cand.source = InitSeedCandidate::Source::FIXED;
      cand.pose.x = fixed[idx].pose.x;
      cand.pose.y = fixed[idx].pose.y;
      cand.pose.yaw = NormalizeAngle(fixed[idx].pose.yaw);
      cand.reason = fixed[idx].name.empty()
                        ? ("startup_fixed_retry[" + std::to_string(rank) + "]")
                        : ("startup_fixed_retry:" + fixed[idx].name);
      cand.fixed_index = static_cast<int>(idx);
      init_seed_candidates_.push_back(cand);
    }
  }

  bool maybeActivateStartupFixedRetryCycle_(const ros::Time &stamp) {
    if (!startup_fixed_retry_waiting_) {
      return false;
    }
    if (!isCurrentTwistStationary_()) {
      startup_fixed_retry_stationary_start_ = ros::Time(0);
      return false;
    }
    if (startup_fixed_retry_stationary_start_.isZero()) {
      startup_fixed_retry_stationary_start_ = stamp;
      return false;
    }
    if ((stamp - startup_fixed_retry_stationary_start_).toSec() <
        startup_fixed_retry_stationary_min_duration_) {
      return false;
    }
    if (!startup_last_fixed_retry_cycle_start_.isZero() &&
        startup_fixed_retry_interval_sec_ > 0.0 &&
        (stamp - startup_last_fixed_retry_cycle_start_) <
            ros::Duration(startup_fixed_retry_interval_sec_)) {
      return false;
    }

    buildFixedOnlyInitSeedCandidates_();
    if (init_seed_candidates_.empty()) {
      return false;
    }
    startup_last_fixed_retry_cycle_start_ = stamp;
    startup_fixed_retry_waiting_ = false;
    ROS_INFO_STREAM("[AMCL] startup fixed retry cycle activate: cand="
                    << init_seed_candidates_.size());
    return activateNextInitSeed_();
  }

  void beginInitValidation_(const std::string &reason) {
    init_validation_pending_ = true;
    init_validation_reason_ = reason;
    init_validate_good_streak_ = 0;
    init_validate_bad_streak_ = 0;
    loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_ = AmclLocState::UNINITIALIZED;
    prev_raw_loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_streak_ = 0;
    clearStationaryLastPoseTracking_();
  }

  bool activateInitSeedCandidate_(const InitSeedCandidate &cand) {
    clearRelocRefine_();
    active_init_seed_ = cand;
    has_active_init_seed_ = true;
    if (cand.type == InitSeedCandidate::Type::GLOBAL) {
      startup_local_reloc_pending_ = false;
      globalLocalization_(cand.reason);
    } else {
      initializeFilterAtPose_(cand.pose, init_cov_xx_, init_cov_yy_, init_cov_aa_,
                              cand.reason);
      startup_local_reloc_pending_ =
          startup_local_reloc_enable_ && auto_local_reloc_ &&
          auto_local_reloc_->isReady();
      startup_local_reloc_center_.x = cand.pose.x;
      startup_local_reloc_center_.y = cand.pose.y;
      startup_local_reloc_center_.yaw = cand.pose.yaw;
      startup_local_reloc_reason_ = cand.reason;
    }
    beginInitValidation_(cand.reason);
    return true;
  }

  bool activateNextInitSeed_() {
    if (init_seed_cursor_ >= init_seed_candidates_.size()) {
      return false;
    }
    const InitSeedCandidate cand = init_seed_candidates_[init_seed_cursor_++];
    ROS_INFO_STREAM("[AMCL] init seed activate idx=" << init_seed_cursor_ << "/"
                    << init_seed_candidates_.size()
                    << " reason=" << cand.reason
                    << " type="
                    << (cand.type == InitSeedCandidate::Type::GLOBAL ? "global"
                                                                     : "pose"));
    return activateInitSeedCandidate_(cand);
  }

  void buildStartupInitSeedCandidates_() {
    init_seed_candidates_.clear();
    init_seed_cursor_ = 0;

    auto append_pose = [&](const Pose2D &pose, const std::string &reason,
                           InitSeedCandidate::Source source,
                           int fixed_index = -1) {
      InitSeedCandidate cand;
      cand.type = InitSeedCandidate::Type::POSE;
      cand.source = source;
      cand.pose = pose;
      cand.reason = reason;
      cand.fixed_index = fixed_index;
      init_seed_candidates_.push_back(cand);
    };

    auto from_mgr_pose = [](const localization_ndt::InitialPoseManager::Pose2D &p)
        -> Pose2D {
      Pose2D out;
      out.x = p.x;
      out.y = p.y;
      out.yaw = NormalizeAngle(p.yaw);
      return out;
    };

    if (use_initial_pose_) {
      append_pose(startup_initial_pose_, "startup_initial_pose",
                  InitSeedCandidate::Source::EXPLICIT);
      return;
    }

    if (initial_pose_mgr_.hasShutdownPose()) {
      append_pose(from_mgr_pose(initial_pose_mgr_.shutdownPose()),
                  "startup_shutdown_pose",
                  InitSeedCandidate::Source::SHUTDOWN);
    }
    if (initial_pose_mgr_.hasLastPose()) {
      append_pose(from_mgr_pose(initial_pose_mgr_.lastPose()), "startup_last_pose",
                  InitSeedCandidate::Source::LAST);
    }

    const auto &fixed = initial_pose_mgr_.fixedPoses();
    if (!fixed.empty()) {
      std::vector<std::size_t> order(fixed.size());
      for (std::size_t i = 0; i < fixed.size(); ++i) {
        order[i] = i;
      }
      std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return fixed[a].priority > fixed[b].priority;
      });
      for (std::size_t rank = 0; rank < order.size(); ++rank) {
        const std::size_t idx = order[rank];
        append_pose(from_mgr_pose(fixed[idx].pose),
                    fixed[idx].name.empty()
                        ? ("startup_fixed_pose[" + std::to_string(rank) + "]")
                        : ("startup_fixed_pose:" + fixed[idx].name),
                    InitSeedCandidate::Source::FIXED, static_cast<int>(idx));
      }
    }

    if (global_init_on_startup_) {
      InitSeedCandidate cand;
      cand.type = InitSeedCandidate::Type::GLOBAL;
      cand.source = InitSeedCandidate::Source::GLOBAL;
      cand.reason = "startup_global_init";
      init_seed_candidates_.push_back(cand);
    }
  }

  void applyLocalRelocSuccess_(const Pose2D &refined, double xy_range_m,
                               double yaw_range_rad, double reseed_sigma_scale,
                               double reseed_xy_min_m,
                               double reseed_yaw_min_rad,
                               const std::string &reason) {
    const double reseed_sigma_xy =
        std::max(reseed_xy_min_m,
                 std::min(xy_range_m, reseed_sigma_scale * xy_range_m));
    const double reseed_sigma_yaw =
        std::max(reseed_yaw_min_rad,
                 std::min(yaw_range_rad, reseed_sigma_scale * yaw_range_rad));

    initializeFilterAtPose_(refined, reseed_sigma_xy * reseed_sigma_xy,
                            reseed_sigma_xy * reseed_sigma_xy,
                            reseed_sigma_yaw * reseed_sigma_yaw, reason);
  }

  bool runStartupLocalRelocIfRequested_(
      const std::vector<Eigen::Vector2f> &scan_xy_base,
      const Pose2D *odom_pose) {
    if (!startup_local_reloc_pending_) {
      return false;
    }

    if (!startup_local_reloc_enable_ || !auto_local_reloc_ ||
        !auto_local_reloc_->isReady()) {
      startup_local_reloc_pending_ = false;
      return false;
    }

    int fixed_attempted_this_scan = 0;
    while (startup_local_reloc_pending_) {
      startup_local_reloc_pending_ = false;
      const InitSeedCandidate::Source source =
          has_active_init_seed_ ? active_init_seed_.source
                                : InitSeedCandidate::Source::FIXED;

      LocalRelocOptions opt = local_reloc_common_opt_;
      opt.xy_range_m = local_reloc_xy_range_m_;
      opt.yaw_range_rad = commonLocalRelocYawRangeRad_();
      const LocalRelocResult r =
          auto_local_reloc_->match(scan_xy_base, startup_local_reloc_center_, opt);
      if (r.ok) {
        Pose2D refined;
        refined.x = r.best_pose.x;
        refined.y = r.best_pose.y;
        refined.yaw = r.best_pose.yaw;
        applyLocalRelocSuccess_(refined, opt.xy_range_m, opt.yaw_range_rad,
                                auto_local_reloc_reseed_sigma_scale_,
                                auto_local_reloc_reseed_xy_min_m_,
                                auto_local_reloc_reseed_yaw_min_rad_,
                                "startup_local_reloc");
        if (odom_pose) {
          beginRelocRefine_(refined, *odom_pose, "startup_local_reloc", true);
        } else {
          beginInitValidation_("startup_local_reloc");
        }
        ROS_INFO_STREAM("[AMCL] startup local_reloc success: x=" << refined.x
                        << " y=" << refined.y << " yaw="
                        << refined.yaw * 180.0 / kPi << "deg"
                        << " score=" << r.best_score
                        << " second=" << r.second_score
                        << " vf=" << r.valid_fraction
                        << " seed=" << startup_local_reloc_reason_);
        return true;
      }

      ROS_WARN_STREAM("[AMCL] startup local_reloc failed: best=" << r.best_score
                      << " second=" << r.second_score
                      << " vf=" << r.valid_fraction
                      << " seed=" << startup_local_reloc_reason_);

      if (source == InitSeedCandidate::Source::FIXED) {
        fixed_attempted_this_scan++;
      }

      if (!activateNextInitSeed_()) {
        return false;
      }

      if (source == InitSeedCandidate::Source::FIXED &&
          fixed_attempted_this_scan >= startup_max_candidates_per_scan_) {
        return true;
      }
    }
    return false;
  }

  bool runLostLocalRelocIfNeeded_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                                  const ros::Time &stamp,
                                  const Pose2D &odom_pose) {
    if (init_validation_pending_ || !lost_local_reloc_enable_ || !auto_local_reloc_ ||
        !auto_local_reloc_->isReady()) {
      return false;
    }
    if (loc_state_ != AmclLocState::LOST || !has_last_good_map_to_odom_) {
      return false;
    }
    if (lostLocalRelocExhausted_()) {
      return false;
    }
    if (!last_lost_local_reloc_attempt_.isZero() &&
        lost_local_reloc_min_interval_sec_ > 0.0 &&
        (stamp - last_lost_local_reloc_attempt_) <
            ros::Duration(lost_local_reloc_min_interval_sec_)) {
      return false;
    }
    last_lost_local_reloc_attempt_ = stamp;

    Pose2D center_pose = ComposePose(last_good_map_to_odom_, odom_pose);
    RelocPose2D center;
    center.x = center_pose.x;
    center.y = center_pose.y;
    center.yaw = center_pose.yaw;

    const int use_expand =
        std::min(lost_local_reloc_fail_count_, lost_local_reloc_max_expansions_);
    const double expand = std::pow(lost_local_reloc_expand_factor_, use_expand);

    LocalRelocOptions opt = local_reloc_common_opt_;
    opt.xy_range_m = local_reloc_xy_range_m_ * expand;
    opt.yaw_range_rad =
        std::min(kPi, commonLocalRelocYawRangeRad_() * expand);

    const LocalRelocResult r = auto_local_reloc_->match(scan_xy_base, center, opt);
    if (!r.ok) {
      lost_local_reloc_fail_count_++;
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[AMCL] lost local_reloc failed: best=" << r.best_score
                                                       << " second=" << r.second_score
                                                       << " vf=" << r.valid_fraction
                                                       << " expand=" << FmtD(expand, 2)
                                                       << " center=(" << center.x
                                                       << "," << center.y << ","
                                                       << center.yaw * 180.0 / kPi
                                                       << "deg)");
      return false;
    }

    lost_local_reloc_fail_count_ = 0;
    Pose2D refined;
    refined.x = r.best_pose.x;
    refined.y = r.best_pose.y;
    refined.yaw = r.best_pose.yaw;
    applyLocalRelocSuccess_(refined, opt.xy_range_m, opt.yaw_range_rad,
                            auto_local_reloc_reseed_sigma_scale_,
                            auto_local_reloc_reseed_xy_min_m_,
                            auto_local_reloc_reseed_yaw_min_rad_,
                            "lost_local_reloc");
    beginRelocRefine_(refined, odom_pose, "lost_local_reloc", false);
    ROS_INFO_STREAM("[AMCL] lost local_reloc success: x=" << refined.x
                    << " y=" << refined.y << " yaw="
                    << refined.yaw * 180.0 / kPi << "deg"
                    << " score=" << r.best_score
                    << " second=" << r.second_score
                    << " vf=" << r.valid_fraction
                    << " expand=" << FmtD(expand, 2));
    return true;
  }

  bool runLostFixedRelocIfNeeded_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                                  const Pose2D &odom_pose,
                                  const ros::Time &stamp) {
    if (init_validation_pending_ || !lost_fixed_reloc_enable_ || !auto_local_reloc_ ||
        !auto_local_reloc_->isReady()) {
      return false;
    }
    if (loc_state_ != AmclLocState::LOST || !lostLocalRelocExhausted_()) {
      clearLostFixedRecoveryTracking_();
      return false;
    }

    if (!isCurrentTwistStationary_()) {
      lost_fixed_stationary_start_ = ros::Time(0);
      return false;
    }

    if (lost_fixed_stationary_start_.isZero()) {
      lost_fixed_stationary_start_ = stamp;
      return false;
    }

    if ((stamp - lost_fixed_stationary_start_).toSec() <
        lost_fixed_stationary_min_duration_) {
      return false;
    }

    if (!lost_fixed_cycle_active_) {
      if (!last_lost_fixed_cycle_start_.isZero() &&
          lost_fixed_min_interval_sec_ > 0.0 &&
          (stamp - last_lost_fixed_cycle_start_) <
              ros::Duration(lost_fixed_min_interval_sec_)) {
        return false;
      }
      rebuildLostFixedCandidates_();
      lost_fixed_cycle_active_ = !lost_fixed_candidates_.empty();
      lost_fixed_cursor_ = 0;
      last_lost_fixed_cycle_start_ = stamp;
    }
    if (lost_fixed_candidates_.empty()) {
      return false;
    }

    const auto &fixed = initial_pose_mgr_.fixedPoses();
    const int K = std::min(lost_fixed_max_candidates_per_scan_,
                           static_cast<int>(lost_fixed_candidates_.size()));

    for (int attempt = 0; attempt < K; ++attempt) {
      if (lost_fixed_cursor_ >= lost_fixed_candidates_.size()) {
        lost_fixed_cycle_active_ = false;
        break;
      }
      const std::size_t fixed_idx = lost_fixed_candidates_[lost_fixed_cursor_++];
      if (fixed_idx >= fixed.size()) {
        continue;
      }

      const auto &entry = fixed[fixed_idx];
      RelocPose2D center;
      center.x = entry.pose.x;
      center.y = entry.pose.y;
      center.yaw = entry.pose.yaw;

      LocalRelocOptions opt = local_reloc_common_opt_;
      const LocalRelocResult r = auto_local_reloc_->match(scan_xy_base, center, opt);
      if (!r.ok) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[AMCL] lost fixed_reloc failed: idx=" << fixed_idx
                                                        << " name=" << entry.name
                                                        << " best=" << r.best_score
                                                        << " second=" << r.second_score
                                                        << " vf=" << r.valid_fraction);
        continue;
      }

      Pose2D refined;
      refined.x = r.best_pose.x;
      refined.y = r.best_pose.y;
      refined.yaw = r.best_pose.yaw;
      applyLocalRelocSuccess_(refined, opt.xy_range_m, opt.yaw_range_rad,
                              auto_local_reloc_reseed_sigma_scale_,
                              auto_local_reloc_reseed_xy_min_m_,
                              auto_local_reloc_reseed_yaw_min_rad_,
                              "lost_fixed_reloc");
      beginRelocRefine_(refined, odom_pose, "lost_fixed_reloc", false);
      (void)initial_pose_mgr_.bumpFixedPriority(fixed_idx, 1);
      clearLostFixedRecoveryTracking_();
      ROS_INFO_STREAM("[AMCL] lost fixed_reloc success: idx=" << fixed_idx
                      << " name=" << entry.name
                      << " x=" << refined.x << " y=" << refined.y
                      << " yaw=" << refined.yaw * 180.0 / kPi << "deg"
                      << " score=" << r.best_score
                      << " second=" << r.second_score
                      << " vf=" << r.valid_fraction);
      return true;
    }

    return false;
  }

  AmclLocState classifyRawState_(const AmclQualitySnapshot &snap) const {
    const auto &g = snap.guard;
    const auto &pf = snap.pf;
    const auto &od = snap.odom;

    if (!g.core_ready || !g.initialized) {
      return AmclLocState::UNINITIALIZED;
    }
    if (!g.odom_ok || g.scan_points_count <= 0 || !g.sensor_update_ok) {
      return AmclLocState::LOST;
    }
    if (!pf.valid) {
      return AmclLocState::LOST;
    }

    const double jump_yaw_deg =
        od.valid ? (od.jump_yaw * 180.0 / kPi) : 0.0;
    const bool jump_bad =
        od.valid &&
        (od.jump_trans > state_cfg_.good_jump_trans_max ||
         jump_yaw_deg > state_cfg_.good_jump_yaw_deg_max);

    const bool lost =
        (std::isfinite(pf.best_cluster_weight) &&
         pf.best_cluster_weight < state_cfg_.lost_best_max) ||
        (pf.cluster_count >= state_cfg_.lost_cluster_min) ||
        (std::isfinite(pf.overall_sigma_xy_major) &&
         pf.overall_sigma_xy_major > state_cfg_.lost_overall_sigma_xy_min);
    if (lost) {
      return AmclLocState::LOST;
    }

    const bool ambiguous =
        (pf.cluster_count >= state_cfg_.ambiguous_min_clusters) &&
        std::isfinite(pf.best_cluster_weight) &&
        (pf.best_cluster_weight >= state_cfg_.ambiguous_best_min) &&
        std::isfinite(pf.ambiguity_ratio) &&
        (pf.ambiguity_ratio >= state_cfg_.ambiguous_ratio_min);
    if (ambiguous) {
      return AmclLocState::AMBIGUOUS;
    }

    const bool good =
        (pf.cluster_count <= state_cfg_.good_max_clusters) &&
        std::isfinite(pf.best_cluster_weight) &&
        (pf.best_cluster_weight >= state_cfg_.good_best_min) &&
        std::isfinite(pf.ambiguity_ratio) &&
        (pf.ambiguity_ratio <= state_cfg_.good_ambiguity_ratio_max) &&
        std::isfinite(pf.overall_sigma_xy_major) &&
        (pf.overall_sigma_xy_major <= state_cfg_.good_overall_sigma_xy_max) &&
        std::isfinite(pf.best_sigma_yaw) &&
        (pf.best_sigma_yaw * 180.0 / kPi <= state_cfg_.good_best_sigma_yaw_deg_max) &&
        !jump_bad;
    if (good) {
      return AmclLocState::GOOD;
    }

    return AmclLocState::WEAK;
  }

  void updateLocState_(const AmclQualitySnapshot &snap) {
    if (frontend_mode_ == FrontendMode::RELOC_REFINE && reloc_refine_.active) {
      raw_loc_state_ = AmclLocState::UNINITIALIZED;
      prev_raw_loc_state_ = AmclLocState::UNINITIALIZED;
      raw_loc_state_streak_ = 1;
      loc_state_ = AmclLocState::UNINITIALIZED;
      return;
    }

    raw_loc_state_ = classifyRawState_(snap);
    if (raw_loc_state_ == prev_raw_loc_state_) {
      raw_loc_state_streak_++;
    } else {
      prev_raw_loc_state_ = raw_loc_state_;
      raw_loc_state_streak_ = 1;
    }

    if (init_validation_pending_) {
      if (localRelocSettlingPending_()) {
        loc_state_ = AmclLocState::UNINITIALIZED;
        return;
      }

      const InitValidationEval init_eval = assessInitValidation_(snap);
      if (init_eval == InitValidationEval::ACCEPT) {
        init_validate_good_streak_++;
        init_validate_bad_streak_ = 0;
      } else if (init_eval == InitValidationEval::REJECT) {
        init_validate_bad_streak_++;
        init_validate_good_streak_ = 0;
      }

      if (init_validate_good_streak_ >=
          std::max(1, state_cfg_.init_validate_good_frames)) {
        init_validation_pending_ = false;
        loc_state_ = (raw_loc_state_ == AmclLocState::UNINITIALIZED)
                         ? AmclLocState::WEAK
                         : raw_loc_state_;
        if (has_active_init_seed_ &&
            active_init_seed_.source == InitSeedCandidate::Source::FIXED &&
            active_init_seed_.fixed_index >= 0) {
          (void)initial_pose_mgr_.bumpFixedPriority(
              static_cast<std::size_t>(active_init_seed_.fixed_index), 1);
        }
        ROS_INFO_STREAM("[AMCL] init validated success: reason="
                        << init_validation_reason_ << " -> "
                        << AmclLocStateStr(loc_state_));
        return;
      }

      if (init_validate_bad_streak_ >=
          std::max(1, state_cfg_.init_validate_bad_frames)) {
        const std::string failed_reason = init_validation_reason_;
        const AmclLocState failed_raw_state = raw_loc_state_;
        init_validation_pending_ = false;
        if (activateNextInitSeed_()) {
          ROS_WARN_STREAM("[AMCL] init seed rejected: reason=" << failed_reason
                          << " raw=" << AmclLocStateStr(failed_raw_state)
                          << " -> try next seed");
          return;
        }
        initialized_ = false;
        initial_pose_is_known_ = false;
        has_last_map_to_odom_ = false;
        startup_local_reloc_pending_ = false;
        has_active_init_seed_ = false;
        if (!initial_pose_mgr_.fixedPoses().empty()) {
          startup_fixed_retry_waiting_ = true;
          startup_fixed_retry_stationary_start_ = ros::Time(0);
          ROS_WARN_STREAM("[AMCL] init exhausted, enter fixed retry wait: reason="
                          << failed_reason << " raw="
                          << AmclLocStateStr(failed_raw_state));
        } else {
          loc_state_ = failed_raw_state;
          ROS_WARN_STREAM("[AMCL] init failed with no more seeds: reason="
                          << failed_reason << " raw="
                          << AmclLocStateStr(failed_raw_state));
        }
        return;
      }

      loc_state_ = AmclLocState::UNINITIALIZED;
      return;
    }

    const int need = stateEnterFrames_(raw_loc_state_, state_cfg_);
    if (raw_loc_state_ == AmclLocState::UNINITIALIZED ||
        raw_loc_state_streak_ >= need) {
      loc_state_ = raw_loc_state_;
    }
  }

  void loadParams() {
    initConfigDirs_();
    resolveMapSelection_();
    loadInterfaceConfig_();
    loadMapParamsYaml_();

    pnh_.param<std::string>("scan_topic", scan_topic_, scan_topic_);
    pnh_.param<std::string>("odom_topic", odom_topic_, odom_topic_);
    pnh_.param<std::string>("imu_topic", imu_topic_, imu_topic_);
    pnh_.param<std::string>("initialpose_topic", initialpose_topic_,
                            initialpose_topic_);
    pnh_.param<std::string>("map_frame_id", map_frame_id_, map_frame_id_);
    pnh_.param<std::string>("base_frame_id", base_frame_id_, base_frame_id_);
    pnh_.param<std::string>("odom_frame_id", odom_frame_id_, odom_frame_id_);
    pnh_.param<std::string>("odom_source_mode", odom_source_mode_,
                            odom_source_mode_);
    pnh_.param<std::string>("odom_pub_topic", odom_pub_topic_, odom_pub_topic_);
    pnh_.param<std::string>("amcl_pose_topic", amcl_pose_topic_, amcl_pose_topic_);
    pnh_.param<std::string>("particlecloud_topic", particlecloud_topic_,
                            particlecloud_topic_);
    pnh_.param<std::string>("map_topic", map_topic_, map_topic_);
    pnh_.param<std::string>("map_metadata_topic", map_metadata_topic_,
                            map_metadata_topic_);

    pnh_.param("publish_tf", publish_tf_, publish_tf_);
    pnh_.param("publish_map", publish_map_, publish_map_);
    pnh_.param("publish_particlecloud", publish_particlecloud_, publish_particlecloud_);
    pnh_.param("publish_map_to_odom_tf", publish_map_to_odom_tf_,
               publish_map_to_odom_tf_);
    pnh_.param("particlecloud_max_samples", particlecloud_max_samples_,
               particlecloud_max_samples_);
    pnh_.param("use_yaml_frame_id", use_yaml_frame_id_, use_yaml_frame_id_);
    pnh_.param("global_init_on_startup", global_init_on_startup_,
               global_init_on_startup_);
    pnh_.param("use_initial_pose", use_initial_pose_, use_initial_pose_);

    pnh_.param("scan_min_range", scan_min_range_, scan_min_range_);
    pnh_.param("scan_max_range", scan_max_range_, scan_max_range_);
    pnh_.param("scan_voxel_m", scan_voxel_m_, scan_voxel_m_);
    pnh_.param("max_scan_points", max_scan_points_, max_scan_points_);

    pnh_.param("laser_min_range", laser_min_range_, laser_min_range_);
    pnh_.param("laser_max_range", laser_max_range_, laser_max_range_);
    pnh_.param("laser_model_type", laser_model_type_, laser_model_type_);
    pnh_.param("max_beams", max_beams_, max_beams_);
    pnh_.param("sigma_hit", sigma_hit_, sigma_hit_);
    pnh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_,
               laser_likelihood_max_dist_);
    pnh_.param("z_hit", z_hit_, z_hit_);
    pnh_.param("z_rand", z_rand_, z_rand_);
    pnh_.param("do_beamskip", do_beamskip_, do_beamskip_);
    pnh_.param("beam_skip_distance", beam_skip_distance_, beam_skip_distance_);
    pnh_.param("beam_skip_threshold", beam_skip_threshold_, beam_skip_threshold_);
    pnh_.param("beam_skip_error_threshold", beam_skip_error_threshold_,
               beam_skip_error_threshold_);

    pnh_.param("alpha1", alpha1_, alpha1_);
    pnh_.param("alpha2", alpha2_, alpha2_);
    pnh_.param("alpha3", alpha3_, alpha3_);
    pnh_.param("alpha4", alpha4_, alpha4_);
    pnh_.param("alpha5", alpha5_, alpha5_);
    pnh_.param("min_particles", min_particles_, min_particles_);
    pnh_.param("max_particles", max_particles_, max_particles_);
    pnh_.param("pf_err", pf_err_, pf_err_);
    pnh_.param("pf_z", pf_z_, pf_z_);
    pnh_.param("recovery_alpha_fast", recovery_alpha_fast_, recovery_alpha_fast_);
    pnh_.param("recovery_alpha_slow", recovery_alpha_slow_, recovery_alpha_slow_);
    pnh_.param("resample_interval", resample_interval_, resample_interval_);
    pnh_.param("update_min_d", update_min_d_, update_min_d_);
    double update_min_a_deg = update_min_a_ * 180.0 / kPi;
    pnh_.param("update_min_a_deg", update_min_a_deg, update_min_a_deg);
    update_min_a_ = Deg2Rad(update_min_a_deg);

    pnh_.param("init_cov_xx", init_cov_xx_, init_cov_xx_);
    pnh_.param("init_cov_yy", init_cov_yy_, init_cov_yy_);
    pnh_.param("init_cov_aa", init_cov_aa_, init_cov_aa_);
    pnh_.param("initial_pose_x", startup_initial_pose_.x, startup_initial_pose_.x);
    pnh_.param("initial_pose_y", startup_initial_pose_.y, startup_initial_pose_.y);
    double startup_yaw_deg = startup_initial_pose_.yaw * 180.0 / kPi;
    pnh_.param("initial_pose_yaw_deg", startup_yaw_deg, startup_yaw_deg);
    startup_initial_pose_.yaw = Deg2Rad(startup_yaw_deg);

    pnh_.param("manual_local_reloc_enable", manual_local_reloc_enable_,
               manual_local_reloc_enable_);
    pnh_.param("manual_local_reloc_allow_external_map_paths",
               manual_local_reloc_allow_external_map_paths_,
               manual_local_reloc_allow_external_map_paths_);
    pnh_.param("manual_local_reloc_coarse_xy_range_m",
               manual_local_reloc_coarse_xy_range_m_,
               manual_local_reloc_coarse_xy_range_m_);
    pnh_.param("manual_local_reloc_coarse_yaw_search_deg",
               manual_local_reloc_coarse_yaw_search_deg_,
               manual_local_reloc_coarse_yaw_search_deg_);
    pnh_.param("manual_local_reloc_xy_min_m", manual_local_reloc_xy_min_m_,
               manual_local_reloc_xy_min_m_);
    pnh_.param("manual_local_reloc_xy_sigma_k",
               manual_local_reloc_xy_sigma_k_,
               manual_local_reloc_xy_sigma_k_);
    pnh_.param("manual_local_reloc_xy_max_m", manual_local_reloc_xy_max_m_,
               manual_local_reloc_xy_max_m_);
    double manual_local_reloc_yaw_min_deg =
        manual_local_reloc_yaw_min_rad_ * 180.0 / kPi;
    pnh_.param("manual_local_reloc_yaw_min_deg",
               manual_local_reloc_yaw_min_deg,
               manual_local_reloc_yaw_min_deg);
    manual_local_reloc_yaw_min_rad_ =
        Deg2Rad(manual_local_reloc_yaw_min_deg);
    pnh_.param("manual_local_reloc_yaw_sigma_k",
               manual_local_reloc_yaw_sigma_k_,
               manual_local_reloc_yaw_sigma_k_);
    pnh_.param("manual_local_reloc_final_xy_range_m",
               manual_local_reloc_final_xy_range_m_,
               manual_local_reloc_final_xy_range_m_);
    pnh_.param("manual_local_reloc_final_yaw_search_deg",
               manual_local_reloc_final_yaw_search_deg_,
               manual_local_reloc_final_yaw_search_deg_);
    double manual_local_reloc_final_yaw_step_deg =
        manual_local_reloc_final_yaw_step_rad_ * 180.0 / kPi;
    pnh_.param("manual_local_reloc_final_yaw_step_deg",
               manual_local_reloc_final_yaw_step_deg,
               manual_local_reloc_final_yaw_step_deg);
    manual_local_reloc_final_yaw_step_rad_ =
        Deg2Rad(manual_local_reloc_final_yaw_step_deg);
    pnh_.param("manual_local_reloc_reseed_sigma_scale",
               manual_local_reloc_reseed_sigma_scale_,
               manual_local_reloc_reseed_sigma_scale_);
    pnh_.param("manual_local_reloc_reseed_xy_min_m",
               manual_local_reloc_reseed_xy_min_m_,
               manual_local_reloc_reseed_xy_min_m_);
    double manual_local_reloc_reseed_yaw_min_deg =
        manual_local_reloc_reseed_yaw_min_rad_ * 180.0 / kPi;
    pnh_.param("manual_local_reloc_reseed_yaw_min_deg",
               manual_local_reloc_reseed_yaw_min_deg,
               manual_local_reloc_reseed_yaw_min_deg);
    manual_local_reloc_reseed_yaw_min_rad_ =
        Deg2Rad(manual_local_reloc_reseed_yaw_min_deg);

    pnh_.param("local_reloc_min_score", local_reloc_common_opt_.min_score,
               local_reloc_common_opt_.min_score);
    pnh_.param("local_reloc_score_margin",
               local_reloc_common_opt_.score_margin,
               local_reloc_common_opt_.score_margin);
    pnh_.param("local_reloc_min_valid_fraction",
               local_reloc_common_opt_.min_valid_fraction,
               local_reloc_common_opt_.min_valid_fraction);
    pnh_.param("local_reloc_bnb_max_level",
               local_reloc_common_opt_.bnb_max_level,
               local_reloc_common_opt_.bnb_max_level);
    pnh_.param("local_reloc_pyr_max_level",
               local_reloc_common_opt_.pyr_max_level,
               local_reloc_common_opt_.pyr_max_level);
    pnh_.param("local_reloc_max_scan_points",
               local_reloc_common_opt_.max_scan_points,
               local_reloc_common_opt_.max_scan_points);
    pnh_.param("local_reloc_scan_voxel_m",
               local_reloc_common_opt_.scan_voxel_m,
               local_reloc_common_opt_.scan_voxel_m);
    pnh_.param("local_reloc_hit_sigma_m",
               local_reloc_common_opt_.hit_sigma_m,
               local_reloc_common_opt_.hit_sigma_m);
    pnh_.param("local_reloc_max_dist_m",
               local_reloc_common_opt_.max_dist_m,
               local_reloc_common_opt_.max_dist_m);
    pnh_.param("local_reloc_min_range",
               local_reloc_common_opt_.min_range,
               local_reloc_common_opt_.min_range);
    pnh_.param("local_reloc_max_range",
               local_reloc_common_opt_.max_range,
               local_reloc_common_opt_.max_range);
    double local_reloc_yaw_step_deg =
        local_reloc_common_opt_.yaw_step_rad * 180.0 / kPi;
    pnh_.param("local_reloc_yaw_step_deg",
               local_reloc_yaw_step_deg,
               local_reloc_yaw_step_deg);
    local_reloc_common_opt_.yaw_step_rad =
        Deg2Rad(local_reloc_yaw_step_deg);
    pnh_.param("local_reloc_xy_range_m", local_reloc_xy_range_m_,
               local_reloc_xy_range_m_);
    pnh_.param("local_reloc_yaw_search_deg", local_reloc_yaw_search_deg_,
               local_reloc_yaw_search_deg_);
    pnh_.param("local_reloc_settle_frames", local_reloc_settle_frames_,
               local_reloc_settle_frames_);

    pnh_.param("startup_local_reloc_enable", startup_local_reloc_enable_,
               startup_local_reloc_enable_);
    pnh_.param("startup_max_candidates_per_scan",
               startup_max_candidates_per_scan_,
               startup_max_candidates_per_scan_);
    pnh_.param("startup_fixed_retry_interval_sec",
               startup_fixed_retry_interval_sec_,
               startup_fixed_retry_interval_sec_);
    pnh_.param("startup_fixed_retry_stationary_min_duration",
               startup_fixed_retry_stationary_min_duration_,
               startup_fixed_retry_stationary_min_duration_);

    pnh_.param("lost_local_reloc_enable", lost_local_reloc_enable_,
               lost_local_reloc_enable_);
    pnh_.param("lost_local_reloc_min_interval_sec",
               lost_local_reloc_min_interval_sec_,
               lost_local_reloc_min_interval_sec_);
    pnh_.param("lost_local_reloc_expand_factor", lost_local_reloc_expand_factor_,
               lost_local_reloc_expand_factor_);
    pnh_.param("lost_local_reloc_max_expansions", lost_local_reloc_max_expansions_,
               lost_local_reloc_max_expansions_);
    pnh_.param("lost_fixed_reloc_enable", lost_fixed_reloc_enable_,
               lost_fixed_reloc_enable_);
    pnh_.param("lost_fixed_min_interval_sec", lost_fixed_min_interval_sec_,
               lost_fixed_min_interval_sec_);
    pnh_.param("lost_fixed_max_candidates_per_scan",
               lost_fixed_max_candidates_per_scan_,
               lost_fixed_max_candidates_per_scan_);
    pnh_.param("lost_fixed_stationary_min_duration",
               lost_fixed_stationary_min_duration_,
               lost_fixed_stationary_min_duration_);

    pnh_.param("auto_local_reloc_reseed_sigma_scale",
               auto_local_reloc_reseed_sigma_scale_,
               auto_local_reloc_reseed_sigma_scale_);
    pnh_.param("auto_local_reloc_reseed_xy_min_m",
               auto_local_reloc_reseed_xy_min_m_,
               auto_local_reloc_reseed_xy_min_m_);
    double auto_local_reloc_reseed_yaw_min_deg =
        auto_local_reloc_reseed_yaw_min_rad_ * 180.0 / kPi;
    pnh_.param("auto_local_reloc_reseed_yaw_min_deg",
               auto_local_reloc_reseed_yaw_min_deg,
               auto_local_reloc_reseed_yaw_min_deg);
    auto_local_reloc_reseed_yaw_min_rad_ =
        Deg2Rad(auto_local_reloc_reseed_yaw_min_deg);

    int random_seed = static_cast<int>(random_seed_);
    pnh_.param("random_seed", random_seed, random_seed);
    random_seed_ = static_cast<long>(random_seed);
    pnh_.param("odom_history_max_age_sec", odom_history_max_age_sec_,
               odom_history_max_age_sec_);
    pnh_.param("odom_history_max_size", odom_history_max_size_,
               odom_history_max_size_);
    pnh_.param("tf_lookup_timeout_sec", tf_lookup_timeout_sec_,
               tf_lookup_timeout_sec_);

    pnh_.param("state_good_best_min", state_cfg_.good_best_min,
               state_cfg_.good_best_min);
    pnh_.param("state_good_ambiguity_ratio_max",
               state_cfg_.good_ambiguity_ratio_max,
               state_cfg_.good_ambiguity_ratio_max);
    pnh_.param("state_good_max_clusters", state_cfg_.good_max_clusters,
               state_cfg_.good_max_clusters);
    pnh_.param("state_good_overall_sigma_xy_max",
               state_cfg_.good_overall_sigma_xy_max,
               state_cfg_.good_overall_sigma_xy_max);
    pnh_.param("state_good_best_sigma_yaw_deg_max",
               state_cfg_.good_best_sigma_yaw_deg_max,
               state_cfg_.good_best_sigma_yaw_deg_max);
    pnh_.param("state_good_jump_trans_max", state_cfg_.good_jump_trans_max,
               state_cfg_.good_jump_trans_max);
    pnh_.param("state_good_jump_yaw_deg_max",
               state_cfg_.good_jump_yaw_deg_max,
               state_cfg_.good_jump_yaw_deg_max);
    pnh_.param("state_ambiguous_min_clusters", state_cfg_.ambiguous_min_clusters,
               state_cfg_.ambiguous_min_clusters);
    pnh_.param("state_ambiguous_best_min", state_cfg_.ambiguous_best_min,
               state_cfg_.ambiguous_best_min);
    pnh_.param("state_ambiguous_ratio_min", state_cfg_.ambiguous_ratio_min,
               state_cfg_.ambiguous_ratio_min);
    pnh_.param("state_lost_best_max", state_cfg_.lost_best_max,
               state_cfg_.lost_best_max);
    pnh_.param("state_lost_cluster_min", state_cfg_.lost_cluster_min,
               state_cfg_.lost_cluster_min);
    pnh_.param("state_lost_overall_sigma_xy_min",
               state_cfg_.lost_overall_sigma_xy_min,
               state_cfg_.lost_overall_sigma_xy_min);
    pnh_.param("state_good_enter_frames", state_cfg_.good_enter_frames,
               state_cfg_.good_enter_frames);
    pnh_.param("state_weak_enter_frames", state_cfg_.weak_enter_frames,
               state_cfg_.weak_enter_frames);
    pnh_.param("state_ambiguous_enter_frames",
               state_cfg_.ambiguous_enter_frames,
               state_cfg_.ambiguous_enter_frames);
    pnh_.param("state_lost_enter_frames", state_cfg_.lost_enter_frames,
               state_cfg_.lost_enter_frames);
    pnh_.param("init_validate_good_frames", state_cfg_.init_validate_good_frames,
               state_cfg_.init_validate_good_frames);
    pnh_.param("init_validate_bad_frames", state_cfg_.init_validate_bad_frames,
               state_cfg_.init_validate_bad_frames);
    pnh_.param("init_accept_best_min", state_cfg_.init_accept_best_min,
               state_cfg_.init_accept_best_min);
    pnh_.param("init_accept_ambiguity_ratio_max",
               state_cfg_.init_accept_ambiguity_ratio_max,
               state_cfg_.init_accept_ambiguity_ratio_max);
    pnh_.param("init_accept_max_clusters", state_cfg_.init_accept_max_clusters,
               state_cfg_.init_accept_max_clusters);
    pnh_.param("init_accept_overall_sigma_xy_max",
               state_cfg_.init_accept_overall_sigma_xy_max,
               state_cfg_.init_accept_overall_sigma_xy_max);
    pnh_.param("init_reject_best_max", state_cfg_.init_reject_best_max,
               state_cfg_.init_reject_best_max);
    pnh_.param("init_reject_cluster_min", state_cfg_.init_reject_cluster_min,
               state_cfg_.init_reject_cluster_min);
    pnh_.param("init_reject_overall_sigma_xy_min",
               state_cfg_.init_reject_overall_sigma_xy_min,
               state_cfg_.init_reject_overall_sigma_xy_min);
    pnh_.param("last_pose_stationary_linear_speed_max",
               state_cfg_.last_pose_stationary_linear_speed_max,
               state_cfg_.last_pose_stationary_linear_speed_max);
    pnh_.param("last_pose_stationary_angular_speed_deg_max",
               state_cfg_.last_pose_stationary_angular_speed_deg_max,
               state_cfg_.last_pose_stationary_angular_speed_deg_max);
    pnh_.param("last_pose_stationary_min_duration",
               state_cfg_.last_pose_stationary_min_duration,
               state_cfg_.last_pose_stationary_min_duration);

    scan_voxel_m_ = std::max(0.005, scan_voxel_m_);
    max_scan_points_ = std::max(100, max_scan_points_);
    max_beams_ = std::max(2, max_beams_);
    min_particles_ = std::max(50, min_particles_);
    max_particles_ = std::max(min_particles_, max_particles_);
    pf_err_ = std::max(1e-6, pf_err_);
    pf_z_ = std::max(0.01, pf_z_);
    resample_interval_ = std::max(1, resample_interval_);
    sigma_hit_ = std::max(1e-3, sigma_hit_);
    laser_likelihood_max_dist_ = std::max(0.05, laser_likelihood_max_dist_);
    beam_skip_distance_ = std::max(0.0, beam_skip_distance_);
    beam_skip_threshold_ = Clamp(beam_skip_threshold_, 0.0, 1.0);
    beam_skip_error_threshold_ = Clamp(beam_skip_error_threshold_, 0.0, 1.0);
    z_hit_ = Clamp(z_hit_, 1e-6, 1.0);
    z_rand_ = Clamp(z_rand_, 0.0, 1.0);
    update_min_d_ = std::max(0.0, update_min_d_);
    update_min_a_ = std::max(0.0, std::min(kPi, update_min_a_));
    init_cov_xx_ = std::max(1e-6, init_cov_xx_);
    init_cov_yy_ = std::max(1e-6, init_cov_yy_);
    init_cov_aa_ = std::max(1e-6, init_cov_aa_);
    odom_history_max_age_sec_ = std::max(0.1, odom_history_max_age_sec_);
    odom_history_max_size_ = std::max(10, odom_history_max_size_);
    tf_lookup_timeout_sec_ = std::max(0.0, tf_lookup_timeout_sec_);
    if (odom_source_mode_ != "auto" && odom_source_mode_ != "tf" &&
        odom_source_mode_ != "topic") {
      ROS_WARN_STREAM("[AMCL] invalid odom_source_mode=" << odom_source_mode_
                      << ", fallback to auto");
      odom_source_mode_ = "auto";
    }
    particlecloud_max_samples_ = std::max(1, particlecloud_max_samples_);
    if (laser_min_range_ <= 0.0 && scan_min_range_ > 0.0) {
      laser_min_range_ = scan_min_range_;
    }
    if (laser_max_range_ <= 0.0 && scan_max_range_ > 0.0) {
      laser_max_range_ = scan_max_range_;
    }

    manual_local_reloc_coarse_xy_range_m_ =
        std::max(0.0, manual_local_reloc_coarse_xy_range_m_);
    manual_local_reloc_coarse_yaw_search_deg_ =
        Clamp(manual_local_reloc_coarse_yaw_search_deg_, 0.0, 180.0);
    manual_local_reloc_xy_min_m_ = std::max(0.0, manual_local_reloc_xy_min_m_);
    manual_local_reloc_xy_sigma_k_ =
        std::max(0.0, manual_local_reloc_xy_sigma_k_);
    manual_local_reloc_xy_max_m_ =
        std::max(manual_local_reloc_xy_min_m_, manual_local_reloc_xy_max_m_);
    manual_local_reloc_yaw_min_rad_ =
        std::max(0.0, std::min(kPi, manual_local_reloc_yaw_min_rad_));
    manual_local_reloc_yaw_sigma_k_ =
        std::max(0.0, manual_local_reloc_yaw_sigma_k_);
    manual_local_reloc_final_xy_range_m_ =
        std::max(0.0, manual_local_reloc_final_xy_range_m_);
    manual_local_reloc_final_yaw_search_deg_ =
        Clamp(manual_local_reloc_final_yaw_search_deg_, 0.0, 180.0);
    manual_local_reloc_final_yaw_step_rad_ =
        std::max(Deg2Rad(0.1), std::fabs(manual_local_reloc_final_yaw_step_rad_));
    manual_local_reloc_reseed_sigma_scale_ =
        Clamp(manual_local_reloc_reseed_sigma_scale_, 0.05, 1.0);
    manual_local_reloc_reseed_xy_min_m_ =
        std::max(0.01, manual_local_reloc_reseed_xy_min_m_);
    manual_local_reloc_reseed_yaw_min_rad_ =
        std::max(Deg2Rad(0.25), manual_local_reloc_reseed_yaw_min_rad_);

    startup_max_candidates_per_scan_ =
        std::max(1, startup_max_candidates_per_scan_);
    startup_fixed_retry_interval_sec_ =
        std::max(0.0, startup_fixed_retry_interval_sec_);
    startup_fixed_retry_stationary_min_duration_ =
        std::max(0.0, startup_fixed_retry_stationary_min_duration_);

    local_reloc_common_opt_.min_score =
        Clamp(local_reloc_common_opt_.min_score, 0.0, 1.0);
    local_reloc_common_opt_.score_margin =
        Clamp(local_reloc_common_opt_.score_margin, 0.0, 1.0);
    local_reloc_common_opt_.min_valid_fraction =
        Clamp(local_reloc_common_opt_.min_valid_fraction, 0.0, 1.0);
    local_reloc_common_opt_.bnb_max_level =
        std::max(0, local_reloc_common_opt_.bnb_max_level);
    local_reloc_common_opt_.pyr_max_level =
        std::max(0, local_reloc_common_opt_.pyr_max_level);
    local_reloc_common_opt_.max_scan_points =
        std::max(1, local_reloc_common_opt_.max_scan_points);
    local_reloc_common_opt_.scan_voxel_m =
        std::max(0.005, local_reloc_common_opt_.scan_voxel_m);
    local_reloc_common_opt_.hit_sigma_m =
        std::max(1e-3, local_reloc_common_opt_.hit_sigma_m);
    local_reloc_common_opt_.max_dist_m =
        std::max(0.0, local_reloc_common_opt_.max_dist_m);
    local_reloc_common_opt_.min_range =
        std::max(0.0, local_reloc_common_opt_.min_range);
    if (local_reloc_common_opt_.max_range > 0.0) {
      local_reloc_common_opt_.max_range =
          std::max(local_reloc_common_opt_.min_range,
                   local_reloc_common_opt_.max_range);
    }
    local_reloc_common_opt_.yaw_step_rad =
        std::max(Deg2Rad(0.25), std::fabs(local_reloc_common_opt_.yaw_step_rad));
    local_reloc_xy_range_m_ = std::max(0.0, local_reloc_xy_range_m_);
    local_reloc_yaw_search_deg_ = Clamp(local_reloc_yaw_search_deg_, 0.0, 180.0);
    local_reloc_settle_frames_ = std::max(0, local_reloc_settle_frames_);

    lost_local_reloc_min_interval_sec_ =
        std::max(0.0, lost_local_reloc_min_interval_sec_);
    lost_local_reloc_expand_factor_ =
        std::max(1.0, lost_local_reloc_expand_factor_);
    lost_local_reloc_max_expansions_ =
        std::max(0, lost_local_reloc_max_expansions_);
    lost_fixed_min_interval_sec_ =
        std::max(0.0, lost_fixed_min_interval_sec_);
    lost_fixed_max_candidates_per_scan_ =
        std::max(1, lost_fixed_max_candidates_per_scan_);
    lost_fixed_stationary_min_duration_ =
        std::max(0.0, lost_fixed_stationary_min_duration_);

    auto_local_reloc_reseed_sigma_scale_ =
        Clamp(auto_local_reloc_reseed_sigma_scale_, 0.05, 1.0);
    auto_local_reloc_reseed_xy_min_m_ =
        std::max(0.01, auto_local_reloc_reseed_xy_min_m_);
    auto_local_reloc_reseed_yaw_min_rad_ =
        std::max(Deg2Rad(0.25), auto_local_reloc_reseed_yaw_min_rad_);

    state_cfg_.good_best_min = Clamp(state_cfg_.good_best_min, 0.0, 1.0);
    state_cfg_.good_ambiguity_ratio_max =
        Clamp(state_cfg_.good_ambiguity_ratio_max, 0.0, 1.0);
    state_cfg_.good_max_clusters = std::max(1, state_cfg_.good_max_clusters);
    state_cfg_.good_overall_sigma_xy_max =
        std::max(0.0, state_cfg_.good_overall_sigma_xy_max);
    state_cfg_.good_best_sigma_yaw_deg_max =
        std::max(0.0, state_cfg_.good_best_sigma_yaw_deg_max);
    state_cfg_.good_jump_trans_max =
        std::max(0.0, state_cfg_.good_jump_trans_max);
    state_cfg_.good_jump_yaw_deg_max =
        std::max(0.0, state_cfg_.good_jump_yaw_deg_max);
    state_cfg_.ambiguous_min_clusters =
        std::max(2, state_cfg_.ambiguous_min_clusters);
    state_cfg_.ambiguous_best_min =
        Clamp(state_cfg_.ambiguous_best_min, 0.0, 1.0);
    state_cfg_.ambiguous_ratio_min =
        Clamp(state_cfg_.ambiguous_ratio_min, 0.0, 1.0);
    state_cfg_.lost_best_max = Clamp(state_cfg_.lost_best_max, 0.0, 1.0);
    state_cfg_.lost_cluster_min = std::max(1, state_cfg_.lost_cluster_min);
    state_cfg_.lost_overall_sigma_xy_min =
        std::max(0.0, state_cfg_.lost_overall_sigma_xy_min);
    state_cfg_.good_enter_frames = std::max(1, state_cfg_.good_enter_frames);
    state_cfg_.weak_enter_frames = std::max(1, state_cfg_.weak_enter_frames);
    state_cfg_.ambiguous_enter_frames =
        std::max(1, state_cfg_.ambiguous_enter_frames);
    state_cfg_.lost_enter_frames = std::max(1, state_cfg_.lost_enter_frames);
    state_cfg_.init_validate_good_frames =
        std::max(1, state_cfg_.init_validate_good_frames);
    state_cfg_.init_validate_bad_frames =
        std::max(1, state_cfg_.init_validate_bad_frames);
    state_cfg_.init_accept_best_min =
        Clamp(state_cfg_.init_accept_best_min, 0.0, 1.0);
    state_cfg_.init_accept_ambiguity_ratio_max =
        Clamp(state_cfg_.init_accept_ambiguity_ratio_max, 0.0, 1.0);
    state_cfg_.init_accept_max_clusters =
        std::max(1, state_cfg_.init_accept_max_clusters);
    state_cfg_.init_accept_overall_sigma_xy_max =
        std::max(0.0, state_cfg_.init_accept_overall_sigma_xy_max);
    state_cfg_.init_reject_best_max =
        Clamp(state_cfg_.init_reject_best_max, 0.0, 1.0);
    state_cfg_.init_reject_cluster_min =
        std::max(1, state_cfg_.init_reject_cluster_min);
    state_cfg_.init_reject_overall_sigma_xy_min =
        std::max(0.0, state_cfg_.init_reject_overall_sigma_xy_min);
    state_cfg_.last_pose_stationary_linear_speed_max =
        std::max(0.0, state_cfg_.last_pose_stationary_linear_speed_max);
    state_cfg_.last_pose_stationary_angular_speed_deg_max =
        std::max(0.0, state_cfg_.last_pose_stationary_angular_speed_deg_max);
    state_cfg_.last_pose_stationary_min_duration =
        std::max(0.0, state_cfg_.last_pose_stationary_min_duration);

    initializeInitialPoseManager_();
  }

  void loadMapOrThrow() {
    if (map_yaml_path_.empty()) {
      throw std::runtime_error("~map_yaml_path is empty");
    }

    std::string err;
    if (!map_.loadFromYaml(map_yaml_path_, &err)) {
      throw std::runtime_error("load map failed: " + err);
    }
    if (use_yaml_frame_id_) {
      map_frame_id_ = map_.yamlFrameId();
    }

    if (amcl_map_) {
      map_free(amcl_map_);
      amcl_map_ = nullptr;
    }
    const nav_msgs::OccupancyGrid map_msg =
        map_.makeOccupancyGrid(map_frame_id_, ros::Time::now());
    amcl_map_ = convertMap_(map_msg);
    createFreeSpaceVector_();
    if (!amcl_map_ || free_space_indices_.empty()) {
      throw std::runtime_error("AMCL map conversion failed or free space is empty");
    }

    if (manual_local_reloc_enable_) {
      manual_local_reloc_.reset(new LocalRelocalizer2D());
      manual_local_reloc_->setAllowExternalMapPaths(
          manual_local_reloc_allow_external_map_paths_);
      manual_local_reloc_->setPyrMaxLevel(local_reloc_common_opt_.pyr_max_level);
      manual_local_reloc_->setScoreParams(local_reloc_common_opt_.hit_sigma_m,
                                          local_reloc_common_opt_.max_dist_m);
      std::string reloc_err;
      if (!manual_local_relocalizerLoad_(&reloc_err)) {
        ROS_WARN_STREAM("[AMCL] manual local_reloc load failed, disable. err="
                        << reloc_err);
        manual_local_reloc_.reset();
        manual_local_reloc_enable_ = false;
      }
    }

    if (startup_local_reloc_enable_ || lost_local_reloc_enable_) {
      auto_local_reloc_.reset(new LocalRelocalizer2D());
      // Startup/lost local_reloc uses the same resolved map yaml as the main map
      // loader. Allow absolute image paths here so stale workspace prefixes in the
      // yaml don't disable the whole auto-recovery path.
      auto_local_reloc_->setAllowExternalMapPaths(true);
      auto_local_reloc_->setPyrMaxLevel(local_reloc_common_opt_.pyr_max_level);
      auto_local_reloc_->setScoreParams(local_reloc_common_opt_.hit_sigma_m,
                                        local_reloc_common_opt_.max_dist_m);
      std::string reloc_err;
      if (!auto_local_reloc_->loadMap(map_yaml_path_, &reloc_err)) {
        ROS_WARN_STREAM("[AMCL] auto local_reloc load failed, disable startup/lost. err="
                        << reloc_err);
        auto_local_reloc_.reset();
        startup_local_reloc_enable_ = false;
        lost_local_reloc_enable_ = false;
      }
    }
  }

  bool manual_local_relocalizerLoad_(std::string *err) {
    if (!manual_local_reloc_) {
      return false;
    }
    return manual_local_reloc_->loadMap(map_yaml_path_, err);
  }

  void initParticleFilterOrThrow() {
    if (!amcl_map_) {
      throw std::runtime_error("AMCL map is not ready");
    }

    if (pf_) {
      pf_free(pf_);
      pf_ = nullptr;
    }

    pf_ = pf_alloc(min_particles_, max_particles_, recovery_alpha_slow_,
                   recovery_alpha_fast_, &AmclLocalizerNode::UniformPoseGenerator);
    if (!pf_) {
      throw std::runtime_error("pf_alloc failed");
    }

    seedRandom_();
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    motion_model_.initialize(alpha1_, alpha2_, alpha3_, alpha4_, alpha5_);

    if (laser_model_type_ != "likelihood_field_prob" &&
        laser_model_type_ != "likelihood_field") {
      ROS_WARN_STREAM("[AMCL] unsupported laser_model_type=" << laser_model_type_
                      << ", fallback to likelihood_field_prob");
    }

    if (laser_model_type_ == "likelihood_field") {
      laser_model_.reset(new nav2_amcl::LikelihoodFieldModel(
          z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_,
          static_cast<size_t>(max_beams_), amcl_map_));
    } else {
      laser_model_.reset(new nav2_amcl::LikelihoodFieldModelProb(
          z_hit_, z_rand_, sigma_hit_, laser_likelihood_max_dist_, do_beamskip_,
          beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_,
          static_cast<size_t>(max_beams_), amcl_map_));
    }

    initialized_ = false;
    initial_pose_is_known_ = false;
    force_update_ = true;
    first_pose_sent_ = false;
    has_last_estimate_ = false;
    pf_odom_initialized_ = false;
    resample_count_ = 0;
    loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_ = AmclLocState::UNINITIALIZED;
    prev_raw_loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_streak_ = 0;
  }

  void seedRandom_() {
    if (random_seed_ >= 0) {
      srand48(random_seed_);
    } else {
      srand48(static_cast<long>(std::time(nullptr)));
    }
  }

  void initRos() {
    odom_sub_ =
        nh_.subscribe(odom_topic_, 200, &AmclLocalizerNode::odomCallback, this);
    scan_sub_ =
        nh_.subscribe(scan_topic_, 10, &AmclLocalizerNode::scanCallback, this);
    initialpose_sub_ = nh_.subscribe(initialpose_topic_, 2,
                                     &AmclLocalizerNode::initialPoseCallback,
                                     this);

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic_, 10, false);
    amcl_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
        amcl_pose_topic_, 10, false);
    if (publish_particlecloud_) {
      particlecloud_pub_ =
          nh_.advertise<geometry_msgs::PoseArray>(particlecloud_topic_, 5, false);
    }
    if (publish_map_) {
      map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_topic_, 1, true);
      map_metadata_pub_ =
          nh_.advertise<nav_msgs::MapMetaData>(map_metadata_topic_, 1, true);
    }
  }

  void publishMap() {
    if (!publish_map_) {
      return;
    }
    const ros::Time stamp = ros::Time::now();
    nav_msgs::OccupancyGrid map_msg = map_.makeOccupancyGrid(map_frame_id_, stamp);
    map_pub_.publish(map_msg);
    map_metadata_pub_.publish(map_msg.info);
  }

  void maybeInitializeOnStartup() {
    buildStartupInitSeedCandidates_();
    if (!activateNextInitSeed_()) {
      ROS_WARN_STREAM("[AMCL] no startup init seed available");
    }
  }

  void logConfig() const {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "\n================ AMCL Localizer ================\n"
        << "run     : map=" << map_name_ << "\n"
        << "config  : interface=" << interface_yaml_path_
        << " params=" << params_yaml_path_ << "\n"
        << "map_yaml=" << map_.yamlPath() << "\n"
        << "image   =" << map_.imagePath() << "\n"
        << "topics  : scan=" << scan_topic_ << " odom=" << odom_topic_
        << " imu=" << imu_topic_ << " initialpose=" << initialpose_topic_ << "\n"
        << "pub     : odom=" << odom_pub_topic_ << " pose=" << amcl_pose_topic_
        << " particles=" << particlecloud_topic_ << "\n"
        << "frames  : map=" << map_frame_id_ << " base=" << base_frame_id_
        << " odom=" << odom_frame_id_ << " yaml=" << map_.yamlFrameId()
        << " use_yaml=" << (use_yaml_frame_id_ ? 1 : 0)
        << " tf_mode=" << (publish_map_to_odom_tf_ ? "map->odom" : "map->base")
        << "\n"
        << "odom    : source_mode=" << odom_source_mode_
        << " history=(" << odom_history_max_age_sec_ << "s,"
        << odom_history_max_size_ << " msgs)\n"
        << "map     : size=" << map_.width() << "x" << map_.height()
        << " res=" << map_.resolution() << " free=" << map_.freeCellCount()
        << " occ=" << map_.occupiedCellCount()
        << " unk=" << map_.unknownCellCount() << "\n"
        << "pf      : min=" << min_particles_ << " max=" << max_particles_
        << " pf_err=" << pf_err_ << " pf_z=" << pf_z_
        << " alpha_fast=" << recovery_alpha_fast_
        << " alpha_slow=" << recovery_alpha_slow_
        << " resample_interval=" << resample_interval_
        << " global_init=" << (global_init_on_startup_ ? 1 : 0) << "\n"
        << "laser   : model=" << laser_model_type_
        << " beams=" << max_beams_
        << " min=" << laser_min_range_ << " max=" << laser_max_range_
        << " sigma=" << sigma_hit_
        << " max_occ=" << laser_likelihood_max_dist_
        << " beamskip=" << (do_beamskip_ ? 1 : 0) << "\n"
        << "motion  : alpha=(" << alpha1_ << "," << alpha2_ << "," << alpha3_
        << "," << alpha4_ << "," << alpha5_ << ")"
        << " update_min=(" << update_min_d_ << "m,"
        << update_min_a_ * 180.0 / kPi << "deg)\n"
        << "manual  : local_reloc=" << (manual_local_reloc_enable_ ? 1 : 0)
        << " coarse=(" << manual_local_reloc_coarse_xy_range_m_ << "m,"
        << manual_local_reloc_coarse_yaw_search_deg_ << "deg)"
        << " win(cov)=max(" << manual_local_reloc_xy_min_m_ << "m,"
        << manual_local_reloc_xy_sigma_k_ << "*sigma_xy)"
        << " <= " << manual_local_reloc_xy_max_m_ << "m"
        << " yaw=max(" << manual_local_reloc_yaw_min_rad_ * 180.0 / kPi
        << "deg," << manual_local_reloc_yaw_sigma_k_ << "*sigma_yaw)"
        << " final=(" << manual_local_reloc_final_xy_range_m_ << "m,"
        << manual_local_reloc_final_yaw_search_deg_ << "deg"
        << "@" << manual_local_reloc_final_yaw_step_rad_ * 180.0 / kPi << "deg)"
        << " reseed_scale=" << manual_local_reloc_reseed_sigma_scale_
        << " settle=" << local_reloc_settle_frames_ << "\n"
        << "local   : score>=" << local_reloc_common_opt_.min_score
        << " margin>=" << local_reloc_common_opt_.score_margin
        << " vf>=" << local_reloc_common_opt_.min_valid_fraction
        << " bnb=" << local_reloc_common_opt_.bnb_max_level
        << " pyr=" << local_reloc_common_opt_.pyr_max_level
        << " yaw_step=" << local_reloc_common_opt_.yaw_step_rad * 180.0 / kPi
        << "deg"
        << " pts<=" << local_reloc_common_opt_.max_scan_points
        << " win=(" << local_reloc_xy_range_m_ << "m,"
        << local_reloc_yaw_search_deg_ << "deg)\n"
        << "startup : local_reloc=" << (startup_local_reloc_enable_ ? 1 : 0)
        << " K=" << startup_max_candidates_per_scan_ << "\n"
        << "lost    : local_reloc=" << (lost_local_reloc_enable_ ? 1 : 0)
        << " dt>=" << lost_local_reloc_min_interval_sec_
        << "s expand=" << lost_local_reloc_expand_factor_
        << "^" << lost_local_reloc_max_expansions_ << "\n"
        << "lostfix : enable=" << (lost_fixed_reloc_enable_ ? 1 : 0)
        << " dt>=" << lost_fixed_min_interval_sec_
        << "s K=" << lost_fixed_max_candidates_per_scan_
        << " stop>=" << lost_fixed_stationary_min_duration_ << "s\n"
        << "state   : good(best>=" << state_cfg_.good_best_min
        << " amb<=" << state_cfg_.good_ambiguity_ratio_max
        << " c<=" << state_cfg_.good_max_clusters
        << " ov_xy<=" << state_cfg_.good_overall_sigma_xy_max
        << " jump=(" << state_cfg_.good_jump_trans_max << "m,"
        << state_cfg_.good_jump_yaw_deg_max << "deg))"
        << " amb(c>=" << state_cfg_.ambiguous_min_clusters
        << " best>=" << state_cfg_.ambiguous_best_min
        << " ratio>=" << state_cfg_.ambiguous_ratio_min << ")"
        << " lost(best<=" << state_cfg_.lost_best_max
        << " c>=" << state_cfg_.lost_cluster_min
        << " ov_xy>=" << state_cfg_.lost_overall_sigma_xy_min << ")"
        << " init(good=" << state_cfg_.init_validate_good_frames
        << " bad=" << state_cfg_.init_validate_bad_frames << ")"
        << " last(v<=" << state_cfg_.last_pose_stationary_linear_speed_max
        << " w_deg<=" << state_cfg_.last_pose_stationary_angular_speed_deg_max
        << " dt>=" << state_cfg_.last_pose_stationary_min_duration << ")\n"
        << "================================================";
    ROS_INFO_STREAM(oss.str());
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &msg) {
    if (!msg) {
      return;
    }

    nav_msgs::Odometry odom = *msg;
    if (odom.header.stamp.isZero()) {
      odom.header.stamp = ros::Time::now();
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_msg_ = odom;
    latest_odom_pose_ = odomMsgToPose2D(odom);
    has_latest_odom_ = true;

    odom_history_.push_back(odom);
    while (!odom_history_.empty() &&
           (odom_history_.back().header.stamp - odom_history_.front().header.stamp)
                   .toSec() > odom_history_max_age_sec_) {
      odom_history_.pop_front();
    }
    while (static_cast<int>(odom_history_.size()) > odom_history_max_size_) {
      odom_history_.pop_front();
    }
  }

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    if (!msg) {
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_id_) {
      ROS_WARN_STREAM("[AMCL] initialpose frame_id=" << msg->header.frame_id
                                                      << " expected map frame "
                                                      << map_frame_id_
                                                      << ", assume map");
    }

    Pose2D p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;
    p.yaw = QuaternionToYaw(msg->pose.pose.orientation);

    const auto &cov = msg->pose.covariance;
    const double cov_xx = std::max(init_cov_xx_, cov[0]);
    const double cov_yy = std::max(init_cov_yy_, cov[7]);
    const double cov_aa = std::max(init_cov_aa_, cov[35]);
    const double sigma_x = std::sqrt(cov_xx);
    const double sigma_y = std::sqrt(cov_yy);
    const double sigma_yaw = std::sqrt(cov_aa);

    PendingInitialPose req;
    req.valid = true;
    req.pose = p;
    req.cov_xx = cov_xx;
    req.cov_yy = cov_yy;
    req.cov_aa = cov_aa;
    req.sigma_x = sigma_x;
    req.sigma_y = sigma_y;
    req.sigma_yaw = sigma_yaw;
    req.stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pending_initial_pose_ = req;
    }
    work_cv_.notify_one();
  }

  void startWorker_() {
    worker_stop_ = false;
    worker_thread_ = std::thread(&AmclLocalizerNode::workerLoop_, this);
  }

  void stopWorker_() {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      worker_stop_ = true;
    }
    work_cv_.notify_all();
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  void workerLoop_() {
    while (true) {
      sensor_msgs::LaserScanConstPtr scan_msg;
      PendingInitialPose init_req;

      {
        std::unique_lock<std::mutex> lock(state_mutex_);
        work_cv_.wait(lock, [this]() {
          return worker_stop_ || pending_scan_ || pending_initial_pose_.valid;
        });
        if (worker_stop_) {
          return;
        }
        scan_msg = pending_scan_;
        pending_scan_.reset();
        init_req = pending_initial_pose_;
        pending_initial_pose_.valid = false;
      }

      if (init_req.valid) {
        handleInitialPoseRequest_(init_req);
      }
      if (scan_msg) {
        processScan_(scan_msg);
      }
    }
  }

  void handleInitialPoseRequest_(const PendingInitialPose &req) {
    clearRelocRefine_();
    initializeFilterAtPose_(req.pose, req.cov_xx, req.cov_yy, req.cov_aa,
                            "manual_initialpose");
    queueManualLocalRelocRequest_(req.pose, req.sigma_x, req.sigma_y,
                                  req.sigma_yaw);

    Pose2D odom_pose;
    const bool have_odom = getBasePoseInOdom_(req.stamp, &odom_pose);
    publishCurrentEstimate_(req.stamp, have_odom ? &odom_pose : nullptr);
  }

  void processScan_(const sensor_msgs::LaserScanConstPtr &scan_msg) {
    if (!scan_msg) {
      return;
    }

    const ros::Time stamp =
        scan_msg->header.stamp.isZero() ? ros::Time::now() : scan_msg->header.stamp;
    if (!pf_ || !amcl_map_ || !laser_model_) {
      reportQualitySnapshot_(stamp, 0, nullptr, false);
      ROS_WARN_THROTTLE(1.0, "[AMCL] core is not ready");
      return;
    }

    LaserMount mount;
    const std::vector<Eigen::Vector2f> scan_xy_base =
        buildScanPointsBase(scan_msg, &mount);
    if (scan_xy_base.empty()) {
      reportQualitySnapshot_(stamp, 0, nullptr, false);
      ROS_WARN_THROTTLE(1.0, "[AMCL] scan has no usable points");
      return;
    }

    Pose2D odom_pose;
    bool used_tf = false;
    const bool odom_ok = getBasePoseInOdom_(stamp, &odom_pose, &used_tf);

    if (frontend_mode_ == FrontendMode::RELOC_REFINE && reloc_refine_.active) {
      if (!odom_ok) {
        reportQualitySnapshot_(stamp, static_cast<int>(scan_xy_base.size()), nullptr,
                               false);
        ROS_WARN_THROTTLE(1.0, "[AMCL] odom pose unavailable during reloc_refine");
        return;
      }
      (void)stepRelocRefine_(scan_xy_base, odom_pose, stamp);
      publishCurrentEstimate_(stamp, &odom_pose);
      return;
    }

    if (runManualLocalRelocIfRequested_(scan_xy_base,
                                        odom_ok ? &odom_pose : nullptr)) {
      if (odom_ok) {
        publishCurrentEstimate_(stamp, &odom_pose);
      }
      return;
    }
    (void)maybeActivateStartupFixedRetryCycle_(stamp);
    if (runStartupLocalRelocIfRequested_(scan_xy_base,
                                         odom_ok ? &odom_pose : nullptr)) {
      if (odom_ok) {
        publishCurrentEstimate_(stamp, &odom_pose);
      }
      return;
    }

    if (!initialized_ || !initial_pose_is_known_) {
      reportQualitySnapshot_(stamp, static_cast<int>(scan_xy_base.size()), nullptr,
                             false);
      ROS_WARN_THROTTLE(1.0,
                        "[AMCL] initial pose unknown, waiting for startup or /initialpose");
      return;
    }

    if (!odom_ok) {
      reportQualitySnapshot_(stamp, static_cast<int>(scan_xy_base.size()), nullptr,
                             false);
      ROS_WARN_THROTTLE(1.0, "[AMCL] odom pose unavailable");
      return;
    }

    if (runLostLocalRelocIfNeeded_(scan_xy_base, stamp, odom_pose)) {
      publishCurrentEstimate_(stamp, &odom_pose);
      return;
    }
    if (runLostFixedRelocIfNeeded_(scan_xy_base, odom_pose, stamp)) {
      publishCurrentEstimate_(stamp, &odom_pose);
      return;
    }

    applyLaserMount_(scan_msg->header.frame_id, mount);

    const pf_vector_t pose = poseToPfVector(odom_pose);
    pf_vector_t delta = pf_vector_zero();
    bool do_sensor_update = false;
    bool do_motion_update = false;

    if (!pf_odom_initialized_) {
      pf_odom_pose_ = pose;
      pf_odom_initialized_ = true;
      resample_count_ = 0;
      do_sensor_update = true;
    } else {
      do_sensor_update = shouldUpdateFilter_(pose, &delta);
      do_motion_update = do_sensor_update;
    }

    if (do_motion_update) {
      motion_model_.odometryUpdate(pf_, pose, delta);
    }

    if (do_sensor_update) {
      if (!updateFilter_(scan_msg, mount, pose)) {
        ROS_WARN_THROTTLE(1.0, "[AMCL] sensor update failed");
        reportQualitySnapshot_(stamp, static_cast<int>(scan_xy_base.size()), &odom_pose,
                               false);
        publishStoredEstimate_(stamp, &odom_pose);
        return;
      }

      ++resample_count_;
      if (resample_interval_ > 0 && (resample_count_ % resample_interval_) == 0) {
        pf_update_resample(pf_, this);
      }
      consumeLocalRelocSettlingFrame_();
      force_update_ = false;
      reportQualitySnapshot_(stamp, static_cast<int>(scan_xy_base.size()), &odom_pose,
                             true);
      publishCurrentEstimate_(stamp, &odom_pose);
      maybeUpdateLastGoodReference_();
      maybeWriteLastPose_(stamp);
      return;
    }

    reportQualitySnapshot_(stamp, static_cast<int>(scan_xy_base.size()), &odom_pose,
                           true);
    if (used_tf && publish_map_to_odom_tf_ && has_last_map_to_odom_) {
      publishStoredEstimate_(stamp, &odom_pose);
    } else if (has_last_estimate_ && has_last_map_to_odom_) {
      publishStoredEstimate_(stamp, &odom_pose);
    } else {
      publishCurrentEstimate_(stamp, &odom_pose);
    }
    maybeUpdateLastGoodReference_();
    maybeWriteLastPose_(stamp);
  }

  void queueManualLocalRelocRequest_(const Pose2D &pose, double sigma_x,
                                     double sigma_y, double sigma_yaw) {
    manual_local_reloc_pending_ = false;
    if (!manual_local_reloc_enable_) {
      return;
    }

    const double sigma_xy = std::max(sigma_x, sigma_y);
    manual_local_reloc_center_.x = pose.x;
    manual_local_reloc_center_.y = pose.y;
    manual_local_reloc_center_.yaw = pose.yaw;
    manual_local_reloc_xy_prior_ref_m_ = std::max(0.3, 2.0 * sigma_xy);
    manual_local_reloc_yaw_prior_ref_rad_ =
        std::max(Deg2Rad(10.0), 2.0 * sigma_yaw);
    manual_local_reloc_xy_range_m_ =
        std::max(manual_local_reloc_xy_min_m_,
                 manual_local_reloc_xy_sigma_k_ * sigma_xy);
    manual_local_reloc_xy_range_m_ =
        std::min(manual_local_reloc_xy_max_m_, manual_local_reloc_xy_range_m_);
    manual_local_reloc_yaw_range_rad_ =
        std::max(manual_local_reloc_yaw_min_rad_,
                 manual_local_reloc_yaw_sigma_k_ * sigma_yaw);
    manual_local_reloc_yaw_range_rad_ =
        std::min(kPi, manual_local_reloc_yaw_range_rad_);
    manual_local_reloc_pending_ = true;

    ROS_INFO_STREAM("[AMCL] manual local_reloc queued: center=("
                    << pose.x << "," << pose.y << ","
                    << pose.yaw * 180.0 / kPi << "deg)"
                    << " win=(" << manual_local_reloc_xy_range_m_ << "m,"
                    << manual_local_reloc_yaw_range_rad_ * 180.0 / kPi
                    << "deg)");
  }

  bool lookupLaserMount_(const sensor_msgs::LaserScanConstPtr &scan_msg,
                         LaserMount *mount) {
    if (!scan_msg || !mount) {
      return false;
    }

    geometry_msgs::TransformStamped tf_bl;
    try {
      tf_bl = tf_buffer_.lookupTransform(
          base_frame_id_, scan_msg->header.frame_id, ros::Time(0),
          ros::Duration(tf_lookup_timeout_sec_));
    } catch (...) {
      try {
        tf_bl = tf_buffer_.lookupTransform(
            base_frame_id_, scan_msg->header.frame_id, scan_msg->header.stamp,
            ros::Duration(tf_lookup_timeout_sec_));
      } catch (const std::exception &e) {
        ROS_WARN_STREAM_THROTTLE(1.0,
                                 "[AMCL] lookup base<-scan failed: " << e.what());
        return false;
      }
    }

    mount->x = tf_bl.transform.translation.x;
    mount->y = tf_bl.transform.translation.y;
    mount->yaw = QuaternionToYaw(tf_bl.transform.rotation);
    return true;
  }

  void applyLaserMount_(const std::string &frame_id, const LaserMount &mount) {
    if (!laser_model_) {
      return;
    }

    pf_vector_t laser_pose = pf_vector_zero();
    laser_pose.v[0] = mount.x;
    laser_pose.v[1] = mount.y;
    laser_pose.v[2] = 0.0;
    laser_model_->SetLaserPose(laser_pose);
    laser_frame_id_ = frame_id;
    laser_mount_yaw_ = mount.yaw;
    laser_pose_ready_ = true;
  }

  std::vector<Eigen::Vector2f>
  buildScanPointsBase(const sensor_msgs::LaserScanConstPtr &scan_msg,
                      LaserMount *mount) {
    std::vector<Eigen::Vector2f> out;
    if (!scan_msg || !mount) {
      return out;
    }

    if (!lookupLaserMount_(scan_msg, mount)) {
      return out;
    }

    out.reserve(std::min(max_scan_points_,
                         static_cast<int>(scan_msg->ranges.size())));

    const double c = std::cos(mount->yaw);
    const double s = std::sin(mount->yaw);
    const double min_range_param =
        (scan_min_range_ > 0.0) ? scan_min_range_ : laser_min_range_;
    const double max_range_param =
        (scan_max_range_ > 0.0) ? scan_max_range_ : laser_max_range_;
    const double rmin = (min_range_param > 0.0)
                            ? std::max(min_range_param,
                                       static_cast<double>(scan_msg->range_min))
                            : static_cast<double>(scan_msg->range_min);
    const double rmax_raw = static_cast<double>(scan_msg->range_max);
    const double rmax = (max_range_param > 0.0)
                            ? std::min(max_range_param, rmax_raw)
                            : rmax_raw;

    auto pack = [](int ix, int iy) -> uint64_t {
      return (uint64_t(uint32_t(ix)) << 32) | uint32_t(iy);
    };
    std::unordered_set<uint64_t> used;
    used.reserve(static_cast<std::size_t>(max_scan_points_) * 2);

    for (std::size_t i = 0; i < scan_msg->ranges.size(); ++i) {
      const float r = scan_msg->ranges[i];
      if (!std::isfinite(r) || r < rmin || r > rmax) {
        continue;
      }
      const double ang = static_cast<double>(scan_msg->angle_min) +
                         static_cast<double>(i) *
                             static_cast<double>(scan_msg->angle_increment);
      const double lx = static_cast<double>(r) * std::cos(ang);
      const double ly = static_cast<double>(r) * std::sin(ang);
      const double bx = c * lx - s * ly + mount->x;
      const double by = s * lx + c * ly + mount->y;

      const int ix = static_cast<int>(std::floor(bx / scan_voxel_m_));
      const int iy = static_cast<int>(std::floor(by / scan_voxel_m_));
      const uint64_t key = pack(ix, iy);
      if (!used.insert(key).second) {
        continue;
      }

      out.emplace_back(static_cast<float>(bx), static_cast<float>(by));
      if (static_cast<int>(out.size()) >= max_scan_points_) {
        break;
      }
    }

    return out;
  }

  bool getOdomPoseAt_(const ros::Time &stamp, Pose2D *pose) const {
    if (!pose) {
      return false;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (odom_history_.empty()) {
      return false;
    }

    if (stamp.isZero() || odom_history_.size() == 1) {
      *pose = latest_odom_pose_;
      return has_latest_odom_;
    }

    if (stamp <= odom_history_.front().header.stamp) {
      *pose = odomMsgToPose2D(odom_history_.front());
      return true;
    }
    if (stamp >= odom_history_.back().header.stamp) {
      *pose = odomMsgToPose2D(odom_history_.back());
      return true;
    }

    for (std::size_t i = 1; i < odom_history_.size(); ++i) {
      const nav_msgs::Odometry &b = odom_history_[i];
      if (b.header.stamp < stamp) {
        continue;
      }
      const nav_msgs::Odometry &a = odom_history_[i - 1];
      const double dt = (b.header.stamp - a.header.stamp).toSec();
      if (dt <= 1e-9) {
        *pose = odomMsgToPose2D(b);
        return true;
      }

      const double t = Clamp((stamp - a.header.stamp).toSec() / dt, 0.0, 1.0);
      const Pose2D pa = odomMsgToPose2D(a);
      const Pose2D pb = odomMsgToPose2D(b);
      pose->x = pa.x + t * (pb.x - pa.x);
      pose->y = pa.y + t * (pb.y - pa.y);
      pose->yaw = NormalizeAngle(pa.yaw + t * NormalizeAngle(pb.yaw - pa.yaw));
      return true;
    }

    *pose = latest_odom_pose_;
    return has_latest_odom_;
  }

  bool lookupBaseInOdomTf_(const ros::Time &stamp, Pose2D *pose) const {
    if (!pose) {
      return false;
    }

    geometry_msgs::TransformStamped tf_ob;
    try {
      tf_ob = tf_buffer_.lookupTransform(
          odom_frame_id_, base_frame_id_, stamp.isZero() ? ros::Time(0) : stamp,
          ros::Duration(tf_lookup_timeout_sec_));
    } catch (const std::exception &e) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[AMCL] lookup odom<-base failed: " << e.what());
      return false;
    }

    pose->x = tf_ob.transform.translation.x;
    pose->y = tf_ob.transform.translation.y;
    pose->yaw = QuaternionToYaw(tf_ob.transform.rotation);
    return true;
  }

  bool getBasePoseInOdom_(const ros::Time &stamp, Pose2D *pose,
                          bool *used_tf = nullptr) const {
    if (used_tf) {
      *used_tf = false;
    }

    const bool try_tf = (odom_source_mode_ == "auto" || odom_source_mode_ == "tf");
    if (try_tf && lookupBaseInOdomTf_(stamp, pose)) {
      if (used_tf) {
        *used_tf = true;
      }
      return true;
    }
    if (odom_source_mode_ == "tf") {
      return false;
    }
    return getOdomPoseAt_(stamp, pose);
  }

  void initializeFilterAtPose_(const Pose2D &pose, double cov_xx, double cov_yy,
                               double cov_aa, const std::string &reason) {
    if (!pf_) {
      return;
    }

    pf_vector_t mean = poseToPfVector(pose);
    pf_matrix_t cov = pf_matrix_zero();
    cov.m[0][0] = std::max(1e-9, cov_xx);
    cov.m[1][1] = std::max(1e-9, cov_yy);
    cov.m[2][2] = std::max(1e-9, cov_aa);
    pf_init(pf_, mean, cov);

    initialized_ = true;
    initial_pose_is_known_ = true;
    force_update_ = true;
    pf_odom_initialized_ = false;
    resample_count_ = 0;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (has_latest_odom_) {
        last_map_to_odom_ = ComposePose(pose, InversePose(latest_odom_pose_));
        has_last_map_to_odom_ = true;
      }
    }

    startup_local_reloc_pending_ = false;
    last_lost_local_reloc_attempt_ = ros::Time(0);
    lost_local_reloc_fail_count_ = 0;
    startup_fixed_retry_waiting_ = false;
    startup_fixed_retry_stationary_start_ = ros::Time(0);
    startup_last_fixed_retry_cycle_start_ = ros::Time(0);
    clearLostFixedRecoveryTracking_();
    clearLocalRelocSettling_();
    loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_ = AmclLocState::UNINITIALIZED;
    prev_raw_loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_streak_ = 0;

    ROS_INFO_STREAM("[AMCL] initialize at pose, reason=" << reason
                    << " pose=(" << pose.x << "," << pose.y << ","
                    << pose.yaw * 180.0 / kPi << "deg)"
                    << " cov=(" << cov_xx << "," << cov_yy << "," << cov_aa
                    << ")");
  }

  void globalLocalization_(const std::string &reason) {
    if (!pf_) {
      return;
    }
    pf_init_model(pf_, &AmclLocalizerNode::UniformPoseGenerator, this);
    initialized_ = true;
    initial_pose_is_known_ = true;
    force_update_ = true;
    pf_odom_initialized_ = false;
    resample_count_ = 0;
    has_last_map_to_odom_ = false;
    startup_local_reloc_pending_ = false;
    last_lost_local_reloc_attempt_ = ros::Time(0);
    lost_local_reloc_fail_count_ = 0;
    startup_fixed_retry_waiting_ = false;
    startup_fixed_retry_stationary_start_ = ros::Time(0);
    startup_last_fixed_retry_cycle_start_ = ros::Time(0);
    clearLostFixedRecoveryTracking_();
    clearLocalRelocSettling_();
    loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_ = AmclLocState::UNINITIALIZED;
    prev_raw_loc_state_ = AmclLocState::UNINITIALIZED;
    raw_loc_state_streak_ = 0;

    ROS_INFO_STREAM("[AMCL] global initialization, reason=" << reason
                    << " free_cells=" << free_space_indices_.size());
  }

  bool runManualLocalRelocIfRequested_(
      const std::vector<Eigen::Vector2f> &scan_xy_base,
      const Pose2D *odom_pose) {
    if (!manual_local_reloc_pending_) {
      return false;
    }

    manual_local_reloc_pending_ = false;
    if (!manual_local_reloc_enable_ || !manual_local_reloc_ ||
        !manual_local_reloc_->isReady()) {
      ROS_WARN_STREAM("[AMCL] manual local_reloc unavailable, keep dragged pose");
      return false;
    }

    LocalRelocOptions coarse_opt = local_reloc_common_opt_;
    coarse_opt.xy_range_m = std::max(manual_local_reloc_xy_range_m_,
                                     manual_local_reloc_coarse_xy_range_m_);
    coarse_opt.yaw_range_rad = std::max(manual_local_reloc_yaw_range_rad_,
                                        manualLocalRelocCoarseYawRangeRad_());

    const LocalRelocResult coarse =
        manual_local_reloc_->match(scan_xy_base, manual_local_reloc_center_,
                                   coarse_opt);
    if (!coarse.ok) {
      ROS_WARN_STREAM("[AMCL] manual coarse local_reloc failed, keep dragged pose. best="
                      << coarse.best_score << " second=" << coarse.second_score
                      << " vf=" << coarse.valid_fraction);
      return false;
    }

    LocalRelocOptions fine_opt = local_reloc_common_opt_;
    fine_opt.xy_range_m = manual_local_reloc_xy_range_m_;
    fine_opt.yaw_range_rad = manual_local_reloc_yaw_range_rad_;

    RelocPose2D fine_center;
    fine_center.x = coarse.best_pose.x;
    fine_center.y = coarse.best_pose.y;
    fine_center.yaw = coarse.best_pose.yaw;

    LocalRelocResult chosen = coarse;
    bool used_fine = false;
    bool used_final = false;
    double chosen_xy_range_m = coarse_opt.xy_range_m;
    double chosen_yaw_range_rad = coarse_opt.yaw_range_rad;
    double chosen_combined = manualRelocCombinedScore_(coarse);
    if ((fine_opt.xy_range_m + 1e-6) < coarse_opt.xy_range_m ||
        (fine_opt.yaw_range_rad + 1e-6) < coarse_opt.yaw_range_rad) {
      const LocalRelocResult fine =
          manual_local_reloc_->match(scan_xy_base, fine_center, fine_opt);
      if (fine.ok) {
        const double fine_combined = manualRelocCombinedScore_(fine);
        if (fine_combined >= chosen_combined) {
          chosen = fine;
          used_fine = true;
          chosen_xy_range_m = fine_opt.xy_range_m;
          chosen_yaw_range_rad = fine_opt.yaw_range_rad;
          chosen_combined = fine_combined;
        }
      } else {
        ROS_WARN_STREAM("[AMCL] manual fine local_reloc failed, fallback to coarse. best="
                        << fine.best_score << " second=" << fine.second_score
                        << " vf=" << fine.valid_fraction);
      }
    }

    LocalRelocOptions final_opt = local_reloc_common_opt_;
    final_opt.xy_range_m = manual_local_reloc_final_xy_range_m_;
    final_opt.yaw_range_rad = manualLocalRelocFinalYawRangeRad_();
    final_opt.yaw_step_rad = manual_local_reloc_final_yaw_step_rad_;
    if ((final_opt.xy_range_m + 1e-6) < chosen_xy_range_m ||
        (final_opt.yaw_range_rad + 1e-6) < chosen_yaw_range_rad) {
      RelocPose2D final_center;
      final_center.x = chosen.best_pose.x;
      final_center.y = chosen.best_pose.y;
      final_center.yaw = chosen.best_pose.yaw;
      const LocalRelocResult final =
          manual_local_reloc_->match(scan_xy_base, final_center, final_opt);
      if (final.ok) {
        const double final_combined = manualRelocCombinedScore_(final);
        if (final_combined >= chosen_combined) {
          chosen = final;
          used_final = true;
          chosen_xy_range_m = final_opt.xy_range_m;
          chosen_yaw_range_rad = final_opt.yaw_range_rad;
          chosen_combined = final_combined;
        }
      } else {
        ROS_WARN_STREAM("[AMCL] manual final local_reloc failed, fallback to "
                        << (used_fine ? "fine" : "coarse")
                        << ". best=" << final.best_score
                        << " second=" << final.second_score
                        << " vf=" << final.valid_fraction);
      }
    }

    Pose2D refined;
    refined.x = chosen.best_pose.x;
    refined.y = chosen.best_pose.y;
    refined.yaw = chosen.best_pose.yaw;
    applyLocalRelocSuccess_(refined, chosen_xy_range_m, chosen_yaw_range_rad,
                            manual_local_reloc_reseed_sigma_scale_,
                            manual_local_reloc_reseed_xy_min_m_,
                            manual_local_reloc_reseed_yaw_min_rad_,
                            "manual_local_reloc");
    if (odom_pose) {
      beginRelocRefine_(refined, *odom_pose, "manual_local_reloc", false);
    }
    ROS_INFO_STREAM("[AMCL] manual local_reloc success: x=" << refined.x
                    << " y=" << refined.y << " yaw="
                    << refined.yaw * 180.0 / kPi << "deg"
                    << " score=" << chosen.best_score
                    << " second=" << chosen.second_score
                    << " vf=" << chosen.valid_fraction
                    << " combined=" << FmtD(chosen_combined, 4)
                    << " stage="
                    << (used_final ? "final" : (used_fine ? "fine" : "coarse"))
                    << " coarse_win=(" << coarse_opt.xy_range_m << "m,"
                    << coarse_opt.yaw_range_rad * 180.0 / kPi << "deg)"
                    << " fine_win=(" << fine_opt.xy_range_m << "m,"
                    << fine_opt.yaw_range_rad * 180.0 / kPi << "deg)"
                    << " final_win=(" << final_opt.xy_range_m << "m,"
                    << final_opt.yaw_range_rad * 180.0 / kPi << "deg@"
                    << final_opt.yaw_step_rad * 180.0 / kPi << "deg)");
    return true;
  }

  bool shouldUpdateFilter_(const pf_vector_t &pose, pf_vector_t *delta) const {
    if (!delta) {
      return false;
    }
    delta->v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta->v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta->v[2] = nav2_amcl::angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]);
    return std::fabs(delta->v[0]) > update_min_d_ ||
           std::fabs(delta->v[1]) > update_min_d_ ||
           std::fabs(delta->v[2]) > update_min_a_ || force_update_ ||
           localRelocSettlingPending_();
  }

  bool updateFilter_(const sensor_msgs::LaserScanConstPtr &scan_msg,
                     const LaserMount &mount, const pf_vector_t &pose) {
    if (!scan_msg || !laser_model_) {
      return false;
    }
    if (scan_msg->range_max <= 0.0) {
      ROS_WARN_STREAM_THROTTLE(1.0,
                               "[AMCL] invalid laser range_max=" << scan_msg->range_max);
      return false;
    }

    nav2_amcl::LaserData ldata;
    ldata.laser = laser_model_.get();
    ldata.range_count = static_cast<int>(scan_msg->ranges.size());
    ldata.range_max =
        (laser_max_range_ > 0.0)
            ? std::min(static_cast<double>(scan_msg->range_max), laser_max_range_)
            : static_cast<double>(scan_msg->range_max);
    const double range_min =
        (laser_min_range_ > 0.0)
            ? std::max(static_cast<double>(scan_msg->range_min), laser_min_range_)
            : static_cast<double>(scan_msg->range_min);

    ldata.ranges = new double[static_cast<std::size_t>(ldata.range_count)][2];
    const double angle_min_base =
        NormalizeAngle(static_cast<double>(scan_msg->angle_min) + mount.yaw);
    const double angle_inc_base = static_cast<double>(scan_msg->angle_increment);

    for (int i = 0; i < ldata.range_count; ++i) {
      const double r = static_cast<double>(scan_msg->ranges[static_cast<std::size_t>(i)]);
      if (r <= range_min) {
        ldata.ranges[i][0] = ldata.range_max;
      } else {
        ldata.ranges[i][0] = r;
      }
      ldata.ranges[i][1] =
          NormalizeAngle(angle_min_base + angle_inc_base * static_cast<double>(i));
    }

    if (!laser_model_->sensorUpdate(pf_, &ldata)) {
      return false;
    }
    pf_odom_pose_ = pose;
    return true;
  }

  bool getBestHypothesis_(AmclHypothesis *hyp, pf_matrix_t *overall_cov) const {
    if (!pf_ || !hyp || !overall_cov) {
      return false;
    }

    const pf_sample_set_t *set = pf_->sets + pf_->current_set;
    *overall_cov = set->cov;
    if (set->sample_count <= 0) {
      return false;
    }

    if (set->cluster_count <= 0) {
      hyp->weight = 1.0;
      hyp->mean = set->mean;
      hyp->cov = set->cov;
      return true;
    }

    double max_weight = -1.0;
    int max_index = -1;
    for (int i = 0; i < set->cluster_count; ++i) {
      double weight = 0.0;
      pf_vector_t mean = pf_vector_zero();
      pf_matrix_t cov = pf_matrix_zero();
      if (!pf_get_cluster_stats(pf_, i, &weight, &mean, &cov)) {
        continue;
      }
      if (weight > max_weight) {
        max_weight = weight;
        max_index = i;
        hyp->weight = weight;
        hyp->mean = mean;
        hyp->cov = cov;
      }
    }

    if (max_index < 0) {
      hyp->weight = 1.0;
      hyp->mean = set->mean;
      hyp->cov = set->cov;
    }
    return true;
  }

  static void planarSigmaFromCov_(const pf_matrix_t &cov, double *major,
                                  double *minor, double *yaw_sigma) {
    if (major) {
      *major = std::numeric_limits<double>::quiet_NaN();
    }
    if (minor) {
      *minor = std::numeric_limits<double>::quiet_NaN();
    }
    if (yaw_sigma) {
      *yaw_sigma = std::numeric_limits<double>::quiet_NaN();
    }

    Eigen::Matrix2d xy_cov = Eigen::Matrix2d::Zero();
    xy_cov(0, 0) = cov.m[0][0];
    xy_cov(0, 1) = cov.m[0][1];
    xy_cov(1, 0) = cov.m[1][0];
    xy_cov(1, 1) = cov.m[1][1];
    xy_cov = 0.5 * (xy_cov + xy_cov.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(xy_cov);
    if (solver.info() == Eigen::Success) {
      const auto evals = solver.eigenvalues();
      if (minor) {
        *minor = SafeSqrt(evals[0]);
      }
      if (major) {
        *major = SafeSqrt(evals[1]);
      }
    }

    if (yaw_sigma) {
      *yaw_sigma = SafeSqrt(cov.m[2][2]);
    }
  }

  AmclGuardMetrics collectGuardMetrics_(int scan_points_count, bool odom_ok,
                                        bool sensor_update_ok) const {
    AmclGuardMetrics m;
    m.core_ready = (pf_ != nullptr) && (amcl_map_ != nullptr) &&
                   static_cast<bool>(laser_model_);
    m.initialized = initialized_ && initial_pose_is_known_;
    m.odom_ok = odom_ok;
    m.sensor_update_ok = sensor_update_ok;
    m.scan_points_count = std::max(0, scan_points_count);
    m.frame_valid = m.core_ready && m.initialized && m.odom_ok &&
                    m.sensor_update_ok && (m.scan_points_count > 0);
    return m;
  }

  AmclPfHealthMetrics collectPfHealthMetrics_() const {
    AmclPfHealthMetrics m;
    if (!pf_) {
      return m;
    }

    const pf_sample_set_t *set = pf_->sets + pf_->current_set;
    if (!set || set->sample_count <= 0) {
      return m;
    }

    m.valid = true;
    m.pf_converged = (pf_->converged != 0) || (set->converged != 0);
    m.sample_count = set->sample_count;

    pf_matrix_t best_cov = set->cov;
    int effective_cluster_count = set->cluster_count;
    if (effective_cluster_count <= 0) {
      effective_cluster_count = 1;
      m.best_cluster_weight = 1.0;
      m.second_cluster_weight = 0.0;
    } else {
      double best_weight = -1.0;
      double second_weight = 0.0;
      bool have_best_cov = false;

      for (int i = 0; i < set->cluster_count; ++i) {
        double weight = 0.0;
        pf_vector_t mean = pf_vector_zero();
        pf_matrix_t cov = pf_matrix_zero();
        if (!pf_get_cluster_stats(pf_, i, &weight, &mean, &cov)) {
          continue;
        }

        if (weight > best_weight) {
          second_weight = std::max(second_weight, best_weight);
          best_weight = weight;
          best_cov = cov;
          have_best_cov = true;
        } else if (weight > second_weight) {
          second_weight = weight;
        }
      }

      if (best_weight < 0.0) {
        m.valid = false;
        return m;
      }

      m.best_cluster_weight = best_weight;
      m.second_cluster_weight = std::max(0.0, second_weight);
      if (!have_best_cov) {
        best_cov = set->cov;
      }
    }

    m.cluster_count = effective_cluster_count;
    if (m.best_cluster_weight > 1e-9) {
      m.ambiguity_ratio = m.second_cluster_weight / m.best_cluster_weight;
    }

    planarSigmaFromCov_(best_cov, &m.best_sigma_xy_major, &m.best_sigma_xy_minor,
                        &m.best_sigma_yaw);

    double overall_sigma_xy_minor = std::numeric_limits<double>::quiet_NaN();
    planarSigmaFromCov_(set->cov, &m.overall_sigma_xy_major,
                        &overall_sigma_xy_minor, &m.overall_sigma_yaw);
    return m;
  }

  AmclOdomContinuityMetrics collectOdomContinuityMetrics_(
      const Pose2D &map_pose, const Pose2D *odom_pose) const {
    AmclOdomContinuityMetrics m;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (has_latest_odom_) {
        m.twist_valid = true;
        m.linear_speed = latest_odom_msg_.twist.twist.linear.x;
        // Temporary: use wheel-odom angular.z until IMU is integrated.
        m.angular_speed = latest_odom_msg_.twist.twist.angular.z;
      }
    }

    if (!odom_pose || !has_last_map_to_odom_) {
      return m;
    }

    m.predicted_pose = ComposePose(last_map_to_odom_, *odom_pose);
    const double dx = map_pose.x - m.predicted_pose.x;
    const double dy = map_pose.y - m.predicted_pose.y;
    m.jump_trans = std::hypot(dx, dy);
    m.jump_yaw = std::fabs(NormalizeAngle(map_pose.yaw - m.predicted_pose.yaw));
    m.valid = true;
    return m;
  }

  AmclQualitySnapshot collectQualitySnapshot_(const ros::Time &stamp,
                                              int scan_points_count,
                                              const Pose2D *odom_pose,
                                              bool sensor_update_ok) const {
    AmclQualitySnapshot snap;
    snap.stamp = stamp;
    snap.guard = collectGuardMetrics_(scan_points_count, odom_pose != nullptr,
                                      sensor_update_ok);
    snap.pf = collectPfHealthMetrics_();

    AmclHypothesis best_hyp;
    pf_matrix_t overall_cov = pf_matrix_zero();
    if (getBestHypothesis_(&best_hyp, &overall_cov)) {
      snap.odom = collectOdomContinuityMetrics_(pfVectorToPose(best_hyp.mean),
                                                odom_pose);
    }
    return snap;
  }

  void reportQualitySnapshot_(const ros::Time &stamp, int scan_points_count,
                              const Pose2D *odom_pose,
                              bool sensor_update_ok) {
    last_quality_snapshot_ =
        collectQualitySnapshot_(stamp, scan_points_count, odom_pose, sensor_update_ok);
    has_last_quality_snapshot_ = true;
    updateLocState_(last_quality_snapshot_);
    handleStableStateTransition_(stamp);

    const auto &snap = last_quality_snapshot_;
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "[AMCL][q] t=" << FmtD(snap.stamp.toSec(), 3)
        << " guard(valid=" << (snap.guard.frame_valid ? 1 : 0)
        << " core=" << (snap.guard.core_ready ? 1 : 0)
        << " init=" << (snap.guard.initialized ? 1 : 0)
        << " odom=" << (snap.guard.odom_ok ? 1 : 0)
        << " scan_n=" << snap.guard.scan_points_count
        << " upd=" << (snap.guard.sensor_update_ok ? 1 : 0) << ")";

    if (snap.pf.valid) {
      oss << " pf(conv=" << (snap.pf.pf_converged ? 1 : 0)
          << " n=" << snap.pf.sample_count
          << " c=" << snap.pf.cluster_count
          << " best=" << FmtD(snap.pf.best_cluster_weight, 3)
          << " second=" << FmtD(snap.pf.second_cluster_weight, 3)
          << " amb=" << FmtD(snap.pf.ambiguity_ratio, 3)
          << " sig_xy=" << FmtD(snap.pf.best_sigma_xy_major, 3)
          << "/" << FmtD(snap.pf.best_sigma_xy_minor, 3)
          << " sig_yaw=" << FmtD(snap.pf.best_sigma_yaw * 180.0 / kPi, 2)
          << "deg"
          << " ov_xy=" << FmtD(snap.pf.overall_sigma_xy_major, 3)
          << " ov_yaw=" << FmtD(snap.pf.overall_sigma_yaw * 180.0 / kPi, 2)
          << "deg)";
    } else {
      oss << " pf(valid=0)";
    }

    oss << " state(raw=" << AmclLocStateStr(raw_loc_state_)
        << " stable=" << AmclLocStateStr(loc_state_)
        << " streak=" << raw_loc_state_streak_
        << " init=" << (init_validation_pending_ ? 1 : 0) << ")";

    if (snap.odom.twist_valid) {
      oss << " twist(v=" << FmtD(snap.odom.linear_speed, 3)
          << "m/s w=" << FmtD(snap.odom.angular_speed, 4)
          << "rad/s w_src=odom_tmp)";
    } else {
      oss << " twist(valid=0)";
    }

    if (snap.odom.valid) {
      oss << " jump(dT=" << FmtD(snap.odom.jump_trans, 3)
          << " dYaw=" << FmtD(snap.odom.jump_yaw * 180.0 / kPi, 2) << "deg)";
    } else {
      oss << " jump(valid=0)";
    }

    if (snap.guard.frame_valid && snap.pf.valid) {
      ROS_INFO_STREAM_THROTTLE(1.0, oss.str());
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0, oss.str());
    }
  }

  void fillPoseCovariance_(const pf_matrix_t &cov,
                           boost::array<double, 36> *out) const {
    if (!out) {
      return;
    }
    out->assign(0.0);
    (*out)[0] = cov.m[0][0];
    (*out)[1] = cov.m[0][1];
    (*out)[5] = cov.m[0][2];
    (*out)[6] = cov.m[1][0];
    (*out)[7] = cov.m[1][1];
    (*out)[11] = cov.m[1][2];
    (*out)[30] = cov.m[2][0];
    (*out)[31] = cov.m[2][1];
    (*out)[35] = cov.m[2][2];
    (*out)[14] = 0.25;
    (*out)[21] = 0.25;
    (*out)[28] = 0.25;
  }

  void publishParticleCloud_(const ros::Time &stamp) {
    if (!publish_particlecloud_ || !particlecloud_pub_ || !pf_) {
      return;
    }

    const pf_sample_set_t *set = pf_->sets + pf_->current_set;
    geometry_msgs::PoseArray msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_id_;
    const int out_count = std::min(set->sample_count, particlecloud_max_samples_);
    msg.poses.reserve(static_cast<std::size_t>(out_count));
    const int step = std::max(1, set->sample_count / std::max(1, out_count));
    for (int i = 0; i < set->sample_count; i += step) {
      geometry_msgs::Pose pose;
      pose.position.x = set->samples[i].pose.v[0];
      pose.position.y = set->samples[i].pose.v[1];
      pose.position.z = 0.0;
      pose.orientation = YawToQuaternion(set->samples[i].pose.v[2]);
      msg.poses.push_back(pose);
      if (static_cast<int>(msg.poses.size()) >= out_count) {
        break;
      }
    }
    particlecloud_pub_.publish(msg);
  }

  bool canUseMapToOdomTf_(const ros::Time &stamp) const {
    if (!publish_map_to_odom_tf_) {
      return false;
    }
    try {
      return tf_buffer_.canTransform(odom_frame_id_, base_frame_id_,
                                     stamp.isZero() ? ros::Time(0) : stamp,
                                     ros::Duration(0.0));
    } catch (...) {
      return false;
    }
  }

  void publishPoseAndTf_(const ros::Time &stamp, const Pose2D &map_pose,
                         const pf_matrix_t &cov, const Pose2D *odom_pose,
                         bool refresh_map_to_odom,
                         bool publish_particlecloud_now) {
    if (odom_pose && (refresh_map_to_odom || !has_last_map_to_odom_)) {
      last_map_to_odom_ = ComposePose(map_pose, InversePose(*odom_pose));
      has_last_map_to_odom_ = true;
    }

    const bool use_map_to_odom_tf = odom_pose && canUseMapToOdomTf_(stamp);
    if (publish_map_to_odom_tf_ && odom_pose && !use_map_to_odom_tf) {
      ROS_WARN_STREAM_THROTTLE(
          2.0,
          "[AMCL] odom->base TF unavailable, fallback to map->base publishing");
    }

    Pose2D pose_to_publish = map_pose;
    if (odom_pose && has_last_map_to_odom_) {
      pose_to_publish = ComposePose(last_map_to_odom_, *odom_pose);
    }

    geometry_msgs::PoseWithCovarianceStamped amcl_pose;
    amcl_pose.header.stamp = stamp;
    amcl_pose.header.frame_id = map_frame_id_;
    amcl_pose.pose.pose.position.x = pose_to_publish.x;
    amcl_pose.pose.pose.position.y = pose_to_publish.y;
    amcl_pose.pose.pose.position.z = 0.0;
    amcl_pose.pose.pose.orientation = YawToQuaternion(pose_to_publish.yaw);
    fillPoseCovariance_(cov, &amcl_pose.pose.covariance);
    amcl_pose_pub_.publish(amcl_pose);

    nav_msgs::Odometry odom;
    odom.header = amcl_pose.header;
    odom.child_frame_id = base_frame_id_;
    odom.pose = amcl_pose.pose;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (has_latest_odom_) {
        odom.twist.twist = latest_odom_msg_.twist.twist;
      }
    }
    odom_pub_.publish(odom);

    if (publish_particlecloud_now) {
      publishParticleCloud_(stamp);
    }

    if (!publish_tf_) {
      first_pose_sent_ = true;
      return;
    }

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    if (use_map_to_odom_tf) {
      tf_msg.header.frame_id = map_frame_id_;
      tf_msg.child_frame_id = odom_frame_id_;
      tf_msg.transform.translation.x = last_map_to_odom_.x;
      tf_msg.transform.translation.y = last_map_to_odom_.y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = YawToQuaternion(last_map_to_odom_.yaw);
    } else {
      tf_msg.header.frame_id = map_frame_id_;
      tf_msg.child_frame_id = base_frame_id_;
      tf_msg.transform.translation.x = pose_to_publish.x;
      tf_msg.transform.translation.y = pose_to_publish.y;
      tf_msg.transform.translation.z = 0.0;
      tf_msg.transform.rotation = YawToQuaternion(pose_to_publish.yaw);
    }
    tf_broadcaster_.sendTransform(tf_msg);
    first_pose_sent_ = true;
  }

  void publishCurrentEstimate_(const ros::Time &stamp, const Pose2D *odom_pose) {
    AmclHypothesis hyp;
    pf_matrix_t cov = pf_matrix_zero();
    if (!getBestHypothesis_(&hyp, &cov)) {
      return;
    }

    last_estimate_pose_ = pfVectorToPose(hyp.mean);
    last_estimate_cov_ = cov;
    has_last_estimate_ = true;
    publishPoseAndTf_(stamp, last_estimate_pose_, last_estimate_cov_, odom_pose,
                      true, true);
  }

  void publishStoredEstimate_(const ros::Time &stamp, const Pose2D *odom_pose) {
    if (!has_last_estimate_) {
      return;
    }
    publishPoseAndTf_(stamp, last_estimate_pose_, last_estimate_cov_, odom_pose,
                      false, false);
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg) {
    if (!scan_msg) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      pending_scan_ = scan_msg;
    }
    work_cv_.notify_one();
  }

  map_t * convertMap_(const nav_msgs::OccupancyGrid &map_msg) {
    map_t *map = map_alloc();
    if (!map) {
      return nullptr;
    }

    map->size_x = static_cast<int>(map_msg.info.width);
    map->size_y = static_cast<int>(map_msg.info.height);
    map->scale = map_msg.info.resolution;
    map->origin_x =
        map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y =
        map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

    map->cells = reinterpret_cast<map_cell_t *>(
        malloc(sizeof(map_cell_t) * map->size_x * map->size_y));
    if (!map->cells) {
      map_free(map);
      return nullptr;
    }

    for (int i = 0; i < map->size_x * map->size_y; ++i) {
      if (map_msg.data[static_cast<std::size_t>(i)] == 0) {
        map->cells[i].occ_state = -1;
      } else if (map_msg.data[static_cast<std::size_t>(i)] == 100) {
        map->cells[i].occ_state = +1;
      } else {
        map->cells[i].occ_state = 0;
      }
      map->cells[i].occ_dist = 0.0f;
    }

    return map;
  }

  void createFreeSpaceVector_() {
    free_space_indices_.clear();
    if (!amcl_map_) {
      return;
    }

    free_space_indices_.reserve(
        static_cast<std::size_t>(amcl_map_->size_x * amcl_map_->size_y) / 2);
    for (int i = 0; i < amcl_map_->size_x; ++i) {
      for (int j = 0; j < amcl_map_->size_y; ++j) {
        if (amcl_map_->cells[MAP_INDEX(amcl_map_, i, j)].occ_state == -1) {
          FreeCellIndex point;
          point.x = i;
          point.y = j;
          free_space_indices_.push_back(point);
        }
      }
    }
  }

  pf_vector_t uniformPoseGenerator_() {
    pf_vector_t p = pf_vector_zero();
    if (!amcl_map_ || free_space_indices_.empty()) {
      return p;
    }

    const std::size_t idx =
        static_cast<std::size_t>(drand48() * free_space_indices_.size()) %
        free_space_indices_.size();
    const FreeCellIndex &cell = free_space_indices_[idx];
    p.v[0] = MAP_WXGX(amcl_map_, cell.x);
    p.v[1] = MAP_WYGY(amcl_map_, cell.y);
    p.v[2] = drand48() * 2.0 * kPi - kPi;
    return p;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber odom_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber initialpose_sub_;

  ros::Publisher odom_pub_;
  ros::Publisher amcl_pose_pub_;
  ros::Publisher particlecloud_pub_;
  ros::Publisher map_pub_;
  ros::Publisher map_metadata_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  mutable std::mutex state_mutex_;
  std::condition_variable work_cv_;
  std::thread worker_thread_;
  bool worker_stop_ = false;
  sensor_msgs::LaserScanConstPtr pending_scan_;
  PendingInitialPose pending_initial_pose_;

  OccupancyMap map_;
  localization_ndt::InitialPoseManager initial_pose_mgr_;
  map_t *amcl_map_ = nullptr;
  pf_t *pf_ = nullptr;
  nav2_amcl::DifferentialMotionModel motion_model_;
  std::unique_ptr<nav2_amcl::Laser> laser_model_;
  std::vector<FreeCellIndex> free_space_indices_;

  bool initialized_ = false;
  bool initial_pose_is_known_ = false;
  bool first_pose_sent_ = false;
  bool force_update_ = true;
  bool has_last_estimate_ = false;
  bool has_last_map_to_odom_ = false;

  bool has_latest_odom_ = false;
  bool pf_odom_initialized_ = false;
  nav_msgs::Odometry latest_odom_msg_;
  Pose2D latest_odom_pose_;
  std::deque<nav_msgs::Odometry> odom_history_;
  pf_vector_t pf_odom_pose_ = pf_vector_zero();

  std::string laser_model_type_ = "likelihood_field_prob";
  std::string laser_frame_id_;
  bool laser_pose_ready_ = false;
  double laser_mount_yaw_ = 0.0;

  Pose2D last_estimate_pose_;
  Pose2D last_map_to_odom_;
  pf_matrix_t last_estimate_cov_ = pf_matrix_zero();

  std::string pkg_path_;
  std::string run_config_dir_;
  std::string interface_config_dir_;
  std::string interface_yaml_path_;
  std::string params_dir_;
  std::string params_yaml_path_;
  std::string pose_base_dir_;
  std::string pose_fixed_dir_;
  std::string pose_last_dir_;
  std::string map_name_;
  std::string map_yaml_path_;
  std::string scan_topic_ = "/scan";
  std::string odom_topic_ = "/odom/wheel";
  std::string imu_topic_ = "/imu";
  std::string initialpose_topic_ = "/initialpose";
  std::string map_frame_id_ = "map";
  std::string base_frame_id_ = "base_footprint";
  std::string odom_frame_id_ = "odom";
  std::string odom_source_mode_ = "auto";
  std::string odom_pub_topic_ = "amcl_odom";
  std::string amcl_pose_topic_ = "amcl_pose";
  std::string particlecloud_topic_ = "particlecloud";
  std::string map_topic_ = "map";
  std::string map_metadata_topic_ = "map_metadata";

  bool publish_tf_ = true;
  bool publish_map_ = true;
  bool publish_particlecloud_ = true;
  bool publish_map_to_odom_tf_ = true;
  int particlecloud_max_samples_ = 400;
  bool use_yaml_frame_id_ = false;
  bool global_init_on_startup_ = true;
  bool use_initial_pose_ = false;

  double scan_min_range_ = 0.0;
  double scan_max_range_ = 0.0;
  double scan_voxel_m_ = 0.03;
  int max_scan_points_ = 1500;

  double laser_min_range_ = -1.0;
  double laser_max_range_ = 100.0;
  int max_beams_ = 60;
  double sigma_hit_ = 0.2;
  double laser_likelihood_max_dist_ = 2.0;
  double z_hit_ = 0.5;
  double z_rand_ = 0.5;
  bool do_beamskip_ = true;
  double beam_skip_distance_ = 0.5;
  double beam_skip_threshold_ = 0.3;
  double beam_skip_error_threshold_ = 0.9;

  double alpha1_ = 0.2;
  double alpha2_ = 0.2;
  double alpha3_ = 0.2;
  double alpha4_ = 0.2;
  double alpha5_ = 0.2;
  int min_particles_ = 500;
  int max_particles_ = 2000;
  double pf_err_ = 0.05;
  double pf_z_ = 0.99;
  double recovery_alpha_fast_ = 0.0;
  double recovery_alpha_slow_ = 0.0;
  int resample_interval_ = 1;
  double update_min_d_ = 0.25;
  double update_min_a_ = Deg2Rad(11.4591559);

  double init_cov_xx_ = 0.25;
  double init_cov_yy_ = 0.25;
  double init_cov_aa_ = Deg2Rad(15.0) * Deg2Rad(15.0);
  Pose2D startup_initial_pose_;

  bool manual_local_reloc_enable_ = true;
  bool manual_local_reloc_allow_external_map_paths_ = true;
  std::unique_ptr<LocalRelocalizer2D> manual_local_reloc_;
  LocalRelocOptions local_reloc_common_opt_;
  double local_reloc_xy_range_m_ = 5.0;
  double local_reloc_yaw_search_deg_ = 180.0;
  int local_reloc_settle_frames_ = 3;
  double manual_local_reloc_coarse_xy_range_m_ = 5.0;
  double manual_local_reloc_coarse_yaw_search_deg_ = 180.0;
  double manual_local_reloc_xy_min_m_ = 0.30;
  double manual_local_reloc_xy_sigma_k_ = 3.0;
  double manual_local_reloc_xy_max_m_ = 5.0;
  double manual_local_reloc_yaw_min_rad_ = Deg2Rad(6.0);
  double manual_local_reloc_yaw_sigma_k_ = 3.0;
  double manual_local_reloc_final_xy_range_m_ = 0.50;
  double manual_local_reloc_final_yaw_search_deg_ = 10.0;
  double manual_local_reloc_final_yaw_step_rad_ = Deg2Rad(0.25);
  double manual_local_reloc_xy_prior_ref_m_ = 0.30;
  double manual_local_reloc_yaw_prior_ref_rad_ = Deg2Rad(10.0);
  double manual_local_reloc_reseed_sigma_scale_ = 0.05;
  double manual_local_reloc_reseed_xy_min_m_ = 0.05;
  double manual_local_reloc_reseed_yaw_min_rad_ = Deg2Rad(2.0);
  bool manual_local_reloc_pending_ = false;
  RelocPose2D manual_local_reloc_center_;
  double manual_local_reloc_xy_range_m_ = 0.0;
  double manual_local_reloc_yaw_range_rad_ = 0.0;

  bool startup_local_reloc_enable_ = true;
  std::unique_ptr<LocalRelocalizer2D> auto_local_reloc_;
  int startup_max_candidates_per_scan_ = 3;
  bool startup_local_reloc_pending_ = false;
  RelocPose2D startup_local_reloc_center_;
  std::string startup_local_reloc_reason_;

  bool lost_local_reloc_enable_ = true;
  double lost_local_reloc_min_interval_sec_ = 2.0;
  double lost_local_reloc_expand_factor_ = 1.5;
  int lost_local_reloc_max_expansions_ = 1;
  ros::Time last_lost_local_reloc_attempt_;
  int lost_local_reloc_fail_count_ = 0;
  bool lost_fixed_reloc_enable_ = true;
  double lost_fixed_min_interval_sec_ = 2.0;
  int lost_fixed_max_candidates_per_scan_ = 3;
  double lost_fixed_stationary_min_duration_ = 2.0;
  ros::Time last_lost_fixed_cycle_start_;
  ros::Time lost_fixed_stationary_start_;
  bool lost_fixed_cycle_active_ = false;
  std::vector<std::size_t> lost_fixed_candidates_;
  std::size_t lost_fixed_cursor_ = 0;

  double auto_local_reloc_reseed_sigma_scale_ = 0.25;
  double auto_local_reloc_reseed_xy_min_m_ = 0.05;
  double auto_local_reloc_reseed_yaw_min_rad_ = Deg2Rad(2.0);

  long random_seed_ = -1;
  double odom_history_max_age_sec_ = 2.0;
  int odom_history_max_size_ = 400;
  double tf_lookup_timeout_sec_ = 0.02;
  int resample_count_ = 0;
  AmclStateConfig state_cfg_;
  AmclLocState loc_state_ = AmclLocState::UNINITIALIZED;
  AmclLocState raw_loc_state_ = AmclLocState::UNINITIALIZED;
  AmclLocState prev_raw_loc_state_ = AmclLocState::UNINITIALIZED;
  int raw_loc_state_streak_ = 0;
  std::vector<InitSeedCandidate> init_seed_candidates_;
  std::size_t init_seed_cursor_ = 0;
  InitSeedCandidate active_init_seed_;
  bool has_active_init_seed_ = false;
  bool init_validation_pending_ = false;
  int init_validate_good_streak_ = 0;
  int init_validate_bad_streak_ = 0;
  std::string init_validation_reason_;
  FrontendMode frontend_mode_ = FrontendMode::NORMAL;
  RelocRefineState reloc_refine_;
  int forced_sensor_updates_remaining_ = 0;
  std::string local_reloc_settling_reason_;
  double startup_fixed_retry_interval_sec_ = 2.0;
  double startup_fixed_retry_stationary_min_duration_ = 2.0;
  bool startup_fixed_retry_waiting_ = false;
  ros::Time startup_fixed_retry_stationary_start_;
  ros::Time startup_last_fixed_retry_cycle_start_;
  bool stationary_phase_ = false;
  bool stationary_pose_written_ = false;
  ros::Time stationary_start_time_;
  Pose2D last_good_pose_;
  Pose2D last_good_map_to_odom_;
  bool has_last_good_pose_ = false;
  bool has_last_good_map_to_odom_ = false;
  AmclLocState prev_loc_state_ = AmclLocState::UNINITIALIZED;
  AmclQualitySnapshot last_quality_snapshot_;
  bool has_last_quality_snapshot_ = false;
};

}  // namespace
}  // namespace localization_ndt

int main(int argc, char **argv) {
  ros::init(argc, argv, "amcl_localizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    localization_ndt::AmclLocalizerNode node(nh, pnh);
    node.spin();
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("[AMCL] fatal: " << e.what());
    return 1;
  }

  return 0;
}

