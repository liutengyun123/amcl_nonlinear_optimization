#include "localization_ndt/initial_pose_manager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <ros/ros.h>
#include <cstdio>

namespace localization_ndt {

static bool FileExists(const std::string &path) {
  std::ifstream f(path.c_str());
  return f.good();
}

std::string
InitialPoseManager::extractStem(const std::string &path_or_name) const {
  std::size_t slash_pos = path_or_name.find_last_of("/\\");
  std::size_t start = (slash_pos == std::string::npos) ? 0 : slash_pos + 1;

  std::size_t dot_pos = path_or_name.find_last_of('.');
  if (dot_pos == std::string::npos || dot_pos <= start) {
    return path_or_name.substr(start);
  }
  return path_or_name.substr(start, dot_pos - start);
}

static std::string TrimCopy(const std::string &s) {
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  std::size_t b = 0, e = s.size();
  while (b < e && is_space((unsigned char)s[b]))
    ++b;
  while (e > b && is_space((unsigned char)s[e - 1]))
    --e;
  return s.substr(b, e - b);
}

static int FindSamePoseIndex(
    const std::vector<localization_ndt::InitialPoseManager::FixedPoseEntry> &v,
    const localization_ndt::InitialPoseManager::Pose2D &pose, double eps_xy,
    double eps_yaw, int skip_index = -1) {
  for (int i = 0; i < (int)v.size(); ++i) {
    if (i == skip_index)
      continue;
    if (localization_ndt::InitialPoseManager::isSamePose(v[i].pose, pose,
                                                         eps_xy, eps_yaw)) {
      return i;
    }
  }
  return -1;
}

static void NormalizeFixedPoseYamlStyle(YAML::Node *root) {
  if (!root)
    return;

  // pending 单行
  YAML::Node pending = (*root)["pending"];
  if (pending && pending.IsMap())
    pending.SetStyle(YAML::EmitterStyle::Flow);

  // poses block + 每条 flow
  YAML::Node poses = (*root)["poses"];
  if (!poses || !poses.IsSequence())
    return;
  poses.SetStyle(YAML::EmitterStyle::Block);
  for (std::size_t i = 0; i < poses.size(); ++i) {
    YAML::Node it = poses[i];
    if (it && it.IsMap())
      it.SetStyle(YAML::EmitterStyle::Flow);
  }
}

bool InitialPoseManager::isSamePose(const Pose2D &a, const Pose2D &b,
                                    double eps_xy, double eps_yaw) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dxy2 = dx * dx + dy * dy;

  auto normAngle = [](double ang) {
    while (ang > M_PI)
      ang -= 2.0 * M_PI;
    while (ang < -M_PI)
      ang += 2.0 * M_PI;
    return ang;
  };
  const double dyaw = std::fabs(normAngle(a.yaw - b.yaw));

  return (dxy2 <= eps_xy * eps_xy) && (dyaw <= eps_yaw);
}

bool InitialPoseManager::initialize(const std::string &map_path,
                                    const std::string &agv_name) {
  return initializeWithMapId(map_path, std::string(), agv_name);
}

bool InitialPoseManager::initializeWithMapId(const std::string &map_path,
                                             const std::string &map_id,
                                             const std::string &agv_name) {
  if (map_path.empty()) {
    ROS_ERROR("[LOC] InitialPoseManager: map_path is empty.");
    return false;
  }
  if (fixed_dir_.empty() || last_dir_.empty()) {
    ROS_ERROR("[LOC] InitialPoseManager: fixed_dir or last_dir empty. Call "
              "setFixedDir/setLastDir.");
    return false;
  }

  // map 必须存在
  {
    std::ifstream f(map_path.c_str());
    if (!f.good()) {
      ROS_ERROR_STREAM(
          "[LOC] InitialPoseManager: map file not exist: " << map_path);
      return false;
    }
  }

  map_stem_ = map_id.empty() ? extractStem(map_path) : extractStem(map_id);
  agv_stem_ = extractStem(agv_name);

  if (map_stem_.empty()) {
    ROS_ERROR_STREAM("[LOC] InitialPoseManager: extract map stem failed. map="
                     << map_path);
    return false;
  }

  if (agv_stem_.empty()) {
    fixed_yaml_path_ = fixed_dir_ + "/" + map_stem_ + ".yaml";
    last_yaml_path_ = last_dir_ + "/" + map_stem_ + ".yaml";
  } else {
    fixed_yaml_path_ = fixed_dir_ + "/" + map_stem_ + "_" + agv_stem_ + ".yaml";
    last_yaml_path_ = last_dir_ + "/" + map_stem_ + "_" + agv_stem_ + ".yaml";
  }

  // fixed：不存在则创建默认模板
  if (!writeFixedFileDefaultIfMissing()) {
    return false;
  }
  if (!loadFixedFile()) {
    // fixed 文件坏了也不要直接炸：用一个 origin fallback
    ROS_WARN_STREAM("[LOC] InitialPoseManager: load fixed failed, fallback to "
                    "one origin pose. file="
                    << fixed_yaml_path_);
    fixed_poses_.clear();
    FixedPoseEntry e;
    e.pose = Pose2D();
    e.priority = 0;
    e.name = "origin";
    fixed_poses_.push_back(e);
  }

  // last：仅在文件存在时加载；不存在不创建
  (void)loadLastFileIfExists();

 ROS_INFO_STREAM("[LOC] InitialPoseManager: fixed="
                << fixed_yaml_path_ << " (#poses=" << fixed_poses_.size()
                << "), last(stable)=" << (has_last_pose_ ? last_yaml_path_ : "(none)")
                << ", last(shutdown)=" << (has_shutdown_pose_ ? last_yaml_path_ : "(none)"));

  return true;
}

bool InitialPoseManager::writeFixedFileDefaultIfMissing() {
  if (FileExists(fixed_yaml_path_))
    return true;

  YAML::Node root;

  // 第一行：pending 输入区（单行）
  YAML::Node pending(YAML::NodeType::Map);
  pending.SetStyle(YAML::EmitterStyle::Flow);
  pending["name"] = "";
  pending["x"] = 0.0;
  pending["y"] = 0.0;
  pending["yaw"] = 0.0; // rad
  pending["priority"] = 0;
  root["pending"] = pending;

  // poses 列表
  YAML::Node poses(YAML::NodeType::Sequence);
  poses.SetStyle(YAML::EmitterStyle::Block);

  YAML::Node p(YAML::NodeType::Map);
  p.SetStyle(YAML::EmitterStyle::Flow);
  p["name"] = "origin";
  p["x"] = 0.0;
  p["y"] = 0.0;
  p["yaw"] = 0.0;
  p["priority"] = 0;
  poses.push_back(p);

  root["poses"] = poses;
  NormalizeFixedPoseYamlStyle(&root);

  try {
    YAML::Emitter out;
    out << root;
    std::ofstream fout(fixed_yaml_path_.c_str());
    if (!fout.is_open()) {
      ROS_ERROR_STREAM(
          "[LOC] InitialPoseManager: cannot write default fixed file: "
          << fixed_yaml_path_);
      return false;
    }
    fout << out.c_str();
    fout.close();
    ROS_WARN_STREAM("[LOC] InitialPoseManager: created default fixed file: "
                    << fixed_yaml_path_);
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM(
        "[LOC] InitialPoseManager: write default fixed failed: " << e.what());
    return false;
  }
}

bool InitialPoseManager::loadFixedFile() {
  fixed_poses_.clear();

  try {
    YAML::Node root = YAML::LoadFile(fixed_yaml_path_);

    // 1) 先读 poses（允许 poses 缺失：先不直接 return false，给 pending 兜底）
    YAML::Node poses = root["poses"];
    if (poses && poses.IsSequence()) {
      for (size_t i = 0; i < poses.size(); ++i) {
        const YAML::Node &n = poses[i];
        FixedPoseEntry e;

        if (n["x"])
          e.pose.x = n["x"].as<double>();
        if (n["y"])
          e.pose.y = n["y"].as<double>();
        if (n["yaw"])
          e.pose.yaw = n["yaw"].as<double>();
        if (n["priority"])
          e.priority = n["priority"].as<int>();
        if (n["name"])
          e.name = n["name"].as<std::string>();

        fixed_poses_.push_back(e);
      }
    }

    // 2) 消费 pending: {name, x, y, yaw, priority}
    bool consumed = false;
    YAML::Node pending = root["pending"];
    if (pending && pending.IsMap()) {
      std::string name;
      if (pending["name"])
        name = TrimCopy(pending["name"].as<std::string>());

      if (!name.empty()) {
        Pose2D pose;
        pose.x = pending["x"] ? pending["x"].as<double>() : 0.0;
        pose.y = pending["y"] ? pending["y"].as<double>() : 0.0;
        pose.yaw = pending["yaw"] ? pending["yaw"].as<double>() : 0.0;
        int pri = pending["priority"] ? pending["priority"].as<int>() : 0;

        const bool ok = std::isfinite(pose.x) && std::isfinite(pose.y) &&
                        std::isfinite(pose.yaw);
        if (!ok) {
          ROS_WARN_STREAM("[LOC] InitialPoseManager: pending has non-finite "
                          "value, ignored. file="
                          << fixed_yaml_path_);
        } else {
          // 先找同名项（同名就是“更新”语义）
          int same_name_idx = -1;
          for (int i = 0; i < (int)fixed_poses_.size(); ++i) {
            if (fixed_poses_[i].name == name) {
              same_name_idx = i;
              break;
            }
          }

          // 查“同位姿”项：如果名字不同但位姿相同，要提示并拒绝写入
          const double eps_xy = 1e-4;  // 可改大点，例如 1e-3
          const double eps_yaw = 1e-4; // 可改大点，例如 1e-3
          int dup_pose_idx = FindSamePoseIndex(fixed_poses_, pose, eps_xy,
                                               eps_yaw, same_name_idx);

          if (dup_pose_idx >= 0) {
            // 位姿重复，但 name 不同（或 same_name_idx==-1），提示重复对象
            ROS_WARN_STREAM("[LOC] InitialPoseManager: pending pose duplicates "
                            "existing pose, "
                            "REJECTED. pending_name='"
                            << name << "' duplicates existing(index="
                            << dup_pose_idx << ", name='"
                            << fixed_poses_[dup_pose_idx].name << "') "
                            << " pose=(" << pose.x << "," << pose.y << ","
                            << pose.yaw << ") "
                            << " file=" << fixed_yaml_path_);
            // 关键：不写入、不清空 pending（能改完再保存）
          } else {
            // 没有位姿重复：允许写入
            if (same_name_idx >= 0) {
              // 更新同名
              fixed_poses_[same_name_idx].pose = pose;
              fixed_poses_[same_name_idx].priority = pri;
            } else {
              // 新增
              FixedPoseEntry e;
              e.name = name;
              e.pose = pose;
              e.priority = pri;
              fixed_poses_.push_back(e);
            }
            consumed = true;
          }
        }
      }
    }

    // 3) 如果消费了 pending，就写回（writeFixedFile 会把 pending 自动写空）
    if (consumed) {
      if (!writeFixedFile()) {
        ROS_WARN_STREAM("[LOC] InitialPoseManager: consumed pending but "
                        "writeback failed. file="
                        << fixed_yaml_path_);
      } else {
        ROS_INFO_STREAM(
            "[LOC] InitialPoseManager: consumed pending and cleared it. file="
            << fixed_yaml_path_);
      }
    }

    // 4) 最终检查：fixed_poses_ 不能为空
    if (fixed_poses_.empty()) {
      ROS_WARN_STREAM("[LOC] InitialPoseManager: fixed poses empty after load: "
                      << fixed_yaml_path_);
      return false;
    }
    return true;

  } catch (const std::exception &e) {
    ROS_WARN_STREAM("[LOC] InitialPoseManager: load fixed yaml failed: "
                    << fixed_yaml_path_ << " err=" << e.what());
    return false;
  }
}

bool InitialPoseManager::loadLastFileIfExists() {
  std::lock_guard<std::mutex> lk(last_mu_);

  has_last_pose_ = false;
  has_shutdown_pose_ = false;
  last_pose_ = Pose2D();
  shutdown_pose_ = Pose2D();

  if (!FileExists(last_yaml_path_)) return false;

  auto parse_pose = [](const YAML::Node& n, Pose2D* out) -> bool {
    if (!out || !n || !n.IsMap()) return false;
    if (!n["x"] || !n["y"] || !n["yaw"]) return false;
    out->x = n["x"].as<double>();
    out->y = n["y"].as<double>();
    out->yaw = n["yaw"].as<double>();
    return std::isfinite(out->x) && std::isfinite(out->y) && std::isfinite(out->yaw);
  };

  try {
    YAML::Node root = YAML::LoadFile(last_yaml_path_);

    // stable: 顶层 x/y/yaw（兼容旧文件）
    Pose2D stable;
    if (parse_pose(root, &stable)) {
      last_pose_ = stable;
      has_last_pose_ = true;
    }

    // shutdown: root["shutdown"]
    Pose2D sh;
    if (parse_pose(root["shutdown"], &sh)) {
      shutdown_pose_ = sh;
      has_shutdown_pose_ = true;
    }

    return has_last_pose_ || has_shutdown_pose_;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("[LOC] InitialPoseManager: load last yaml failed: "
                    << last_yaml_path_ << " err=" << e.what());
    has_last_pose_ = false;
    has_shutdown_pose_ = false;
    last_pose_ = Pose2D();
    shutdown_pose_ = Pose2D();
    return false;
  }
}


bool InitialPoseManager::writeFixedFile() {
  try {
    YAML::Node root;

    // 永远写一个空 pending（这样消费后会自动清空）
    YAML::Node pending(YAML::NodeType::Map);
    pending.SetStyle(YAML::EmitterStyle::Flow);
    pending["name"] = "";
    pending["x"] = 0.0;
    pending["y"] = 0.0;
    pending["yaw"] = 0.0;
    pending["priority"] = 0;
    root["pending"] = pending;

    // poses
    YAML::Node poses(YAML::NodeType::Sequence);
    poses.SetStyle(YAML::EmitterStyle::Block);

    for (const auto &e : fixed_poses_) {
      YAML::Node p(YAML::NodeType::Map);
      p.SetStyle(YAML::EmitterStyle::Flow);

      p["name"] = e.name;
      p["x"] = e.pose.x;
      p["y"] = e.pose.y;
      p["yaw"] = e.pose.yaw;
      p["priority"] = e.priority;

      poses.push_back(p);
    }
    root["poses"] = poses;

    NormalizeFixedPoseYamlStyle(&root);

    YAML::Emitter out;
    out << root;

    std::ofstream fout(fixed_yaml_path_.c_str());
    if (!fout.is_open()) {
      ROS_WARN_STREAM("[LOC] InitialPoseManager: failed to write fixed yaml: "
                      << fixed_yaml_path_);
      return false;
    }
    fout << out.c_str();
    fout.close();
    return true;
  } catch (const std::exception &e) {
    ROS_WARN_STREAM(
        "[LOC] InitialPoseManager: write fixed yaml failed: " << e.what());
    return false;
  }
}


bool InitialPoseManager::writeLastFile() {
  // NOTE: caller must hold last_mu_.
  if (!has_last_pose_ && !has_shutdown_pose_) {
    ROS_WARN_STREAM("[LOC] InitialPoseManager: writeLastFile called but no pose.");
    return false;
  }

  try {
    YAML::Node root;

    // stable：只在 has_last_pose_ 时写顶层（语义：顶层=稳定）
    if (has_last_pose_) {
      root["x"] = last_pose_.x;
      root["y"] = last_pose_.y;
      root["yaw"] = last_pose_.yaw;
    }

    // shutdown：写入子节点
    if (has_shutdown_pose_) {
      YAML::Node sh(YAML::NodeType::Map);
      sh["x"] = shutdown_pose_.x;
      sh["y"] = shutdown_pose_.y;
      sh["yaw"] = shutdown_pose_.yaw;
      root["shutdown"] = sh;
    }

    YAML::Emitter out;
    out << root;

    const std::string tmp = last_yaml_path_ + ".tmp";
    {
      std::ofstream fout(tmp.c_str(), std::ios::out | std::ios::trunc);
      if (!fout.is_open()) {
        ROS_WARN_STREAM("[LOC] InitialPoseManager: failed to write last tmp: " << tmp);
        return false;
      }
      fout << out.c_str();
      fout.close();
    }

    // Linux/POSIX: rename 会原子覆盖目标（同一文件系统内）
    if (std::rename(tmp.c_str(), last_yaml_path_.c_str()) != 0) {
      ROS_WARN_STREAM("[LOC] InitialPoseManager: rename tmp->last failed: tmp="
                      << tmp << " last=" << last_yaml_path_);
      std::remove(tmp.c_str());
      return false;
    }

    return true;

  } catch (const std::exception& e) {
    ROS_WARN_STREAM("[LOC] InitialPoseManager: write last yaml failed: " << e.what());
    return false;
  }
}


bool InitialPoseManager::updateLastPose(const Pose2D &pose) {
  std::lock_guard<std::mutex> lk(last_mu_);
  last_pose_ = pose;
  has_last_pose_ = true;

  if (!writeLastFile()) {
    ROS_WARN_STREAM("[LOC] InitialPoseManager: update last pose failed: "
                    << last_yaml_path_);
    return false;
  }
  ROS_INFO_STREAM("[LOC] InitialPoseManager: last pose updated: x="
                  << pose.x << " y=" << pose.y << " yaw=" << pose.yaw << " -> "
                  << last_yaml_path_);
  return true;
}

bool InitialPoseManager::updateShutdownPose(const Pose2D &pose) {
  std::lock_guard<std::mutex> lk(last_mu_);
  shutdown_pose_ = pose;
  has_shutdown_pose_ = true;

  if (!writeLastFile()) {
    ROS_WARN_STREAM("[LOC] InitialPoseManager: update shutdown pose failed: "
                    << last_yaml_path_);
    return false;
  }
  ROS_WARN_STREAM("[LOC] InitialPoseManager: shutdown pose updated: x="
                  << pose.x << " y=" << pose.y << " yaw=" << pose.yaw << " -> "
                  << last_yaml_path_);
  return true;
}



bool InitialPoseManager::bumpFixedPriority(size_t index, int inc) {
  if (index >= fixed_poses_.size())
    return false;
  fixed_poses_[index].priority += inc;
  if (!writeFixedFile()) {
    ROS_WARN_STREAM(
        "[LOC] InitialPoseManager: bump fixed priority writeback failed: "
        << fixed_yaml_path_);
    return false;
  }
  return true;
}

} // namespace localization_ndt
