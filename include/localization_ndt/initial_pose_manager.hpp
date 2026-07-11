#pragma once

#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <mutex>   // NEW

namespace localization_ndt {

class InitialPoseManager {
public:
  struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;  // rad
  };

  struct FixedPoseEntry {
    Pose2D pose;
    int priority = 0;        // 越大越优先
    std::string name;        
  };

  InitialPoseManager() = default;

  void setFixedDir(const std::string& dir) { fixed_dir_ = dir; }
  void setLastDir(const std::string& dir)  { last_dir_  = dir; }

  bool initialize(const std::string& map_path,
                  const std::string& agv_name);
  bool initializeWithMapId(const std::string& map_path,
                           const std::string& map_id,
                           const std::string& agv_name);

  // ----- stable last（顶层 x/y/yaw）-----
  bool hasLastPose() const { return has_last_pose_; }
  const Pose2D& lastPose() const { return last_pose_; }
  bool updateLastPose(const Pose2D& pose);

  // ----- shutdown last（last.yaml 内的 shutdown: {x,y,yaw}）-----
  bool hasShutdownPose() const { return has_shutdown_pose_; }
  const Pose2D& shutdownPose() const { return shutdown_pose_; }
  bool updateShutdownPose(const Pose2D& pose);  // NEW

  // ----- fixed -----
  const std::vector<FixedPoseEntry>& fixedPoses() const { return fixed_poses_; }
  bool bumpFixedPriority(size_t index, int inc = 1);

  const std::string& fixedYamlPath() const { return fixed_yaml_path_; }
  const std::string& lastYamlPath()  const { return last_yaml_path_; }

  static bool isSamePose(const Pose2D& a, const Pose2D& b,
                         double eps_xy = 1e-4, double eps_yaw = 1e-4);

private:
  std::string extractStem(const std::string& path_or_name) const;

  bool loadFixedFile();
  bool writeFixedFileDefaultIfMissing();
  bool writeFixedFile();
  bool loadLastFileIfExists();

  bool writeLastFile();  // 写 stable + shutdown（同文件）

private:
  std::string fixed_dir_;
  std::string last_dir_;

  std::string map_stem_;
  std::string agv_stem_;

  std::string fixed_yaml_path_;
  std::string last_yaml_path_;

  std::vector<FixedPoseEntry> fixed_poses_;

  // stable
  Pose2D last_pose_;
  bool has_last_pose_ = false;

  // shutdown
  Pose2D shutdown_pose_;
  bool has_shutdown_pose_ = false;

  // protect last.yaml writes (stable/shutdown share one file)
  mutable std::mutex last_mu_;
};

}  // namespace localization_ndt
