#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace localization_ndt {

constexpr double kPiReloc = 3.14159265358979323846;

// 避免和 InitialPoseManager::Pose2D 冲突
struct RelocPose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0; // rad
};

struct LocalRelocResult {
  bool ok = false;
  RelocPose2D best_pose;
  double best_score = 0.0;
  double second_score = 0.0;
  double valid_fraction = 0.0;
  Eigen::Matrix4f T_seed = Eigen::Matrix4f::Identity();
};

struct LocalRelocMapMeta {
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  double resolution = 0.05;
  double size_w = 0.0;
  double size_h = 0.0;
};

// ======= LocalRelocOptions (REPLACE WHOLE STRUCT) =======
struct LocalRelocOptions {
  // ---------- search window ----------
  double xy_range_m = 3.0;
  double yaw_range_rad = 15.0 * kPiReloc / 180.0;

  // ---------- map pyramid build max level (loadMap time) ----------
  int pyr_max_level = 6;

  // ---------- score0 build params (loadMap time) ----------
  // 这两个用于 gm->buildScore0(sigma, maxd)
  double hit_sigma_m = 0.20; // sigma in exp(-d^2/(2*sigma^2))
  double max_dist_m = 1.00;  // d > max_dist -> score0=0

  // ---------- scan preprocess ----------
  int max_scan_points = 1200;
  double min_range = 1.0;
  double max_range = 35.0;
  double scan_voxel_m = 0.02;

  // ---------- acceptance gates ----------
  double min_score = 0.20;
  double score_margin = 0.02;
  double min_valid_fraction = 0.30;

  // ---------- BnB params ----------
  int bnb_max_level = 3;
  double yaw_step_rad = 1.0 * kPiReloc / 180.0;

};

// ======= LocalRelocalizer2D (REPLACE WHOLE CLASS DECLARATION) =======
class LocalRelocalizer2D {
public:
  LocalRelocalizer2D() = default;

  //  控制 loadMap() 时 buildPyramid 的最大层数
  void setPyrMaxLevel(int L) { pyr_max_level_ = std::max(0, L); }
  int pyrMaxLevel() const { return pyr_max_level_; }

  void setAllowExternalMapPaths(bool allow) {
    allow_external_map_paths_ = allow;
  }
  bool allowExternalMapPaths() const { return allow_external_map_paths_; }

  bool loadMap(const std::string &map_yaml, std::string *err);
  bool loadMapFromOccPoints(const LocalRelocMapMeta& meta,
                            const std::vector<std::pair<double, double>>& occupy_xy,
                            const uint8_t* known_mask_bytes,
                            size_t known_mask_len,
                            int known_w,
                            int known_h,
                            std::string* err);

  LocalRelocResult match(const std::vector<Eigen::Vector2f> &scan_xy_base,
                         const RelocPose2D &center,
                         const LocalRelocOptions &opt) const;

  const std::string &mapFrame() const { return map_frame_; }
  bool isReady() const { return ready_; }
  void setScoreParams(double sigma, double maxd) {
    score_sigma_ = sigma;
    score_maxd_ = maxd;
  }
  double scoreSigma() const { return score_sigma_; }
  double scoreMaxDist() const { return score_maxd_; }

private:
  struct GridMap;

  static Eigen::Matrix4f PoseToMat_(const RelocPose2D &p);
  static double NormalizeAngle_(double a);

  static std::vector<Eigen::Vector2f>
  PreprocessScan_(const std::vector<Eigen::Vector2f> &in,
                  const LocalRelocOptions &opt);

  std::pair<double, double> ScorePose_(const std::vector<Eigen::Vector2f> &scan,
                                       const RelocPose2D &pose,
                                       const LocalRelocOptions &opt) const;
  RelocPose2D RefinePoseSubcell_(const std::vector<Eigen::Vector2f> &scan,
                                 const RelocPose2D &seed,
                                 const LocalRelocOptions &opt,
                                 double *out_score,
                                 double *out_vf) const;

  struct GridPt {
    int16_t x = 0, y = 0;
  };
  using GridScan = std::vector<GridPt>;

  struct BnBResult {
    bool ok = false;
    int tx0 = 0;
    int ty0 = 0;
    uint64_t best_sum = 0;
    uint64_t second_sum = 0;
  };

  static GridScan
  BuildScanGrid0_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                  double yaw_map, const GridMap &m, double leaf_m);

  static std::vector<GridScan> BuildScanByLevel_(const GridScan &scan0,
                                                 int Lmax);

  BnBResult SearchXYBnB_(const std::vector<GridScan> &scanL, int center_tx0,
                         int center_ty0, double xy_range_m, int Lmax,
                         int max_nodes) const;

  uint64_t SumScoreAtL_(int L, int txL, int tyL, const GridScan &scanL) const;

  uint64_t SumScoreAtL0_(int tx0, int ty0, const GridScan &scan0,
                         int *out_known_cnt) const;

  RelocPose2D GridToPoseMap_(int tx0, int ty0, double yaw_map) const;

private:
  bool ready_ = false;
  std::string map_frame_ = "map";
  std::string map_yaml_path_;
  std::shared_ptr<GridMap> map_;

  // loadMap 时用这个
  int pyr_max_level_ = 6;

  double score_sigma_ = 0.20;
  double score_maxd_ = 1.00;
  bool allow_external_map_paths_ = false;

  Eigen::Vector2f fail_attempt_anchor_odom_xy_ = Eigen::Vector2f::Zero();
  bool has_fail_attempt_anchor_ = false;

  bool fail_moved_far_since_edge_ = false; // 一旦挪远就 true（FAIL 周期内不再回到 periodic）
};

} // namespace localization_ndt
