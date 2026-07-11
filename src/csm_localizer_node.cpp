#include "localization_ndt/local_relocalizer_2d.hpp"
#include "localization_ndt/simple_predictor.hpp"
#include "localization_ndt/types.hpp"
#include "localization_ndt/dynamic_point_filter.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <limits>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_set>
#include <unordered_map>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace localization_ndt {
namespace {

constexpr double kPi = 3.14159265358979323846;

inline double Deg2Rad(double deg) { return deg * kPi / 180.0; }

inline double Clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

inline double NormalizeAngle(double a) {
  while (a > kPi)
    a -= 2.0 * kPi;
  while (a < -kPi)
    a += 2.0 * kPi;
  return a;
}

inline bool FileExists(const std::string &path) {
  std::ifstream ifs(path.c_str(), std::ios::binary);
  return ifs.good();
}

inline std::string Dirname(const std::string &path) {
  const std::size_t pos = path.find_last_of("/\\");
  if (pos == std::string::npos)
    return ".";
  if (pos == 0)
    return "/";
  return path.substr(0, pos);
}

inline std::string JoinPath(const std::string &dir, const std::string &name) {
  if (dir.empty())
    return name;
  if (!name.empty() && name[0] == '/')
    return name;
  if (dir.back() == '/')
    return dir + name;
  return dir + "/" + name;
}

struct Pose2D {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct ScanBuildStats {
  int raw_ranges = 0;
  int finite_ranges = 0;
  int invalid_ranges = 0;
  int out_of_range_ranges = 0;
  int range_kept = 0;
  int duplicate_points = 0;
  int output_points = 0;
  std::array<int, 4> output_sector_counts{{0, 0, 0, 0}};
  std::array<int, 8> output_angle_bins{{0, 0, 0, 0, 0, 0, 0, 0}};
  bool tf_ok = false;
  double tf_tx = 0.0;
  double tf_ty = 0.0;
  double tf_yaw = 0.0;
};

inline int SectorIndexFromAngle(double angle_rad) {
  const double a = NormalizeAngle(angle_rad);
  if (a >= -kPi / 4.0 && a < kPi / 4.0)
    return 0; // front
  if (a >= kPi / 4.0 && a < 3.0 * kPi / 4.0)
    return 1; // left
  if (a >= 3.0 * kPi / 4.0 || a < -3.0 * kPi / 4.0)
    return 2; // back
  return 3;   // right
}

inline int AngleBinIndex8(double angle_rad) {
  const double a = NormalizeAngle(angle_rad);
  const double shifted = a + kPi;
  int idx = static_cast<int>(std::floor(shifted / (kPi / 4.0)));
  if (idx < 0)
    idx = 0;
  if (idx > 7)
    idx = 7;
  return idx;
}

Pose2D MatToPose2D(const Eigen::Matrix4f &T) {
  Pose2D p;
  p.x = static_cast<double>(T(0, 3));
  p.y = static_cast<double>(T(1, 3));
  p.yaw =
      std::atan2(static_cast<double>(T(1, 0)), static_cast<double>(T(0, 0)));
  p.yaw = NormalizeAngle(p.yaw);
  return p;
}

Eigen::Matrix4f Pose2DToMat(const Pose2D &p) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(p.yaw));
  const float s = static_cast<float>(std::sin(p.yaw));
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  T(0, 3) = static_cast<float>(p.x);
  T(1, 3) = static_cast<float>(p.y);
  return T;
}

struct PgmImage {
  int width = 0;
  int height = 0;
  int maxval = 255;
  std::vector<uint8_t> data;
};

bool ReadToken(std::istream &is, std::string *out) {
  if (!out)
    return false;
  out->clear();

  while (true) {
    const int c = is.peek();
    if (c == EOF)
      return false;
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
    if (c == EOF || std::isspace(c) || c == '#')
      break;
    out->push_back(static_cast<char>(is.get()));
  }
  return !out->empty();
}

bool LoadPgm(const std::string &path, PgmImage *img, std::string *err) {
  if (!img)
    return false;

  std::ifstream ifs(path.c_str(), std::ios::binary);
  if (!ifs.is_open()) {
    if (err)
      *err = "open pgm failed: " + path;
    return false;
  }

  std::string magic;
  if (!ReadToken(ifs, &magic)) {
    if (err)
      *err = "pgm read magic failed";
    return false;
  }
  if (magic != "P5" && magic != "P2") {
    if (err)
      *err = "unsupported pgm magic: " + magic;
    return false;
  }

  std::string tok;
  if (!ReadToken(ifs, &tok))
    return false;
  img->width = std::stoi(tok);
  if (!ReadToken(ifs, &tok))
    return false;
  img->height = std::stoi(tok);
  if (!ReadToken(ifs, &tok))
    return false;
  img->maxval = std::stoi(tok);

  if (img->width <= 0 || img->height <= 0 || img->maxval <= 0) {
    if (err)
      *err = "invalid pgm header";
    return false;
  }

  img->data.assign(static_cast<std::size_t>(img->width) *
                       static_cast<std::size_t>(img->height),
                   0);

  if (magic == "P5") {
    ifs.get();
    if (img->maxval > 255) {
      if (err)
        *err = "16-bit pgm is not supported";
      return false;
    }
    ifs.read(reinterpret_cast<char *>(img->data.data()),
             static_cast<std::streamsize>(img->data.size()));
    if (!ifs.good()) {
      if (err)
        *err = "pgm binary payload is shorter than expected";
      return false;
    }
  } else {
    for (int i = 0; i < img->width * img->height; ++i) {
      if (!ReadToken(ifs, &tok)) {
        if (err)
          *err = "pgm ascii payload is shorter than expected";
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

struct BoundaryFeature {
  float x = 0.0f;
  float y = 0.0f;
  float nx = 0.0f;
  float ny = 0.0f;
};

class CsmMap {
public:
  bool loadFromYaml(const std::string &map_yaml, std::string *err) {
    YAML::Node y;
    try {
      y = YAML::LoadFile(map_yaml);
    } catch (const std::exception &e) {
      if (err)
        *err = std::string("yaml load failed: ") + e.what();
      return false;
    }

    if (!y["image"] || !y["resolution"] || !y["origin"] ||
        !y["origin"].IsSequence() || y["origin"].size() < 3) {
      if (err)
        *err = "yaml missing image/resolution/origin";
      return false;
    }

    const std::string image = y["image"].as<std::string>();
    res_ = static_cast<float>(y["resolution"].as<double>());
    if (!(res_ > 0.0f)) {
      if (err)
        *err = "resolution must be positive";
      return false;
    }

    ox_ = static_cast<float>(y["origin"][0].as<double>());
    oy_ = static_cast<float>(y["origin"][1].as<double>());
    oyaw_ = NormalizeAngle(y["origin"][2].as<double>());
    c_ = std::cos(oyaw_);
    s_ = std::sin(oyaw_);

    int negate = 0;
    double occupied_thresh = 0.65;
    double free_thresh = 0.196;
    if (y["negate"])
      negate = y["negate"].as<int>();
    if (y["occupied_thresh"])
      occupied_thresh = y["occupied_thresh"].as<double>();
    if (y["free_thresh"])
      free_thresh = y["free_thresh"].as<double>();

    yaml_frame_id_ = "map";
    if (y["frame_id"]) {
      try {
        yaml_frame_id_ = y["frame_id"].as<std::string>();
      } catch (...) {
        yaml_frame_id_ = "map";
      }
      if (yaml_frame_id_.empty())
        yaml_frame_id_ = "map";
    }

    const std::string image_path = resolveImagePath(map_yaml, image);
    if (image_path.empty()) {
      if (err)
        *err = "failed to resolve image path from yaml";
      return false;
    }

    PgmImage pgm;
    std::string pgm_err;
    if (!LoadPgm(image_path, &pgm, &pgm_err)) {
      if (err)
        *err = pgm_err;
      return false;
    }

    w_ = pgm.width;
    h_ = pgm.height;
    const std::size_t sz =
        static_cast<std::size_t>(w_) * static_cast<std::size_t>(h_);
    known_.assign(sz, 0);
    occ_.assign(sz, 0);
    known_cells_ = 0;
    occupied_cells_ = 0;
    unknown_cells_ = 0;

    for (int row = 0; row < h_; ++row) {
      const int y_flip = h_ - 1 - row;
      for (int x = 0; x < w_; ++x) {
        const std::size_t id = idx(x, y_flip);
        const uint8_t v =
            pgm.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(w_) +
                     static_cast<std::size_t>(x)];
        const double occ_prob =
            negate ? (double(v) / 255.0) : ((255.0 - double(v)) / 255.0);
        if (occ_prob > occupied_thresh) {
          known_[id] = 1;
          occ_[id] = 1;
          ++known_cells_;
          ++occupied_cells_;
        } else if (occ_prob < free_thresh) {
          known_[id] = 1;
          occ_[id] = 0;
          ++known_cells_;
        } else {
          known_[id] = 0;
          occ_[id] = 0;
          ++unknown_cells_;
        }
      }
    }

    buildBoundaryFeatures();
    ready_ = !features_->empty();
    if (!ready_) {
      if (err)
        *err = "boundary feature map is empty";
      return false;
    }

    ROS_INFO_STREAM("[CSM] map loaded: yaml="
                    << map_yaml << " image=" << image_path << " size=" << w_
                    << "x" << h_ << " res=" << res_ << " origin=(" << ox_ << ","
                    << oy_ << "," << oyaw_ << ")"
                    << " known=" << known_cells_ << " occupied="
                    << occupied_cells_ << " unknown=" << unknown_cells_
                    << " features=" << features_->size()
                    << " yaml_frame=" << yaml_frame_id_);
    return true;
  }

  bool isReady() const { return ready_; }
  const std::string &yamlFrameId() const { return yaml_frame_id_; }
  int width() const { return w_; }
  int height() const { return h_; }
  float resolution() const { return res_; }
  std::size_t knownCellCount() const { return known_cells_; }
  std::size_t occupiedCellCount() const { return occupied_cells_; }
  std::size_t unknownCellCount() const { return unknown_cells_; }
  std::size_t featureCount() const { return features_ ? features_->size() : 0; }
  PointCloudT::ConstPtr featureCloud() const { return features_; }

  bool nearestFeature(float mx, float my, float max_dist_m, BoundaryFeature *out,
                      float *out_sq_dist = nullptr) const {
    if (!ready_ || !out || !features_ || features_->empty())
      return false;

    PointT query;
    query.x = mx;
    query.y = my;
    query.z = 0.0f;
    query.intensity = 0.0f;

    std::vector<int> ids(1, -1);
    std::vector<float> sq_dists(1, std::numeric_limits<float>::infinity());
    if (kdtree_.nearestKSearch(query, 1, ids, sq_dists) <= 0)
      return false;

    const float max_sq = max_dist_m * max_dist_m;
    if (!(sq_dists[0] >= 0.0f) || sq_dists[0] > max_sq || ids[0] < 0 ||
        static_cast<std::size_t>(ids[0]) >= normals_.size()) {
      return false;
    }

    const PointT &pt = features_->points[static_cast<std::size_t>(ids[0])];
    const Eigen::Vector2f &n = normals_[static_cast<std::size_t>(ids[0])];
    out->x = pt.x;
    out->y = pt.y;
    out->nx = n.x();
    out->ny = n.y();
    if (out_sq_dist)
      *out_sq_dist = sq_dists[0];
    return true;
  }

private:
  std::size_t idx(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(w_) +
           static_cast<std::size_t>(x);
  }

  std::string resolveImagePath(const std::string &yaml_path,
                               const std::string &image_in) const {
    if (image_in.empty())
      return std::string();
    if (!image_in.empty() && image_in.front() == '/')
      return FileExists(image_in) ? image_in : std::string();
    const std::string p = JoinPath(Dirname(yaml_path), image_in);
    return FileExists(p) ? p : std::string();
  }

  bool isKnownFree(int x, int y) const {
    if (x < 0 || y < 0 || x >= w_ || y >= h_)
      return false;
    const std::size_t id = idx(x, y);
    return known_[id] != 0 && occ_[id] == 0;
  }

  void gridToWorld(double gx, double gy, double *wx, double *wy) const {
    *wx = static_cast<double>(ox_) + c_ * gx - s_ * gy;
    *wy = static_cast<double>(oy_) + s_ * gx + c_ * gy;
  }

  Eigen::Vector2f rotateNormal(double nx, double ny) const {
    return Eigen::Vector2f(static_cast<float>(c_ * nx - s_ * ny),
                           static_cast<float>(s_ * nx + c_ * ny));
  }

  void buildBoundaryFeatures() {
    struct Accum {
      double sx = 0.0;
      double sy = 0.0;
      double snx = 0.0;
      double sny = 0.0;
      int count = 0;
    };

    const double leaf = std::max(0.01, static_cast<double>(res_));
    auto pack = [](int ix, int iy) -> int64_t {
      return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
    };

    std::unordered_map<int64_t, Accum> accums;
    accums.reserve(static_cast<std::size_t>(occupied_cells_) * 2 + 1);

    const auto add_feature = [&](double gx, double gy, double nx, double ny) {
      double wx = 0.0;
      double wy = 0.0;
      gridToWorld(gx, gy, &wx, &wy);
      const Eigen::Vector2f n = rotateNormal(nx, ny);
      const int ix = static_cast<int>(std::floor(wx / leaf));
      const int iy = static_cast<int>(std::floor(wy / leaf));
      Accum &a = accums[pack(ix, iy)];
      a.sx += wx;
      a.sy += wy;
      a.snx += n.x();
      a.sny += n.y();
      ++a.count;
    };

    for (int y = 0; y < h_; ++y) {
      for (int x = 0; x < w_; ++x) {
        const std::size_t id = idx(x, y);
        if (!known_[id] || !occ_[id])
          continue;

        if (isKnownFree(x - 1, y)) {
          add_feature(static_cast<double>(x) * res_,
                      (static_cast<double>(y) + 0.5) * res_, -1.0, 0.0);
        }
        if (isKnownFree(x + 1, y)) {
          add_feature((static_cast<double>(x) + 1.0) * res_,
                      (static_cast<double>(y) + 0.5) * res_, 1.0, 0.0);
        }
        if (isKnownFree(x, y - 1)) {
          add_feature((static_cast<double>(x) + 0.5) * res_,
                      static_cast<double>(y) * res_, 0.0, -1.0);
        }
        if (isKnownFree(x, y + 1)) {
          add_feature((static_cast<double>(x) + 0.5) * res_,
                      (static_cast<double>(y) + 1.0) * res_, 0.0, 1.0);
        }
      }
    }

    features_.reset(new PointCloudT());
    normals_.clear();
    features_->points.reserve(accums.size());
    normals_.reserve(accums.size());

    for (const auto &kv : accums) {
      const Accum &a = kv.second;
      if (a.count <= 0)
        continue;
      Eigen::Vector2f n(static_cast<float>(a.snx), static_cast<float>(a.sny));
      const float n_norm = n.norm();
      if (!(n_norm > 1e-6f))
        continue;
      n /= n_norm;

      PointT pt;
      pt.x = static_cast<float>(a.sx / static_cast<double>(a.count));
      pt.y = static_cast<float>(a.sy / static_cast<double>(a.count));
      pt.z = 0.0f;
      pt.intensity = 1.0f;
      features_->points.push_back(pt);
      normals_.push_back(n);
    }

    features_->width = static_cast<uint32_t>(features_->points.size());
    features_->height = 1;
    features_->is_dense = true;

    if (!features_->empty())
      kdtree_.setInputCloud(features_);
  }

private:
  bool ready_ = false;
  int w_ = 0;
  int h_ = 0;
  float res_ = 0.05f;
  float ox_ = 0.0f;
  float oy_ = 0.0f;
  double oyaw_ = 0.0;
  double c_ = 1.0;
  double s_ = 0.0;
  std::string yaml_frame_id_ = "map";

  std::size_t known_cells_ = 0;
  std::size_t occupied_cells_ = 0;
  std::size_t unknown_cells_ = 0;
  std::vector<uint8_t> known_;
  std::vector<uint8_t> occ_;

  PointCloudT::Ptr features_;
  std::vector<Eigen::Vector2f> normals_;
  pcl::KdTreeFLANN<PointT> kdtree_;
};

struct CsmMatcherOptions {
  int max_iterations = 12;
  int min_valid_points = 80;
  double min_valid_fraction = 0.30;
  double correspondence_max_dist_m = 0.25;
  double huber_delta_m = 0.10;
  double max_step_trans_m = 0.15;
  double max_step_yaw_rad = Deg2Rad(5.0);
  double converge_trans_eps_m = 0.002;
  double converge_yaw_eps_rad = Deg2Rad(0.05);

  double lm_lambda_init = 1e-3;
  double lm_lambda_min = 1e-8;
  double lm_lambda_max = 1e6;
  double lm_lambda_up = 8.0;
  double lm_lambda_down = 0.5;
  double lm_diag_min = 1e-6;
  int lm_max_inner_trials = 6;
  double lm_accept_abs_cost_drop = 1e-6;
  double lm_accept_rel_cost_drop = 1e-4;

  bool use_range_weight = true;
  double range_weight_ref_m = 8.0;
  double range_weight_power = 1.0;
  double range_weight_min = 0.35;

  double improved_abs_min_cost_drop = 1e-3;
  double improved_rel_min_drop = 0.01;
};

struct CsmMatcherResult {
  bool solved = false;
  bool improved = false;
  bool converged = false;
  bool final_hessian_valid = false;
  Pose2D pose;
  int iterations = 0;
  int used_points = 0;
  int total_points = 0;
  int reject_no_correspondence = 0;
  double valid_fraction = 0.0;
  double initial_cost = std::numeric_limits<double>::quiet_NaN();
  double final_cost = std::numeric_limits<double>::quiet_NaN();
  double mean_abs_residual_m = std::numeric_limits<double>::quiet_NaN();
  std::array<int, 4> matched_sector_counts{{0, 0, 0, 0}};
  std::array<int, 8> matched_angle_bins{{0, 0, 0, 0, 0, 0, 0, 0}};
  Eigen::Matrix3d final_hessian = Eigen::Matrix3d::Zero();
  std::string reason;
};

class CsmScanMatcher {
public:
  CsmMatcherResult optimize(const std::vector<Eigen::Vector2f> &scan_xy_base,
                            const Pose2D &seed, const CsmMap &map,
                            const CsmMatcherOptions &opt) const {
    CsmMatcherResult out;
    out.pose = seed;

    if (scan_xy_base.empty()) {
      out.reason = "empty_scan";
      return out;
    }
    if (!map.isReady()) {
      out.reason = "map_not_ready";
      return out;
    }

    Pose2D cur = seed;
    Pose2D best_pose = seed;
    Eval best_eval;
    bool have_best = false;
    double lambda = std::min(opt.lm_lambda_max,
                             std::max(opt.lm_lambda_min, opt.lm_lambda_init));

    Eval cur_eval = evaluateAtPose(scan_xy_base, cur, map, opt);
    copyEval(cur_eval, &out);
    out.initial_cost = cur_eval.cost;
    out.final_cost = cur_eval.cost;

    if (!isEvalAcceptable(cur_eval, opt)) {
      out.reason = (cur_eval.used_points >= opt.min_valid_points)
                       ? "valid_fraction_too_low"
                       : "insufficient_valid_points";
      return out;
    }

    if (cur_eval.ok && std::isfinite(cur_eval.cost)) {
      best_pose = cur;
      best_eval = cur_eval;
      have_best = true;
      out.solved = true;
    }

    std::string stop_reason;
    for (int iter = 0; iter < opt.max_iterations; ++iter) {
      out.iterations = iter + 1;

      bool accepted_step = false;
      Pose2D next_pose = cur;
      Eval next_eval;
      Eigen::Vector3d accepted_delta = Eigen::Vector3d::Zero();

      for (int trial = 0; trial < opt.lm_max_inner_trials; ++trial) {
        Eigen::Matrix3d H_lm = cur_eval.H;
        addLmDiagonal(H_lm, lambda, opt.lm_diag_min);

        Eigen::LDLT<Eigen::Matrix3d> ldlt(H_lm);
        if (ldlt.info() != Eigen::Success) {
          lambda = std::min(opt.lm_lambda_max, lambda * opt.lm_lambda_up);
          continue;
        }

        Eigen::Vector3d delta = -ldlt.solve(cur_eval.b);
        if (ldlt.info() != Eigen::Success || !delta.allFinite()) {
          lambda = std::min(opt.lm_lambda_max, lambda * opt.lm_lambda_up);
          continue;
        }

        const double trans = std::hypot(delta.x(), delta.y());
        if (trans > opt.max_step_trans_m && trans > 1e-12) {
          const double scale = opt.max_step_trans_m / trans;
          delta.x() *= scale;
          delta.y() *= scale;
        }
        delta.z() = std::max(-opt.max_step_yaw_rad,
                             std::min(opt.max_step_yaw_rad, delta.z()));

        Pose2D trial_pose = cur;
        trial_pose.x += delta.x();
        trial_pose.y += delta.y();
        trial_pose.yaw = NormalizeAngle(trial_pose.yaw + delta.z());

        Eval trial_eval = evaluateAtPose(scan_xy_base, trial_pose, map, opt);
        if (!isEvalAcceptable(trial_eval, opt)) {
          lambda = std::min(opt.lm_lambda_max, lambda * opt.lm_lambda_up);
          continue;
        }

        const double abs_drop = cur_eval.cost - trial_eval.cost;
        const double rel_drop =
            abs_drop / std::max(1e-9, std::fabs(cur_eval.cost));
        const bool small_step =
            (std::hypot(delta.x(), delta.y()) < opt.converge_trans_eps_m) &&
            (std::fabs(delta.z()) < opt.converge_yaw_eps_rad);
        const bool accept_step =
            (abs_drop > opt.lm_accept_abs_cost_drop) ||
            (rel_drop > opt.lm_accept_rel_cost_drop) ||
            (small_step && trial_eval.cost <= cur_eval.cost + 1e-12);
        if (!accept_step) {
          lambda = std::min(opt.lm_lambda_max, lambda * opt.lm_lambda_up);
          continue;
        }

        accepted_step = true;
        next_pose = trial_pose;
        next_eval = trial_eval;
        accepted_delta = delta;
        lambda = std::max(opt.lm_lambda_min, lambda * opt.lm_lambda_down);
        break;
      }

      if (!accepted_step) {
        stop_reason = out.solved ? "lm_no_acceptable_step_after_solved"
                                 : "lm_no_acceptable_step";
        break;
      }

      cur = next_pose;
      cur_eval = next_eval;
      out.pose = cur;
      out.solved = true;
      out.final_cost = cur_eval.cost;
      copyEval(cur_eval, &out);

      if (!have_best || cur_eval.cost < best_eval.cost) {
        best_pose = cur;
        best_eval = cur_eval;
        have_best = true;
      }

      if (std::hypot(accepted_delta.x(), accepted_delta.y()) <
              opt.converge_trans_eps_m &&
          std::fabs(accepted_delta.z()) < opt.converge_yaw_eps_rad) {
        out.converged = true;
        out.reason = "ok";
        break;
      }
    }

    if (out.solved && have_best && !out.converged) {
      out.pose = best_pose;
      out.final_cost = best_eval.cost;
      copyEval(best_eval, &out);
    }

    if (out.solved && std::isfinite(out.initial_cost) &&
        std::isfinite(out.final_cost)) {
      const double abs_drop = out.initial_cost - out.final_cost;
      const double rel_drop =
          abs_drop / std::max(1e-9, std::fabs(out.initial_cost));
      out.improved = (abs_drop > opt.improved_abs_min_cost_drop) ||
                     (rel_drop > opt.improved_rel_min_drop);
    }

    if (out.reason.empty())
      out.reason = stop_reason.empty() ? "max_iter_reached" : stop_reason;
    return out;
  }

private:
  struct Eval {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    double cost = 0.0;
    double residual_abs_sum = 0.0;
    int used_points = 0;
    int total_points = 0;
    int reject_no_correspondence = 0;
    double valid_fraction = 0.0;
    std::array<int, 4> matched_sector_counts{{0, 0, 0, 0}};
    std::array<int, 8> matched_angle_bins{{0, 0, 0, 0, 0, 0, 0, 0}};
    bool ok = false;
  };

  static void addLmDiagonal(Eigen::Matrix3d &H, double lambda,
                            double diag_min) {
    H(0, 0) += lambda * std::max(diag_min, std::fabs(H(0, 0)));
    H(1, 1) += lambda * std::max(diag_min, std::fabs(H(1, 1)));
    H(2, 2) += lambda * std::max(diag_min, std::fabs(H(2, 2)));
  }

  static bool isEvalAcceptable(const Eval &e, const CsmMatcherOptions &opt) {
    return e.ok && e.used_points >= opt.min_valid_points &&
           e.valid_fraction >= opt.min_valid_fraction;
  }

  static void copyEval(const Eval &e, CsmMatcherResult *out) {
    if (!out)
      return;
    out->used_points = e.used_points;
    out->total_points = e.total_points;
    out->reject_no_correspondence = e.reject_no_correspondence;
    out->valid_fraction = e.valid_fraction;
    out->matched_sector_counts = e.matched_sector_counts;
    out->matched_angle_bins = e.matched_angle_bins;
    out->mean_abs_residual_m =
        (e.used_points > 0)
            ? (e.residual_abs_sum / static_cast<double>(std::max(1, e.used_points)))
            : std::numeric_limits<double>::quiet_NaN();
    out->final_hessian = e.H;
    out->final_hessian_valid = e.ok && e.H.allFinite();
  }

  static double computeRangeWeight(double bx, double by,
                                   const CsmMatcherOptions &opt) {
    if (!opt.use_range_weight)
      return 1.0;
    const double r = std::max(1e-6, std::hypot(bx, by));
    const double ref = std::max(1e-3, opt.range_weight_ref_m);
    if (r <= ref)
      return 1.0;
    const double w = std::pow(ref / r, std::max(0.0, opt.range_weight_power));
    return std::max(opt.range_weight_min, std::min(1.0, w));
  }

  Eval evaluateAtPose(const std::vector<Eigen::Vector2f> &scan_xy_base,
                      const Pose2D &pose, const CsmMap &map,
                      const CsmMatcherOptions &opt) const {
    Eval ev;
    ev.total_points = static_cast<int>(scan_xy_base.size());
    if (scan_xy_base.empty() || !map.isReady())
      return ev;

    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);

    for (const auto &pb : scan_xy_base) {
      const double bx = pb.x();
      const double by = pb.y();

      const double mx = pose.x + c * bx - s * by;
      const double my = pose.y + s * bx + c * by;

      BoundaryFeature feat;
      float sq_dist = 0.0f;
      if (!map.nearestFeature(static_cast<float>(mx), static_cast<float>(my),
                              static_cast<float>(opt.correspondence_max_dist_m),
                              &feat, &sq_dist)) {
        ++ev.reject_no_correspondence;
        continue;
      }

      const double residual =
          static_cast<double>(feat.nx) * (mx - static_cast<double>(feat.x)) +
          static_cast<double>(feat.ny) * (my - static_cast<double>(feat.y));
      const double abs_r = std::fabs(residual);
      const double huber = std::max(1e-6, opt.huber_delta_m);
      const double w_robust = (abs_r <= huber) ? 1.0 : (huber / abs_r);
      const double w_range = computeRangeWeight(bx, by, opt);
      const double w = std::max(1e-9, w_robust * w_range);

      const double dmx_dyaw = -s * bx - c * by;
      const double dmy_dyaw = c * bx - s * by;
      const double jyaw = static_cast<double>(feat.nx) * dmx_dyaw +
                          static_cast<double>(feat.ny) * dmy_dyaw;
      const Eigen::Vector3d J(static_cast<double>(feat.nx),
                              static_cast<double>(feat.ny), jyaw);

      ev.H.noalias() += w * (J * J.transpose());
      ev.b.noalias() += w * (J * residual);
      ev.cost += 0.5 * w * residual * residual;
      ev.residual_abs_sum += abs_r;
      const double bearing = std::atan2(by, bx);
      ++ev.matched_sector_counts[static_cast<std::size_t>(
          SectorIndexFromAngle(bearing))];
      ++ev.matched_angle_bins[static_cast<std::size_t>(
          AngleBinIndex8(bearing))];
      ++ev.used_points;
    }

    ev.valid_fraction =
        static_cast<double>(ev.used_points) /
        static_cast<double>(std::max(1, ev.total_points));
    ev.ok = (ev.used_points > 0) && std::isfinite(ev.cost) &&
            ev.H.allFinite() && ev.b.allFinite();
    return ev;
  }
};

enum class LocState : uint8_t {
  INIT = 0,
  RELOCALIZING = 1,
  TRACKING = 2,
  DR_TRACKING = 3,
  LOST = 4,
};

inline const char *LocStateName(LocState s) {
  switch (s) {
  case LocState::INIT:
    return "INIT";
  case LocState::RELOCALIZING:
    return "RELOCALIZING";
  case LocState::TRACKING:
    return "TRACKING";
  case LocState::DR_TRACKING:
    return "DR_TRACKING";
  case LocState::LOST:
    return "LOST";
  default:
    return "UNKNOWN";
  }
}

struct WeakDirDecision {
  bool valid = false;
  bool active = false;
  bool yaw_weak = false;
  double cond = std::numeric_limits<double>::quiet_NaN();
  double lambda_min = std::numeric_limits<double>::quiet_NaN();
  double lambda_max = std::numeric_limits<double>::quiet_NaN();
  double yaw_curv = std::numeric_limits<double>::quiet_NaN();
  double alpha_w = 1.0;
  double alpha_s = 1.0;
  double alpha_yaw = 1.0;
  Eigen::Vector2d weak_dir = Eigen::Vector2d::UnitX();
};

class CsmLocalizerNode {
public:
  CsmLocalizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_) {
    loadParams();
    loadMapOrThrow();
    initRos();
    logConfig();
  }

  void spin() { ros::spin(); }

private:
  void loadParams() {
    pnh_.param<std::string>("map_yaml_path", map_yaml_path_, std::string(""));
    pnh_.param<std::string>("scan_topic", scan_topic_, std::string("/scan"));
    pnh_.param<std::string>("odom_topic", odom_topic_,
                            std::string("/odom/wheel"));
    pnh_.param<std::string>("imu_topic", imu_topic_, std::string("/imu"));
    pnh_.param<std::string>("base_frame_id", base_frame_id_,
                            std::string("base_footprint"));
    pnh_.param<std::string>("map_frame_id", map_frame_id_, std::string("map"));
    pnh_.param<std::string>("odom_pub_topic", odom_pub_topic_,
                            std::string("csm_odom"));
    pnh_.param<std::string>("initialpose_topic", initialpose_topic_,
                            std::string("/initialpose"));

    pnh_.param("publish_tf", publish_tf_, true);
    pnh_.param("debug_verbose", debug_verbose_, true);
    pnh_.param("debug_log_throttle_sec", debug_log_throttle_sec_, 0.5);
    pnh_.param("debug_publish_scan_clouds", debug_publish_scan_clouds_, true);
    pnh_.param<std::string>("debug_pred_scan_cloud_topic",
                            debug_pred_scan_cloud_topic_,
                            std::string("csm_scan_pred_cloud"));
    pnh_.param<std::string>("debug_out_scan_cloud_topic",
                            debug_out_scan_cloud_topic_,
                            std::string("csm_scan_out_cloud"));
    pnh_.param("predictor_enable", predictor_enable_, true);
    pnh_.param("use_yaml_frame_id", use_yaml_frame_id_, false);
    pnh_.param("dr_mode_enable", dr_mode_enable_, true);
    pnh_.param("dr_max_predict_distance_m", dr_max_predict_distance_m_, 3.0);
    double dr_max_predict_yaw_deg = 45.0;
    pnh_.param("dr_max_predict_yaw_deg", dr_max_predict_yaw_deg, 45.0);
    dr_max_predict_yaw_rad_ = Deg2Rad(dr_max_predict_yaw_deg);
    pnh_.param("dr_recover_accept_frames", dr_recover_accept_frames_, 3);

    pnh_.param("scan_min_range", scan_min_range_, 0.0);
    pnh_.param("scan_max_range", scan_max_range_, 0.0);
    pnh_.param("max_scan_points", max_scan_points_, 1200);
    pnh_.param("scan_voxel_m", scan_voxel_m_, 0.02);
    pnh_.param("dyn_filter_enable", dyn_cfg_.enable, true);
    pnh_.param("dyn_grid_res_m", dyn_cfg_.grid_res_m, 0.10);
    pnh_.param("dyn_inflate_m", dyn_cfg_.inflate_m, 0.20);
    pnh_.param("dyn_padding_m", dyn_cfg_.padding_m, 5.0);
    pnh_.param("dyn_keep_ratio", dyn_cfg_.keep_ratio, 0.60);
    pnh_.param("dyn_min_keep_points", dyn_cfg_.min_keep_points, 200);
    pnh_.param("dyn_hard_max_dist_m", dyn_cfg_.hard_max_dist_m, 2.0);
    pnh_.param("dyn_ray_enable", dyn_cfg_.ray_enable, true);
    pnh_.param("dyn_ray_margin_near_m", dyn_cfg_.ray_margin_near_m, 0.20);
    pnh_.param("dyn_ray_max_range_m", dyn_cfg_.ray_max_range_m, 35.0);
    pnh_.param("dyn_ray_hit_eps_m", dyn_cfg_.ray_hit_eps_m, -1.0);
    pnh_.param("dyn_ray_step_m", dyn_cfg_.ray_step_m, -1.0);
    pnh_.param("dyn_ray_max_steps", dyn_cfg_.ray_max_steps, 256);
    pnh_.param("dyn_ray_confirm_df_m", dyn_cfg_.ray_confirm_df_m, 0.15);
    pnh_.param("dyn_front_protect_enable", dyn_cfg_.front_protect_enable, true);
    pnh_.param("dyn_front_protect_x_min_m",
               dyn_cfg_.front_protect_x_min_m, 0.10);
    pnh_.param("dyn_front_protect_x_max_m",
               dyn_cfg_.front_protect_x_max_m, 3.00);
    pnh_.param("dyn_front_protect_abs_y_max_m",
               dyn_cfg_.front_protect_abs_y_max_m, 0.80);
    pnh_.param("dyn_front_protect_use_y_axis",
               dyn_cfg_.front_protect_use_y_axis, true);
    pnh_.param("dyn_front_protect_forward_sign",
               dyn_cfg_.front_protect_forward_sign, 1.0);
    pnh_.param("dyn_front_protect_max_drop_ratio",
               dyn_cfg_.front_protect_max_drop_ratio, 0.45);
    pnh_.param("dyn_front_protect_min_in_points",
               dyn_cfg_.front_protect_min_in_points, 12);

    pnh_.param("startup_reloc_enable", startup_reloc_enable_, true);
    pnh_.param("startup_reloc_allow_external_map_paths",
               startup_reloc_allow_external_map_paths_, true);
    pnh_.param<std::string>("startup_reloc_map_yaml", startup_reloc_map_yaml_,
                            std::string(""));
    pnh_.param("lost_reloc_enable", lost_reloc_enable_, true);
    pnh_.param("lost_reloc_xy_range_m", lost_reloc_xy_range_m_, 3.0);
    double lost_reloc_yaw_range_deg = 45.0;
    pnh_.param("lost_reloc_yaw_range_deg", lost_reloc_yaw_range_deg, 45.0);
    lost_reloc_yaw_range_rad_ = Deg2Rad(lost_reloc_yaw_range_deg);
    pnh_.param("lost_reloc_min_score", lost_reloc_min_score_, 0.20);
    pnh_.param("lost_reloc_score_margin", lost_reloc_score_margin_, 0.00);
    pnh_.param("lost_reloc_min_valid_fraction", lost_reloc_min_valid_fraction_,
               0.25);
    pnh_.param("lost_reloc_min_interval_sec", lost_reloc_min_interval_sec_, 0.5);
    pnh_.param("lost_reloc_publish_smooth_enable",
               lost_reloc_publish_smooth_enable_, true);
    pnh_.param("lost_reloc_publish_smooth_frames",
               lost_reloc_publish_smooth_frames_, 4);
    pnh_.param("startup_reloc_xy_range_m", startup_reloc_opt_.xy_range_m, 5.0);
    pnh_.param("startup_reloc_min_score", startup_reloc_opt_.min_score, 0.20);
    pnh_.param("startup_reloc_score_margin", startup_reloc_opt_.score_margin,
               0.00);
    pnh_.param("startup_reloc_min_valid_fraction",
               startup_reloc_opt_.min_valid_fraction, 0.30);
    pnh_.param("startup_reloc_bnb_max_level", startup_reloc_opt_.bnb_max_level,
               startup_reloc_opt_.bnb_max_level);
    pnh_.param("startup_reloc_pyr_max_level", startup_reloc_opt_.pyr_max_level,
               startup_reloc_opt_.pyr_max_level);
    pnh_.param("startup_reloc_max_scan_points",
               startup_reloc_opt_.max_scan_points, 1200);
    pnh_.param("startup_reloc_scan_voxel_m", startup_reloc_opt_.scan_voxel_m,
               0.02);
    pnh_.param("startup_reloc_hit_sigma_m", startup_reloc_opt_.hit_sigma_m,
               0.20);
    pnh_.param("startup_reloc_max_dist_m", startup_reloc_opt_.max_dist_m, 1.00);
    pnh_.param("startup_reloc_min_range", startup_reloc_opt_.min_range, 0.0);
    pnh_.param("startup_reloc_max_range", startup_reloc_opt_.max_range, 0.0);

    double startup_reloc_yaw_range_deg = 180.0;
    double startup_reloc_yaw_step_deg = 1.0;
    pnh_.param("startup_reloc_yaw_range_deg", startup_reloc_yaw_range_deg,
               startup_reloc_yaw_range_deg);
    pnh_.param("startup_reloc_yaw_step_deg", startup_reloc_yaw_step_deg,
               startup_reloc_yaw_step_deg);
    startup_reloc_opt_.yaw_range_rad = Deg2Rad(startup_reloc_yaw_range_deg);
    startup_reloc_opt_.yaw_step_rad = Deg2Rad(startup_reloc_yaw_step_deg);

    pnh_.param("manual_reloc_enable", manual_reloc_enable_, true);
    pnh_.param("manual_reloc_xy_min_m", manual_reloc_xy_min_m_, 0.30);
    pnh_.param("manual_reloc_xy_sigma_k", manual_reloc_xy_sigma_k_, 3.0);
    pnh_.param("manual_reloc_xy_max_m", manual_reloc_xy_max_m_, 5.0);
    double manual_reloc_yaw_min_deg = 6.0;
    pnh_.param("manual_reloc_yaw_min_deg", manual_reloc_yaw_min_deg,
               manual_reloc_yaw_min_deg);
    manual_reloc_yaw_min_rad_ = Deg2Rad(manual_reloc_yaw_min_deg);
    pnh_.param("manual_reloc_yaw_sigma_k", manual_reloc_yaw_sigma_k_, 3.0);
    pnh_.param("manual_reloc_score_margin", manual_reloc_score_margin_, 0.0);

    pnh_.param("optimizer_max_iterations", match_opt_.max_iterations, 12);
    pnh_.param("optimizer_min_valid_points", match_opt_.min_valid_points, 80);
    pnh_.param("optimizer_min_valid_fraction", match_opt_.min_valid_fraction,
               0.30);
    pnh_.param("optimizer_corr_max_dist_m",
               match_opt_.correspondence_max_dist_m, 0.25);
    pnh_.param("optimizer_huber_delta_m", match_opt_.huber_delta_m, 0.10);
    pnh_.param("optimizer_max_step_trans_m", match_opt_.max_step_trans_m, 0.15);

    double max_step_yaw_deg = 5.0;
    double converge_yaw_deg = 0.05;
    pnh_.param("optimizer_max_step_yaw_deg", max_step_yaw_deg,
               max_step_yaw_deg);
    pnh_.param("optimizer_converge_yaw_eps_deg", converge_yaw_deg,
               converge_yaw_deg);
    pnh_.param("optimizer_converge_trans_eps_m",
               match_opt_.converge_trans_eps_m, 0.002);
    match_opt_.max_step_yaw_rad = Deg2Rad(max_step_yaw_deg);
    match_opt_.converge_yaw_eps_rad = Deg2Rad(converge_yaw_deg);

    pnh_.param("optimizer_lm_lambda_init", match_opt_.lm_lambda_init, 1e-3);
    pnh_.param("optimizer_lm_lambda_min", match_opt_.lm_lambda_min, 1e-8);
    pnh_.param("optimizer_lm_lambda_max", match_opt_.lm_lambda_max, 1e6);
    pnh_.param("optimizer_lm_lambda_up", match_opt_.lm_lambda_up, 8.0);
    pnh_.param("optimizer_lm_lambda_down", match_opt_.lm_lambda_down, 0.5);
    pnh_.param("optimizer_lm_diag_min", match_opt_.lm_diag_min, 1e-6);
    pnh_.param("optimizer_lm_max_inner_trials", match_opt_.lm_max_inner_trials,
               6);
    pnh_.param("optimizer_lm_accept_abs_cost_drop",
               match_opt_.lm_accept_abs_cost_drop, 1e-6);
    pnh_.param("optimizer_lm_accept_rel_cost_drop",
               match_opt_.lm_accept_rel_cost_drop, 1e-4);
    pnh_.param("optimizer_use_range_weight", match_opt_.use_range_weight, true);
    pnh_.param("optimizer_range_weight_ref_m", match_opt_.range_weight_ref_m,
               8.0);
    pnh_.param("optimizer_range_weight_power", match_opt_.range_weight_power,
               1.0);
    pnh_.param("optimizer_range_weight_min", match_opt_.range_weight_min, 0.35);
    pnh_.param("optimizer_improved_abs_min_cost_drop",
               match_opt_.improved_abs_min_cost_drop, 1e-3);
    pnh_.param("optimizer_improved_rel_min_drop",
               match_opt_.improved_rel_min_drop, 0.01);

    pnh_.param("output_correction_gain", correction_gain_, 0.8);
    pnh_.param("output_small_gain_scale", small_gain_scale_, 0.5);
    pnh_.param("output_hold_gain_scale", hold_gain_scale_, 0.3);
    pnh_.param("output_max_correction_trans_m", max_total_correction_trans_m_,
               0.30);
    double max_total_correction_yaw_deg = 10.0;
    pnh_.param("output_max_correction_yaw_deg", max_total_correction_yaw_deg,
               max_total_correction_yaw_deg);
    max_total_correction_yaw_rad_ = Deg2Rad(max_total_correction_yaw_deg);

    pnh_.param("accept_small_min_valid_fraction",
               accept_small_min_valid_fraction_, 0.60);
    pnh_.param("accept_small_min_used_points", accept_small_min_used_points_,
               250);
    pnh_.param("accept_small_max_corr_trans_m", accept_small_max_corr_trans_m_,
               0.03);
    double accept_small_max_corr_yaw_deg = 0.5;
    pnh_.param("accept_small_max_corr_yaw_deg", accept_small_max_corr_yaw_deg,
               accept_small_max_corr_yaw_deg);
    accept_small_max_corr_yaw_rad_ = Deg2Rad(accept_small_max_corr_yaw_deg);

    pnh_.param("accept_quality_enable", accept_quality_enable_, true);
    pnh_.param("accept_quality_min_valid_fraction",
               accept_quality_min_valid_fraction_, 0.80);
    pnh_.param("accept_quality_min_used_points",
               accept_quality_min_used_points_, 450);
    pnh_.param("accept_quality_max_reject_fraction",
               accept_quality_max_reject_fraction_, 0.25);
    pnh_.param("accept_quality_max_mean_cost", accept_quality_max_mean_cost_,
               0.0025);
    pnh_.param("accept_quality_max_corr_trans_m",
               accept_quality_max_corr_trans_m_, 0.06);
    double accept_quality_max_corr_yaw_deg = 0.80;
    pnh_.param("accept_quality_max_corr_yaw_deg",
               accept_quality_max_corr_yaw_deg,
               accept_quality_max_corr_yaw_deg);
    accept_quality_max_corr_yaw_rad_ = Deg2Rad(accept_quality_max_corr_yaw_deg);
    pnh_.param("accept_quality_gain_scale", accept_quality_gain_scale_, 0.25);

    pnh_.param("accept_hold_enable", accept_hold_enable_, false);
    pnh_.param("accept_hold_min_valid_fraction", accept_hold_min_valid_fraction_,
               0.75);
    pnh_.param("accept_hold_min_used_points", accept_hold_min_used_points_, 350);
    pnh_.param("accept_hold_max_corr_trans_m", accept_hold_max_corr_trans_m_,
               0.010);
    double accept_hold_max_corr_yaw_deg = 0.20;
    pnh_.param("accept_hold_max_corr_yaw_deg", accept_hold_max_corr_yaw_deg,
               accept_hold_max_corr_yaw_deg);
    accept_hold_max_corr_yaw_rad_ = Deg2Rad(accept_hold_max_corr_yaw_deg);

    pnh_.param("sector_degen_enable", sector_degen_enable_, true);
    pnh_.param("sector_degen_min_scan_points", sector_degen_min_scan_points_, 60);
    pnh_.param("sector_degen_front_min_match_ratio",
               sector_degen_front_min_match_ratio_, 0.45);
    pnh_.param("sector_degen_lr_balance_min", sector_degen_lr_balance_min_, 0.55);
    pnh_.param("sector_degen_fb_balance_min", sector_degen_fb_balance_min_, 0.45);
    pnh_.param("sector_degen_gain_scale", sector_degen_gain_scale_, 0.35);
    pnh_.param("sector_degen_max_corr_trans_m",
               sector_degen_max_corr_trans_m_, 0.03);
    double sector_degen_max_corr_yaw_deg = 0.30;
    pnh_.param("sector_degen_max_corr_yaw_deg",
               sector_degen_max_corr_yaw_deg, sector_degen_max_corr_yaw_deg);
    sector_degen_max_corr_yaw_rad_ = Deg2Rad(sector_degen_max_corr_yaw_deg);
    pnh_.param("front_degen_gain_scale", front_degen_gain_scale_, 0.60);
    pnh_.param("front_degen_forward_scale", front_degen_forward_scale_, 0.20);
    pnh_.param("front_degen_lateral_scale", front_degen_lateral_scale_, 1.00);
    pnh_.param("front_degen_yaw_scale", front_degen_yaw_scale_, 0.50);
    pnh_.param("lr_degen_gain_scale", lr_degen_gain_scale_, 0.60);
    pnh_.param("lr_degen_forward_scale", lr_degen_forward_scale_, 1.00);
    pnh_.param("lr_degen_lateral_scale", lr_degen_lateral_scale_, 0.25);
    pnh_.param("lr_degen_yaw_scale", lr_degen_yaw_scale_, 0.35);
    pnh_.param("combo_degen_predictor_only", combo_degen_predictor_only_, true);
    pnh_.param("weak_dir_fusion_enable", weak_dir_fusion_enable_, true);
    pnh_.param("weak_dir_min_used_points", weak_dir_min_used_points_, 200);
    pnh_.param("weak_dir_cond_on", weak_dir_cond_on_, 30.0);
    pnh_.param("weak_dir_cond_bad", weak_dir_cond_bad_, 120.0);
    pnh_.param("weak_dir_alpha_weak_min", weak_dir_alpha_weak_min_, 0.05);
    pnh_.param("weak_dir_alpha_strong", weak_dir_alpha_strong_, 1.0);
    pnh_.param("weak_dir_alpha_yaw", weak_dir_alpha_yaw_, 0.5);
    pnh_.param("weak_dir_lambda_min_thresh", weak_dir_lambda_min_thresh_, 1e-3);

    pnh_.param("state_reject_to_lost_count", state_reject_to_lost_count_, 6);

    debug_log_throttle_sec_ = std::max(0.1, debug_log_throttle_sec_);
    max_scan_points_ = std::max(1, max_scan_points_);
    scan_voxel_m_ = std::max(0.001, scan_voxel_m_);
    dr_max_predict_distance_m_ = std::max(0.0, dr_max_predict_distance_m_);
    dr_max_predict_yaw_rad_ =
        std::max(0.0, std::min(kPi, dr_max_predict_yaw_rad_));
    dr_recover_accept_frames_ = std::max(1, dr_recover_accept_frames_);
    dyn_cfg_.grid_res_m = std::max(0.02, dyn_cfg_.grid_res_m);
    dyn_cfg_.inflate_m = std::max(0.0, dyn_cfg_.inflate_m);
    dyn_cfg_.padding_m = std::max(0.0, dyn_cfg_.padding_m);
    dyn_cfg_.keep_ratio = std::max(0.05, std::min(1.0, dyn_cfg_.keep_ratio));
    dyn_cfg_.min_keep_points = std::max(1, dyn_cfg_.min_keep_points);
    if (dyn_cfg_.hard_max_dist_m > 0.0)
      dyn_cfg_.hard_max_dist_m = std::max(0.05, dyn_cfg_.hard_max_dist_m);
    dyn_cfg_.ray_margin_near_m = std::max(0.0, dyn_cfg_.ray_margin_near_m);
    dyn_cfg_.ray_max_range_m = std::max(1.0, dyn_cfg_.ray_max_range_m);
    dyn_cfg_.ray_max_steps = std::max(16, dyn_cfg_.ray_max_steps);
    dyn_cfg_.ray_confirm_df_m = std::max(0.0, dyn_cfg_.ray_confirm_df_m);
    dyn_cfg_.front_protect_x_min_m =
        std::max(0.0, dyn_cfg_.front_protect_x_min_m);
    dyn_cfg_.front_protect_x_max_m =
        std::max(dyn_cfg_.front_protect_x_min_m,
                 dyn_cfg_.front_protect_x_max_m);
    dyn_cfg_.front_protect_abs_y_max_m =
        std::max(0.0, dyn_cfg_.front_protect_abs_y_max_m);
    dyn_cfg_.front_protect_max_drop_ratio =
        std::max(0.0, std::min(1.0, dyn_cfg_.front_protect_max_drop_ratio));
    dyn_cfg_.front_protect_min_in_points =
        std::max(1, dyn_cfg_.front_protect_min_in_points);
    correction_gain_ = std::max(0.0, std::min(1.0, correction_gain_));
    small_gain_scale_ = std::max(0.0, std::min(1.0, small_gain_scale_));
    hold_gain_scale_ = std::max(0.0, std::min(1.0, hold_gain_scale_));
    max_total_correction_trans_m_ = std::max(0.0, max_total_correction_trans_m_);
    max_total_correction_yaw_rad_ =
        std::max(0.0, std::min(kPi, max_total_correction_yaw_rad_));

    accept_small_min_valid_fraction_ =
        std::max(0.0, std::min(1.0, accept_small_min_valid_fraction_));
    accept_small_min_used_points_ = std::max(1, accept_small_min_used_points_);
    accept_small_max_corr_trans_m_ =
        std::max(0.0, accept_small_max_corr_trans_m_);
    accept_small_max_corr_yaw_rad_ =
        std::max(0.0, std::min(kPi, accept_small_max_corr_yaw_rad_));

    accept_quality_min_valid_fraction_ =
        std::max(0.0, std::min(1.0, accept_quality_min_valid_fraction_));
    accept_quality_min_used_points_ =
        std::max(1, accept_quality_min_used_points_);
    accept_quality_max_reject_fraction_ =
        std::max(0.0, std::min(1.0, accept_quality_max_reject_fraction_));
    accept_quality_max_mean_cost_ =
        std::max(0.0, accept_quality_max_mean_cost_);
    accept_quality_max_corr_trans_m_ =
        std::max(0.0, accept_quality_max_corr_trans_m_);
    accept_quality_max_corr_yaw_rad_ =
        std::max(0.0, std::min(kPi, accept_quality_max_corr_yaw_rad_));
    accept_quality_gain_scale_ =
        std::max(0.0, std::min(1.0, accept_quality_gain_scale_));

    accept_hold_min_valid_fraction_ =
        std::max(0.0, std::min(1.0, accept_hold_min_valid_fraction_));
    accept_hold_min_used_points_ = std::max(1, accept_hold_min_used_points_);
    accept_hold_max_corr_trans_m_ =
        std::max(0.0, accept_hold_max_corr_trans_m_);
    accept_hold_max_corr_yaw_rad_ =
        std::max(0.0, std::min(kPi, accept_hold_max_corr_yaw_rad_));

    sector_degen_min_scan_points_ = std::max(1, sector_degen_min_scan_points_);
    sector_degen_front_min_match_ratio_ =
        std::max(0.0, std::min(1.0, sector_degen_front_min_match_ratio_));
    sector_degen_lr_balance_min_ =
        std::max(0.0, std::min(1.0, sector_degen_lr_balance_min_));
    sector_degen_fb_balance_min_ =
        std::max(0.0, std::min(1.0, sector_degen_fb_balance_min_));
    sector_degen_gain_scale_ =
        std::max(0.0, std::min(1.0, sector_degen_gain_scale_));
    sector_degen_max_corr_trans_m_ =
        std::max(0.0, sector_degen_max_corr_trans_m_);
    sector_degen_max_corr_yaw_rad_ =
        std::max(0.0, std::min(kPi, sector_degen_max_corr_yaw_rad_));
    front_degen_gain_scale_ =
        std::max(0.0, std::min(1.0, front_degen_gain_scale_));
    front_degen_forward_scale_ =
        std::max(0.0, std::min(1.0, front_degen_forward_scale_));
    front_degen_lateral_scale_ =
        std::max(0.0, std::min(1.0, front_degen_lateral_scale_));
    front_degen_yaw_scale_ =
        std::max(0.0, std::min(1.0, front_degen_yaw_scale_));
    lr_degen_gain_scale_ =
        std::max(0.0, std::min(1.0, lr_degen_gain_scale_));
    lr_degen_forward_scale_ =
        std::max(0.0, std::min(1.0, lr_degen_forward_scale_));
    lr_degen_lateral_scale_ =
        std::max(0.0, std::min(1.0, lr_degen_lateral_scale_));
    lr_degen_yaw_scale_ =
        std::max(0.0, std::min(1.0, lr_degen_yaw_scale_));
    weak_dir_min_used_points_ = std::max(1, weak_dir_min_used_points_);
    weak_dir_cond_on_ = std::max(1.0, weak_dir_cond_on_);
    weak_dir_cond_bad_ = std::max(weak_dir_cond_on_, weak_dir_cond_bad_);
    weak_dir_alpha_weak_min_ = Clamp01(weak_dir_alpha_weak_min_);
    weak_dir_alpha_strong_ = Clamp01(weak_dir_alpha_strong_);
    weak_dir_alpha_yaw_ = Clamp01(weak_dir_alpha_yaw_);
    weak_dir_lambda_min_thresh_ = std::max(1e-12, weak_dir_lambda_min_thresh_);
    state_reject_to_lost_count_ = std::max(1, state_reject_to_lost_count_);

    match_opt_.min_valid_points = std::max(1, match_opt_.min_valid_points);
    match_opt_.min_valid_fraction =
        std::max(0.0, std::min(1.0, match_opt_.min_valid_fraction));
    match_opt_.correspondence_max_dist_m =
        std::max(0.02, match_opt_.correspondence_max_dist_m);
    match_opt_.huber_delta_m = std::max(1e-3, match_opt_.huber_delta_m);
    match_opt_.max_step_trans_m = std::max(1e-3, match_opt_.max_step_trans_m);
    match_opt_.converge_trans_eps_m =
        std::max(1e-5, match_opt_.converge_trans_eps_m);
    match_opt_.lm_lambda_min = std::max(1e-12, match_opt_.lm_lambda_min);
    match_opt_.lm_lambda_init =
        std::max(match_opt_.lm_lambda_min, match_opt_.lm_lambda_init);
    match_opt_.lm_lambda_max =
        std::max(match_opt_.lm_lambda_init, match_opt_.lm_lambda_max);
    match_opt_.lm_lambda_up = std::max(1.1, match_opt_.lm_lambda_up);
    match_opt_.lm_lambda_down =
        std::min(0.95, std::max(0.1, match_opt_.lm_lambda_down));
    match_opt_.lm_diag_min = std::max(1e-12, match_opt_.lm_diag_min);
    match_opt_.lm_max_inner_trials =
        std::max(1, match_opt_.lm_max_inner_trials);
    match_opt_.lm_accept_abs_cost_drop =
        std::max(0.0, match_opt_.lm_accept_abs_cost_drop);
    match_opt_.lm_accept_rel_cost_drop =
        std::max(0.0, match_opt_.lm_accept_rel_cost_drop);
    match_opt_.range_weight_ref_m =
        std::max(0.1, match_opt_.range_weight_ref_m);
    match_opt_.range_weight_power =
        std::max(0.0, match_opt_.range_weight_power);
    match_opt_.range_weight_min =
        std::max(0.01, std::min(1.0, match_opt_.range_weight_min));

    startup_reloc_opt_.xy_range_m =
        std::max(0.0, startup_reloc_opt_.xy_range_m);
    startup_reloc_opt_.yaw_range_rad =
        std::max(0.0, std::fabs(startup_reloc_opt_.yaw_range_rad));
    startup_reloc_opt_.yaw_step_rad =
        std::max(Deg2Rad(0.25), std::fabs(startup_reloc_opt_.yaw_step_rad));
    startup_reloc_opt_.min_score =
        std::max(0.0, std::min(1.0, startup_reloc_opt_.min_score));
    startup_reloc_opt_.score_margin =
        std::max(0.0, std::min(1.0, startup_reloc_opt_.score_margin));
    startup_reloc_opt_.min_valid_fraction =
        std::max(0.0, std::min(1.0, startup_reloc_opt_.min_valid_fraction));
    startup_reloc_opt_.bnb_max_level =
        std::max(0, startup_reloc_opt_.bnb_max_level);
    startup_reloc_opt_.pyr_max_level =
        std::max(0, startup_reloc_opt_.pyr_max_level);
    startup_reloc_opt_.max_scan_points =
        std::max(1, startup_reloc_opt_.max_scan_points);
    startup_reloc_opt_.scan_voxel_m =
        std::max(0.005, startup_reloc_opt_.scan_voxel_m);
    startup_reloc_opt_.hit_sigma_m =
        std::max(1e-3, startup_reloc_opt_.hit_sigma_m);
    startup_reloc_opt_.max_dist_m =
        std::max(0.0, startup_reloc_opt_.max_dist_m);
    startup_reloc_opt_.min_range = std::max(0.0, startup_reloc_opt_.min_range);
    if (startup_reloc_opt_.max_range > 0.0) {
      startup_reloc_opt_.max_range =
          std::max(startup_reloc_opt_.min_range, startup_reloc_opt_.max_range);
    }

    manual_reloc_xy_min_m_ = std::max(0.0, manual_reloc_xy_min_m_);
    manual_reloc_xy_sigma_k_ = std::max(0.0, manual_reloc_xy_sigma_k_);
    manual_reloc_xy_max_m_ =
        std::max(manual_reloc_xy_min_m_, manual_reloc_xy_max_m_);
    manual_reloc_yaw_min_rad_ =
        std::max(0.0, std::min(kPi, manual_reloc_yaw_min_rad_));
    manual_reloc_yaw_sigma_k_ = std::max(0.0, manual_reloc_yaw_sigma_k_);
    manual_reloc_score_margin_ =
        std::max(0.0, std::min(1.0, manual_reloc_score_margin_));
    lost_reloc_xy_range_m_ = std::max(0.0, lost_reloc_xy_range_m_);
    lost_reloc_yaw_range_rad_ =
        std::max(0.0, std::min(kPi, std::fabs(lost_reloc_yaw_range_rad_)));
    lost_reloc_min_score_ = std::max(0.0, std::min(1.0, lost_reloc_min_score_));
    lost_reloc_score_margin_ =
        std::max(0.0, std::min(1.0, lost_reloc_score_margin_));
    lost_reloc_min_valid_fraction_ =
        std::max(0.0, std::min(1.0, lost_reloc_min_valid_fraction_));
    lost_reloc_min_interval_sec_ = std::max(0.0, lost_reloc_min_interval_sec_);
    lost_reloc_publish_smooth_frames_ =
        std::max(0, lost_reloc_publish_smooth_frames_);
  }

  void loadMapOrThrow() {
    if (map_yaml_path_.empty())
      throw std::runtime_error("~map_yaml_path is empty");

    std::string err;
    if (!map_.loadFromYaml(map_yaml_path_, &err))
      throw std::runtime_error("load map failed: " + err);

    if (use_yaml_frame_id_)
      map_frame_id_ = map_.yamlFrameId();

    dyn_filter_.setConfig(dyn_cfg_);
    if (dyn_cfg_.enable) {
      if (!dyn_filter_.setMap(map_.featureCloud())) {
        ROS_WARN_STREAM("[CSM] dyn_filter setMap failed -> disable dyn_filter");
        dyn_cfg_.enable = false;
        dyn_filter_.setConfig(dyn_cfg_);
      }
    }

    if (startup_reloc_enable_) {
      startup_local_reloc_.reset(new LocalRelocalizer2D());
      startup_local_reloc_->setAllowExternalMapPaths(
          startup_reloc_allow_external_map_paths_);
      startup_local_reloc_->setPyrMaxLevel(startup_reloc_opt_.pyr_max_level);
      startup_local_reloc_->setScoreParams(startup_reloc_opt_.hit_sigma_m,
                                           startup_reloc_opt_.max_dist_m);

      const std::string reloc_map_yaml = startup_reloc_map_yaml_.empty()
                                             ? map_yaml_path_
                                             : startup_reloc_map_yaml_;
      std::string reloc_err;
      if (!startup_local_reloc_->loadMap(reloc_map_yaml, &reloc_err)) {
        ROS_WARN_STREAM("[CSM] startup local_reloc map load failed, disable. yaml="
                        << reloc_map_yaml << " err=" << reloc_err);
        startup_local_reloc_.reset();
        startup_reloc_enable_ = false;
      } else {
        ROS_INFO_STREAM("[CSM] startup local_reloc map ready: yaml="
                        << reloc_map_yaml
                        << " frame=" << startup_local_reloc_->mapFrame()
                        << " pyr_Lmax=" << startup_local_reloc_->pyrMaxLevel());
      }
    }
  }

  void initRos() {
    odom_sub_ =
        nh_.subscribe(odom_topic_, 100, &CsmLocalizerNode::odomCallback, this);
    imu_sub_ =
        nh_.subscribe(imu_topic_, 200, &CsmLocalizerNode::imuCallback, this);
    scan_sub_ =
        nh_.subscribe(scan_topic_, 5, &CsmLocalizerNode::scanCallback, this);
    initialpose_sub_ = nh_.subscribe(
        initialpose_topic_, 2, &CsmLocalizerNode::initialPoseCallback, this);
    pose_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_pub_topic_, 10, false);
    if (debug_publish_scan_clouds_) {
      pred_scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
          debug_pred_scan_cloud_topic_, 5, false);
      out_scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
          debug_out_scan_cloud_topic_, 5, false);
    }
  }

  void logConfig() const {
    std::ostringstream oss;
    oss << std::endl
        << "================ CSM Localizer =================" << std::endl
        << "map_yaml=" << map_yaml_path_ << std::endl
        << "debug   : verbose=" << (debug_verbose_ ? 1 : 0)
        << " throttle=" << debug_log_throttle_sec_ << "s"
        << " clouds=" << (debug_publish_scan_clouds_ ? 1 : 0)
        << " predictor=" << (predictor_enable_ ? 1 : 0)
        << " dr_mode=" << (dr_mode_enable_ ? 1 : 0)
        << " dr_budget=(" << dr_max_predict_distance_m_ << "m,"
        << dr_max_predict_yaw_rad_ * 180.0 / kPi << "deg)"
        << " dr_recover=" << dr_recover_accept_frames_ << "f" << std::endl
        << "topics  : scan=" << scan_topic_ << " odom=" << odom_topic_
        << " imu=" << imu_topic_ << " pub=" << odom_pub_topic_
        << " initialpose=" << initialpose_topic_ << std::endl
        << "frames  : map=" << map_frame_id_ << " base=" << base_frame_id_
        << " (yaml_frame=" << map_.yamlFrameId()
        << " use_yaml=" << (use_yaml_frame_id_ ? 1 : 0) << ")" << std::endl
        << "map     : size=" << map_.width() << "x" << map_.height()
        << " res=" << map_.resolution() << " known=" << map_.knownCellCount()
        << " occ=" << map_.occupiedCellCount()
        << " unk=" << map_.unknownCellCount()
        << " feat=" << map_.featureCount() << std::endl
        << "scan    : min=" << scan_min_range_ << " max=" << scan_max_range_
        << " voxel=" << scan_voxel_m_ << " max_pts=" << max_scan_points_
        << std::endl
        << "dyn     : enable=" << (dyn_cfg_.enable ? 1 : 0)
        << " grid=" << dyn_cfg_.grid_res_m
        << " inflate=" << dyn_cfg_.inflate_m
        << " keep=" << dyn_cfg_.keep_ratio
        << " min_keep=" << dyn_cfg_.min_keep_points
        << " ray=" << (dyn_cfg_.ray_enable ? 1 : 0)
        << " front=" << (dyn_cfg_.front_protect_enable ? 1 : 0)
        << std::endl
        << "optim   : iter=" << match_opt_.max_iterations
        << " corr<=" << match_opt_.correspondence_max_dist_m
        << " valid(pts>=" << match_opt_.min_valid_points
        << " vf>=" << match_opt_.min_valid_fraction << ")"
        << " step<(" << match_opt_.max_step_trans_m << "m,"
        << match_opt_.max_step_yaw_rad * 180.0 / kPi << "deg)"
        << " huber=" << match_opt_.huber_delta_m << " conv<("
        << match_opt_.converge_trans_eps_m << "m,"
        << match_opt_.converge_yaw_eps_rad * 180.0 / kPi << "deg)"
        << std::endl
        << "startup : enable=" << (startup_reloc_enable_ ? 1 : 0)
        << " map="
        << (startup_reloc_map_yaml_.empty() ? map_yaml_path_
                                            : startup_reloc_map_yaml_)
        << " win=(" << startup_reloc_opt_.xy_range_m << "m,"
        << startup_reloc_opt_.yaw_range_rad * 180.0 / kPi << "deg)"
        << " yaw_step=" << startup_reloc_opt_.yaw_step_rad * 180.0 / kPi
        << "deg" << std::endl
        << "lost    : enable=" << (lost_reloc_enable_ ? 1 : 0)
        << " win=(" << lost_reloc_xy_range_m_ << "m,"
        << lost_reloc_yaw_range_rad_ * 180.0 / kPi << "deg)"
        << " min_score=" << lost_reloc_min_score_
        << " margin=" << lost_reloc_score_margin_
        << " min_vf=" << lost_reloc_min_valid_fraction_
        << " dt>=" << lost_reloc_min_interval_sec_ << "s"
        << " smooth=" << (lost_reloc_publish_smooth_enable_ ? 1 : 0)
        << "/" << lost_reloc_publish_smooth_frames_ << "f" << std::endl
        << "manual  : enable=" << (manual_reloc_enable_ ? 1 : 0)
        << " win(cov)=max(" << manual_reloc_xy_min_m_ << "m,"
        << manual_reloc_xy_sigma_k_ << "*sigma_xy)"
        << " <= " << manual_reloc_xy_max_m_ << "m"
        << " yaw=max(" << manual_reloc_yaw_min_rad_ * 180.0 / kPi << "deg,"
        << manual_reloc_yaw_sigma_k_ << "*sigma_yaw) score_margin="
        << manual_reloc_score_margin_ << std::endl
        << "output  : gain=" << correction_gain_
        << " small_gain=" << small_gain_scale_
        << " quality_gain=" << accept_quality_gain_scale_
        << " hold_gain=" << hold_gain_scale_ << " max_corr=("
        << max_total_correction_trans_m_ << "m,"
        << max_total_correction_yaw_rad_ * 180.0 / kPi << "deg)"
        << " accept_small(vf>=" << accept_small_min_valid_fraction_
        << " used>=" << accept_small_min_used_points_ << " corr<("
        << accept_small_max_corr_trans_m_ << "m,"
        << accept_small_max_corr_yaw_rad_ * 180.0 / kPi << "deg))"
        << " quality(en=" << (accept_quality_enable_ ? 1 : 0)
        << " vf>=" << accept_quality_min_valid_fraction_
        << " used>=" << accept_quality_min_used_points_
        << " rej<=" << accept_quality_max_reject_fraction_
        << " mean<=" << accept_quality_max_mean_cost_
        << " corr<(" << accept_quality_max_corr_trans_m_ << "m,"
        << accept_quality_max_corr_yaw_rad_ * 180.0 / kPi
        << "deg))"
        << " hold_en=" << (accept_hold_enable_ ? 1 : 0)
        << " sector_degen(en=" << (sector_degen_enable_ ? 1 : 0)
        << " min_scan=" << sector_degen_min_scan_points_
        << " front>=" << sector_degen_front_min_match_ratio_
        << " lr>=" << sector_degen_lr_balance_min_
        << " fb>=" << sector_degen_fb_balance_min_
        << " gain=" << sector_degen_gain_scale_
        << " corr<(" << sector_degen_max_corr_trans_m_ << "m,"
        << sector_degen_max_corr_yaw_rad_ * 180.0 / kPi << "deg))"
        << " front_degen(g=" << front_degen_gain_scale_
        << " fwd=" << front_degen_forward_scale_
        << " lat=" << front_degen_lateral_scale_
        << " yaw=" << front_degen_yaw_scale_ << ")"
        << " lr_degen(g=" << lr_degen_gain_scale_
        << " fwd=" << lr_degen_forward_scale_
        << " lat=" << lr_degen_lateral_scale_
        << " yaw=" << lr_degen_yaw_scale_ << ")"
        << " combo_predictor=" << (combo_degen_predictor_only_ ? 1 : 0)
        << " weak_dir(en=" << (weak_dir_fusion_enable_ ? 1 : 0)
        << " used>=" << weak_dir_min_used_points_
        << " cond_on=" << weak_dir_cond_on_
        << " cond_bad=" << weak_dir_cond_bad_
        << " alpha(w/s/y)=" << weak_dir_alpha_weak_min_ << "/"
        << weak_dir_alpha_strong_ << "/" << weak_dir_alpha_yaw_
        << " lam_min>=" << weak_dir_lambda_min_thresh_ << ")"
        << " dr_fail=(" << dr_max_predict_distance_m_ << "m,"
        << dr_max_predict_yaw_rad_ * 180.0 / kPi << "deg)"
        << " legacy_reject_to_lost=" << state_reject_to_lost_count_ << std::endl
        << "================================================" << std::endl;
    ROS_INFO_STREAM(oss.str());
  }

  void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg) {
    predictor_.updateOdom(odom_msg);
    last_odom_msg_ = *odom_msg;
    has_last_odom_ = true;
  }

  void imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
    predictor_.updateImu(imu_msg);
    last_imu_gyro_z_ = imu_msg->angular_velocity.z;
    has_last_imu_ = true;
  }

  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {
    if (!msg)
      return;

    Pose2D p;
    p.x = msg->pose.pose.position.x;
    p.y = msg->pose.pose.position.y;

    const auto &q = msg->pose.pose.orientation;
    Eigen::Quaterniond qq(q.w, q.x, q.y, q.z);
    qq.normalize();
    p.yaw = std::atan2(2.0 * (qq.w() * qq.z() + qq.x() * qq.y()),
                       1.0 - 2.0 * (qq.y() * qq.y() + qq.z() * qq.z()));
    p.yaw = NormalizeAngle(p.yaw);

    const Eigen::Matrix4f T_manual = Pose2DToMat(p);
    resetPredictorWithPose_(T_manual);
    last_output_ = T_manual;
    has_last_output_ = true;
    last_published_output_ = T_manual;
    has_last_published_output_ = true;
    lost_reloc_publish_frames_left_ = 0;
    noteSeed_("manual_seed");
    const ros::Time stamp =
        msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    publishOdom(stamp, T_manual);

    manual_reloc_pending_ = false;
    if (manual_reloc_enable_) {
      const auto &cov = msg->pose.covariance;
      const double cov_xx = std::max(0.0, cov[0]);
      const double cov_yy = std::max(0.0, cov[7]);
      const double cov_yaw = std::max(0.0, cov[35]);
      const double sigma_xy = std::sqrt(std::max(cov_xx, cov_yy));
      const double sigma_yaw = std::sqrt(cov_yaw);

      manual_reloc_center_.x = p.x;
      manual_reloc_center_.y = p.y;
      manual_reloc_center_.yaw = p.yaw;
      manual_reloc_xy_range_m_ =
          std::max(manual_reloc_xy_min_m_, manual_reloc_xy_sigma_k_ * sigma_xy);
      manual_reloc_xy_range_m_ =
          std::min(manual_reloc_xy_max_m_, manual_reloc_xy_range_m_);
      manual_reloc_yaw_range_rad_ = std::max(
          manual_reloc_yaw_min_rad_, manual_reloc_yaw_sigma_k_ * sigma_yaw);
      manual_reloc_yaw_range_rad_ = std::min(kPi, manual_reloc_yaw_range_rad_);
      manual_reloc_pending_ = true;
      ROS_INFO_STREAM("[CSM] manual initialpose queued: x=" << p.x
                      << " y=" << p.y << " yaw=" << p.yaw << " win=("
                      << manual_reloc_xy_range_m_ << "m,"
                      << manual_reloc_yaw_range_rad_ * 180.0 / kPi << "deg)");
    } else {
      ROS_INFO_STREAM("[CSM] manual initialpose applied directly: x="
                      << p.x << " y=" << p.y << " yaw=" << p.yaw);
    }
  }

  bool lookupBaseFromScan(const sensor_msgs::LaserScanConstPtr &scan_msg,
                          double *tx, double *ty, double *yaw) {
    if (!tx || !ty || !yaw)
      return false;

    geometry_msgs::TransformStamped tf_bl;
    try {
      tf_bl =
          tf_buffer_.lookupTransform(base_frame_id_, scan_msg->header.frame_id,
                                     ros::Time(0), ros::Duration(0.01));
    } catch (...) {
      try {
        tf_bl = tf_buffer_.lookupTransform(
            base_frame_id_, scan_msg->header.frame_id, scan_msg->header.stamp,
            ros::Duration(0.01));
      } catch (const std::exception &e) {
        ROS_WARN_STREAM_THROTTLE(1.0,
                                 "[CSM] lookup base<-scan failed: " << e.what());
        return false;
      }
    }

    *tx = tf_bl.transform.translation.x;
    *ty = tf_bl.transform.translation.y;
    const auto &q = tf_bl.transform.rotation;
    Eigen::Quaterniond qq(q.w, q.x, q.y, q.z);
    qq.normalize();
    *yaw = std::atan2(2.0 * (qq.w() * qq.z() + qq.x() * qq.y()),
                      1.0 - 2.0 * (qq.y() * qq.y() + qq.z() * qq.z()));
    *yaw = NormalizeAngle(*yaw);
    return true;
  }

  std::vector<Eigen::Vector2f>
  buildScanPointsBase(const sensor_msgs::LaserScanConstPtr &scan_msg,
                      ScanBuildStats *stats = nullptr) {
    std::vector<Eigen::Vector2f> out;
    if (stats) {
      *stats = ScanBuildStats();
      stats->raw_ranges = static_cast<int>(scan_msg->ranges.size());
    }
    out.reserve(std::min<int>(max_scan_points_,
                              static_cast<int>(scan_msg->ranges.size())));

    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (!lookupBaseFromScan(scan_msg, &tx, &ty, &yaw))
      return out;
    if (stats) {
      stats->tf_ok = true;
      stats->tf_tx = tx;
      stats->tf_ty = ty;
      stats->tf_yaw = yaw;
    }

    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    const double rmin = (scan_min_range_ > 0.0)
                            ? std::max(scan_min_range_,
                                       static_cast<double>(scan_msg->range_min))
                            : static_cast<double>(scan_msg->range_min);
    const double rmax_raw = static_cast<double>(scan_msg->range_max);
    const double rmax = (scan_max_range_ > 0.0)
                            ? std::min(scan_max_range_, rmax_raw)
                            : rmax_raw;

    auto pack = [](int ix, int iy) -> uint64_t {
      return (uint64_t(uint32_t(ix)) << 32) | uint32_t(iy);
    };
    std::unordered_set<uint64_t> used;
    used.reserve(static_cast<std::size_t>(max_scan_points_) * 2);

    const double leaf = std::max(0.001, scan_voxel_m_);
    const int N = static_cast<int>(scan_msg->ranges.size());
    for (int i = 0; i < N; ++i) {
      const float r = scan_msg->ranges[static_cast<std::size_t>(i)];
      if (!std::isfinite(r)) {
        if (stats)
          ++stats->invalid_ranges;
        continue;
      }
      if (stats)
        ++stats->finite_ranges;
      if (r < rmin || r > rmax) {
        if (stats)
          ++stats->out_of_range_ranges;
        continue;
      }
      if (stats)
        ++stats->range_kept;

      const double ang = static_cast<double>(scan_msg->angle_min) +
                         static_cast<double>(i) *
                             static_cast<double>(scan_msg->angle_increment);
      const double lx = static_cast<double>(r) * std::cos(ang);
      const double ly = static_cast<double>(r) * std::sin(ang);
      const double bx = c * lx - s * ly + tx;
      const double by = s * lx + c * ly + ty;

      const int ix = static_cast<int>(std::floor(bx / leaf));
      const int iy = static_cast<int>(std::floor(by / leaf));
      const uint64_t key = pack(ix, iy);
      if (!used.insert(key).second) {
        if (stats)
          ++stats->duplicate_points;
        continue;
      }

      out.emplace_back(static_cast<float>(bx), static_cast<float>(by));
      if (stats) {
        const double bearing = std::atan2(by, bx);
        ++stats->output_sector_counts[static_cast<std::size_t>(
            SectorIndexFromAngle(bearing))];
        ++stats->output_angle_bins[static_cast<std::size_t>(
            AngleBinIndex8(bearing))];
      }
      if (stats)
        stats->output_points = static_cast<int>(out.size());
      if (static_cast<int>(out.size()) >= max_scan_points_)
        break;
    }

    return out;
  }

  bool buildTranslationSchurFromHessian_(const Eigen::Matrix3d &H,
                                         Eigen::Matrix2d *Ht,
                                         double *Hyy) const {
    if (!Ht || !H.allFinite())
      return false;
    Eigen::Matrix2d out = H.topLeftCorner<2, 2>();
    const Eigen::Vector2d hty = H.topRightCorner<2, 1>();
    const double hyy = H(2, 2);
    if (Hyy)
      *Hyy = hyy;
    if (std::isfinite(hyy) && std::fabs(hyy) > 1e-12 && hty.allFinite())
      out.noalias() -= (hty * hty.transpose()) / hyy;
    out = 0.5 * (out + out.transpose());
    if (!out.allFinite())
      return false;
    *Ht = out;
    return true;
  }

  WeakDirDecision evaluateWeakDirDecision_(const CsmMatcherResult &mres) const {
    WeakDirDecision out;
    if (!weak_dir_fusion_enable_ || !mres.final_hessian_valid ||
        mres.used_points < weak_dir_min_used_points_) {
      return out;
    }

    Eigen::Matrix2d Ht = Eigen::Matrix2d::Zero();
    double hyy = std::numeric_limits<double>::quiet_NaN();
    if (!buildTranslationSchurFromHessian_(mres.final_hessian, &Ht, &hyy))
      return out;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(Ht);
    if (es.info() != Eigen::Success)
      return out;

    const Eigen::Vector2d eig = es.eigenvalues();
    int idx_min_abs = 0;
    double min_abs = std::fabs(eig(0));
    double max_abs = min_abs;
    for (int i = 1; i < 2; ++i) {
      const double a = std::fabs(eig(i));
      if (a < min_abs) {
        min_abs = a;
        idx_min_abs = i;
      }
      if (a > max_abs)
        max_abs = a;
    }

    if (!std::isfinite(min_abs) || !std::isfinite(max_abs) || max_abs <= 0.0)
      return out;

    Eigen::Vector2d weak_dir = es.eigenvectors().col(idx_min_abs);
    if (!weak_dir.allFinite() || weak_dir.norm() < 1e-6)
      weak_dir = Eigen::Vector2d::UnitX();
    else
      weak_dir.normalize();

    out.valid = true;
    out.lambda_min = min_abs;
    out.lambda_max = max_abs;
    out.cond = max_abs / std::max(min_abs, weak_dir_lambda_min_thresh_);
    out.yaw_curv = std::fabs(hyy);
    out.yaw_weak =
        (!std::isfinite(out.yaw_curv) ||
         out.yaw_curv <= std::max(weak_dir_lambda_min_thresh_, out.lambda_min));
    out.weak_dir = weak_dir;
    out.alpha_w = 1.0;
    out.alpha_s = weak_dir_alpha_strong_;
    out.alpha_yaw = 1.0;

    if (!std::isfinite(out.cond) || out.cond < weak_dir_cond_on_)
      return out;

    out.active = true;
    if (out.cond >= weak_dir_cond_bad_) {
      out.alpha_w = weak_dir_alpha_weak_min_;
    } else {
      const double log_on = std::log(std::max(1.0, weak_dir_cond_on_));
      const double log_bad = std::log(std::max(weak_dir_cond_on_ + 1e-9,
                                               weak_dir_cond_bad_));
      const double log_cond = std::log(std::max(weak_dir_cond_on_, out.cond));
      const double t =
          (log_cond - log_on) / std::max(1e-9, log_bad - log_on);
      out.alpha_w = (1.0 - Clamp01(t)) * 1.0 +
                    Clamp01(t) * weak_dir_alpha_weak_min_;
    }
    out.alpha_w = std::max(weak_dir_alpha_weak_min_, Clamp01(out.alpha_w));
    out.alpha_s = Clamp01(out.alpha_s);
    out.alpha_yaw = out.yaw_weak
                        ? std::min(Clamp01(weak_dir_alpha_yaw_),
                                   std::max(weak_dir_alpha_weak_min_,
                                            out.alpha_w))
                        : 1.0;
    return out;
  }

  Pose2D fusePoseByWeakDir_(const Pose2D &pred, const Pose2D &opt,
                            const WeakDirDecision &decision) const {
    if (!decision.active)
      return opt;

    Eigen::Vector2d ew = decision.weak_dir;
    if (!ew.allFinite() || ew.norm() < 1e-6)
      ew = Eigen::Vector2d::UnitX();
    else
      ew.normalize();
    const Eigen::Vector2d es(-ew.y(), ew.x());
    const Eigen::Vector2d t_pred(pred.x, pred.y);
    const Eigen::Vector2d t_opt(opt.x, opt.y);
    const Eigen::Vector2d dt = t_opt - t_pred;
    const double dw = dt.dot(ew);
    const double ds = dt.dot(es);
    const Eigen::Vector2d t_out =
        t_pred + Clamp01(decision.alpha_w) * dw * ew +
        Clamp01(decision.alpha_s) * ds * es;

    Pose2D out = opt;
    out.x = t_out.x();
    out.y = t_out.y();
    out.yaw = NormalizeAngle(pred.yaw + Clamp01(decision.alpha_yaw) *
                                            NormalizeAngle(opt.yaw - pred.yaw));
    return out;
  }

  Pose2D interpolatePose_(const Pose2D &from, const Pose2D &to, double t) const {
    const double u = Clamp01(t);
    Pose2D out;
    out.x = from.x + u * (to.x - from.x);
    out.y = from.y + u * (to.y - from.y);
    out.yaw = NormalizeAngle(from.yaw + u * NormalizeAngle(to.yaw - from.yaw));
    return out;
  }

  Pose2D blendAndClampCorrection(const Pose2D &pred, const Pose2D &opt,
                                 double gain_scale = 1.0,
                                 double forward_scale = 1.0,
                                 double lateral_scale = 1.0,
                                 double yaw_scale = 1.0) const {
    Pose2D out = pred;
    double dx = opt.x - pred.x;
    double dy = opt.y - pred.y;
    double dyaw = NormalizeAngle(opt.yaw - pred.yaw);

    const double c = std::cos(pred.yaw);
    const double s = std::sin(pred.yaw);
    double d_forward = c * dx + s * dy;
    double d_lateral = -s * dx + c * dy;
    d_forward *= std::max(0.0, forward_scale);
    d_lateral *= std::max(0.0, lateral_scale);
    dyaw *= std::max(0.0, yaw_scale);

    dx = c * d_forward - s * d_lateral;
    dy = s * d_forward + c * d_lateral;

    const double corr_trans = std::hypot(dx, dy);
    if (corr_trans > max_total_correction_trans_m_ && corr_trans > 1e-12) {
      const double scale = max_total_correction_trans_m_ / corr_trans;
      dx *= scale;
      dy *= scale;
    }
    dyaw = std::max(-max_total_correction_yaw_rad_,
                    std::min(max_total_correction_yaw_rad_, dyaw));

    const double gain =
        std::max(0.0, std::min(1.0, correction_gain_ * gain_scale));
    out.x = pred.x + gain * dx;
    out.y = pred.y + gain * dy;
    out.yaw = NormalizeAngle(pred.yaw + gain * dyaw);
    return out;
  }

  std::string poseDebugString_(const Pose2D &p) const {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss.precision(3);
    oss << "(" << p.x << "," << p.y << "," << p.yaw * 180.0 / kPi << "deg)";
    return oss.str();
  }

  std::string formatSectorCounts_(const std::array<int, 4> &counts) const {
    std::ostringstream oss;
    oss << "F=" << counts[0] << ",L=" << counts[1]
        << ",B=" << counts[2] << ",R=" << counts[3];
    return oss.str();
  }

  std::string formatAngleBins_(const std::array<int, 8> &bins) const {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < bins.size(); ++i) {
      if (i > 0)
        oss << ",";
      oss << bins[i];
    }
    oss << "]";
    return oss.str();
  }

  PointCloudT::Ptr buildScanCloudInBase_(
      const std::vector<Eigen::Vector2f> &scan_xy_base) const {
    PointCloudT::Ptr cloud(new PointCloudT());
    cloud->points.reserve(scan_xy_base.size());
    for (const auto &pb : scan_xy_base) {
      PointT pt;
      pt.x = pb.x();
      pt.y = pb.y();
      pt.z = 0.0f;
      pt.intensity = 1.0f;
      cloud->points.push_back(pt);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  std::vector<Eigen::Vector2f> cloudToScanXYBase_(
      const PointCloudT::ConstPtr &cloud) const {
    std::vector<Eigen::Vector2f> out;
    if (!cloud)
      return out;
    out.reserve(cloud->size());
    for (const auto &pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
        continue;
      out.emplace_back(pt.x, pt.y);
    }
    return out;
  }

  std::array<int, 4> computeSectorCounts_(
      const std::vector<Eigen::Vector2f> &scan_xy_base) const {
    std::array<int, 4> counts{{0, 0, 0, 0}};
    for (const auto &pb : scan_xy_base) {
      const double bearing = std::atan2(pb.y(), pb.x());
      ++counts[static_cast<std::size_t>(SectorIndexFromAngle(bearing))];
    }
    return counts;
  }

  PointCloudT::Ptr buildScanCloudInMap_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                                        const Pose2D &pose) const {
    PointCloudT::Ptr cloud(new PointCloudT());
    cloud->points.reserve(scan_xy_base.size());
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    for (const auto &pb : scan_xy_base) {
      PointT pt;
      pt.x = static_cast<float>(pose.x + c * pb.x() - s * pb.y());
      pt.y = static_cast<float>(pose.y + s * pb.x() + c * pb.y());
      pt.z = 0.0f;
      pt.intensity = 1.0f;
      cloud->points.push_back(pt);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  void publishDebugScanCloud_(const ros::Publisher &pub, const ros::Time &stamp,
                              const std::vector<Eigen::Vector2f> &scan_xy_base,
                              const Pose2D &pose) const {
    if (!debug_publish_scan_clouds_ || !pub)
      return;
    PointCloudT::Ptr cloud = buildScanCloudInMap_(scan_xy_base, pose);
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_id_;
    pub.publish(msg);
  }

  void resetDrTracking_() {
    dr_predict_distance_accum_m_ = 0.0;
    dr_predict_yaw_accum_rad_ = 0.0;
    dr_predict_frames_ = 0;
    has_dr_last_pose_ = false;
  }

  void updateDrTracking_(const Pose2D &pub_pose) {
    if (!has_dr_last_pose_) {
      dr_last_pose_ = pub_pose;
      has_dr_last_pose_ = true;
      dr_predict_frames_ = 1;
      return;
    }
    dr_predict_distance_accum_m_ +=
        std::hypot(pub_pose.x - dr_last_pose_.x, pub_pose.y - dr_last_pose_.y);
    dr_predict_yaw_accum_rad_ +=
        std::fabs(NormalizeAngle(pub_pose.yaw - dr_last_pose_.yaw));
    dr_last_pose_ = pub_pose;
    ++dr_predict_frames_;
  }

  void setLocState_(LocState s, const std::string &why) {
    if (s == loc_state_)
      return;
    ROS_INFO_STREAM("[CSM][state] " << LocStateName(loc_state_) << " -> "
                                    << LocStateName(s) << " why=" << why);
    loc_state_ = s;
  }

  void noteSeed_(const char *source) {
    reject_streak_ = 0;
    resetDrTracking_();
    setLocState_(LocState::RELOCALIZING, source ? source : "seed");
  }

  void publishOdom(const ros::Time &stamp, const Eigen::Matrix4f &T_map_base) {
    nav_msgs::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = map_frame_id_;
    odom.child_frame_id = base_frame_id_;

    const Eigen::Matrix3f R = T_map_base.block<3, 3>(0, 0);
    const Eigen::Vector3f t = T_map_base.block<3, 1>(0, 3);
    Eigen::Quaternionf q(R);
    q.normalize();

    odom.pose.pose.position.x = t.x();
    odom.pose.pose.position.y = t.y();
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    if (has_last_odom_) {
      odom.twist.twist.linear = last_odom_msg_.twist.twist.linear;
    }

    if (has_last_imu_) {
      odom.twist.twist.angular.z = last_imu_gyro_z_;
    } else if (has_last_odom_) {
      odom.twist.twist.angular = last_odom_msg_.twist.twist.angular;
    }

    pose_pub_.publish(odom);

    if (!publish_tf_)
      return;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = map_frame_id_;
    tf_msg.child_frame_id = base_frame_id_;
    tf_msg.transform.translation.x = t.x();
    tf_msg.transform.translation.y = t.y();
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_.sendTransform(tf_msg);
  }

  void resetPredictorWithPose_(const Eigen::Matrix4f &T_map_base) {
    if (has_last_odom_) {
      predictor_.resetWithCorrection(T_map_base, last_odom_msg_);
      predictor_anchor_pending_ = false;
      return;
    }
    pending_predictor_anchor_pose_ = T_map_base;
    predictor_anchor_pending_ = true;
    ROS_WARN_THROTTLE(2.0,
                      "[CSM] odom not ready, defer predictor anchor update");
  }

  void tryApplyPendingPredictorAnchor_() {
    if (!predictor_anchor_pending_ || !has_last_odom_)
      return;
    predictor_.resetWithCorrection(pending_predictor_anchor_pose_,
                                   last_odom_msg_);
    predictor_anchor_pending_ = false;
    ROS_INFO_STREAM("[CSM] applied deferred predictor anchor");
  }

  bool runManualRelocIfRequested_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                                  Eigen::Matrix4f *T_pred) {
    if (!T_pred || !manual_reloc_pending_)
      return false;

    manual_reloc_pending_ = false;
    Pose2D manual_pose;
    manual_pose.x = manual_reloc_center_.x;
    manual_pose.y = manual_reloc_center_.y;
    manual_pose.yaw = manual_reloc_center_.yaw;
    *T_pred = Pose2DToMat(manual_pose);

    if (!manual_reloc_enable_) {
      resetPredictorWithPose_(*T_pred);
      last_output_ = *T_pred;
      has_last_output_ = true;
      last_published_output_ = *T_pred;
      has_last_published_output_ = true;
      resetDrTracking_();
      return true;
    }

    if (startup_local_reloc_ && startup_local_reloc_->isReady()) {
      LocalRelocOptions opt = startup_reloc_opt_;
      opt.xy_range_m = manual_reloc_xy_range_m_;
      opt.yaw_range_rad = manual_reloc_yaw_range_rad_;
      opt.score_margin = manual_reloc_score_margin_;

      const LocalRelocResult r =
          startup_local_reloc_->match(scan_xy_base, manual_reloc_center_, opt);
      if (r.ok) {
        *T_pred = r.T_seed;
        ROS_INFO_STREAM("[CSM] manual local_reloc success: x="
                        << r.best_pose.x << " y=" << r.best_pose.y << " yaw="
                        << r.best_pose.yaw << " score=" << r.best_score
                        << " second=" << r.second_score
                        << " vf=" << r.valid_fraction);
      } else {
        ROS_WARN_STREAM("[CSM] manual local_reloc failed, fallback to dragged pose. "
                        << "best=" << r.best_score
                        << " second=" << r.second_score
                        << " vf=" << r.valid_fraction);
      }
    } else {
      ROS_WARN_STREAM("[CSM] manual local_reloc unavailable, fallback to dragged pose");
    }

    resetPredictorWithPose_(*T_pred);
    last_output_ = *T_pred;
    has_last_output_ = true;
    last_published_output_ = *T_pred;
    has_last_published_output_ = true;
    resetDrTracking_();
    return true;
  }

  bool runStartupRelocOnce_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                            Eigen::Matrix4f *T_pred) {
    if (!T_pred || !startup_reloc_enable_ || startup_reloc_attempted_ ||
        !startup_local_reloc_ || !startup_local_reloc_->isReady()) {
      return false;
    }

    startup_reloc_attempted_ = true;

    const Pose2D center_pose = MatToPose2D(*T_pred);
    RelocPose2D center;
    center.x = center_pose.x;
    center.y = center_pose.y;
    center.yaw = center_pose.yaw;

    const LocalRelocResult r =
        startup_local_reloc_->match(scan_xy_base, center, startup_reloc_opt_);
    if (!r.ok) {
      ROS_WARN_STREAM("[CSM] startup local_reloc failed: best=" << r.best_score
                      << " second=" << r.second_score
                      << " vf=" << r.valid_fraction);
      return false;
    }

    *T_pred = r.T_seed;
    resetPredictorWithPose_(*T_pred);
    last_output_ = *T_pred;
    has_last_output_ = true;
    last_published_output_ = *T_pred;
    has_last_published_output_ = true;
    noteSeed_("startup_seed");
    ROS_INFO_STREAM("[CSM] startup local_reloc success: x="
                    << r.best_pose.x << " y=" << r.best_pose.y
                    << " yaw=" << r.best_pose.yaw << " score=" << r.best_score
                    << " second=" << r.second_score
                    << " vf=" << r.valid_fraction);
    return true;
  }

  bool runLostRelocIfNeeded_(const std::vector<Eigen::Vector2f> &scan_xy_base,
                             const ros::Time &stamp, Eigen::Matrix4f *T_pred) {
    if (!T_pred || !lost_reloc_enable_ || loc_state_ != LocState::LOST ||
        !startup_local_reloc_ || !startup_local_reloc_->isReady()) {
      return false;
    }
    if (!last_lost_reloc_attempt_.isZero() && lost_reloc_min_interval_sec_ > 0.0 &&
        (stamp - last_lost_reloc_attempt_) <
            ros::Duration(lost_reloc_min_interval_sec_)) {
      return false;
    }
    last_lost_reloc_attempt_ = stamp;

    const Pose2D center_pose = MatToPose2D(*T_pred);
    RelocPose2D center;
    center.x = center_pose.x;
    center.y = center_pose.y;
    center.yaw = center_pose.yaw;

    LocalRelocOptions opt = startup_reloc_opt_;
    opt.xy_range_m = lost_reloc_xy_range_m_;
    opt.yaw_range_rad = lost_reloc_yaw_range_rad_;
    opt.min_score = lost_reloc_min_score_;
    opt.score_margin = lost_reloc_score_margin_;
    opt.min_valid_fraction = lost_reloc_min_valid_fraction_;

    const LocalRelocResult r = startup_local_reloc_->match(scan_xy_base, center, opt);
    if (!r.ok) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[CSM] lost local_reloc failed: best=" << r.best_score
                                                      << " second=" << r.second_score
                                                      << " vf=" << r.valid_fraction
                                                      << " center=(" << center.x
                                                      << "," << center.y << ","
                                                      << center.yaw * 180.0 / kPi
                                                      << "deg)");
      return false;
    }

    *T_pred = r.T_seed;
    noteSeed_("lost_seed");
    ROS_INFO_STREAM("[CSM] lost local_reloc success: x="
                    << r.best_pose.x << " y=" << r.best_pose.y
                    << " yaw=" << r.best_pose.yaw << " score=" << r.best_score
                    << " second=" << r.second_score
                    << " vf=" << r.valid_fraction);
    return true;
  }

  void scanCallback(const sensor_msgs::LaserScanConstPtr &scan_msg) {
    if (!map_.isReady()) {
      ROS_WARN_THROTTLE(1.0, "[CSM] map not ready");
      return;
    }

    ScanBuildStats scan_stats;
    const std::vector<Eigen::Vector2f> scan_xy_base =
        buildScanPointsBase(scan_msg, &scan_stats);
    if (scan_xy_base.empty()) {
      ROS_WARN_STREAM_THROTTLE(debug_log_throttle_sec_,
                               "[CSM] scan points empty after preprocessing raw="
                                   << scan_stats.raw_ranges
                                   << " finite=" << scan_stats.finite_ranges
                                   << " in_range=" << scan_stats.range_kept
                                   << " dup=" << scan_stats.duplicate_points
                                   << " tf_ok=" << (scan_stats.tf_ok ? 1 : 0));
      return;
    }

    const ros::Time stamp = scan_msg->header.stamp.isZero()
                                ? ros::Time::now()
                                : scan_msg->header.stamp;
    tryApplyPendingPredictorAnchor_();

    Eigen::Matrix4f T_pred = Eigen::Matrix4f::Identity();
    const bool used_manual_seed =
        runManualRelocIfRequested_(scan_xy_base, &T_pred);

    bool used_startup_seed = false;
    bool used_lost_seed = false;
    std::string seed_mode = used_manual_seed ? "manual_seed" : "";
    if (!used_manual_seed) {
      if (predictor_enable_) {
        if (!predictor_.predict(T_pred)) {
          if (has_last_output_) {
            T_pred = last_output_;
            seed_mode = "last_output_fallback";
          } else {
            T_pred = Eigen::Matrix4f::Identity();
            seed_mode = "identity_fallback";
          }
          ROS_WARN_THROTTLE(2.0,
                            "[CSM] predictor not ready, fallback to last/identity");
        } else {
          seed_mode = "motion_model";
        }
      } else {
        if (has_last_output_) {
          T_pred = last_output_;
          seed_mode = "last_output_only";
        } else {
          T_pred = Eigen::Matrix4f::Identity();
          seed_mode = "identity_no_predictor";
        }
      }
      used_startup_seed = runStartupRelocOnce_(scan_xy_base, &T_pred);
      if (used_startup_seed)
        seed_mode = "startup_seed";
      const Eigen::Matrix4f T_seed_fallback = T_pred;
      if (!used_startup_seed) {
        used_lost_seed = runLostRelocIfNeeded_(scan_xy_base, stamp, &T_pred);
        if (used_lost_seed)
          seed_mode = "lost_seed";
      }
      if (used_lost_seed)
        raw_pred_before_lost_seed_ = T_seed_fallback;
      else
        raw_pred_before_lost_seed_ = T_pred;
    } else {
      raw_pred_before_lost_seed_ = T_pred;
    }

    const Pose2D pred_pose = MatToPose2D(T_pred);
    const Pose2D fallback_pred_pose = MatToPose2D(raw_pred_before_lost_seed_);

    std::vector<Eigen::Vector2f> match_scan_xy_base = scan_xy_base;
    DynamicFilterStats dyn_stats{};
    bool dyn_filter_used = false;
    bool dyn_filter_effective = false;
    bool dyn_filter_used_raw_fallback = false;
    if (dyn_cfg_.enable && dyn_filter_.isReady()) {
      dyn_filter_used = true;
      const PointCloudT::Ptr scan_cloud_base = buildScanCloudInBase_(scan_xy_base);
      const Eigen::Vector2f sensor_origin_base(
          static_cast<float>(scan_stats.tf_tx),
          static_cast<float>(scan_stats.tf_ty));
      PointCloudT::Ptr cloud_dyn =
          dyn_filter_.filter(scan_cloud_base, T_pred, sensor_origin_base, &dyn_stats);
      if (cloud_dyn && !cloud_dyn->empty()) {
        match_scan_xy_base = cloudToScanXYBase_(cloud_dyn);
      } else {
        dyn_filter_used_raw_fallback = true;
        match_scan_xy_base = scan_xy_base;
      }
      dyn_filter_effective =
          (!dyn_stats.insufficient_static && dyn_stats.in > 0 &&
           dyn_stats.kept > 0 && dyn_stats.kept < dyn_stats.in);
      dyn_filter_used_raw_fallback =
          dyn_filter_used_raw_fallback ||
          dyn_stats.insufficient_static || dyn_stats.kept >= dyn_stats.in ||
          dyn_stats.front_recovered;
    }

    const CsmMatcherResult mres =
        matcher_.optimize(match_scan_xy_base, pred_pose, map_, match_opt_);

    const double corr_dx = mres.pose.x - pred_pose.x;
    const double corr_dy = mres.pose.y - pred_pose.y;
    const double corr_trans = std::hypot(corr_dx, corr_dy);
    const double corr_yaw =
        std::fabs(NormalizeAngle(mres.pose.yaw - pred_pose.yaw));
    const double mean_cost =
        (mres.used_points > 0)
            ? (mres.final_cost / static_cast<double>(std::max(1, mres.used_points)))
            : std::numeric_limits<double>::infinity();
    const WeakDirDecision weak_dir_decision = evaluateWeakDirDecision_(mres);
    const Pose2D weak_dir_fused_pose =
        weak_dir_decision.active
            ? fusePoseByWeakDir_(pred_pose, mres.pose, weak_dir_decision)
            : mres.pose;

    const bool accept_strong =
        mres.solved && mres.converged &&
        mres.used_points >= match_opt_.min_valid_points &&
        mres.valid_fraction >= match_opt_.min_valid_fraction;

    const bool accept_small =
        mres.solved && (!mres.converged) && mres.improved &&
        mres.used_points >= accept_small_min_used_points_ &&
        mres.valid_fraction >= accept_small_min_valid_fraction_ &&
        corr_trans <= accept_small_max_corr_trans_m_ &&
        corr_yaw <= accept_small_max_corr_yaw_rad_;

    const bool accept_hold =
        accept_hold_enable_ && mres.solved && mres.improved &&
        mres.used_points >= accept_hold_min_used_points_ &&
        mres.valid_fraction >= accept_hold_min_valid_fraction_ &&
        corr_trans <= accept_hold_max_corr_trans_m_ &&
        corr_yaw <= accept_hold_max_corr_yaw_rad_;

    const bool accept_stable =
        mres.solved && (!mres.converged) && (!mres.improved) &&
        mres.used_points >= accept_small_min_used_points_ &&
        mres.valid_fraction >= accept_small_min_valid_fraction_ &&
        corr_trans <= 1e-3 && corr_yaw <= Deg2Rad(0.05);

    const auto coverage_ratio = [](int matched, int total) -> double {
      return (total > 0) ? (static_cast<double>(matched) / static_cast<double>(total))
                         : 0.0;
    };
    const auto balance_ratio = [](double a, double b) -> double {
      if (a <= 1e-9 && b <= 1e-9)
        return 1.0;
      if (a <= 1e-9 || b <= 1e-9)
        return 0.0;
      return std::min(a, b) / std::max(a, b);
    };

    const std::array<int, 4> match_input_sector_counts =
        computeSectorCounts_(match_scan_xy_base);
    const int scan_front = match_input_sector_counts[0];
    const int scan_left = match_input_sector_counts[1];
    const int scan_back = match_input_sector_counts[2];
    const int scan_right = match_input_sector_counts[3];

    const int match_front = mres.matched_sector_counts[0];
    const int match_left = mres.matched_sector_counts[1];
    const int match_back = mres.matched_sector_counts[2];
    const int match_right = mres.matched_sector_counts[3];

    const double front_match_ratio = coverage_ratio(match_front, scan_front);
    const double left_match_ratio = coverage_ratio(match_left, scan_left);
    const double back_match_ratio = coverage_ratio(match_back, scan_back);
    const double right_match_ratio = coverage_ratio(match_right, scan_right);

    const bool sector_front_check = scan_front >= sector_degen_min_scan_points_;
    const bool sector_lr_check =
        scan_left >= sector_degen_min_scan_points_ &&
        scan_right >= sector_degen_min_scan_points_;
    const bool sector_fb_check =
        scan_front >= sector_degen_min_scan_points_ &&
        scan_back >= sector_degen_min_scan_points_;

    const double lr_balance_ratio =
        sector_lr_check ? balance_ratio(left_match_ratio, right_match_ratio) : 1.0;
    const double fb_balance_ratio =
        sector_fb_check ? balance_ratio(front_match_ratio, back_match_ratio) : 1.0;

    const bool sector_front_bad =
        sector_degen_enable_ && sector_front_check &&
        front_match_ratio < sector_degen_front_min_match_ratio_;
    const bool sector_lr_bad =
        sector_degen_enable_ && sector_lr_check &&
        lr_balance_ratio < sector_degen_lr_balance_min_;
    const bool sector_fb_bad =
        sector_degen_enable_ && sector_fb_check &&
        fb_balance_ratio < sector_degen_fb_balance_min_;
    const bool front_like_degen = sector_front_bad || sector_fb_bad;
    const bool combo_degen = front_like_degen && sector_lr_bad;
    const bool sector_degen = front_like_degen || sector_lr_bad;
    const bool sector_corr_gate_ok =
        (!sector_degen) ||
        (corr_trans <= sector_degen_max_corr_trans_m_ &&
         corr_yaw <= sector_degen_max_corr_yaw_rad_);
    const bool combo_predictor_only =
        combo_degen_predictor_only_ && combo_degen;

    const bool accept_strong_effective =
        (!combo_predictor_only) && accept_strong && sector_corr_gate_ok;
    const bool accept_small_effective =
        (!combo_predictor_only) && accept_small && sector_corr_gate_ok;
    const double reject_fraction =
        (match_scan_xy_base.empty())
            ? 1.0
            : (static_cast<double>(mres.reject_no_correspondence) /
               static_cast<double>(match_scan_xy_base.size()));
    const bool accept_quality =
        accept_quality_enable_ && mres.solved && (!mres.converged) &&
        mres.improved && (!sector_degen) &&
        mres.used_points >= accept_quality_min_used_points_ &&
        mres.valid_fraction >= accept_quality_min_valid_fraction_ &&
        reject_fraction <= accept_quality_max_reject_fraction_ &&
        mean_cost <= accept_quality_max_mean_cost_ &&
        corr_trans <= accept_quality_max_corr_trans_m_ &&
        corr_yaw <= accept_quality_max_corr_yaw_rad_;
    const bool accept_hold_effective =
        (!combo_predictor_only) && accept_hold && sector_corr_gate_ok;
    const bool accept_stable_effective =
        (!combo_predictor_only) && accept_stable;
    double degen_gain_scale = 1.0;
    double degen_forward_scale = 1.0;
    double degen_lateral_scale = 1.0;
    double degen_yaw_scale = 1.0;
    std::string degen_mode = "none";
    if (sector_degen) {
      degen_gain_scale *= sector_degen_gain_scale_;
    }
    if (front_like_degen) {
      degen_gain_scale *= front_degen_gain_scale_;
      degen_forward_scale *= front_degen_forward_scale_;
      degen_lateral_scale *= front_degen_lateral_scale_;
      degen_yaw_scale *= front_degen_yaw_scale_;
      degen_mode = "front";
    }
    if (sector_lr_bad) {
      degen_gain_scale *= lr_degen_gain_scale_;
      degen_forward_scale *= lr_degen_forward_scale_;
      degen_lateral_scale *= lr_degen_lateral_scale_;
      degen_yaw_scale *= lr_degen_yaw_scale_;
      degen_mode = (degen_mode == "none") ? "lr" : "front+lr";
    }
    if (!sector_degen) {
      degen_gain_scale = 1.0;
      degen_forward_scale = 1.0;
      degen_lateral_scale = 1.0;
      degen_yaw_scale = 1.0;
    }

    Pose2D internal_out_pose = pred_pose;
    bool accepted_match = false;
    bool weak_dir_applied = false;
    double gain_scale = 1.0;
    if (combo_predictor_only) {
      internal_out_pose = pred_pose;
      accepted_match = false;
      gain_scale = 0.0;
    } else if (accept_strong_effective) {
      internal_out_pose = blendAndClampCorrection(pred_pose, weak_dir_fused_pose,
                                                  degen_gain_scale,
                                                  degen_forward_scale,
                                                  degen_lateral_scale,
                                                  degen_yaw_scale);
      accepted_match = true;
      weak_dir_applied = weak_dir_decision.active;
      gain_scale = degen_gain_scale;
    } else if (accept_quality) {
      internal_out_pose = blendAndClampCorrection(pred_pose, weak_dir_fused_pose,
                                                  accept_quality_gain_scale_);
      accepted_match = true;
      weak_dir_applied = weak_dir_decision.active;
      gain_scale = accept_quality_gain_scale_;
    } else if (accept_small_effective) {
      internal_out_pose = blendAndClampCorrection(pred_pose, weak_dir_fused_pose,
                                                  small_gain_scale_ * degen_gain_scale,
                                                  degen_forward_scale,
                                                  degen_lateral_scale,
                                                  degen_yaw_scale);
      accepted_match = true;
      weak_dir_applied = weak_dir_decision.active;
      gain_scale = small_gain_scale_ * degen_gain_scale;
    } else if (accept_hold_effective) {
      internal_out_pose = blendAndClampCorrection(pred_pose, weak_dir_fused_pose,
                                                  hold_gain_scale_ * degen_gain_scale,
                                                  degen_forward_scale,
                                                  degen_lateral_scale,
                                                  degen_yaw_scale);
      accepted_match = true;
      weak_dir_applied = weak_dir_decision.active;
      gain_scale = hold_gain_scale_ * degen_gain_scale;
    } else if (accept_stable_effective) {
      internal_out_pose = pred_pose;
      accepted_match = true;
      gain_scale = 0.0;
    }

    if (accepted_match) {
      if (used_lost_seed && lost_reloc_publish_smooth_enable_ &&
          lost_reloc_publish_smooth_frames_ > 0) {
        lost_reloc_publish_frames_left_ = lost_reloc_publish_smooth_frames_;
      }
    } else if (used_lost_seed) {
      internal_out_pose = fallback_pred_pose;
      lost_reloc_publish_frames_left_ = 0;
    }

    Pose2D pub_pose = internal_out_pose;
    if (lost_reloc_publish_frames_left_ > 0 && has_last_published_output_) {
      const Pose2D last_pub_pose = MatToPose2D(last_published_output_);
      pub_pose = interpolatePose_(
          last_pub_pose, internal_out_pose,
          1.0 / static_cast<double>(std::max(1, lost_reloc_publish_frames_left_)));
      --lost_reloc_publish_frames_left_;
    }

    const Eigen::Matrix4f T_internal_out = Pose2DToMat(internal_out_pose);
    const Eigen::Matrix4f T_pub = Pose2DToMat(pub_pose);
    publishDebugScanCloud_(pred_scan_cloud_pub_, stamp, scan_xy_base, pred_pose);
    publishDebugScanCloud_(out_scan_cloud_pub_, stamp, scan_xy_base, pub_pose);
    publishOdom(stamp, T_pub);

    const bool seed_retry_frame =
        used_lost_seed || used_startup_seed || used_manual_seed;
    bool dr_budget_exceeded = false;

    if (accepted_match) {
      resetPredictorWithPose_(T_internal_out);
      reject_streak_ = 0;
      if (used_lost_seed) {
        resetDrTracking_();
        dr_recover_accept_streak_ = 0;
        if (loc_state_ != LocState::TRACKING)
          setLocState_(LocState::TRACKING, "lost_reloc_accepted");
      } else if (loc_state_ == LocState::DR_TRACKING) {
        dr_recover_accept_streak_ =
            std::min(dr_recover_accept_streak_ + 1, dr_recover_accept_frames_);
        if (dr_recover_accept_streak_ >= dr_recover_accept_frames_) {
          resetDrTracking_();
          dr_recover_accept_streak_ = 0;
          setLocState_(LocState::TRACKING, "dr_recovered");
        }
      } else {
        resetDrTracking_();
        dr_recover_accept_streak_ = 0;
        if (loc_state_ != LocState::TRACKING)
          setLocState_(LocState::TRACKING, "accepted_match");
      }
    } else {
      if (predictor_enable_)
        predictor_.commitPredictionAsCorrection(used_lost_seed
                                                    ? raw_pred_before_lost_seed_
                                                    : T_pred);
      ++reject_streak_;
      dr_recover_accept_streak_ = 0;
      if (seed_retry_frame) {
        resetDrTracking_();
        lost_reloc_publish_frames_left_ = 0;
        setLocState_(LocState::LOST, "seed_reject");
      } else if (!has_last_output_ || !dr_mode_enable_) {
        setLocState_(LocState::LOST,
                     has_last_output_ ? "reject_no_dr" : "no_pose_reference");
      } else if (loc_state_ == LocState::LOST) {
        lost_reloc_publish_frames_left_ = 0;
      } else {
        if (loc_state_ != LocState::DR_TRACKING)
          setLocState_(LocState::DR_TRACKING, "predictor_only");
        updateDrTracking_(pub_pose);
        dr_budget_exceeded =
            (dr_predict_distance_accum_m_ >= dr_max_predict_distance_m_) ||
            (dr_predict_yaw_accum_rad_ >= dr_max_predict_yaw_rad_);
        if (dr_budget_exceeded) {
          setLocState_(LocState::LOST, "dr_budget_exceeded");
          lost_reloc_publish_frames_left_ = 0;
        }
      }
    }

    last_output_ = T_internal_out;
    has_last_output_ = true;
    last_published_output_ = T_pub;
    has_last_published_output_ = true;

    const char *out_mode = "pred";
    if (used_manual_seed) {
      out_mode = "pred_manual_seed";
    } else if (used_startup_seed) {
      out_mode = "pred_startup_seed";
    } else if (used_lost_seed) {
      out_mode = accepted_match ? "opt_lost_seed" : "pred_lost_seed_hold";
    } else if (!accepted_match && loc_state_ == LocState::DR_TRACKING) {
      if (combo_predictor_only) {
        out_mode = "pred_dr_combo_degen";
      } else if (degen_mode == "front") {
        out_mode = "pred_dr_front_degen";
      } else if (degen_mode == "lr") {
        out_mode = "pred_dr_lr_degen";
      } else if (sector_degen) {
        out_mode = "pred_dr_degen";
      } else {
        out_mode = "pred_dr";
      }
    } else if (!accepted_match && loc_state_ == LocState::LOST) {
      out_mode = combo_predictor_only ? "pred_fail_combo_degen" : "pred_fail";
    } else if (combo_predictor_only) {
      out_mode = "pred_combo_degen";
    } else if (accept_strong_effective) {
      if (!sector_degen) {
        out_mode = "opt_conv";
      } else if (degen_mode == "front") {
        out_mode = "opt_conv_front_degen";
      } else if (degen_mode == "lr") {
        out_mode = "opt_conv_lr_degen";
      } else {
        out_mode = "opt_conv_combo_degen";
      }
    } else if (accept_quality) {
      out_mode = "opt_quality";
    } else if (accept_small_effective) {
      if (!sector_degen) {
        out_mode = "opt_small";
      } else if (degen_mode == "front") {
        out_mode = "opt_small_front_degen";
      } else if (degen_mode == "lr") {
        out_mode = "opt_small_lr_degen";
      } else {
        out_mode = "opt_small_combo_degen";
      }
    } else if (accept_hold_effective) {
      out_mode = "opt_hold";
    } else if (accept_stable_effective) {
      out_mode = "opt_stable";
    }

    std::ostringstream log;
    log << "[CSM] state=" << LocStateName(loc_state_) << " out(" << out_mode
        << ")"
        << " x=" << pub_pose.x << " y=" << pub_pose.y << " yaw=" << pub_pose.yaw
        << " used_points=" << mres.used_points << "/" << match_scan_xy_base.size()
        << " raw_points=" << scan_xy_base.size()
        << " valid_fraction=" << mres.valid_fraction
        << " reject_no_corr=" << mres.reject_no_correspondence
        << " iter=" << mres.iterations << " final_cost=" << mres.final_cost
        << " init_cost=" << mres.initial_cost
        << " mean_cost=" << mean_cost
        << " mean_abs_residual_m=" << mres.mean_abs_residual_m
        << " corr_trans=" << corr_trans
        << " corr_yaw_deg=" << corr_yaw * 180.0 / kPi
        << " solved=" << (mres.solved ? 1 : 0)
        << " improved=" << (mres.improved ? 1 : 0)
        << " converged=" << (mres.converged ? 1 : 0)
        << " accept_strong=" << (accept_strong_effective ? 1 : 0)
        << " accept_small=" << (accept_small_effective ? 1 : 0)
        << " accept_quality=" << (accept_quality ? 1 : 0)
        << " accept_hold=" << (accept_hold_effective ? 1 : 0)
        << " accept_stable=" << (accept_stable_effective ? 1 : 0)
        << " combo_predictor=" << (combo_predictor_only ? 1 : 0)
        << " weak_dir=" << (weak_dir_decision.active ? 1 : 0)
        << " weak_applied=" << (weak_dir_applied ? 1 : 0)
        << " dr(dist=" << dr_predict_distance_accum_m_
        << ",yaw_deg=" << dr_predict_yaw_accum_rad_ * 180.0 / kPi
        << ",frames=" << dr_predict_frames_
        << ",recover=" << dr_recover_accept_streak_ << "/"
        << dr_recover_accept_frames_
        << ",fail=" << (dr_budget_exceeded ? 1 : 0) << ")"
        << " gain=" << gain_scale << " reason=" << mres.reason;

    if (debug_verbose_) {
      log << " seed_mode=" << seed_mode
          << " scan(raw=" << scan_stats.raw_ranges
          << " finite=" << scan_stats.finite_ranges
          << " in_range=" << scan_stats.range_kept
          << " dup=" << scan_stats.duplicate_points
          << " out=" << scan_stats.output_points << ")"
          << " dyn(used=" << (dyn_filter_used ? 1 : 0)
          << ",effective=" << (dyn_filter_effective ? 1 : 0)
          << ",use_raw=" << (dyn_filter_used_raw_fallback ? 1 : 0)
          << ",keep=" << dyn_stats.kept << "/" << dyn_stats.in
          << ",ratio=" << dyn_stats.keep_ratio
          << ",ray=" << dyn_stats.ray_rejected << "/" << dyn_stats.ray_tested
          << ",front=" << dyn_stats.front_kept << "/" << dyn_stats.front_in
          << ",frec=" << (dyn_stats.front_recovered ? 1 : 0) << ")"
          << " scan_sector(" << formatSectorCounts_(scan_stats.output_sector_counts)
          << ")"
          << " scan_bins8=" << formatAngleBins_(scan_stats.output_angle_bins)
          << " match_sector(" << formatSectorCounts_(mres.matched_sector_counts)
          << ")"
          << " match_bins8=" << formatAngleBins_(mres.matched_angle_bins)
          << " sector_cov(F=" << front_match_ratio
          << ",L=" << left_match_ratio
          << ",B=" << back_match_ratio
          << ",R=" << right_match_ratio << ")"
          << " sector_balance(lr=" << lr_balance_ratio
          << ",fb=" << fb_balance_ratio << ")"
          << " reject_fraction=" << reject_fraction
          << " weak_dir(valid=" << (weak_dir_decision.valid ? 1 : 0)
          << ",active=" << (weak_dir_decision.active ? 1 : 0)
          << ",yaw_weak=" << (weak_dir_decision.yaw_weak ? 1 : 0)
          << ",cond=" << weak_dir_decision.cond
          << ",lam=(" << weak_dir_decision.lambda_min << ","
          << weak_dir_decision.lambda_max << ")"
          << ",yaw_curv=" << weak_dir_decision.yaw_curv
          << ",alpha=(" << weak_dir_decision.alpha_w << ","
          << weak_dir_decision.alpha_s << ","
          << weak_dir_decision.alpha_yaw << ")"
          << ",dir=(" << weak_dir_decision.weak_dir.x() << ","
          << weak_dir_decision.weak_dir.y() << "))"
          << " degen_mode=" << degen_mode
          << " degen_scale(g=" << degen_gain_scale
          << ",fwd=" << degen_forward_scale
          << ",lat=" << degen_lateral_scale
          << ",yaw=" << degen_yaw_scale << ")"
          << " sector_check(front=" << (sector_front_check ? 1 : 0)
          << ",lr=" << (sector_lr_check ? 1 : 0)
          << ",fb=" << (sector_fb_check ? 1 : 0) << ")"
          << " sector_bad(front=" << (sector_front_bad ? 1 : 0)
          << ",lr=" << (sector_lr_bad ? 1 : 0)
          << ",fb=" << (sector_fb_bad ? 1 : 0)
          << ",gate=" << (sector_corr_gate_ok ? 1 : 0)
          << ",degen=" << (sector_degen ? 1 : 0) << ")"
          << " base_from_scan=(" << scan_stats.tf_tx << "," << scan_stats.tf_ty
          << "," << scan_stats.tf_yaw * 180.0 / kPi << "deg)"
          << " fallback_pred=" << poseDebugString_(fallback_pred_pose)
          << " pred=" << poseDebugString_(pred_pose)
          << " opt=" << poseDebugString_(mres.pose)
          << " weak_opt=" << poseDebugString_(weak_dir_fused_pose)
          << " internal_out=" << poseDebugString_(internal_out_pose)
          << " out_pose=" << poseDebugString_(pub_pose)
          << " reject_streak=" << reject_streak_;
    }

    ROS_INFO_STREAM_THROTTLE(debug_log_throttle_sec_, log.str());
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber initialpose_sub_;
  ros::Publisher pose_pub_;
  ros::Publisher pred_scan_cloud_pub_;
  ros::Publisher out_scan_cloud_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  SimplePredictor predictor_;
  CsmMap map_;
  CsmScanMatcher matcher_;
  CsmMatcherOptions match_opt_;
  DynamicPointFilter dyn_filter_;
  DynamicFilterConfig dyn_cfg_;

  std::unique_ptr<LocalRelocalizer2D> startup_local_reloc_;
  LocalRelocOptions startup_reloc_opt_;

  std::string map_yaml_path_;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string initialpose_topic_ = "/initialpose";
  std::string odom_pub_topic_;
  std::string map_frame_id_ = "map";
  std::string base_frame_id_ = "base_footprint";

  bool publish_tf_ = true;
  bool debug_verbose_ = true;
  double debug_log_throttle_sec_ = 0.5;
  bool debug_publish_scan_clouds_ = true;
  std::string debug_pred_scan_cloud_topic_ = "csm_scan_pred_cloud";
  std::string debug_out_scan_cloud_topic_ = "csm_scan_out_cloud";
  bool predictor_enable_ = true;
  bool use_yaml_frame_id_ = false;
  bool dr_mode_enable_ = true;
  double dr_max_predict_distance_m_ = 3.0;
  double dr_max_predict_yaw_rad_ = Deg2Rad(45.0);
  int dr_recover_accept_frames_ = 3;

  bool startup_reloc_enable_ = true;
  bool startup_reloc_allow_external_map_paths_ = true;
  bool startup_reloc_attempted_ = false;
  std::string startup_reloc_map_yaml_;
  bool lost_reloc_enable_ = true;
  double lost_reloc_xy_range_m_ = 3.0;
  double lost_reloc_yaw_range_rad_ = Deg2Rad(45.0);
  double lost_reloc_min_score_ = 0.20;
  double lost_reloc_score_margin_ = 0.0;
  double lost_reloc_min_valid_fraction_ = 0.25;
  double lost_reloc_min_interval_sec_ = 0.5;
  bool lost_reloc_publish_smooth_enable_ = true;
  int lost_reloc_publish_smooth_frames_ = 4;

  bool manual_reloc_enable_ = true;
  double manual_reloc_xy_min_m_ = 0.30;
  double manual_reloc_xy_sigma_k_ = 3.0;
  double manual_reloc_xy_max_m_ = 5.0;
  double manual_reloc_yaw_min_rad_ = Deg2Rad(6.0);
  double manual_reloc_yaw_sigma_k_ = 3.0;
  double manual_reloc_score_margin_ = 0.0;
  bool manual_reloc_pending_ = false;
  RelocPose2D manual_reloc_center_;
  double manual_reloc_xy_range_m_ = 0.0;
  double manual_reloc_yaw_range_rad_ = 0.0;

  double scan_min_range_ = 0.0;
  double scan_max_range_ = 0.0;
  int max_scan_points_ = 1200;
  double scan_voxel_m_ = 0.02;

  double correction_gain_ = 0.8;
  double small_gain_scale_ = 0.5;
  double hold_gain_scale_ = 0.3;
  double max_total_correction_trans_m_ = 0.30;
  double max_total_correction_yaw_rad_ = Deg2Rad(10.0);

  double accept_small_min_valid_fraction_ = 0.60;
  int accept_small_min_used_points_ = 250;
  double accept_small_max_corr_trans_m_ = 0.03;
  double accept_small_max_corr_yaw_rad_ = Deg2Rad(0.5);

  bool accept_quality_enable_ = true;
  double accept_quality_min_valid_fraction_ = 0.80;
  int accept_quality_min_used_points_ = 450;
  double accept_quality_max_reject_fraction_ = 0.25;
  double accept_quality_max_mean_cost_ = 0.0025;
  double accept_quality_max_corr_trans_m_ = 0.06;
  double accept_quality_max_corr_yaw_rad_ = Deg2Rad(0.80);
  double accept_quality_gain_scale_ = 0.25;

  bool accept_hold_enable_ = false;
  double accept_hold_min_valid_fraction_ = 0.75;
  int accept_hold_min_used_points_ = 350;
  double accept_hold_max_corr_trans_m_ = 0.010;
  double accept_hold_max_corr_yaw_rad_ = Deg2Rad(0.20);

  bool sector_degen_enable_ = true;
  int sector_degen_min_scan_points_ = 60;
  double sector_degen_front_min_match_ratio_ = 0.45;
  double sector_degen_lr_balance_min_ = 0.55;
  double sector_degen_fb_balance_min_ = 0.45;
  double sector_degen_gain_scale_ = 0.35;
  double sector_degen_max_corr_trans_m_ = 0.03;
  double sector_degen_max_corr_yaw_rad_ = Deg2Rad(0.30);
  double front_degen_gain_scale_ = 0.60;
  double front_degen_forward_scale_ = 0.20;
  double front_degen_lateral_scale_ = 1.00;
  double front_degen_yaw_scale_ = 0.50;
  double lr_degen_gain_scale_ = 0.60;
  double lr_degen_forward_scale_ = 1.00;
  double lr_degen_lateral_scale_ = 0.25;
  double lr_degen_yaw_scale_ = 0.35;
  bool combo_degen_predictor_only_ = true;
  bool weak_dir_fusion_enable_ = true;
  int weak_dir_min_used_points_ = 200;
  double weak_dir_cond_on_ = 30.0;
  double weak_dir_cond_bad_ = 120.0;
  double weak_dir_alpha_weak_min_ = 0.05;
  double weak_dir_alpha_strong_ = 1.0;
  double weak_dir_alpha_yaw_ = 0.5;
  double weak_dir_lambda_min_thresh_ = 1e-3;

  nav_msgs::Odometry last_odom_msg_;
  bool has_last_odom_ = false;
  double last_imu_gyro_z_ = 0.0;
  bool has_last_imu_ = false;

  Eigen::Matrix4f last_output_ = Eigen::Matrix4f::Identity();
  bool has_last_output_ = false;
  Eigen::Matrix4f last_published_output_ = Eigen::Matrix4f::Identity();
  bool has_last_published_output_ = false;
  Eigen::Matrix4f raw_pred_before_lost_seed_ = Eigen::Matrix4f::Identity();
  int lost_reloc_publish_frames_left_ = 0;

  Eigen::Matrix4f pending_predictor_anchor_pose_ = Eigen::Matrix4f::Identity();
  bool predictor_anchor_pending_ = false;
  ros::Time last_lost_reloc_attempt_;

  LocState loc_state_ = LocState::INIT;
  int reject_streak_ = 0;
  int state_reject_to_lost_count_ = 6;
  Pose2D dr_last_pose_;
  bool has_dr_last_pose_ = false;
  double dr_predict_distance_accum_m_ = 0.0;
  double dr_predict_yaw_accum_rad_ = 0.0;
  int dr_predict_frames_ = 0;
  int dr_recover_accept_streak_ = 0;
};

} // namespace
} // namespace localization_ndt

int main(int argc, char **argv) {
  ros::init(argc, argv, "csm_localizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    localization_ndt::CsmLocalizerNode node(nh, pnh);
    node.spin();
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("[CSM] fatal: " << e.what());
    return 1;
  }
  return 0;
}
