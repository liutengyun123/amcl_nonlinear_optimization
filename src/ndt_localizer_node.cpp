#include "localization_ndt/ndt_localizer_node.hpp"
#include "localization_ndt/config/yaml_utils.hpp"
#include "localization_ndt/dynamic_point_filter.hpp"
#include "localization_ndt/map_json_utils.hpp"
#include "localization_ndt/map_processing.hpp"

#include "localization_msgs/RegistLoc.h"
#include "map_server/GetMap.h"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace localization_ndt {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTemporalMinDt = 0.02; // 20ms 以下不做时序检查
constexpr int kDegenProcessFrames = 25;
constexpr int kDegenExitSafeFrames = 25;

constexpr uint8_t kReasonInputScanTimeout = 50;
constexpr uint8_t kReasonInputOdomTimeout = 51;
constexpr uint8_t kReasonInputImuTimeout = 52;
constexpr uint8_t kReasonInputMultiTimeout = 53;

static inline std::string JoinStr(const std::vector<std::string> &v,
                                  const std::string &sep) {
  std::ostringstream oss;
  for (size_t i = 0; i < v.size(); ++i) {
    if (i)
      oss << sep;
    oss << v[i];
  }
  return oss.str();
}

static inline std::string TrimCopy(const std::string &s) {
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  size_t b = 0, e = s.size();
  while (b < e && is_space(static_cast<unsigned char>(s[b])))
    ++b;
  while (e > b && is_space(static_cast<unsigned char>(s[e - 1])))
    --e;
  return s.substr(b, e - b);
}

static inline bool ParseMapLayerSuffix(const std::string &in, std::string *base,
                                       int *layer) {
  if (!base || !layer)
    return false;
  *base = in;
  *layer = 0;

  if (in.size() < 3)
    return false;

  const size_t pos = in.find_last_of('_');
  if (pos == std::string::npos || pos + 2 > in.size())
    return false;
  const char c = in[pos + 1];
  if (c != 'l' && c != 'L')
    return false;

  const std::string num = in.substr(pos + 2);
  if (num.empty())
    return false;

  for (char ch : num) {
    if (std::isdigit(static_cast<unsigned char>(ch)) == 0)
      return false;
  }

  int v = 0;
  try {
    v = std::stoi(num);
  } catch (...) {
    return false;
  }

  if (v <= 0)
    return false;

  *base = in.substr(0, pos);
  if (base->empty())
    return false;

  *layer = v;
  return true;
}

static inline bool HasParamsYaml(const std::string &params_dir,
                                 const std::string &map_id) {
  if (map_id.empty())
    return false;
  return localization_ndt::config::FileExists(params_dir + "/" + map_id +
                                              ".yaml");
}

static inline bool HasPoseYaml(const std::string &fixed_dir,
                               const std::string &last_dir,
                               const std::string &map_id) {
  if (map_id.empty())
    return false;
  const std::string name = map_id + ".yaml";
  return localization_ndt::config::FileExists(fixed_dir + "/" + name) ||
         localization_ndt::config::FileExists(last_dir + "/" + name);
}

static inline bool HasMapAssets(const std::string &params_dir,
                                const std::string &fixed_dir,
                                const std::string &last_dir,
                                const std::string &map_id) {
  return HasParamsYaml(params_dir, map_id) ||
         HasPoseYaml(fixed_dir, last_dir, map_id);
}

static std::string
SelectMapStem(const std::string &params_dir, const std::string &fixed_dir,
              const std::string &last_dir,
              const std::string &preferred, const std::string &alt,
              const std::string &json_stem) {
  // Prefer explicit map id (even if assets missing) so params/pose follow map
  // name and can be auto-created by existing default generation logic.
  if (!preferred.empty())
    return preferred;
  if (HasMapAssets(params_dir, fixed_dir, last_dir, alt))
    return alt;
  if (HasMapAssets(params_dir, fixed_dir, last_dir, json_stem))
    return json_stem;
  if (!alt.empty())
    return alt;
  return json_stem;
}

static inline void YawToQuat(double yaw, double *qx, double *qy, double *qz,
                             double *qw) {
  const double half = 0.5 * yaw;
  if (qx)
    *qx = 0.0;
  if (qy)
    *qy = 0.0;
  if (qz)
    *qz = std::sin(half);
  if (qw)
    *qw = std::cos(half);
}

inline double NormalizeAngle(double angle) {
  while (angle > kPi)
    angle -= 2.0 * kPi;
  while (angle < -kPi)
    angle += 2.0 * kPi;
  return angle;
}

static double LerpAngle(double a, double b, double w) {
  double d = NormalizeAngle(b - a);
  return NormalizeAngle(a + w * d);
}

static inline int ComputeInitGrade(double tp_raw, double score, double tp_hard,
                                   double tp_soft, bool soft_enable,
                                   double score_strict, double score_relaxed) {
  const bool tp_h = std::isfinite(tp_raw) && (tp_raw >= tp_hard);
  const bool tp_s = std::isfinite(tp_raw) && (tp_raw >= tp_soft);
  const bool sc_strict_ok = std::isfinite(score) && (score <= score_strict);
  const bool sc_relaxed_ok = std::isfinite(score) && (score <= score_relaxed);
  const bool pass_hard = tp_h && sc_relaxed_ok;
  const bool pass_soft = soft_enable && tp_s && sc_strict_ok;
  return pass_hard ? 2 : (pass_soft ? 1 : 0);
}

static inline bool BetterPassNoPerturb(int grade_a, double score_a, double tp_a,
                                       int grade_b, double score_b,
                                       double tp_b) {
  if (grade_a != grade_b)
    return grade_a > grade_b;
  if (score_a != score_b)
    return score_a < score_b;
  return tp_a > tp_b;
}

bool QueryOdomPose(const std::deque<Odom2D> &buf, const ros::Time &t,
                   double max_age, Odom2D *out) {
  if (!out || buf.size() < 2)
    return false;
  if (t < buf.front().t || t > buf.back().t)
    return false;

  // 找到 [i, i+1] 包住 t
  size_t i = 0;
  while (i + 1 < buf.size() && buf[i + 1].t < t)
    ++i;
  if (i + 1 >= buf.size())
    return false;

  const auto &a = buf[i];
  const auto &b = buf[i + 1];

  const double ta = a.t.toSec(), tb = b.t.toSec(), tt = t.toSec();
  const double dt = tb - ta;
  if (dt <= 1e-6)
    return false;

  // 新鲜度
  const double age = std::min(std::fabs(tt - ta), std::fabs(tb - tt));
  if (age > max_age)
    return false;

  const double w = (tt - ta) / dt;

  out->t = t;
  out->x = a.x + w * (b.x - a.x);
  out->y = a.y + w * (b.y - a.y);
  out->yaw = LerpAngle(a.yaw, b.yaw, w);
  return true;
}

// 允许超出 buf 范围时“夹取到边界”，用于 deskew 的鲁棒性：
// - t < front.t: 直接用 front
// - t > back.t : 直接用 back
// - 其余区间：走线性插值
// max_age: 与目标时间的最大允许差（秒）
bool QueryOdomPoseClamped(const std::deque<Odom2D> &buf, const ros::Time &t,
                          double max_age, Odom2D *out) {
  if (!out || buf.empty())
    return false;

  const double tt = t.toSec();

  // buf 只有一个点：只要不太旧就用它
  if (buf.size() == 1) {
    const double age = std::fabs(buf.front().t.toSec() - tt);
    if (age > max_age) return false;
    *out = buf.front();
    out->t = t;
    return true;
  }

  // 夹取到边界
  if (t <= buf.front().t) {
    const double age = std::fabs(buf.front().t.toSec() - tt);
    if (age > max_age) return false;
    *out = buf.front();
    out->t = t;
    return true;
  }
  if (t >= buf.back().t) {
    const double age = std::fabs(buf.back().t.toSec() - tt);
    if (age > max_age) return false;
    *out = buf.back();
    out->t = t;
    return true;
  }

  // 区间内：插值（带新鲜度检查）
  if (QueryOdomPose(buf, t, max_age, out)) return true;

  // 万一 QueryOdomPose 因为边缘 dt 过小失败，退化为“最近邻”
  size_t best = 0;
  double best_dt = 1e18;
  for (size_t i = 0; i < buf.size(); ++i) {
    const double d = std::fabs(buf[i].t.toSec() - tt);
    if (d < best_dt) { best_dt = d; best = i; }
  }
  if (best_dt > max_age) return false;
  *out = buf[best];
  out->t = t;
  return true;
}

// 线性插值 gyro_z：t 在 buf 内则插值；超出则夹取到端点（但要求不太“旧”）
bool QueryGyroZInterpClamped(const std::deque<GyroZ> &buf,
                             const ros::Time &t,
                             double max_age,
                             double *wz_out,
                             size_t *hint_idx /*可为nullptr*/) {
  if (!wz_out || buf.empty()) return false;

  const double tt = t.toSec();

  // clamp to ends
  if (t <= buf.front().t) {
    const double age = std::fabs(buf.front().t.toSec() - tt);
    if (age > max_age) return false;
    *wz_out = buf.front().wz;
    if (hint_idx) *hint_idx = 0;
    return true;
  }
  if (t >= buf.back().t) {
    const double age = std::fabs(buf.back().t.toSec() - tt);
    if (age > max_age) return false;
    *wz_out = buf.back().wz;
    if (hint_idx) *hint_idx = (buf.size() >= 2 ? buf.size() - 2 : 0);
    return true;
  }

  // t inside range: find bracket [i, i+1]
  size_t i = 0;
  if (hint_idx && *hint_idx < buf.size()) i = *hint_idx;

  // make sure buf[i].t <= t < buf[i+1].t
  while (i + 1 < buf.size() && buf[i + 1].t < t) ++i;
  while (i > 0 && buf[i].t > t) --i;

  if (i + 1 >= buf.size()) {
    // should not happen due to clamp above, but keep safe
    *wz_out = buf.back().wz;
    if (hint_idx) *hint_idx = (buf.size() >= 2 ? buf.size() - 2 : 0);
    return true;
  }

  const auto &a = buf[i];
  const auto &b = buf[i + 1];

  const double ta = a.t.toSec(), tb = b.t.toSec();
  const double dt = tb - ta;
  if (dt <= 1e-9) {
    *wz_out = a.wz;
    if (hint_idx) *hint_idx = i;
    return true;
  }

  // “新鲜度”：t 离两端不要太远（防止 buffer 被 prune 后插值跨很久）
  const double age = std::min(std::fabs(tt - ta), std::fabs(tb - tt));
  if (age > max_age) return false;

  const double w = (tt - ta) / dt;
  *wz_out = a.wz + w * (b.wz - a.wz);

  if (hint_idx) *hint_idx = i;
  return true;
}


bool QueryGyroZNearest(const std::deque<GyroZ> &buf, const ros::Time &t,
                       double max_age, double *wz_out) {
  if (!wz_out || buf.empty())
    return false;
  const double tt = t.toSec();
  size_t best = 0;
  double best_dt = 1e18;
  for (size_t i = 0; i < buf.size(); ++i) {
    const double d = std::fabs(buf[i].t.toSec() - tt);
    if (d < best_dt) {
      best_dt = d;
      best = i;
    }
  }
  if (best_dt > max_age)
    return false;
  *wz_out = buf[best].wz;
  return true;
}

inline const char *LocLevelStr(localization_ndt::LocLevel lvl) {
  using L = localization_ndt::LocLevel;
  switch (lvl) {
  case L::INITIALIZING:
    return "INIT";
  case L::GOOD:
    return "GOOD";
  case L::FOLLOWING_DR:
    return "DR";
  case L::FAIL:
    return "FAIL";
  default:
    return "UNK";
  }
}

inline const char *ModeStr(uint8_t mode) {
  using S = localization_msgs::LocalizationStatus;
  switch (mode) {
  case S::MODE_INITIALIZING:
    return "INIT";
  case S::MODE_NDT:
    return "NDT";
  case S::MODE_FOLLOWING_DR:
    return "DR";
  case S::MODE_FAIL:
    return "FAIL";
  default:
    return "UNK";
  }
}

inline std::string FmtD(double v, int prec = 2) {
  if (!std::isfinite(v))
    return "nan";
  std::ostringstream oss;
  oss.setf(std::ios::fixed);
  oss << std::setprecision(prec) << v;
  return oss.str();
}

static Eigen::Matrix4f
Pose2DToMat(const localization_ndt::InitialPoseManager::Pose2D &p) {
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

static inline Eigen::Matrix3d SE2(double x, double y, double yaw) {
  Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
  const double c = std::cos(yaw), s = std::sin(yaw);
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  T(0, 2) = x;
  T(1, 2) = y;
  return T;
}

static inline Eigen::Matrix3d SE2Inv(const Eigen::Matrix3d &T) {
  Eigen::Matrix3d inv = Eigen::Matrix3d::Identity();
  inv.block<2, 2>(0, 0) = T.block<2, 2>(0, 0).transpose();
  Eigen::Vector2d t(T(0, 2), T(1, 2));
  Eigen::Vector2d tinv = -inv.block<2, 2>(0, 0) * t;
  inv(0, 2) = tinv.x();
  inv(1, 2) = tinv.y();
  return inv;
}

struct DegeneracyDecision {
  bool valid = false;
  bool degenerate = false;
  bool yaw_weak = false;
  double cond = std::numeric_limits<double>::quiet_NaN();
  double lambda_min = std::numeric_limits<double>::quiet_NaN();
  double lambda_max = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector2f weak_dir = Eigen::Vector2f(1.0f, 0.0f);
  double alpha_w = 1.0;
  double alpha_s = 1.0;
  double alpha_yaw = 1.0;
};

inline double Clamp01(double v) {
  return std::max(0.0, std::min(1.0, v));
}

inline double Lerp(double a, double b, double t) {
  return a + (b - a) * Clamp01(t);
}

bool BuildPlanarTranslationSchurFrom6D(const Eigen::Matrix<double, 6, 6> &H6,
                                       Eigen::Matrix2d *Ht_out,
                                       double *Hyy_out) {
  if (!Ht_out)
    return false;

  const int ix = 0, iy = 1, iyaw = 5; // x, y, yaw
  Eigen::Matrix2d Htt = Eigen::Matrix2d::Zero();
  Htt(0, 0) = H6(ix, ix);
  Htt(0, 1) = H6(ix, iy);
  Htt(1, 0) = H6(iy, ix);
  Htt(1, 1) = H6(iy, iy);

  Eigen::Vector2d Hty = Eigen::Vector2d::Zero();
  Hty(0) = H6(ix, iyaw);
  Hty(1) = H6(iy, iyaw);

  const double Hyy = H6(iyaw, iyaw);
  if (Hyy_out)
    *Hyy_out = Hyy;

  if (!Htt.allFinite() || !Hty.allFinite() || !std::isfinite(Hyy))
    return false;

  // H_t = H_tt - H_ttheta * H_thetatheta^{-1} * H_thetat
  // If yaw curvature is near zero, Schur becomes numerically unstable.
  // Fall back to plain translational block instead of returning invalid.
  constexpr double kMinAbsHyy = 1e-9;
  Eigen::Matrix2d Ht = Htt;
  if (std::fabs(Hyy) > kMinAbsHyy) {
    Ht = Htt - (Hty * (1.0 / Hyy) * Hty.transpose());
  }
  Ht = 0.5 * (Ht + Ht.transpose());

  if (!Ht.allFinite())
    return false;

  *Ht_out = Ht;
  return true;
}

DegeneracyDecision EvaluateDegeneracyFromHessian(
    const Eigen::Matrix<double, 6, 6> &H6,
    const config::NdtLocalizerConfig &cfg) {
  DegeneracyDecision out;

  Eigen::Matrix2d Ht = Eigen::Matrix2d::Zero();
  double Hyy = std::numeric_limits<double>::quiet_NaN();
  if (!BuildPlanarTranslationSchurFrom6D(H6, &Ht, &Hyy))
    return out;

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(Ht);
  if (es.info() != Eigen::Success)
    return out;

  const Eigen::Vector2d eig = es.eigenvalues(); // ascending (algebraic)

  // Use |lambda| to be robust to sign convention differences.
  int idx_min_abs = 0;
  double min_abs = std::fabs(eig(0));
  double max_abs = min_abs;
  for (int i = 1; i < 2; ++i) {
    const double a = std::fabs(eig(i));
    if (a < min_abs) {
      min_abs = a;
      idx_min_abs = i;
    }
    if (a > max_abs) {
      max_abs = a;
    }
  }

  out.lambda_min = min_abs;
  out.lambda_max = max_abs;

  if (!std::isfinite(out.lambda_min) || !std::isfinite(out.lambda_max) ||
      out.lambda_max <= 0.0)
    return out;

  const double eps = std::max(1e-12, cfg.degen_cov_eps);
  out.cond = out.lambda_max / std::max(out.lambda_min, eps);
  if (!std::isfinite(out.cond))
    return out;

  Eigen::Vector2f weak_xy = es.eigenvectors().col(idx_min_abs).cast<float>();
  if (!weak_xy.allFinite() || weak_xy.norm() < 1e-5f) {
    weak_xy = Eigen::Vector2f(1.0f, 0.0f);
  } else {
    weak_xy.normalize();
  }
  out.weak_dir = weak_xy;

  // Yaw observability from yaw curvature magnitude (after decoupling translation).
  const double yaw_curv = std::fabs(Hyy);
  out.yaw_weak = (!std::isfinite(yaw_curv) ||
                  yaw_curv <= std::max(cfg.degen_lambda_min_thresh, out.lambda_min));

  // Degeneracy decision for triggering is primarily ratio-based (cond),
  // avoiding raw lambda magnitude sensitivity across point/filter scales.
  out.degenerate = (out.cond >= cfg.degen_cond_ok);

  // Use log(cond) interpolation: cond spans orders of magnitude.
  double alpha_w = 1.0;
  if (out.cond >= cfg.degen_cond_ok) {
    if (out.cond >= cfg.degen_cond_bad) {
      alpha_w = cfg.degen_alpha_w_min;
    } else {
      const double c0 = std::max(cfg.degen_cond_ok, 1.0 + 1e-6);
      const double c1 = std::max(cfg.degen_cond_bad, c0 + 1e-6);
      const double cc = std::max(out.cond, c0);
      const double t = (std::log(cc) - std::log(c0)) /
                       std::max(1e-6, std::log(c1) - std::log(c0));
      alpha_w = Lerp(1.0, cfg.degen_alpha_w_min, t);
    }
  }
  // Keep lambda_min as an auxiliary clamp, but don't let it dominate alone.
  if (out.lambda_min <= cfg.degen_lambda_min_thresh &&
      out.cond >= std::max(1.0, 0.5 * cfg.degen_cond_ok)) {
    alpha_w = std::min(alpha_w, cfg.degen_alpha_w_min);
  }

  out.alpha_w = std::max(cfg.degen_alpha_w_min, std::min(1.0, alpha_w));
  out.alpha_s = Clamp01(cfg.degen_alpha_s);
  out.alpha_yaw = Clamp01(cfg.degen_alpha_yaw);
  if (out.yaw_weak) {
    out.alpha_yaw = std::min(out.alpha_yaw, std::max(cfg.degen_alpha_w_min, out.alpha_w));
  }

  out.valid = true;
  return out;
}

Eigen::Matrix4f FusePoseAnisotropic(const Eigen::Matrix4f &T_pred,
                                    const Eigen::Matrix4f &T_ndt,
                                    const Eigen::Vector2f &weak_dir,
                                    double alpha_w,
                                    double alpha_s,
                                    double alpha_yaw) {
  Eigen::Vector2f ew = weak_dir;
  if (!ew.allFinite() || ew.norm() < 1e-5f) {
    ew = Eigen::Vector2f(1.0f, 0.0f);
  } else {
    ew.normalize();
  }
  const Eigen::Vector2f es(-ew.y(), ew.x());

  const Eigen::Vector2f t_pred = T_pred.block<3, 1>(0, 3).head<2>();
  const Eigen::Vector2f t_ndt = T_ndt.block<3, 1>(0, 3).head<2>();

  const Eigen::Vector2f dt = t_ndt - t_pred;
  const double dw = dt.dot(ew);
  const double ds = dt.dot(es);

  const Eigen::Vector2f t_out =
      t_pred + static_cast<float>(Clamp01(alpha_w) * dw) * ew +
      static_cast<float>(Clamp01(alpha_s) * ds) * es;

  const double yaw_pred = std::atan2(T_pred(1, 0), T_pred(0, 0));
  const double yaw_ndt = std::atan2(T_ndt(1, 0), T_ndt(0, 0));
  const double yaw_out = LerpAngle(yaw_pred, yaw_ndt, Clamp01(alpha_yaw));

  Eigen::Matrix4f T_out = T_ndt;
  const float c = static_cast<float>(std::cos(yaw_out));
  const float s = static_cast<float>(std::sin(yaw_out));
  T_out(0, 0) = c;
  T_out(0, 1) = -s;
  T_out(0, 2) = 0.0f;
  T_out(1, 0) = s;
  T_out(1, 1) = c;
  T_out(1, 2) = 0.0f;
  T_out(2, 0) = 0.0f;
  T_out(2, 1) = 0.0f;
  T_out(2, 2) = 1.0f;

  T_out(0, 3) = t_out.x();
  T_out(1, 3) = t_out.y();
  return T_out;
}

Eigen::Matrix4f OffsetPoseAlongWeak(const Eigen::Matrix4f &T,
                                    const Eigen::Vector2f &weak_dir,
                                    float offset_m) {
  Eigen::Vector2f ew = weak_dir;
  if (!ew.allFinite() || ew.norm() < 1e-5f) {
    ew = Eigen::Vector2f(1.0f, 0.0f);
  } else {
    ew.normalize();
  }

  Eigen::Matrix4f out = T;
  out(0, 3) += offset_m * ew.x();
  out(1, 3) += offset_m * ew.y();
  return out;
}

} // namespace

// ====================== NdtLocalizerNode ======================

void NdtLocalizerNode::spin() { ros::spin(); }

NdtLocalizerNode::~NdtLocalizerNode() { writeShutdownPose_(); }

bool NdtLocalizerNode::registerToLocManager_() {
  localization_msgs::RegistLoc srv;
  srv.request.name = algo_name_;
  srv.request.need_mask = 1u; // NEED_JSON
  srv.request.capability_mask = 0u;

  srv.request.enable_srv = pnh_.resolveName("algo_enable");
  srv.request.switch_map_srv = pnh_.resolveName("algo_switch_map");
  srv.request.set_initial_pose_srv = pnh_.resolveName("algo_set_initial_pose");

  srv.request.pose_topic = pose_pub_.getTopic();
  srv.request.status_topic = status_pub_.getTopic();

  ros::ServiceClient c =
      nh_.serviceClient<localization_msgs::RegistLoc>(loc_manager_regist_srv_);
  if (!c.exists())
    c.waitForExistence(ros::Duration(1.0));

  if (!c.call(srv)) {
    ROS_WARN_STREAM("[LOC] regist_loc call failed: " << loc_manager_regist_srv_);
    return false;
  }
  if (!srv.response.ok) {
    ROS_WARN_STREAM("[LOC] regist_loc rejected: " << srv.response.message);
    return false;
  }

  ROS_INFO_STREAM("[LOC] regist_loc ok: "
                  << srv.response.message << " name=" << algo_name_
                  << " enable_srv=" << srv.request.enable_srv
                  << " switch_map_srv=" << srv.request.switch_map_srv
                  << " set_initial_pose_srv=" << srv.request.set_initial_pose_srv);
  return true;
}

bool NdtLocalizerNode::fetchMapJsonFromServer_(const std::string &map,
                                               int layer,
                                               std::string *out_json_path,
                                               std::string *out_err,
                                               std::string *out_resolved_map,
                                               int *out_resolved_layer) {
  if (out_err)
    out_err->clear();
  if (out_json_path)
    out_json_path->clear();
  if (out_resolved_map)
    out_resolved_map->clear();
  if (out_resolved_layer)
    *out_resolved_layer = 0;

  const std::string req_map = TrimCopy(map);
  const int req_layer = req_map.empty() ? 0 : (layer <= 0 ? 1 : layer);

  map_server::GetMap srv;
  srv.request.map = req_map;
  srv.request.layer = req_layer;
  srv.request.need_mask = 1u; // NEED_JSON

  ros::ServiceClient c =
      nh_.serviceClient<map_server::GetMap>(map_server_get_map_srv_);
  if (!c.exists())
    c.waitForExistence(ros::Duration(1.0));

  if (!c.call(srv)) {
    if (out_err)
      *out_err = "call failed: " + map_server_get_map_srv_;
    return false;
  }
  if (!srv.response.ok) {
    if (out_err)
      *out_err = "map_server/get_map failed: " + srv.response.message;
    return false;
  }

  const std::string resolved_map = TrimCopy(srv.response.map);
  const int resolved_layer = srv.response.layer;

  if (out_json_path)
    *out_json_path = srv.response.json_path;
  if (out_json_path && out_json_path->empty()) {
    if (out_err)
      *out_err = "map_server returned empty json_path";
    return false;
  }

  if (out_resolved_map) {
    if (!resolved_map.empty()) {
      *out_resolved_map = resolved_map;
    } else {
      *out_resolved_map = req_map;
    }
  }
  if (out_resolved_layer) {
    if (resolved_layer > 0) {
      *out_resolved_layer = resolved_layer;
    } else if (req_layer > 0) {
      *out_resolved_layer = req_layer;
    } else {
      *out_resolved_layer = 1;
    }
  }
  return true;
}

void NdtLocalizerNode::applyConfigToDynFilter_() {
  dyn_cfg_.enable = cfg_.dyn_filter_enable;
  dyn_cfg_.grid_res_m = cfg_.dyn_grid_res_m;
  dyn_cfg_.inflate_m = cfg_.dyn_inflate_m;
  dyn_cfg_.padding_m = cfg_.dyn_padding_m;
  dyn_cfg_.keep_ratio = cfg_.dyn_keep_ratio;
  dyn_cfg_.min_keep_points = cfg_.dyn_min_keep_points;
  dyn_cfg_.hard_max_dist_m = cfg_.dyn_hard_max_dist_m;

  dyn_cfg_.ray_enable = cfg_.dyn_ray_enable;
  dyn_cfg_.ray_margin_near_m = cfg_.dyn_ray_margin_near_m;
  dyn_cfg_.ray_max_range_m = cfg_.dyn_ray_max_range_m;
  dyn_cfg_.ray_hit_eps_m = cfg_.dyn_ray_hit_eps_m;
  dyn_cfg_.ray_max_steps = cfg_.dyn_ray_max_steps;
  dyn_cfg_.ray_confirm_df_m = cfg_.dyn_ray_confirm_df_m;
  dyn_cfg_.ray_step_m = cfg_.dyn_ray_step_m;

  dyn_cfg_.front_protect_enable = cfg_.dyn_front_protect_enable;
  dyn_cfg_.front_protect_x_min_m = cfg_.dyn_front_protect_x_min_m;
  dyn_cfg_.front_protect_x_max_m = cfg_.dyn_front_protect_x_max_m;
  dyn_cfg_.front_protect_abs_y_max_m = cfg_.dyn_front_protect_abs_y_max_m;
  dyn_cfg_.front_protect_use_y_axis = cfg_.dyn_front_protect_use_y_axis;
  dyn_cfg_.front_protect_forward_sign = cfg_.dyn_front_protect_forward_sign;
  dyn_cfg_.front_protect_max_drop_ratio = cfg_.dyn_front_protect_max_drop_ratio;
  dyn_cfg_.front_protect_min_in_points = cfg_.dyn_front_protect_min_in_points;
}

bool NdtLocalizerNode::applyMapSwitch_(const std::string &map, int layer,
                                       const std::string &json_path,
                                       std::string *out_err) {
  if (out_err)
    out_err->clear();

  if (map.empty()) {
    if (out_err)
      *out_err = "map is empty";
    return false;
  }
  if (json_path.empty()) {
    if (out_err)
      *out_err = "json_path is empty";
    return false;
  }

  const std::string json_stem =
      localization_ndt::config::ExtractFileStem(json_path);
  const int use_layer = (layer <= 0 ? 1 : layer);

  const std::string base_id = map;
  const std::string layer_id =
      base_id.empty() ? std::string()
                      : (base_id + "_l" + std::to_string(use_layer));

  std::string preferred_id;
  if (!base_id.empty()) {
    preferred_id = layer_id;
  }

  std::string alt_id;
  if (!base_id.empty()) {
    alt_id = base_id;
  }

  const std::string map_stem =
      SelectMapStem(params_dir_, pose_fixed_dir_, pose_last_dir_,
                    preferred_id, alt_id, json_stem);
  if (map_stem.empty()) {
    if (out_err)
      *out_err = "map_stem empty for json_path=" + json_path;
    return false;
  }

  if (!json_stem.empty() && map_stem != json_stem) {
    ROS_INFO_STREAM("[LOC] map_stem select: map="
                    << map << " layer=" << use_layer
                    << " json_stem=" << json_stem << " use=" << map_stem);
  }

  // reload config for new map
  (void)localization_ndt::config::LoadNdtLocalizerConfig(pkg_path_, params_dir_,
                                                         map_stem, pnh_, &cfg_);
  cfg_.local_reloc_map_yaml = json_path; // force JSON for local_reloc
  applyConfigToDynFilter_();

  reloc_mgr_.setParams(cfg_.reloc_mgr_params);
  reloc_mgr_.ack();

  ndt_matcher_.setResolution(cfg_.ndt_resolution);
  ndt_matcher_.setNumThreads(cfg_.ndt_num_threads);

  // load map cloud from JSON and derive ndt target
  std::string load_err;
  MapArtifacts art;
  if (!LoadMapArtifactsFromJson(json_path, &art, &load_err,
                                cfg_.ndt_target_use_known_mask_filter)) {
    if (out_err)
      *out_err = "load JSON map failed: " + load_err;
    return false;
  }

  // strict mode: require ndt_target_cloud
  if (!art.ndt_target_cloud || art.ndt_target_cloud->empty()) {
    if (out_err)
      *out_err =
          "ndt_target_cloud is empty (surface build failed) - strict mode.";
    return false;
  }

  // ---- commit state only after validation ----
  map_ = art.cloud;
  map_json_path_ = json_path;
  map_stem_ = map_stem;
  map_name_ = map;
  map_layer_ = use_layer;

  ndt_matcher_.setTargetMap(art.ndt_target_cloud);

  ROS_INFO_STREAM("[LOC] setTargetMap strict: ndt_target pts="
                  << art.ndt_target_cloud->size()
                  << " raw_occ pts=" << (art.cloud ? art.cloud->size() : 0));

  // initial pose manager reload
  initial_pose_mgr_.setFixedDir(pose_fixed_dir_);
  initial_pose_mgr_.setLastDir(pose_last_dir_);
  if (!initial_pose_mgr_.initializeWithMapId(map_json_path_, map_stem,
                                             std::string())) {
    if (out_err)
      *out_err =
          "InitialPoseManager::initialize failed for map=" + map_json_path_;
    return false;
  }

  // dyn filter
  dyn_filter_.setConfig(dyn_cfg_);
  if (dyn_cfg_.enable) {
    if (!dyn_filter_.setMap(map_)) {
      ROS_WARN_STREAM("[LOC] dyn_filter setMap failed -> disable dyn_filter");
      dyn_cfg_.enable = false;
      cfg_.dyn_filter_enable = false;
      dyn_filter_.setConfig(dyn_cfg_);
    }
  }

  // local reloc (use same JSON-derived artifacts as NDT map)
  if (cfg_.local_reloc_enable) {
    std::unique_ptr<LocalRelocalizer2D> lr(new LocalRelocalizer2D());
    lr->setPyrMaxLevel(cfg_.local_opt_base.pyr_max_level);
    lr->setScoreParams(cfg_.local_opt_base.hit_sigma_m,
                       cfg_.local_opt_base.max_dist_m);

    std::string lr_err;
    if (!BuildLocalRelocFromArtifacts(art, lr.get(), &lr_err)) {
      ROS_WARN_STREAM("[LOC] local_reloc map load failed: "
                      << lr_err << " -> disable local_reloc");
      cfg_.local_reloc_enable = false;
      local_reloc_.reset();
    } else {
      local_reloc_ = std::move(lr);
    }
  }

  // reset runtime states
  const bool had_odom = has_last_odom_msg_;
  const nav_msgs::Odometry odom_snapshot = last_odom_msg_;

  resetRuntimeState_();
  has_last_good_pose_ = false;
  prev_loc_level_ = LocLevel::INITIALIZING;
  has_map_odom_anchor_ = false;

  sensor_force_fail_ = false;
  sensor_force_reason_ = 0;
  sensor_force_detail_.clear();
  sensor_force_until_ = ros::Time(0);

  odom_buf_.clear();
  gyro_buf_.clear();

  if (had_odom) {
    predictor_.resetWithCorrection(Eigen::Matrix4f::Identity(), odom_snapshot);
  } else {
    nav_msgs::Odometry dummy;
    dummy.pose.pose.orientation.w = 1.0;
    predictor_.resetWithCorrection(Eigen::Matrix4f::Identity(), dummy);
  }

  buildInitCandidates_();

  logEffectiveConfig_(map_name_, map_stem_);
  ROS_INFO_STREAM("[LOC] map switched: map=" << map_name_
                                             << " layer=" << map_layer_
                                             << " json=" << map_json_path_);

  return true;
}

bool NdtLocalizerNode::onAlgoEnable(localization_msgs::AlgoEnable::Request &req,
                                    localization_msgs::AlgoEnable::Response &res) {
  const bool was_active = algo_enabled_;

  if (!req.enable) {
    algo_enabled_ = false;
    watchdog_timer_.stop();
    dtc_pub_.Shutdown();

    res.ok = true;
    res.message = was_active ? "standby" : "already standby";
    ROS_WARN_STREAM("[LOC] algo_enable=OFF -> standby (map kept loaded)");
    return true;
  }

  if (!map_) {
    std::string err;
    if (!bootstrapFromRunSelection_(&err)) {
      algo_enabled_ = false;
      res.ok = false;
      res.message = err.empty() ? "bootstrap failed" : err;
      ROS_WARN_STREAM("[LOC] algo_enable init failed: " << res.message);
      return true;
    }

    initRosInterfaces_();
    initWatchdog_();
    resetRuntimeState_();

    dtc_pub_.Init(nh_, "/diagnostics", 1.0 /*Hz*/, "localization",
                  "ndt_localizer");

    logEffectiveConfig_(map_name_, map_stem_);
  } else if (!was_active) {
    initWatchdog_();
    dtc_pub_.Init(nh_, "/diagnostics", 1.0 /*Hz*/, "localization",
                  "ndt_localizer");
  }

  algo_enabled_ = true;

  if (has_last_published_) {
    ros::Time stamp = ros::Time::now();
    if (stamp.isZero()) stamp = t_last_published_;
    publishOdom(stamp, T_last_published_);
  }

  res.ok = true;
  res.message = was_active ? "already active" : "active";
  ROS_WARN_STREAM("[LOC] algo_enable=ON -> active");
  return true;
}

bool NdtLocalizerNode::onAlgoSwitchMap(
    localization_msgs::AlgoSwitchMap::Request &req,
    localization_msgs::AlgoSwitchMap::Response &res) {
  if (!map_) {
    res.ok = false;
    res.message = "not initialized, enable first";
    return true;
  }
  std::string map = TrimCopy(req.map);
  int layer = req.layer;

  if (layer <= 0) {
    std::string base;
    int layer_from_name = 0;
    if (ParseMapLayerSuffix(map, &base, &layer_from_name)) {
      map = base;
      layer = layer_from_name;
    }
  }
  if (layer <= 0)
    layer = 1;

  if (map.empty()) {
    res.ok = false;
    res.message = "map is empty";
    return true;
  }

  std::string json_path;
  std::string err;
  std::string resolved_map;
  int resolved_layer = 0;
  if (!fetchMapJsonFromServer_(map, layer, &json_path, &err, &resolved_map,
                               &resolved_layer)) {
    res.ok = false;
    res.message = err;
    return true;
  }

  if (resolved_map.empty())
    resolved_map = map;
  if (resolved_layer <= 0)
    resolved_layer = layer;

  if (!applyMapSwitch_(resolved_map, resolved_layer, json_path, &err)) {
    res.ok = false;
    res.message = err;
    return true;
  }

  res.ok = true;
  res.message = "ok";
  return true;
}

bool NdtLocalizerNode::onAlgoSetInitialPose(
    localization_msgs::AlgoSetInitialPose::Request &req,
    localization_msgs::AlgoSetInitialPose::Response &res) {
  if (!map_) {
    res.ok = false;
    res.message = "not initialized, enable first";
    return true;
  }
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = req.frame_id.empty() ? "map" : req.frame_id;

  msg.pose.pose.position.x = req.x;
  msg.pose.pose.position.y = req.y;
  msg.pose.pose.position.z = 0.0;

  double qx, qy, qz, qw;
  YawToQuat(req.yaw, &qx, &qy, &qz, &qw);
  msg.pose.pose.orientation.x = qx;
  msg.pose.pose.orientation.y = qy;
  msg.pose.pose.orientation.z = qz;
  msg.pose.pose.orientation.w = qw;

  for (double &v : msg.pose.covariance)
    v = 0.0;
  msg.pose.covariance[0] = req.cov_x;
  msg.pose.covariance[7] = req.cov_y;
  msg.pose.covariance[35] = req.cov_yaw;

  applyInitialPoseMsg_(msg);

  res.ok = true;
  res.message = "ok";
  return true;
}

void NdtLocalizerNode::applyInitialPoseMsg_(
    const geometry_msgs::PoseWithCovarianceStamped &msg) {
  geometry_msgs::PoseWithCovarianceStampedPtr p(
      new geometry_msgs::PoseWithCovarianceStamped(msg));
  initialPoseCallback(p);
}

void NdtLocalizerNode::writeShutdownPose_() {
  if (shutdown_written_.exchange(true))
    return;

  // 只允许写“最后一次有效 NDT accept 位姿”
  if (!has_last_good_pose_) {
    ROS_WARN_STREAM("[LOC] event=shutdown_pose_skip reason=no_good_ndt_pose"
                    << " file=" << initial_pose_mgr_.lastYamlPath());
    return;
  }

  InitialPoseManager::Pose2D p;
  p.x = last_good_pose_.x;
  p.y = last_good_pose_.y;
  p.yaw = last_good_pose_.yaw;

  (void)initial_pose_mgr_.updateShutdownPose(p);

  ROS_WARN_STREAM("[LOC] event=shutdown_pose_written x="
                  << FmtD(p.x, 3) << " y=" << FmtD(p.y, 3)
                  << " yaw=" << FmtD(p.yaw, 3)
                  << " file=" << initial_pose_mgr_.lastYamlPath());
}

bool NdtLocalizerNode::loadMap(const std::string &path) {
  map_.reset(new PointCloudT);
  ROS_INFO_STREAM("Loading map PCD: " << path);
  if (pcl::io::loadPCDFile<PointT>(path, *map_) != 0) {
    ROS_ERROR_STREAM("Could not read PCD file: " << path);
    map_.reset();
    return false;
  }
  ROS_INFO_STREAM("Map loaded. Point size = " << map_->size());

  if (!map_->empty()) {
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*map_, min_pt, max_pt);
    ROS_INFO_STREAM("Map bounds: "
                    << "min[" << min_pt.x << ", " << min_pt.y << ", "
                    << min_pt.z << "], "
                    << "max[" << max_pt.x << ", " << max_pt.y << ", "
                    << max_pt.z << "]");
  }

  // Only set NDT target map here.
  ndt_matcher_.setTargetMap(map_);

  return true;
}


void NdtLocalizerNode::odomCallback(
    const nav_msgs::OdometryConstPtr &odom_msg) {
  predictor_.updateOdom(odom_msg);

  last_odom_msg_ = *odom_msg;
  has_last_odom_msg_ = true;

  latest_linear_speed_ = std::hypot(odom_msg->twist.twist.linear.x,
                                    odom_msg->twist.twist.linear.y);
  last_odom_stamp_ = odom_msg->header.stamp;

  // 提取 yaw
  const auto &q = odom_msg->pose.pose.orientation;
  Eigen::Quaterniond qq(q.w, q.x, q.y, q.z);
  qq.normalize();
  const double yaw =
      std::atan2(2.0 * (qq.w() * qq.z() + qq.x() * qq.y()),
                 1.0 - 2.0 * (qq.y() * qq.y() + qq.z() * qq.z()));

  odom_buf_.push_back({odom_msg->header.stamp, odom_msg->pose.pose.position.x,
                       odom_msg->pose.pose.position.y, NormalizeAngle(yaw)});

  // prune
  const ros::Time now = odom_msg->header.stamp;
  while (!odom_buf_.empty() &&
         (now - odom_buf_.front().t).toSec() > odom_buf_keep_sec_) {
    odom_buf_.pop_front();
  }
}

void NdtLocalizerNode::imuCallback(const sensor_msgs::ImuConstPtr &imu_msg) {
  predictor_.updateImu(imu_msg);

  last_imu_gyro_z_ = imu_msg->angular_velocity.z;
  has_last_imu_ = true;
  last_imu_stamp_ = imu_msg->header.stamp;

  gyro_buf_.push_back({imu_msg->header.stamp, imu_msg->angular_velocity.z});

  const ros::Time now = imu_msg->header.stamp;
  while (!gyro_buf_.empty() &&
         (now - gyro_buf_.front().t).toSec() > imu_buf_keep_sec_) {
    gyro_buf_.pop_front();
  }
}

bool NdtLocalizerNode::isInTemporalGrace(const ros::Time &stamp) const {
  return cfg_.temporal_grace_enable && !temporal_grace_until_.isZero() &&
         (stamp <= temporal_grace_until_);
}

// ===================== init candidates =====================

void NdtLocalizerNode::buildInitCandidates_() {
  init_candidates_.clear();
  init_tried_.clear();
  init_next_idx_ = 0;
  init_exhausted_ = false;

  const bool include_fixed =
      (init_stage_ == InitStage::FULL) || (loc_level_ == LocLevel::FAIL);

  auto norm_yaw = [&](double yaw) { return NormalizeAngle(yaw); };

  struct Key {
    long long ix, iy, iyaw;
    bool operator==(const Key &o) const {
      return ix == o.ix && iy == o.iy && iyaw == o.iyaw;
    }
  };
  struct KeyHash {
    std::size_t operator()(const Key &k) const {
      std::size_t h1 = std::hash<long long>{}(k.ix);
      std::size_t h2 = std::hash<long long>{}(k.iy);
      std::size_t h3 = std::hash<long long>{}(k.iyaw);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  auto q = [](double v, double eps) -> long long {
    return static_cast<long long>(std::llround(v / eps));
  };
  auto make_key = [&](const InitialPoseManager::Pose2D &p) -> Key {
    const double yaw = norm_yaw(p.yaw);
    return Key{q(p.x, cfg_.init_dedupe_xy_eps), q(p.y, cfg_.init_dedupe_xy_eps),
               q(yaw, cfg_.init_dedupe_yaw_eps)};
  };

  std::unordered_set<Key, KeyHash> seen;

  // 1) stable last（顶层 x/y/yaw）
  if (initial_pose_mgr_.hasLastPose()) {
    InitCandidate c;
    c.pose = initial_pose_mgr_.lastPose();
    c.pose.yaw = norm_yaw(c.pose.yaw);
    c.src = InitCandidate::Source::LAST;
    c.fixed_index = -1;
    c.name = "last";
    c.priority = 0;
    if (seen.insert(make_key(c.pose)).second)
      init_candidates_.push_back(c);
  }

  // 2) shutdown last（last.yaml 内 shutdown: {x,y,yaw}）
  if (initial_pose_mgr_.hasShutdownPose()) {
    InitCandidate c;
    c.pose = initial_pose_mgr_.shutdownPose();
    c.pose.yaw = norm_yaw(c.pose.yaw);
    c.src = InitCandidate::Source::LAST_SHUTDOWN;
    c.fixed_index = -1;
    c.name = "shutdown";
    c.priority = -1;
    if (seen.insert(make_key(c.pose)).second)
      init_candidates_.push_back(c);
  }

  // 3) fixed（仅 FULL 或 FAIL 用）
  if (include_fixed) {
    const auto &fixed = initial_pose_mgr_.fixedPoses();
    std::vector<size_t> order(fixed.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return fixed[a].priority > fixed[b].priority;
    });

    for (size_t k = 0; k < order.size(); ++k) {
      const size_t idx = order[k];
      InitCandidate c;
      c.pose = fixed[idx].pose;
      c.pose.yaw = norm_yaw(c.pose.yaw);
      c.src = InitCandidate::Source::FIXED;
      c.fixed_index = static_cast<int>(idx);
      c.name = fixed[idx].name.empty() ? ("fixed[" + std::to_string(idx) + "]")
                                       : fixed[idx].name;
      c.priority = fixed[idx].priority;

      if (seen.insert(make_key(c.pose)).second)
        init_candidates_.push_back(c);
    }

    // FULL/FAIL fallback：origin
    if (init_candidates_.empty()) {
      InitCandidate c;
      c.pose = InitialPoseManager::Pose2D();
      c.src = InitCandidate::Source::FIXED;
      c.fixed_index = -1;
      c.name = "fallback_origin";
      c.priority = 0;
      init_candidates_.push_back(c);
    }
  }

  // try_once：只对 FULL 生效（FAST 不走 exhausted/freeze）
  if (include_fixed && cfg_.init_try_once_per_candidate) {
    init_tried_.assign(init_candidates_.size(), 0);
  }

  ROS_INFO_STREAM(
      "[LOC] event=init_build stage="
      << (include_fixed ? "FULL" : "FAST")
      << " cand=" << init_candidates_.size() << " try_once="
      << ((include_fixed && cfg_.init_try_once_per_candidate) ? 1 : 0)
      << " K=" << cfg_.init_max_candidates_per_scan);
}

bool NdtLocalizerNode::maybeRearmInit_(const ros::Time &stamp) {
  if (!cfg_.init_rearm_enable)
    return false;
  if (!init_exhausted_)
    return false;
  if (!has_last_odom_msg_)
    return false;

  // cooldown
  if (!init_last_rearm_time_.isZero()) {
    const double cd = (stamp - init_last_rearm_time_).toSec();
    if (cd < cfg_.init_rearm_cooldown)
      return false;
  }

  const bool stationary = (latest_linear_speed_ < cfg_.stationary_speed_thresh);
  if (!stationary) {
    init_rearm_stationary_start_ = ros::Time(0);
    return false;
  }
  if (init_rearm_stationary_start_.isZero()) {
    init_rearm_stationary_start_ = stamp;
    return false;
  }
  const double stop_dt = (stamp - init_rearm_stationary_start_).toSec();
  if (stop_dt < cfg_.init_rearm_min_stop_duration)
    return false;

  // moved distance w.r.t. anchor
  Eigen::Vector2f odom_xy;
  odom_xy.x() = static_cast<float>(last_odom_msg_.pose.pose.position.x);
  odom_xy.y() = static_cast<float>(last_odom_msg_.pose.pose.position.y);

  double moved = std::numeric_limits<double>::infinity();
  if (has_init_exhausted_anchor_) {
    moved = (odom_xy - init_exhausted_odom_xy_).norm();
  }
  if (moved < cfg_.init_rearm_dist)
    return false;

  buildInitCandidates_();
  init_last_rearm_time_ = stamp;
  init_rearm_stationary_start_ = ros::Time(0);
  has_init_exhausted_anchor_ = false;

  ROS_INFO_STREAM("[LOC] event=init_rearm moved="
                  << FmtD(moved, 2) << " stop_dt=" << FmtD(stop_dt, 2));
  return true;
}

bool NdtLocalizerNode::tryInitializeFromCandidates_(
    const ros::Time &stamp, const PointCloudT::Ptr &cloud,
    const Eigen::Vector2f &sensor_origin_base) {
  if (initial_pose_applied_)
    return true;
  if (!has_last_odom_msg_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] event=init waiting=odom");
    return false;
  }
  if (!cloud || cloud->empty())
    return false;

  auto SrcStr = [](InitCandidate::Source s) -> const char * {
    switch (s) {
    case InitCandidate::Source::LAST:
      return "last";
    case InitCandidate::Source::LAST_SHUTDOWN:
      return "shutdown";
    case InitCandidate::Source::FIXED:
      return "fixed";
    default:
      return "unk";
    }
  };

  auto poseToMat = [&](const InitialPoseManager::Pose2D &p) {
    return Pose2DToMat(p);
  };

  // local_seed 
  const bool can_local_seed =
      (cfg_.local_reloc_enable && local_reloc_ && local_reloc_->isReady());

  std::vector<Eigen::Vector2f> scan_xy;
  if (can_local_seed) {
    const int maxN = std::max(1, cfg_.local_opt_base.max_scan_points);
    const size_t N = cloud->points.size();
    const size_t step =
        (N > static_cast<size_t>(maxN)) ? (N / static_cast<size_t>(maxN)) : 1;

    scan_xy.reserve(std::min<size_t>(N, static_cast<size_t>(maxN)));
    for (size_t i = 0; i < N; i += step) {
      const auto &pt = cloud->points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
        continue;
      scan_xy.emplace_back(pt.x, pt.y);
      if (static_cast<int>(scan_xy.size()) >= maxN)
        break;
    }

    if (scan_xy.empty()) {
      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "[LOC] event=init_local_seed scan_xy empty -> fallback raw seed");
    }
  }

  // 允许在一次调用里：FAST -> (失败) -> FULL
  for (int stage_iter = 0; stage_iter < 2; ++stage_iter) {
    const bool fast = (init_stage_ == InitStage::FAST);
    const bool try_once = (!fast) && cfg_.init_try_once_per_candidate;

    // FULL 才允许 exhausted 冻结 + rearm
    if (!fast && init_exhausted_) {
      if (!maybeRearmInit_(stamp)) {
        ROS_WARN_STREAM_THROTTLE(2.0, "[LOC] event=init_frozen exhausted=1");
        return false;
      }
      // rearm 成功后 candidates 已重建（buildInitCandidates_ 在 maybeRearmInit_
      // 内部会被调用）
    }

    // candidates 若为空就构建
    if (init_candidates_.empty()) {
      buildInitCandidates_();
      // FAST 阶段如果连 last/shutdown 都没有，直接切 FULL 再构建
      if (fast && init_candidates_.empty()) {
        init_stage_ = InitStage::FULL;
        buildInitCandidates_();
      }
    }

    // 计算 available
    int available = static_cast<int>(init_candidates_.size());

    // try_once：FULL 阶段会用 init_tried_ 限制剩余数量；FAST 不用
    if (try_once) {
      if (init_tried_.empty())
        init_tried_.assign(init_candidates_.size(), 0);
      int remain = 0;
      for (auto v : init_tried_)
        if (!v)
          ++remain;
      available = remain;

      if (available <= 0) {
        // exhausted 冻结
        init_exhausted_ = true;
        init_exhausted_odom_xy_.x() =
            static_cast<float>(last_odom_msg_.pose.pose.position.x);
        init_exhausted_odom_xy_.y() =
            static_cast<float>(last_odom_msg_.pose.pose.position.y);
        has_init_exhausted_anchor_ = true;
        init_rearm_stationary_start_ = ros::Time(0);
        ROS_WARN_STREAM("[LOC] event=init_exhausted exhausted=1");
        return false;
      }
    }

    if (available <= 0) {
      // FAST 阶段可出现：init_candidates_==0（无 last/shutdown），上面已切 FULL
      // FULL 阶段理论不会出现（至少会有 fallback_origin）
      return false;
    }

    // FAST：强制同帧把 last+shutdown 都试完
    int K = std::min(cfg_.init_max_candidates_per_scan, available);
    if (fast)
      K = available;
    if (K <= 0)
      return false;

    struct Best {
      bool ok = false;
      int grade = 0;
      double tp = -1e9;
      double score = 1e18;
      Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
      InitCandidate cand;

      bool used_local_seed = false;
      double local_best = std::numeric_limits<double>::quiet_NaN();
      double local_second = std::numeric_limits<double>::quiet_NaN();
      double local_vf = std::numeric_limits<double>::quiet_NaN();

      bool used_dyn = false;
      DynamicFilterStats dyn_stats{};

    } best;

    double max_tp_seen = std::numeric_limits<double>::quiet_NaN();
    double min_score_seen = std::numeric_limits<double>::quiet_NaN();
    int ndt_ok_cnt = 0;

    int attempted = 0;
    size_t idx = init_next_idx_;

    while (attempted < K) {
      size_t guard = 0;

      if (try_once && !init_tried_.empty()) {
        while (guard < init_candidates_.size() && init_tried_[idx]) {
          idx = (idx + 1) % init_candidates_.size();
          ++guard;
        }
        if (guard >= init_candidates_.size())
          break;
      }

      const size_t cand_idx = idx;
      idx = (idx + 1) % init_candidates_.size();
      ++attempted;

      const auto &cand = init_candidates_[cand_idx];

      Eigen::Matrix4f T_seed = poseToMat(cand.pose);

      bool used_local = false;
      double lb = std::numeric_limits<double>::quiet_NaN();
      double ls = std::numeric_limits<double>::quiet_NaN();
      double lvf = std::numeric_limits<double>::quiet_NaN();

      if (can_local_seed && !scan_xy.empty()) {
        RelocPose2D center;
        center.x = cand.pose.x;
        center.y = cand.pose.y;
        center.yaw = cand.pose.yaw;

        LocalRelocOptions opt = cfg_.local_opt_base;

        const auto r = local_reloc_->match(scan_xy, center, opt);
        if (r.ok) {
          T_seed = r.T_seed;
          used_local = true;
          lb = r.best_score;
          ls = r.second_score;
          lvf = r.valid_fraction;

          ROS_INFO_STREAM("[LOC] event=init_local_seed ok=1 src="
                          << SrcStr(cand.src) << " name=" << cand.name
                          << " best=" << FmtD(r.best_score, 3)
                          << " second=" << FmtD(r.second_score, 3)
                          << " vf=" << FmtD(r.valid_fraction, 2)
                          << " out(x=" << FmtD(r.best_pose.x, 3)
                          << " y=" << FmtD(r.best_pose.y, 3)
                          << " yaw=" << FmtD(r.best_pose.yaw, 2) << "rad)");
        } else {
          ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] event=init_local_seed ok=0 src="
                                            << SrcStr(cand.src)
                                            << " name=" << cand.name);
        }
      }

      // 记录：本候选是否真的做过 align（用于 try_once 的 tried 标记）
      bool attempted_align = false;

      // 本候选内的最好结果
      bool cand_ok = false;
      int cand_grade = 0;
      double cand_tp = -1e9;
      double cand_score = 1e18;
      Eigen::Matrix4f cand_T = Eigen::Matrix4f::Identity();
      bool cand_used_dyn = false;
      DynamicFilterStats cand_ds{};

      // 一个小工具：跑一次 align + 算 grade
      auto run_align = [&](const PointCloudT::Ptr &cloud_in,
                           Eigen::Matrix4f *T_out, double *score_out,
                           double *tp_out, int *grade_out) -> bool {
        attempted_align = true;

        Eigen::Matrix4f T_ndt = T_seed;
        double score = std::numeric_limits<double>::max();
        const bool ok = ndt_matcher_.align(cloud_in, T_seed, T_ndt, score);
        if (!ok)
          return false;

        ndt_ok_cnt++;

        const double tp_raw = ndt_matcher_.getTransformationProbability();

        if (std::isfinite(tp_raw)) {
          max_tp_seen = std::isfinite(max_tp_seen)
                            ? std::max(max_tp_seen, tp_raw)
                            : tp_raw;
        }
        if (std::isfinite(score)) {
          min_score_seen = std::isfinite(min_score_seen)
                               ? std::min(min_score_seen, score)
                               : score;
        }

        const int grade =
            ComputeInitGrade(tp_raw, score, cfg_.init_trans_prob_min,
                             cfg_.init_tp_soft, cfg_.init_score_soft_enable,
                             cfg_.init_score_max, cfg_.init_score_relaxed_max);

        if (T_out)
          *T_out = T_ndt;
        if (score_out)
          *score_out = score;
        if (tp_out)
          *tp_out = tp_raw;
        if (grade_out)
          *grade_out = grade;

        return true;
      };

      // -------------------- 1) dyn-first --------------------
      PointCloudT::Ptr cloud_dyn = cloud;
      DynamicFilterStats ds{};
      const bool dyn_ready = (dyn_cfg_.enable && dyn_filter_.isReady());
      bool dyn_effective = false;

      if (dyn_ready) {
        cloud_dyn = dyn_filter_.filter(cloud, T_seed, sensor_origin_base, &ds);
        if (!cloud_dyn || cloud_dyn->empty())
          cloud_dyn = cloud;

        // dyn 是否“确实过滤生效”：不是 insufficient_static 且砍掉了点
        dyn_effective = (!ds.insufficient_static && ds.in > 0 && ds.kept > 0 &&
                         ds.kept < ds.in);
      }

      {
        Eigen::Matrix4f T_try;
        double score_try;
        double tp_try;
        int grade_try;

        const bool ok =
            run_align(cloud_dyn, &T_try, &score_try, &tp_try, &grade_try);
        if (ok && grade_try > 0) {
          cand_ok = true;
          cand_grade = grade_try;
          cand_score = std::isfinite(score_try) ? score_try : 1e18;
          cand_tp = std::isfinite(tp_try) ? tp_try : -1e9;
          cand_T = T_try;
          cand_used_dyn = dyn_ready;
          cand_ds = ds;
        }
      }

      // -------------------- 2) raw fallback (only when needed)
      // -------------------- 只有在 dyn“确实过滤生效”但仍未通过门控时，才回退跑 raw，避免稳定翻倍。
      const bool need_raw_fallback = (dyn_ready && dyn_effective && !cand_ok);

      if (need_raw_fallback) {
        Eigen::Matrix4f T_try;
        double score_try;
        double tp_try;
        int grade_try;

        const bool ok =
            run_align(cloud, &T_try, &score_try, &tp_try, &grade_try);
        if (ok && grade_try > 0) {
          const double tp_eff = std::isfinite(tp_try) ? tp_try : -1e9;
          const double sc_eff = std::isfinite(score_try) ? score_try : 1e18;

          if (!cand_ok ||
              BetterPassNoPerturb(grade_try, sc_eff, tp_eff, cand_grade,
                                  cand_score, cand_tp)) {
            cand_ok = true;
            cand_grade = grade_try;
            cand_score = sc_eff;
            cand_tp = tp_eff;
            cand_T = T_try;
            cand_used_dyn = false; // raw 胜出
          }
        }
      }

      // -------------------- 3) try_once：只有真的跑过 align 才算 tried
      if (try_once && !init_tried_.empty() && attempted_align) {
        init_tried_[cand_idx] = 1;
      }

      // -------------------- 4) cand_ok 才参与全局 best 竞争
      if (cand_ok) {
        if (!best.ok || BetterPassNoPerturb(cand_grade, cand_score, cand_tp,
                                            best.grade, best.score, best.tp)) {
          best.ok = true;
          best.grade = cand_grade;
          best.score = cand_score;
          best.tp = cand_tp;
          best.T = cand_T;
          best.cand = cand;

          best.used_local_seed = used_local;
          best.local_best = lb;
          best.local_second = ls;
          best.local_vf = lvf;

          best.used_dyn = cand_used_dyn;
          best.dyn_stats = cand_ds;
        }
      }
    }

    init_next_idx_ = idx;

    if (!best.ok) {
      // FAST：last+shutdown 都不通过 -> 切 FULL 并继续（同一次调用里）
      if (fast) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[LOC] event=init_fast_try ok=0 attempted="
                     << attempted << " ndt_ok=" << ndt_ok_cnt
                     << " max_tp=" << FmtD(max_tp_seen, 3) << " min_score="
                     << FmtD(min_score_seen, 6) << " -> switch_to_FULL");
        init_stage_ = InitStage::FULL;
        buildInitCandidates_();
        continue;
      }

      // FULL： exhausted/freeze 日志逻辑
      int remain = -1;
      if (try_once) {
        remain = 0;
        for (auto v : init_tried_)
          if (!v)
            ++remain;
        if (remain == 0) {
          init_exhausted_ = true;
          init_exhausted_odom_xy_.x() =
              static_cast<float>(last_odom_msg_.pose.pose.position.x);
          init_exhausted_odom_xy_.y() =
              static_cast<float>(last_odom_msg_.pose.pose.position.y);
          has_init_exhausted_anchor_ = true;
          init_rearm_stationary_start_ = ros::Time(0);
        }
      }

      ROS_WARN_STREAM_THROTTLE(
          1.0,
          "[LOC] event=init_try ok=0 attempted="
              << attempted << " tp_min=" << FmtD(cfg_.init_trans_prob_min, 3)
              << " ndt_ok=" << ndt_ok_cnt << " max_tp=" << FmtD(max_tp_seen, 3)
              << " min_score=" << FmtD(min_score_seen, 6)
              << " soft(en=" << (cfg_.init_score_soft_enable ? 1 : 0)
              << " tp_soft=" << FmtD(cfg_.init_tp_soft, 3)
              << " score_max=" << FmtD(cfg_.init_score_max, 6) << ")"
              << " remain=" << remain
              << " exhausted=" << (init_exhausted_ ? 1 : 0));

      return false;
    }

    // ---- best.ok: 初始化成功（ commit/reset 等逻辑） ----
    predictor_.resetWithCorrection(best.T, last_odom_msg_);
    reloc_mgr_.ack();
    initial_pose_applied_ = true;

    // 一旦初始化成功，后续不再处于 FAST
    init_stage_ = InitStage::FULL;

    if (best.cand.src == InitCandidate::Source::FIXED &&
        best.cand.fixed_index >= 0) {
      (void)initial_pose_mgr_.bumpFixedPriority(
          static_cast<size_t>(best.cand.fixed_index), 1);
    }

    stationary_phase_ = false;
    stationary_pose_written_ = false;

    T_last_published_ = best.T;
    t_last_published_ = stamp;
    has_last_published_ = true;

    T_last_temporal_ = best.T;
    t_last_temporal_ = stamp;
    has_last_temporal_ = true;

    if (cfg_.temporal_grace_enable) {
      temporal_grace_until_ =
          stamp + ros::Duration(cfg_.temporal_grace_duration);
    }

    const double yaw_rad = std::atan2(best.T(1, 0), best.T(0, 0));
    const Eigen::Vector3f t = best.T.block<3, 1>(0, 3);

    ROS_INFO_STREAM("[LOC] t="
                << FmtD(stamp.toSec(), 3)
                << " event=init ok=1 src=" << SrcStr(best.cand.src)
                << " seed=" << (best.used_local_seed ? "local" : "raw")
                << " name=" << best.cand.name
                << " tp=" << FmtD(best.tp, 3)
                << " score=" << FmtD(best.score, 3)
                << " dyn=" << (best.used_dyn ? 1 : 0)
                << (best.used_dyn
                        ? (std::string(" keep=") + std::to_string(best.dyn_stats.kept) +
                           "/" + std::to_string(best.dyn_stats.in) +
                           " insuff=" + std::to_string(best.dyn_stats.insufficient_static ? 1 : 0))
                        : std::string(""))
                << " out(x=" << FmtD(t.x(), 3) << " y=" << FmtD(t.y(), 3)
                << " yaw=" << FmtD(yaw_rad, 2) << "rad)");
    return true;
  }

  return false;
}

bool NdtLocalizerNode::tryFixedLastRelocInFail_(
    const ros::Time &stamp,
    const PointCloudT::Ptr &cloud,
    const Eigen::Vector2f &sensor_origin_base) {

  if (!has_last_odom_msg_) return false;
  if (!cloud || cloud->empty()) return false;

  // FAIL 恢复：强制使用 FULL candidates（包含 fixed）
  init_stage_ = InitStage::FULL;
  buildInitCandidates_();

  auto poseToMat = [&](const InitialPoseManager::Pose2D &p) {
    return Pose2DToMat(p);
  };

  auto SrcStr = [](InitCandidate::Source s) -> const char * {
    switch (s) {
    case InitCandidate::Source::LAST: return "last";
    case InitCandidate::Source::LAST_SHUTDOWN: return "shutdown";
    case InitCandidate::Source::FIXED: return "fixed";
    default: return "unk";
    }
  };

  const bool can_local_seed =
      (cfg_.local_reloc_enable && local_reloc_ && local_reloc_->isReady());

  std::vector<Eigen::Vector2f> scan_xy;
  if (can_local_seed) {
    const int maxN = std::max(1, cfg_.local_opt_base.max_scan_points);
    const size_t N = cloud->points.size();
    const size_t step = (N > static_cast<size_t>(maxN)) ? (N / static_cast<size_t>(maxN)) : 1;

    scan_xy.reserve(std::min<size_t>(N, static_cast<size_t>(maxN)));
    for (size_t i = 0; i < N; i += step) {
      const auto &pt = cloud->points[i];
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
      scan_xy.emplace_back(pt.x, pt.y);
      if (static_cast<int>(scan_xy.size()) >= maxN) break;
    }
  }

  struct Best {
    bool ok = false;
    int grade = 0;
    double tp = -1e9;
    double score = 1e18;
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    InitCandidate cand;

    bool used_local_seed = false;

    bool used_dyn = false;
    DynamicFilterStats dyn_stats{};
  } best;

  int ndt_ok_cnt = 0;
  double max_tp_seen = std::numeric_limits<double>::quiet_NaN();
  double min_score_seen = std::numeric_limits<double>::quiet_NaN();

  for (size_t i = 0; i < init_candidates_.size(); ++i) {
    const auto &cand = init_candidates_[i];
    Eigen::Matrix4f T_seed = poseToMat(cand.pose);

    bool used_local = false;
    if (can_local_seed && !scan_xy.empty()) {
      RelocPose2D center{cand.pose.x, cand.pose.y, cand.pose.yaw};
      LocalRelocOptions opt = cfg_.local_opt_base;
      auto r = local_reloc_->match(scan_xy, center, opt);
      if (r.ok) {
        T_seed = r.T_seed;
        used_local = true;
      }
    }

    // 本候选内择优：dyn-first + 必要时 raw fallback
    bool cand_ok = false;
    int cand_grade = 0;
    double cand_tp = -1e9;
    double cand_score = 1e18;
    Eigen::Matrix4f cand_T = Eigen::Matrix4f::Identity();
    bool cand_used_dyn = false;
    DynamicFilterStats cand_ds{};

    auto run_align = [&](const PointCloudT::Ptr &cloud_in,
                         Eigen::Matrix4f *T_out,
                         double *score_out,
                         double *tp_out,
                         int *grade_out) -> bool {
      Eigen::Matrix4f T_ndt = T_seed;
      double score = std::numeric_limits<double>::max();
      const bool ok = ndt_matcher_.align(cloud_in, T_seed, T_ndt, score);
      if (!ok) return false;

      ndt_ok_cnt++;
      const double tp_raw = ndt_matcher_.getTransformationProbability();

      if (std::isfinite(tp_raw)) {
        max_tp_seen = std::isfinite(max_tp_seen) ? std::max(max_tp_seen, tp_raw) : tp_raw;
      }
      if (std::isfinite(score)) {
        min_score_seen = std::isfinite(min_score_seen) ? std::min(min_score_seen, score) : score;
      }

      const int grade =
          ComputeInitGrade(tp_raw, score,
                           cfg_.init_trans_prob_min,
                           cfg_.init_tp_soft,
                           cfg_.init_score_soft_enable,
                           cfg_.init_score_max,
                           cfg_.init_score_relaxed_max);

      if (T_out) *T_out = T_ndt;
      if (score_out) *score_out = score;
      if (tp_out) *tp_out = tp_raw;
      if (grade_out) *grade_out = grade;
      return true;
    };

    // ---- 1) dyn-first ----
    const bool dyn_ready = (dyn_cfg_.enable && dyn_filter_.isReady());
    PointCloudT::Ptr cloud_dyn = cloud;
    DynamicFilterStats ds{};
    bool dyn_effective = false;

    if (dyn_ready) {
      cloud_dyn = dyn_filter_.filter(cloud, T_seed, sensor_origin_base, &ds);
      if (!cloud_dyn || cloud_dyn->empty()) cloud_dyn = cloud;

      dyn_effective = (!ds.insufficient_static && ds.in > 0 && ds.kept > 0 && ds.kept < ds.in);
    }

    {
      Eigen::Matrix4f T_try;
      double score_try;
      double tp_try;
      int grade_try;

      const bool ok = run_align(cloud_dyn, &T_try, &score_try, &tp_try, &grade_try);
      if (ok && grade_try > 0) {
        cand_ok = true;
        cand_grade = grade_try;
        cand_score = std::isfinite(score_try) ? score_try : 1e18;
        cand_tp = std::isfinite(tp_try) ? tp_try : -1e9;
        cand_T = T_try;
        cand_used_dyn = dyn_ready;
        cand_ds = ds;
      }
    }

    // ---- 2) raw fallback：只有 dyn 确实生效但仍失败才回退 ----
    const bool need_raw_fallback = (dyn_ready && dyn_effective && !cand_ok);
    if (need_raw_fallback) {
      Eigen::Matrix4f T_try;
      double score_try;
      double tp_try;
      int grade_try;

      const bool ok = run_align(cloud, &T_try, &score_try, &tp_try, &grade_try);
      if (ok && grade_try > 0) {
        const double tp_eff = std::isfinite(tp_try) ? tp_try : -1e9;
        const double sc_eff = std::isfinite(score_try) ? score_try : 1e18;

        if (!cand_ok ||
            BetterPassNoPerturb(grade_try, sc_eff, tp_eff,
                                cand_grade, cand_score, cand_tp)) {
          cand_ok = true;
          cand_grade = grade_try;
          cand_score = sc_eff;
          cand_tp = tp_eff;
          cand_T = T_try;
          cand_used_dyn = false;
        }
      }
    }

    if (!cand_ok) continue;

    if (!best.ok ||
        BetterPassNoPerturb(cand_grade, cand_score, cand_tp,
                            best.grade, best.score, best.tp)) {
      best.ok = true;
      best.grade = cand_grade;
      best.tp = cand_tp;
      best.score = cand_score;
      best.T = cand_T;
      best.cand = cand;
      best.used_local_seed = used_local;

      best.used_dyn = cand_used_dyn;
      best.dyn_stats = cand_ds;
    }
  }

  if (!best.ok) {
    ROS_WARN_STREAM("[LOC] event=fail_fixedlast_try ok=0 cand="
                    << init_candidates_.size()
                    << " ndt_ok=" << ndt_ok_cnt
                    << " max_tp=" << FmtD(max_tp_seen, 3)
                    << " min_score=" << FmtD(min_score_seen, 6)
                    << " tp_min=" << FmtD(cfg_.init_trans_prob_min, 3)
                    << " score_max=" << FmtD(cfg_.init_score_max, 6)
                    << " score_relaxed=" << FmtD(cfg_.init_score_relaxed_max, 6));
    return false;
  }

  // accept: reset predictor & state
  predictor_.resetWithCorrection(best.T, last_odom_msg_);

  has_ever_had_good_ndt_ = true;
  dr_distance_since_last_ndt_ = 0.0;
  dr_frames_since_last_ndt_ = 0;
  loc_level_ = LocLevel::GOOD;

  stationary_phase_ = false;
  stationary_pose_written_ = false;

  if (cfg_.temporal_grace_enable) {
    temporal_grace_until_ = stamp + ros::Duration(cfg_.temporal_grace_duration);
  }

  T_last_published_ = best.T;
  t_last_published_ = stamp;
  has_last_published_ = true;

  T_last_temporal_ = best.T;
  t_last_temporal_ = stamp;
  has_last_temporal_ = true;

  last_good_pose_.x = best.T(0, 3);
  last_good_pose_.y = best.T(1, 3);
  last_good_pose_.yaw = std::atan2(best.T(1, 0), best.T(0, 0));
  has_last_good_pose_ = true;

  last_output_position_ = best.T.block<3, 1>(0, 3);
  has_last_output_position_ = true;
  degen_active_latched_ = false;
  degen_active_streak_ = 0;
  degen_hessian_miss_streak_ = 0;
  degen_hold_frames_left_ = 0;
  degen_exit_safe_streak_ = 0;
  has_degen_last_motion_dir_ = false;
  degen_last_motion_dir_ = Eigen::Vector2f(1.0f, 0.0f);

  if (best.cand.src == InitCandidate::Source::FIXED &&
      best.cand.fixed_index >= 0) {
    (void)initial_pose_mgr_.bumpFixedPriority(
        static_cast<size_t>(best.cand.fixed_index), 1);
  }

  publishOdom(stamp, best.T);
  {
    using S = localization_msgs::LocalizationStatus;
    NdtQualityMetrics q{};
    q.ndt_converged = true;
    q.score = best.score;
    q.trans_probability = best.tp;
    publishStatus(stamp, q, S::MODE_NDT, S::REASON_OK, true, true);
  }

  const double yaw_rad = std::atan2(best.T(1, 0), best.T(0, 0));
  ROS_WARN_STREAM("[LOC] event=fail_fixedlast_try ok=1 src="
                  << SrcStr(best.cand.src)
                  << " seed=" << (best.used_local_seed ? "local" : "raw")
                  << " name=" << best.cand.name
                  << " tp=" << FmtD(best.tp, 3)
                  << " score=" << FmtD(best.score, 3)
                  << " dyn=" << (best.used_dyn ? 1 : 0)
                  << (best.used_dyn
                          ? (std::string(" keep=") + std::to_string(best.dyn_stats.kept) +
                             "/" + std::to_string(best.dyn_stats.in) +
                             " insuff=" + std::to_string(best.dyn_stats.insufficient_static ? 1 : 0))
                          : std::string(""))
                  << " out(x=" << FmtD(best.T(0, 3), 3)
                  << " y=" << FmtD(best.T(1, 3), 3)
                  << " yaw=" << FmtD(yaw_rad, 2) << "rad)");

  return true;
}


// ===================== temporal gates =====================

bool NdtLocalizerNode::temporalCandidateMotionGate(
    const ros::Time &stamp, const Eigen::Matrix4f &T_candidate,
    bool ndt_internal_ok, bool do_check, double *out_dt, double *out_v_cand,
    double *out_v_odom, double *out_w_cand_deg, double *out_w_imu_deg,
    double *out_v_diff, double *out_w_diff_deg) {

  auto set_nan = [](double *p) {
    if (p)
      *p = std::numeric_limits<double>::quiet_NaN();
  };
  set_nan(out_dt);
  set_nan(out_v_cand);
  set_nan(out_v_odom);
  set_nan(out_w_cand_deg);
  set_nan(out_w_imu_deg);
  set_nan(out_v_diff);
  set_nan(out_w_diff_deg);

  // 只有 internal ok 才允许刷新 candidate cache
  if (!ndt_internal_ok)
    return true;

  bool ok = true;

  if (do_check && has_last_temporal_) {
    const double dt = (stamp - t_last_temporal_).toSec();
    if (out_dt)
      *out_dt = dt;

    if (dt >= kTemporalMinDt && dt <= cfg_.temporal_max_dt) {
      const Eigen::Vector3f p_last = T_last_temporal_.block<3, 1>(0, 3);
      const Eigen::Vector3f p_now = T_candidate.block<3, 1>(0, 3);
      const Eigen::Vector2f dp = (p_now - p_last).head<2>();
      const double v_cand = dp.norm() / dt;

      const double yaw_last =
          std::atan2(T_last_temporal_(1, 0), T_last_temporal_(0, 0));
      const double yaw_now = std::atan2(T_candidate(1, 0), T_candidate(0, 0));
      const double dyaw = NormalizeAngle(yaw_now - yaw_last);
      const double w_cand_deg = (dyaw / dt) * 180.0 / kPi;

      if (out_v_cand)
        *out_v_cand = v_cand;
      if (out_w_cand_deg)
        *out_w_cand_deg = w_cand_deg;

      Odom2D odom_prev, odom_now;
      const bool odom_interp_prev = QueryOdomPose(
          odom_buf_, t_last_temporal_, cfg_.temporal_odom_max_age, &odom_prev);
      const bool odom_interp_now = QueryOdomPose(
          odom_buf_, stamp, cfg_.temporal_odom_max_age, &odom_now);

      bool odom_ok_prev = odom_interp_prev;
      bool odom_ok_now = odom_interp_now;
      if (!odom_ok_prev) {
        odom_ok_prev = QueryOdomPoseClamped(
            odom_buf_, t_last_temporal_, cfg_.temporal_odom_max_age, &odom_prev);
      }
      if (!odom_ok_now) {
        odom_ok_now = QueryOdomPoseClamped(
            odom_buf_, stamp, cfg_.temporal_odom_max_age, &odom_now);
      }

      const bool odom_speed_reliable = odom_interp_prev && odom_interp_now;
      double v_vehicle = std::numeric_limits<double>::quiet_NaN();
      if (odom_speed_reliable && odom_ok_prev && odom_ok_now) {
        const double dx = odom_now.x - odom_prev.x;
        const double dy = odom_now.y - odom_prev.y;
        v_vehicle = std::hypot(dx, dy) / dt;
      }
      if (out_v_odom)
        *out_v_odom = v_vehicle;

      double wz = std::numeric_limits<double>::quiet_NaN();
      const bool imu_ok =
          QueryGyroZNearest(gyro_buf_, stamp, cfg_.temporal_imu_max_age, &wz);

      const double w_vehicle_deg =
          imu_ok ? (wz * 180.0 / kPi)
                 : std::numeric_limits<double>::quiet_NaN();
      if (out_w_imu_deg)
        *out_w_imu_deg = w_vehicle_deg;

      const double v_diff_local =
          (std::isfinite(v_vehicle) ? std::fabs(v_cand - v_vehicle)
                                    : std::numeric_limits<double>::quiet_NaN());
      if (out_v_diff)
        *out_v_diff = v_diff_local;

      const double w_diff_local =
          (std::isfinite(w_vehicle_deg)
               ? std::fabs(w_cand_deg - w_vehicle_deg)
               : std::numeric_limits<double>::quiet_NaN());
      if (out_w_diff_deg)
        *out_w_diff_deg = w_diff_local;

      // A1/A2: 仅在 odom 速度可可靠估计时启用。
      // 若 odom 只能夹取/最近邻（常见于输入时序抖动），
      // v_vehicle 可能被低估到 0，直接硬拒会造成 DR/NDT 抖动切换。
      if (odom_speed_reliable && std::isfinite(v_vehicle)) {
        if (v_vehicle > cfg_.temporal_move_min &&
            v_cand < cfg_.temporal_stuck_max) {
          ok = false;
        }
        if (ok && cfg_.temporal_jump_when_stop > 0.0 &&
            v_vehicle < cfg_.temporal_stop_max &&
            v_cand > cfg_.temporal_jump_when_stop) {
          ok = false;
        }
      }

      // A3/A4
      if (ok && std::isfinite(w_vehicle_deg)) {
        const double aw_imu = std::fabs(w_vehicle_deg);
        const double aw_cand = std::fabs(w_cand_deg);

        if (aw_imu > cfg_.temporal_ang_move_min_deg &&
            aw_cand < cfg_.temporal_ang_stuck_max_deg) {
          ok = false;
        }

        if (ok && cfg_.temporal_ang_jump_when_stop_deg > 0.0 &&
            aw_imu < cfg_.temporal_ang_stuck_max_deg &&
            aw_cand > cfg_.temporal_ang_jump_when_stop_deg) {
          ok = false;
        }
      }
    }
  }

  // Refresh temporal cache only when this candidate passes gate.
  if (ok) {
    T_last_temporal_ = T_candidate;
    t_last_temporal_ = stamp;
    has_last_temporal_ = true;
  }

  return ok;
}

bool NdtLocalizerNode::temporalJumpBoundsGate(
    const ros::Time &stamp, const Eigen::Matrix4f &T_candidate, bool do_check,
    double *out_dt, double *out_v_jump, double *out_w_jump_deg) const {

  auto set_nan = [](double *p) {
    if (p)
      *p = std::numeric_limits<double>::quiet_NaN();
  };
  set_nan(out_dt);
  set_nan(out_v_jump);
  set_nan(out_w_jump_deg);

  if (!do_check || !has_last_published_)
    return true;

  const double dt = (stamp - t_last_published_).toSec();
  if (out_dt)
    *out_dt = dt;

  if (dt < kTemporalMinDt || dt > cfg_.temporal_max_dt)
    return true;

  const Eigen::Vector3f p_last = T_last_published_.block<3, 1>(0, 3);
  const Eigen::Vector3f p_now = T_candidate.block<3, 1>(0, 3);

  const float yaw_last =
      std::atan2(T_last_published_(1, 0), T_last_published_(0, 0));
  const float yaw_now = std::atan2(T_candidate(1, 0), T_candidate(0, 0));

  const Eigen::Vector2f dp = (p_now - p_last).head<2>();
  const double v_jump = dp.norm() / dt;

  const double dyaw = NormalizeAngle(static_cast<double>(yaw_now - yaw_last));
  const double w_jump_deg = (dyaw / dt) * 180.0 / kPi;

  if (out_v_jump)
    *out_v_jump = v_jump;
  if (out_w_jump_deg)
    *out_w_jump_deg = w_jump_deg;

  if (cfg_.temporal_reject_backtrack_enable && has_prev_output_position_ &&
      has_last_output_position_ && std::isfinite(latest_linear_speed_) &&
      latest_linear_speed_ >= cfg_.temporal_reject_backtrack_min_speed_mps) {
    const Eigen::Vector2f d_prev =
        (last_output_position_ - prev_output_position_).head<2>();
    const double d_prev_norm = d_prev.norm();
    if (d_prev_norm >= cfg_.temporal_reject_backtrack_dir_min_step_m) {
      const Eigen::Vector2f dir_prev = d_prev / static_cast<float>(d_prev_norm);
      const double along = static_cast<double>(dp.dot(dir_prev));
      if (along < -cfg_.temporal_reject_backtrack_max_m) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[LOC] event=temp_backtrack_reject along="
                     << FmtD(along, 3)
                     << " lim=-" << FmtD(cfg_.temporal_reject_backtrack_max_m, 3)
                     << " prev_step=" << FmtD(d_prev_norm, 3)
                     << " v=" << FmtD(latest_linear_speed_, 2)
                     << " v_jump=" << FmtD(v_jump, 2));
        return false;
      }
    } else {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[LOC] event=temp_backtrack_skip prev_step="
                   << FmtD(d_prev_norm, 3)
                   << " need=" << FmtD(cfg_.temporal_reject_backtrack_dir_min_step_m, 3)
                   << " v=" << FmtD(latest_linear_speed_, 2));
    }
  }

  if (v_jump > cfg_.temporal_max_lin_vel)
    return false;
  if (std::fabs(w_jump_deg) > cfg_.temporal_max_ang_vel_deg)
    return false;

  return true;
}

// ===================== core scan =====================
void NdtLocalizerNode::scanCallback(
    const sensor_msgs::LaserScanConstPtr &scan_msg) {
  if (!algo_enabled_) {
    return;
  }

  using S = localization_msgs::LocalizationStatus;
  const ros::WallTime cb_t0 = ros::WallTime::now();
  static double last_scan_cb_ms = std::numeric_limits<double>::quiet_NaN();
  static uint32_t amb_strong_probe_rr = 0;

  // 0) stamp sanitize（用于发布）
  auto sanitize_stamp = [this](const ros::Time &msg_stamp) -> ros::Time {
    const ros::Time now = ros::Time::now();
    ros::Time s = msg_stamp.isZero() ? now : msg_stamp;

    if (now.isZero()) {
      if (has_last_published_ && !t_last_published_.isZero() &&
          s <= t_last_published_) {
        s = t_last_published_ + ros::Duration(cfg_.stamp_monotonic_eps_sec);
      }
      return s;
    }

    const double skew = std::fabs((now - s).toSec());
    if (skew > cfg_.stamp_max_skew_sec) {
      s = now;
    }

    if (has_last_published_ && !t_last_published_.isZero() &&
        s <= t_last_published_) {
      s = t_last_published_ + ros::Duration(cfg_.stamp_monotonic_eps_sec);
    }

    if (s > now + ros::Duration(cfg_.stamp_future_tol_sec)) {
      s = now;
      if (has_last_published_ && !t_last_published_.isZero() &&
          s <= t_last_published_) {
        s = t_last_published_ + ros::Duration(cfg_.stamp_monotonic_eps_sec);
      }
    }
    return s;
  };

  // --- 0) 统一使用“scan末端时刻(测量时间轴)”做 deskew 参考；发布仍用 sanitize 后的 stamp_pub ---
  const int N = static_cast<int>(scan_msg->ranges.size());

  // ===== scan timing / frequency debug (no behavior change) =====
{
  const double scan_time = scan_msg->scan_time;         // sec
  const double time_inc  = scan_msg->time_increment;    // sec

  const double total_by_inc =
      (N > 1 && time_inc > 0.0) ? (time_inc * (N - 1)) : 0.0;

  const double hz_by_scan_time =
      (scan_time > 1e-9) ? (1.0 / scan_time) : std::numeric_limits<double>::quiet_NaN();

  const double hz_by_inc =
      (total_by_inc > 1e-9) ? (1.0 / total_by_inc) : std::numeric_limits<double>::quiet_NaN();

  // 相邻帧 stamp 间隔（注意：stamp 可能为 0 或使用 sim_time）
  static ros::Time last_stamp_raw(0);
  static ros::Time last_arrival_ros(0);

  const ros::Time stamp_raw = scan_msg->header.stamp;
  const ros::Time now_ros   = ros::Time::now();

  double dt_stamp = std::numeric_limits<double>::quiet_NaN();
  double hz_stamp = std::numeric_limits<double>::quiet_NaN();
  if (!last_stamp_raw.isZero() && !stamp_raw.isZero()) {
    dt_stamp = (stamp_raw - last_stamp_raw).toSec();
    if (dt_stamp > 1e-9) hz_stamp = 1.0 / dt_stamp;
  }

  double dt_arr = std::numeric_limits<double>::quiet_NaN();
  double hz_arr = std::numeric_limits<double>::quiet_NaN();
  if (!last_arrival_ros.isZero() && !now_ros.isZero()) {
    dt_arr = (now_ros - last_arrival_ros).toSec();
    if (dt_arr > 1e-9) hz_arr = 1.0 / dt_arr;
  }

  // 一致性检查：scan_time vs total_by_inc
  double rel_err = std::numeric_limits<double>::quiet_NaN();
  if (scan_time > 1e-9 && total_by_inc > 1e-9) {
    rel_err = std::fabs(scan_time - total_by_inc) / std::max(scan_time, total_by_inc);
  }

  ROS_INFO_STREAM_THROTTLE(
      1.0,
      "[LOC] scan_timing N=" << N
      << " scan_time=" << FmtD(scan_time, 6) << "s (hz=" << FmtD(hz_by_scan_time, 2) << ")"
      << " time_inc=" << FmtD(time_inc, 9) << "s"
      << " total_by_inc=" << FmtD(total_by_inc, 6) << "s (hz=" << FmtD(hz_by_inc, 2) << ")"
      << " dt_stamp=" << FmtD(dt_stamp, 6) << "s (hz=" << FmtD(hz_stamp, 2) << ")"
      << " dt_arr=" << FmtD(dt_arr, 6) << "s (hz=" << FmtD(hz_arr, 2) << ")"
      << " rel_err(scan_time vs inc)=" << FmtD(rel_err, 3));
  
  if (std::isfinite(rel_err) && rel_err > 0.2) {
    ROS_WARN_STREAM_THROTTLE(
        1.0,
        "[LOC] scan_time and time_increment*(N-1) mismatch: scan_time="
            << FmtD(scan_time, 6) << " total_by_inc=" << FmtD(total_by_inc, 6)
            << " rel_err=" << FmtD(rel_err, 3));
  }
  const double gap = (std::isfinite(dt_stamp) ? (dt_stamp - total_by_inc) : std::numeric_limits<double>::quiet_NaN());
  ROS_INFO_STREAM_THROTTLE(1.0, "[LOC] scan_timing gap_period_minus_scan=" << FmtD(gap, 6) << "s");

  last_stamp_raw = stamp_raw;
  last_arrival_ros = now_ros;
}
// ===== end debug =====

  // 推断本帧持续时间与每束时间增量
  double dt_inc = scan_msg->time_increment;  // 每束时间
  double total  = 0.0;                       // 整帧时间
  if (N > 1) {
    if (dt_inc > 0.0) {
      total = dt_inc * (N - 1);
    } else if (scan_msg->scan_time > 0.0) {
      total  = scan_msg->scan_time;
      dt_inc = total / (N - 1);
    } else {
      // driver 没给时间信息：退化为“无时序”，仍走 deskew 管线但等价于不补偿
      dt_inc = 0.0;
      total  = 0.0;
    }
  } else {
    dt_inc = 0.0;
    total  = 0.0;
  }

  // Measurement time axis 
  ros::Time t_end_meas = scan_msg->header.stamp;
  if (t_end_meas.isZero()) {
    t_end_meas = ros::Time::now();
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] scan stamp is zero -> use now");
  }
  t_end_meas += ros::Duration(total);
  const ros::Time t_start_meas = t_end_meas - ros::Duration(total);

  // Publish / gate time axis (sanitized, monotonic)
  const ros::Time stamp_pub = sanitize_stamp(t_end_meas);

  // 如果 sanitize 发生了明显跳变，提示一下（deskew 仍按 meas 做）
  const double pub_meas_skew = std::fabs((stamp_pub - t_end_meas).toSec());
  if (pub_meas_skew > 0.05) {  // 50ms 只是提示阈值，可按需改
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] stamp_pub skew from meas end = "
                                      << FmtD(pub_meas_skew, 3) << "s");
  }

  // watchdog: record scan arrival (use publish time)
  has_last_scan_ = true;
  last_scan_stamp_ = stamp_pub;

  // watchdog: check other inputs BEFORE heavy work
  if (cfg_.watchdog_enable) {
    uint8_t wd_reason = 0;
    std::string wd_detail;
    if (!watchdogCheck_(stamp_pub, &wd_reason, &wd_detail)) {
      forceFail_(stamp_pub, wd_reason, wd_detail);
      return;
    } else {
      clearForceFailIfRecovered_(stamp_pub);
    }
  }

  // --- 1) 取 base<-laser 外参（2D: x,y,yaw），同时得到 sensor_origin_base ---
  Eigen::Vector2f sensor_origin_base(0.f, 0.f);
  Eigen::Matrix3d T_bl = Eigen::Matrix3d::Identity();   // base <- laser (SE2)

  auto quat_to_yaw = [](const geometry_msgs::Quaternion &qmsg) -> double {
    Eigen::Quaterniond q(qmsg.w, qmsg.x, qmsg.y, qmsg.z);
    q.normalize();
    return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                      1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  };

  bool have_tf_bl = false;
  geometry_msgs::TransformStamped tf_bl;

  // 外参通常是静态的，优先用 time=0 避免 TF buffer 没覆盖某个时间点
  try {
    tf_bl = tf_buffer_.lookupTransform(base_frame_id_, scan_msg->header.frame_id,
                                       ros::Time(0), ros::Duration(0.01));
    have_tf_bl = true;
  } catch (...) {
    // 退而求其次：用 stamp_pub
    try {
      tf_bl = tf_buffer_.lookupTransform(base_frame_id_, scan_msg->header.frame_id,
                                         stamp_pub, ros::Duration(0.01));
      have_tf_bl = true;
    } catch (const std::exception &e) {
      have_tf_bl = false;
      dtc_pub_.Event(2, 3014, 1.5);
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[LOC] tf base<-laser lookup failed -> assume identity: " << e.what());
    }
  }

  if (have_tf_bl) {
    const double x = tf_bl.transform.translation.x;
    const double y = tf_bl.transform.translation.y;

    geometry_msgs::Quaternion qmsg;
    qmsg.x = tf_bl.transform.rotation.x;
    qmsg.y = tf_bl.transform.rotation.y;
    qmsg.z = tf_bl.transform.rotation.z;
    qmsg.w = tf_bl.transform.rotation.w;
    const double yaw = quat_to_yaw(qmsg);

    T_bl = SE2(x, y, yaw);
    sensor_origin_base.x() = static_cast<float>(x);
    sensor_origin_base.y() = static_cast<float>(y);
  }

  // --- 2) LaserScan -> PCL (base@t_end_meas) + deskew（始终开启）
  //     deskew: odom 平移 + IMU yaw 增量（wz积分）
  PointCloudT::Ptr cloud(new PointCloudT);
  cloud->reserve(std::max(1, N));

  const double odom_max_age = cfg_.temporal_odom_max_age;
  const double imu_max_age  = cfg_.temporal_imu_max_age;

  // 1) 参考 odom：用 t_end_meas 做 ref（不要用 stamp_pub）
  Odom2D od_ref;
  bool have_ref_odom =
      QueryOdomPoseClamped(odom_buf_, t_end_meas, odom_max_age, &od_ref);
  if (have_ref_odom) {
    od_ref.t = t_end_meas;
  }

  if (!have_ref_odom && has_last_odom_msg_) {
    const auto &p = last_odom_msg_.pose.pose.position;
    const double yaw = quat_to_yaw(last_odom_msg_.pose.pose.orientation);
    od_ref.t = t_end_meas;
    od_ref.x = p.x;
    od_ref.y = p.y;
    od_ref.yaw = NormalizeAngle(yaw);
    have_ref_odom = true;
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] deskew: odom_buf miss ref -> use last_odom_msg pose");
  }

  if (!have_ref_odom) {
    od_ref.t = t_end_meas;
    od_ref.x = 0.0;
    od_ref.y = 0.0;
    od_ref.yaw = 0.0;
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] deskew: no odom -> use identity ref");
  }

  const double yaw_ref = od_ref.yaw;
  const double c_ref = std::cos(yaw_ref);
  const double s_ref = std::sin(yaw_ref);

  auto RotMinusYawRef = [&](double dx, double dy) -> Eigen::Vector2d {
    // R(-yaw_ref) * [dx, dy]
    return Eigen::Vector2d(c_ref * dx + s_ref * dy,
                          -s_ref * dx + c_ref * dy);
  };

  // 2) 预计算每束的 yaw 积分：cum_yaw[i] = ∫_{t_start_meas}^{t_i} wz dt
  //    total_yaw = ∫_{t_start_meas}^{t_end_meas} wz dt
  //    delta_yaw_i = cum_yaw[i] - total_yaw = ∫_{t_end_meas}^{t_i} wz dt
  std::vector<double> cum_yaw(std::max(1, N), 0.0);

  if (N > 1 && dt_inc > 0.0) {
    size_t hint = 0;  // 加速插值查找（t_i 单调递增）
    auto get_wz = [&](const ros::Time &t, double *wz) -> bool {
      if (QueryGyroZInterpClamped(gyro_buf_, t, imu_max_age, wz, &hint)) return true;
      return QueryGyroZNearest(gyro_buf_, t, imu_max_age, wz);
    };

    double wz_prev = 0.0;
    if (!get_wz(t_start_meas, &wz_prev)) {
      wz_prev = 0.0;
      ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] deskew: no imu at scan start -> wz=0");
    }

    for (int i = 1; i < N; ++i) {
      const ros::Time t_i = t_start_meas + ros::Duration(dt_inc * i);

      double wz_i = wz_prev;
      if (!get_wz(t_i, &wz_i)) {
        wz_i = wz_prev;  // piecewise constant fallback
      }

      // trapezoid integrate
      cum_yaw[i] = cum_yaw[i - 1] + 0.5 * (wz_prev + wz_i) * dt_inc;
      wz_prev = wz_i;
    }
  }

  const double total_yaw = (N > 0) ? cum_yaw[N - 1] : 0.0;

  // 3) 逐束生成 deskew 后点（表达在 base_ref = base@t_end_meas）
  const bool has_scan_intensity =
      (scan_msg->intensities.size() == static_cast<size_t>(N));
  if (!scan_msg->intensities.empty() && !has_scan_intensity) {
    ROS_WARN_STREAM_THROTTLE(
        2.0, "[LOC] scan intensity size mismatch, ranges="
                 << N << " intensities=" << scan_msg->intensities.size()
                 << " -> fallback raw_intensity=0");
  }

  for (int i = 0; i < N; ++i) {
    const float r = scan_msg->ranges[i];
    if (!std::isfinite(r)) continue;
    if (r < scan_msg->range_min || r > scan_msg->range_max) continue;

    const double ang = scan_msg->angle_min +
                       static_cast<double>(i) * scan_msg->angle_increment;

    // beam 测量时间（meas time axis）
    ros::Time t_i = t_end_meas;
    if (N > 1 && dt_inc > 0.0) {
      t_i = t_start_meas + ros::Duration(dt_inc * i);
    }

    // ---- 平移：用 odom 的 x/y（yaw 不用来累积旋转，只用于把 dp 转到 base_ref）----
    Odom2D od_i = od_ref;
    (void)QueryOdomPoseClamped(odom_buf_, t_i, odom_max_age, &od_i);

    const double dx_o = (od_i.x - od_ref.x);
    const double dy_o = (od_i.y - od_ref.y);

    // dp in base_ref
    const Eigen::Vector2d dp_bref = RotMinusYawRef(dx_o, dy_o);

    // ---- 旋转：用 IMU yaw 增量（相对 t_end_meas）----
    const double delta_yaw = (N > 0) ? (cum_yaw[i] - total_yaw) : 0.0;
    const double c = std::cos(delta_yaw);
    const double s = std::sin(delta_yaw);

    // 点在 laser frame
    const double xl = r * std::cos(ang);
    const double yl = r * std::sin(ang);

    // base_i <- laser（外参）
    const Eigen::Vector3d p_l(xl, yl, 1.0);
    const Eigen::Vector3d p_bi = T_bl * p_l;  // [x, y, 1] in base_i

    // 旋转到 base_ref：R(delta_yaw)*p_bi_xy
    const double x_rot = c * p_bi.x() - s * p_bi.y();
    const double y_rot = s * p_bi.x() + c * p_bi.y();

    // 加上平移补偿
    PointT pt;
    pt.x = static_cast<float>(x_rot + dp_bref.x());
    pt.y = static_cast<float>(y_rot + dp_bref.y());
    pt.z = 0.f;
    float raw_intensity = 0.f;
    if (has_scan_intensity) {
      raw_intensity = scan_msg->intensities[i];
      if (!std::isfinite(raw_intensity) || raw_intensity < 0.f)
        raw_intensity = 0.f;
    }
    pt.intensity = raw_intensity;
    cloud->push_back(pt);
  }

  if (!cloud || cloud->empty()) {
    dtc_pub_.Event(2, 3016, 1.5);
    ROS_WARN_THROTTLE(1.0, "[LOC] deskewed cloud is empty.");
    return;
  }

  PointCloudT::Ptr cloud_used = cloud;
  
  // 2.5) range filter
  if (cfg_.scan_min_range > 0.0 || cfg_.scan_max_range > 0.0) {
    const float min_r2 =
        static_cast<float>(cfg_.scan_min_range * cfg_.scan_min_range);
    const float max_r2 =
        (cfg_.scan_max_range > 0.0)
            ? static_cast<float>(cfg_.scan_max_range * cfg_.scan_max_range)
            : std::numeric_limits<float>::infinity();

    PointCloudT::Ptr filtered(new PointCloudT);
    filtered->reserve(cloud->size());

    for (const auto &pt : cloud->points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
        continue;
      const float r2 = pt.x * pt.x + pt.y * pt.y;
      if (r2 < min_r2 || r2 > max_r2)
        continue;
      filtered->push_back(pt);
    }
    if (filtered->empty()) {
      dtc_pub_.Event(2, 3016, 1.5);
      ROS_WARN_THROTTLE(
          1.0, "All points removed by range filter. min=%.2f, max=%.2f",
          cfg_.scan_min_range, cfg_.scan_max_range);
      return;
    }
    cloud.swap(filtered);
    // for viz & ndt input (will be overwritten by dyn filter in normal flow)
    cloud_used = cloud;
  }

  // 2.6) intensity prefilter: distance compensation + keep top ratio
  if (cfg_.intensity_prefilter_enable && cloud && !cloud->empty()) {
    const bool degen_scene =
        degen_active_latched_ || degen_dr_hold_active_ || degen_dr_cruise_active_;
    const bool adaptive_relax_window =
        cfg_.intensity_prefilter_adaptive_enable &&
        (intensity_prefilter_adaptive_frames_left_ > 0);
    const bool run_intensity_prefilter =
        degen_scene;

    if (!run_intensity_prefilter) {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[LOC] intensity_core used=0 reason=normal_scene"
                   << " degen=" << (degen_scene ? 1 : 0)
                   << " ar=" << (adaptive_relax_window ? 1 : 0)
                   << " al=" << intensity_prefilter_adaptive_frames_left_);
    } else {
    const double comp_p = std::max(0.0, cfg_.intensity_prefilter_range_comp_p);
    const double norm_min = std::max(0.0, cfg_.intensity_prefilter_norm_min);
    const double norm_max =
        std::max(norm_min + 1e-6, cfg_.intensity_prefilter_norm_max);
    const double keep_ratio_base =
        std::max(0.0, std::min(1.0, cfg_.intensity_prefilter_keep_ratio));
    const double keep_ratio_relaxed =
        std::max(keep_ratio_base,
                 std::max(0.0, std::min(1.0,
                                         cfg_.intensity_prefilter_adaptive_keep_ratio)));
    const bool adaptive_relax =
        cfg_.intensity_prefilter_adaptive_enable &&
        (intensity_prefilter_adaptive_frames_left_ > 0);
    const int adaptive_left_before = intensity_prefilter_adaptive_frames_left_;
    const double keep_ratio = adaptive_relax ? keep_ratio_relaxed : keep_ratio_base;

    std::vector<float> norm_vals;
    norm_vals.reserve(cloud->size());
    for (const auto &pt : cloud->points) {
      double raw_i =
          std::isfinite(pt.intensity) ? std::max(0.0, static_cast<double>(pt.intensity)) : 0.0;
      const double r = std::hypot(static_cast<double>(pt.x), static_cast<double>(pt.y));
      if (comp_p > 0.0) {
        raw_i *= std::pow(std::max(1e-3, r), comp_p);
      }
      if (!std::isfinite(raw_i)) {
        raw_i = norm_min;
      }
      raw_i = std::max(norm_min, std::min(norm_max, raw_i));
      norm_vals.push_back(static_cast<float>(raw_i));
    }

    auto percentile_value = [&](double q) -> double {
      if (norm_vals.empty()) {
        return norm_min;
      }
      q = std::max(0.0, std::min(1.0, q));
      std::vector<float> tmp = norm_vals;
      const size_t k = std::min(
          tmp.size() - 1,
          static_cast<size_t>(q * static_cast<double>(tmp.size() - 1)));
      std::nth_element(tmp.begin(), tmp.begin() + k, tmp.end());
      return static_cast<double>(tmp[k]);
    };

    const double p50 = percentile_value(0.50);
    const double p90 = percentile_value(0.90);

    double thr = norm_min;
    if (keep_ratio <= 0.0) {
      thr = std::numeric_limits<double>::infinity();
    } else if (keep_ratio < 1.0) {
      thr = percentile_value(1.0 - keep_ratio);
    }

    PointCloudT::Ptr filtered(new PointCloudT);
    filtered->reserve(cloud->size());
    for (size_t i = 0; i < cloud->size(); ++i) {
      if (static_cast<double>(norm_vals[i]) < thr)
        continue;
      PointT pt = cloud->points[i];
      pt.intensity = norm_vals[i];
      filtered->push_back(pt);
    }

    const size_t keep_count = filtered->size();
    bool fallback = false;
    const size_t min_keep =
        std::min(cloud->size(),
                 static_cast<size_t>(std::max(1, cfg_.intensity_prefilter_min_keep_points)));
    if (keep_count < min_keep) {
      fallback = true;
    } else {
      cloud.swap(filtered);
      cloud_used = cloud;
    }

    if (adaptive_relax && intensity_prefilter_adaptive_frames_left_ > 0) {
      --intensity_prefilter_adaptive_frames_left_;
    }

    ROS_INFO_STREAM_THROTTLE(
        1.0, "[LOC] intensity_core used=1 keep="
                 << keep_count << "/" << norm_vals.size()
                 << " ratio="
                 << FmtD(norm_vals.empty()
                             ? 0.0
                             : static_cast<double>(keep_count) /
                                   static_cast<double>(norm_vals.size()),
                         2)
                 << " thr=" << FmtD(thr, 2)
                 << " p50=" << FmtD(p50, 2)
                 << " p90=" << FmtD(p90, 2)
                 << " fallback=" << (fallback ? 1 : 0)
                 << " ar=" << (adaptive_relax ? 1 : 0)
                 << " al=" << adaptive_left_before
                 << "->" << intensity_prefilter_adaptive_frames_left_
                 << " keep_cfg=" << FmtD(keep_ratio_base, 2)
                 << " keep_run=" << FmtD(keep_ratio, 2));
    }
  }

  // viz: publish scan_cloud_map
  auto publish_scan_cloud_map = [&](const Eigen::Matrix4f &T_map_base_viz) {
    if (!cfg_.viz_publish_scan_cloud || !scan_cloud_viz_pub_)
      return;

    if (!cloud_used || cloud_used->empty())
      return;

    PointCloudT cloud_map;
    cloud_map.reserve(cloud_used->size());
    pcl::transformPointCloud(*cloud_used, cloud_map, T_map_base_viz);

    sensor_msgs::PointCloud2 cloud_map_msg;
    pcl::toROSMsg(cloud_map, cloud_map_msg);
    cloud_map_msg.header.stamp = stamp_pub;
    cloud_map_msg.header.frame_id = "map";
    scan_cloud_viz_pub_.publish(cloud_map_msg);
  };

  // 3) INIT stage
  if (!initial_pose_applied_) {
    Eigen::Matrix4f T_boot = Eigen::Matrix4f::Identity();

    if (initial_pose_mgr_.hasShutdownPose()) {
      T_boot = Pose2DToMat(initial_pose_mgr_.shutdownPose());
    } else if (initial_pose_mgr_.hasLastPose()) {
      T_boot = Pose2DToMat(initial_pose_mgr_.lastPose());
    } else if (!predictor_.predict(T_boot)) {
      T_boot = Eigen::Matrix4f::Identity();
    }

    publishOdom(stamp_pub, T_boot);
    publish_scan_cloud_map(T_boot);

    T_last_published_ = T_boot;
    t_last_published_ = stamp_pub;
    has_last_published_ = true;

    T_last_temporal_ = T_boot;
    t_last_temporal_ = stamp_pub;
    has_last_temporal_ = true;

    const bool init_ok =
        tryInitializeFromCandidates_(stamp_pub, cloud, sensor_origin_base);
    if (!init_ok) {
      NdtQualityMetrics q_init{};
      publishStatus(stamp_pub, q_init, S::MODE_INITIALIZING, S::REASON_OK,
                    false, true);
      return;
    }
  }

  // 4) normal flow
  const bool in_grace = isInTemporalGrace(stamp_pub);
  const bool do_temporal = cfg_.temporal_check_enable && !in_grace;

  // Ambiguity probing is expensive (extra NDT aligns).
  // If callback already overloaded or publish/measurement skew is large,
  // skip probing for this frame to protect realtime input freshness.
  const bool amb_probe_overloaded =
      std::isfinite(last_scan_cb_ms) && (last_scan_cb_ms > 35.0);
  const bool amb_probe_skewed = (pub_meas_skew > 0.06);
  const bool amb_probe_forced_dr = degen_dr_hold_active_ || degen_dr_cruise_active_;
  const bool amb_probe_skip =
      amb_probe_overloaded || amb_probe_skewed || amb_probe_forced_dr;

  bool ndt_internal_ok = false;
  bool innovation_ok = false;
  bool ndt_quality_ok = false;

  bool temporalA_ok = true;
  bool temporalB_ok = true;

  double dtA = std::numeric_limits<double>::quiet_NaN();
  double v_cand = std::numeric_limits<double>::quiet_NaN();
  double v_odom = std::numeric_limits<double>::quiet_NaN();
  double w_cand_deg = std::numeric_limits<double>::quiet_NaN();
  double w_imu_deg = std::numeric_limits<double>::quiet_NaN();
  double v_diff = std::numeric_limits<double>::quiet_NaN();
  double w_diff_deg = std::numeric_limits<double>::quiet_NaN();

  double dtB = std::numeric_limits<double>::quiet_NaN();
  double v_jump = std::numeric_limits<double>::quiet_NaN();
  double w_jump_deg = std::numeric_limits<double>::quiet_NaN();

  uint8_t reason_code = S::REASON_OK;

  // FAIL recovery scheduler (keep your original logic)
  const bool was_fail = (loc_level_ == LocLevel::FAIL);
  const bool is_parking_now =
      (latest_linear_speed_ < cfg_.stationary_speed_thresh);

  if (!sensor_force_fail_ && was_fail && has_last_good_pose_ &&
      has_last_odom_msg_) {
    const Eigen::Vector2f odom_xy(
        static_cast<float>(last_odom_msg_.pose.pose.position.x),
        static_cast<float>(last_odom_msg_.pose.pose.position.y));

    if (has_fail_anchor_) {
      const double moved_from_edge = (odom_xy - fail_anchor_odom_xy_).norm();
      if (moved_from_edge >= cfg_.init_rearm_dist) {
        fail_moved_far_since_edge_ = true;
      }
    }

    if (!is_parking_now) {
      fail_stationary_start_ = ros::Time(0);
    } else {
      if (fail_stationary_start_.isZero())
        fail_stationary_start_ = stamp_pub;

      const double stop_dt = (stamp_pub - fail_stationary_start_).toSec();
      const bool stop_ok = (stop_dt >= cfg_.init_rearm_min_stop_duration);

      if (!fail_moved_far_since_edge_) {
        const double kPeriodicSec = 2.0;
        const double dt_period =
            fail_last_periodic_local_.isZero()
                ? 1e9
                : (stamp_pub - fail_last_periodic_local_).toSec();

        if (dt_period >= kPeriodicSec) {
          RelocPose2D center = last_good_pose_;

          if (has_map_odom_anchor_) {
            Odom2D od_now;
            if (QueryOdomPose(odom_buf_, stamp_pub, cfg_.temporal_odom_max_age,
                              &od_now)) {
              const Eigen::Matrix3d T_ob_now =
                  SE2(od_now.x, od_now.y, od_now.yaw);
              const Eigen::Matrix3d T_mb_dr = T_map_odom_anchor_ * T_ob_now;

              center.x = T_mb_dr(0, 2);
              center.y = T_mb_dr(1, 2);
              center.yaw = std::atan2(T_mb_dr(1, 0), T_mb_dr(0, 0));
            }
          }

          reloc_mgr_.onDrFail(center);

          fail_last_periodic_local_ = stamp_pub;

          ROS_WARN_STREAM("[LOC] event=fail_periodic_local schedule "
                          << "center(x=" << FmtD(last_good_pose_.x, 3)
                          << " y=" << FmtD(last_good_pose_.y, 3)
                          << " yaw=" << FmtD(last_good_pose_.yaw, 2) << "rad)");
        }
      } else {
        if (stop_ok) {
          double moved_from_attempt = std::numeric_limits<double>::infinity();
          if (has_fail_attempt_anchor_) {
            moved_from_attempt =
                (odom_xy - fail_attempt_anchor_odom_xy_).norm();
          }

          if (moved_from_attempt >= cfg_.init_rearm_dist) {
             if (tryFixedLastRelocInFail_(stamp_pub, cloud, sensor_origin_base)) {
              dtc_pub_.Event(0, 3017, 2.0);  // 3017 relocalized success

              fail_stationary_start_ = ros::Time(0);
              fail_last_periodic_local_ = ros::Time(0);
              has_fail_anchor_ = false;
              has_fail_attempt_anchor_ = false;
              fail_moved_far_since_edge_ = false;

              prev_loc_level_ = loc_level_;
              return;
            } else {
              dtc_pub_.Event(2, 3018, 2.0);  // 3018 relocalize failed
            }

            fail_attempt_anchor_odom_xy_ = odom_xy;
            has_fail_attempt_anchor_ = true;
          }
        }
      }
    }
  }

  // predict
  Eigen::Matrix4f T_pred = Eigen::Matrix4f::Identity();
  bool has_pred = predictor_.predict(T_pred);
  if (!has_pred) {
    ROS_WARN_THROTTLE(5.0,
                      "Predictor not ready yet. Using Identity as init guess.");
    T_pred = Eigen::Matrix4f::Identity();
  }

  // local seed
  Eigen::Matrix4f T_init = T_pred;
  bool used_local_seed = false;
  bool used_manual_local_seed = false;

  if (cfg_.local_reloc_enable && local_reloc_ && local_reloc_->isReady()) {
    RelocHint hint;
    bool hint_peeked = false;
    if (reloc_mgr_.peek(stamp_pub, &hint)) {
      hint_peeked = true;

      std::vector<Eigen::Vector2f> scan_xy;
      scan_xy.reserve(cloud->size());
      for (const auto &pt : cloud->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
          continue;
        scan_xy.emplace_back(pt.x, pt.y);
      }

      LocalRelocOptions opt = cfg_.local_opt_base;
      opt.xy_range_m = hint.xy_range_m;
      opt.yaw_range_rad = hint.yaw_range_rad;
      if (hint.override_score_margin >= 0.0)
        opt.score_margin = hint.override_score_margin;

      auto r = local_reloc_->match(scan_xy, hint.center, opt);

      if (r.ok) {
        T_init = r.T_seed;
        used_local_seed = true;
        used_manual_local_seed = (hint.trig == RelocTrigger::MANUAL);

        ROS_INFO_STREAM("[LOC] event=local_seed ok=1 trig="
                        << int(hint.trig) << " best=" << FmtD(r.best_score, 3)
                        << " second=" << FmtD(r.second_score, 3)
                        << " vf=" << FmtD(r.valid_fraction, 2)
                        << " out(x=" << FmtD(r.best_pose.x, 3)
                        << " y=" << FmtD(r.best_pose.y, 3)
                        << " yaw=" << FmtD(r.best_pose.yaw, 2) << "rad)");
      } else {
        ROS_WARN_STREAM("[LOC] event=local_seed ok=0 trig=" << int(hint.trig));
      }
    }
  }

  // ===== dynamic filter (core layer) BEFORE NDT =====
  DynamicFilterStats ds;
  cloud_used = cloud; // default to raw

  if (dyn_cfg_.enable && dyn_filter_.isReady()) {
    // Use precomputed sensor_origin_base from scanCallback.
    auto out = dyn_filter_.filter(cloud, T_init, sensor_origin_base, &ds);
    if (out && !out->empty()) {
      cloud_used = out;
    } else {
      cloud_used = cloud; // fallback to avoid empty cloud
    }

    const bool dyn_use_raw = (ds.insufficient_static || ds.kept >= ds.in);
    if (dyn_cfg_.front_protect_enable && ds.in > 200 && ds.front_in == 0) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "[LOC] event=dyn_front_empty in=" << ds.in
                   << " front=0 axis="
                   << (dyn_cfg_.front_protect_use_y_axis ? "y" : "x")
                   << " sign=" << FmtD(dyn_cfg_.front_protect_forward_sign, 1));
    }

    ROS_INFO_STREAM_THROTTLE(
        1.0, "[LOC] dyn_core used=1 keep="
                 << ds.kept << "/" << ds.in << " ratio="
                 << FmtD(ds.keep_ratio, 2) << " p50=" << FmtD(ds.dist_p50, 2)
                 << " p90=" << FmtD(ds.dist_p90, 2)
                 << " ray=" << ds.ray_rejected << "/" << ds.ray_tested
                 << " rmap50=" << FmtD(ds.rmap_p50, 2)
                 << " rmap90=" << FmtD(ds.rmap_p90, 2)
                 << " front=" << ds.front_kept << "/" << ds.front_in
                 << " fdrop=" << FmtD(ds.front_drop_ratio, 2)
                 << " frec=" << (ds.front_recovered ? 1 : 0)
                 << " use_raw=" << (dyn_use_raw ? 1 : 0)
                 << " insuff=" << (ds.insufficient_static ? 1 : 0));
  }

  // NDT
  Eigen::Matrix4f T_ndt = T_init;
  double score = std::numeric_limits<double>::max();

  const auto t0 = ros::WallTime::now();
  bool ndt_ok = ndt_matcher_.align(cloud_used, T_init, T_ndt, score);
  const double ms = (ros::WallTime::now() - t0).toSec() * 1000.0;
  ROS_INFO_STREAM_THROTTLE(1.0, "[LOC] ndt_align_ms=" << FmtD(ms, 2));

  double trans_prob = std::numeric_limits<double>::quiet_NaN();
  if (ndt_ok)
    trans_prob = ndt_matcher_.getTransformationProbability();

  NdtQualityMetrics q_status{};
  if (ndt_ok) {
    // 注意：这里必须用纯预测 T_pred，不能用 T_init（可能被 local seed 改过）
    q_status = evaluateNdtQuality(score, trans_prob, T_pred, T_ndt, true);
  } else {
    q_status.score = score;
    q_status.trans_probability = trans_prob;
    q_status.ndt_converged = false;
  }

  Eigen::Matrix4f T_output = T_pred; // 默认输出纯预测（DR 连续）
  uint8_t mode_code = S::MODE_FOLLOWING_DR;
  bool using_prediction_frame = true;

  bool temporal_ok_flag = true;
  bool temporal_checked = false;

  bool ndt_converged_flag = ndt_ok;
  bool quality_good_for_last_pose = false;

  bool degen_valid = false;
  bool degen_active = false;
  bool degen_yaw_weak = false;
  bool degen_ambiguity_high = false;

  double degen_cond = std::numeric_limits<double>::quiet_NaN();
  double degen_lambda_min = std::numeric_limits<double>::quiet_NaN();
  double degen_lambda_max = std::numeric_limits<double>::quiet_NaN();

  double degen_alpha_w = 1.0;
  double degen_alpha_s = 1.0;
  double degen_alpha_yaw = 1.0;

  double degen_amb_score_gap = std::numeric_limits<double>::quiet_NaN();
  double degen_amb_tp_gap = std::numeric_limits<double>::quiet_NaN();
  double degen_amb_weak_sep = std::numeric_limits<double>::quiet_NaN();
  double degen_amb_strong_sep = std::numeric_limits<double>::quiet_NaN();
  double degen_amb_probe_ms = std::numeric_limits<double>::quiet_NaN();
  int degen_amb_probe_runs = 0;

  double degen_pred_dw_abs = std::numeric_limits<double>::quiet_NaN();
  double degen_pred_ds_abs = std::numeric_limits<double>::quiet_NaN();
  double degen_pred_dyaw_abs = std::numeric_limits<double>::quiet_NaN();
  bool degen_close_to_pred = false;
  double degen_weak_motion_align = std::numeric_limits<double>::quiet_NaN();
  bool degen_force_when_close = false;

  Eigen::Vector2f degen_weak_dir(1.0f, 0.0f);
  Eigen::Vector2f degen_strong_dir(0.0f, 1.0f);
  bool degen_hard = false;
  bool degen_force_dr_no_hessian = false;
  bool degen_dr_hold_forced = false;
  bool degen_dr_hold_state_updated = false;
  bool degen_dr_cruise_forced = false;
  bool degen_dr_cruise_state_updated = false;
  bool degen_state_updated = false;
  bool degen_hessian_observed_frame = false;

  auto updateDegenDrHold = [&](bool trigger_obs, bool recover_obs) -> bool {
    degen_dr_hold_state_updated = true;

    if (!cfg_.degen_dr_hold_enable) {
      degen_dr_hold_active_ = false;
      degen_dr_hold_frames_left_ = 0;
      degen_dr_hold_trigger_count_ = 0;
      degen_dr_hold_recover_count_ = 0;
      return false;
    }

    const int trig_need = std::max(1, cfg_.degen_dr_hold_trigger_frames);
    const int hold_need = std::max(1, cfg_.degen_dr_hold_frames);
    const int recover_need = std::max(1, cfg_.degen_dr_hold_recover_frames);

    if (degen_dr_hold_active_) {
      if (trigger_obs) {
        degen_dr_hold_frames_left_ = hold_need;
        degen_dr_hold_recover_count_ = 0;
      } else {
        if (degen_dr_hold_frames_left_ > 0) {
          --degen_dr_hold_frames_left_;
        }
        if (recover_obs) {
          ++degen_dr_hold_recover_count_;
        } else {
          degen_dr_hold_recover_count_ = 0;
        }
      }

      const bool hold_done = (degen_dr_hold_frames_left_ <= 0);
      const bool recovered = (degen_dr_hold_recover_count_ >= recover_need);
      if (hold_done || recovered) {
        degen_dr_hold_active_ = false;
        degen_dr_hold_frames_left_ = 0;
        degen_dr_hold_trigger_count_ = 0;
        degen_dr_hold_recover_count_ = 0;
        ROS_INFO_STREAM("[LOC] event=degen_hold_exit hold_done="
                        << (hold_done ? 1 : 0)
                        << " recovered=" << (recovered ? 1 : 0));
      }
    } else {
      if (trigger_obs) {
        ++degen_dr_hold_trigger_count_;
      } else {
        degen_dr_hold_trigger_count_ = 0;
      }

      if (degen_dr_hold_trigger_count_ >= trig_need) {
        degen_dr_hold_active_ = true;
        degen_dr_hold_frames_left_ = hold_need;
        degen_dr_hold_trigger_count_ = 0;
        degen_dr_hold_recover_count_ = 0;
        ROS_WARN_STREAM("[LOC] event=degen_hold_enter hold="
                        << hold_need << " trig=" << trig_need
                        << " cond=" << FmtD(degen_cond, 1)
                        << " sep_w=" << FmtD(degen_amb_weak_sep, 2)
                        << " sep_s=" << FmtD(degen_amb_strong_sep, 2));
      }
    }

    return degen_dr_hold_active_;
  };

  auto updateDegenDrCruise = [&](bool trigger_obs, const Eigen::Vector2f &xy_now) -> bool {
    degen_dr_cruise_state_updated = true;

    if (!cfg_.degen_dr_cruise_enable) {
      degen_dr_cruise_active_ = false;
      degen_dr_cruise_trigger_count_ = 0;
      degen_dr_cruise_frames_ = 0;
      degen_dr_cruise_dist_accum_ = 0.0;
      degen_dr_cruise_has_last_xy_ = false;
  degen_dr_cruise_last_xy_ = Eigen::Vector2f::Zero();
      return false;
    }

    const int trig_need = std::max(1, cfg_.degen_dr_cruise_trigger_frames);
    const int max_frames = std::max(1, cfg_.degen_dr_cruise_max_frames);
    const double hold_dist = std::max(0.0, cfg_.degen_dr_cruise_hold_dist_m);

    if (degen_dr_cruise_active_) {
      if (xy_now.allFinite()) {
        if (degen_dr_cruise_has_last_xy_) {
          degen_dr_cruise_dist_accum_ += (xy_now - degen_dr_cruise_last_xy_).norm();
        }
        degen_dr_cruise_last_xy_ = xy_now;
        degen_dr_cruise_has_last_xy_ = true;
      }

      ++degen_dr_cruise_frames_;
      const bool dist_done = (degen_dr_cruise_dist_accum_ >= hold_dist);
      const bool frame_done = (degen_dr_cruise_frames_ >= max_frames);
      if (dist_done || frame_done) {
        degen_dr_cruise_active_ = false;
        degen_dr_cruise_trigger_count_ = 0;
        degen_dr_cruise_frames_ = 0;
        degen_dr_cruise_dist_accum_ = 0.0;
        degen_dr_cruise_has_last_xy_ = false;
  degen_dr_cruise_last_xy_ = Eigen::Vector2f::Zero();
        ROS_INFO_STREAM("[LOC] event=degen_cruise_exit dist_done="
                        << (dist_done ? 1 : 0)
                        << " frame_done=" << (frame_done ? 1 : 0));
      }
    } else {
      if (trigger_obs) {
        ++degen_dr_cruise_trigger_count_;
      } else {
        degen_dr_cruise_trigger_count_ = 0;
      }

      if (degen_dr_cruise_trigger_count_ >= trig_need) {
        degen_dr_cruise_active_ = true;
        degen_dr_cruise_trigger_count_ = 0;
        degen_dr_cruise_frames_ = 0;
        degen_dr_cruise_dist_accum_ = 0.0;
        degen_dr_cruise_has_last_xy_ = xy_now.allFinite();
        if (degen_dr_cruise_has_last_xy_) {
          degen_dr_cruise_last_xy_ = xy_now;
        }
        ROS_WARN_STREAM("[LOC] event=degen_cruise_enter dist="
                        << FmtD(hold_dist, 2)
                        << " trig=" << trig_need
                        << " max_f=" << max_frames
                        << " cond=" << FmtD(degen_cond, 1)
                        << " amb=" << (degen_ambiguity_high ? 1 : 0));
      }
    }

    return degen_dr_cruise_active_;
  };


  auto updateDegenState = [&](bool degen_detected_now,
                              bool hessian_observed_now) {
    degen_state_updated = true;
    degen_hessian_observed_frame = hessian_observed_now;

    const bool prev_active = degen_active_latched_;
    const int prev_hold = degen_hold_frames_left_;
    const int prev_exit_safe = degen_exit_safe_streak_;

    if (!degen_active_latched_) {
      if (degen_detected_now || cfg_.degen_force_active_always) {
        degen_active_latched_ = true;
        degen_hold_frames_left_ = kDegenProcessFrames;
        degen_exit_safe_streak_ = 0;
      } else {
        degen_hold_frames_left_ = 0;
        degen_exit_safe_streak_ = 0;
      }
    } else {
      if (degen_detected_now || cfg_.degen_force_active_always) {
        degen_hold_frames_left_ = kDegenProcessFrames;
        degen_exit_safe_streak_ = 0;
      } else if (degen_hold_frames_left_ > 0) {
        --degen_hold_frames_left_;
        degen_exit_safe_streak_ = 0;
      } else {
        const bool safe_now = hessian_observed_now && !degen_detected_now;
        if (safe_now) {
          degen_exit_safe_streak_ =
              std::min(degen_exit_safe_streak_ + 1, 1000000);
        } else {
          degen_exit_safe_streak_ = 0;
        }
        if (degen_exit_safe_streak_ >= kDegenExitSafeFrames) {
          degen_active_latched_ = false;
          degen_hold_frames_left_ = 0;
          degen_exit_safe_streak_ = 0;
        }
      }
    }

    degen_active = degen_active_latched_;
    if (degen_active_latched_) {
      ++degen_active_streak_;
    } else {
      degen_active_streak_ = 0;
      degen_force_when_close = false;
    }

    if (prev_active != degen_active_latched_) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[LOC] event=degen_state_toggle active="
                   << (degen_active_latched_ ? 1 : 0)
                   << " cond=" << FmtD(degen_cond, 1)
                   << " hold_left=" << degen_hold_frames_left_
                   << " exit_safe=" << degen_exit_safe_streak_
                   << " cfg(process/exit)=" << kDegenProcessFrames << "/"
                   << kDegenExitSafeFrames
                   << " hobs=" << (hessian_observed_now ? 1 : 0)
                   << " detect=" << (degen_detected_now ? 1 : 0)
                   << " lmin=" << FmtD(degen_lambda_min, 6));
    } else if (degen_active_latched_ &&
               (prev_hold != degen_hold_frames_left_ ||
                prev_exit_safe != degen_exit_safe_streak_)) {
      ROS_INFO_STREAM_THROTTLE(
          1.0, "[LOC] event=degen_state_step hold_left="
                   << degen_hold_frames_left_
                   << " exit_safe=" << degen_exit_safe_streak_
                   << " hobs=" << (hessian_observed_now ? 1 : 0)
                   << " detect=" << (degen_detected_now ? 1 : 0)
                   << " cond=" << FmtD(degen_cond, 1));
    }
  };

  const bool bypass_quality_for_manual_local_seed =
      (used_local_seed && used_manual_local_seed);

  if (!ndt_ok) {
    reason_code = S::REASON_NDT_NO_CONVERGE;
  } else {
    // internal gate (tp only)
    if (!cfg_.quality_enable) {
      ndt_internal_ok = true;
    } else {
      ndt_internal_ok = std::isfinite(trans_prob) &&
                        (trans_prob >= cfg_.quality_trans_prob_bad);
    }

    const bool ndt_super_good = ndt_internal_ok && std::isfinite(trans_prob) &&
                                (trans_prob >= cfg_.quality_trans_prob_good);

    // innovation gate
    innovation_ok = true;
    if (!bypass_quality_for_manual_local_seed && cfg_.quality_enable &&
        ndt_internal_ok && !ndt_super_good) {
      const double max_delta_yaw_rad =
          cfg_.quality_max_delta_yaw_deg * kPi / 180.0;
      if (q_status.delta_trans > cfg_.quality_max_delta_trans ||
          std::fabs(q_status.delta_yaw) > max_delta_yaw_rad) {
        innovation_ok = false;
      }
    }

    ndt_quality_ok =
        ndt_internal_ok &&
        (bypass_quality_for_manual_local_seed || innovation_ok);

    quality_good_for_last_pose = ndt_quality_ok && std::isfinite(trans_prob) &&
                                 (trans_prob >= cfg_.quality_trans_prob_good);

    if (ndt_ok && ndt_internal_ok && !ndt_quality_ok) {
      reason_code = S::REASON_QUALITY_REJECT;
      degen_hessian_miss_streak_ =
          std::min(degen_hessian_miss_streak_ + 1, 1000000);
      // Keep degen latch sticky on Hessian miss / quality reject.
      // 既然没 accept，就不能让 local seed 污染 temporal
      // cache；用输出同源的T_pred
      (void)temporalCandidateMotionGate(stamp_pub, T_pred, true, false);
    }

    if (ndt_quality_ok) {
      Eigen::Matrix4f T_candidate = T_ndt;
      bool degen_hessian_observed = false;

      if (!used_local_seed && cfg_.degen_fusion_enable && cloud_used &&
          static_cast<int>(cloud_used->size()) >= cfg_.degen_min_points) {
        Eigen::Matrix<double, 6, 6> H6;
        if (ndt_matcher_.getFinalHessian(&H6) && H6.allFinite()) {
          degen_hessian_observed = true;
          const DegeneracyDecision d = EvaluateDegeneracyFromHessian(H6, cfg_);
          if (d.valid) {
            degen_valid = true;
            degen_yaw_weak = d.yaw_weak;
            degen_cond = d.cond;
            degen_lambda_min = d.lambda_min;
            degen_lambda_max = d.lambda_max;
            const bool degen_detected_now =
                cfg_.degen_force_active_always ||
                (degen_cond >= cfg_.degen_cond_hyst_on);
            updateDegenState(degen_detected_now, true);

            degen_weak_dir = d.weak_dir;
            degen_strong_dir = Eigen::Vector2f(-degen_weak_dir.y(), degen_weak_dir.x());
            if (!degen_strong_dir.allFinite() || degen_strong_dir.norm() < 1e-5f) {
              degen_strong_dir = Eigen::Vector2f(0.0f, 1.0f);
            } else {
              degen_strong_dir.normalize();
            }

            degen_alpha_w = d.alpha_w;
            degen_alpha_s = d.alpha_s;
            degen_alpha_yaw = d.alpha_yaw;

            degen_hard = (degen_cond >= cfg_.degen_cond_bad);

            if (degen_hard && cfg_.degen_ambiguity_enable &&
                cfg_.degen_ambiguity_seed_offset_m > 0.0 && !amb_probe_skip) {
              const ros::WallTime t_amb0 = ros::WallTime::now();
              int amb_runs_local = 0;
              constexpr double kAmbProbeBudgetMs = 14.0;
              constexpr int kAmbProbeMaxRuns = 8;

              auto amb_elapsed_ms = [&]() -> double {
                return (ros::WallTime::now() - t_amb0).toSec() * 1000.0;
              };

              struct AmbSol {
                Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
                double score = std::numeric_limits<double>::quiet_NaN();
                double tp = std::numeric_limits<double>::quiet_NaN();
              };

              std::vector<AmbSol> sols;
              sols.reserve(9);
              sols.push_back(AmbSol{T_ndt, score, trans_prob});

              const int old_max_iters = ndt_matcher_.getMaxIterations();
              const int low_max_iters = std::max(1, cfg_.degen_ambiguity_ndt_max_iters);
              if (low_max_iters < old_max_iters) {
                ndt_matcher_.setMaxIterations(low_max_iters);
              }

              auto run_seed = [&](const Eigen::Matrix4f &seed) -> bool {
                if (amb_runs_local >= kAmbProbeMaxRuns) {
                  return false;
                }
                if (amb_elapsed_ms() >= kAmbProbeBudgetMs) {
                  return false;
                }
                ++amb_runs_local;
                Eigen::Matrix4f T_try = seed;
                double score_try = std::numeric_limits<double>::max();
                if (!ndt_matcher_.align(cloud_used, seed, T_try, score_try)) {
                  return false;
                }
                const double tp_try = ndt_matcher_.getTransformationProbability();
                sols.push_back(AmbSol{T_try, score_try, tp_try});
                return true;
              };

              const float off_base =
                  static_cast<float>(cfg_.degen_ambiguity_seed_offset_m);
              const float off_expand = off_base * 1.8f;
              const float off_shrink = std::max(0.20f, off_base * 0.6f);

              int weak_success = 0;
              if (run_seed(OffsetPoseAlongWeak(T_pred, degen_weak_dir, +off_base))) {
                ++weak_success;
              }
              if (run_seed(OffsetPoseAlongWeak(T_pred, degen_weak_dir, -off_base))) {
                ++weak_success;
              }

              if (weak_success == 0 && off_expand > off_base + 1e-3f) {
                if (run_seed(OffsetPoseAlongWeak(T_pred, degen_weak_dir, +off_expand))) {
                  ++weak_success;
                }
                if (run_seed(OffsetPoseAlongWeak(T_pred, degen_weak_dir, -off_expand))) {
                  ++weak_success;
                }
              }

              const Eigen::Matrix4f &center_pose = (weak_success > 0) ? T_ndt : T_pred;
              const float off_center = (weak_success > 0) ? off_base : off_shrink;
              run_seed(OffsetPoseAlongWeak(center_pose, degen_weak_dir, +off_center));
              run_seed(OffsetPoseAlongWeak(center_pose, degen_weak_dir, -off_center));

              // Strong-direction probes are decimated and bounded by time budget.
              const bool allow_strong_probe_this_frame =
                  ((amb_strong_probe_rr++ % 3u) == 0u);
              if (allow_strong_probe_this_frame &&
                  amb_elapsed_ms() < kAmbProbeBudgetMs) {
                run_seed(OffsetPoseAlongWeak(T_pred, degen_strong_dir, +off_base));
              }
              if (allow_strong_probe_this_frame &&
                  amb_elapsed_ms() < kAmbProbeBudgetMs) {
                run_seed(OffsetPoseAlongWeak(T_pred, degen_strong_dir, -off_base));
              }

              if (low_max_iters < old_max_iters) {
                ndt_matcher_.setMaxIterations(old_max_iters);
              }

              for (size_t i = 0; i < sols.size(); ++i) {
                for (size_t j = i + 1; j < sols.size(); ++j) {
                  if (!std::isfinite(sols[i].score) || !std::isfinite(sols[j].score) ||
                      !std::isfinite(sols[i].tp) || !std::isfinite(sols[j].tp)) {
                    continue;
                  }

                  const double score_gap = std::fabs(sols[i].score - sols[j].score);
                  const double tp_gap = std::fabs(sols[i].tp - sols[j].tp);

                  const Eigen::Vector2f ti = sols[i].T.block<3, 1>(0, 3).head<2>();
                  const Eigen::Vector2f tj = sols[j].T.block<3, 1>(0, 3).head<2>();
                  const Eigen::Vector2f d_ij = ti - tj;
                  const double weak_sep = std::fabs(d_ij.dot(degen_weak_dir));
                  const double strong_sep = std::fabs(d_ij.dot(degen_strong_dir));

                  const bool quality_close =
                      (score_gap <= cfg_.degen_ambiguity_score_margin) &&
                      (tp_gap <= cfg_.degen_ambiguity_tp_margin);
                  const bool sep_ambiguous =
                      (weak_sep >= cfg_.degen_ambiguity_weak_sep_m) ||
                      (strong_sep >= cfg_.degen_ambiguity_strong_sep_m);
                  if (quality_close && sep_ambiguous) {
                    degen_ambiguity_high = true;
                    const double sep_metric = std::max(weak_sep, strong_sep);
                    const double best_metric = std::max(
                        std::isfinite(degen_amb_weak_sep) ? degen_amb_weak_sep : -1.0,
                        std::isfinite(degen_amb_strong_sep) ? degen_amb_strong_sep : -1.0);
                    if (sep_metric > best_metric) {
                      degen_amb_weak_sep = weak_sep;
                      degen_amb_strong_sep = strong_sep;
                      degen_amb_score_gap = score_gap;
                      degen_amb_tp_gap = tp_gap;
                    }
                  }
                }
              }

              degen_amb_probe_runs = amb_runs_local;
              degen_amb_probe_ms =
                  (ros::WallTime::now() - t_amb0).toSec() * 1000.0;
            }

            {
              const Eigen::Vector2f t_pred = T_pred.block<3, 1>(0, 3).head<2>();
              const Eigen::Vector2f t_ndt = T_ndt.block<3, 1>(0, 3).head<2>();
              const Eigen::Vector2f dt_xy = (t_ndt - t_pred);

              Eigen::Vector2f ew = degen_weak_dir;
              if (!ew.allFinite() || ew.norm() < 1e-5f) {
                ew = Eigen::Vector2f(1.0f, 0.0f);
              } else {
                ew.normalize();
              }
              const Eigen::Vector2f es = degen_strong_dir;

              degen_pred_dw_abs = std::fabs(dt_xy.dot(ew));
              degen_pred_ds_abs = std::fabs(dt_xy.dot(es));

              const double yaw_pred = std::atan2(T_pred(1, 0), T_pred(0, 0));
              const double yaw_ndt = std::atan2(T_ndt(1, 0), T_ndt(0, 0));
              degen_pred_dyaw_abs = std::fabs(NormalizeAngle(yaw_ndt - yaw_pred));

              degen_close_to_pred =
                  std::isfinite(degen_pred_dw_abs) &&
                  std::isfinite(degen_pred_ds_abs) &&
                  std::isfinite(degen_pred_dyaw_abs) &&
                  (degen_pred_dw_abs <= cfg_.degen_amb_accept_dw_m) &&
                  (degen_pred_ds_abs <= cfg_.degen_amb_accept_ds_m) &&
                  (degen_pred_dyaw_abs <= cfg_.degen_amb_accept_dyaw_rad);
            }

            {
              degen_weak_motion_align = std::numeric_limits<double>::quiet_NaN();
              constexpr float kMinMotionStepForAlign = 2e-4f;

              Eigen::Vector2f weak_dir = degen_weak_dir;
              if (!weak_dir.allFinite() || weak_dir.norm() < 1e-5f) {
                weak_dir = Eigen::Vector2f(1.0f, 0.0f);
              } else {
                weak_dir.normalize();
              }

              auto try_update_align_from_delta =
                  [&](const Eigen::Vector2f &delta_xy) -> bool {
                if (!delta_xy.allFinite())
                  return false;
                const float n = delta_xy.norm();
                if (!(n > kMinMotionStepForAlign))
                  return false;
                const Eigen::Vector2f v_dir = delta_xy / n;
                if (!v_dir.allFinite())
                  return false;
                has_degen_last_motion_dir_ = true;
                degen_last_motion_dir_ = v_dir;
                degen_weak_motion_align = std::fabs(v_dir.dot(weak_dir));
                return std::isfinite(degen_weak_motion_align);
              };

              if (has_last_published_) {
                const Eigen::Vector2f t_pred_xy = T_pred.block<3, 1>(0, 3).head<2>();
                const Eigen::Vector2f t_last_xy =
                    T_last_published_.block<3, 1>(0, 3).head<2>();
                (void)try_update_align_from_delta(t_pred_xy - t_last_xy);
              }

              if (!std::isfinite(degen_weak_motion_align) && has_last_published_) {
                Odom2D od_prev, od_now;
                const bool od_prev_ok = QueryOdomPoseClamped(
                    odom_buf_, t_last_published_, cfg_.temporal_odom_max_age,
                    &od_prev);
                const bool od_now_ok = QueryOdomPoseClamped(
                    odom_buf_, stamp_pub, cfg_.temporal_odom_max_age, &od_now);
                if (od_prev_ok && od_now_ok) {
                  const Eigen::Vector2f d_odom(
                      static_cast<float>(od_now.x - od_prev.x),
                      static_cast<float>(od_now.y - od_prev.y));
                  (void)try_update_align_from_delta(d_odom);
                }
              }

              if (!std::isfinite(degen_weak_motion_align) &&
                  has_degen_last_motion_dir_) {
                if (degen_last_motion_dir_.allFinite() &&
                    degen_last_motion_dir_.norm() > 1e-5f) {
                  degen_weak_motion_align =
                      std::fabs(degen_last_motion_dir_.dot(weak_dir));
                } else {
                  has_degen_last_motion_dir_ = false;
                }
              }

              const bool motion_aligned =
                  std::isfinite(degen_weak_motion_align) &&
                  (degen_weak_motion_align >= 0.85);
              const int persistent_need =
                  std::max(2, cfg_.degen_dr_cruise_trigger_frames);
              const int degen_active_streak_next =
                  degen_active ? (degen_active_streak_ + 1) : 0;
              degen_force_when_close =
                  degen_close_to_pred &&
                  (degen_active_streak_next >= persistent_need) &&
                  motion_aligned;
              if (degen_force_when_close) {
                degen_alpha_w = std::min(degen_alpha_w, cfg_.degen_alpha_w_min);
              }
            }

            if (degen_ambiguity_high) {
              degen_alpha_w = std::min(degen_alpha_w, Clamp01(cfg_.degen_alpha_w_ambiguity));
              degen_alpha_s = std::min(degen_alpha_s, Clamp01(cfg_.degen_alpha_s_ambiguity));
              // Only down-weight yaw when yaw itself is weakly observable.
              if (degen_yaw_weak) {
                degen_alpha_yaw = std::min(degen_alpha_yaw, Clamp01(cfg_.degen_alpha_yaw_ambiguity));
              }
            }

            if (degen_active) {
              // Degenerate corridor mode: weak direction follows prediction,
              // strong direction follows NDT.
              degen_alpha_w = 0.0;
              degen_alpha_s = 1.0;
              T_candidate = FusePoseAnisotropic(T_pred, T_ndt, degen_weak_dir,
                                                degen_alpha_w, degen_alpha_s,
                                                degen_alpha_yaw);
            }
          }
        }
      }
      if (degen_hessian_observed) {
        degen_hessian_miss_streak_ = 0;
      } else {
        degen_hessian_miss_streak_ =
            std::min(degen_hessian_miss_streak_ + 1, 1000000);
      }

      if (!degen_state_updated) {
        updateDegenState(false, degen_hessian_observed);
      }

      if (!degen_hessian_observed && degen_active_latched_) {
        degen_force_dr_no_hessian = true;
        ROS_WARN_STREAM_THROTTLE(
            1.0, "[LOC] event=degen_no_hessian_force_dr miss="
                     << degen_hessian_miss_streak_
                     << " active=" << (degen_active_latched_ ? 1 : 0)
                     << " hold_left=" << degen_hold_frames_left_
                     << " exit_safe=" << degen_exit_safe_streak_
                     << " min_pts=" << cfg_.degen_min_points);
      }

      const bool hold_trigger_obs =
          degen_hard && degen_ambiguity_high &&
          (!degen_close_to_pred || degen_force_when_close);
      // Recover as soon as the strict hold-trigger condition clears.
      // This avoids being stuck in hold in long corridors where degen_hard can persist.
      const bool hold_recover_obs = (!hold_trigger_obs);
      degen_dr_hold_forced =
          updateDegenDrHold(hold_trigger_obs, hold_recover_obs);

      const bool straight_motion =
          (std::fabs(last_imu_gyro_z_) * 180.0 / kPi <=
           cfg_.degen_dr_cruise_max_yaw_rate_deg);
      const bool moving_forward =
          std::isfinite(latest_linear_speed_) &&
          (latest_linear_speed_ >= cfg_.degen_dr_cruise_min_speed_mps);
      bool cruise_trigger_obs = degen_active;
      if (cfg_.degen_dr_cruise_require_ambiguity) cruise_trigger_obs = cruise_trigger_obs && degen_ambiguity_high;
      if (cfg_.degen_dr_cruise_require_hard) cruise_trigger_obs = cruise_trigger_obs && degen_hard;
      // Prefer non-close solutions, but allow cruise when close-to-pred is
      // persistent under weak-direction corridor degeneracy.
      cruise_trigger_obs =
          cruise_trigger_obs && (!degen_close_to_pred || degen_force_when_close);
      cruise_trigger_obs = cruise_trigger_obs && moving_forward && straight_motion;

      const Eigen::Vector2f t_pred_xy = T_pred.block<3, 1>(0, 3).head<2>();
      degen_dr_cruise_forced = updateDegenDrCruise(cruise_trigger_obs, t_pred_xy);

      if (degen_force_dr_no_hessian || degen_dr_hold_forced || degen_dr_cruise_forced) {
        reason_code = S::REASON_TEMPORAL_REJECT;
        temporal_ok_flag = false;
        // Keep temporal cache aligned with DR output during forced DR phases.
        (void)temporalCandidateMotionGate(stamp_pub, T_pred, true, false);
      } else {
        temporalA_ok = temporalCandidateMotionGate(
            stamp_pub, T_candidate, ndt_internal_ok, do_temporal, &dtA, &v_cand,
            &v_odom, &w_cand_deg, &w_imu_deg, &v_diff, &w_diff_deg);

        temporalB_ok = temporalJumpBoundsGate(stamp_pub, T_candidate, do_temporal,
                                              &dtB, &v_jump, &w_jump_deg);

        const bool a_checked = do_temporal && std::isfinite(dtA) &&
                               (dtA >= kTemporalMinDt) &&
                               (dtA <= cfg_.temporal_max_dt);
        const bool b_checked = do_temporal && std::isfinite(dtB) &&
                               (dtB >= kTemporalMinDt) &&
                               (dtB <= cfg_.temporal_max_dt);
        temporal_checked = (a_checked || b_checked);

        temporal_ok_flag = (!do_temporal) ? true : (temporalA_ok && temporalB_ok);

        if (do_temporal && !temporal_ok_flag) {
          reason_code = S::REASON_TEMPORAL_REJECT;

          const bool force_enter_by_temp_reject =
              degen_valid && std::isfinite(degen_pred_dw_abs) &&
              (degen_pred_dw_abs >= cfg_.degen_temp_reject_enter_dw_m);
          if (force_enter_by_temp_reject) {
            const bool was_active = degen_active_latched_;
            degen_active_latched_ = true;
            degen_active = true;
            degen_hold_frames_left_ = kDegenProcessFrames;
            degen_exit_safe_streak_ = 0;
            degen_active_streak_ = std::max(degen_active_streak_, 1);
            ROS_WARN_STREAM_THROTTLE(
                1.0, "[LOC] event=degen_force_enter_temp_reject was_active="
                         << (was_active ? 1 : 0)
                         << " dw=" << FmtD(degen_pred_dw_abs, 3)
                         << " thr=" << FmtD(cfg_.degen_temp_reject_enter_dw_m, 3)
                         << " cond=" << FmtD(degen_cond, 1)
                         << " hobs=" << (degen_hessian_observed_frame ? 1 : 0));
          }
        } else {
          // accept
          T_output = T_candidate;
          mode_code = S::MODE_NDT;
          using_prediction_frame = false;
          reason_code = S::REASON_OK;

          predictor_.commitPredictionAsCorrection(T_output);

          reloc_mgr_.ack();
          if (cfg_.temporal_grace_enable &&
              (used_local_seed || !has_ever_had_good_ndt_)) {
            temporal_grace_until_ =
                stamp_pub + ros::Duration(cfg_.temporal_grace_duration);
          }

          // stationary + last pose write
          const bool now_stationary =
              (latest_linear_speed_ < cfg_.stationary_speed_thresh);
          if (!now_stationary) {
            stationary_phase_ = false;
            stationary_pose_written_ = false;
          } else {
            if (!stationary_phase_) {
              stationary_phase_ = true;
              stationary_pose_written_ = false;
              stationary_start_time_ = stamp_pub;
            }
          }

          const bool allow_last_pose_write =
              !(degen_ambiguity_high && cfg_.degen_block_last_pose_on_ambiguity);

          if (allow_last_pose_write && quality_good_for_last_pose && stationary_phase_ &&
              !stationary_pose_written_) {
            const double dt_stop = (stamp_pub - stationary_start_time_).toSec();
            if (dt_stop >= cfg_.stationary_min_duration) {
              InitialPoseManager::Pose2D pose2d;
              pose2d.x = T_output(0, 3);
              pose2d.y = T_output(1, 3);
              pose2d.yaw = std::atan2(T_output(1, 0), T_output(0, 0));
              initial_pose_mgr_.updateLastPose(pose2d);
              stationary_pose_written_ = true;
              ROS_INFO_STREAM("[LOC] event=last_pose_written x="
                              << FmtD(pose2d.x, 3) << " y=" << FmtD(pose2d.y, 3)
                              << " yaw=" << FmtD(pose2d.yaw, 3));
            }
          }
        }
      }

    } else {
      if (!ndt_internal_ok)
        reason_code = S::REASON_TP_REJECT;
      degen_hessian_miss_streak_ =
          std::min(degen_hessian_miss_streak_ + 1, 1000000);
      // Keep degen latch sticky on Hessian miss / quality reject.
    }
  }

  if (!degen_state_updated) {
    updateDegenState(false, false);
  }

  if (degen_active_latched_ && !degen_hessian_observed_frame) {
    degen_force_dr_no_hessian = true;
  }

  if (degen_active_latched_ && degen_hold_frames_left_ <= 0 &&
      reason_code == S::REASON_TEMPORAL_REJECT) {
    if (degen_exit_safe_streak_ > 0) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "[LOC] event=degen_exit_reset reason=temp_reject"
                   << " safe_streak=" << degen_exit_safe_streak_);
    }
    degen_exit_safe_streak_ = 0;
  }

  degen_active = degen_active_latched_;

  if (!degen_dr_hold_state_updated) {
    degen_dr_hold_forced = updateDegenDrHold(false, false);
  } else {
    degen_dr_hold_forced = degen_dr_hold_active_;
  }

  const Eigen::Vector2f t_pred_xy_post = T_pred.block<3, 1>(0, 3).head<2>();
  if (!degen_dr_cruise_state_updated) {
    degen_dr_cruise_forced = updateDegenDrCruise(false, t_pred_xy_post);
  } else {
    degen_dr_cruise_forced = degen_dr_cruise_active_;
  }

  // Adaptive intensity prefilter relax: if recent frame is degen/ambiguous
  // (or already forced DR by degen handlers), relax intensity pruning for
  // next frames to recover geometric observability.
  if (cfg_.intensity_prefilter_enable && cfg_.intensity_prefilter_adaptive_enable) {
    bool intensity_relax_trigger = false;
    if (degen_valid && (degen_hard || degen_ambiguity_high)) {
      intensity_relax_trigger = true;
    }
    if (degen_force_dr_no_hessian || degen_dr_hold_forced || degen_dr_cruise_forced) {
      intensity_relax_trigger = true;
    }
    if (intensity_relax_trigger) {
      const int relax_frames =
          std::max(1, cfg_.intensity_prefilter_adaptive_frames);
      const int before = intensity_prefilter_adaptive_frames_left_;
      intensity_prefilter_adaptive_frames_left_ =
          std::max(intensity_prefilter_adaptive_frames_left_, relax_frames);
      if (before <= 0) {
        ROS_WARN_STREAM("[LOC] event=intensity_relax_enter frames="
                        << relax_frames
                        << " keep="
                        << FmtD(cfg_.intensity_prefilter_adaptive_keep_ratio, 2)
                        << " cond=" << FmtD(degen_cond, 1)
                        << " amb=" << (degen_ambiguity_high ? 1 : 0)
                        << " hold=" << (degen_dr_hold_forced ? 1 : 0)
                        << " cruise=" << (degen_dr_cruise_forced ? 1 : 0));
      }
    }
  } else {
    intensity_prefilter_adaptive_frames_left_ = 0;
  }

  // loc_level update
  const bool is_parking = (latest_linear_speed_ < cfg_.stationary_speed_thresh);
  const bool ndt_accept =
      (!using_prediction_frame && ndt_converged_flag && temporal_ok_flag);

  Eigen::Vector3f pos_now = T_output.block<3, 1>(0, 3);
  const bool had_last_output_position = has_last_output_position_;
  if (!has_last_output_position_) {
    last_output_position_ = pos_now;
    has_last_output_position_ = true;
    has_prev_output_position_ = false;
  }

  if (!has_ever_had_good_ndt_) {
    if (ndt_accept) {
      has_ever_had_good_ndt_ = true;
      dr_distance_since_last_ndt_ = 0.0;
      dr_frames_since_last_ndt_ = 0;
      loc_level_ = LocLevel::GOOD;
    } else {
      loc_level_ = LocLevel::INITIALIZING;
    }
  } else {
    if (ndt_accept) {
      dr_distance_since_last_ndt_ = 0.0;
      dr_frames_since_last_ndt_ = 0;
      loc_level_ = LocLevel::GOOD;
    } else {
      if (!is_parking) {
        const double step = (pos_now - last_output_position_).head<2>().norm();
        dr_distance_since_last_ndt_ += step;
        dr_frames_since_last_ndt_++;
      }

      if (dr_distance_since_last_ndt_ < cfg_.dr_fail_dist_thresh &&
          dr_frames_since_last_ndt_ < cfg_.dr_frame_fail_thresh) {
        loc_level_ = LocLevel::FOLLOWING_DR;
      } else {
        loc_level_ = LocLevel::FAIL;
      }
    }
  }

  if (had_last_output_position) {
    prev_output_position_ = last_output_position_;
    has_prev_output_position_ = true;
  }
  last_output_position_ = pos_now;
  has_last_output_position_ = true;

  // ndt_accept 已经算了：(!using_prediction_frame && ndt_converged_flag &&
  // temporal_ok_flag) 再加一条 reason=OK，避免异常路径污染
  const bool good_pose_for_shutdown =
      ndt_accept && (reason_code == S::REASON_OK) && (mode_code == S::MODE_NDT);

  const bool block_anchor_update =
      degen_ambiguity_high && cfg_.degen_block_anchor_on_ambiguity;
  const bool block_last_good_update =
      degen_ambiguity_high && cfg_.degen_block_last_pose_on_ambiguity;

  if (good_pose_for_shutdown && !block_last_good_update) {
    last_good_pose_.x = T_output(0, 3);
    last_good_pose_.y = T_output(1, 3);
    last_good_pose_.yaw = std::atan2(T_output(1, 0), T_output(0, 0));
    has_last_good_pose_ = true;
  } else if (good_pose_for_shutdown && block_last_good_update) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[LOC] event=block_last_good_on_amb cond=" << FmtD(degen_cond, 1)
                  << " lmin=" << FmtD(degen_lambda_min, 6));
  }

  if (good_pose_for_shutdown && !block_anchor_update) {
    // 记录锚：T_map_odom = T_map_base * inv(T_odom_base)
    Odom2D od;
    if (QueryOdomPose(odom_buf_, stamp_pub, cfg_.temporal_odom_max_age, &od)) {
      const double out_yaw = std::atan2(T_output(1, 0), T_output(0, 0));
      const Eigen::Matrix3d T_mb = SE2(T_output(0, 3), T_output(1, 3), out_yaw);
      const Eigen::Matrix3d T_ob = SE2(od.x, od.y, od.yaw);
      T_map_odom_anchor_ = T_mb * SE2Inv(T_ob);
      has_map_odom_anchor_ = true;
    } else {
      has_map_odom_anchor_ = false;
    }
  }

  // FAIL edge -> schedule local
  if (cfg_.local_reloc_enable) {
    if (prev_loc_level_ != LocLevel::FAIL && loc_level_ == LocLevel::FAIL) {
      RelocPose2D c;
      if (has_last_good_pose_) {
        c = last_good_pose_;
      } else {
        c.x = T_last_published_(0, 3);
        c.y = T_last_published_(1, 3);
        c.yaw = std::atan2(T_last_published_(1, 0), T_last_published_(0, 0));
      }
      reloc_mgr_.onFailEdge(c);

      fail_moved_far_since_edge_ = false;

      if (has_last_odom_msg_) {
        const Eigen::Vector2f odom_xy(
            static_cast<float>(last_odom_msg_.pose.pose.position.x),
            static_cast<float>(last_odom_msg_.pose.pose.position.y));

        fail_anchor_odom_xy_ = odom_xy;
        has_fail_anchor_ = true;

        fail_attempt_anchor_odom_xy_ = odom_xy;
        has_fail_attempt_anchor_ = true;
      } else {
        has_fail_anchor_ = false;
        has_fail_attempt_anchor_ = false;
      }

      fail_stationary_start_ = ros::Time(0);
      fail_last_periodic_local_ = stamp_pub;

      ROS_WARN_STREAM("[LOC] event=fail_edge -> schedule local_seed "
                      << "center(x=" << FmtD(c.x, 3) << " y=" << FmtD(c.y, 3)
                      << " yaw=" << FmtD(c.yaw, 2) << "rad)");
    }
    prev_loc_level_ = loc_level_;
  }

  publishStatus(stamp_pub, q_status, mode_code, reason_code, ndt_converged_flag,
                temporal_ok_flag);

  T_last_published_ = T_output;
  t_last_published_ = stamp_pub;
  has_last_published_ = true;

  auto make_line = [&]() {
    const Eigen::Vector3f t = T_output.block<3, 1>(0, 3);
    const double yaw_rad = std::atan2(T_output(1, 0), T_output(0, 0));

    std::ostringstream oss;
    oss << "[LOC] t=" << FmtD(stamp_pub.toSec(), 3)
        << " lvl=" << LocLevelStr(loc_level_) << " mode=" << ModeStr(mode_code)
        << " grace=" << (in_grace ? 1 : 0)

        << " ndt(conv=" << (ndt_ok ? 1 : 0)
        << " score=" << FmtD(q_status.score, 3)
        << " tp=" << FmtD(q_status.trans_probability, 3)
        << " dT=" << FmtD(q_status.delta_trans, 3)
        << " dYaw=" << FmtD(q_status.delta_yaw * 180.0 / kPi, 2) << "deg)"

        << " gate(int=" << (ndt_internal_ok ? 1 : 0)
        << " innov=" << (innovation_ok ? 1 : 0)
        << " q=" << (ndt_quality_ok ? 1 : 0) << ")"

        << " temp_chk=" << (temporal_checked ? 1 : 0)
        << " A=" << (temporalA_ok ? 1 : 0) << " dt=" << FmtD(dtA, 3)
        << " v=" << FmtD(v_cand, 2) << "/" << FmtD(v_odom, 2)
        << " dv=" << FmtD(v_diff, 2) << " w=" << FmtD(w_cand_deg, 1) << "/"
        << FmtD(w_imu_deg, 1) << " dw=" << FmtD(w_diff_deg, 1)

        << " tempB_ok=" << (temporalB_ok ? 1 : 0) << " dt=" << FmtD(dtB, 3)
        << " v_jump=" << FmtD(v_jump, 2) << " w_jump=" << FmtD(w_jump_deg, 1)
        << ")"

        << " temp_ok=" << (temporal_ok_flag ? 1 : 0)

        << " degen(v=" << (degen_valid ? 1 : 0)
        << " act=" << (degen_active ? 1 : 0)
        << " yawW=" << (degen_yaw_weak ? 1 : 0)
        << " cond=" << FmtD(degen_cond, 1)
        << " lmin=" << FmtD(degen_lambda_min, 6)
        << " lmax=" << FmtD(degen_lambda_max, 3)
        << " ew=(" << FmtD(degen_weak_dir.x(), 2) << ","
        << FmtD(degen_weak_dir.y(), 2) << ")"
        << " a=" << FmtD(degen_alpha_w, 2) << "/" << FmtD(degen_alpha_s, 2)
        << "/" << FmtD(degen_alpha_yaw, 2)
        << " amb=" << (degen_ambiguity_high ? 1 : 0)
        << " sep_w=" << FmtD(degen_amb_weak_sep, 2)
        << " sep_s=" << FmtD(degen_amb_strong_sep, 2)
        << " ds=" << FmtD(degen_amb_score_gap, 4)
        << " dtp=" << FmtD(degen_amb_tp_gap, 3)
        << " ctp=" << (degen_close_to_pred ? 1 : 0)
        << " c_align=" << FmtD(degen_weak_motion_align, 2)
        << " c_force=" << (degen_force_when_close ? 1 : 0)
        << " c_dw=" << FmtD(degen_pred_dw_abs, 2)
        << " c_ds=" << FmtD(degen_pred_ds_abs, 2)
        << " c_dy=" << FmtD(degen_pred_dyaw_abs * 180.0 / kPi, 1)
        << " ap_ms=" << FmtD(degen_amb_probe_ms, 2)
        << " ap_n=" << degen_amb_probe_runs
        << " sm=" << degen_hold_frames_left_ << "/" << degen_exit_safe_streak_
        << " hold=" << (degen_dr_hold_active_ ? 1 : 0)
        << "/" << degen_dr_hold_frames_left_
        << " hc=" << degen_dr_hold_trigger_count_
        << "/" << degen_dr_hold_recover_count_
        << " hf=" << (degen_dr_hold_forced ? 1 : 0)
        << " hmf=" << (degen_force_dr_no_hessian ? 1 : 0)
        << " cruise=" << (degen_dr_cruise_active_ ? 1 : 0)
        << " cd=" << FmtD(degen_dr_cruise_dist_accum_, 2)
        << " cf=" << degen_dr_cruise_frames_
        << " cforce=" << (degen_dr_cruise_forced ? 1 : 0) << ")"

        << " out(x=" << FmtD(t.x(), 3) << " y=" << FmtD(t.y(), 3)
        << " yaw=" << FmtD(yaw_rad, 2) << "rad)"

        << " reason=" << static_cast<int>(reason_code);

    return oss.str();
  };

  if (reason_code == S::REASON_OK) {
    ROS_INFO_STREAM_THROTTLE(1.0, make_line());
  } else {
    ROS_WARN_STREAM_THROTTLE(1.0, make_line());
  }

  publishOdom(stamp_pub, T_output);
  publish_scan_cloud_map(T_output);

  const double scan_cb_ms = (ros::WallTime::now() - cb_t0).toSec() * 1000.0;
  last_scan_cb_ms = scan_cb_ms;
  ROS_INFO_STREAM_THROTTLE(
      1.0, "[LOC] scan_cb_ms=" << FmtD(scan_cb_ms, 2)
                                << " amb_probe_ms=" << FmtD(degen_amb_probe_ms, 2)
                                << " amb_probe_n=" << degen_amb_probe_runs
                                << " amb_skip=" << (amb_probe_skip ? 1 : 0)
                                << " sm=" << degen_hold_frames_left_ << "/"
                                << degen_exit_safe_streak_
                                << " hold=" << (degen_dr_hold_active_ ? 1 : 0)
                                << "/" << degen_dr_hold_frames_left_
                                << " hold_forced=" << (degen_dr_hold_forced ? 1 : 0)
                                << " hmiss_forced=" << (degen_force_dr_no_hessian ? 1 : 0)
                                << " cruise=" << (degen_dr_cruise_active_ ? 1 : 0)
                                << " cruise_d=" << FmtD(degen_dr_cruise_dist_accum_, 2)
                                << " cruise_f=" << degen_dr_cruise_frames_
                                << " cruise_forced=" << (degen_dr_cruise_forced ? 1 : 0)
                                << " irelax="
                                << (intensity_prefilter_adaptive_frames_left_ > 0 ? 1 : 0)
                                << "/" << intensity_prefilter_adaptive_frames_left_);
}

// ===================== manual initialpose =====================

void NdtLocalizerNode::initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg) {

  auto sanitize_stamp = [&](const ros::Time &msg_stamp) -> ros::Time {
    const ros::Time now = ros::Time::now();
    ros::Time s = msg_stamp.isZero() ? now : msg_stamp;
    if (has_last_published_ && !t_last_published_.isZero() &&
        s < t_last_published_) {
      s = t_last_published_;
    }
    if (!now.isZero() && s > now)
      s = now;
    return s;
  };

  const ros::Time stamp_in = msg->header.stamp;
  const ros::Time stamp_pub = sanitize_stamp(stamp_in);

  Eigen::Matrix4f T_map_base = Eigen::Matrix4f::Identity();
  const auto &p = msg->pose.pose.position;
  const auto &q_msg = msg->pose.pose.orientation;

  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  q.normalize();
  Eigen::Matrix3d R = q.toRotationMatrix();

  T_map_base.block<3, 3>(0, 0) = R.cast<float>();
  T_map_base(0, 3) = static_cast<float>(p.x);
  T_map_base(1, 3) = static_cast<float>(p.y);
  T_map_base(2, 3) = 0.0f;

  bool used_dummy_odom = false;
  if (!has_last_odom_msg_) {
    used_dummy_odom = true;
    nav_msgs::Odometry dummy;
    dummy.pose.pose.orientation.w = 1.0;
    predictor_.resetWithCorrection(T_map_base, dummy);
  } else {
    predictor_.resetWithCorrection(T_map_base, last_odom_msg_);
  }

  initial_pose_applied_ = true;
  loc_level_ = LocLevel::INITIALIZING;
  has_ever_had_good_ndt_ = false;

  dr_distance_since_last_ndt_ = 0.0;
  dr_frames_since_last_ndt_ = 0;

  stationary_phase_ = false;
  stationary_pose_written_ = false;

  if (cfg_.temporal_grace_enable) {
    temporal_grace_until_ =
        stamp_pub + ros::Duration(cfg_.temporal_grace_duration);
  }

  T_last_published_ = T_map_base;
  t_last_published_ = stamp_pub;
  has_last_published_ = true;

  T_last_temporal_ = T_map_base;
  t_last_temporal_ = stamp_pub;
  has_last_temporal_ = true;

  last_output_position_ = T_map_base.block<3, 1>(0, 3);
  has_last_output_position_ = true;
  prev_output_position_ = last_output_position_;
  has_prev_output_position_ = false;
  degen_active_latched_ = false;
  degen_active_streak_ = 0;
  degen_hessian_miss_streak_ = 0;
  degen_hold_frames_left_ = 0;
  degen_exit_safe_streak_ = 0;
  has_degen_last_motion_dir_ = false;
  degen_last_motion_dir_ = Eigen::Vector2f(1.0f, 0.0f);

  publishOdom(stamp_pub, T_map_base);
  {
    using S = localization_msgs::LocalizationStatus;
    NdtQualityMetrics q0{};
    publishStatus(stamp_pub, q0, S::MODE_INITIALIZING, S::REASON_OK, false,
                  true);
  }

  if (cfg_.local_reloc_enable) {
    RelocPose2D center;
    center.x = p.x;
    center.y = p.y;
    center.yaw = std::atan2(T_map_base(1, 0), T_map_base(0, 0));

    const auto &cov = msg->pose.covariance;
    const double cov_xx = cov[0];
    const double cov_yy = cov[7];
    const double cov_yaw = cov[35];

    reloc_mgr_.ack();
    reloc_mgr_.onManual(center, cov_xx, cov_yy, cov_yaw);
  }

  const double yaw_rad = std::atan2(T_map_base(1, 0), T_map_base(0, 0));
  ROS_INFO_STREAM("[LOC] event=initialpose "
                  << "t=" << FmtD(stamp_pub.toSec(), 3)
                  << " odom=" << (used_dummy_odom ? "dummy" : "last")
                  << " out(x=" << FmtD(p.x, 3) << " y=" << FmtD(p.y, 3)
                  << " yaw=" << FmtD(yaw_rad, 2) << "rad)"
                  << " -> local_hint(cov->window, score_margin=0)");
}

// ===================== quality/status/odom =====================

NdtQualityMetrics NdtLocalizerNode::evaluateNdtQuality(
    double score, double trans_probability, const Eigen::Matrix4f &T_pred,
    const Eigen::Matrix4f &T_ndt, bool ndt_converged) const {
  NdtQualityMetrics q;
  q.score = score;
  q.trans_probability = trans_probability;
  q.ndt_converged = ndt_converged;

  if (!ndt_converged) {
    q.delta_trans = std::numeric_limits<double>::quiet_NaN();
    q.delta_yaw = std::numeric_limits<double>::quiet_NaN();
    return q;
  }

  Eigen::Vector2f t_pred = T_pred.block<3, 1>(0, 3).head<2>();
  Eigen::Vector2f t_ndt = T_ndt.block<3, 1>(0, 3).head<2>();
  q.delta_trans = (t_ndt - t_pred).norm();

  float yaw_pred = std::atan2(T_pred(1, 0), T_pred(0, 0));
  float yaw_ndt = std::atan2(T_ndt(1, 0), T_ndt(0, 0));
  float dyaw = yaw_ndt - yaw_pred;
  while (dyaw > kPi)
    dyaw -= 2.0f * static_cast<float>(kPi);
  while (dyaw < -kPi)
    dyaw += 2.0f * static_cast<float>(kPi);
  q.delta_yaw = dyaw;

  return q;
}

void NdtLocalizerNode::publishOdom(const ros::Time &stamp,
                                   const Eigen::Matrix4f &T_map_base) {
  if (!algo_enabled_) {
    return;
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = base_frame_id_;

  Eigen::Matrix3f R = T_map_base.block<3, 3>(0, 0);
  Eigen::Vector3f t = T_map_base.block<3, 1>(0, 3);

  Eigen::Quaternionf q(R);
  q.normalize();

  odom.pose.pose.position.x = t.x();
  odom.pose.pose.position.y = t.y();
  odom.pose.pose.position.z = 0.0f;
  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  if (has_last_odom_msg_) {
    odom.twist.twist.linear = last_odom_msg_.twist.twist.linear;
  } else {
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
  }

  if (has_last_imu_) {
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = last_imu_gyro_z_;
  } else if (has_last_odom_msg_) {
    odom.twist.twist.angular = last_odom_msg_.twist.twist.angular;
  } else {
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
  }

  pose_pub_.publish(odom);

  geometry_msgs::TransformStamped tf_msg;
  tf_msg.header.stamp = stamp;
  tf_msg.header.frame_id = "map";
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

void NdtLocalizerNode::publishStatus(const ros::Time &stamp,
                                     const NdtQualityMetrics &q, uint8_t mode,
                                     uint8_t reason, bool ndt_converged,
                                     bool temporal_ok) {
  using S = localization_msgs::LocalizationStatus;

  // -------- 1) status topic：有就发，没有也不影响 DTC --------
  const bool forced_fail =
      sensor_force_fail_ &&
      (sensor_force_until_.isZero() || stamp <= sensor_force_until_);

  if (status_pub_) {
    S msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "map";

    if (forced_fail) {
      msg.health = msg.HEALTH_FAIL;
    } else if (!has_ever_had_good_ndt_) {
      msg.health = msg.HEALTH_INITIALIZING;
    } else if (loc_level_ == LocLevel::FAIL) {
      msg.health = msg.HEALTH_FAIL;
    } else {
      msg.health = msg.HEALTH_GOOD;
    }

    if (msg.health == msg.HEALTH_INITIALIZING) {
      msg.mode = msg.MODE_INITIALIZING;
    } else if (msg.health == msg.HEALTH_FAIL) {
      msg.mode = msg.MODE_FAIL;
    } else {
      msg.mode = mode;
    }

    msg.reason = forced_fail ? sensor_force_reason_ : reason;

    msg.ndt_converged = ndt_converged;
    msg.temporal_ok = temporal_ok;

    msg.score = static_cast<float>(q.score);
    msg.trans_probability = static_cast<float>(q.trans_probability);
    msg.delta_trans = static_cast<float>(q.delta_trans);
    msg.delta_yaw_deg = static_cast<float>(q.delta_yaw * 180.0 / kPi);

    status_pub_.publish(msg);
  }

  // -------- 2) DTC Base --------
  if (!map_ || map_->empty()) {
    dtc_pub_.SetBase(2, 3015);  // map not loaded
    return;
  }

  // watchdog 强制 fail：用 forced_fail + sensor_force_reason_ 来映射 DTC
  if (forced_fail) {
    int dtc = 3001;
    switch (sensor_force_reason_) {
      case 50: dtc = 3010; break; // scan timeout
      case 51: dtc = 3011; break; // odom timeout
      case 52: dtc = 3012; break; // imu timeout
      case 53: dtc = 3013; break; // multi timeout
      default: dtc = 3001; break; // generic fail
    }
    dtc_pub_.SetBase(2, dtc);
    return;
  }

  if (loc_level_ == LocLevel::INITIALIZING) {
    dtc_pub_.SetBase(1, 3019);  // initializing (WARN)
  } else if (loc_level_ == LocLevel::FAIL) {
    dtc_pub_.SetBase(2, 3001);  // localization fail
  } else {
    dtc_pub_.SetBase(0, 3000);  // ok
  }
}


// ===================== watchdog =====================

bool NdtLocalizerNode::watchdogCheck_(const ros::Time &now, uint8_t *out_reason,
                                      std::string *out_detail,
                                      double *out_age_scan,
                                      double *out_age_odom,
                                      double *out_age_imu) {
  if (out_reason)
    *out_reason = 0;
  if (out_detail)
    out_detail->clear();

  auto age_of = [&](bool has, const ros::Time &last) -> double {
    if (!has)
      return std::numeric_limits<double>::infinity();
    const double a = (now - last).toSec();
    return (a < 0.0) ? 0.0 : a;
  };

  const double age_scan = age_of(has_last_scan_, last_scan_stamp_);
  const double age_odom = age_of(has_last_odom_msg_, last_odom_stamp_);
  const double age_imu = age_of(has_last_imu_, last_imu_stamp_);

  if (out_age_scan)
    *out_age_scan = age_scan;
  if (out_age_odom)
    *out_age_odom = age_odom;
  if (out_age_imu)
    *out_age_imu = age_imu;

  // warn
  if (cfg_.watchdog_require_scan && age_scan > cfg_.watchdog_scan_warn_sec &&
      age_scan <= cfg_.watchdog_scan_fail_sec) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "[LOC] watchdog warn: scan age=" << FmtD(age_scan, 3) << "s");
  }
  if (cfg_.watchdog_require_odom && age_odom > cfg_.watchdog_odom_warn_sec &&
      age_odom <= cfg_.watchdog_odom_fail_sec) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] watchdog warn: odom age="
                                      << FmtD(age_odom, 3)
                                      << "s topic=" << odom_topic_);
  }
  if (cfg_.watchdog_require_imu && age_imu > cfg_.watchdog_imu_warn_sec &&
      age_imu <= cfg_.watchdog_imu_fail_sec) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] watchdog warn: imu age="
                                      << FmtD(age_imu, 3)
                                      << "s topic=" << imu_topic_);
  }

  // fail
  std::vector<std::string> bad;
  if (cfg_.watchdog_require_scan && age_scan > cfg_.watchdog_scan_fail_sec)
    bad.push_back("scan");
  if (cfg_.watchdog_require_odom && age_odom > cfg_.watchdog_odom_fail_sec)
    bad.push_back("odom");
  if (cfg_.watchdog_require_imu && age_imu > cfg_.watchdog_imu_fail_sec)
    bad.push_back("imu");

  if (!bad.empty()) {
    uint8_t r = kReasonInputMultiTimeout;
    if (bad.size() == 1) {
      if (bad[0] == "scan")
        r = kReasonInputScanTimeout;
      else if (bad[0] == "odom")
        r = kReasonInputOdomTimeout;
      else if (bad[0] == "imu")
        r = kReasonInputImuTimeout;
    }

    if (out_reason)
      *out_reason = r;

    if (out_detail) {
      std::ostringstream oss;
      oss << "missing=" << JoinStr(bad, "+")
          << " age(scan/odom/imu)=" << FmtD(age_scan, 3) << "/"
          << FmtD(age_odom, 3) << "/" << FmtD(age_imu, 3) << "s";
      *out_detail = oss.str();
    }
    return false;
  }

  return true;
}

void NdtLocalizerNode::forceFail_(const ros::Time &now, uint8_t reason,
                                  const std::string &detail) {
  const bool rising_edge = (!sensor_force_fail_);

  sensor_force_fail_ = true;
  sensor_force_reason_ = reason;
  sensor_force_detail_ = detail;

  if (cfg_.watchdog_fail_hold_sec > 0.0) {
    sensor_force_until_ = now + ros::Duration(cfg_.watchdog_fail_hold_sec);
  } else {
    sensor_force_until_ = ros::Time(0);
  }

  Eigen::Matrix4f T_out = Eigen::Matrix4f::Identity();
  if (has_last_published_) {
    T_out = T_last_published_;
  } else {
    (void)predictor_.predict(T_out);
  }

  // 把运行时状态机也打入 FAIL（只在边沿做一次）
  if (rising_edge) {
    prev_loc_level_ = loc_level_;
    loc_level_ = LocLevel::FAIL;

    // 记录 fail anchor（用于 scanCallback 里的 fail recovery scheduler）
    if (has_last_odom_msg_) {
      const Eigen::Vector2f odom_xy(
          static_cast<float>(last_odom_msg_.pose.pose.position.x),
          static_cast<float>(last_odom_msg_.pose.pose.position.y));
      fail_anchor_odom_xy_ = odom_xy;
      fail_attempt_anchor_odom_xy_ = odom_xy;
      has_fail_anchor_ = true;
      has_fail_attempt_anchor_ = true;
    } else {
      has_fail_anchor_ = false;
      has_fail_attempt_anchor_ = false;
    }

    fail_stationary_start_ = ros::Time(0);
    fail_last_periodic_local_ = ros::Time(0);
    fail_moved_far_since_edge_ = false;

    // 触发一次 fail edge 的 local hint（如果启用了 local_reloc / reloc_mgr）
    if (cfg_.local_reloc_enable) {
      RelocPose2D c;
      if (has_last_good_pose_) {
        c = last_good_pose_;
      } else {
        c.x = T_out(0, 3);
        c.y = T_out(1, 3);
        c.yaw = std::atan2(T_out(1, 0), T_out(0, 0));
      }
      reloc_mgr_.onFailEdge(c);
    }
  }

  {
    NdtQualityMetrics q{};
    publishStatus(now, q, localization_msgs::LocalizationStatus::MODE_FAIL,
                  reason, false, false);
  }
  publishOdom(now, T_out);

  ROS_ERROR_STREAM_THROTTLE(
      1.0, "[LOC] watchdog FAIL reason=" << int(reason) << " " << detail);
}


void NdtLocalizerNode::clearForceFailIfRecovered_(const ros::Time &now) {
  if (!sensor_force_fail_) return;

  if (!sensor_force_until_.isZero() && now < sensor_force_until_) return;

  uint8_t r = 0;
  std::string d;
  if (!watchdogCheck_(now, &r, &d)) return;

  sensor_force_fail_ = false;
  sensor_force_reason_ = 0;
  sensor_force_detail_.clear();
  sensor_force_until_ = ros::Time(0);

  // 恢复时重同步 predictor，修正“驱动重启后 odom 参考系跳变”
  if (has_last_published_ && has_last_odom_msg_) {
    predictor_.resetWithCorrection(T_last_published_, last_odom_msg_);
  }

  // 让 temporal 在恢复后几帧别太敏感
  if (cfg_.temporal_grace_enable) {
    temporal_grace_until_ = now + ros::Duration(cfg_.temporal_grace_duration);
  }
  has_last_temporal_ = false;  // 避免拿“掉线前的 temporal cache”做速度判别

  degen_dr_hold_active_ = false;
  degen_dr_hold_frames_left_ = 0;
  degen_dr_hold_trigger_count_ = 0;
  degen_dr_hold_recover_count_ = 0;
  degen_dr_cruise_active_ = false;
  degen_dr_cruise_trigger_count_ = 0;
  degen_dr_cruise_frames_ = 0;
  degen_dr_cruise_dist_accum_ = 0.0;
  degen_dr_cruise_has_last_xy_ = false;
  degen_dr_cruise_last_xy_ = Eigen::Vector2f::Zero();
  degen_active_latched_ = false;
  degen_active_streak_ = 0;
  degen_hessian_miss_streak_ = 0;
  degen_hold_frames_left_ = 0;
  degen_exit_safe_streak_ = 0;
  has_degen_last_motion_dir_ = false;
  degen_last_motion_dir_ = Eigen::Vector2f(1.0f, 0.0f);
  intensity_prefilter_adaptive_frames_left_ = 0;

  ROS_WARN_STREAM_THROTTLE(1.0, "[LOC] watchdog recovered");
}


void NdtLocalizerNode::watchdogTimerCb_(const ros::TimerEvent & /*ev*/) {
  if (!cfg_.watchdog_enable || !algo_enabled_)
    return;

  const ros::Time now = ros::Time::now();
  if (now.isZero())
    return;

  uint8_t r = 0;
  std::string d;
  if (!watchdogCheck_(now, &r, &d)) {
    forceFail_(now, r, d);
  } else {
    clearForceFailIfRecovered_(now);
  }
}

// ===================== reset + config summary =====================

void NdtLocalizerNode::resetRuntimeState_() {
  initial_pose_applied_ = false;
  has_last_odom_msg_ = false;
  has_last_imu_ = false;
  latest_linear_speed_ = 0.0;
  stationary_phase_ = false;
  stationary_pose_written_ = false;
  stationary_start_time_ = ros::Time(0);
  init_stage_ = InitStage::FAST;

  loc_level_ = LocLevel::INITIALIZING;
  dr_distance_since_last_ndt_ = 0.0;
  dr_frames_since_last_ndt_ = 0;
  last_output_position_ = Eigen::Vector3f::Zero();
  has_last_output_position_ = false;
  prev_output_position_ = Eigen::Vector3f::Zero();
  has_prev_output_position_ = false;
  has_ever_had_good_ndt_ = false;

  degen_dr_hold_active_ = false;
  degen_dr_hold_frames_left_ = 0;
  degen_dr_hold_trigger_count_ = 0;
  degen_dr_hold_recover_count_ = 0;
  degen_dr_cruise_active_ = false;
  degen_dr_cruise_trigger_count_ = 0;
  degen_dr_cruise_frames_ = 0;
  degen_dr_cruise_dist_accum_ = 0.0;
  degen_dr_cruise_has_last_xy_ = false;
  degen_dr_cruise_last_xy_ = Eigen::Vector2f::Zero();
  degen_active_latched_ = false;
  degen_active_streak_ = 0;
  degen_hessian_miss_streak_ = 0;
  degen_hold_frames_left_ = 0;
  degen_exit_safe_streak_ = 0;
  has_degen_last_motion_dir_ = false;
  degen_last_motion_dir_ = Eigen::Vector2f(1.0f, 0.0f);
  intensity_prefilter_adaptive_frames_left_ = 0;

  has_last_temporal_ = false;
  has_last_published_ = false;
  temporal_grace_until_ = ros::Time(0);

  init_exhausted_ = false;
  init_next_idx_ = 0;
  has_init_exhausted_anchor_ = false;
  init_last_rearm_time_ = ros::Time(0);
  init_rearm_stationary_start_ = ros::Time(0);

  // FAIL recovery runtime
  fail_stationary_start_ = ros::Time(0);
  fail_last_periodic_local_ = ros::Time(0);
  has_fail_anchor_ = false;
  has_fail_attempt_anchor_ = false;
  fail_moved_far_since_edge_ = false;

  init_candidates_.clear();
  init_tried_.clear();
  init_next_idx_ = 0;
  init_exhausted_ = false;
}

void NdtLocalizerNode::logEffectiveConfig_(const std::string &map_name,
                                           const std::string &map_stem) const {
  std::ostringstream oss;
  oss << "\n"
      << "==================== [LOC] Effective Config ====================\n"
      << "Run     : map=" << map_name
      << " layer=" << map_layer_ << "\n"
      << "Topics  : scan=" << scan_topic_ << " odom=" << odom_topic_
      << " imu=" << imu_topic_ << "\n"
      << "Frames  : base=" << base_frame_id_ << " map=map\n"
      << "Map     : json=" << map_json_path_ << " (stem=" << map_stem << ")\n"
      << "NDT     : resolution=" << FmtD(cfg_.ndt_resolution, 3)
      << " threads=" << cfg_.ndt_num_threads << "\n"
      << "Quality : enable=" << (cfg_.quality_enable ? 1 : 0)
      << " tp_bad=" << FmtD(cfg_.quality_trans_prob_bad, 3)
      << " tp_good=" << FmtD(cfg_.quality_trans_prob_good, 3)
      << " max_dT=" << FmtD(cfg_.quality_max_delta_trans, 3)
      << " max_dYaw_deg=" << FmtD(cfg_.quality_max_delta_yaw_deg, 2) << "\n"
      << "Degen   : enable=" << (cfg_.degen_fusion_enable ? 1 : 0)
      << " force=" << (cfg_.degen_force_active_always ? 1 : 0)
      << " min_pts=" << cfg_.degen_min_points
      << " cond(ok/bad/detect/off_unused)=" << FmtD(cfg_.degen_cond_ok, 1) << "/"
      << FmtD(cfg_.degen_cond_bad, 1) << "/"
      << FmtD(cfg_.degen_cond_hyst_on, 1) << "/"
      << FmtD(cfg_.degen_cond_hyst_off, 1)
      << " lmin=" << FmtD(cfg_.degen_lambda_min_thresh, 6)
      << " alpha(w/s/y)=" << FmtD(cfg_.degen_alpha_w_min, 2) << "/"
      << FmtD(cfg_.degen_alpha_s, 2) << "/" << FmtD(cfg_.degen_alpha_yaw, 2)
      << " temp_rej_dw=" << FmtD(cfg_.degen_temp_reject_enter_dw_m, 3)
      << " state(process/exit)=" << kDegenProcessFrames << "/"
      << kDegenExitSafeFrames
      << "\n"
      << "          amb(en=" << (cfg_.degen_ambiguity_enable ? 1 : 0)
      << " off=" << FmtD(cfg_.degen_ambiguity_seed_offset_m, 2)
      << " score_m=" << FmtD(cfg_.degen_ambiguity_score_margin, 4)
      << " tp_m=" << FmtD(cfg_.degen_ambiguity_tp_margin, 3)
      << " sep(w/s)=" << FmtD(cfg_.degen_ambiguity_weak_sep_m, 2) << "/"
      << FmtD(cfg_.degen_ambiguity_strong_sep_m, 2)
      << " accept(dw/ds/dyaw_deg)=" << FmtD(cfg_.degen_amb_accept_dw_m, 2)
      << "/" << FmtD(cfg_.degen_amb_accept_ds_m, 2)
      << "/" << FmtD(cfg_.degen_amb_accept_dyaw_rad * 180.0 / kPi, 1)
      << " iters=" << cfg_.degen_ambiguity_ndt_max_iters
      << " alpha_amb(w/s/y)=" << FmtD(cfg_.degen_alpha_w_ambiguity, 2) << "/"
      << FmtD(cfg_.degen_alpha_s_ambiguity, 2) << "/"
      << FmtD(cfg_.degen_alpha_yaw_ambiguity, 2)
      << " block(anchor/last)=" << (cfg_.degen_block_anchor_on_ambiguity ? 1 : 0)
      << "/" << (cfg_.degen_block_last_pose_on_ambiguity ? 1 : 0)
      << " hold(en/t/h/r)=" << (cfg_.degen_dr_hold_enable ? 1 : 0)
      << "/" << cfg_.degen_dr_hold_trigger_frames
      << "/" << cfg_.degen_dr_hold_frames
      << "/" << cfg_.degen_dr_hold_recover_frames
      << " cruise(en/t/dist/f/mv/yw/a/h)="
      << (cfg_.degen_dr_cruise_enable ? 1 : 0)
      << "/" << cfg_.degen_dr_cruise_trigger_frames
      << "/" << FmtD(cfg_.degen_dr_cruise_hold_dist_m, 2)
      << "/" << cfg_.degen_dr_cruise_max_frames
      << "/" << FmtD(cfg_.degen_dr_cruise_min_speed_mps, 2)
      << "/" << FmtD(cfg_.degen_dr_cruise_max_yaw_rate_deg, 1)
      << "/" << (cfg_.degen_dr_cruise_require_ambiguity ? 1 : 0)
      << "/" << (cfg_.degen_dr_cruise_require_hard ? 1 : 0)
      << ")\n"
      << "Temporal: enable=" << (cfg_.temporal_check_enable ? 1 : 0)
      << " max_dt=" << FmtD(cfg_.temporal_max_dt, 3)
      << " max_v=" << FmtD(cfg_.temporal_max_lin_vel, 3)
      << " max_w_deg=" << FmtD(cfg_.temporal_max_ang_vel_deg, 2) << "\n"
      << "          backtrack(en/min_v/dir_step/max_back)="
      << (cfg_.temporal_reject_backtrack_enable ? 1 : 0)
      << "/" << FmtD(cfg_.temporal_reject_backtrack_min_speed_mps, 2)
      << "/" << FmtD(cfg_.temporal_reject_backtrack_dir_min_step_m, 3)
      << "/" << FmtD(cfg_.temporal_reject_backtrack_max_m, 3) << "\n"
      << "          odom_max_age=" << FmtD(cfg_.temporal_odom_max_age, 3)
      << " imu_max_age=" << FmtD(cfg_.temporal_imu_max_age, 3) << "\n"
      << "          move_min=" << FmtD(cfg_.temporal_move_min, 3)
      << " stuck_max=" << FmtD(cfg_.temporal_stuck_max, 3)
      << " stop_max=" << FmtD(cfg_.temporal_stop_max, 3)
      << " jump_when_stop=" << FmtD(cfg_.temporal_jump_when_stop, 3) << "\n"
      << "          ang_move_min_deg="
      << FmtD(cfg_.temporal_ang_move_min_deg, 2)
      << " ang_stuck_max_deg=" << FmtD(cfg_.temporal_ang_stuck_max_deg, 2)
      << " ang_jump_when_stop_deg="
      << FmtD(cfg_.temporal_ang_jump_when_stop_deg, 2) << "\n"
      << "Grace   : enable=" << (cfg_.temporal_grace_enable ? 1 : 0)
      << " dur=" << FmtD(cfg_.temporal_grace_duration, 3) << "\n"
      << "Scan    : range=[" << FmtD(cfg_.scan_min_range, 2) << ", "
      << FmtD(cfg_.scan_max_range, 2) << "]\n"
      << "Inten   : en=" << (cfg_.intensity_prefilter_enable ? 1 : 0)
      << " p=" << FmtD(cfg_.intensity_prefilter_range_comp_p, 2)
      << " norm=[" << FmtD(cfg_.intensity_prefilter_norm_min, 1)
      << ", " << FmtD(cfg_.intensity_prefilter_norm_max, 1) << "]"
      << " keep=" << FmtD(cfg_.intensity_prefilter_keep_ratio, 2)
      << " min_pts=" << cfg_.intensity_prefilter_min_keep_points
      << " adapt(en/keep/f)="
      << (cfg_.intensity_prefilter_adaptive_enable ? 1 : 0) << "/"
      << FmtD(cfg_.intensity_prefilter_adaptive_keep_ratio, 2) << "/"
      << cfg_.intensity_prefilter_adaptive_frames << "\n"
      << "Station : v_thresh=" << FmtD(cfg_.stationary_speed_thresh, 4)
      << " min_dur=" << FmtD(cfg_.stationary_min_duration, 2) << "\n"
      << "DR      : follow_dist=" << FmtD(cfg_.dr_follow_dist_thresh, 2)
      << " fail_dist=" << FmtD(cfg_.dr_fail_dist_thresh, 2)
      << " fail_frames=" << cfg_.dr_frame_fail_thresh << "\n"
      << "Init    : tp_min=" << FmtD(cfg_.init_trans_prob_min, 3)
      << " tp_soft=" << FmtD(cfg_.init_tp_soft, 3)
      << " score_max=" << FmtD(cfg_.init_score_max, 6)
      << " score_relaxed=" << FmtD(cfg_.init_score_relaxed_max, 6)
      << " K=" << cfg_.init_max_candidates_per_scan
      << " try_once=" << (cfg_.init_try_once_per_candidate ? 1 : 0) << "\n"
      << "InitRearm: enable=" << (cfg_.init_rearm_enable ? 1 : 0)
      << " dist=" << FmtD(cfg_.init_rearm_dist, 2)
      << " stop_dur=" << FmtD(cfg_.init_rearm_min_stop_duration, 2)
      << " cooldown=" << FmtD(cfg_.init_rearm_cooldown, 2) << "\n"
      << "StampSan: max_skew=" << FmtD(cfg_.stamp_max_skew_sec, 3)
      << " future_tol=" << FmtD(cfg_.stamp_future_tol_sec, 3)
      << " mono_eps=" << FmtD(cfg_.stamp_monotonic_eps_sec, 6) << "\n"
      << "Viz     : scan_cloud=" << (cfg_.viz_publish_scan_cloud ? 1 : 0)
      << " topic=" << cfg_.viz_scan_cloud_topic << "\n"
      << "LocalReloc: enable=" << (cfg_.local_reloc_enable ? 1 : 0)
      << " map_yaml=" << cfg_.local_reloc_map_yaml << "\n"
      << "           base_win(xy=" << FmtD(cfg_.local_opt_base.xy_range_m, 2)
      << " yaw_deg=" << FmtD(cfg_.local_opt_base.yaw_range_rad * 180.0 / kPi, 1)
      << ")"
      << " yaw_deg=" << FmtD(cfg_.local_opt_base.yaw_step_rad * 180.0 / kPi, 2)
      << ")"
      << " pyr=" << cfg_.local_opt_base.pyr_max_level
      << " bnb=" << cfg_.local_opt_base.bnb_max_level << "\n"
      << "RelocMgr: enable=" << (cfg_.reloc_mgr_params.enable ? 1 : 0)
      << " cooldown=" << FmtD(cfg_.reloc_mgr_params.cooldown_sec, 2)
      << " fail_win(xy=" << FmtD(cfg_.reloc_mgr_params.fail_xy_range_m, 2)
      << " yaw_deg="
      << FmtD(cfg_.reloc_mgr_params.fail_yaw_range_rad * 180.0 / kPi, 1)
      << ")\n"
      << "Watchdog: enable=" << (cfg_.watchdog_enable ? 1 : 0)
      << " require(scan/odom/imu)=" << (cfg_.watchdog_require_scan ? 1 : 0)
      << "/" << (cfg_.watchdog_require_odom ? 1 : 0) << "/"
      << (cfg_.watchdog_require_imu ? 1 : 0)
      << " hz=" << FmtD(cfg_.watchdog_timer_hz, 1) << "\n"
      << "          warn(scan/odom/imu)="
      << FmtD(cfg_.watchdog_scan_warn_sec, 2) << "/"
      << FmtD(cfg_.watchdog_odom_warn_sec, 2) << "/"
      << FmtD(cfg_.watchdog_imu_warn_sec, 2)
      << " fail=" << FmtD(cfg_.watchdog_scan_fail_sec, 2) << "/"
      << FmtD(cfg_.watchdog_odom_fail_sec, 2) << "/"
      << FmtD(cfg_.watchdog_imu_fail_sec, 2)
      << " hold=" << FmtD(cfg_.watchdog_fail_hold_sec, 2) << "\n"
      << "=================================================================\n";

  ROS_INFO_STREAM(oss.str());
}

} // namespace localization_ndt

int main(int argc, char **argv) {
  ros::init(argc, argv, "ndt_localizer_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try {
    localization_ndt::NdtLocalizerNode node(nh, pnh);
    node.spin();
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("Exception in NdtLocalizerNode: " << e.what());
    return 1;
  }
  return 0;
}
