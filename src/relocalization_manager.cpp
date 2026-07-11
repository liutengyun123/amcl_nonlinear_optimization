#include "localization_ndt/relocalization_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace localization_ndt {

static inline double clamp_pos(double v, double fallback) {
  if (!std::isfinite(v) || v < 0.0)
    return fallback;
  return v;
}

void RelocalizationManager::onManual(const RelocPose2D &center, double cov_xx,
                                     double cov_yy, double cov_yaw) {
  if (!p_.enable)
    return;

  const double sx = std::sqrt(clamp_pos(cov_xx, 0.0));
  const double sy = std::sqrt(clamp_pos(cov_yy, 0.0));
  const double sxy = std::max(sx, sy);

  const double syaw = std::sqrt(clamp_pos(cov_yaw, 0.0));

  RelocHint h;
  h.trig = RelocTrigger::MANUAL;
  h.center = center;

  // window = max(min, k*sigma)
  h.xy_range_m = std::max(p_.manual_xy_min_m, p_.manual_xy_sigma_k * sxy);
  h.yaw_range_rad =
      std::max(p_.manual_yaw_min_rad, p_.manual_yaw_sigma_k * syaw);

  // 手动阶段：score_margin 强制 0
  h.override_score_margin = 0.0;

  // 手动通常希望立刻生效，不受 cooldown 限制
  h.bypass_cooldown = true;

  hint_ = h;
  pending_ = true;
  attempts_ = 0;
  last_attempt_ = ros::Time(0);
}

void RelocalizationManager::onFailEdge(const RelocPose2D &center) {
  if (!p_.enable)
    return;

  RelocHint h;
  h.trig = RelocTrigger::FAIL_EDGE;
  h.center = center;
  h.xy_range_m = p_.fail_xy_range_m;
  h.yaw_range_rad = p_.fail_yaw_range_rad;
  h.override_score_margin = -1.0;
  h.bypass_cooldown = false;

  hint_ = h;
  pending_ = true;
  attempts_ = 0;
  last_attempt_ = ros::Time(0);
}

void RelocalizationManager::onDrFail(const RelocPose2D &center) {
  if (!p_.enable)
    return;

  RelocHint h;
  h.trig = RelocTrigger::DR_FAIL;
  h.center = center;
  h.xy_range_m = p_.fail_xy_range_m;
  h.yaw_range_rad = p_.fail_yaw_range_rad;
  h.override_score_margin = -1.0;
  h.bypass_cooldown = false;

  hint_ = h;
  pending_ = true;
  attempts_ = 0;
  last_attempt_ = ros::Time(0);
}

bool RelocalizationManager::peek(const ros::Time &now, RelocHint *out) {
  if (!p_.enable)
    return false;
  if (!pending_)
    return false;
  if (!out)
    return false;

  // max attempts: 超过就自动放弃这个 hint
  if (p_.max_attempts > 0 && attempts_ >= p_.max_attempts) {
    pending_ = false;
    attempts_ = 0;
    last_attempt_ = ros::Time(0);
    return false;
  }

  // cooldown（bypass_cooldown=true 的 hint 不受 cooldown 限制）
  if (!hint_.bypass_cooldown && !last_attempt_.isZero()) {
    const double dt = (now - last_attempt_).toSec();
    if (dt < p_.cooldown_sec)
      return false;
  }

  *out = hint_;

  last_attempt_ = now;
  attempts_++; // 只要 peek 成功返回一次，就算一次“尝试”
  return true; // 注意：不清 pending_
}
void RelocalizationManager::ack() {
  if (!p_.enable)
    return;
  pending_ = false;
  attempts_ = 0;
  // last_attempt_ 不清也可以，保持语义清理。
  last_attempt_ = ros::Time(0);
}

} // namespace localization_ndt
