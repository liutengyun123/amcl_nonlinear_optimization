#pragma once

#include <ros/time.h>
#include "localization_ndt/local_relocalizer_2d.hpp"

namespace localization_ndt {

enum class RelocTrigger : int {
  NONE = 0,
  MANUAL = 1,
  FAIL_EDGE = 2,
  DR_FAIL = 3,
  INIT_AROUND_CAND = 4
};

struct RelocHint {
  RelocTrigger trig = RelocTrigger::NONE;
  RelocPose2D center;
  double xy_range_m = 0.0;
  double yaw_range_rad = 0.0;
  double override_score_margin = -1.0; // >=0:覆盖 local score_margin
  bool bypass_cooldown = false;
};

struct RelocMgrParams {
  bool enable = true;
  double cooldown_sec = 1.0;

  // FAIL/DR 默认窗口
  double fail_xy_range_m = 2.0;
  double fail_yaw_range_rad = 20.0 * kPiReloc / 180.0;

  // MANUAL：cov -> window
  double manual_xy_min_m = 0.30;
  double manual_xy_sigma_k = 3.0;
  double manual_yaw_min_rad = 6.0 * kPiReloc / 180.0;
  double manual_yaw_sigma_k = 3.0;
  // 尝试次数，一个 hint 最多尝试次数（<=0 表示无限）
  int max_attempts = 5;
};

class RelocalizationManager {
public:
  void setParams(const RelocMgrParams& p) { p_ = p; }

  // 手动拖拽触发：covariance -> window
  void onManual(const RelocPose2D& center,
                double cov_xx, double cov_yy, double cov_yaw);

  // FAIL 边沿触发
  void onFailEdge(const RelocPose2D& center);

  // 可扩展：DR fail 触发
  void onDrFail(const RelocPose2D& center);

  bool hasPending() const { return pending_; }

  bool peek(const ros::Time& now, RelocHint* out);
  void ack();

private:
  RelocMgrParams p_;
  bool pending_ = false;
  RelocHint hint_;
  ros::Time last_attempt_;
  int attempts_ = 0;        // 已尝试次数（peek 成功一次就 +1）
};

}  // namespace localization_ndt
