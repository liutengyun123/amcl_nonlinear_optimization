#pragma once

#include <Eigen/Core>

#include "localization_ndt/types.hpp"
#include "localization_ndt/distance_field_2d.hpp"

namespace localization_ndt {

struct DynamicFilterConfig {
  bool enable = true;

  // DistanceField params
  double grid_res_m = 0.10;
  double inflate_m  = 0.20;
  double padding_m  = 8.0;

  // keep policy (fallback / second stage)
  double keep_ratio = 0.50;      // 0~1
  int    min_keep_points = 200;  // 保底最少点数
  double hard_max_dist_m = -1.0; // >0 则 d>hard_max 直接丢

  // Ray-consistency
  bool   ray_enable = true;
  double ray_margin_near_m = 0.20; // r_obs < r_map - margin => reject
  double ray_max_range_m   = 35.0; // 射线预测最大距离
  double ray_hit_eps_m     = -1.0; // <0 => 0.5*grid_res
  double ray_step_m       = -1.0;  // <0 => 0.5*grid_res 固定步长
  int    ray_max_steps     = 256;

  double ray_confirm_df_m = 0.15;  // 二次确认：点到地图距离 > 该值 才允许按 ray 判动态

  // Front-sector protection in base frame.
  // Used to avoid removing near-forward structure that often anchors longitudinal motion.
  bool   front_protect_enable = true;
  double front_protect_x_min_m = 0.10;
  double front_protect_x_max_m = 3.00;
  double front_protect_abs_y_max_m = 0.80;
  // If true, use +Y/-Y as forward axis; otherwise use +X/-X.
  bool   front_protect_use_y_axis = false;
  // >=0 means +axis is forward, <0 means -axis is forward.
  double front_protect_forward_sign = 1.0;
  double front_protect_max_drop_ratio = 0.45;
  int    front_protect_min_in_points = 12;


};

struct DynamicFilterStats {
  int in = 0;
  int kept = 0;
  double keep_ratio = 1.0;

  double dist_p50 = 0.0;
  double dist_p90 = 0.0;

  bool insufficient_static = false;

  int ray_tested = 0;
  int ray_rejected = 0;

  double rmap_p50 = 0.0;
  double rmap_p90 = 0.0;

  int front_in = 0;
  int front_kept = 0;
  double front_drop_ratio = 0.0;
  bool front_recovered = false;

};

class DynamicPointFilter {
public:
  void setConfig(const DynamicFilterConfig& c) { cfg_ = c; }
  const DynamicFilterConfig& config() const { return cfg_; }

  bool setMap(const PointCloudT::ConstPtr& map);
  bool isReady() const { return df_.isReady(); }
  PointCloudT::Ptr filter(const PointCloudT::ConstPtr& scan_base,
                        const Eigen::Matrix4f& T_map_base_guess,
                        const Eigen::Vector2f& sensor_origin_base = Eigen::Vector2f::Zero(),
                        DynamicFilterStats* stats = nullptr) const;


private:
  DynamicFilterConfig cfg_;
  DistanceField2D df_;
};

}  // namespace localization_ndt
