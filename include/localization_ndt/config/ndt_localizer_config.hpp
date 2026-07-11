#pragma once

#include <ros/ros.h>
#include <string>

#include "localization_ndt/local_relocalizer_2d.hpp"
#include "localization_ndt/relocalization_manager.hpp"

namespace localization_ndt {
namespace config {

struct NdtLocalizerConfig {

  // Dynamic point filter
  bool  dyn_filter_enable = true;
  double dyn_grid_res_m = 0.10;
  double dyn_inflate_m = 0.20;
  double dyn_keep_ratio = 0.60;
  double dyn_padding_m = 5.0;
  int    dyn_min_keep_points = 200;
  double dyn_hard_max_dist_m = 2.0;

    // Dynamic point filter - Ray-consistency
  bool   dyn_ray_enable = true;
  double dyn_ray_margin_near_m = 0.20;
  double dyn_ray_max_range_m   = 35.0;
  double dyn_ray_hit_eps_m     = -1.0;   // <0 => 0.5*grid_res
  int    dyn_ray_max_steps     = 256;
  double dyn_ray_confirm_df_m  = 0.15;   // d(df) <= confirm => protect (no ray reject)
  double dyn_ray_step_m        = -1.0;   // <0 => 0.5*grid_res

  // Dynamic point filter - Front-sector protection
  bool   dyn_front_protect_enable = true;
  double dyn_front_protect_x_min_m = 0.10;
  double dyn_front_protect_x_max_m = 3.00;
  double dyn_front_protect_abs_y_max_m = 0.80;
  bool   dyn_front_protect_use_y_axis = false;
  double dyn_front_protect_forward_sign = 1.0;
  double dyn_front_protect_max_drop_ratio = 0.45;
  int    dyn_front_protect_min_in_points = 12;


  // NDT
  double ndt_resolution = 1.0;
  int ndt_num_threads = 4;
  // Whether to filter occupied-unknown interfaces using freeBand when
  // building NDT target cloud from JSON map.
  bool ndt_target_use_known_mask_filter = false;

  // Quality
  bool quality_enable = true;
  double quality_max_delta_trans = 1.5;
  double quality_max_delta_yaw_deg = 20.0;
  double quality_trans_prob_bad = 1.8;
  double quality_trans_prob_good = 1.5;

  // Degeneration-aware fusion (corridor / weak-observable direction)
  bool degen_fusion_enable = true;
  // Force degeneration handling always on (for validation / A-B testing).
  bool degen_force_active_always = false;
  int degen_min_points = 80;
  double degen_cov_eps = 1e-6;
  double degen_cond_ok = 80.0;
  double degen_cond_bad = 350.0;
  // Hysteresis for degen activation to avoid boundary toggling.
  double degen_cond_hyst_on = 65.0;
  double degen_cond_hyst_off = 60.0;
  double degen_lambda_min_thresh = 5e-4;
  double degen_alpha_w_min = 0.10;
  double degen_alpha_s = 0.95;
  double degen_alpha_yaw = 0.90;

  bool degen_ambiguity_enable = true;
  double degen_ambiguity_seed_offset_m = 0.80;
  double degen_ambiguity_score_margin = 0.003;
  double degen_ambiguity_tp_margin = 0.08;
  double degen_ambiguity_weak_sep_m = 0.50;
  double degen_ambiguity_strong_sep_m = 0.50;
  int degen_ambiguity_ndt_max_iters = 6;

  // Consecutive hard-degen + ambiguity guard: short DR hold
  bool degen_dr_hold_enable = true;
  int degen_dr_hold_trigger_frames = 3;
  int degen_dr_hold_frames = 5;
  int degen_dr_hold_recover_frames = 2;

  // Corridor cruise DR: when degeneration persists while moving straight,
  // force DR for a short distance to avoid "stuck in place" behavior.
  bool degen_dr_cruise_enable = true;
  int degen_dr_cruise_trigger_frames = 8;
  double degen_dr_cruise_hold_dist_m = 1.5;
  int degen_dr_cruise_max_frames = 80;
  double degen_dr_cruise_min_speed_mps = 0.15;
  double degen_dr_cruise_max_yaw_rate_deg = 20.0;
  bool degen_dr_cruise_require_ambiguity = true;
  bool degen_dr_cruise_require_hard = false;

  // If ambiguity is high but NDT stays close to prediction, skip DR hold.
  double degen_amb_accept_dw_m = 1.00;
  double degen_amb_accept_ds_m = 0.30;
  double degen_amb_accept_dyaw_rad = 0.174532925; // 10 deg
  // Temporal reject fallback: if weak-direction jump is too large, force-enter
  // degen handling immediately even when cond has not reached detect threshold.
  double degen_temp_reject_enter_dw_m = 0.15;

  double degen_alpha_w_ambiguity = 0.0;
  double degen_alpha_s_ambiguity = 1.0;
  double degen_alpha_yaw_ambiguity = 0.0;
  bool degen_block_anchor_on_ambiguity = true;
  bool degen_block_last_pose_on_ambiguity = true;

  // Temporal
  bool temporal_check_enable = true;
  double temporal_max_dt = 1.0;
  double temporal_max_lin_vel = 2.5;
  double temporal_max_ang_vel_deg = 180.0;
  // Reject physically-impossible immediate backtrack while moving forward.
  bool temporal_reject_backtrack_enable = true;
  double temporal_reject_backtrack_min_speed_mps = 0.15;
  double temporal_reject_backtrack_dir_min_step_m = 0.005;
  double temporal_reject_backtrack_max_m = 0.08;

  double temporal_odom_max_age = 0.2;
  double temporal_imu_max_age = 0.2;

  double temporal_move_min = 0.10;
  double temporal_stuck_max = 0.03;
  double temporal_stop_max = 0.03;
  double temporal_jump_when_stop = 0.30;

  double temporal_ang_move_min_deg = 10.0;
  double temporal_ang_stuck_max_deg = 3.0;
  double temporal_ang_jump_when_stop_deg = 30.0;

  // Scan
  double scan_min_range = 1.0;
  double scan_max_range = 35.0;

  // Intensity prefilter (minimal viable): distance-compensated top-ratio keep
  bool intensity_prefilter_enable = true;
  double intensity_prefilter_range_comp_p = 2.0;
  double intensity_prefilter_norm_min = 0.0;
  double intensity_prefilter_norm_max = 1e6;
  double intensity_prefilter_keep_ratio = 0.30;
  int intensity_prefilter_min_keep_points = 200;
  // Adaptive relaxation: when degeneration/ambiguity appears, temporarily
  // increase keep ratio for a few frames to restore geometry observability.
  bool intensity_prefilter_adaptive_enable = true;
  double intensity_prefilter_adaptive_keep_ratio = 0.80;
  int intensity_prefilter_adaptive_frames = 20;

  // Stationary
  double stationary_speed_thresh = 0.001;
  double stationary_min_duration = 2.0;

  // DR
  double dr_follow_dist_thresh = 5.0;
  double dr_fail_dist_thresh = 10.0;
  int dr_frame_fail_thresh = 300;

  // Grace
  bool temporal_grace_enable = true;
  double temporal_grace_duration = 0.8;

  // Init
  double init_trans_prob_min = 1.9;
  int init_max_candidates_per_scan = 3;
  bool init_try_once_per_candidate = true;
  double init_dedupe_xy_eps = 1e-3;
  double init_dedupe_yaw_eps = 1e-3;

  bool init_score_soft_enable = true;
  double init_score_max = 0.009;
  double init_tp_soft = 1.8;
  double init_score_relaxed_max = 0.01;

  bool init_rearm_enable = true;
  double init_rearm_dist = 2.0;
  double init_rearm_min_stop_duration = 2.0;
  double init_rearm_cooldown = 8.0;

  // Stamp sanitize
  double stamp_max_skew_sec = 0.20;
  double stamp_future_tol_sec = 0.05;
  double stamp_monotonic_eps_sec = 1e-6;

  // Viz
  bool viz_publish_scan_cloud = false;
  std::string viz_scan_cloud_topic = "scan_cloud_map";

  // LocalReloc
  bool local_reloc_enable = true;
  std::string local_reloc_map_yaml; // 若空，loader 会补默认
  LocalRelocOptions local_opt_base;

  // RelocMgr
  RelocMgrParams reloc_mgr_params;

  // Watchdog
  bool watchdog_enable = true;
  bool watchdog_require_scan = true;
  bool watchdog_require_odom = true;
  bool watchdog_require_imu = true;
  double watchdog_timer_hz = 10.0;

  double watchdog_scan_warn_sec = 0.10;
  double watchdog_scan_fail_sec = 0.50;
  double watchdog_odom_warn_sec = 0.10;
  double watchdog_odom_fail_sec = 0.50;
  double watchdog_imu_warn_sec = 0.10;
  double watchdog_imu_fail_sec = 0.50;
  double watchdog_fail_hold_sec = 0.0;
};

bool LoadNdtLocalizerConfig(const std::string &pkg_path,
                            const std::string &params_dir,
                            const std::string &map_stem, ros::NodeHandle &pnh,
                            NdtLocalizerConfig *out);

} // namespace config
} // namespace localization_ndt
