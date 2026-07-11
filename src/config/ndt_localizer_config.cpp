#include "localization_ndt/config/ndt_localizer_config.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "localization_ndt/config/params_yaml.hpp"
#include "localization_ndt/config/yaml_utils.hpp"

namespace localization_ndt {
namespace config {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultTemporalGrace = 0.8;

inline void ClampPos(double &v, double lo, double fallback) {
  if (!std::isfinite(v) || v < lo)
    v = fallback;
}

inline int ClampMinInt(int v, int lo, int fallback) {
  if (v < lo)
    return fallback;
  return v;
}

static NdtLocalizerConfig MakeDefaultConfig() {
  NdtLocalizerConfig c;

  c.ndt_resolution = 1.0;
  c.ndt_num_threads = 4;
  c.ndt_target_use_known_mask_filter = false;

  c.dyn_filter_enable = true;
  c.dyn_grid_res_m = 0.10;
  c.dyn_inflate_m  = 0.20;
  c.dyn_padding_m  = 5.0;
  c.dyn_keep_ratio = 0.80;
  c.dyn_min_keep_points = 200;
  c.dyn_hard_max_dist_m = 2.0;

  c.dyn_ray_enable = true;
  c.dyn_ray_margin_near_m = 0.20;
  c.dyn_ray_max_range_m   = c.scan_max_range;  // 默认跟 scan_max_range 走
  c.dyn_ray_hit_eps_m     = -1.0;
  c.dyn_ray_max_steps     = 256;
  c.dyn_ray_confirm_df_m  = 0.15;
  c.dyn_ray_step_m        = -1.0;

  c.dyn_front_protect_enable = true;
  c.dyn_front_protect_x_min_m = 0.10;
  c.dyn_front_protect_x_max_m = 3.00;
  c.dyn_front_protect_abs_y_max_m = 0.80;
  c.dyn_front_protect_use_y_axis = false;
  c.dyn_front_protect_forward_sign = 1.0;
  c.dyn_front_protect_max_drop_ratio = 0.45;
  c.dyn_front_protect_min_in_points = 12;

  c.quality_enable = true;
  c.quality_max_delta_trans = 1.5;
  c.quality_max_delta_yaw_deg = 20.0;
  c.quality_trans_prob_bad = 1.8;
  c.quality_trans_prob_good = 1.5;

  c.degen_fusion_enable = true;
  c.degen_force_active_always = false;
  c.degen_min_points = 80;
  c.degen_cov_eps = 1e-6;
  c.degen_cond_ok = 80.0;
  c.degen_cond_bad = 350.0;
  c.degen_cond_hyst_on = 65.0;
  c.degen_cond_hyst_off = 60.0;
  c.degen_lambda_min_thresh = 5e-4;
  c.degen_alpha_w_min = 0.10;
  c.degen_alpha_s = 0.95;
  c.degen_alpha_yaw = 0.90;

  c.degen_ambiguity_enable = true;
  c.degen_ambiguity_seed_offset_m = 0.80;
  c.degen_ambiguity_score_margin = 0.003;
  c.degen_ambiguity_tp_margin = 0.08;
  c.degen_ambiguity_weak_sep_m = 0.50;
  c.degen_ambiguity_strong_sep_m = 0.50;
  c.degen_ambiguity_ndt_max_iters = 6;

  c.degen_dr_hold_enable = true;
  c.degen_dr_hold_trigger_frames = 3;
  c.degen_dr_hold_frames = 5;
  c.degen_dr_hold_recover_frames = 2;

  c.degen_dr_cruise_enable = true;
  c.degen_dr_cruise_trigger_frames = 8;
  c.degen_dr_cruise_hold_dist_m = 1.5;
  c.degen_dr_cruise_max_frames = 80;
  c.degen_dr_cruise_min_speed_mps = 0.15;
  c.degen_dr_cruise_max_yaw_rate_deg = 20.0;
  c.degen_dr_cruise_require_ambiguity = true;
  c.degen_dr_cruise_require_hard = false;

  c.degen_amb_accept_dw_m = 1.00;
  c.degen_amb_accept_ds_m = 0.30;
  c.degen_amb_accept_dyaw_rad = 10.0 * kPi / 180.0;
  c.degen_temp_reject_enter_dw_m = 0.15;

  c.degen_alpha_w_ambiguity = 0.0;
  c.degen_alpha_s_ambiguity = 1.0;
  c.degen_alpha_yaw_ambiguity = 0.0;
  c.degen_block_anchor_on_ambiguity = true;
  c.degen_block_last_pose_on_ambiguity = true;

  c.temporal_check_enable = true;
  c.temporal_max_dt = 1.0;
  c.temporal_max_lin_vel = 2.5;
  c.temporal_max_ang_vel_deg = 180.0;
  c.temporal_reject_backtrack_enable = true;
  c.temporal_reject_backtrack_min_speed_mps = 0.15;
  c.temporal_reject_backtrack_dir_min_step_m = 0.005;
  c.temporal_reject_backtrack_max_m = 0.08;
  c.temporal_odom_max_age = 0.2;
  c.temporal_imu_max_age = 0.2;

  c.temporal_move_min = 0.10;
  c.temporal_stuck_max = 0.03;
  c.temporal_stop_max = 0.03;
  c.temporal_jump_when_stop = 0.30;

  c.temporal_ang_move_min_deg = 10.0;
  c.temporal_ang_stuck_max_deg = 3.0;
  c.temporal_ang_jump_when_stop_deg = 30.0;

  c.scan_min_range = 1.0;
  c.scan_max_range = 35.0;
  c.intensity_prefilter_enable = true;
  c.intensity_prefilter_range_comp_p = 2.0;
  c.intensity_prefilter_norm_min = 0.0;
  c.intensity_prefilter_norm_max = 1e6;
  c.intensity_prefilter_keep_ratio = 0.30;
  c.intensity_prefilter_min_keep_points = 200;
  c.intensity_prefilter_adaptive_enable = true;
  c.intensity_prefilter_adaptive_keep_ratio = 0.80;
  c.intensity_prefilter_adaptive_frames = 20;

  c.stationary_speed_thresh = 0.001;
  c.stationary_min_duration = 2.0;

  c.dr_follow_dist_thresh = 5.0;
  c.dr_fail_dist_thresh = 10.0;
  c.dr_frame_fail_thresh = 300;

  c.temporal_grace_enable = true;
  c.temporal_grace_duration = kDefaultTemporalGrace;

  c.init_trans_prob_min = 1.9;
  c.init_max_candidates_per_scan = 3;
  c.init_try_once_per_candidate = true;
  c.init_dedupe_xy_eps = 1e-3;
  c.init_dedupe_yaw_eps = 1e-3;

  c.init_score_soft_enable = true;
  c.init_score_max = 0.009;
  c.init_tp_soft = 1.8;
  c.init_score_relaxed_max = 0.01;

  c.init_rearm_enable = true;
  c.init_rearm_dist = 2.0;
  c.init_rearm_min_stop_duration = 2.0;
  c.init_rearm_cooldown = 8.0;

  c.stamp_max_skew_sec = 0.20;
  c.stamp_future_tol_sec = 0.05;
  c.stamp_monotonic_eps_sec = 1e-6;

  c.viz_publish_scan_cloud = false;
  c.viz_scan_cloud_topic = "scan_cloud_map";

  c.local_reloc_enable = true;
  c.local_reloc_map_yaml = ""; // loader 后续会补默认路径
  c.local_opt_base = LocalRelocOptions{};
  c.local_opt_base.xy_range_m = 2.0;
  c.local_opt_base.yaw_range_rad = 20.0 * kPi / 180.0;
  c.local_opt_base.min_score = 0.20;
  c.local_opt_base.score_margin = 0.02;
  c.local_opt_base.min_valid_fraction = 0.30;
  c.local_opt_base.bnb_max_level = 3;
  c.local_opt_base.yaw_step_rad = 1.0 * kPi / 180.0;
  c.local_opt_base.max_scan_points = 1200;
  c.local_opt_base.scan_voxel_m = 0.02;
  c.local_opt_base.pyr_max_level = 6;
  c.local_opt_base.hit_sigma_m = 0.20;
  c.local_opt_base.max_dist_m = 1.00;

  // RelocMgr 默认（保持一致）
  c.reloc_mgr_params.enable = true;
  c.reloc_mgr_params.cooldown_sec = 1.0;
  c.reloc_mgr_params.fail_xy_range_m = 2.0;
  c.reloc_mgr_params.fail_yaw_range_rad = 20.0 * kPi / 180.0;
  c.reloc_mgr_params.manual_xy_min_m = 0.30;
  c.reloc_mgr_params.manual_xy_sigma_k = 3.0;
  c.reloc_mgr_params.manual_yaw_min_rad = 180.0 * kPi / 180.0;
  c.reloc_mgr_params.manual_yaw_sigma_k = 3.0;
  c.reloc_mgr_params.max_attempts = 5;

  // Watchdog 默认
  c.watchdog_enable = true;
  c.watchdog_require_scan = true;
  c.watchdog_require_odom = true;
  c.watchdog_require_imu = true;
  c.watchdog_timer_hz = 10.0;

  c.watchdog_scan_warn_sec = 0.10;
  c.watchdog_scan_fail_sec = 0.50;
  c.watchdog_odom_warn_sec = 0.10;
  c.watchdog_odom_fail_sec = 0.50;
  c.watchdog_imu_warn_sec = 0.10;
  c.watchdog_imu_fail_sec = 0.50;
  c.watchdog_fail_hold_sec = 0.0;

  return c;
}

} // namespace

bool LoadNdtLocalizerConfig(const std::string &pkg_path,
                            const std::string &params_dir,
                            const std::string &map_stem, ros::NodeHandle &pnh,
                            NdtLocalizerConfig *out) {
  if (!out)
    return false;
  *out = MakeDefaultConfig();

  YAML::Node y = LoadOrCreateParamsYaml(params_dir, map_stem);

  // --- YAML overrides ---
  using localization_ndt::config::LoadFieldFromYaml;

  LoadFieldFromYaml(y, "ndt_resolution", out->ndt_resolution);
  LoadFieldFromYaml(y, "ndt_num_threads", out->ndt_num_threads);
  LoadFieldFromYaml(y, "ndt_target_use_known_mask_filter",
                    out->ndt_target_use_known_mask_filter);

  LoadFieldFromYaml(y, "dyn_filter_enable", out->dyn_filter_enable);
  LoadFieldFromYaml(y, "dyn_grid_res_m", out->dyn_grid_res_m);
  LoadFieldFromYaml(y, "dyn_inflate_m", out->dyn_inflate_m);
  LoadFieldFromYaml(y, "dyn_keep_ratio", out->dyn_keep_ratio);
  LoadFieldFromYaml(y, "dyn_padding_m", out->dyn_padding_m);
  LoadFieldFromYaml(y, "dyn_min_keep_points", out->dyn_min_keep_points);
  LoadFieldFromYaml(y, "dyn_hard_max_dist_m", out->dyn_hard_max_dist_m);

  LoadFieldFromYaml(y, "dyn_ray_enable", out->dyn_ray_enable);
  LoadFieldFromYaml(y, "dyn_ray_margin_near_m", out->dyn_ray_margin_near_m);
  LoadFieldFromYaml(y, "dyn_ray_max_range_m", out->dyn_ray_max_range_m);
  LoadFieldFromYaml(y, "dyn_ray_hit_eps_m", out->dyn_ray_hit_eps_m);
  LoadFieldFromYaml(y, "dyn_ray_max_steps", out->dyn_ray_max_steps);
  LoadFieldFromYaml(y, "dyn_ray_confirm_df_m", out->dyn_ray_confirm_df_m);
  LoadFieldFromYaml(y, "dyn_ray_step_m", out->dyn_ray_step_m);

  LoadFieldFromYaml(y, "dyn_front_protect_enable", out->dyn_front_protect_enable);
  LoadFieldFromYaml(y, "dyn_front_protect_x_min_m", out->dyn_front_protect_x_min_m);
  LoadFieldFromYaml(y, "dyn_front_protect_x_max_m", out->dyn_front_protect_x_max_m);
  LoadFieldFromYaml(y, "dyn_front_protect_abs_y_max_m", out->dyn_front_protect_abs_y_max_m);
  LoadFieldFromYaml(y, "dyn_front_protect_use_y_axis", out->dyn_front_protect_use_y_axis);
  LoadFieldFromYaml(y, "dyn_front_protect_forward_sign", out->dyn_front_protect_forward_sign);
  LoadFieldFromYaml(y, "dyn_front_protect_max_drop_ratio", out->dyn_front_protect_max_drop_ratio);
  LoadFieldFromYaml(y, "dyn_front_protect_min_in_points", out->dyn_front_protect_min_in_points);

  LoadFieldFromYaml(y, "quality_enable", out->quality_enable);
  LoadFieldFromYaml(y, "quality_max_delta_trans", out->quality_max_delta_trans);
  LoadFieldFromYaml(y, "quality_max_delta_yaw_deg",
                    out->quality_max_delta_yaw_deg);
  LoadFieldFromYaml(y, "quality_trans_prob_bad", out->quality_trans_prob_bad);
  LoadFieldFromYaml(y, "quality_trans_prob_good", out->quality_trans_prob_good);

  LoadFieldFromYaml(y, "degen_fusion_enable", out->degen_fusion_enable);
  LoadFieldFromYaml(y, "degen_force_active_always", out->degen_force_active_always);
  LoadFieldFromYaml(y, "degen_min_points", out->degen_min_points);
  LoadFieldFromYaml(y, "degen_cov_eps", out->degen_cov_eps);
  LoadFieldFromYaml(y, "degen_cond_ok", out->degen_cond_ok);
  LoadFieldFromYaml(y, "degen_cond_bad", out->degen_cond_bad);
  LoadFieldFromYaml(y, "degen_cond_hyst_on", out->degen_cond_hyst_on);
  LoadFieldFromYaml(y, "degen_cond_hyst_off", out->degen_cond_hyst_off);
  LoadFieldFromYaml(y, "degen_lambda_min_thresh", out->degen_lambda_min_thresh);
  LoadFieldFromYaml(y, "degen_alpha_w_min", out->degen_alpha_w_min);
  LoadFieldFromYaml(y, "degen_alpha_s", out->degen_alpha_s);
  LoadFieldFromYaml(y, "degen_alpha_yaw", out->degen_alpha_yaw);

  LoadFieldFromYaml(y, "degen_ambiguity_enable", out->degen_ambiguity_enable);
  LoadFieldFromYaml(y, "degen_ambiguity_seed_offset_m", out->degen_ambiguity_seed_offset_m);
  LoadFieldFromYaml(y, "degen_ambiguity_score_margin", out->degen_ambiguity_score_margin);
  LoadFieldFromYaml(y, "degen_ambiguity_tp_margin", out->degen_ambiguity_tp_margin);
  LoadFieldFromYaml(y, "degen_ambiguity_weak_sep_m", out->degen_ambiguity_weak_sep_m);
  LoadFieldFromYaml(y, "degen_ambiguity_strong_sep_m", out->degen_ambiguity_strong_sep_m);
  LoadFieldFromYaml(y, "degen_ambiguity_ndt_max_iters", out->degen_ambiguity_ndt_max_iters);

  LoadFieldFromYaml(y, "degen_dr_hold_enable", out->degen_dr_hold_enable);
  LoadFieldFromYaml(y, "degen_dr_hold_trigger_frames", out->degen_dr_hold_trigger_frames);
  LoadFieldFromYaml(y, "degen_dr_hold_frames", out->degen_dr_hold_frames);
  LoadFieldFromYaml(y, "degen_dr_hold_recover_frames", out->degen_dr_hold_recover_frames);

  LoadFieldFromYaml(y, "degen_dr_cruise_enable", out->degen_dr_cruise_enable);
  LoadFieldFromYaml(y, "degen_dr_cruise_trigger_frames", out->degen_dr_cruise_trigger_frames);
  LoadFieldFromYaml(y, "degen_dr_cruise_hold_dist_m", out->degen_dr_cruise_hold_dist_m);
  LoadFieldFromYaml(y, "degen_dr_cruise_max_frames", out->degen_dr_cruise_max_frames);
  LoadFieldFromYaml(y, "degen_dr_cruise_min_speed_mps", out->degen_dr_cruise_min_speed_mps);
  LoadFieldFromYaml(y, "degen_dr_cruise_max_yaw_rate_deg", out->degen_dr_cruise_max_yaw_rate_deg);
  LoadFieldFromYaml(y, "degen_dr_cruise_require_ambiguity", out->degen_dr_cruise_require_ambiguity);
  LoadFieldFromYaml(y, "degen_dr_cruise_require_hard", out->degen_dr_cruise_require_hard);

  LoadFieldFromYaml(y, "degen_amb_accept_dw_m", out->degen_amb_accept_dw_m);
  LoadFieldFromYaml(y, "degen_amb_accept_ds_m", out->degen_amb_accept_ds_m);
  LoadFieldFromYaml(y, "degen_amb_accept_dyaw_rad", out->degen_amb_accept_dyaw_rad);
  LoadFieldFromYaml(y, "degen_temp_reject_enter_dw_m", out->degen_temp_reject_enter_dw_m);

  LoadFieldFromYaml(y, "degen_alpha_w_ambiguity", out->degen_alpha_w_ambiguity);
  LoadFieldFromYaml(y, "degen_alpha_s_ambiguity", out->degen_alpha_s_ambiguity);
  LoadFieldFromYaml(y, "degen_alpha_yaw_ambiguity", out->degen_alpha_yaw_ambiguity);
  LoadFieldFromYaml(y, "degen_block_anchor_on_ambiguity", out->degen_block_anchor_on_ambiguity);
  LoadFieldFromYaml(y, "degen_block_last_pose_on_ambiguity", out->degen_block_last_pose_on_ambiguity);


  LoadFieldFromYaml(y, "temporal_check_enable", out->temporal_check_enable);
  LoadFieldFromYaml(y, "temporal_max_dt", out->temporal_max_dt);
  LoadFieldFromYaml(y, "temporal_max_lin_vel", out->temporal_max_lin_vel);
  LoadFieldFromYaml(y, "temporal_max_ang_vel_deg",
                    out->temporal_max_ang_vel_deg);
  LoadFieldFromYaml(y, "temporal_reject_backtrack_enable", out->temporal_reject_backtrack_enable);
  LoadFieldFromYaml(y, "temporal_reject_backtrack_min_speed_mps", out->temporal_reject_backtrack_min_speed_mps);
  LoadFieldFromYaml(y, "temporal_reject_backtrack_dir_min_step_m", out->temporal_reject_backtrack_dir_min_step_m);
  LoadFieldFromYaml(y, "temporal_reject_backtrack_max_m", out->temporal_reject_backtrack_max_m);
  LoadFieldFromYaml(y, "temporal_odom_max_age", out->temporal_odom_max_age);
  LoadFieldFromYaml(y, "temporal_imu_max_age", out->temporal_imu_max_age);

  LoadFieldFromYaml(y, "temporal_move_min", out->temporal_move_min);
  LoadFieldFromYaml(y, "temporal_stuck_max", out->temporal_stuck_max);
  LoadFieldFromYaml(y, "temporal_stop_max", out->temporal_stop_max);
  LoadFieldFromYaml(y, "temporal_jump_when_stop", out->temporal_jump_when_stop);

  LoadFieldFromYaml(y, "temporal_ang_move_min_deg",
                    out->temporal_ang_move_min_deg);
  LoadFieldFromYaml(y, "temporal_ang_stuck_max_deg",
                    out->temporal_ang_stuck_max_deg);
  LoadFieldFromYaml(y, "temporal_ang_jump_when_stop_deg",
                    out->temporal_ang_jump_when_stop_deg);

  LoadFieldFromYaml(y, "scan_min_range", out->scan_min_range);
  LoadFieldFromYaml(y, "scan_max_range", out->scan_max_range);
  LoadFieldFromYaml(y, "intensity_prefilter_enable", out->intensity_prefilter_enable);
  LoadFieldFromYaml(y, "intensity_prefilter_range_comp_p", out->intensity_prefilter_range_comp_p);
  LoadFieldFromYaml(y, "intensity_prefilter_norm_min", out->intensity_prefilter_norm_min);
  LoadFieldFromYaml(y, "intensity_prefilter_norm_max", out->intensity_prefilter_norm_max);
  LoadFieldFromYaml(y, "intensity_prefilter_keep_ratio", out->intensity_prefilter_keep_ratio);
  LoadFieldFromYaml(y, "intensity_prefilter_min_keep_points", out->intensity_prefilter_min_keep_points);
  LoadFieldFromYaml(y, "intensity_prefilter_adaptive_enable", out->intensity_prefilter_adaptive_enable);
  LoadFieldFromYaml(y, "intensity_prefilter_adaptive_keep_ratio", out->intensity_prefilter_adaptive_keep_ratio);
  LoadFieldFromYaml(y, "intensity_prefilter_adaptive_frames", out->intensity_prefilter_adaptive_frames);

  LoadFieldFromYaml(y, "stationary_speed_thresh", out->stationary_speed_thresh);
  LoadFieldFromYaml(y, "stationary_min_duration", out->stationary_min_duration);

  LoadFieldFromYaml(y, "dr_follow_dist_thresh", out->dr_follow_dist_thresh);
  LoadFieldFromYaml(y, "dr_fail_dist_thresh", out->dr_fail_dist_thresh);
  LoadFieldFromYaml(y, "dr_frame_fail_thresh", out->dr_frame_fail_thresh);

  LoadFieldFromYaml(y, "temporal_grace_enable", out->temporal_grace_enable);
  LoadFieldFromYaml(y, "temporal_grace_duration", out->temporal_grace_duration);

  LoadFieldFromYaml(y, "init_trans_prob_min", out->init_trans_prob_min);
  LoadFieldFromYaml(y, "init_max_candidates_per_scan",
                    out->init_max_candidates_per_scan);
  LoadFieldFromYaml(y, "init_try_once_per_candidate",
                    out->init_try_once_per_candidate);
  LoadFieldFromYaml(y, "init_dedupe_xy_eps", out->init_dedupe_xy_eps);
  LoadFieldFromYaml(y, "init_dedupe_yaw_eps", out->init_dedupe_yaw_eps);

  LoadFieldFromYaml(y, "init_score_soft_enable", out->init_score_soft_enable);
  LoadFieldFromYaml(y, "init_score_max", out->init_score_max);
  LoadFieldFromYaml(y, "init_tp_soft", out->init_tp_soft);
  LoadFieldFromYaml(y, "init_score_relaxed_max", out->init_score_relaxed_max);

  LoadFieldFromYaml(y, "init_rearm_enable", out->init_rearm_enable);
  LoadFieldFromYaml(y, "init_rearm_dist", out->init_rearm_dist);
  LoadFieldFromYaml(y, "init_rearm_min_stop_duration",
                    out->init_rearm_min_stop_duration);
  LoadFieldFromYaml(y, "init_rearm_cooldown", out->init_rearm_cooldown);

  LoadFieldFromYaml(y, "stamp_max_skew_sec", out->stamp_max_skew_sec);
  LoadFieldFromYaml(y, "stamp_future_tol_sec", out->stamp_future_tol_sec);
  LoadFieldFromYaml(y, "stamp_monotonic_eps_sec", out->stamp_monotonic_eps_sec);

  LoadFieldFromYaml(y, "viz_publish_scan_cloud", out->viz_publish_scan_cloud);
  LoadFieldFromYaml(y, "viz_scan_cloud_topic", out->viz_scan_cloud_topic);

  LoadFieldFromYaml(y, "local_reloc_enable", out->local_reloc_enable);
  LoadFieldFromYaml(y, "local_reloc_map_yaml", out->local_reloc_map_yaml);

  // ---- LocalReloc options (deg keys) ----
  double local_yaw_range_deg = out->local_opt_base.yaw_range_rad * 180.0 / kPi;
  double local_yaw_step_deg = out->local_opt_base.yaw_step_rad * 180.0 / kPi;

  LoadFieldFromYaml(y, "local_reloc_xy_range_m",
                    out->local_opt_base.xy_range_m);
  LoadFieldFromYaml(y, "local_reloc_yaw_range_deg", local_yaw_range_deg);

  LoadFieldFromYaml(y, "local_reloc_min_score", out->local_opt_base.min_score);
  LoadFieldFromYaml(y, "local_reloc_score_margin",
                    out->local_opt_base.score_margin);
  LoadFieldFromYaml(y, "local_reloc_min_valid_fraction",
                    out->local_opt_base.min_valid_fraction);

  LoadFieldFromYaml(y, "local_reloc_bnb_max_level",
                    out->local_opt_base.bnb_max_level);
  LoadFieldFromYaml(y, "local_reloc_yaw_step_deg", local_yaw_step_deg);
  LoadFieldFromYaml(y, "local_reloc_max_scan_points",
                    out->local_opt_base.max_scan_points);
  LoadFieldFromYaml(y, "local_reloc_scan_voxel_m",
                    out->local_opt_base.scan_voxel_m);
  LoadFieldFromYaml(y, "local_reloc_pyr_max_level",
                    out->local_opt_base.pyr_max_level);
  LoadFieldFromYaml(y, "local_reloc_hit_sigma_m",
                    out->local_opt_base.hit_sigma_m);
  LoadFieldFromYaml(y, "local_reloc_max_dist_m",
                    out->local_opt_base.max_dist_m);

  out->local_opt_base.yaw_range_rad = local_yaw_range_deg * kPi / 180.0;
  out->local_opt_base.yaw_step_rad = local_yaw_step_deg * kPi / 180.0;

  // ---- RelocMgr (deg keys) ----
  double mgr_fail_yaw_deg =
      out->reloc_mgr_params.fail_yaw_range_rad * 180.0 / kPi;
  double mgr_manual_yaw_min_deg =
      out->reloc_mgr_params.manual_yaw_min_rad * 180.0 / kPi;

  LoadFieldFromYaml(y, "reloc_mgr_enable", out->reloc_mgr_params.enable);
  LoadFieldFromYaml(y, "reloc_mgr_cooldown_sec",
                    out->reloc_mgr_params.cooldown_sec);
  LoadFieldFromYaml(y, "reloc_mgr_fail_xy_range_m",
                    out->reloc_mgr_params.fail_xy_range_m);
  LoadFieldFromYaml(y, "reloc_mgr_fail_yaw_range_deg", mgr_fail_yaw_deg);
  LoadFieldFromYaml(y, "reloc_mgr_manual_xy_min_m",
                    out->reloc_mgr_params.manual_xy_min_m);
  LoadFieldFromYaml(y, "reloc_mgr_manual_xy_sigma_k",
                    out->reloc_mgr_params.manual_xy_sigma_k);
  LoadFieldFromYaml(y, "reloc_mgr_manual_yaw_min_deg", mgr_manual_yaw_min_deg);
  LoadFieldFromYaml(y, "reloc_mgr_manual_yaw_sigma_k",
                    out->reloc_mgr_params.manual_yaw_sigma_k);
  LoadFieldFromYaml(y, "reloc_mgr_max_attempts",
                    out->reloc_mgr_params.max_attempts);

  out->reloc_mgr_params.fail_yaw_range_rad = mgr_fail_yaw_deg * kPi / 180.0;
  out->reloc_mgr_params.manual_yaw_min_rad =
      mgr_manual_yaw_min_deg * kPi / 180.0;

  // ---- Watchdog ----
  LoadFieldFromYaml(y, "watchdog_enable", out->watchdog_enable);
  LoadFieldFromYaml(y, "watchdog_require_scan", out->watchdog_require_scan);
  LoadFieldFromYaml(y, "watchdog_require_odom", out->watchdog_require_odom);
  LoadFieldFromYaml(y, "watchdog_require_imu", out->watchdog_require_imu);
  LoadFieldFromYaml(y, "watchdog_timer_hz", out->watchdog_timer_hz);

  LoadFieldFromYaml(y, "watchdog_scan_warn_sec", out->watchdog_scan_warn_sec);
  LoadFieldFromYaml(y, "watchdog_scan_fail_sec", out->watchdog_scan_fail_sec);
  LoadFieldFromYaml(y, "watchdog_odom_warn_sec", out->watchdog_odom_warn_sec);
  LoadFieldFromYaml(y, "watchdog_odom_fail_sec", out->watchdog_odom_fail_sec);
  LoadFieldFromYaml(y, "watchdog_imu_warn_sec", out->watchdog_imu_warn_sec);
  LoadFieldFromYaml(y, "watchdog_imu_fail_sec", out->watchdog_imu_fail_sec);
  LoadFieldFromYaml(y, "watchdog_fail_hold_sec", out->watchdog_fail_hold_sec);

  // --- launch overrides（可临时在 launch 覆盖）---
  pnh.param("viz_publish_scan_cloud", out->viz_publish_scan_cloud,
            out->viz_publish_scan_cloud);
  pnh.param<std::string>("viz_scan_cloud_topic", out->viz_scan_cloud_topic,
                         out->viz_scan_cloud_topic);
  pnh.param("stamp_max_skew_sec", out->stamp_max_skew_sec,
            out->stamp_max_skew_sec);
  pnh.param("stamp_future_tol_sec", out->stamp_future_tol_sec,
            out->stamp_future_tol_sec);
  pnh.param("stamp_monotonic_eps_sec", out->stamp_monotonic_eps_sec,
            out->stamp_monotonic_eps_sec);

  // --- derived fields ---
  out->local_opt_base.min_range = out->scan_min_range;
  out->local_opt_base.max_range = out->scan_max_range;

  if (out->local_reloc_enable && out->local_reloc_map_yaml.empty()) {
    out->local_reloc_map_yaml = pkg_path + "/map/" + map_stem + ".yaml";
  }

  // --- clamp/sanitize（约束逻辑）---
  out->init_max_candidates_per_scan =
      ClampMinInt(out->init_max_candidates_per_scan, 1, 1);

  if (out->init_dedupe_xy_eps <= 0.0)
    out->init_dedupe_xy_eps = 1e-3;
  if (out->init_dedupe_yaw_eps <= 0.0)
    out->init_dedupe_yaw_eps = 1e-3;

  if (out->temporal_grace_duration <= 0.0)
    out->temporal_grace_duration = kDefaultTemporalGrace;
  if (out->stamp_monotonic_eps_sec <= 0.0)
    out->stamp_monotonic_eps_sec = 1e-6;

  if (!std::isfinite(out->temporal_reject_backtrack_min_speed_mps) ||
      out->temporal_reject_backtrack_min_speed_mps < 0.0) {
    out->temporal_reject_backtrack_min_speed_mps = 0.15;
  }
  if (!std::isfinite(out->temporal_reject_backtrack_dir_min_step_m) ||
      out->temporal_reject_backtrack_dir_min_step_m <= 1e-4) {
    out->temporal_reject_backtrack_dir_min_step_m = 0.005;
  }
  if (!std::isfinite(out->temporal_reject_backtrack_max_m) ||
      out->temporal_reject_backtrack_max_m <= 1e-4) {
    out->temporal_reject_backtrack_max_m = 0.08;
  }

  if (out->init_score_max <= 0.0)
    out->init_score_max = 0.01;
  if (out->init_tp_soft <= 0.0)
    out->init_tp_soft = out->init_trans_prob_min;

  if (out->local_opt_base.scan_voxel_m <= 0.0)
    out->local_opt_base.scan_voxel_m = 0.02;

  ClampPos(out->init_score_relaxed_max, 1e-9, 0.03);

  if (out->local_opt_base.pyr_max_level < 0)
    out->local_opt_base.pyr_max_level = 0;
  if (!std::isfinite(out->local_opt_base.hit_sigma_m) ||
      out->local_opt_base.hit_sigma_m <= 1e-3) {
    out->local_opt_base.hit_sigma_m = 0.20;
  }
  if (!std::isfinite(out->local_opt_base.max_dist_m) ||
      out->local_opt_base.max_dist_m <= 1e-3) {
    out->local_opt_base.max_dist_m = 1.00;
  }
  
  if (!std::isfinite(out->dyn_grid_res_m) || out->dyn_grid_res_m <= 1e-3) out->dyn_grid_res_m = 0.10;
  if (!std::isfinite(out->dyn_inflate_m)  || out->dyn_inflate_m  < 0.0)  out->dyn_inflate_m  = 0.20;
  if (!std::isfinite(out->dyn_padding_m)  || out->dyn_padding_m  < 0.0)  out->dyn_padding_m  = 5.0;

  if (!std::isfinite(out->dyn_keep_ratio) || out->dyn_keep_ratio <= 0.0 || out->dyn_keep_ratio > 1.0) out->dyn_keep_ratio = 0.80;
  if (out->dyn_min_keep_points < 0) out->dyn_min_keep_points = 0;
  if (!std::isfinite(out->dyn_hard_max_dist_m) || out->dyn_hard_max_dist_m <= 1e-3) out->dyn_hard_max_dist_m = 2.0;

    // dyn ray sanitize
  // max_range: <=0 则跟 scan_max_range
  if (!std::isfinite(out->dyn_ray_max_range_m) || out->dyn_ray_max_range_m <= 0.0) {
    out->dyn_ray_max_range_m = out->scan_max_range;
  }
  if (!std::isfinite(out->dyn_ray_margin_near_m) || out->dyn_ray_margin_near_m < 0.0) {
    out->dyn_ray_margin_near_m = 0.20;
  }
  if (!std::isfinite(out->dyn_ray_hit_eps_m)) out->dyn_ray_hit_eps_m = -1.0;
  if (out->dyn_ray_max_steps < 16) out->dyn_ray_max_steps = 16;
  if (!std::isfinite(out->dyn_ray_confirm_df_m) || out->dyn_ray_confirm_df_m < 0.0) {
    out->dyn_ray_confirm_df_m = 0.15;
  }
  if (!std::isfinite(out->dyn_ray_step_m)) out->dyn_ray_step_m = -1.0;

  if (!std::isfinite(out->dyn_front_protect_x_min_m) || out->dyn_front_protect_x_min_m < 0.0) {
    out->dyn_front_protect_x_min_m = 0.10;
  }
  if (!std::isfinite(out->dyn_front_protect_x_max_m) ||
      out->dyn_front_protect_x_max_m <= out->dyn_front_protect_x_min_m + 1e-3) {
    out->dyn_front_protect_x_max_m = std::max(out->dyn_front_protect_x_min_m + 0.5, 3.0);
  }
  if (!std::isfinite(out->dyn_front_protect_abs_y_max_m) || out->dyn_front_protect_abs_y_max_m <= 1e-3) {
    out->dyn_front_protect_abs_y_max_m = 0.80;
  }
  if (!std::isfinite(out->dyn_front_protect_forward_sign) ||
      std::fabs(out->dyn_front_protect_forward_sign) < 1e-6) {
    out->dyn_front_protect_forward_sign = 1.0;
  }
  if (!std::isfinite(out->dyn_front_protect_max_drop_ratio) ||
      out->dyn_front_protect_max_drop_ratio < 0.0 || out->dyn_front_protect_max_drop_ratio > 1.0) {
    out->dyn_front_protect_max_drop_ratio = 0.45;
  }
  if (out->dyn_front_protect_min_in_points < 1) out->dyn_front_protect_min_in_points = 1;

  // watchdog hz 下限（与 initWatchdog_ 一致）
  if (!std::isfinite(out->watchdog_timer_hz) || out->watchdog_timer_hz <= 0.0) {
    out->watchdog_timer_hz = 10.0;
  }


  // degeneration-aware fusion sanitize
  if (out->degen_min_points < 20) out->degen_min_points = 20;
  if (!std::isfinite(out->degen_cov_eps) || out->degen_cov_eps <= 1e-12) out->degen_cov_eps = 1e-6;

  if (!std::isfinite(out->degen_cond_ok) || out->degen_cond_ok < 1.0) out->degen_cond_ok = 80.0;
  if (!std::isfinite(out->degen_cond_bad) || out->degen_cond_bad < out->degen_cond_ok) out->degen_cond_bad = std::max(out->degen_cond_ok + 1.0, 350.0);
  if (!std::isfinite(out->degen_cond_hyst_on) || out->degen_cond_hyst_on < 1.0) out->degen_cond_hyst_on = 65.0;
  if (!std::isfinite(out->degen_cond_hyst_off) || out->degen_cond_hyst_off < 1.0) out->degen_cond_hyst_off = 0.75 * out->degen_cond_hyst_on;
  if (out->degen_cond_hyst_off > out->degen_cond_hyst_on) {
    out->degen_cond_hyst_off = 0.75 * out->degen_cond_hyst_on;
  }

  if (!std::isfinite(out->degen_lambda_min_thresh) || out->degen_lambda_min_thresh <= 0.0) out->degen_lambda_min_thresh = 5e-4;

  if (!std::isfinite(out->degen_alpha_w_min)) out->degen_alpha_w_min = 0.10;
  out->degen_alpha_w_min = std::max(0.0, std::min(1.0, out->degen_alpha_w_min));

  if (!std::isfinite(out->degen_alpha_s)) out->degen_alpha_s = 0.95;
  out->degen_alpha_s = std::max(0.0, std::min(1.0, out->degen_alpha_s));

  if (!std::isfinite(out->degen_alpha_yaw)) out->degen_alpha_yaw = 0.90;
  out->degen_alpha_yaw = std::max(0.0, std::min(1.0, out->degen_alpha_yaw));

  if (!std::isfinite(out->degen_ambiguity_seed_offset_m) || out->degen_ambiguity_seed_offset_m <= 0.0) out->degen_ambiguity_seed_offset_m = 0.80;
  if (!std::isfinite(out->degen_ambiguity_score_margin) || out->degen_ambiguity_score_margin < 0.0) out->degen_ambiguity_score_margin = 0.003;
  if (!std::isfinite(out->degen_ambiguity_tp_margin) || out->degen_ambiguity_tp_margin < 0.0) out->degen_ambiguity_tp_margin = 0.08;
  if (!std::isfinite(out->degen_ambiguity_weak_sep_m) || out->degen_ambiguity_weak_sep_m <= 0.0) out->degen_ambiguity_weak_sep_m = 0.50;
  if (!std::isfinite(out->degen_ambiguity_strong_sep_m) || out->degen_ambiguity_strong_sep_m <= 0.0) out->degen_ambiguity_strong_sep_m = 0.50;

  if (out->degen_ambiguity_ndt_max_iters < 1) out->degen_ambiguity_ndt_max_iters = 6;

  if (out->degen_dr_hold_trigger_frames < 1) out->degen_dr_hold_trigger_frames = 3;
  if (out->degen_dr_hold_frames < 1) out->degen_dr_hold_frames = 5;
  if (out->degen_dr_hold_recover_frames < 1) out->degen_dr_hold_recover_frames = 2;

  if (out->degen_dr_cruise_trigger_frames < 1) out->degen_dr_cruise_trigger_frames = 8;
  if (!std::isfinite(out->degen_dr_cruise_hold_dist_m) || out->degen_dr_cruise_hold_dist_m <= 0.0) out->degen_dr_cruise_hold_dist_m = 1.5;
  if (out->degen_dr_cruise_max_frames < 1) out->degen_dr_cruise_max_frames = 80;
  if (!std::isfinite(out->degen_dr_cruise_min_speed_mps) || out->degen_dr_cruise_min_speed_mps < 0.0) out->degen_dr_cruise_min_speed_mps = 0.15;
  if (!std::isfinite(out->degen_dr_cruise_max_yaw_rate_deg) || out->degen_dr_cruise_max_yaw_rate_deg <= 0.0) out->degen_dr_cruise_max_yaw_rate_deg = 20.0;

  if (!std::isfinite(out->degen_amb_accept_dw_m) || out->degen_amb_accept_dw_m <= 0.0) out->degen_amb_accept_dw_m = 1.00;
  if (!std::isfinite(out->degen_amb_accept_ds_m) || out->degen_amb_accept_ds_m <= 0.0) out->degen_amb_accept_ds_m = 0.30;
  if (!std::isfinite(out->degen_amb_accept_dyaw_rad) || out->degen_amb_accept_dyaw_rad <= 0.0) out->degen_amb_accept_dyaw_rad = 10.0 * kPi / 180.0;
  if (!std::isfinite(out->degen_temp_reject_enter_dw_m) || out->degen_temp_reject_enter_dw_m <= 0.0) {
    out->degen_temp_reject_enter_dw_m = 0.15;
  }

  if (!std::isfinite(out->degen_alpha_w_ambiguity)) out->degen_alpha_w_ambiguity = 0.0;
  out->degen_alpha_w_ambiguity = std::max(0.0, std::min(1.0, out->degen_alpha_w_ambiguity));

  if (!std::isfinite(out->degen_alpha_s_ambiguity)) out->degen_alpha_s_ambiguity = 1.0;
  out->degen_alpha_s_ambiguity = std::max(0.0, std::min(1.0, out->degen_alpha_s_ambiguity));

  if (!std::isfinite(out->degen_alpha_yaw_ambiguity)) out->degen_alpha_yaw_ambiguity = 0.0;
  out->degen_alpha_yaw_ambiguity = std::max(0.0, std::min(1.0, out->degen_alpha_yaw_ambiguity));

  // intensity prefilter sanitize
  if (!std::isfinite(out->intensity_prefilter_range_comp_p) || out->intensity_prefilter_range_comp_p < 0.0) {
    out->intensity_prefilter_range_comp_p = 2.0;
  }
  if (!std::isfinite(out->intensity_prefilter_norm_min) || out->intensity_prefilter_norm_min < 0.0) {
    out->intensity_prefilter_norm_min = 0.0;
  }
  if (!std::isfinite(out->intensity_prefilter_norm_max) ||
      out->intensity_prefilter_norm_max <= out->intensity_prefilter_norm_min) {
    out->intensity_prefilter_norm_max = std::max(out->intensity_prefilter_norm_min + 1.0, 1e6);
  }
  if (!std::isfinite(out->intensity_prefilter_keep_ratio)) {
    out->intensity_prefilter_keep_ratio = 0.30;
  }
  out->intensity_prefilter_keep_ratio =
      std::max(0.0, std::min(1.0, out->intensity_prefilter_keep_ratio));
  if (out->intensity_prefilter_min_keep_points < 20) {
    out->intensity_prefilter_min_keep_points = 20;
  }
  if (!std::isfinite(out->intensity_prefilter_adaptive_keep_ratio)) {
    out->intensity_prefilter_adaptive_keep_ratio = 0.80;
  }
  out->intensity_prefilter_adaptive_keep_ratio =
      std::max(0.0, std::min(1.0, out->intensity_prefilter_adaptive_keep_ratio));
  if (out->intensity_prefilter_adaptive_frames < 1) {
    out->intensity_prefilter_adaptive_frames = 1;
  }

  return true;
}

} // namespace config
} // namespace localization_ndt
