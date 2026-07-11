#include "localization_ndt/config/params_yaml.hpp"
#include "localization_ndt/config/yaml_utils.hpp"

#include <ros/ros.h>
#include <sstream>

namespace localization_ndt {
namespace config {

// ------- params/<map>.yaml -------
std::string MakeDefaultParamsYamlText(const std::string& map_stem) {
  std::ostringstream oss;
  oss << "# ============================================================\n"
      << "# NDT 定位参数（自动生成模板）\n"
      << "# - 对应地图: " << map_stem << "\n"
      << "# - 修改数值后重启节点生效\n"
      << "# ============================================================\n"
      << "\n"
      << "# ---------- 0. 初始化位姿 (last + fixed) ----------\n"
      << "init_trans_prob_min: 2.0\n"
      << "init_max_candidates_per_scan: 3\n"
      << "init_try_once_per_candidate: true\n"
      << "init_dedupe_xy_eps: 0.001\n"
      << "init_dedupe_yaw_eps: 0.001\n"
      << "init_score_soft_enable: true\n"
      << "init_score_max: 0.5\n"
      << "init_tp_soft: 1.8\n"
      << "\n"
      << "# ---------- 0.1 init re-arm（耗尽冻结后：移动 + 停稳 + 冷却再重定位）----------\n"
      << "init_rearm_enable: true\n"
      << "init_rearm_dist: 2.0\n"
      << "init_rearm_min_stop_duration: 2.0\n"
      << "init_rearm_cooldown: 8.0\n"
      << "\n"
      << "# hard-pass(tp>=init_trans_prob_min) 时允许更“松”的 score 上限（防 tp 很强但 score 略大被卡死）\n"
      << "# 建议 >= init_score_max\n"
      << "init_score_relaxed_max: 0.01\n"
      << "\n"
      << "# ---------- 1. NDT 基本参数 ----------\n"
      << "ndt_resolution: 1.0\n"
      << "ndt_num_threads: 4\n"
      << "ndt_target_use_known_mask_filter: false\n"
      << "\n"
      << "# ---------- 2. NDT 质量门控 (tp + innovation) ----------\n"
      << "quality_enable: true\n"
      << "quality_max_delta_trans: 1.5\n"
      << "quality_max_delta_yaw_deg: 20.0\n"
      << "quality_trans_prob_bad: 1.8\n"
      << "quality_trans_prob_good: 1.9\n"
      << "\n"
      << "# ---------- 2.5 退化检测与各向异性融合（长走廊防滑移） ----------\n"
      << "degen_fusion_enable: true\n"
      << "degen_force_active_always: false\n"
      << "degen_min_points: 80\n"
      << "degen_cov_eps: 0.000001\n"
      << "degen_cond_ok: 80.0\n"
      << "degen_cond_bad: 350.0\n"
      << "degen_cond_hyst_on: 65.0\n"
      << "degen_cond_hyst_off: 60.0\n"
      << "degen_lambda_min_thresh: 0.0005\n"
      << "degen_alpha_w_min: 0.10\n"
      << "degen_alpha_s: 0.95\n"
      << "degen_alpha_yaw: 0.90\n"
      << "degen_temp_reject_enter_dw_m: 0.15\n"
      << "degen_ambiguity_enable: true\n"
      << "degen_ambiguity_seed_offset_m: 0.80\n"
      << "degen_ambiguity_score_margin: 0.003\n"
      << "degen_ambiguity_tp_margin: 0.08\n"
      << "degen_ambiguity_weak_sep_m: 0.50\n"
      << "degen_ambiguity_strong_sep_m: 0.50\n"
      << "degen_ambiguity_ndt_max_iters: 12\n"
      << "degen_alpha_w_ambiguity: 0.0\n"
      << "degen_alpha_s_ambiguity: 0.20\n"
      << "degen_alpha_yaw_ambiguity: 0.20\n"
      << "degen_block_anchor_on_ambiguity: true\n"
      << "degen_block_last_pose_on_ambiguity: true\n"
      << "\n"
      << "# ---------- 3. 时序一致性检查（Temporal A/B） ----------\n"
      << "temporal_check_enable: true\n"
      << "temporal_max_dt: 1.0\n"
      << "temporal_max_lin_vel: 2.5\n"
      << "temporal_max_ang_vel_deg: 180.0\n"
      << "temporal_reject_backtrack_enable: true\n"
      << "temporal_reject_backtrack_min_speed_mps: 0.15\n"
      << "temporal_reject_backtrack_dir_min_step_m: 0.005\n"
      << "temporal_reject_backtrack_max_m: 0.08\n"
      << "temporal_odom_max_age: 0.2\n"
      << "temporal_imu_max_age: 0.2\n"
      << "\n"
      << "temporal_move_min: 0.10\n"
      << "temporal_stuck_max: 0.03\n"
      << "temporal_stop_max: 0.03\n"
      << "temporal_jump_when_stop: 0.30\n"
      << "\n"
      << "temporal_ang_move_min_deg: 10.0\n"
      << "temporal_ang_stuck_max_deg: 3.0\n"
      << "temporal_ang_jump_when_stop_deg: 30.0\n"
      << "\n"
      << "# ---------- 4. 激光预处理（距离裁剪） ----------\n"
      << "scan_min_range: 1.0\n"
      << "scan_max_range: 35.0\n"
      << "\n"
      << "# ---------- 4.1 intensity prefilter ----------\n"
      << "intensity_prefilter_enable: true\n"
      << "intensity_prefilter_range_comp_p: 2.0\n"
      << "intensity_prefilter_norm_min: 0.0\n"
      << "intensity_prefilter_norm_max: 1000000.0\n"
      << "intensity_prefilter_keep_ratio: 0.30\n"
      << "intensity_prefilter_min_keep_points: 200\n"
      << "intensity_prefilter_adaptive_enable: true\n"
      << "intensity_prefilter_adaptive_keep_ratio: 0.80\n"
      << "intensity_prefilter_adaptive_frames: 20\n"
      << "\n"
      << "# ---------- 5. 静止 & last_pose 写入策略 ----------\n"
      << "stationary_speed_thresh: 0.02\n"
      << "stationary_min_duration: 2.0\n"
      << "\n"
      << "# ---------- 6. DR 累计距离阈值 ----------\n"
      << "dr_follow_dist_thresh: 5.0\n"
      << "dr_fail_dist_thresh: 10.0\n"
      << "dr_frame_fail_thresh: 300\n"
      << "\n"
      << "# ---------- 7. RViz 手动初始化：Temporal Grace ----------\n"
      << "temporal_grace_enable: true\n"
      << "temporal_grace_duration: 0.8\n"
      << "\n"
      << "# ---------- 8. 发布时间戳消毒（bag stamp 异常用） ----------\n"
      << "stamp_max_skew_sec: 0.20\n"
      << "stamp_future_tol_sec: 0.05\n"
      << "stamp_monotonic_eps_sec: 0.000001\n"
      << "\n"
      << "# ---------- 9. RViz 辅助话题 ----------\n"
      << "viz_publish_scan_cloud: false\n"
      << "viz_scan_cloud_topic: \"scan_cloud_map\"\n"
      << "\n"
      << "# ---------- 10. LocalReloc (FCSM/BnB seed for re-localization) ----------\n"
      << "local_reloc_enable: true\n"
      << "local_reloc_map_yaml: \"\"\n"
      << "local_reloc_xy_range_m: 10.0\n"
      << "local_reloc_yaw_range_deg: 180.0\n"
      << "local_reloc_min_score: 0.20\n"
      << "local_reloc_score_margin: 0.02\n"
      << "local_reloc_min_valid_fraction: 0.30\n"
      << "local_reloc_pyr_max_level: 6\n"
      << "local_reloc_bnb_max_level: 5\n"
      << "local_reloc_yaw_step_deg: 1.0\n"
      << "local_reloc_max_scan_points: 1200\n"
      << "local_reloc_scan_voxel_m: 0.02\n"
      << "local_reloc_hit_sigma_m: 0.20\n"
      << "local_reloc_max_dist_m: 1.00\n"
      << "\n"
      << "# ---------- 11. RelocalizationManager (trigger logic) ----------\n"
      << "reloc_mgr_enable: true\n"
      << "reloc_mgr_cooldown_sec: 1.0\n"
      << "reloc_mgr_fail_xy_range_m: 10.0\n"
      << "reloc_mgr_fail_yaw_range_deg: 20.0\n"
      << "reloc_mgr_manual_xy_min_m: 10.0\n"
      << "reloc_mgr_manual_xy_sigma_k: 3.0\n"
      << "reloc_mgr_manual_yaw_min_deg: 180.0\n"
      << "reloc_mgr_manual_yaw_sigma_k: 3.0\n"
      << "reloc_mgr_max_attempts: 5\n"
      << "\n"
      << "# ---------- 12. Input Watchdog (topic stall / drop) ----------\n"
      << "watchdog_enable: true\n"
      << "watchdog_require_scan: true\n"
      << "watchdog_require_odom: true\n"
      << "watchdog_require_imu: true\n"
      << "watchdog_timer_hz: 10.0\n"
      << "watchdog_scan_warn_sec: 0.10\n"
      << "watchdog_scan_fail_sec: 0.50\n"
      << "watchdog_odom_warn_sec: 0.10\n"
      << "watchdog_odom_fail_sec: 0.50\n"
      << "watchdog_imu_warn_sec: 0.10\n"
      << "watchdog_imu_fail_sec: 0.50\n"
      << "watchdog_fail_hold_sec: 0.0\n"
      << "\n"
      << "# ---------- 13. Dynamic point filter (scan->static) ----------\n"
      << "dyn_filter_enable: true\n"
      << "dyn_grid_res_m: 0.10\n"
      << "dyn_inflate_m: 0.20\n"
      << "dyn_padding_m: 5.0\n"
      << "dyn_keep_ratio: 0.80\n"
      << "dyn_min_keep_points: 200\n"
      << "dyn_hard_max_dist_m: 2.0\n"
      << "dyn_ray_enable: true\n"
      << "dyn_ray_margin_near_m: 0.20\n"
      << "dyn_ray_max_range_m: 35.0\n"
      << "dyn_ray_hit_eps_m: -1.0\n"
      << "dyn_ray_max_steps: 256\n"
      << "dyn_ray_confirm_df_m: 0.15\n"
      << "dyn_ray_step_m: -1.0\n"
      << "dyn_front_protect_enable: true\n"
      << "dyn_front_protect_x_min_m: 0.10\n"
      << "dyn_front_protect_x_max_m: 3.00\n"
      << "dyn_front_protect_abs_y_max_m: 0.80\n"
      << "dyn_front_protect_use_y_axis: false\n"
      << "dyn_front_protect_forward_sign: 1.0\n"
      << "dyn_front_protect_max_drop_ratio: 0.45\n"
      << "dyn_front_protect_min_in_points: 12\n"
      << "\n";


  return oss.str();
}

YAML::Node LoadOrCreateParamsYaml(const std::string& params_dir,
                                  const std::string& map_stem) {
  const std::string params_yaml = params_dir + "/" + map_stem + ".yaml";

  if (FileExists(params_yaml)) {
    try {
      ROS_INFO_STREAM("Loading params yaml: " << params_yaml);
      return YAML::LoadFile(params_yaml);
    } catch (const std::exception& e) {
      ROS_WARN_STREAM("Failed to load params yaml '"
                      << params_yaml << "', error: " << e.what()
                      << ". Will KEEP the file and use an empty YAML node instead.");
      return YAML::Node();
    }
  }

  const std::string text = MakeDefaultParamsYamlText(map_stem);
  if (!SaveTextToFile(params_yaml, text)) {
    ROS_WARN_STREAM("Failed to write default params yaml: " << params_yaml);
    return YAML::Node();
  }

  ROS_INFO_STREAM("Created params yaml: " << params_yaml);

  try {
    return YAML::LoadFile(params_yaml);
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Unexpected: failed to load freshly created params yaml '"
                    << params_yaml << "', error: " << e.what());
    return YAML::Node();
  }
}

}  // namespace config
}  // namespace localization_ndt
