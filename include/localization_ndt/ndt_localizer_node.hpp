#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <atomic>
#include <ros/ros.h>

#include <laser_geometry/laser_geometry.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Dense>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <localization_msgs/AlgoEnable.h>
#include <localization_msgs/AlgoSwitchMap.h>
#include <localization_msgs/AlgoSetInitialPose.h>

#include <localization_msgs/LocalizationStatus.h>

#include "localization_ndt/config/ndt_localizer_config.hpp"
#include "localization_ndt/initial_pose_manager.hpp"
#include "localization_ndt/local_relocalizer_2d.hpp"
#include "localization_ndt/ndt_matcher.hpp"
#include "localization_ndt/relocalization_manager.hpp"
#include "localization_ndt/simple_predictor.hpp"
#include "localization_ndt/types.hpp"
#include "localization_ndt/dynamic_point_filter.hpp"
#include "localization_ndt/diagnostics_dtc_publisher.hpp"


namespace localization_ndt {

// 全局定位状态（Lightning-LM 风格）
enum class LocLevel {
  INITIALIZING = 0,
  GOOD = 1,
  FOLLOWING_DR = 2,
  FAIL = 3,
};

struct Odom2D {
  ros::Time t;
  double x, y, yaw;
};

struct GyroZ {
  ros::Time t;
  double wz;  // rad/s
};

// 只做“数值监控”的 NDT 质量指标
struct NdtQualityMetrics {
  double score = std::numeric_limits<double>::quiet_NaN();             // 仅监控
  double trans_probability = std::numeric_limits<double>::quiet_NaN(); // 监控/门控
  double delta_trans = std::numeric_limits<double>::quiet_NaN();       // [m]
  double delta_yaw = std::numeric_limits<double>::quiet_NaN();         // [rad]
  bool ndt_converged = false;
};

class NdtLocalizerNode {
public:
  NdtLocalizerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spin();
  ~NdtLocalizerNode();

private:
  // ===================== init / boot helpers =====================
  void initDirs_(const std::string& pkg_path);

  void resolveRunSelection_(const std::string& pkg_path,
                            std::string* out_map_name,
                            int* out_run_target);

  void resolveMapPath_(const std::string& pkg_path,
                       const std::string& map_name,
                       std::string* out_map_stem);

  // 统一加载 cfg_：defaults + yaml + launch overrides + clamp/sanitize
  void loadConfig_(const std::string& pkg_path, const std::string& map_stem);

  void loadInterfaceConfig_();

  void initLocalReloc_();      // 基于 cfg_ 初始化 local_reloc_
  void initRosInterfaces_();
  void initWatchdog_();
  void initMatcher_();
  void resetRuntimeState_();
  void logEffectiveConfig_(const std::string& map_name,
                           const std::string& map_stem) const;
  bool bootstrapFromRunSelection_(std::string* out_err);

  // ===================== loc_manager integration =====================
  bool registerToLocManager_();
  bool onAlgoEnable(localization_msgs::AlgoEnable::Request& req,
                    localization_msgs::AlgoEnable::Response& res);
  bool onAlgoSwitchMap(localization_msgs::AlgoSwitchMap::Request& req,
                       localization_msgs::AlgoSwitchMap::Response& res);
  bool onAlgoSetInitialPose(localization_msgs::AlgoSetInitialPose::Request& req,
                            localization_msgs::AlgoSetInitialPose::Response& res);
  bool fetchMapJsonFromServer_(const std::string& map, int layer,
                               std::string* out_json_path,
                               std::string* out_err,
                               std::string* out_resolved_map = nullptr,
                               int* out_resolved_layer = nullptr);
  bool applyMapSwitch_(const std::string& map, int layer,
                       const std::string& json_path,
                       std::string* out_err);
  void applyConfigToDynFilter_();
  void applyInitialPoseMsg_(const geometry_msgs::PoseWithCovarianceStamped& msg);

  // ===================== map / metrics =====================
  bool loadMap(const std::string& path);

  // 只负责计算 score / trans_probability / delta_* 等数值
  NdtQualityMetrics evaluateNdtQuality(double score, double trans_probability,
                                       const Eigen::Matrix4f& T_pred,
                                       const Eigen::Matrix4f& T_ndt,
                                       bool ndt_converged) const;

  // ===================== callbacks =====================
  void scanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg);
  void initialPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  // ===================== publish =====================
  void publishOdom(const ros::Time& stamp, const Eigen::Matrix4f& T_map_base);
  void publishStatus(const ros::Time& stamp, const NdtQualityMetrics& q,
                     uint8_t mode, uint8_t reason,
                     bool ndt_converged, bool temporal_ok);

  // ===================== Temporal gate helpers (two-layer) =====================
  bool isInTemporalGrace(const ros::Time& stamp) const;

  // Temporal-A: 专抓“走廊钉死/假高置信”
  bool temporalCandidateMotionGate(
      const ros::Time& stamp, const Eigen::Matrix4f& T_candidate,
      bool ndt_internal_ok, bool do_check,
      double* out_dt = nullptr,
      double* out_v_cand = nullptr, double* out_v_odom = nullptr,
      double* out_w_cand_deg = nullptr, double* out_w_imu_deg = nullptr,
      double* out_v_diff = nullptr, double* out_w_diff_deg = nullptr);

  // Temporal-B: 防瞬移，不“锁死纠偏”
  bool temporalJumpBoundsGate(const ros::Time& stamp,
                              const Eigen::Matrix4f& T_candidate,
                              bool do_check,
                              double* out_dt = nullptr,
                              double* out_v_jump = nullptr,
                              double* out_w_jump_deg = nullptr) const;

  // ===================== init candidates =====================
  void buildInitCandidates_();
  bool tryInitializeFromCandidates_(const ros::Time& stamp,
                                  const PointCloudT::Ptr& cloud,
                                  const Eigen::Vector2f& sensor_origin_base);
  bool maybeRearmInit_(const ros::Time& stamp);

  bool tryFixedLastRelocInFail_(const ros::Time& stamp,
                              const PointCloudT::Ptr& cloud,
                              const Eigen::Vector2f& sensor_origin_base);
  void writeShutdownPose_(); 

  // ===================== watchdog =====================
  bool watchdogCheck_(const ros::Time& now,
                      uint8_t* out_reason,
                      std::string* out_detail,
                      double* out_age_scan = nullptr,
                      double* out_age_odom = nullptr,
                      double* out_age_imu  = nullptr);

  void forceFail_(const ros::Time& now, uint8_t reason,
                  const std::string& detail);
  void clearForceFailIfRecovered_(const ros::Time& now);
  void watchdogTimerCb_(const ros::TimerEvent& ev);

private:
  // ===================== ROS =====================
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber scan_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber initialpose_sub_;

  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;

  ros::ServiceServer srv_algo_enable_;
  ros::ServiceServer srv_algo_switch_map_;
  ros::ServiceServer srv_algo_set_initial_pose_;

  // ===================== RViz / viz helpers =====================
  ros::Publisher scan_cloud_viz_pub_;  // scan cloud already in map frame
  DiagnosticsDtcPublisher dtc_pub_;

  // ===================== topics / frames =====================
  std::string map_json_path_;
  std::string map_stem_;
  std::string map_name_;
  int map_layer_ = 1;
  std::string scan_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string base_frame_id_ = "base_footprint";

  // ===================== config (ALL tunables live here) =====================
  localization_ndt::config::NdtLocalizerConfig cfg_;

  // ===================== map / core modules =====================
  PointCloudT::Ptr map_;

  laser_geometry::LaserProjection projector_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  SimplePredictor predictor_;
  NdtMatcher ndt_matcher_;

  // ===================== dynamic point filter =====================
  DynamicPointFilter dyn_filter_;
  DynamicFilterConfig dyn_cfg_;

  // ===================== initial pose manager =====================
  InitialPoseManager initial_pose_mgr_;
  std::string pkg_path_;        // package root
  std::string pose_base_dir_;   // config/initial_pose
  std::string pose_fixed_dir_;  // config/initial_pose/fixed
  std::string pose_last_dir_;   // config/initial_pose/last

  std::string params_dir_;            // config/ndt
  std::string interface_config_dir_;  // config/interface

  std::string algo_name_ = "localization_ndt";
  bool algo_enabled_ = true;
  std::string loc_manager_regist_srv_ = "/regist_loc";
  std::string map_server_get_map_srv_ = "/get_map";

  struct InitCandidate {
    InitialPoseManager::Pose2D pose;
    enum class Source { LAST, LAST_SHUTDOWN, FIXED } src = Source::FIXED;
    int fixed_index = -1;  // FIXED 时有效：对应 fixedPoses() 的下标
    std::string name;      // 日志用
    int priority = 0;      // 日志用
  };
  enum class InitStage { FAST, FULL };
  InitStage init_stage_ = InitStage::FAST;
  std::atomic<bool> shutdown_written_{false};
  // ---- init runtime state ----
  bool init_exhausted_ = false;  // 本轮候选是否耗尽并冻结
  std::vector<InitCandidate> init_candidates_;
  std::vector<uint8_t> init_tried_;  // try_once=true 时使用
  std::size_t init_next_idx_ = 0;

  ros::Time init_last_rearm_time_;
  ros::Time init_rearm_stationary_start_;

  bool has_init_exhausted_anchor_ = false;
  Eigen::Vector2f init_exhausted_odom_xy_ = Eigen::Vector2f::Zero();

  // ===================== sensor caches =====================
  // IMU
  double last_imu_gyro_z_ = 0.0;  // rad/s
  bool has_last_imu_ = false;
  ros::Time last_imu_stamp_;

  // Odom
  nav_msgs::Odometry last_odom_msg_;
  bool has_last_odom_msg_ = false;
  double latest_linear_speed_ = 0.0;
  ros::Time last_odom_stamp_;

  // ===================== init flag =====================
  bool initial_pose_applied_ = false;

  // ===================== stationary -> write last pose (runtime) =====================
  bool stationary_phase_ = false;
  bool stationary_pose_written_ = false;
  ros::Time stationary_start_time_;

  // ===================== global loc level (runtime) =====================
  LocLevel loc_level_ = LocLevel::INITIALIZING;

  bool has_last_output_position_ = false;
  Eigen::Vector3f last_output_position_ = Eigen::Vector3f::Zero();
  bool has_prev_output_position_ = false;
  Eigen::Vector3f prev_output_position_ = Eigen::Vector3f::Zero();

  int dr_frames_since_last_ndt_ = 0;
  double dr_distance_since_last_ndt_ = 0.0;
  bool has_ever_had_good_ndt_ = false;
  bool degen_active_latched_ = false;
  int degen_active_streak_ = 0;
  int degen_hessian_miss_streak_ = 0;
  // Degeneracy state machine:
  // - enter immediately when degen is observed
  // - keep degen handling for at least N frames
  // - then require M consecutive safe frames to exit
  int degen_hold_frames_left_ = 0;
  int degen_exit_safe_streak_ = 0;
  bool has_degen_last_motion_dir_ = false;
  Eigen::Vector2f degen_last_motion_dir_ = Eigen::Vector2f(1.0f, 0.0f);

  // hard-degen + ambiguity short DR hold state
  bool degen_dr_hold_active_ = false;
  int degen_dr_hold_frames_left_ = 0;
  int degen_dr_hold_trigger_count_ = 0;
  int degen_dr_hold_recover_count_ = 0;

  // degeneration cruise DR state (distance-based)
  bool degen_dr_cruise_active_ = false;
  int degen_dr_cruise_trigger_count_ = 0;
  int degen_dr_cruise_frames_ = 0;
  double degen_dr_cruise_dist_accum_ = 0.0;
  bool degen_dr_cruise_has_last_xy_ = false;
  Eigen::Vector2f degen_dr_cruise_last_xy_ = Eigen::Vector2f::Zero();

  // intensity prefilter adaptive relax state (frames remaining)
  int intensity_prefilter_adaptive_frames_left_ = 0;

  // ===================== temporal caches (runtime) =====================
  // Temporal-A cache（仅在 gate 通过或显式同步时刷新）
  Eigen::Matrix4f T_last_temporal_ = Eigen::Matrix4f::Identity();
  ros::Time t_last_temporal_;
  bool has_last_temporal_ = false;

  // Temporal-B cache（真正 publish 的输出）
  Eigen::Matrix4f T_last_published_ = Eigen::Matrix4f::Identity();
  ros::Time t_last_published_;
  bool has_last_published_ = false;

  std::deque<Odom2D> odom_buf_;
  std::deque<GyroZ> gyro_buf_;

  double odom_buf_keep_sec_ = 3.0;  // 缓存 3s 足够
  double imu_buf_keep_sec_ = 3.0;

  // 拖拽重定位 grace：跳过 temporal（runtime）
  ros::Time temporal_grace_until_;

  // ===================== local relocalization (runtime) =====================
  RelocalizationManager reloc_mgr_;
  std::unique_ptr<LocalRelocalizer2D> local_reloc_;

  // FAIL edge 需要的状态
  LocLevel prev_loc_level_ = LocLevel::INITIALIZING;

  // 记录最后一次 NDT accept 的位姿，用作 FAIL 触发时的 center
  bool has_last_good_pose_ = false;
  RelocPose2D last_good_pose_;

  // ===== FAIL recovery scheduler (runtime) =====
  ros::Time fail_stationary_start_;      // FAIL 状态下停稳起点
  ros::Time fail_last_periodic_local_;   // FAIL+停稳：上次 2s 周期 local 调度时刻

  Eigen::Vector2f fail_anchor_odom_xy_{0.f, 0.f}; // FAIL 锚点（用于 moved 判定）
  bool has_fail_anchor_ = false;

  // ===================== Input Watchdog (runtime) =====================
  ros::Time last_scan_stamp_;
  bool has_last_scan_ = false;

  // 强制 FAIL 状态（独立于 loc_level_，避免触发 fail_edge/local_reloc 等副作用）
  bool sensor_force_fail_ = false;
  uint8_t sensor_force_reason_ = 0;
  ros::Time sensor_force_until_;
  std::string sensor_force_detail_;

  // watchdog timer（用于 scan 断流时也能报 FAIL）
  ros::Timer watchdog_timer_;

  // ---- FAIL C2 recovery state ----
  bool fail_moved_far_since_edge_ = false;

  // fixed+last 遍历的“attempt anchor”：用于失败后要求再次移动够距离才重试
  Eigen::Vector2f fail_attempt_anchor_odom_xy_ = Eigen::Vector2f::Zero();
  bool has_fail_attempt_anchor_ = false;

  Eigen::Matrix3d T_map_odom_anchor_ = Eigen::Matrix3d::Identity();
  bool has_map_odom_anchor_ = false;
};

}  // namespace localization_ndt
