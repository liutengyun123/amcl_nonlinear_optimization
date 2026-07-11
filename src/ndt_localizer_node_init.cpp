#include "localization_ndt/ndt_localizer_node.hpp"

#include <cctype>
#include <exception>

#include <ros/package.h>

#include "localization_ndt/config/ndt_localizer_config.hpp"
#include "localization_ndt/config/run_config.hpp"
#include "localization_ndt/config/yaml_utils.hpp"

namespace localization_ndt {
namespace {

static inline bool ParseMapLayerSuffix(const std::string &in,
                                       std::string *base,
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

} // namespace

void NdtLocalizerNode::initDirs_(const std::string &pkg_path) {
  pnh_.param<std::string>("pose_base_dir", pose_base_dir_,
                          pkg_path + "/config/initial_pose");
  pose_fixed_dir_ = pose_base_dir_ + "/fixed";
  pose_last_dir_ = pose_base_dir_ + "/last";

  pnh_.param<std::string>("params_dir", params_dir_,
                          pkg_path + "/config/ndt");
  pnh_.param<std::string>("interface_config_dir", interface_config_dir_,
                          pkg_path + "/config/interface");

  localization_ndt::config::EnsureDir(pose_base_dir_);
  localization_ndt::config::EnsureDir(pose_fixed_dir_);
  localization_ndt::config::EnsureDir(pose_last_dir_);
  localization_ndt::config::EnsureDir(params_dir_);
  localization_ndt::config::EnsureDir(interface_config_dir_);
}

void NdtLocalizerNode::resolveRunSelection_(const std::string &pkg_path,
                                            std::string *out_map,
                                            int *out_id) {
  std::string run_config_dir = pkg_path + "/config/run";
  pnh_.param<std::string>("run_config_dir", run_config_dir, run_config_dir);
  localization_ndt::config::EnsureDir(run_config_dir);

  const std::string run_yaml_path = run_config_dir + "/run.yaml";

  localization_ndt::config::RunSelection sel;
  std::string err;
  if (!localization_ndt::config::ResolveRunSelection(run_yaml_path, &sel,
                                                      &err)) {
    ROS_FATAL_STREAM(err);
    throw std::runtime_error("run.yaml selection failed");
  }

  if (out_map)
    *out_map = sel.map_name;
  if (out_id)
    *out_id = sel.run_target;

  ROS_INFO_STREAM("Using run_target id=" << sel.run_target
                                          << " (map=" << sel.map_name << ")");
}

void NdtLocalizerNode::resolveMapPath_(const std::string & /*pkg_path*/,
                                       const std::string &map_name,
                                       std::string *out_map_stem) {
  if (out_map_stem) {
    *out_map_stem = localization_ndt::config::ExtractFileStem(map_name);
  }
}

void NdtLocalizerNode::loadConfig_(const std::string &pkg_path,
                                   const std::string &map_stem) {
  (void)localization_ndt::config::LoadNdtLocalizerConfig(pkg_path, params_dir_,
                                                          map_stem, pnh_, &cfg_);
  reloc_mgr_.setParams(cfg_.reloc_mgr_params);
  applyConfigToDynFilter_();
  dyn_filter_.setConfig(dyn_cfg_);
}

void NdtLocalizerNode::loadInterfaceConfig_() {
  const std::string interface_yaml = interface_config_dir_ + "/interface.yaml";

  YAML::Node iface;
  if (localization_ndt::config::FileExists(interface_yaml)) {
    try {
      iface = YAML::LoadFile(interface_yaml);
    } catch (const std::exception &e) {
      ROS_WARN_STREAM("[LOC] load interface yaml failed: " << interface_yaml
                      << " err=" << e.what());
    }
  } else {
    iface["scan_topic"] = "/scan";
    iface["odom_topic"] = "/odom/wheel";
    iface["imu_topic"] = "/imu";
    iface["base_frame_id"] = "base_footprint";
    (void)localization_ndt::config::SaveYamlToFile(interface_yaml, iface);
    ROS_WARN_STREAM("[LOC] created interface yaml template: " << interface_yaml);
  }

  std::string default_scan_topic = "/scan";
  std::string default_odom_topic = "/odom/wheel";
  std::string default_imu_topic = "/imu";
  std::string default_base_frame_id = "base_footprint";

  using localization_ndt::config::LoadFieldFromYaml;
  LoadFieldFromYaml(iface, "scan_topic", default_scan_topic);
  LoadFieldFromYaml(iface, "odom_topic", default_odom_topic);
  LoadFieldFromYaml(iface, "imu_topic", default_imu_topic);
  LoadFieldFromYaml(iface, "base_frame_id", default_base_frame_id);

  pnh_.param<std::string>("scan_topic", scan_topic_, default_scan_topic);
  pnh_.param<std::string>("odom_topic", odom_topic_, default_odom_topic);
  pnh_.param<std::string>("imu_topic", imu_topic_, default_imu_topic);
  pnh_.param<std::string>("base_frame_id", base_frame_id_,
                          default_base_frame_id);
}

void NdtLocalizerNode::initRosInterfaces_() {
  odom_sub_ =
      nh_.subscribe(odom_topic_, 100, &NdtLocalizerNode::odomCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic_, 200, &NdtLocalizerNode::imuCallback,
                           this);
  scan_sub_ =
      nh_.subscribe(scan_topic_, 5, &NdtLocalizerNode::scanCallback, this);

  initialpose_sub_ = nh_.subscribe("initialpose", 1,
                                   &NdtLocalizerNode::initialPoseCallback,
                                   this);

  if (cfg_.viz_publish_scan_cloud) {
    scan_cloud_viz_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        cfg_.viz_scan_cloud_topic, 2, false);
  }
}

void NdtLocalizerNode::initWatchdog_() {
  if (!cfg_.watchdog_enable)
    return;

  const double hz = std::max(1.0, cfg_.watchdog_timer_hz);
  watchdog_timer_ = nh_.createTimer(ros::Duration(1.0 / hz),
                                    &NdtLocalizerNode::watchdogTimerCb_, this);

  ROS_INFO_STREAM("[LOC] watchdog enabled"
                  << " hz=" << hz << " scan(warn="
                  << cfg_.watchdog_scan_warn_sec
                  << ", fail=" << cfg_.watchdog_scan_fail_sec << ")"
                  << " odom(warn=" << cfg_.watchdog_odom_warn_sec
                  << ", fail=" << cfg_.watchdog_odom_fail_sec << ")"
                  << " imu(warn=" << cfg_.watchdog_imu_warn_sec
                  << ", fail=" << cfg_.watchdog_imu_fail_sec << ")"
                  << " hold=" << cfg_.watchdog_fail_hold_sec);
}

void NdtLocalizerNode::initMatcher_() {
  ndt_matcher_.setResolution(cfg_.ndt_resolution);
  ndt_matcher_.setNumThreads(cfg_.ndt_num_threads);
}

void NdtLocalizerNode::initLocalReloc_() {
  if (!cfg_.local_reloc_enable)
    return;

  local_reloc_.reset(new LocalRelocalizer2D());
  local_reloc_->setPyrMaxLevel(cfg_.local_opt_base.pyr_max_level);
  local_reloc_->setScoreParams(cfg_.local_opt_base.hit_sigma_m,
                               cfg_.local_opt_base.max_dist_m);

  std::string err;
  if (!local_reloc_->loadMap(cfg_.local_reloc_map_yaml, &err)) {
    ROS_WARN_STREAM("[LOC] local_reloc map load failed: "
                    << err << " -> disable local_reloc");
    cfg_.local_reloc_enable = false;
    local_reloc_.reset();
  }
}

bool NdtLocalizerNode::bootstrapFromRunSelection_(std::string *out_err) {
  if (out_err)
    out_err->clear();

  if (pkg_path_.empty()) {
    pkg_path_ = ros::package::getPath("localization_ndt");
  }
  if (pkg_path_.empty()) {
    if (out_err)
      *out_err = "ros::package::getPath(localization_ndt) failed";
    return false;
  }

  initDirs_(pkg_path_);

  std::string map_name;
  int run_target = 0;
  try {
    resolveRunSelection_(pkg_path_, &map_name, &run_target);
  } catch (const std::exception &e) {
    if (out_err)
      *out_err = e.what();
    return false;
  }

  pnh_.param<std::string>("map_name", map_name, map_name);

  int map_layer = 0;
  pnh_.param("map_layer", map_layer, map_layer);

  std::string base;
  int layer_from_name = 0;
  if (ParseMapLayerSuffix(map_name, &base, &layer_from_name)) {
    map_name = base;
    if (map_layer <= 0)
      map_layer = layer_from_name;
  }
  if (map_layer <= 0)
    map_layer = 1;

  std::string map_json_path;
  pnh_.param<std::string>("map_json_path", map_json_path, std::string(""));

  std::string fetched_map;
  int fetched_layer = 0;

  if (map_json_path.empty()) {
    std::string active_fetch_err;
    if (fetchMapJsonFromServer_("", 0, &map_json_path, &active_fetch_err,
                                &fetched_map, &fetched_layer)) {
      if (!fetched_map.empty())
        map_name = fetched_map;
      if (fetched_layer > 0)
        map_layer = fetched_layer;

      ROS_INFO_STREAM("[LOC] bootstrap map from map_server active: map="
                      << map_name << " layer=" << map_layer
                      << " json=" << map_json_path);
    } else {
      if (map_name.empty()) {
        if (out_err)
          *out_err = "map name is empty in run selection";
        return false;
      }

      std::string fallback_fetch_err;
      if (!fetchMapJsonFromServer_(map_name, map_layer, &map_json_path,
                                   &fallback_fetch_err, &fetched_map,
                                   &fetched_layer)) {
        if (out_err)
          *out_err = fallback_fetch_err;
        return false;
      }

      if (!fetched_map.empty())
        map_name = fetched_map;
      if (fetched_layer > 0)
        map_layer = fetched_layer;

      ROS_WARN_STREAM("[LOC] map_server active query failed: "
                      << active_fetch_err
                      << "; fallback to run selection map=" << map_name
                      << " layer=" << map_layer
                      << " json=" << map_json_path);
    }
  }

  if (map_name.empty()) {
    map_name = localization_ndt::config::ExtractFileStem(map_json_path);
  }
  if (map_layer <= 0)
    map_layer = 1;

  map_name_ = map_name;
  map_layer_ = map_layer;

  loadInterfaceConfig_();

  initial_pose_mgr_.setFixedDir(pose_fixed_dir_);
  initial_pose_mgr_.setLastDir(pose_last_dir_);

  std::string apply_err;
  if (!applyMapSwitch_(map_name_, map_layer_, map_json_path, &apply_err)) {
    if (out_err)
      *out_err = apply_err;
    return false;
  }

  algo_enabled_ = true;
  return true;
}

NdtLocalizerNode::NdtLocalizerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf_buffer_(), tf_listener_(tf_buffer_) {
  pkg_path_ = ros::package::getPath("localization_ndt");
  if (pkg_path_.empty()) {
    ROS_FATAL("Failed to get package path for 'localization_ndt'.");
    throw std::runtime_error("ros::package::getPath failed");
  }

  pnh_.param<std::string>("algo_name", algo_name_, algo_name_);
  pnh_.param<std::string>("loc_manager_regist_srv", loc_manager_regist_srv_,
                          loc_manager_regist_srv_);
  pnh_.param<std::string>("map_server_get_map_srv", map_server_get_map_srv_,
                          map_server_get_map_srv_);

  bool wait_for_enable = false;
  pnh_.param("wait_for_enable", wait_for_enable, wait_for_enable);

  // control interfaces (services/pubs)
  pose_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_odom", 10, false);
  status_pub_ = nh_.advertise<localization_msgs::LocalizationStatus>(
      "localization_status", 10, false);

  srv_algo_enable_ =
      pnh_.advertiseService("algo_enable", &NdtLocalizerNode::onAlgoEnable,
                            this);
  srv_algo_switch_map_ = pnh_.advertiseService(
      "algo_switch_map", &NdtLocalizerNode::onAlgoSwitchMap, this);
  srv_algo_set_initial_pose_ = pnh_.advertiseService(
      "algo_set_initial_pose", &NdtLocalizerNode::onAlgoSetInitialPose, this);

  dtc_pub_.Init(nh_, "/diagnostics", 1.0 /*Hz*/, "localization",
                "ndt_localizer");

  (void)registerToLocManager_();

  if (wait_for_enable) {
    algo_enabled_ = false;
    dtc_pub_.Shutdown();
    ROS_INFO_STREAM("[LOC] wait_for_enable=1: defer init until algo_enable");
    return;
  }

  std::string err;
  if (!bootstrapFromRunSelection_(&err)) {
    ROS_FATAL_STREAM("[LOC] bootstrap failed: " << err);
    throw std::runtime_error("bootstrap failed");
  }

  initRosInterfaces_();
  initWatchdog_();
  resetRuntimeState_();
  logEffectiveConfig_(map_name_, map_stem_);
}

} // namespace localization_ndt
