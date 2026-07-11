// DTC Catalog 
// level: 0=OK, 1=WARN, 2=ERROR, 3=STALE (ROS diagnostic convention)
//
// Base DTCs (SetBase):
// 3000 OK / Localizing
// 3001 Generic FAIL
// 3019 Initializing
// 3010 Scan timeout
// 3011 Odom timeout
// 3012 Imu timeout
// 3013 Multi timeout
//
// Event DTCs (Event hold for N seconds then fallback to base):
// 3014 TF lookup failed
// 3016 Empty / too few points
// 3017 Local relocalization success
// 3018 Local relocalization failed

#pragma once
#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#include <atomic>
#include <string>
#include <algorithm>

namespace localization_ndt {

class DiagnosticsDtcPublisher {
public:
  void Init(ros::NodeHandle& nh,
            const std::string& topic = "/diagnostics",
            double hz = 1.0,
            const std::string& name = "localization",
            const std::string& hardware_id = "ndt_localizer") {
    name_ = name;
    hardware_id_ = hardware_id;

    pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>(topic, 1);

    const double safe_hz = std::max(0.1, hz);
    timer_ = nh.createWallTimer(
        ros::WallDuration(1.0 / safe_hz),
        &DiagnosticsDtcPublisher::WallTimerCb_, this);

    SetBase(1, 3019);
  }

  // 常态：会一直发布（除非被 Event 临时覆盖）
  void SetBase(uint8_t level, int dtc_id) {
    base_level_.store(level, std::memory_order_relaxed);
    base_dtc_.store(dtc_id, std::memory_order_relaxed);
  }

  // 瞬时：短时间覆盖 base，然后自动回退
  // 用 WallTime 计时，避免 /use_sim_time 时 override 不过期
  void Event(uint8_t level, int dtc_id, double hold_sec = 1.0) {
    const double now = ros::WallTime::now().toSec();
    ovr_level_.store(level, std::memory_order_relaxed);
    ovr_dtc_.store(dtc_id, std::memory_order_relaxed);
    ovr_until_.store(now + std::max(0.0, hold_sec), std::memory_order_relaxed);
  }

  // 对外给一个“直接发一条（不等 timer）”
  void PublishOnce() { Publish_(); }

  void Shutdown() {
    if (timer_.hasStarted()) {
      timer_.stop();
    }
    if (pub_) {
      pub_.shutdown();
    }
  }

private:
  void WallTimerCb_(const ros::WallTimerEvent&) { Publish_(); }

  void Publish_() {
    if (!pub_) return;

    int dtc = base_dtc_.load(std::memory_order_relaxed);
    uint8_t lvl = base_level_.load(std::memory_order_relaxed);

    const double now = ros::WallTime::now().toSec();
    const double until = ovr_until_.load(std::memory_order_relaxed);
    if (now < until) {
      dtc = ovr_dtc_.load(std::memory_order_relaxed);
      lvl = ovr_level_.load(std::memory_order_relaxed);
    }

    diagnostic_msgs::DiagnosticArray arr;

    // stamp：优先用 ros::Time::now()；若为 0（无 /clock），退化为 wall time
    ros::Time stamp = ros::Time::now();
    if (stamp.isZero()) {
      stamp.fromSec(ros::WallTime::now().toSec());
    }
    arr.header.stamp = stamp;

    diagnostic_msgs::DiagnosticStatus st;
    st.level = lvl;                    // 0 OK, 1 WARN, 2 ERROR, 3 STALE
    st.name = name_;
    st.message = std::to_string(dtc);  // dtc_id 放 message
    st.hardware_id = hardware_id_;

    arr.status.push_back(st);
    pub_.publish(arr);
  }

  ros::Publisher pub_;
  ros::WallTimer timer_;

  std::string name_{"localization"}; //模块名
  std::string hardware_id_{"ndt_localizer"}; //节点名

  // 对齐：默认 initializing 应为 WARN(1)
  std::atomic<int> base_dtc_{3019};
  std::atomic<uint8_t> base_level_{1};

  std::atomic<int> ovr_dtc_{0};
  std::atomic<uint8_t> ovr_level_{0};
  std::atomic<double> ovr_until_{0.0};
};

} // namespace localization_ndt
