#pragma once

#include <mutex>
#include <Eigen/Dense>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

namespace localization_ndt {

/**
 * @brief 简单预测器：
 * - 位置来自里程计（/odom/wheel）
 * - yaw 来自 IMU 积分（angular_velocity.z）
 *
 * 工作方式：
 *  1）内部维护一个“上一次校正”的状态：
 *      - T_map_base_last_  : 上一次认为可靠的 map->base 位姿（来自 NDT 或初始）
 *      - odom_last_        : 那一刻里程计的 (x, y, yaw_odom)
 *      - yaw_imu_at_last_  : 那一刻 IMU 积分的 yaw
 *
 *  2）平时：
 *      - updateOdom() 更新最新的 odom
 *      - updateImu()  积分最新的 yaw_imu_now_
 *
 *  3）predict()：
 *      - 计算 odom 增量 Δp_base（用 odom 的姿态算“自车坐标系下的位移”）
 *      - yaw 增量使用 IMU：Δyaw = yaw_imu_now_ - yaw_imu_at_last_
 *      - 得到 ΔT_base = [Rz(Δyaw), Δp_base]
 *      - 返回 T_map_base_pred = T_map_base_last_ * ΔT_base
 *
 *  初始化策略：
 *   - 如果还没有“上一次校正”，但已经收到 odom + imu，则把当前时刻当作“初始校正”：
 *       * T_map_base_last_ = Identity（即暂时认为 map 和 odom 原点重合）
 *       * odom_last_ = odom_latest_
 *       * yaw_imu_at_last_ = yaw_imu_now_
 *   这样在没有 NDT 之前，它就是一个“纯 odom+IMU 推演”的 2D 里程计。
 */
class SimplePredictor {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SimplePredictor() = default;

  /// NDT 校正成功时调用：重设“上一次校正”的锚点
  void resetWithCorrection(const Eigen::Matrix4f& T_map_base,
                           const nav_msgs::Odometry& odom_at_correction) {
    std::lock_guard<std::mutex> lock(mutex_);

    T_map_base_last_ = T_map_base;
    odom_last_ = odomMsgToPose2D(odom_at_correction);
    yaw_imu_at_last_ = yaw_imu_now_;  // 以当前 IMU yaw 作为校正锚点
    has_last_correction_ = true;
  }

  /// 里程计回调：更新最新的 2D pose (x,y,yaw_odom)
  void updateOdom(const nav_msgs::OdometryConstPtr& odom_msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_latest_ = odomMsgToPose2D(*odom_msg);
    has_latest_odom_ = true;
  }

  /// IMU 回调：积分 yaw（只用 z 轴角速度）
  void updateImu(const sensor_msgs::ImuConstPtr& imu_msg) {
    std::lock_guard<std::mutex> lock(mutex_);

    const ros::Time& stamp = imu_msg->header.stamp;
    if (!has_imu_) {
      last_imu_stamp_ = stamp;
      yaw_imu_now_ = 0.0;  // 第一帧作为 yaw=0 的参考
      has_imu_ = true;
      return;
    }

    double dt = (stamp - last_imu_stamp_).toSec();
    last_imu_stamp_ = stamp;

    if (dt <= 0.0 || dt > 0.1) {
      // 时间跳变太大/负数，简单丢掉这次积分，防止一口吃太多
      return;
    }

    double wz = imu_msg->angular_velocity.z;  // rad/s
    yaw_imu_now_ += wz * dt;
  }

  /// 预测器是否已经完全 ready（有锚点 + 有最新 odom + imu）
  bool ready() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return has_last_correction_ && has_latest_odom_ && has_imu_;
  }

  /// 计算预测位姿 T_map_base_pred
  /// - 如果还没准备好，返回 false，不修改 T_map_base_pred
  /// - 如果是第一次（还没有 last_correction_），会自动以当前 odom+imu 初始化锚点
  bool predict(Eigen::Matrix4f& T_map_base_pred) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_latest_odom_ || !has_imu_) {
      return false;
    }

    // 如果还没有校正过，用当前时刻初始化为锚点
    if (!has_last_correction_) {
      odom_last_ = odom_latest_;
      yaw_imu_at_last_ = yaw_imu_now_;
      T_map_base_last_ = Eigen::Matrix4f::Identity();  // 初始 map = odom
      has_last_correction_ = true;
      // 第一帧没有增量，预测就是锚点本身
      T_map_base_pred = T_map_base_last_;
      return true;
    }

    // 1) 计算自车坐标系下的平移增量 Δp_base
    const double x_last = odom_last_.x;
    const double y_last = odom_last_.y;
    const double yaw_odom_last = odom_last_.yaw;

    const double x_now = odom_latest_.x;
    const double y_now = odom_latest_.y;
    const double yaw_odom_now = odom_latest_.yaw;

    Eigen::Vector2d dp_odom(x_now - x_last, y_now - y_last);

    // 把位移从 odom 坐标系旋转到“上一帧 base 坐标系”下
    double cos_last = std::cos(yaw_odom_last);
    double sin_last = std::sin(yaw_odom_last);
    Eigen::Matrix2d R_odom_last_T;
    R_odom_last_T <<  cos_last, sin_last,
                     -sin_last, cos_last;

    Eigen::Vector2d dp_base = R_odom_last_T * dp_odom;  // 自车坐标系下的位移

    // 2) yaw 增量用 IMU：Δyaw_imu
    double dyaw_imu = yaw_imu_now_ - yaw_imu_at_last_;

    // 3) 构造 ΔT_base_k = [Rz(dyaw_imu), Δp_base]
    Eigen::Matrix4d T_delta = Eigen::Matrix4d::Identity();
    Eigen::AngleAxisd yaw_rot(dyaw_imu, Eigen::Vector3d::UnitZ());
    T_delta.block<3,3>(0,0) = yaw_rot.toRotationMatrix();
    T_delta(0,3) = dp_base.x();
    T_delta(1,3) = dp_base.y();
    T_delta(2,3) = 0.0;

    // 4) map 下的预测位姿：T_map_base_pred = T_map_base_last_ * ΔT
    T_map_base_pred = T_map_base_last_ * T_delta.cast<float>();

    return true;
  }

  /// 给 NDT 用：当某帧匹配成功，并且用 T_map_base_pred 作为最终位姿时，
  /// 记得调用这个函数更新“上一帧状态”为最新。
  void commitPredictionAsCorrection(const Eigen::Matrix4f& T_map_base_pred) {
    // 这里默认在外面已经更新过 odom_latest_ & yaw_imu_now_
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_latest_odom_ || !has_imu_) {
      return;
    }
    T_map_base_last_  = T_map_base_pred;
    odom_last_        = odom_latest_;
    yaw_imu_at_last_  = yaw_imu_now_;
    has_last_correction_ = true;
  }

  void reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    T_map_base_last_ = Eigen::Matrix4f::Identity();
    has_last_correction_ = false;
    odom_last_ = Pose2D();
    odom_latest_ = Pose2D();
    has_latest_odom_ = false;
    has_imu_ = false;
    last_imu_stamp_ = ros::Time(0);
    yaw_imu_now_ = 0.0;
    yaw_imu_at_last_ = 0.0;
  }

private:
  struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
  };

  static Pose2D odomMsgToPose2D(const nav_msgs::Odometry& odom) {
    Pose2D pose;
    pose.x = odom.pose.pose.position.x;
    pose.y = odom.pose.pose.position.y;

    const auto& q = odom.pose.pose.orientation;
    Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
    quat.normalize();

    // 提取 yaw（绕 z 轴）
    pose.yaw = std::atan2(
      2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
      1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z())
    );

    return pose;
  }

private:
  mutable std::mutex mutex_;

  // 上一次“校正”时的 map->base
  Eigen::Matrix4f T_map_base_last_ = Eigen::Matrix4f::Identity();
  bool has_last_correction_ = false;

  // 里程计：上一帧 & 最新帧
  Pose2D odom_last_;
  Pose2D odom_latest_;
  bool   has_latest_odom_ = false;

  // IMU yaw
  bool      has_imu_ = false;
  ros::Time last_imu_stamp_;
  double    yaw_imu_now_ = 0.0;
  double    yaw_imu_at_last_ = 0.0;
};

}  // namespace localization_ndt
