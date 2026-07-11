#pragma once

#include <memory>
#include <vector>
#include <limits>

#include <Eigen/Dense>
#include <pclomp/ndt_omp.h>
#include <pcl/registration/registration.h>  // 为了 getSearchMethodTarget

#include "localization_ndt/types.hpp"

namespace localization_ndt {

class NdtMatcher {
 public:
  using PointT = localization_ndt::PointT;
  using PointCloudT = localization_ndt::PointCloudT;

  NdtMatcher() {
    ndt_.reset(new pclomp::NormalDistributionsTransform<PointT, PointT>());
    // 一些默认参数，可以在外面再覆盖
    ndt_->setTransformationEpsilon(0.01);
    ndt_->setResolution(1.0);
    ndt_->setStepSize(0.1);
    ndt_->setMaximumIterations(30);
    ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
  }

  void setResolution(double res) { ndt_->setResolution(res); }
  void setNumThreads(int n_threads) { ndt_->setNumThreads(n_threads); }
  void setTransformationEpsilon(double eps) { ndt_->setTransformationEpsilon(eps); }
  void setStepSize(double step) { ndt_->setStepSize(step); }
  void setMaxIterations(int iters) { ndt_->setMaximumIterations(iters); }
  int getMaxIterations() const { return ndt_->getMaximumIterations(); }

  void setTargetMap(const PointCloudT::ConstPtr& map) {
    ndt_->setInputTarget(map);
    has_target_ = true;
  }

  double getTransformationProbability() const {
    return ndt_->getTransformationProbability();
  }

  bool getFinalHessian(Eigen::Matrix<double, 6, 6>* out) const {
    return ndt_->getFinalHessian(out);
  }

  /// @brief 对当前帧做一次 NDT 匹配
  /// @param cloud 当前帧（在 base_frame 下）
  /// @param init_guess 初始位姿 T_map_base
  /// @param final_pose 输出的最终位姿 T_map_base
  /// @param score NDT 自带的 fitness_score（cost），仅用于监控 / 输出
  /// @param aligned_out 输出对齐到 map 坐标系下的点云
  /// @return 是否成功（收敛 && 有 target map）
  bool align(const PointCloudT::ConstPtr& cloud,
             const Eigen::Matrix4f& init_guess,
             Eigen::Matrix4f& final_pose,
             double& score,
             PointCloudT::Ptr aligned_out = nullptr) {
    if (!has_target_) {
      return false;
    }

    ndt_->setInputSource(cloud);
    PointCloudT aligned;
    ndt_->align(aligned, init_guess);

    if (!ndt_->hasConverged()) {
      return false;
    }

    final_pose = ndt_->getFinalTransformation();
    score = ndt_->getFitnessScore();

    if (aligned_out) {
      *aligned_out = aligned;
    }
    return true;
  }

 

 private:
  pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt_;
  bool has_target_ = false;
};

}  // namespace localization_ndt
