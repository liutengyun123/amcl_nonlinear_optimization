#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "localization_ndt/types.hpp"

namespace localization_ndt {

class DistanceField2D {
public:
  DistanceField2D() = default;

  bool buildFromMap(const PointCloudT& map,
                    double resolution,
                    double inflate_radius_m,
                    double padding_m);

  bool isReady() const { return ready_; }

  // 距离到最近占据栅格(含inflate)的欧式距离（米）
  float distance(float x, float y) const;

private:
  bool worldToGrid(float x, float y, int* ix, int* iy) const;

  bool ready_{false};
  float res_{0.1f};
  float ox_{0.f}, oy_{0.f};
  int w_{0}, h_{0};
  std::vector<float> dist_;
};

}  // namespace localization_ndt
