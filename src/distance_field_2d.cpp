#include "localization_ndt/distance_field_2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace localization_ndt {
namespace {

constexpr float kInf = 1e20f;

// 1D squared distance transform (Felzenszwalb & Huttenlocher)
static void dt1d(const float* f, int n, float* d) {
  std::vector<int> v(n);
  std::vector<float> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;

  auto sq = [](float x) { return x * x; };

  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + sq((float)q)) - (f[v[k]] + sq((float)v[k])))
              / (2.f * (q - v[k]));
    while (k > 0 && s <= z[k]) {
      --k;
      s = ((f[q] + sq((float)q)) - (f[v[k]] + sq((float)v[k])))
          / (2.f * (q - v[k]));
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }

  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    const int vk = v[k];
    d[q] = sq((float)(q - vk)) + f[vk];
  }
}

static inline int idx(int x, int y, int w) { return y * w + x; }

}  // namespace

bool DistanceField2D::worldToGrid(float x, float y, int* ix, int* iy) const {
  if (!ix || !iy) return false;
  const float fx = (x - ox_) / res_;
  const float fy = (y - oy_) / res_;
  const int gx = (int)std::floor(fx);
  const int gy = (int)std::floor(fy);
  if (gx < 0 || gy < 0 || gx >= w_ || gy >= h_) return false;
  *ix = gx;
  *iy = gy;
  return true;
}

float DistanceField2D::distance(float x, float y) const {
  if (!ready_) return std::numeric_limits<float>::infinity();
  int ix, iy;
  if (!worldToGrid(x, y, &ix, &iy)) return std::numeric_limits<float>::infinity();
  return dist_[idx(ix, iy, w_)];
}

bool DistanceField2D::buildFromMap(const PointCloudT& map,
                                  double resolution,
                                  double inflate_radius_m,
                                  double padding_m) {
  ready_ = false;
  dist_.clear();
  if (map.empty()) return false;

  res_ = (float)std::max(1e-3, resolution);

  float minx = std::numeric_limits<float>::infinity();
  float miny = std::numeric_limits<float>::infinity();
  float maxx = -std::numeric_limits<float>::infinity();
  float maxy = -std::numeric_limits<float>::infinity();

  for (const auto& p : map.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
    minx = std::min(minx, p.x);
    miny = std::min(miny, p.y);
    maxx = std::max(maxx, p.x);
    maxy = std::max(maxy, p.y);
  }
  if (!std::isfinite(minx)) return false;

  const float pad = (float)std::max(0.0, padding_m);
  minx -= pad; miny -= pad;
  maxx += pad; maxy += pad;

  ox_ = minx;
  oy_ = miny;
  w_ = (int)std::ceil((maxx - minx) / res_) + 1;
  h_ = (int)std::ceil((maxy - miny) / res_) + 1;
  if (w_ <= 1 || h_ <= 1) return false;

  std::vector<uint8_t> occ((size_t)w_ * (size_t)h_, 0);

  // mark occupied
  for (const auto& p : map.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
    int ix, iy;
    if (!worldToGrid(p.x, p.y, &ix, &iy)) continue;
    occ[idx(ix, iy, w_)] = 1;
  }

  // inflate occupied
  const int r = (int)std::ceil(std::max(0.0, inflate_radius_m) / res_);
  if (r > 0) {
    std::vector<std::pair<int, int> > offsets;
    offsets.reserve((2 * r + 1) * (2 * r + 1));
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        if (dx * dx + dy * dy <= r * r) offsets.push_back(std::make_pair(dx, dy));
      }
    }

    std::vector<int> occ_cells;
    occ_cells.reserve((size_t)w_ * (size_t)h_);
    for (int y = 0; y < h_; ++y) {
      for (int x = 0; x < w_; ++x) {
        if (occ[idx(x, y, w_)]) occ_cells.push_back(idx(x, y, w_));
      }
    }

    std::vector<uint8_t> occ2 = occ;
    for (int id : occ_cells) {
      const int y = id / w_;
      const int x = id - y * w_;
      for (const auto& off : offsets) {
        const int nx = x + off.first;
        const int ny = y + off.second;
        if ((unsigned)nx >= (unsigned)w_ || (unsigned)ny >= (unsigned)h_) continue;
        occ2[idx(nx, ny, w_)] = 1;
      }
    }
    occ.swap(occ2);
  }

  // f = 0 for occupied, INF for free
  std::vector<float> f((size_t)w_ * (size_t)h_, kInf);
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) {
      if (occ[idx(x, y, w_)]) f[idx(x, y, w_)] = 0.f;
    }
  }

  // EDT: along x
  std::vector<float> tmp((size_t)w_ * (size_t)h_, kInf);
  std::vector<float> row_in((size_t)w_), row_out((size_t)w_);
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) row_in[(size_t)x] = f[idx(x, y, w_)];
    dt1d(row_in.data(), w_, row_out.data());
    for (int x = 0; x < w_; ++x) tmp[idx(x, y, w_)] = row_out[(size_t)x];
  }

  // EDT: along y
  dist_.assign((size_t)w_ * (size_t)h_, std::numeric_limits<float>::infinity());
  std::vector<float> col_in((size_t)h_), col_out((size_t)h_);
  for (int x = 0; x < w_; ++x) {
    for (int y = 0; y < h_; ++y) col_in[(size_t)y] = tmp[idx(x, y, w_)];
    dt1d(col_in.data(), h_, col_out.data());
    for (int y = 0; y < h_; ++y) {
      const float d2 = col_out[(size_t)y];
      const float d_cells = (d2 >= kInf * 0.5f)
                                ? std::numeric_limits<float>::infinity()
                                : std::sqrt(std::max(0.f, d2));
      dist_[idx(x, y, w_)] = d_cells * res_;
    }
  }

  ready_ = true;
  return true;
}

}  // namespace localization_ndt
