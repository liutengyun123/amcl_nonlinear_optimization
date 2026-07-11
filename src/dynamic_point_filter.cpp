#include "localization_ndt/dynamic_point_filter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

namespace localization_ndt {

bool DynamicPointFilter::setMap(const PointCloudT::ConstPtr& map) {
  if (!map || map->empty()) return false;
  return df_.buildFromMap(*map, cfg_.grid_res_m, cfg_.inflate_m, cfg_.padding_m);
}

static inline double percentile_inplace(std::vector<float>& v, double p01) {
  if (v.empty()) return 0.0;
  p01 = std::min(1.0, std::max(0.0, p01));
  const size_t k = (size_t)std::floor(p01 * (v.size() - 1));
  std::nth_element(v.begin(), v.begin() + k, v.end());
  return (double)v[k];
}

// Distance-field sphere tracing：返回射线首次触碰“膨胀占用边界”的距离 t（单位：米）
static inline float RaycastExpectedRangeDF_FixedStep(
    const DistanceField2D& df,
    const Eigen::Vector2f& o_map,
    const Eigen::Vector2f& dir_unit_map,
    float max_range,
    float hit_eps,
    float step,
    int max_steps)
{
  float t = 0.f;
  for (int it = 0; it < max_steps && t <= max_range; ++it) {
    const Eigen::Vector2f p = o_map + t * dir_unit_map;
    const float d = df.distance(p.x(), p.y());

    // 出 DF 范围：未知，不做 ray 判定（返回 inf）
    if (!std::isfinite(d)) return std::numeric_limits<float>::infinity();

    // 命中（靠近占用/膨胀边界）
    if (d <= hit_eps) return t;

    t += step;
  }

  // 在 DF 内走满 max_range 都没命中：地图预测“这条束应当是空的”
  return max_range;
}

PointCloudT::Ptr DynamicPointFilter::filter(
    const PointCloudT::ConstPtr& scan_base,
    const Eigen::Matrix4f& T_map_base_guess,
    const Eigen::Vector2f& sensor_origin_base,
    DynamicFilterStats* stats) const {
  auto out = PointCloudT::Ptr(new PointCloudT);

  if (!scan_base || scan_base->empty()) {
    if (stats) *stats = DynamicFilterStats{};
    return out;
  }

  const int N = (int)scan_base->size();
  auto count_front_points = [&](const PointCloudT& pc) -> int {
    if (!cfg_.front_protect_enable) return 0;

    const bool use_y = cfg_.front_protect_use_y_axis;
    const double sign = (cfg_.front_protect_forward_sign >= 0.0) ? 1.0 : -1.0;

    int n = 0;
    for (const auto& pt : pc.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
      const double x = static_cast<double>(pt.x);
      const double y = static_cast<double>(pt.y);

      const double fwd_raw = use_y ? y : x;
      const double lat = use_y ? x : y;
      const double fwd = sign * fwd_raw;

      if (fwd < cfg_.front_protect_x_min_m || fwd > cfg_.front_protect_x_max_m) continue;
      if (std::fabs(lat) > cfg_.front_protect_abs_y_max_m) continue;
      ++n;
    }
    return n;
  };

  if (stats) {
    *stats = DynamicFilterStats{};
    stats->in = N;
  }

  // disabled / not ready => pass-through
  if (!cfg_.enable || !df_.isReady()) {
    *out = *scan_base;
    if (stats) {
      stats->kept = N;
      stats->keep_ratio = 1.0;
      stats->front_in = count_front_points(*scan_base);
      stats->front_kept = stats->front_in;
      stats->front_drop_ratio = 0.0;
      stats->front_recovered = false;
    }
    return out;
  }

  struct Item { float d; int idx; };

  auto build_items = [&](bool use_ray,
                         std::vector<Item>* items,
                         std::vector<float>* dists,
                         std::vector<float>* rmap_valid,
                         int* ray_tested,
                         int* ray_rejected) {
    items->clear();
    dists->clear();
    if (rmap_valid) rmap_valid->clear();
    if (ray_tested) *ray_tested = 0;
    if (ray_rejected) *ray_rejected = 0;

    const Eigen::Matrix3f R = T_map_base_guess.block<3,3>(0,0);
    const Eigen::Vector2f o_map_lidar = (T_map_base_guess *
                                         Eigen::Vector4f(sensor_origin_base.x(),
                                                         sensor_origin_base.y(),
                                                         0.f, 1.f)).head<2>();

    const float max_range = (float)((cfg_.ray_max_range_m > 0.0) ? cfg_.ray_max_range_m : 50.0);
const float hit_eps   = (float)((cfg_.ray_hit_eps_m > 0.0) ? cfg_.ray_hit_eps_m : (0.5 * cfg_.grid_res_m));

// 固定步长 step：优先用参数；否则默认 0.5*grid_res
const float step      = (float)((cfg_.ray_step_m > 0.0) ? cfg_.ray_step_m : (0.5 * cfg_.grid_res_m));

const int   max_steps = std::max(16, cfg_.ray_max_steps);
const float margin    = (float)std::max(0.0, cfg_.ray_margin_near_m);


    items->reserve((size_t)N);
    dists->reserve((size_t)N);
    if (rmap_valid) rmap_valid->reserve((size_t)N);

    for (int i = 0; i < N; ++i) {
  const auto& pt = scan_base->points[i];
  if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;

  // --- Distance-to-map compute (用于 keep + 贴墙保护) ---
  const Eigen::Vector4f pb(pt.x, pt.y, 0.f, 1.f);
  const Eigen::Vector4f pm = T_map_base_guess * pb;
  const float d = df_.distance(pm.x(), pm.y());
  if (!std::isfinite(d)) continue;
  if (cfg_.hard_max_dist_m > 0.0 && d > (float)cfg_.hard_max_dist_m) continue;

  // —— 二次确认：贴墙/贴静态结构的点优先保留 ——
  const float confirm_df = (float)std::max(0.0, cfg_.ray_confirm_df_m);
  const bool allow_ray_reject = (d > confirm_df);

  // --- Ray consistency reject (optional) ---
  if (use_ray && cfg_.ray_enable && allow_ray_reject) {
    const Eigen::Vector2f p_base(pt.x, pt.y);
    const Eigen::Vector2f v_base = p_base - sensor_origin_base;
    const float r_obs = v_base.norm();

    if (r_obs > 1e-3f && r_obs <= max_range) {
      Eigen::Vector3f dir_b(v_base.x()/r_obs, v_base.y()/r_obs, 0.f);
      Eigen::Vector3f dir_m3 = R * dir_b;
      Eigen::Vector2f dir_m(dir_m3.x(), dir_m3.y());
      const float n = dir_m.norm();
      if (n > 1e-6f) {
        dir_m /= n;

        if (ray_tested) (*ray_tested)++;

        const float r_map = RaycastExpectedRangeDF_FixedStep(
            df_, o_map_lidar, dir_m,
            max_range, hit_eps, step, max_steps);

        if (rmap_valid && std::isfinite(r_map)) rmap_valid->push_back(r_map);

        // r_map == max_range 也有意义：地图预测这条束是空的
        if (std::isfinite(r_map) && (r_obs < (r_map - margin))) {
          if (ray_rejected) (*ray_rejected)++;
          continue;
        }
      }
    }
  }

  // keep-stage candidates
  items->push_back({d, i});
  dists->push_back(d);
}

  };

  std::vector<Item> items;
  std::vector<float> dists;
  std::vector<float> rmap_v;
  int ray_tested = 0, ray_rejected = 0;

  // pass-1: with ray
  build_items(true, &items, &dists, &rmap_v, &ray_tested, &ray_rejected);

  // 如果 ray 把点砍太多：再做一次不带 ray 的候选（避免完全退化成“不滤”）
  bool insufficient = false;
  if ((int)items.size() < cfg_.min_keep_points) {
    insufficient = true;
    std::vector<Item> items2;
    std::vector<float> dists2;
    build_items(false, &items2, &dists2, nullptr, nullptr, nullptr);

    if ((int)items2.size() >= cfg_.min_keep_points) {
      items.swap(items2);
      dists.swap(dists2);
    } else {
      // 真的信息太少：整帧不滤（避免 NDT 崩）
      *out = *scan_base;
      if (stats) {
        stats->kept = N;
        stats->keep_ratio = 1.0;
        stats->insufficient_static = true;
        stats->ray_tested = ray_tested;
        stats->ray_rejected = ray_rejected;
        stats->front_in = count_front_points(*scan_base);
        stats->front_kept = stats->front_in;
        stats->front_drop_ratio = 0.0;
        stats->front_recovered = false;
      }
      return out;
    }
  }

  // 统计 dist 分位（候选集合）
  double dist_p10 = 0.0, dist_p50 = 0.0, dist_p90 = 0.0;
  {
    std::vector<float> t = dists;
    dist_p10 = percentile_inplace(t, 0.10);
    t = dists;
    dist_p50 = percentile_inplace(t, 0.50);
    t = dists;
    dist_p90 = percentile_inplace(t, 0.90);
  }

  if (stats) {
    stats->dist_p50 = dist_p50;
    stats->dist_p90 = dist_p90;

    stats->ray_tested = ray_tested;
    stats->ray_rejected = ray_rejected;

    if (!rmap_v.empty()) {
      std::vector<float> rr = rmap_v;
      stats->rmap_p50 = percentile_inplace(rr, 0.50);
      rr = rmap_v;
      stats->rmap_p90 = percentile_inplace(rr, 0.90);
    }
  }

  const double dist_spread = dist_p90 - dist_p10;
  if (ray_rejected == 0 && (dist_p90 <= 1e-6 || dist_spread <= 1e-4)) {
    *out = *scan_base;
    if (stats) {
      stats->kept = N;
      stats->keep_ratio = 1.0;
      stats->insufficient_static = true;
      stats->front_in = count_front_points(*scan_base);
      stats->front_kept = stats->front_in;
      stats->front_drop_ratio = 0.0;
      stats->front_recovered = false;
    }
    return out;
  }

  const double keep_ratio = std::min(1.0, std::max(0.05, cfg_.keep_ratio));
  int k = (int)std::ceil(keep_ratio * (double)items.size());
  k = std::max(k, cfg_.min_keep_points);
  k = std::min(k, (int)items.size());

  std::nth_element(items.begin(), items.begin() + (k - 1), items.end(),
                   [](const Item& a, const Item& b) { return a.d < b.d; });

  out->clear();
  out->reserve((size_t)k);
  for (int i = 0; i < k; ++i) {
    out->push_back(scan_base->points[items[i].idx]);
  }

  int front_in = count_front_points(*scan_base);
  int front_kept = count_front_points(*out);
  double front_drop_ratio = 0.0;
  bool front_recovered = false;

  if (cfg_.front_protect_enable) {
    const int min_front = std::max(1, cfg_.front_protect_min_in_points);
    if (front_in >= min_front) {
      front_drop_ratio =
          static_cast<double>(std::max(0, front_in - front_kept)) /
          static_cast<double>(front_in);

      if (front_drop_ratio > cfg_.front_protect_max_drop_ratio) {
        *out = *scan_base;
        front_recovered = true;
        front_kept = front_in;
      }
    }
  }

  if (stats) {
    stats->kept = (int)out->size();
    stats->keep_ratio = (stats->in > 0) ? (double)stats->kept / (double)stats->in : 0.0;
    stats->insufficient_static = insufficient;
    stats->front_in = front_in;
    stats->front_kept = front_kept;
    stats->front_drop_ratio = front_drop_ratio;
    stats->front_recovered = front_recovered;
  }

  return out;
}

}  // namespace localization_ndt
