#pragma once

#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/filters/voxel_grid.h>

#include "localization_ndt/local_relocalizer_2d.hpp"
#include "localization_ndt/map_json_utils.hpp"
#include "localization_ndt/types.hpp"

namespace localization_ndt {

struct MapArtifacts {
  PointCloudT::Ptr cloud;
  JsonMapHeader header;
  bool header_valid = false;
  PointCloudT::Ptr ndt_target_cloud;

  // -------- knownMask (optional) --------
  bool has_known_mask = false;
  int known_w = 0; // grid width in cells
  int known_h = 0; // grid height in cells
  std::vector<uint8_t>
      known_mask_bytes; // bitset bytes, LSB-first, row-major, y-bottom-up

  // -------- freeBand (optional) --------
  bool has_free_band = false;
  int free_band_w = 0; // grid width in cells
  int free_band_h = 0; // grid height in cells
  std::vector<uint8_t>
      free_band_bytes; // bitset bytes, LSB-first, row-major, y-bottom-up
};

namespace {

// base64 decode (whitespace tolerant), returns decoded bytes
static inline bool Base64Decode(const std::string &in,
                                std::vector<uint8_t> *out, std::string *err) {
  if (err)
    err->clear();
  if (!out)
    return false;
  out->clear();

  static bool inited = false;
  static int8_t dec[256];
  if (!inited) {
    for (int i = 0; i < 256; ++i)
      dec[i] = -1;
    const char *A =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (int i = 0; A[i]; ++i)
      dec[(uint8_t)A[i]] = (int8_t)i;
    dec[(uint8_t)'='] = 0;
    inited = true;
  }

  int val = 0;
  int valb = -8;
  for (unsigned char c : in) {
    if (std::isspace(c))
      continue;
    if (c == '=')
      break;
    const int8_t d = dec[c];
    if (d < 0) {
      if (err)
        *err = "Base64Decode: invalid char in base64";
      return false;
    }
    val = (val << 6) + d;
    valb += 6;
    if (valb >= 0) {
      out->push_back(static_cast<uint8_t>((val >> valb) & 0xFF));
      valb -= 8;
    }
  }
  return true;
}

// Read knownMask from json_path (optional). If absent => has_known=false and
// return true.
static inline bool LoadKnownMaskFromJson(const std::string &json_path,
                                         bool *has_known, int *out_w,
                                         int *out_h,
                                         std::vector<uint8_t> *out_bytes,
                                         bool strict_meta, std::string *err) {
  if (err)
    err->clear();
  if (has_known)
    *has_known = false;
  if (out_w)
    *out_w = 0;
  if (out_h)
    *out_h = 0;
  if (out_bytes)
    out_bytes->clear();

  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(json_path, root);
  } catch (const std::exception &e) {
    if (err)
      *err =
          std::string("LoadKnownMaskFromJson: read_json failed: ") + e.what();
    return false;
  }

  // Support a couple possible keys to be tolerant
  auto opt = root.get_child_optional("knownMask");
  if (!opt)
    opt = root.get_child_optional("known_mask");
  if (!opt) {
    // not present => ok
    return true;
  }

  const auto &km = opt.get();

  int w = 0, h = 0;
  std::string b64;

  try {
    w = km.get<int>("width");
    h = km.get<int>("height");
    // common field names: data / b64
    b64 = km.get<std::string>("data", km.get<std::string>("b64", ""));
  } catch (const std::exception &e) {
    if (err)
      *err = std::string("LoadKnownMaskFromJson: parse knownMask failed: ") +
             e.what();
    return false;
  }

  if (w <= 0 || h <= 0 || b64.empty()) {
    if (err)
      *err = "LoadKnownMaskFromJson: knownMask invalid (width/height/data)";
    return false;
  }

  const std::string bitOrder = km.get<std::string>("bitOrder", "lsb0");
  const std::string layout = km.get<std::string>("layout", "row-major");
  const std::string yAxis = km.get<std::string>("yAxis", "bottom-up");
  if (bitOrder != "lsb0" || layout != "row-major" || yAxis != "bottom-up") {
    if (strict_meta) {
      if (err) {
        *err = "LoadKnownMaskFromJson: knownMask meta mismatch "
               "(expected lsb0/row-major/bottom-up)";
      }
      return false;
    }
    std::fprintf(stderr,
                 "[map_processing][WARN] knownMask meta mismatch: bitOrder=%s "
                 "layout=%s yAxis=%s -> ignore knownMask\n",
                 bitOrder.c_str(), layout.c_str(), yAxis.c_str());
    return true;
  }

  std::vector<uint8_t> bytes;
  std::string berr;
  if (!Base64Decode(b64, &bytes, &berr)) {
    if (err)
      *err =
          std::string("LoadKnownMaskFromJson: base64 decode failed: ") + berr;
    return false;
  }

  const size_t ncell = static_cast<size_t>(w) * static_cast<size_t>(h);
  const size_t need = (ncell + 7) / 8;
  if (bytes.size() < need) {
    if (err)
      *err = "LoadKnownMaskFromJson: knownMask bytes too short";
    return false;
  }

  if (has_known)
    *has_known = true;
  if (out_w)
    *out_w = w;
  if (out_h)
    *out_h = h;
  if (out_bytes)
    *out_bytes = std::move(bytes);
  return true;
}

// Read freeBand from json_path (optional). If absent => has_free_band=false and
// return true.
static inline bool LoadFreeBandFromJson(const std::string &json_path,
                                        bool *has_free_band, int *out_w,
                                        int *out_h,
                                        std::vector<uint8_t> *out_bytes,
                                        bool strict_meta, std::string *err) {
  if (err)
    err->clear();
  if (has_free_band)
    *has_free_band = false;
  if (out_w)
    *out_w = 0;
  if (out_h)
    *out_h = 0;
  if (out_bytes)
    out_bytes->clear();

  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(json_path, root);
  } catch (const std::exception &e) {
    if (err)
      *err =
          std::string("LoadFreeBandFromJson: read_json failed: ") + e.what();
    return false;
  }

  auto opt = root.get_child_optional("freeBand");
  if (!opt)
    opt = root.get_child_optional("free_band");
  if (!opt) {
    return true;
  }

  const auto &fb = opt.get();
  int w = 0, h = 0;
  std::string b64;
  try {
    w = fb.get<int>("width");
    h = fb.get<int>("height");
    b64 = fb.get<std::string>("data", fb.get<std::string>("b64", ""));
  } catch (const std::exception &e) {
    if (err)
      *err = std::string("LoadFreeBandFromJson: parse freeBand failed: ") +
             e.what();
    return false;
  }

  if (w <= 0 || h <= 0 || b64.empty()) {
    if (err)
      *err = "LoadFreeBandFromJson: freeBand invalid (width/height/data)";
    return false;
  }

  const std::string bitOrder = fb.get<std::string>("bitOrder", "lsb0");
  const std::string layout = fb.get<std::string>("layout", "row-major");
  const std::string yAxis = fb.get<std::string>("yAxis", "bottom-up");
  if (bitOrder != "lsb0" || layout != "row-major" || yAxis != "bottom-up") {
    if (strict_meta) {
      if (err) {
        *err = "LoadFreeBandFromJson: freeBand meta mismatch "
               "(expected lsb0/row-major/bottom-up)";
      }
      return false;
    }
    std::fprintf(stderr,
                 "[map_processing][WARN] freeBand meta mismatch: bitOrder=%s "
                 "layout=%s yAxis=%s -> ignore freeBand\n",
                 bitOrder.c_str(), layout.c_str(), yAxis.c_str());
    return true;
  }

  std::vector<uint8_t> bytes;
  std::string berr;
  if (!Base64Decode(b64, &bytes, &berr)) {
    if (err) {
      *err =
          std::string("LoadFreeBandFromJson: base64 decode failed: ") + berr;
    }
    return false;
  }

  const size_t ncell = static_cast<size_t>(w) * static_cast<size_t>(h);
  const size_t need = (ncell + 7) / 8;
  if (bytes.size() < need) {
    if (err)
      *err = "LoadFreeBandFromJson: freeBand bytes too short";
    return false;
  }

  if (has_free_band)
    *has_free_band = true;
  if (out_w)
    *out_w = w;
  if (out_h)
    *out_h = h;
  if (out_bytes)
    *out_bytes = std::move(bytes);
  return true;
}

} // namespace

// ----------------- NDT surface cloud builder -----------------

static inline size_t Idx2(int x, int y, int w) {
  return static_cast<size_t>(y) * static_cast<size_t>(w) +
         static_cast<size_t>(x);
}

// world -> grid (supports origin_yaw). grid frame origin at (origin_x,
// origin_y).
static inline bool WorldToGridYaw(double x, double y, double ox, double oy,
                                  double yaw, double res, int w, int h, int *ix,
                                  int *iy) {
  const double dx = x - ox;
  const double dy = y - oy;
  const double c = std::cos(-yaw);
  const double s = std::sin(-yaw);
  const double gx = c * dx - s * dy;
  const double gy = s * dx + c * dy;
  const int ix0 = static_cast<int>(std::floor(gx / res));
  const int iy0 = static_cast<int>(std::floor(gy / res));
  if (ix0 < 0 || iy0 < 0 || ix0 >= w || iy0 >= h)
    return false;
  *ix = ix0;
  *iy = iy0;
  return true;
}

// grid -> world (cell center), supports yaw
static inline void GridToWorldYaw(int ix, int iy, double ox, double oy,
                                  double yaw, double res, double *x,
                                  double *y) {
  const double gx = (static_cast<double>(ix) + 0.5) * res;
  const double gy = (static_cast<double>(iy) + 0.5) * res;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  *x = ox + c * gx - s * gy;
  *y = oy + s * gx + c * gy;
}

// 从 artifacts 里的 occupyPoints(cloud) 还原 occ，
// 提取“边界+加厚”点云供 NDT 使用 thick_steps:
// 0=纯边界；1~2=向占据内部加厚，避免边界太薄（推荐先 1） 注意：不会修改
// art.cloud / occupyPoints 语义，因此不会影响 FCSM 和 map_server 的栅格结果
static inline PointCloudT::Ptr
BuildNdtTargetCloudFromArtifacts(const MapArtifacts &art, int thick_steps,
                                 std::string *warn_msg = nullptr,
                                 bool enable_known_mask_filter = true) {

  if (warn_msg)
    warn_msg->clear();
  if (!art.header_valid || !art.cloud)
    return PointCloudT::Ptr();

  const auto &h = art.header;
  const double res = h.resolution;

  if (!(std::isfinite(res) && res > 0.0))
    return PointCloudT::Ptr();

  int w_from_size = static_cast<int>(std::lround(h.size_w / res));
  int h_from_size = static_cast<int>(std::lround(h.size_h / res));
  if (w_from_size <= 0 || h_from_size <= 0)
    return PointCloudT::Ptr();

  bool free_band_grid_ok = false;
  if (art.has_free_band && art.free_band_w > 0 && art.free_band_h > 0 &&
      !art.free_band_bytes.empty()) {
    const size_t fb_cells = static_cast<size_t>(art.free_band_w) *
                            static_cast<size_t>(art.free_band_h);
    const size_t fb_need = (fb_cells + 7) / 8;
    free_band_grid_ok = art.free_band_bytes.size() >= fb_need;
  }

  int w = w_from_size;
  int hh = h_from_size;
  if (free_band_grid_ok) {
    w = art.free_band_w;
    hh = art.free_band_h;

    const double expect_w = static_cast<double>(w) * res;
    const double expect_h = static_cast<double>(hh) * res;
    const double err_w = std::fabs(h.size_w - expect_w);
    const double err_h = std::fabs(h.size_h - expect_h);
    const double tol = 0.5 * res;
    if ((err_w > tol || err_h > tol) && warn_msg) {
      *warn_msg += (warn_msg->empty() ? "" : " | ");
      *warn_msg +=
          "header.size and freeBand dims mismatch beyond 0.5*res";
    }
  }

  // warn if yaw is non-zero because map_server currently ignores yaw (your
  // buildOccupancy hardcodes yaw=0)
  if (warn_msg && std::fabs(h.origin_yaw) > 1e-6) {
    *warn_msg += (warn_msg->empty() ? "" : " | ");
    *warn_msg +=
        "origin_yaw is non-zero; note map_server OccupancyGrid ignores yaw "
        "(hardcoded 0).";
  }

  const size_t ncell = static_cast<size_t>(w) * static_cast<size_t>(hh);
  if (ncell == 0)
    return PointCloudT::Ptr();

  bool use_free_band_filter = false;
  if (enable_known_mask_filter) {
    if (free_band_grid_ok) {
      use_free_band_filter = true;
    } else if (warn_msg) {
      *warn_msg += (warn_msg->empty() ? "" : " | ");
      *warn_msg += "freeBand dims/bytes mismatch; cannot use freeBand filter";
    }
  }

  if (enable_known_mask_filter && !use_free_band_filter) {
    if (warn_msg) {
      *warn_msg += (warn_msg->empty() ? "" : " | ");
      *warn_msg +=
          "freeBand filter enabled but freeBand is missing/invalid for this map";
    }
    return PointCloudT::Ptr();
  }

  // 1) build occ grid from occupy points
  std::vector<uint8_t> occ(ncell, 0);
  size_t occ_cells = 0;
  for (const auto &pt : *art.cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
      continue;
    int ix = 0, iy = 0;
    if (!WorldToGridYaw(pt.x, pt.y, h.origin_x, h.origin_y, h.origin_yaw, res,
                        w, hh, &ix, &iy))
      continue;
    const size_t id = Idx2(ix, iy, w);
    if (!occ[id]) {
      occ[id] = 1;
      ++occ_cells;
    }
  }

  auto is_free_band_cell = [&](int x, int y) -> bool {
    if (!use_free_band_filter)
      return false;
    if (x < 0 || y < 0 || x >= w || y >= hh)
      return false;
    const size_t id = Idx2(x, y, w);
    return ((art.free_band_bytes[id >> 3] >> (id & 7)) & 0x01u) != 0;
  };

  auto is_face_non_occ = [&](int x, int y) -> bool {
    if (x < 0 || y < 0 || x >= w || y >= hh) {
      // Keep legacy behavior only when freeBand filter is inactive.
      return !use_free_band_filter;
    }
    const size_t id = Idx2(x, y, w);
    if (occ[id])
      return false;
    if (use_free_band_filter)
      return is_free_band_cell(x, y);
    return true;
  };

  // 2) boundary = occ && neighbor is known non-occupied (4-neighborhood)
  std::vector<uint8_t> boundary(ncell, 0);
  auto rebuild_boundary = [&]() -> size_t {
    std::fill(boundary.begin(), boundary.end(), 0);
    size_t boundary_cells = 0;
    for (int y = 0; y < hh; ++y) {
      for (int x = 0; x < w; ++x) {
        const size_t id = Idx2(x, y, w);
        if (!occ[id])
          continue;

        bool b = false;
        const int nx[4] = {x - 1, x + 1, x, x};
        const int ny[4] = {y, y, y - 1, y + 1};
        for (int k = 0; k < 4; ++k) {
          const int xx = nx[k], yy = ny[k];
          if (is_face_non_occ(xx, yy)) {
            b = true;
            break;
          }
        }
        if (b) {
          boundary[id] = 1;
          ++boundary_cells;
        }
      }
    }
    return boundary_cells;
  };

  size_t boundary_cells = rebuild_boundary();
  if (use_free_band_filter && occ_cells > 0) {
    const double boundary_ratio =
        static_cast<double>(boundary_cells) / static_cast<double>(occ_cells);
    constexpr double kMinBoundaryRatio = 0.05;
    if (!std::isfinite(boundary_ratio) || boundary_ratio < kMinBoundaryRatio) {
      if (warn_msg) {
        *warn_msg += (warn_msg->empty() ? "" : " | ");
        *warn_msg += "freeBand filtering too aggressive; reject map (ratio=" +
                     std::to_string(boundary_ratio) + ", occ=" +
                     std::to_string(occ_cells) + ", boundary=" +
                     std::to_string(boundary_cells) + ")";
      }
      return PointCloudT::Ptr();
    }
  }

  // 3) thickness strategy: keep boundary cells fixed and emit inward layers along normal.
  if (thick_steps < 0)
    thick_steps = 0;

// 4) generate edge points on occupied/non-occupied interfaces + oversample + XY thick
PointCloudT::Ptr out(new PointCloudT);

// 参数：你可以先用这组
const int tangent_samples = 1; // 1 => [-1,0,1], 2 => 5 samples
const float leaf_xy = static_cast<float>(std::max(0.01, res * 0.5)); // 1cm
const double tangent_step = static_cast<double>(leaf_xy); // align tangent spacing with voxel leaf
const double normal_step = static_cast<double>(leaf_xy); // align thickness with voxel leaf
const int normal_layers = std::max(0, thick_steps);

// === Z band params (step-1 conservative) ===
// Scan points are built on z=0 plane; keep map target centered at 0 first.
const float z_center = -0.05f;
const float z_half   = 0.55f;
const float z_step   = 0.10f;

// Guard against extreme params.
const float z_half_c = std::max(0.0f, std::min(z_half, 2.0f));
const float z_step_c = std::max(0.02f, std::min(z_step, 0.50f));

out->points.reserve(static_cast<size_t>(w) * static_cast<size_t>(hh) / 3 * 3 * 4);

auto pushZBand = [&](double wx, double wy) {
  PointT p;
  p.x = static_cast<float>(wx);
  p.y = static_cast<float>(wy);
  p.intensity = 1.0f;

  const float z0 = z_center - z_half_c;
  const float z1 = z_center + z_half_c;

  const int layers = std::max(
      1, static_cast<int>(std::floor((z1 - z0) / z_step_c + 0.5f)) + 1);

  for (int i = 0; i < layers; ++i) {
    float z = z0 + static_cast<float>(i) * z_step_c;
    if (z > z1)
      z = z1;
    p.z = z;
    out->points.push_back(p);
  }
};

auto grid_to_world_xy = [&](double gx, double gy, double* wx, double* wy) {
  const double c = std::cos(h.origin_yaw);
  const double s = std::sin(h.origin_yaw);
  *wx = h.origin_x + c * gx - s * gy;
  *wy = h.origin_y + s * gx + c * gy;
};

auto is_occ = [&](int x, int y) -> bool {
  return occ[Idx2(x, y, w)] != 0;
};
auto is_boundary = [&](int x, int y) -> bool {
  return boundary[Idx2(x, y, w)] != 0;
};
// 发射一条“边”：base=(gx,gy) 是边界中心点；t 是切线方向；n 是法线方向（指向 occupied 内部）
auto emit_edge = [&](double gx, double gy, double tx, double ty, double nx, double ny) {
  for (int nl = 0; nl <= normal_layers; ++nl) {
    const double gxn = gx + nx * (nl * normal_step);
    const double gyn = gy + ny * (nl * normal_step);

    for (int k = -tangent_samples; k <= tangent_samples; ++k) {
      const double gxt = gxn + tx * (k * tangent_step);
      const double gyt = gyn + ty * (k * tangent_step);

      double wx, wy;
      grid_to_world_xy(gxt, gyt, &wx, &wy);
      pushZBand(wx, wy);
    }
  }
};

for (int y = 0; y < hh; ++y) {
  for (int x = 0; x < w; ++x) {
    if (!is_occ(x, y) || !is_boundary(x, y)) continue;

    // left: boundary at gx=x*res, gy=(y+0.5)*res
    if (is_face_non_occ(x - 1, y)) {
      const double gx = x * res;
      const double gy = (y + 0.5) * res;
      // 切线沿 +y，法线指向 occupied 内部 => +x
      emit_edge(gx, gy, /*t*/0.0, 1.0, /*n*/1.0, 0.0);
    }
    // right: gx=(x+1)*res, normal => -x
    if (is_face_non_occ(x + 1, y)) {
      const double gx = (x + 1) * res;
      const double gy = (y + 0.5) * res;
      emit_edge(gx, gy, 0.0, 1.0, -1.0, 0.0);
    }
    // down: gy=y*res, tangent => +x, normal => +y
    if (is_face_non_occ(x, y - 1)) {
      const double gx = (x + 0.5) * res;
      const double gy = y * res;
      emit_edge(gx, gy, 1.0, 0.0, 0.0, 1.0);
    }
    // up: gy=(y+1)*res, normal => -y
    if (is_face_non_occ(x, y + 1)) {
      const double gx = (x + 0.5) * res;
      const double gy = (y + 1) * res;
      emit_edge(gx, gy, 1.0, 0.0, 0.0, -1.0);
    }
  }
}

// --- Downsample NDT target to control point count ---
{
  const std::size_t before = out->points.size();

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(out);

  const float leaf_z = 0.1f;
  vg.setLeafSize(leaf_xy, leaf_xy, leaf_z);

  PointCloudT::Ptr ds(new PointCloudT);
  vg.filter(*ds);
  out.swap(ds);

  const std::size_t after = out->points.size();
  if (warn_msg) {
    *warn_msg += (warn_msg->empty() ? "" : " | ");
    *warn_msg += "voxel_ds: " + std::to_string(before) + " -> " +
                 std::to_string(after) + " (leaf_xy=" +
                 std::to_string(leaf_xy) + ", leaf_z=" +
                 std::to_string(leaf_z) + ")";
  }
}

out->width = static_cast<uint32_t>(out->points.size());
out->height = 1;
out->is_dense = true;
return out;

}

inline bool LoadMapArtifactsFromJson(
    const std::string &json_path, MapArtifacts *out, std::string *err,
    bool enable_known_mask_filter = true) {
  if (err)
    err->clear();
  if (!out)
    return false;

  // reset everything
  out->cloud.reset();
  out->header = JsonMapHeader();
  out->header_valid = false;
  out->has_known_mask = false;
  out->known_w = 0;
  out->known_h = 0;
  out->known_mask_bytes.clear();
  out->has_free_band = false;
  out->free_band_w = 0;
  out->free_band_h = 0;
  out->free_band_bytes.clear();
  out->ndt_target_cloud.reset();

  JsonMapHeader h;
  PointCloudT::Ptr cloud;
  std::string load_err;
  if (!LoadPointCloudFromJson(json_path, &cloud, &h, &load_err)) {
    if (err)
      *err = load_err;
    return false;
  }

  // knownMask/freeBand are optional in JSON; NDT strict mode requires freeBand.
  bool has_known = false;
  int km_w = 0, km_h = 0;
  std::vector<uint8_t> km_bytes;
  std::string km_err;
  if (!LoadKnownMaskFromJson(json_path, &has_known, &km_w, &km_h, &km_bytes,
                             /*strict_meta=*/false, &km_err)) {
    if (err)
      *err = km_err;
    return false;
  }

  bool has_free_band = false;
  int fb_w = 0, fb_h = 0;
  std::vector<uint8_t> fb_bytes;
  std::string fb_err;
  if (!LoadFreeBandFromJson(json_path, &has_free_band, &fb_w, &fb_h, &fb_bytes,
                            enable_known_mask_filter, &fb_err)) {
    if (err)
      *err = fb_err;
    return false;
  }

  out->cloud = cloud;
  out->header = h;
  out->header_valid = true;

  if (has_known) {
    out->has_known_mask = true;
    out->known_w = km_w;
    out->known_h = km_h;
    out->known_mask_bytes = std::move(km_bytes);
  }

  if (has_free_band) {
    out->has_free_band = true;
    out->free_band_w = fb_w;
    out->free_band_h = fb_h;
    out->free_band_bytes = std::move(fb_bytes);
  }

  if (enable_known_mask_filter && !has_free_band) {
    if (err) {
      *err = "freeBand filter enabled but freeBand is missing or ignored";
    }
    return false;
  }

  // ---- derive NDT target cloud (surface/boundary) ----
  {
    const auto t0 = std::chrono::steady_clock::now();
    std::string warn;
    out->ndt_target_cloud = BuildNdtTargetCloudFromArtifacts(
        *out, /*thick_steps=*/0, &warn, enable_known_mask_filter);
    const auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    if (!warn.empty()) {
      std::fprintf(stderr, "[map_processing][WARN] %s\n", warn.c_str());
    }

    std::fprintf(stderr,
                 "[map_processing] ndt_target_cloud built: %u points (raw "
                 "occupy cloud: %u) time=%.2fms\n",
                 (unsigned)(out->ndt_target_cloud ? out->ndt_target_cloud->size()
                                                  : 0),
                 (unsigned)out->cloud->size(), ms);

    if ((!out->ndt_target_cloud || out->ndt_target_cloud->empty()) &&
        enable_known_mask_filter) {
      if (err) {
        *err = warn.empty() ? "ndt_target_cloud build failed under mask strict "
                              "mode"
                            : warn;
      }
      return false;
    }

    if (!out->ndt_target_cloud) {
      std::fprintf(stderr,
                   "[map_processing][WARN] ndt_target_cloud build failed.\n");
    }
  }

  return true;
}

inline bool BuildLocalRelocFromArtifacts(const MapArtifacts &art,
                                         LocalRelocalizer2D *lr,
                                         std::string *err) {
  if (err)
    err->clear();
  if (!lr) {
    if (err)
      *err = "local_reloc is null";
    return false;
  }
  if (!art.cloud || !art.header_valid) {
    if (err)
      *err = "map artifacts invalid";
    return false;
  }

  LocalRelocMapMeta meta;
  meta.origin_x = art.header.origin_x;
  meta.origin_y = art.header.origin_y;
  meta.origin_yaw = art.header.origin_yaw;
  meta.resolution = art.header.resolution;
  meta.size_w = art.header.size_w;
  meta.size_h = art.header.size_h;

  std::vector<std::pair<double, double>> occupy_xy;
  occupy_xy.reserve(art.cloud->size());
  for (const auto &pt : *art.cloud) {
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y))
      continue;
    occupy_xy.emplace_back(pt.x, pt.y);
  }

  // ---- knownMask passthrough (safe with length check) ----
  const uint8_t *known_bytes = nullptr;
  size_t known_len = 0;
  int known_w = 0, known_h = 0;

  if (art.has_known_mask && art.known_w > 0 && art.known_h > 0 &&
      !art.known_mask_bytes.empty()) {
    const size_t ncell =
        static_cast<size_t>(art.known_w) * static_cast<size_t>(art.known_h);
    const size_t need = (ncell + 7) / 8;
    if (art.known_mask_bytes.size() >= need) {
      known_bytes = art.known_mask_bytes.data();
      known_len = art.known_mask_bytes.size();
      known_w = art.known_w;
      known_h = art.known_h;
    } else {
      std::fprintf(stderr,
                   "[local_reloc][WARN] knownMask bytes too short: have=%zu "
                   "need=%zu -> ignore knownMask\n",
                   art.known_mask_bytes.size(), need);
    }
  }

  return lr->loadMapFromOccPoints(meta, occupy_xy, known_bytes, known_len,
                                  known_w, known_h, err);
}

} // namespace localization_ndt
