#include "localization_ndt/local_relocalizer_2d.hpp"
#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>

#include <boost/filesystem.hpp>
#include <yaml-cpp/yaml.h>

#include <cctype>
#include <ros/package.h>

#include <cstdio> // fprintf
#include <iostream>
#include <unordered_map>
namespace localization_ndt {

namespace {

static std::string ResolveImagePath(const std::string &yaml_path,
                                    const std::string &image_in,
                                    bool allow_external_map_paths) {
  namespace fs = boost::filesystem;

  // Optional external-map mode: allow standard map_server resolution
  // (absolute image path, or relative path beside yaml).
  if (allow_external_map_paths) {
    try {
      const fs::path image_path(image_in);
      if (image_path.is_absolute() && fs::exists(image_path)) {
        return fs::canonical(image_path).string();
      }
      const fs::path yaml_parent = fs::path(yaml_path).parent_path();
      const fs::path relative_image = yaml_parent / image_path;
      if (fs::exists(relative_image)) {
        return fs::canonical(relative_image).string();
      }
    } catch (...) {
      // Fall through to strict packaged-map mode.
    }
  }

  const fs::path map_root =
      fs::path(ros::package::getPath("localization_ndt")) / "map";
  try {
    const fs::path yaml_canon = fs::canonical(fs::path(yaml_path));
    const fs::path root_canon = fs::canonical(map_root);

    // 判断 yaml_canon 是否在 root_canon 下
    auto it_p = yaml_canon.begin();
    for (auto it_r = root_canon.begin(); it_r != root_canon.end();
         ++it_r, ++it_p) {
      if (it_p == yaml_canon.end() || *it_p != *it_r) {
        return std::string(); // 不在 localization_ndt/map 下，直接拒绝
      }
    }
  } catch (...) {
    return std::string(); // canonical 失败也拒绝（比如文件不存在）
  }

  // 1) 强制只取文件名：绝对路径/相对路径/带目录的一律裁剪
  const fs::path basename = fs::path(image_in).filename();

  // 2) 只在 localization_ndt/map 下找
  const fs::path p1 = map_root / basename;
  if (fs::exists(p1))
    return p1.string();

  // 3) 回退：用 yaml 同名 .pgm（也只在 map_root 下）
  const fs::path p2 = map_root / (fs::path(yaml_path).stem().string() + ".pgm");
  if (fs::exists(p2))
    return p2.string();

  // 失败：返回空，让上层明确报错
  return std::string();
}

// 读取 PGM（支持 P5 binary + P2 ascii）
struct PgmImage {
  int width = 0;
  int height = 0;
  int maxval = 255;
  std::vector<uint8_t> data; // row-major, top->bottom
};

static bool ReadToken(std::istream &is, std::string *out) {
  out->clear();
  while (true) {
    int c = is.peek();
    if (c == EOF)
      return false;
    if (std::isspace(c)) {
      is.get();
      continue;
    }
    if (c == '#') { // comment
      std::string dummy;
      std::getline(is, dummy);
      continue;
    }
    break;
  }
  (*out) = "";
  while (true) {
    int c = is.peek();
    if (c == EOF)
      break;
    if (std::isspace(c) || c == '#')
      break;
    out->push_back(static_cast<char>(is.get()));
  }
  return !out->empty();
}

static bool LoadPgm(const std::string &path, PgmImage *img, std::string *err) {
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs.is_open()) {
    if (err)
      *err = "LoadPgm: cannot open: " + path;
    return false;
  }
  std::string magic;
  if (!ReadToken(ifs, &magic)) {
    if (err)
      *err = "LoadPgm: read magic failed";
    return false;
  }
  if (magic != "P5" && magic != "P2") {
    if (err)
      *err = "LoadPgm: unsupported magic: " + magic;
    return false;
  }
  std::string tok;
  if (!ReadToken(ifs, &tok))
    return false;
  img->width = std::stoi(tok);
  if (!ReadToken(ifs, &tok))
    return false;
  img->height = std::stoi(tok);
  if (!ReadToken(ifs, &tok))
    return false;
  img->maxval = std::stoi(tok);
  if (img->width <= 0 || img->height <= 0 || img->maxval <= 0) {
    if (err)
      *err = "LoadPgm: invalid header";
    return false;
  }

  img->data.assign(img->width * img->height, 0);

  if (magic == "P5") {
    // binary: skip single whitespace after maxval
    ifs.get();
    if (img->maxval <= 255) {
      ifs.read(reinterpret_cast<char *>(img->data.data()),
               static_cast<std::streamsize>(img->data.size()));
      if (!ifs) {
        if (err)
          *err = "LoadPgm: binary read failed";
        return false;
      }
    } else {
      // 16-bit not supported
      if (err)
        *err = "LoadPgm: maxval>255 not supported";
      return false;
    }
  } else {
    // ascii P2
    for (int i = 0; i < img->width * img->height; ++i) {
      if (!ReadToken(ifs, &tok)) {
        if (err)
          *err = "LoadPgm: ascii data short";
        return false;
      }
      int v = std::stoi(tok);
      v = std::max(0, std::min(img->maxval, v));
      img->data[i] = static_cast<uint8_t>(std::lround(v * 255.0 / img->maxval));
    }
  }
  return true;
}

} // namespace

// ======================= GridMap (private) =======================
struct LocalRelocalizer2D::GridMap {
  int w = 0;
  int h = 0;
  double res = 0.05;

  // origin in map frame: (x,y,yaw)
  double ox = 0.0;
  double oy = 0.0;
  double oyaw = 0.0;

  double c0 = 1.0; // cos(-oyaw)
  double s0 = 0.0; // sin(-oyaw)
  double c1 = 1.0; // cos(oyaw)   (后面 grid->world 会用)
  double s1 = 0.0; // sin(oyaw)

  std::vector<uint16_t> score0; // L0 每格得分，0..65535
  double score_sigma = 0.0;
  double score_maxd = 0.0;

  struct ScoreLevel {
    int w = 0, h = 0;
    std::vector<uint16_t> s; // max pooled score
  };
  std::vector<ScoreLevel> pyr;

  // occ: true=occupied
  std::vector<uint8_t> occ;   // size w*h
  std::vector<uint8_t> known; // size w*h
  std::vector<float> dist_m; // distance-to-nearest-occupied in meters, size w*h

  inline int idx(int x, int y) const { return y * w + x; }

  bool worldToGrid(double mx, double my, int *gx, int *gy) const {
    // apply origin yaw: local = R(-oyaw) * ([mx,my]-[ox,oy])
    const double dx = mx - ox;
    const double dy = my - oy;
    const double lx = c0 * dx - s0 * dy;
    const double ly = s0 * dx + c0 * dy;

    const int ix = static_cast<int>(std::floor(lx / res));
    const int iy = static_cast<int>(std::floor(ly / res));
    if (ix < 0 || ix >= w || iy < 0 || iy >= h)
      return false;
    if (gx)
      *gx = ix;
    if (gy)
      *gy = iy;
    return true;
  }

  static ScoreLevel downsampleMax(const ScoreLevel &src) {
    ScoreLevel dst;
    dst.w = (src.w + 1) / 2;
    dst.h = (src.h + 1) / 2;
    dst.s.assign(dst.w * dst.h, 0);

    auto at = [&](int x, int y) -> uint16_t {
      if (x < 0 || x >= src.w || y < 0 || y >= src.h)
        return 0;
      return src.s[y * src.w + x];
    };

    for (int y = 0; y < dst.h; ++y) {
      for (int x = 0; x < dst.w; ++x) {
        const int sx = x * 2;
        const int sy = y * 2;
        uint16_t m = 0;
        m = std::max(m, at(sx, sy));
        m = std::max(m, at(sx + 1, sy));
        m = std::max(m, at(sx, sy + 1));
        m = std::max(m, at(sx + 1, sy + 1));
        dst.s[y * dst.w + x] = m;
      }
    }
    return dst;
  }

  void buildPyramid(int Lmax) {
    pyr.clear();
    pyr.reserve(std::max(1, Lmax + 1));

    ScoreLevel L0;
    L0.w = w;
    L0.h = h;
    L0.s = score0; // L0 用 score0
    pyr.push_back(std::move(L0));

    for (int L = 1; L <= Lmax; ++L) {
      pyr.push_back(downsampleMax(pyr.back()));
    }
  }

  void buildScore0(double sigma, double maxd) {
    score_sigma = sigma;
    score_maxd = maxd;
    score0.assign(w * h, 0);

    const double s = std::max(1e-3, sigma);
    const double inv2sig2 = 1.0 / (2.0 * s * s);
    const double md = std::max(1e-3, maxd);

    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int id = idx(x, y);
        if (!known.empty() && known[id] == 0) {
          score0[id] = 0;
          continue;
        } // unknown -> 0

        const float df = dist_m[id];
        const double d = static_cast<double>(df);
        if (!(d >= 0.0) || d > md) {
          score0[id] = 0;
          continue;
        } // 太远 -> 0

        const double wgt = std::exp(-(d * d) * inv2sig2); // 0..1
        score0[id] = static_cast<uint16_t>(std::lround(wgt * 65535.0));
      }
    }
  }

  float distAtWorld(double mx, double my, bool *in_map) const {
    int gx = 0, gy = 0;
    if (!worldToGrid(mx, my, &gx, &gy)) {
      if (in_map)
        *in_map = false;
      return std::numeric_limits<float>::infinity();
    }

    const int id = idx(gx, gy);

    // unknown: treat as not usable
    if (!known.empty() && known[id] == 0) {
      if (in_map)
        *in_map = false;
      return std::numeric_limits<float>::infinity();
    }

    if (in_map)
      *in_map = true;
    return dist_m[id];
  }

  void buildDistanceField() {
    // chamfer 3x3 distance transform (approx euclidean)
    const float INF = 1e9f;
    dist_m.assign(w * h, INF);

    // init: occupied -> 0
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        if (occ[idx(x, y)])
          dist_m[idx(x, y)] = 0.0f;
      }
    }

    const float w1 = 1.0f;
    const float w2 = 1.41421356f; // sqrt(2)

    // forward
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        float d = dist_m[idx(x, y)];
        if (x > 0)
          d = std::min(d, dist_m[idx(x - 1, y)] + w1);
        if (y > 0)
          d = std::min(d, dist_m[idx(x, y - 1)] + w1);
        if (x > 0 && y > 0)
          d = std::min(d, dist_m[idx(x - 1, y - 1)] + w2);
        if (x + 1 < w && y > 0)
          d = std::min(d, dist_m[idx(x + 1, y - 1)] + w2);
        dist_m[idx(x, y)] = d;
      }
    }
    // backward
    for (int y = h - 1; y >= 0; --y) {
      for (int x = w - 1; x >= 0; --x) {
        float d = dist_m[idx(x, y)];
        if (x + 1 < w)
          d = std::min(d, dist_m[idx(x + 1, y)] + w1);
        if (y + 1 < h)
          d = std::min(d, dist_m[idx(x, y + 1)] + w1);
        if (x + 1 < w && y + 1 < h)
          d = std::min(d, dist_m[idx(x + 1, y + 1)] + w2);
        if (x > 0 && y + 1 < h)
          d = std::min(d, dist_m[idx(x - 1, y + 1)] + w2);
        dist_m[idx(x, y)] = d;
      }
    }

    // to meters
    for (auto &v : dist_m)
      v *= static_cast<float>(res);

    // optional but recommended: unknown -> inf (defensive)
    if (!known.empty() && known.size() == dist_m.size()) {
      for (size_t i = 0; i < dist_m.size(); ++i) {
        if (known[i] == 0)
          dist_m[i] = std::numeric_limits<float>::infinity();
      }
    }
  }
};

// ======================= LocalRelocalizer2D impl =======================

double LocalRelocalizer2D::NormalizeAngle_(double a) {
  while (a > kPiReloc)
    a -= 2.0 * kPiReloc;
  while (a < -kPiReloc)
    a += 2.0 * kPiReloc;
  return a;
}

Eigen::Matrix4f LocalRelocalizer2D::PoseToMat_(const RelocPose2D &p) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  const float c = static_cast<float>(std::cos(p.yaw));
  const float s = static_cast<float>(std::sin(p.yaw));
  T(0, 0) = c;
  T(0, 1) = -s;
  T(1, 0) = s;
  T(1, 1) = c;
  T(0, 3) = static_cast<float>(p.x);
  T(1, 3) = static_cast<float>(p.y);
  return T;
}

std::vector<Eigen::Vector2f>
LocalRelocalizer2D::PreprocessScan_(const std::vector<Eigen::Vector2f> &in,
                                    const LocalRelocOptions &opt) {
  std::vector<Eigen::Vector2f> out;
  out.reserve(std::min<int>(opt.max_scan_points, static_cast<int>(in.size())));

  const double min_r2 = opt.min_range > 0 ? opt.min_range * opt.min_range : 0.0;
  const double max_r2 = opt.max_range > 0
                            ? opt.max_range * opt.max_range
                            : std::numeric_limits<double>::infinity();

  // 2D voxel downsample（用 step 做 leaf，不至于太稀）
  const double voxel = (opt.scan_voxel_m > 0.0) ? opt.scan_voxel_m : 0.02;
  const float leaf =
      static_cast<float>(std::max(0.005, voxel)); // 下限给 5mm 防止太小

  auto pack = [](int ix, int iy) -> int64_t {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
  };

  std::unordered_set<int64_t> used;
  used.reserve(out.capacity() * 2);

  for (const auto &p : in) {
    if (!std::isfinite(p.x()) || !std::isfinite(p.y()))
      continue;
    const double r2 = double(p.x()) * p.x() + double(p.y()) * p.y();
    if (r2 < min_r2 || r2 > max_r2)
      continue;

    const int ix = static_cast<int>(std::floor(p.x() / leaf));
    const int iy = static_cast<int>(std::floor(p.y() / leaf));
    const int64_t key = pack(ix, iy);
    if (!used.insert(key).second)
      continue;

    out.push_back(p);
    if (static_cast<int>(out.size()) >= opt.max_scan_points)
      break;
  }
  return out;
}

bool LocalRelocalizer2D::loadMap(const std::string &map_yaml,
                                 std::string *err) {
  ready_ = false;
  map_.reset();
  map_yaml_path_ = map_yaml;
  map_frame_ = "map";

  YAML::Node yaml;
  try {
    yaml = YAML::LoadFile(map_yaml);
  } catch (const std::exception &e) {
    if (err)
      *err = std::string("loadMap: YAML load failed: ") + e.what();
    return false;
  }

  const auto getS = [&](const char *key, std::string *out) -> bool {
    if (!yaml[key])
      return false;
    *out = yaml[key].as<std::string>();
    return true;
  };
  const auto getD = [&](const char *key, double *out) -> bool {
    if (!yaml[key])
      return false;
    *out = yaml[key].as<double>();
    return true;
  };

  std::string image;
  if (!getS("image", &image)) {
    if (err)
      *err = "loadMap: missing 'image' in yaml";
    return false;
  }

  double resolution = 0.0;
  if (!getD("resolution", &resolution) || resolution <= 0) {
    if (err)
      *err = "loadMap: invalid 'resolution'";
    return false;
  }

  std::vector<double> origin;
  if (!yaml["origin"] || !yaml["origin"].IsSequence() ||
      yaml["origin"].size() < 3) {
    if (err)
      *err = "loadMap: missing/invalid 'origin'";
    return false;
  }
  origin.resize(3);
  origin[0] = yaml["origin"][0].as<double>();
  origin[1] = yaml["origin"][1].as<double>();
  origin[2] = yaml["origin"][2].as<double>();

  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  if (yaml["negate"])
    negate = yaml["negate"].as<int>();
  if (yaml["occupied_thresh"])
    occupied_thresh = yaml["occupied_thresh"].as<double>();
  if (yaml["free_thresh"])
    free_thresh = yaml["free_thresh"].as<double>();

  if (yaml["frame_id"]) {
    try {
      map_frame_ = yaml["frame_id"].as<std::string>();
    } catch (...) {
    }
    if (map_frame_.empty())
      map_frame_ = "map";
  }

  const std::string image_path =
      ResolveImagePath(map_yaml, image, allow_external_map_paths_);
  if (image_path.empty()) {
    if (err)
      *err = "loadMap: ResolveImagePath failed (yaml='" + map_yaml +
             "', image_in='" + image + "')";
    return false;
  }

  PgmImage pgm;
  std::string pgm_err;
  if (!LoadPgm(image_path, &pgm, &pgm_err)) {
    if (err)
      *err = "loadMap: " + pgm_err + " (image_in='" + image +
             "', image_resolved='" + image_path + "')";
    return false;
  }

  auto gm = std::make_shared<GridMap>();
  gm->w = pgm.width;
  gm->h = pgm.height;
  gm->res = resolution;
  gm->ox = origin[0];
  gm->oy = origin[1];
  gm->oyaw = origin[2];

  gm->c0 = std::cos(-gm->oyaw);
  gm->s0 = std::sin(-gm->oyaw);
  gm->c1 = std::cos(gm->oyaw);
  gm->s1 = std::sin(gm->oyaw);

  gm->occ.assign(gm->w * gm->h, 0);
  gm->known.assign(gm->w * gm->h, 0);

  for (int row = 0; row < gm->h; ++row) {
    const int y = (gm->h - 1 - row); // flip
    for (int x = 0; x < gm->w; ++x) {
      const uint8_t v = pgm.data[row * gm->w + x]; // 0..255
      const double occ_prob =
          (negate ? (double(v) / 255.0) : ((255.0 - double(v)) / 255.0));
      const int id = gm->idx(x, y);

      if (occ_prob > occupied_thresh) {
        gm->known[id] = 1;
        gm->occ[id] = 1;
      } else if (occ_prob < free_thresh) {
        gm->known[id] = 1;
        gm->occ[id] = 0;
      } else {
        gm->known[id] = 0;
        gm->occ[id] = 0; // unknown 占位
      }
    }
  }

  gm->buildDistanceField();

  gm->buildScore0(score_sigma_, score_maxd_);

  //  由 setPyrMaxLevel() 控制地图金字塔预建最大层数
  const int pyr_Lmax = std::max(0, pyr_max_level_);
  gm->buildPyramid(pyr_Lmax);

  map_ = gm;
  ready_ = true;

  // P0：关键确认输出
std::fprintf(stderr,
             "[local_reloc] loadMap OK: yaml=%s image=%s w=%d h=%d res=%.4f "
             "origin=(%.3f,%.3f,%.3fdeg) "
             "score0(req_sigma=%.3f,req_maxd=%.3f used_sigma=%.3f,used_maxd=%.3f) "
             "pyr_req_Lmax=%d pyr_built_Lmax=%d\n",
             map_yaml.c_str(), image_path.c_str(), gm->w, gm->h, gm->res,
             gm->ox, gm->oy, gm->oyaw * 180.0 / kPiReloc,
             score_sigma_, score_maxd_,
             gm->score_sigma, gm->score_maxd,
             pyr_max_level_, (int)gm->pyr.size() - 1);


  return true;
}


bool LocalRelocalizer2D::loadMapFromOccPoints(
    const LocalRelocMapMeta &meta,
    const std::vector<std::pair<double, double>> &occupy_xy,
    const uint8_t *known_mask_bytes, size_t known_mask_len, int known_w,
    int known_h, std::string *err) {

  if (err)
    err->clear();
  ready_ = false;
  map_.reset();
  map_frame_ = "map";

  if (!(std::isfinite(meta.origin_x) && std::isfinite(meta.origin_y) &&
        std::isfinite(meta.origin_yaw) && std::isfinite(meta.resolution) &&
        meta.resolution > 0.0 && std::isfinite(meta.size_w) &&
        std::isfinite(meta.size_h) && meta.size_w > 0.0 && meta.size_h > 0.0)) {
    if (err)
      *err = "loadMapFromOccPoints: meta invalid";
    return false;
  }

  const int w_from_size =
      static_cast<int>(std::lround(meta.size_w / meta.resolution));
  const int h_from_size =
      static_cast<int>(std::lround(meta.size_h / meta.resolution));
  if (w_from_size <= 0 || h_from_size <= 0) {
    if (err)
      *err = "loadMapFromOccPoints: invalid grid size from meta.size/res";
    return false;
  }

  const bool has_known =
      (known_mask_bytes != nullptr && known_w > 0 && known_h > 0);
  const int w = has_known ? known_w : w_from_size;
  const int h = has_known ? known_h : h_from_size;
  if (w <= 0 || h <= 0) {
    if (err)
      *err = "loadMapFromOccPoints: invalid grid size";
    return false;
  }

  if (has_known) {
    const int dw = std::abs(w - w_from_size);
    const int dh = std::abs(h - h_from_size);
    if (dw > 1 || dh > 1) {
      if (err)
        *err =
            "loadMapFromOccPoints: knownMask dims mismatch with meta.size/res";
      return false;
    }
  }

  auto gm = std::make_shared<GridMap>();
  gm->w = w;
  gm->h = h;
  gm->res = meta.resolution;
  gm->ox = meta.origin_x;
  gm->oy = meta.origin_y;
  gm->oyaw = meta.origin_yaw;

  gm->c0 = std::cos(-gm->oyaw);
  gm->s0 = std::sin(-gm->oyaw);
  gm->c1 = std::cos(gm->oyaw);
  gm->s1 = std::sin(gm->oyaw);

  const size_t ncell = static_cast<size_t>(gm->w) * static_cast<size_t>(gm->h);
  gm->occ.assign(ncell, 0);
  gm->known.assign(ncell, 1);

  size_t known_cnt = ncell;
  size_t unknown_cnt = 0;
  if (has_known) {
    const size_t nbytes_need = (ncell + 7) / 8;
    if (known_mask_len < nbytes_need) {
      if (err)
        *err = "loadMapFromOccPoints: knownMask bytes too short";
      return false;
    }

    known_cnt = 0;
    for (size_t i = 0; i < ncell; ++i) {
      const uint8_t b = known_mask_bytes[i >> 3];
      const uint8_t bit = static_cast<uint8_t>((b >> (i & 7)) & 1);
      gm->known[i] = bit ? 1 : 0;
      if (bit)
        ++known_cnt;
    }
    unknown_cnt = ncell - known_cnt;
  }

  size_t occ_cnt = 0;
  for (const auto &p : occupy_xy) {
    int ix = 0;
    int iy = 0;
    if (!gm->worldToGrid(p.first, p.second, &ix, &iy))
      continue;
    const int id = gm->idx(ix, iy);
    if (gm->occ[id] == 0) {
      gm->occ[id] = 1;
      ++occ_cnt;
    }
    gm->known[id] = 1;
  }

  gm->buildDistanceField();
  gm->buildScore0(score_sigma_, score_maxd_);
  gm->buildPyramid(std::max(0, pyr_max_level_));

  map_ = gm;
  ready_ = true;

  const double unk_ratio =
      (ncell > 0) ? (double)unknown_cnt / (double)ncell : 0.0;
  std::fprintf(stderr,
               "[local_reloc] loadMapFromOccPoints OK: w=%d h=%d res=%.4f "
               "origin=(%.3f,%.3f,%.3fdeg) occ_cells=%zu "
               "knownMask=%d known=%zu unknown=%zu unk_ratio=%.3f "
               "score0(sigma=%.3f maxd=%.3f) pyr_Lmax=%d\n",
               gm->w, gm->h, gm->res, gm->ox, gm->oy,
               gm->oyaw * 180.0 / kPiReloc, occ_cnt, has_known ? 1 : 0,
               known_cnt, unknown_cnt, unk_ratio, gm->score_sigma,
               gm->score_maxd, (int)gm->pyr.size() - 1);
  return true;
}


LocalRelocalizer2D::GridScan LocalRelocalizer2D::BuildScanGrid0_(
    const std::vector<Eigen::Vector2f> &scan_xy_base, double yaw_map,
    const GridMap &m, double leaf_m) {

  // leaf_m: 这里用于 scan 去重（ = map.res 或者 opt.scan_voxel_m 的近似）
  const double leaf = std::max(0.01, leaf_m);
  const double yaw_grid = yaw_map - m.oyaw; // 关键：转到 grid-local
  const double c = std::cos(yaw_grid);
  const double s = std::sin(yaw_grid);

  GridScan out;
  out.reserve(scan_xy_base.size());

  auto pack = [](int ix, int iy) -> int64_t {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
  };
  std::unordered_set<int64_t> used;
  used.reserve(scan_xy_base.size() * 2);

  for (const auto &p : scan_xy_base) {
    const double x = p.x();
    const double y = p.y();

    // 先做一个粗的 voxel 去重（米）
    const int vx = static_cast<int>(std::floor(x / leaf));
    const int vy = static_cast<int>(std::floor(y / leaf));
    const int64_t vkey = pack(vx, vy);
    if (!used.insert(vkey).second)
      continue;

    // 旋到 grid-local（米）
    const double lx = c * x - s * y;
    const double ly = s * x + c * y;

    // 转 L0 cell offset（整数）
    const int ix = static_cast<int>(std::lround(lx / m.res));
    const int iy = static_cast<int>(std::lround(ly / m.res));

    GridPt gp;
    gp.x = static_cast<int16_t>(std::max(-32768, std::min(32767, ix)));
    gp.y = static_cast<int16_t>(std::max(-32768, std::min(32767, iy)));
    out.push_back(gp);
  }
  return out;
}
std::vector<LocalRelocalizer2D::GridScan>
LocalRelocalizer2D::BuildScanByLevel_(const GridScan &scan0, int Lmax) {
  std::vector<GridScan> v;
  v.resize(Lmax + 1);
  v[0] = scan0;

  auto pack = [](int ix, int iy) -> int64_t {
    return (static_cast<int64_t>(ix) << 32) ^ static_cast<uint32_t>(iy);
  };

  for (int L = 1; L <= Lmax; ++L) {
    std::unordered_set<int64_t> used;
    used.reserve(v[L - 1].size());

    v[L].reserve(v[L - 1].size());
    for (const auto &p : scan0) {
      const int ix = (int)p.x >> L;
      const int iy = (int)p.y >> L;
      const int64_t key = pack(ix, iy);
      if (!used.insert(key).second)
        continue;
      GridPt q;
      q.x = (int16_t)ix;
      q.y = (int16_t)iy;
      v[L].push_back(q);
    }
  }
  return v;
}
uint64_t LocalRelocalizer2D::SumScoreAtL_(int L, int txL, int tyL,
                                          const GridScan &scanL) const {
  const auto &lvl = map_->pyr[L];
  uint64_t sum = 0;
  for (const auto &p : scanL) {
    const int gx = txL + (int)p.x;
    const int gy = tyL + (int)p.y;
    if (gx < 0 || gx >= lvl.w || gy < 0 || gy >= lvl.h)
      continue;
    sum += lvl.s[gy * lvl.w + gx];
  }
  return sum;
}
uint64_t LocalRelocalizer2D::SumScoreAtL0_(int tx0, int ty0,
                                           const GridScan &scan0,
                                           int *out_known_cnt) const {
  int known_cnt = 0;
  uint64_t sum = 0;
  for (const auto &p : scan0) {
    const int gx = tx0 + (int)p.x;
    const int gy = ty0 + (int)p.y;
    if (gx < 0 || gx >= map_->w || gy < 0 || gy >= map_->h)
      continue;
    const int id = map_->idx(gx, gy);
    if (!map_->known.empty() && map_->known[id] == 0)
      continue; // unknown 不算known
    ++known_cnt;
    sum += map_->score0[id];
  }
  if (out_known_cnt)
    *out_known_cnt = known_cnt;
  return sum;
}
LocalRelocalizer2D::BnBResult LocalRelocalizer2D::SearchXYBnB_(
    const std::vector<GridScan> &scanL, int center_tx0, int center_ty0,
    double xy_range_m, int Lmax, int max_nodes) const {
  BnBResult r;

  struct Node {
    int L;
    int tx;
    int ty;
    uint64_t bound;
  };
  auto cmp = [](const Node &a, const Node &b) { return a.bound < b.bound; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> pq(cmp);

  // root window at level Lmax
  const int center_txL = center_tx0 >> Lmax;
  const int center_tyL = center_ty0 >> Lmax;
  const int rangeL = (int)std::ceil(xy_range_m / (map_->res * (1 << Lmax)));

  for (int tx = center_txL - rangeL; tx <= center_txL + rangeL; ++tx) {
    for (int ty = center_tyL - rangeL; ty <= center_tyL + rangeL; ++ty) {
      Node n;
      n.L = Lmax;
      n.tx = tx;
      n.ty = ty;
      n.bound = SumScoreAtL_(Lmax, tx, ty, scanL[Lmax]);
      pq.push(n);
    }
  }

  uint64_t best = 0;
  uint64_t second = 0;
  int best_tx0 = center_tx0, best_ty0 = center_ty0;

  int visited = 0;
  while (!pq.empty() && visited < max_nodes) {
    Node n = pq.top();
    pq.pop();
    ++visited;

    if (n.bound <= best)
      continue; // 关键剪枝（可再加 margin）

    if (n.L == 0) {
      int known_cnt = 0;
      const uint64_t sum0 = SumScoreAtL0_(n.tx, n.ty, scanL[0], &known_cnt);
      if (sum0 > best) {
        second = best;
        best = sum0;
        best_tx0 = n.tx;
        best_ty0 = n.ty;
      } else if (sum0 > second) {
        second = sum0;
      }
      continue;
    }

    // expand children
    const int Lc = n.L - 1;
    for (int dx = 0; dx <= 1; ++dx) {
      for (int dy = 0; dy <= 1; ++dy) {
        Node c;
        c.L = Lc;
        c.tx = n.tx * 2 + dx;
        c.ty = n.ty * 2 + dy;
        c.bound = SumScoreAtL_(Lc, c.tx, c.ty, scanL[Lc]);
        if (c.bound > best)
          pq.push(c);
      }
    }
  }

  r.ok = (best > 0);
  r.tx0 = best_tx0;
  r.ty0 = best_ty0;
  r.best_sum = best;
  r.second_sum = second;
  return r;
}
RelocPose2D LocalRelocalizer2D::GridToPoseMap_(int tx0, int ty0,
                                               double yaw_map) const {
  // L0 cell -> grid-local meters（用 cell center）
  const double lx = (double(tx0) + 0.5) * map_->res;
  const double ly = (double(ty0) + 0.5) * map_->res;

  // grid-local -> map (R(oyaw)*[lx,ly] + origin)
  const double mx = map_->ox + map_->c1 * lx - map_->s1 * ly;
  const double my = map_->oy + map_->s1 * lx + map_->c1 * ly;

  RelocPose2D p;
  p.x = mx;
  p.y = my;
  p.yaw = NormalizeAngle_(yaw_map);
  return p;
}

std::pair<double, double>
LocalRelocalizer2D::ScorePose_(const std::vector<Eigen::Vector2f> &scan,
                               const RelocPose2D &pose,
                               const LocalRelocOptions &opt) const {

  if (!ready_ || !map_ || scan.empty())
    return {0.0, 0.0};

  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  const double denom = std::max<size_t>(1, scan.size());

  int known_cnt = 0;
  uint64_t sum_u = 0; // 0..65535*scan.size()

  for (const auto &pb : scan) {
    const double mx = pose.x + c * pb.x() - s * pb.y();
    const double my = pose.y + s * pb.x() + c * pb.y();

    int gx = 0, gy = 0;
    if (!map_->worldToGrid(mx, my, &gx, &gy))
      continue;

    const int id = map_->idx(gx, gy);
    if (!map_->known.empty() && map_->known[id] == 0)
      continue; // unknown

    ++known_cnt;
    sum_u += map_->score0[id]; // 查表
  }

  const double score = (double(sum_u) / 65535.0) / double(denom);
  const double vf = double(known_cnt) / double(denom);
  return {score, vf};
}

RelocPose2D LocalRelocalizer2D::RefinePoseSubcell_(
    const std::vector<Eigen::Vector2f> &scan, const RelocPose2D &seed,
    const LocalRelocOptions &opt, double *out_score, double *out_vf) const {
  if (out_score) *out_score = 0.0;
  if (out_vf) *out_vf = 0.0;
  if (!ready_ || !map_ || scan.empty()) return seed;

  RelocPose2D best = seed;
  auto sv = ScorePose_(scan, best, opt);
  double best_score = sv.first;
  double best_vf = sv.second;

  double step_xy = std::max(0.5 * map_->res, 0.25 * map_->res);
  double step_yaw = std::max(0.5 * std::abs(opt.yaw_step_rad),
                             0.1 * kPiReloc / 180.0);

  for (int level = 0; level < 3; ++level) {
    bool improved = true;
    while (improved) {
      improved = false;
      RelocPose2D round_best = best;
      double round_score = best_score;
      double round_vf = best_vf;
      for (int ix = -1; ix <= 1; ++ix) {
        for (int iy = -1; iy <= 1; ++iy) {
          for (int ia = -1; ia <= 1; ++ia) {
            if (ix == 0 && iy == 0 && ia == 0) continue;
            RelocPose2D cand = best;
            cand.x += ix * step_xy;
            cand.y += iy * step_xy;
            cand.yaw = NormalizeAngle_(cand.yaw + ia * step_yaw);
            auto sc = ScorePose_(scan, cand, opt);
            if (sc.second < opt.min_valid_fraction * 0.8) continue;
            const bool better_score = sc.first > round_score + 1e-9;
            const bool better_vf = (std::fabs(sc.first - round_score) <= 1e-9) &&
                                   (sc.second > round_vf + 1e-9);
            if (better_score || better_vf) {
              round_best = cand;
              round_score = sc.first;
              round_vf = sc.second;
              improved = true;
            }
          }
        }
      }
      best = round_best;
      best_score = round_score;
      best_vf = round_vf;
    }
    step_xy *= 0.5;
    step_yaw *= 0.5;
  }

  if (out_score) *out_score = best_score;
  if (out_vf) *out_vf = best_vf;
  return best;
}

LocalRelocResult
LocalRelocalizer2D::match(const std::vector<Eigen::Vector2f> &scan_xy_base,
                          const RelocPose2D &center,
                          const LocalRelocOptions &opt) const {
  LocalRelocResult res;
  res.ok = false;

  if (!ready_ || !map_ || map_->w <= 0 || map_->h <= 0 ||
      map_->score0.empty() || map_->pyr.empty()) {
    return res;
  }

  // 1) preprocess scan (range + voxel + limit)
  const auto scan = PreprocessScan_(scan_xy_base, opt);
  if (scan.empty()) return res;

  // 2) center pose -> grid cell (L0) even if outside map (允许负数/越界作为搜索中心)
  const double dx = center.x - map_->ox;
  const double dy = center.y - map_->oy;
  const double lx = map_->c0 * dx - map_->s0 * dy;  // grid-local meters
  const double ly = map_->s0 * dx + map_->c0 * dy;

  const int center_tx0 = static_cast<int>(std::floor(lx / map_->res));
  const int center_ty0 = static_cast<int>(std::floor(ly / map_->res));

  // 3) choose Lmax limited by built pyramid
    const int built_Lmax = static_cast<int>(map_->pyr.size()) - 1;
  const int req_Lmax = std::max(0, opt.bnb_max_level);
  const int Lmax_map = std::min(req_Lmax, built_Lmax);

  static int s_print = 0;
  if (req_Lmax != Lmax_map || s_print < 3) {
    ++s_print;
    std::fprintf(stderr,
                 "[local_reloc] match: bnb_req_Lmax=%d built_Lmax=%d use_Lmax=%d\n",
                 req_Lmax, built_Lmax, Lmax_map);
  }


  // 4) yaw candidate steps
  const double kDeg = kPiReloc / 180.0;
  const double yaw_step_fine = std::max(0.25 * kDeg, std::abs(opt.yaw_step_rad));

  // coarse step: clamp into [5deg, 15deg], and tie to fine step
  double yaw_step_coarse = std::max(5.0 * kDeg, 5.0 * yaw_step_fine);
  yaw_step_coarse = std::min(yaw_step_coarse, 15.0 * kDeg);

  // full-circle if half-window >= pi
  const bool full_circle = (opt.yaw_range_rad >= kPiReloc);

  auto yaw_key = [&](double yaw) -> int {
    // quantize in 1e-4 rad to dedupe across wrap
    const double a = NormalizeAngle_(yaw);
    return static_cast<int>(std::lround(a * 10000.0));
  };

  // Cache scan pyramids per yaw to avoid rebuilding repeatedly.
  struct ScanCacheEntry {
    GridScan scan0;
    std::vector<GridScan> scanL;  // size >= Lmax_map+1
  };
  std::unordered_map<int, ScanCacheEntry> scan_cache;
  scan_cache.reserve(256);

  auto get_scan_cache = [&](double yaw_map) -> ScanCacheEntry* {
    const int key = yaw_key(yaw_map);
    auto it = scan_cache.find(key);
    if (it != scan_cache.end()) return &it->second;

    ScanCacheEntry e;
    // NOTE: leaf_m 用 scan_voxel（避免重复强下采样），下限 1cm
    const double leaf_m = std::max(0.01, opt.scan_voxel_m > 0.0 ? opt.scan_voxel_m : 0.02);
    e.scan0 = BuildScanGrid0_(scan, NormalizeAngle_(yaw_map), *map_, leaf_m);
    if (e.scan0.empty()) {
      // 也缓存空，避免反复构建
      auto ret = scan_cache.emplace(key, std::move(e));
      return &ret.first->second;
    }
    e.scanL = BuildScanByLevel_(e.scan0, Lmax_map);
    auto ret = scan_cache.emplace(key, std::move(e));
    return &ret.first->second;
  };

  // 5) build coarse yaw list (unique)
  std::vector<double> yaw_coarse;
  yaw_coarse.reserve(128);
  {
    std::unordered_set<int> used;
    used.reserve(256);

    if (full_circle) {
      // cover exactly one 2pi, start from (center - pi)
      const double start = NormalizeAngle_(center.yaw - kPiReloc);
      const int N = std::max(1, static_cast<int>(std::ceil((2.0 * kPiReloc) / yaw_step_coarse)));
      for (int i = 0; i < N; ++i) {
        const double yaw = NormalizeAngle_(start + i * yaw_step_coarse);
        const int k = yaw_key(yaw);
        if (used.insert(k).second) yaw_coarse.push_back(yaw);
      }
    } else {
      const double a0 = center.yaw - opt.yaw_range_rad;
      const double a1 = center.yaw + opt.yaw_range_rad;
      const int N = std::max(1, static_cast<int>(std::floor((a1 - a0) / yaw_step_coarse)) + 1);
      for (int i = 0; i < N; ++i) {
        const double yaw = NormalizeAngle_(a0 + i * yaw_step_coarse);
        const int k = yaw_key(yaw);
        if (used.insert(k).second) yaw_coarse.push_back(yaw);
      }
      // ensure end included
      const int k_end = yaw_key(NormalizeAngle_(a1));
      if (used.insert(k_end).second) yaw_coarse.push_back(NormalizeAngle_(a1));
    }
  }

  if (yaw_coarse.empty()) return res;

  // 6) BnB node budgets
  const int rangeL = static_cast<int>(
      std::ceil(opt.xy_range_m / (map_->res * (1 << Lmax_map))));
  const int root_nodes = (2 * rangeL + 1) * (2 * rangeL + 1);

  // coarse: very small budget (for yaw ranking)
  const int max_nodes_coarse = std::max(1500, root_nodes * 6);
  // full: larger budget (final)
  int max_nodes_full = std::max(20000, root_nodes * 120);
  max_nodes_full = std::min(max_nodes_full, 350000);

  // 7) coarse yaw prescreen: rank by coarse_score = best_sum / (65535*|scan0|)
  struct YawRank {
    double yaw = 0.0;
    double score = 0.0;
  };
  std::vector<YawRank> ranked;
  ranked.reserve(yaw_coarse.size());

  for (double yaw : yaw_coarse) {
    auto *ce = get_scan_cache(yaw);
    if (!ce || ce->scan0.empty()) continue;

    const double denom = static_cast<double>(std::max<size_t>(1, ce->scan0.size()));
    const BnBResult r = SearchXYBnB_(ce->scanL, center_tx0, center_ty0,
                                    opt.xy_range_m, Lmax_map, max_nodes_coarse);
    if (!r.ok || r.best_sum == 0) continue;

    const double sc = (static_cast<double>(r.best_sum) / 65535.0) / denom;
    ranked.push_back({NormalizeAngle_(yaw), sc});
  }

  if (ranked.empty()) return res;

  std::sort(ranked.begin(), ranked.end(),
            [](const YawRank &a, const YawRank &b) { return a.score > b.score; });

  // keep more yaw seeds (关键：防止真值 yaw 被粗筛掉)
  // 要更激进可把 15 改成 20。
  const int kYawKeep = 15;
  if ((int)ranked.size() > kYawKeep) ranked.resize(kYawKeep);

  // 8) build fine yaw list around top seeds (dedupe)
  std::vector<double> yaw_fine;
  yaw_fine.reserve(256);
  {
    std::unordered_set<int> used;
    used.reserve(512);

    // refine range: ±yaw_step_coarse
    const double half = yaw_step_coarse;

    for (const auto &yr : ranked) {
      for (double da = -half; da <= half + 1e-12; da += yaw_step_fine) {
        const double yaw = NormalizeAngle_(yr.yaw + da);
        const int k = yaw_key(yaw);
        if (used.insert(k).second) yaw_fine.push_back(yaw);
      }
    }

    // 如果不是 full_circle，但窗口较大，也把 center.yaw 附近强行补上
    const int k0 = yaw_key(center.yaw);
    if (used.insert(k0).second) yaw_fine.push_back(NormalizeAngle_(center.yaw));
  }

  if (yaw_fine.empty()) return res;

  // 9) final search: for each yaw, run full BnB for XY; pick global best
  struct BestCand {
    bool ok = false;
    double yaw = 0.0;
    int tx0 = 0;
    int ty0 = 0;
    double score = 0.0;         // normalized [0..1]
    double second_score = 0.0;  // normalized [0..1] (same yaw's second best xy)
    double vf = 0.0;            // valid fraction
  };

  BestCand best;
  double best_other_score = 0.0;  // best score among non-best candidates

  for (double yaw : yaw_fine) {
    auto *ce = get_scan_cache(yaw);
    if (!ce || ce->scan0.empty()) continue;

    const double denom = static_cast<double>(std::max<size_t>(1, ce->scan0.size()));

    const BnBResult r = SearchXYBnB_(ce->scanL, center_tx0, center_ty0,
                                    opt.xy_range_m, Lmax_map, max_nodes_full);
    if (!r.ok || r.best_sum == 0) continue;

    // compute vf via known_cnt at best xy
    int known_cnt = 0;
    (void)SumScoreAtL0_(r.tx0, r.ty0, ce->scan0, &known_cnt);

    const double sc = (static_cast<double>(r.best_sum) / 65535.0) / denom;
    const double sc2 = (static_cast<double>(r.second_sum) / 65535.0) / denom;
    const double vf = static_cast<double>(known_cnt) / denom;

    if (!best.ok || sc > best.score) {
      if (best.ok) best_other_score = std::max(best_other_score, best.score);
      best.ok = true;
      best.yaw = NormalizeAngle_(yaw);
      best.tx0 = r.tx0;
      best.ty0 = r.ty0;
      best.score = sc;
      best.second_score = sc2;
      best.vf = vf;
    } else {
      best_other_score = std::max(best_other_score, sc);
    }
  }

  if (!best.ok) return res;

  // 10) fill result
  const RelocPose2D coarse_pose = GridToPoseMap_(best.tx0, best.ty0, best.yaw);
  double refined_score = best.score;
  double refined_vf = best.vf;
  const RelocPose2D refined_pose =
      RefinePoseSubcell_(scan, coarse_pose, opt, &refined_score, &refined_vf);
  res.best_pose = refined_pose;
  res.best_score = refined_score;

  // global second = max( best's own second-best-xy , other yaws' best )
  const double second_global = std::max(best.second_score, best_other_score);
  res.second_score = second_global;
  res.valid_fraction = refined_vf;
  res.T_seed = PoseToMat_(res.best_pose);


    // ---------------- P1 log (NO logic change) ----------------
  {
    const double best_yaw_deg = res.best_pose.yaw * 180.0 / kPiReloc;
    const double second_global = res.second_score;
    const double margin = res.best_score - second_global;

    // 只在失败时一定打印；成功时前 3 次打印（避免刷屏）
    static int s_ok_print = 0;
    const bool need_print = (!res.ok) || (s_ok_print < 3);
    if (need_print) {
      if (res.ok) ++s_ok_print;

      std::fprintf(stderr,
                   "[local_reloc][P1] ok=%d "
                   "best_score=%.4f second=%.4f margin=%.4f "
                   "vf=%.3f "
                   "gate(score=%d vf=%d margin=%d) "
                   "best_pose(x=%.3f y=%.3f yaw=%.1fdeg) "
                   "win(xy=%.2f yaw=%.1fdeg) step(yaw=%.2fdeg) "
                   "bnb(req=%d used=%d built=%d)\n",
                   res.ok ? 1 : 0,
                   res.best_score, second_global, margin,
                   res.valid_fraction,
                   (res.best_score >= opt.min_score) ? 1 : 0,
                   (res.valid_fraction >= opt.min_valid_fraction) ? 1 : 0,
                   ((margin) >= opt.score_margin) ? 1 : 0,
                   res.best_pose.x, res.best_pose.y, best_yaw_deg,
                   opt.xy_range_m, opt.yaw_range_rad * 180.0 / kPiReloc,
                   opt.yaw_step_rad * 180.0 / kPiReloc,
                   std::max(0, opt.bnb_max_level),
                   Lmax_map,
                   static_cast<int>(map_->pyr.size()) - 1);
    }
  }
  // ----------------------------------------------------------


  // 11) acceptance gates
  const bool pass_score = (res.best_score >= opt.min_score);
  const bool pass_vf = (res.valid_fraction >= opt.min_valid_fraction);
  const bool pass_margin = ((res.best_score - res.second_score) >= opt.score_margin);
  res.ok = pass_score && pass_vf && pass_margin;

  return res;
}

} // namespace localization_ndt

