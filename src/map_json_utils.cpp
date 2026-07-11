#include "localization_ndt/map_json_utils.hpp"

#include <cmath>
#include <fstream>

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace localization_ndt {
namespace {

static inline bool FileExists2(const std::string& p) {
  std::ifstream f(p.c_str(), std::ios::in | std::ios::binary);
  return f.good();
}

}  // namespace

bool LoadPointCloudFromJson(const std::string& json_path,
                            PointCloudT::Ptr* out_cloud,
                            JsonMapHeader* out_h,
                            std::string* out_err) {
  if (out_err) out_err->clear();
  if (!out_cloud) return false;

  if (!FileExists2(json_path)) {
    if (out_err) *out_err = "json not exists: " + json_path;
    return false;
  }

  boost::property_tree::ptree root;
  try {
    boost::property_tree::read_json(json_path, root);
  } catch (const std::exception& e) {
    if (out_err) *out_err = std::string("read_json failed: ") + e.what();
    return false;
  }

  JsonMapHeader h;
  std::vector<std::pair<double, double>> occupy_xy;

  try {
    auto& header = root.get_child("header");
    h.origin_x = header.get<double>("origin.x");
    h.origin_y = header.get<double>("origin.y");
    h.origin_yaw = header.get<double>("origin.yaw", 0.0);
    h.resolution = header.get<double>("resolution");
    h.size_w = header.get<double>("size.width");
    h.size_h = header.get<double>("size.height");

    if (!(std::isfinite(h.origin_x) && std::isfinite(h.origin_y) &&
          std::isfinite(h.origin_yaw) &&
          std::isfinite(h.resolution) && h.resolution > 0.0 &&
          std::isfinite(h.size_w) && std::isfinite(h.size_h) &&
          h.size_w > 0.0 && h.size_h > 0.0)) {
      if (out_err) *out_err = "header fields invalid.";
      return false;
    }

    // Explicitly reject non-zero yaw: current JSON pipeline is yaw=0 only.
    if (std::fabs(h.origin_yaw) > 1e-9) {
      if (out_err) {
        *out_err = "origin.yaw != 0 is not supported by this map pipeline.";
      }
      return false;
    }
    h.origin_yaw = 0.0;

    if (auto opt = root.get_child_optional("occupyPoints")) {
      for (auto& kv : opt.get()) {
        const auto& n = kv.second;
        const double x = n.get<double>("x");
        const double y = n.get<double>("y");
        if (std::isfinite(x) && std::isfinite(y))
          occupy_xy.push_back({x, y});
      }
    }
  } catch (const std::exception& e) {
    if (out_err) *out_err = std::string("parse json fields failed: ") + e.what();
    return false;
  }

  PointCloudT::Ptr cloud(new PointCloudT);
  cloud->points.reserve(occupy_xy.size());
  for (const auto& p : occupy_xy) {
    PointT pt;
    pt.x = static_cast<float>(p.first);
    pt.y = static_cast<float>(p.second);
    pt.z = 0.0f;
    pt.intensity = 1.0f;
    cloud->points.push_back(pt);
  }
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;

  *out_cloud = cloud;
  if (out_h) *out_h = h;
  return true;
}

}  // namespace localization_ndt
