#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>

namespace localization_ndt {
namespace config {

// string utils
std::string TrimCopy(const std::string& s);
std::string ToUpper(const std::string& s);

// /xxx/xw.pcd -> xw
// /xxx/ABTR006.urdf -> ABTR006
// ABTR006 -> ABTR006
std::string ExtractFileStem(const std::string& path_or_name);

// file / dir utils
bool FileExists(const std::string& path);
bool SaveTextToFile(const std::string& path, const std::string& text);
bool SaveYamlToFile(const std::string& path, const YAML::Node& node);
void EnsureDir(const std::string& dir);

// yaml field loader (template)
template <typename T>
inline void LoadFieldFromYaml(const YAML::Node& node,
                              const std::string& key,
                              T& value) {
  if (!node || !node[key]) return;
  try {
    value = node[key].as<T>();
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("LoadFieldFromYaml: key='" << key
                                               << "' parse error: " << e.what());
  }
}

}  // namespace config
}  // namespace localization_ndt
