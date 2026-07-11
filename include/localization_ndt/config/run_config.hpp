#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace localization_ndt {
namespace config {

struct RunSelection {
  int run_target = 0;
  std::string map_name;
  bool wrote_back = false;
};

bool ResolveRunSelection(const std::string& run_yaml_path,
                         RunSelection* out,
                         std::string* out_error);

std::string MakeDefaultRunYamlTextV2();
YAML::Node LoadOrCreateRunYamlV2(const std::string& run_yaml_path);

}  // namespace config
}  // namespace localization_ndt
