#pragma once

#include <string>
#include <yaml-cpp/yaml.h>

namespace localization_ndt {
namespace config {

std::string MakeDefaultParamsYamlText(const std::string& map_stem);

YAML::Node LoadOrCreateParamsYaml(const std::string& params_dir,
                                  const std::string& map_stem);

}  // namespace config
}  // namespace localization_ndt
