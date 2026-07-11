#include "localization_ndt/config/run_config.hpp"
#include "localization_ndt/config/yaml_utils.hpp"

#include <ros/ros.h>

#include <algorithm>
#include <sstream>
#include <unordered_set>
#include <vector>

namespace localization_ndt {
namespace config {

static const char* kRunListKey = "--启动列表--";

std::string MakeDefaultRunYamlTextV2() {
  std::ostringstream oss;
  oss << R"RUN(# ============================================================
# run.yaml (map-only)
#
# 用法：
# 1) 直接选择：
#    run_target: <id>
#
# 2) 临时输入：
#    map: <name>
#    run_target: 0
#
#    节点启动时若检测到 map 不为空：
#      - 若列表已有该地图 -> run_target 切换到对应 id
#      - 若列表没有该地图 -> 自动追加新条目，并把 run_target 指向新 id
#    然后会自动清空 map。
# ============================================================
map: ""
run_target: 0
)RUN";
  oss << '"' << kRunListKey << R"RUN(": []
)RUN";
  return oss.str();
}

YAML::Node LoadOrCreateRunYamlV2(const std::string& run_yaml_path) {
  if (!FileExists(run_yaml_path)) {
    (void)SaveTextToFile(run_yaml_path, MakeDefaultRunYamlTextV2());
    ROS_WARN_STREAM("Created run.yaml template: " << run_yaml_path);
    return YAML::Node();
  }
  try {
    return YAML::LoadFile(run_yaml_path);
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Failed to load run.yaml '" << run_yaml_path
                                                << "', error: " << e.what()
                                                << ". Recreating template.");
    (void)SaveTextToFile(run_yaml_path, MakeDefaultRunYamlTextV2());
    return YAML::Node();
  }
}

struct RunEntry {
  int id = 0;
  std::string map;
};

static bool ExtractRunList(const YAML::Node& run_yaml,
                           std::vector<RunEntry>* out,
                           int* out_max_id) {
  if (!out || !out_max_id) return false;
  out->clear();
  *out_max_id = 0;

  const YAML::Node list = run_yaml[kRunListKey];
  if (!list) return true;
  if (!list.IsSequence()) return false;

  std::unordered_set<int> seen_id;
  for (std::size_t i = 0; i < list.size(); ++i) {
    const YAML::Node it = list[i];
    if (!it || !it.IsMap()) continue;

    RunEntry e;
    LoadFieldFromYaml(it, "id", e.id);
    LoadFieldFromYaml(it, "map", e.map);
    e.map = TrimCopy(e.map);

    if (e.id <= 0 || e.map.empty()) continue;
    if (!seen_id.insert(e.id).second) continue;

    out->push_back(e);
    *out_max_id = std::max(*out_max_id, e.id);
  }
  return true;
}

static int FindIdByMap(const std::vector<RunEntry>& list,
                       const std::string& map) {
  const std::string key = ToUpper(TrimCopy(map));
  for (const auto& e : list) {
    if (ToUpper(TrimCopy(e.map)) == key) return e.id;
  }
  return 0;
}

static bool GetMapById(const std::vector<RunEntry>& list,
                       int id,
                       std::string* map) {
  for (const auto& e : list) {
    if (e.id == id) {
      if (map) *map = e.map;
      return true;
    }
  }
  return false;
}

static void EnsureRunListNode(YAML::Node* run_yaml) {
  if (!run_yaml) return;
  if (!(*run_yaml)[kRunListKey] || !(*run_yaml)[kRunListKey].IsSequence()) {
    (*run_yaml)[kRunListKey] = YAML::Node(YAML::NodeType::Sequence);
  }
  (*run_yaml)[kRunListKey].SetStyle(YAML::EmitterStyle::Block);
}

static void NormalizeRunListStyle(YAML::Node* run_yaml) {
  if (!run_yaml) return;
  YAML::Node list = (*run_yaml)[kRunListKey];
  if (!list || !list.IsSequence()) return;
  list.SetStyle(YAML::EmitterStyle::Block);
  for (std::size_t i = 0; i < list.size(); ++i) {
    YAML::Node it = list[i];
    if (it && it.IsMap()) it.SetStyle(YAML::EmitterStyle::Flow);
  }
}

bool ResolveRunSelection(const std::string& run_yaml_path,
                         RunSelection* out,
                         std::string* out_error) {
  if (out) *out = RunSelection{};
  if (out_error) out_error->clear();

  YAML::Node run_yaml = LoadOrCreateRunYamlV2(run_yaml_path);

  int run_target = 0;
  LoadFieldFromYaml(run_yaml, "run_target", run_target);

  std::string map_from_run;
  LoadFieldFromYaml(run_yaml, "map", map_from_run);
  map_from_run = TrimCopy(map_from_run);

  std::vector<RunEntry> run_list;
  int max_id = 0;
  if (!ExtractRunList(run_yaml, &run_list, &max_id)) {
    if (out_error) {
      *out_error = std::string("run.yaml: '") + kRunListKey + "' must be a YAML sequence.";
    }
    return false;
  }

  std::string map_name;
  bool wrote_back = false;

  if (!map_from_run.empty()) {
    int use_id = FindIdByMap(run_list, map_from_run);
    EnsureRunListNode(&run_yaml);

    if (use_id <= 0) {
      use_id = max_id + 1;
      YAML::Node item(YAML::NodeType::Map);
      item.SetStyle(YAML::EmitterStyle::Flow);
      item["id"] = use_id;
      item["map"] = map_from_run;
      run_yaml[kRunListKey].push_back(item);
      NormalizeRunListStyle(&run_yaml);
      run_list.push_back(RunEntry{use_id, map_from_run});
      ROS_INFO_STREAM("Added run entry id=" << use_id << " (map=" << map_from_run << ")");
    } else {
      ROS_INFO_STREAM("Switch run_target to existing id=" << use_id
                      << " (map=" << map_from_run << ")");
    }

    run_yaml["run_target"] = use_id;
    run_yaml["map"] = "";

    if (!SaveYamlToFile(run_yaml_path, run_yaml)) {
      ROS_WARN_STREAM("run.yaml: failed to save. Will continue with in-memory config.");
    } else {
      wrote_back = true;
    }

    run_target = use_id;
    if (!GetMapById(run_list, run_target, &map_name)) {
      if (out_error) {
        std::ostringstream oss;
        oss << "run.yaml: internal error, id=" << run_target
            << " not found after update.";
        *out_error = oss.str();
      }
      return false;
    }
  } else if (run_target > 0) {
    if (!GetMapById(run_list, run_target, &map_name)) {
      if (out_error) {
        std::ostringstream oss;
        oss << "run.yaml: run_target=" << run_target
            << " not found in list. Please set map to switch.";
        *out_error = oss.str();
      }
      return false;
    }
  } else {
    if (out_error) {
      std::ostringstream oss;
      oss << "\nrun.yaml: no selection.\n"
          << "Set either:\n"
          << "  - run_target: <id>\n"
          << "or:\n"
          << "  - map: <name>\n";
      *out_error = oss.str();
    }
    return false;
  }

  if (out) {
    out->run_target = run_target;
    out->map_name = map_name;
    out->wrote_back = wrote_back;
  }
  return true;
}

}  // namespace config
}  // namespace localization_ndt
