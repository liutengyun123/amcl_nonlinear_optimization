#include "localization_ndt/config/yaml_utils.hpp"

#include <boost/filesystem.hpp>

namespace localization_ndt {
namespace config {

std::string TrimCopy(const std::string& s) {
  auto is_space = [](unsigned char c) { return std::isspace(c) != 0; };
  std::size_t b = 0, e = s.size();
  while (b < e && is_space(static_cast<unsigned char>(s[b]))) ++b;
  while (e > b && is_space(static_cast<unsigned char>(s[e - 1]))) --e;
  return s.substr(b, e - b);
}

std::string ToUpper(const std::string& s) {
  std::string out = s;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::toupper(c));
  });
  return out;
}

std::string ExtractFileStem(const std::string& path_or_name) {
  std::size_t slash_pos = path_or_name.find_last_of("/\\");
  std::size_t start = (slash_pos == std::string::npos) ? 0 : slash_pos + 1;

  std::size_t dot_pos = path_or_name.find_last_of('.');
  if (dot_pos == std::string::npos || dot_pos <= start) {
    return path_or_name.substr(start);
  }
  return path_or_name.substr(start, dot_pos - start);
}

bool FileExists(const std::string& path) {
  std::ifstream f(path.c_str());
  return f.good();
}

bool SaveTextToFile(const std::string& path, const std::string& text) {
  std::ofstream ofs(path.c_str());
  if (!ofs.is_open()) {
    ROS_WARN_STREAM("Failed to open file for writing: " << path);
    return false;
  }
  ofs << text;
  ofs.close();
  return true;
}

bool SaveYamlToFile(const std::string& path, const YAML::Node& node) {
  try {
    YAML::Emitter out;
    out << node;
    std::ofstream fout(path.c_str());
    if (!fout.is_open()) {
      ROS_WARN_STREAM("SaveYamlToFile: failed to open '" << path << "' for write.");
      return false;
    }
    fout << out.c_str();
    fout.close();
    return true;
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("SaveYamlToFile: failed to write '" << path
                                                        << "', error: " << e.what());
    return false;
  }
}

void EnsureDir(const std::string& dir) {
  if (dir.empty()) return;
  try {
    boost::filesystem::create_directories(dir);
  } catch (...) {
    ROS_WARN_STREAM("Failed to create directory: " << dir);
  }
}

}  // namespace config
}  // namespace localization_ndt
