// This reads workspace.yaml into the workspace model. The format itself is described in
// cell_description/config/workspace.yaml, the one file the TF frames, the MuJoCo geometry and
// the skills all take the layout from. Every parse error names the key path it came from, such
// as "trays.tray_1.pitch[1]", so a typo in the YAML points straight at the line to fix.
// Checking that the layout makes sense is left to Workspace::validate().
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "cell_workspace/workspace.hpp"

namespace cell_workspace
{
namespace
{

std::runtime_error parseError(const std::string & where, const std::string & what)
{
  return std::runtime_error("workspace config: " + where + ": " + what);
}

YAML::Node require(const YAML::Node & node, const std::string & key, const std::string & where)
{
  if (!node[key]) {
    throw parseError(where, "missing required key '" + key + "'");
  }
  return node[key];
}

double number(const YAML::Node & node, const std::string & where)
{
  try {
    return node.as<double>();
  } catch (const YAML::Exception &) {
    throw parseError(where, "expected a number");
  }
}

int integer(const YAML::Node & node, const std::string & where)
{
  try {
    return node.as<int>();
  } catch (const YAML::Exception &) {
    throw parseError(where, "expected an integer");
  }
}

template<int N>
Eigen::Matrix<double, N, 1> vector(const YAML::Node & node, const std::string & where)
{
  if (!node.IsSequence() || node.size() != static_cast<std::size_t>(N)) {
    throw parseError(where, "expected a list of " + std::to_string(N) + " numbers");
  }
  Eigen::Matrix<double, N, 1> v;
  for (int i = 0; i < N; ++i) {
    v[i] = number(node[i], where + "[" + std::to_string(i) + "]");
  }
  return v;
}

Eigen::Isometry3d poseFromXyzRpy(const Eigen::Matrix<double, 6, 1> & v)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = v.head<3>();
  // fixed-axis roll-pitch-yaw, as in URDF: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  pose.linear() = (Eigen::AngleAxisd(v[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(v[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(v[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}

// Calls fn for each entry of a mapping section such as `trays:`. A missing section is allowed
// and simply has no entries.
template<typename Fn>
void forEachEntry(const YAML::Node & root, const std::string & section, Fn && fn)
{
  const YAML::Node node = root[section];
  if (!node || node.IsNull()) {
    return;
  }
  if (!node.IsMap()) {
    throw parseError(section, "expected a mapping of name: {...}");
  }
  for (const auto & entry : node) {
    const std::string name = entry.first.as<std::string>();
    const std::string where = section + "." + name;
    if (!entry.second.IsMap()) {
      throw parseError(where, "expected a mapping");
    }
    fn(name, entry.second, where);
  }
}

Workspace parse(const YAML::Node & root)
{
  if (!root.IsMap()) {
    throw parseError("<root>", "expected a mapping");
  }
  Workspace ws;
  if (root["frame"]) {
    ws.frame = root["frame"].as<std::string>();
  }

  forEachEntry(
    root, "trays", [&ws](const std::string & name, const YAML::Node & n, const std::string & where) {
      Tray t;
      t.name = name;
      t.origin = vector<3>(require(n, "origin", where), where + ".origin");
      t.yaw = n["yaw"] ? number(n["yaw"], where + ".yaw") : 0.0;
      t.rows = integer(require(n, "rows", where), where + ".rows");
      t.cols = integer(require(n, "cols", where), where + ".cols");
      t.pitch = vector<2>(require(n, "pitch", where), where + ".pitch");
      t.slot_size = number(require(n, "slot_size", where), where + ".slot_size");
      t.slot_depth = number(require(n, "slot_depth", where), where + ".slot_depth");
      ws.trays.push_back(t);
    });

  forEachEntry(
    root, "bins", [&ws](const std::string & name, const YAML::Node & n, const std::string & where) {
      Bin b;
      b.name = name;
      b.center = vector<3>(require(n, "center", where), where + ".center");
      b.size = vector<3>(require(n, "size", where), where + ".size");
      if (n["spots"]) {
        const Eigen::Vector2d spots = vector<2>(n["spots"], where + ".spots");
        b.spot_rows = static_cast<int>(spots.x());
        b.spot_cols = static_cast<int>(spots.y());
        if (b.spot_rows != spots.x() || b.spot_cols != spots.y()) {
          throw std::runtime_error(where + ".spots: expected two integers");
        }
      }
      if (n["wall"]) {
        b.wall = number(n["wall"], where + ".wall");
      }
      ws.bins.push_back(b);
    });

  forEachEntry(
    root, "fixtures",
    [&ws](const std::string & name, const YAML::Node & n, const std::string & where) {
      Fixture f;
      f.name = name;
      f.hole_pose = poseFromXyzRpy(vector<6>(require(n, "hole_pose", where), where + ".hole_pose"));
      f.hole_diameter = number(require(n, "hole_diameter", where), where + ".hole_diameter");
      f.insert_depth = number(require(n, "insert_depth", where), where + ".insert_depth");
      if (n["block_size"]) {
        f.block_size = number(n["block_size"], where + ".block_size");
      }
      ws.fixtures.push_back(f);
    });

  return ws;
}

}  // namespace

Workspace loadWorkspaceString(const std::string & yaml)
{
  try {
    return parse(YAML::Load(yaml));
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("workspace config: YAML error: ") + e.what());
  }
}

Workspace loadWorkspaceFile(const std::string & path)
{
  try {
    return parse(YAML::LoadFile(path));
  } catch (const YAML::BadFile &) {
    throw std::runtime_error("workspace config: cannot open '" + path + "'");
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("workspace config: " + path + ": YAML error: " + e.what());
  }
}

}  // namespace cell_workspace
