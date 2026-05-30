// This is the test that keeps the simulation and the ROS side of the workspace in step. The
// layout has one source of truth, cell_description/config/workspace.yaml: the C++ workspace
// model reads it directly, and the MuJoCo geometry in cell_description/mjcf/workspace.xml is
// generated from it by scripts/build_workspace_mjcf.py. If someone edits the YAML and forgets to
// regenerate the MJCF, the robot would be sent to TF frames that no longer match where the trays
// are in MuJoCo, so this fails instead: every TF frame must have a MuJoCo site with the same name
// and pose, and the counts must agree. Run it with colcon test --packages-select cell_workspace.
#include <cmath>
#include <map>
#include <sstream>
#include <string>

#include <gtest/gtest.h>
#include <tinyxml2.h>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cell_workspace/workspace.hpp"

namespace
{

struct Site
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
};

std::map<std::string, Site> loadSites(const std::string & path)
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("cannot parse " + path);
  }
  std::map<std::string, Site> sites;
  const auto * world = doc.FirstChildElement("mujoco")->FirstChildElement("worldbody");
  // workspace frames are written as direct children of <worldbody>, in world coordinates
  for (auto * e = world->FirstChildElement("site"); e; e = e->NextSiblingElement("site")) {
    Site s;
    std::istringstream(e->Attribute("pos")) >> s.pos.x() >> s.pos.y() >> s.pos.z();
    if (const char * q = e->Attribute("quat")) {
      double w, x, y, z;
      std::istringstream(q) >> w >> x >> y >> z;
      s.quat = Eigen::Quaterniond(w, x, y, z).normalized();
    }
    sites[e->Attribute("name")] = s;
  }
  return sites;
}

}  // namespace

TEST(MjcfAlignment, EveryWorkspaceFrameMatchesAMujocoSite)
{
  const std::string share = ament_index_cpp::get_package_share_directory("cell_description");
  const auto ws = cell_workspace::loadWorkspaceFile(share + "/config/workspace.yaml");
  ASSERT_TRUE(ws.validate().empty());
  const auto sites = loadSites(share + "/mjcf/workspace.xml");

  const auto frames = ws.allFrames();
  EXPECT_EQ(frames.size(), sites.size())
    << "workspace.xml is out of date: re-run cell_description/scripts/build_workspace_mjcf.py";

  for (const auto & f : frames) {
    const auto it = sites.find(f.frame_id);
    if (it == sites.end()) {
      ADD_FAILURE() << "no MuJoCo site named '" << f.frame_id << "'";
      continue;
    }
    const double pos_err = (it->second.pos - f.pose.translation()).norm();
    const double ang_err = it->second.quat.angularDistance(Eigen::Quaterniond(f.pose.linear()));
    EXPECT_LT(pos_err, 1e-5) << f.frame_id << ": position differs by " << pos_err << " m";
    EXPECT_LT(ang_err, 1e-5) << f.frame_id << ": orientation differs by " << ang_err << " rad";
  }
}
