// These are the tests for the detector, on synthetic images drawn with the cell's camera
// geometry: the overhead camera at (0.45, 0, 1.0) looking straight down, 640x480, fx = fy = 433
// px. They check that each object type is found, classified and located to within 1.5 mm, that a
// box's yaw is folded into a quarter turn, that the tray, a wrong-sized block, a cube behind the
// robot base and a flat mark on the table are all ignored, and that the reported height follows
// the measured top. No camera or simulator is needed. Run them with colcon test
// --packages-select cell_perception.
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <opencv2/imgproc.hpp>

#include "cell_perception/detector.hpp"

using namespace cell_perception;

namespace
{

const Intrinsics kK{432.97, 432.97, 320.0, 240.0};

Eigen::Isometry3d cameraPose()
{
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(0.45, 0.0, 1.0);
  t.linear() << 0, -1, 0,
    -1, 0, 0,
    0, 0, -1;
  return t;
}

DetectorConfig config()
{
  DetectorConfig c;
  c.colors = {
    {"red", {{cv::Scalar(0, 120, 50), cv::Scalar(8, 255, 255)},
      {cv::Scalar(172, 120, 50), cv::Scalar(180, 255, 255)}}},
    {"blue", {{cv::Scalar(100, 170, 40), cv::Scalar(125, 255, 255)}}},
    {"yellow", {{cv::Scalar(18, 120, 70), cv::Scalar(35, 255, 255)}}},
  };
  c.types = {
    {"red_cube", "cube", "red", "box", {0.04, 0.04, 0.04}},
    {"blue_cylinder", "cylinder", "blue", "cylinder", {0.03, 0.03, 0.05}},
    {"yellow_peg", "peg", "yellow", "cylinder", {0.02, 0.02, 0.06}},
  };
  return c;
}

// Synthetic RGB and depth of the table with the objects drawn as their top faces. The camera
// looks straight down, so the side faces are invisible at these positions.
struct Scene
{
  cv::Mat rgb{480, 640, CV_8UC3, cv::Scalar(200, 195, 190)};
  cv::Mat depth{480, 640, CV_32FC1, cv::Scalar(1.0f)};  // table top, z = 0
  Eigen::Isometry3d cam = cameraPose();

  cv::Point2d project(const Eigen::Vector3d & p) const
  {
    const Eigen::Vector3d c = cam.inverse() * p;
    return {kK.fx * c.x() / c.z() + kK.cx, kK.fy * c.y() / c.z() + kK.cy};
  }

  void box(double x, double y, double yaw, double side, double height, cv::Scalar rgb_color)
  {
    std::vector<cv::Point> pts;
    for (auto [dx, dy] : {std::pair{1, 1}, {-1, 1}, {-1, -1}, {1, -1}}) {
      const double lx = 0.5 * side * dx, ly = 0.5 * side * dy;
      const Eigen::Vector3d p(x + std::cos(yaw) * lx - std::sin(yaw) * ly,
        y + std::sin(yaw) * lx + std::cos(yaw) * ly, height);
      const auto px = project(p);
      pts.emplace_back(cvRound(px.x * 16), cvRound(px.y * 16));
    }
    cv::fillConvexPoly(rgb, pts, rgb_color, cv::LINE_8, 4);
    cv::fillConvexPoly(depth, pts, cv::Scalar(1.0 - height), cv::LINE_8, 4);
  }

  void disk(double x, double y, double diameter, double height, cv::Scalar rgb_color)
  {
    const auto c = project({x, y, height});
    const double r = 0.5 * diameter * kK.fx / (1.0 - height);
    const cv::Point center(cvRound(c.x * 16), cvRound(c.y * 16));
    cv::circle(rgb, center, cvRound(r * 16), rgb_color, cv::FILLED, cv::LINE_8, 4);
    cv::circle(depth, center, cvRound(r * 16), cv::Scalar(1.0 - height), cv::FILLED, cv::LINE_8, 4);
  }
};

const cv::Scalar kRed(217, 25, 25), kBlue(25, 76, 217), kYellow(230, 204, 25);
const cv::Scalar kTrayBlue(89, 140, 191);

const Detection * find(const std::vector<Detection> & ds, const std::string & type)
{
  for (const auto & d : ds) {
    if (d.type == type) {return &d;}
  }
  return nullptr;
}

}  // namespace

TEST(Detector, LocatesAndClassifiesEachObjectType)
{
  Scene s;
  s.box(0.40, 0.15, 0.3, 0.04, 0.04, kRed);
  s.disk(0.62, -0.20, 0.03, 0.05, kBlue);
  s.disk(0.30, -0.10, 0.02, 0.06, kYellow);
  const auto ds = Detector(config()).detect(s.rgb, s.depth, kK, s.cam);
  ASSERT_EQ(ds.size(), 3u);

  const auto * cube = find(ds, "red_cube");
  ASSERT_NE(cube, nullptr);
  EXPECT_LT((cube->pose.translation() - Eigen::Vector3d(0.40, 0.15, 0.02)).norm(), 0.0015);
  const double yaw = std::atan2(cube->pose.linear()(1, 0), cube->pose.linear()(0, 0));
  EXPECT_NEAR(std::remainder(yaw - 0.3, M_PI / 2), 0.0, 0.04);  // within ~2 deg, mod 90
  EXPECT_EQ(cube->object_class, "cube");
  EXPECT_GT(cube->confidence, 0.5);

  const auto * cyl = find(ds, "blue_cylinder");
  ASSERT_NE(cyl, nullptr);
  EXPECT_LT((cyl->pose.translation() - Eigen::Vector3d(0.62, -0.20, 0.025)).norm(), 0.0015);

  const auto * peg = find(ds, "yellow_peg");
  ASSERT_NE(peg, nullptr);
  EXPECT_LT((peg->pose.translation() - Eigen::Vector3d(0.30, -0.10, 0.03)).norm(), 0.0015);
}

TEST(Detector, YawIsFoldedIntoAQuarterTurn)
{
  for (double yaw : {-0.7, -0.2, 0.0, 0.5, 1.2}) {
    Scene s;
    s.box(0.5, 0.0, yaw, 0.04, 0.04, kRed);
    const auto ds = Detector(config()).detect(s.rgb, s.depth, kK, s.cam);
    ASSERT_EQ(ds.size(), 1u) << "yaw " << yaw;
    const double got = std::atan2(ds[0].pose.linear()(1, 0), ds[0].pose.linear()(0, 0));
    EXPECT_LE(std::abs(got), M_PI / 4 + 1e-9);
    EXPECT_NEAR(std::remainder(got - yaw, M_PI / 2), 0.0, 0.04) << "yaw " << yaw;
  }
}

TEST(Detector, IgnoresThingsThatAreNotObjects)
{
  Scene s;
  // a tray-coloured slab (desaturated blue, 20 mm high) is not a cylinder
  s.box(0.50, 0.0, 0.0, 0.16, 0.02, kTrayBlue);
  // the right colour but the wrong size (a 70 mm blue block)
  s.box(0.35, 0.30, 0.0, 0.07, 0.05, kBlue);
  // a real cube outside the region of interest, behind the robot base
  s.box(0.05, 0.0, 0.0, 0.04, 0.04, kRed);
  // a red mark printed on the table, with no height
  s.box(0.60, -0.30, 0.0, 0.04, 0.0, kRed);
  EXPECT_TRUE(Detector(config()).detect(s.rgb, s.depth, kK, s.cam).empty());
}

TEST(Detector, UsesTheMeasuredTopHeight)
{
  // A cylinder standing in a tray pocket: its top is 5 mm higher than on the table, and the
  // reported centre follows the measured top rather than assuming it rests on the table.
  Scene s;
  s.disk(0.45, -0.10, 0.03, 0.055, kBlue);
  const auto ds = Detector(config()).detect(s.rgb, s.depth, kK, s.cam);
  ASSERT_EQ(ds.size(), 1u);
  EXPECT_NEAR(ds[0].top_center.z(), 0.055, 0.001);
  EXPECT_NEAR(ds[0].pose.translation().z(), 0.030, 0.001);
}

TEST(Detector, RejectsMismatchedImages)
{
  const cv::Mat rgb(480, 640, CV_8UC3), depth(240, 320, CV_32FC1);
  EXPECT_THROW(Detector(config()).detect(rgb, depth, kK, cameraPose()), std::invalid_argument);
}
