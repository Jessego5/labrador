// This is the object detector for the overhead camera looking down at the table, working from
// colour and depth. For each colour it thresholds the image in HSV, finds the blobs, keeps the
// pixels closest to the camera (the top face), back-projects the top face's centroid with the
// depth image, and matches the measured footprint and height against the known object types. It
// is plain C++ over OpenCV, Eigen and yaml-cpp with no ROS in it, so test_detector can run it on
// synthetic images; perception_node is what runs it on the camera's frames.
#pragma once

#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace cell_perception
{

// one HSV interval, in OpenCV's ranges (H 0..180, S and V 0..255)
struct HsvRange
{
  cv::Scalar lo;
  cv::Scalar hi;
};

struct ColorClass
{
  std::string name;
  std::vector<HsvRange> ranges;  // unioned, since red wraps around H = 0
};

struct ObjectType
{
  std::string name;          // e.g. "blue_cylinder"
  std::string object_class;  // cube, cylinder, peg
  std::string color;
  std::string shape;         // box or cylinder
  Eigen::Vector3d size = Eigen::Vector3d::Zero();  // bounding box (x, y, z)
  // a colour to measure inside the top face, such as a colony on a culture dish ("" for none)
  std::string inner_color;
};

struct Intrinsics
{
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

struct DetectorConfig
{
  std::vector<ColorClass> colors;
  std::vector<ObjectType> types;
  Eigen::Vector2d roi_min{0.15, -0.55};  // accepted object centres, table x/y
  Eigen::Vector2d roi_max{0.85, 0.55};
  double min_area_px = 25.0;       // smallest blob considered
  double top_band = 0.004;         // m; pixels this close to the nearest depth form the top face
  double max_size_error = 0.012;   // m; footprint mismatch allowed when matching a type
  double max_height_error = 0.015; // m; top-height mismatch allowed
  double min_top_height = 0.005;   // m; ignore flat things (table markings, tray floor)
  double max_top_height = 0.15;    // m; ignore the robot
};

struct Detection
{
  std::string type;
  std::string object_class;
  std::string color;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();  // bounding-box centre, table frame
  Eigen::Vector3d size = Eigen::Vector3d::Zero();          // of the matched type
  double confidence = 0.0;
  // measurements, kept for debugging and the tests
  Eigen::Vector3d top_center = Eigen::Vector3d::Zero();  // centre of the top face, table frame
  Eigen::Vector2d footprint = Eigen::Vector2d::Zero();   // measured top face (long, short side)
  cv::RotatedRect top_rect;                              // top face in pixels
  double hue = 0.0;         // degrees 0..360, circular mean over the top face
  double inner_area = 0.0;  // m^2 of the type's inner colour inside the top face
};

class Detector
{
public:
  explicit Detector(DetectorConfig config);

  // The objects in one frame. `rgb` is CV_8UC3 in RGB order. `depth` is CV_32FC1 in metres
  // along the optical axis, where a non-finite value or one of 0 or less means no data.
  // `table_T_camera` is the pose in the table frame of the camera optical frame (x right, y
  // down, z forward). Throws std::invalid_argument when the images are of the wrong type or of
  // different sizes.
  std::vector<Detection> detect(
    const cv::Mat & rgb, const cv::Mat & depth, const Intrinsics & intrinsics,
    const Eigen::Isometry3d & table_T_camera) const;

  // an RGB copy of `rgb` with the detections outlined and labelled
  static cv::Mat drawDetections(const cv::Mat & rgb, const std::vector<Detection> & detections);

  const DetectorConfig & config() const {return config_;}

private:
  DetectorConfig config_;
};

// The detector configuration, with the object types from cell_description/config/objects.yaml
// (its `types:` section) and the thresholds and filters from a perception YAML file such as
// cell_perception/config/perception.yaml. Throws std::runtime_error naming the offending key.
DetectorConfig loadDetectorConfig(const std::string & objects_file, const std::string & perception_file);

}  // namespace cell_perception
