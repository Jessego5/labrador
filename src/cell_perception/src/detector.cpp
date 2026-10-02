// This implements the detector declared in detector.hpp and the loader for its configuration. The
// height of an object comes from the measured depth of its top face rather than being assumed, so
// a cylinder standing in a tray pocket is placed at its real height. A box's yaw is folded into a
// quarter turn, because a square top face looks the same every 90 degrees. Confidence falls off
// with the size and height mismatch and is cut to 0.7 of that when the blob's shape (judged by how
// much of its rectangle it fills) disagrees with the matched type.
#include "cell_perception/detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <utility>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace cell_perception
{
namespace
{

// the point in the camera optical frame seen at pixel `px` with depth `z`
Eigen::Vector3d backProject(const cv::Point2d & px, double z, const Intrinsics & k)
{
  return {(px.x - k.cx) * z / k.fx, (px.y - k.cy) * z / k.fy, z};
}

// `yaw` folded into (-pi/4, pi/4]
double foldQuarterTurn(double yaw)
{
  double folded = std::remainder(yaw, M_PI / 2.0);
  if (folded <= -M_PI / 4.0 + 1e-12) {
    folded += M_PI / 2.0;
  }
  return folded;
}

// the value at `fraction` (0..1) of the way through the sorted values
float percentile(std::vector<float> values, double fraction)
{
  const auto k = static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1));
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(k), values.end());
  return values[k];
}

}  // namespace

Detector::Detector(DetectorConfig config)
: config_(std::move(config)) {}

std::vector<Detection> Detector::detect(
  const cv::Mat & rgb, const cv::Mat & depth, const Intrinsics & k,
  const Eigen::Isometry3d & table_T_camera) const
{
  if (rgb.type() != CV_8UC3 || depth.type() != CV_32FC1 || rgb.size() != depth.size()) {
    throw std::invalid_argument("detect: expected CV_8UC3 rgb and CV_32FC1 depth of equal size");
  }
  cv::Mat hsv;
  cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);
  const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3});

  std::vector<Detection> detections;
  for (const auto & color : config_.colors) {
    cv::Mat mask = cv::Mat::zeros(rgb.size(), CV_8UC1);
    for (const auto & range : color.ranges) {
      cv::Mat part;
      cv::inRange(hsv, range.lo, range.hi, part);
      mask |= part;
    }
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const auto & contour : contours) {
      if (cv::contourArea(contour) < config_.min_area_px) {
        continue;
      }
      // the pixels of this blob that have a valid depth
      cv::Mat blob = cv::Mat::zeros(rgb.size(), CV_8UC1);
      cv::drawContours(blob, std::vector<std::vector<cv::Point>>{contour}, 0, 255, cv::FILLED);
      blob &= mask;
      std::vector<cv::Point> pixels;
      cv::findNonZero(blob, pixels);
      std::vector<float> depths;
      depths.reserve(pixels.size());
      for (const auto & p : pixels) {
        const float z = depth.at<float>(p);
        if (std::isfinite(z) && z > 0.0f) {
          depths.push_back(z);
        }
      }
      if (depths.size() < static_cast<std::size_t>(config_.min_area_px)) {
        continue;
      }

      // the top face is the pixels nearest the camera, since the sides, seen in perspective, are
      // deeper
      const double z_near = percentile(depths, 0.05);
      std::vector<cv::Point> top;
      std::vector<float> top_depths;
      for (const auto & p : pixels) {
        const float z = depth.at<float>(p);
        if (std::isfinite(z) && z > 0.0f && z <= z_near + config_.top_band) {
          top.push_back(p);
          top_depths.push_back(z);
        }
      }
      if (top.size() < static_cast<std::size_t>(0.5 * config_.min_area_px)) {
        continue;
      }
      const double z_top = percentile(top_depths, 0.5);

      cv::Point2d centroid(0.0, 0.0);
      for (const auto & p : top) {
        centroid += cv::Point2d(p.x, p.y);
      }
      centroid *= 1.0 / static_cast<double>(top.size());
      const Eigen::Vector3d top_center = table_T_camera * backProject(centroid, z_top, k);
      if (top_center.z() < config_.min_top_height || top_center.z() > config_.max_top_height ||
        (top_center.head<2>().array() < config_.roi_min.array()).any() ||
        (top_center.head<2>().array() > config_.roi_max.array()).any())
      {
        continue;
      }

      // The footprint of the top face in metres. Pixels are cells, so the rectangle through the
      // pixel centres is one pixel short across each dimension.
      const cv::RotatedRect rect = cv::minAreaRect(top);
      const double m_per_px = z_top / k.fx;
      const double a = (rect.size.width + 1.0) * m_per_px;
      const double b = (rect.size.height + 1.0) * m_per_px;
      const Eigen::Vector2d footprint(std::max(a, b), std::min(a, b));
      const double fill = static_cast<double>(top.size()) /
        std::max(1.0, static_cast<double>((rect.size.width + 1.0) * (rect.size.height + 1.0)));
      const std::string shape = fill > 0.88 ? "box" : "cylinder";

      // the best-matching type of this colour
      const ObjectType * best = nullptr;
      double best_size_err = std::numeric_limits<double>::infinity();
      double best_height_err = 0.0;
      for (const auto & type : config_.types) {
        if (type.color != color.name) {
          continue;
        }
        const Eigen::Vector2d expected(
          std::max(type.size.x(), type.size.y()), std::min(type.size.x(), type.size.y()));
        const double size_err = (footprint - expected).cwiseAbs().maxCoeff();
        const double height_err = std::abs(top_center.z() - type.size.z());
        if (size_err + height_err < best_size_err + best_height_err) {
          best = &type;
          best_size_err = size_err;
          best_height_err = height_err;
        }
      }
      if (!best || best_size_err > config_.max_size_error ||
        best_height_err > config_.max_height_error)
      {
        continue;
      }

      Detection d;
      d.type = best->name;
      d.object_class = best->object_class;
      d.color = color.name;
      d.size = best->size;
      d.top_center = top_center;
      d.footprint = footprint;
      d.top_rect = rect;
      d.confidence = std::exp(
        -std::pow(best_size_err / 0.006, 2) - std::pow(best_height_err / 0.008, 2)) *
        (shape == best->shape ? 1.0 : 0.7);

      // The hue of the top face as a number, for readings that a colour name is too coarse for
      // (a test strip between positive and negative). OpenCV's H is half a degree, and the mean
      // is taken on the circle because red sits on both sides of 0.
      double hs = 0.0, hc = 0.0;
      for (const auto & p : top) {
        const double angle = 2.0 * hsv.at<cv::Vec3b>(p)[0] * M_PI / 180.0;
        hs += std::sin(angle);
        hc += std::cos(angle);
      }
      d.hue = std::fmod(std::atan2(hs, hc) * 180.0 / M_PI + 360.0, 360.0);

      // The area of the type's inner colour inside its outline: a culture dish's colony, which
      // grows across the dish's top. The outline is the blob's outer contour filled in, since the
      // colony itself is a hole in the dish colour's mask.
      if (!best->inner_color.empty()) {
        const auto inner = std::find_if(config_.colors.begin(), config_.colors.end(),
            [&](const ColorClass & c) {return c.name == best->inner_color;});
        if (inner != config_.colors.end()) {
          cv::Mat region = cv::Mat::zeros(rgb.size(), CV_8UC1);
          cv::drawContours(region, std::vector<std::vector<cv::Point>>{contour}, 0, 255,
            cv::FILLED);
          cv::Mat inner_mask = cv::Mat::zeros(rgb.size(), CV_8UC1);
          for (const auto & range : inner->ranges) {
            cv::Mat part;
            cv::inRange(hsv, range.lo, range.hi, part);
            inner_mask |= part;
          }
          d.inner_area = cv::countNonZero(region & inner_mask) * m_per_px * m_per_px;
        }
      }

      double yaw = 0.0;
      if (best->shape == "box") {
        // the direction of one rectangle edge, measured on the table
        cv::Point2f corners[4];
        rect.points(corners);
        const Eigen::Vector3d p0 = table_T_camera * backProject(corners[0], z_top, k);
        const Eigen::Vector3d p1 = table_T_camera * backProject(corners[1], z_top, k);
        yaw = foldQuarterTurn(std::atan2(p1.y() - p0.y(), p1.x() - p0.x()));
      }
      d.pose = Eigen::Isometry3d::Identity();
      d.pose.translation() = top_center - Eigen::Vector3d(0.0, 0.0, 0.5 * best->size.z());
      d.pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
      detections.push_back(d);
    }
  }

  // A culture dish's colony is the dish's inner colour inside its outline, and once it has grown
  // to a vial's width (30 mm across from the third stage) it also passed as a yellow vial standing
  // there, so every growth run reported phantom vials. A blob of a dish type's inner colour whose
  // centre lies on a detected dish is part of that dish.
  auto inner_of = [&](const Detection & dish) -> const std::string * {
      const auto t = std::find_if(config_.types.begin(), config_.types.end(),
          [&](const ObjectType & type) {return type.name == dish.type;});
      return t == config_.types.end() || t->inner_color.empty() ? nullptr : &t->inner_color;
    };
  std::vector<Detection> kept;
  for (const auto & d : detections) {
    const bool on_dish = std::any_of(detections.begin(), detections.end(),
        [&](const Detection & dish) {
          const std::string * inner = inner_of(dish);
          return inner && *inner == d.color &&
          (dish.top_center - d.top_center).head<2>().norm() < 0.5 * dish.footprint.x();
        });
    if (!on_dish) {
      kept.push_back(d);
    }
  }
  return kept;
}

cv::Mat Detector::drawDetections(const cv::Mat & rgb, const std::vector<Detection> & detections)
{
  cv::Mat out = rgb.clone();
  for (const auto & d : detections) {
    cv::Point2f corners[4];
    d.top_rect.points(corners);
    for (int i = 0; i < 4; ++i) {
      cv::line(out, corners[i], corners[(i + 1) % 4], {255, 0, 255}, 1, cv::LINE_AA);
    }
    char label[64];
    std::snprintf(label, sizeof(label), "%s %.2f", d.type.c_str(), d.confidence);
    cv::putText(
      out, label, d.top_rect.center + cv::Point2f(8.0f, -8.0f), cv::FONT_HERSHEY_SIMPLEX, 0.35,
      {255, 0, 255}, 1, cv::LINE_AA);
  }
  return out;
}

// Configuration

namespace
{

std::runtime_error configError(const std::string & where, const std::string & what)
{
  return std::runtime_error("perception config: " + where + ": " + what);
}

template<int N>
Eigen::Matrix<double, N, 1> vec(const YAML::Node & node, const std::string & where)
{
  if (!node || !node.IsSequence() || node.size() != static_cast<std::size_t>(N)) {
    throw configError(where, "expected a list of " + std::to_string(N) + " numbers");
  }
  Eigen::Matrix<double, N, 1> v;
  for (int i = 0; i < N; ++i) {
    v[i] = node[i].as<double>();
  }
  return v;
}

YAML::Node loadFile(const std::string & path)
{
  try {
    return YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("perception config: cannot read '" + path + "': " + e.what());
  }
}

}  // namespace

DetectorConfig loadDetectorConfig(const std::string & objects_file, const std::string & perception_file)
{
  DetectorConfig config;
  try {
    const YAML::Node objects = loadFile(objects_file);
    if (!objects["types"] || !objects["types"].IsMap()) {
      throw configError(objects_file, "missing 'types' mapping");
    }
    for (const auto & entry : objects["types"]) {
      const std::string name = entry.first.as<std::string>();
      const YAML::Node & n = entry.second;
      // a look the detector should not try to tell apart (the lab's per-stage twins)
      if (n["perceive"] && !n["perceive"].as<bool>()) {
        continue;
      }
      const std::string where = "types." + name;
      ObjectType t;
      t.name = name;
      t.object_class = n["class"].as<std::string>();
      t.color = n["color"].as<std::string>();
      t.shape = n["shape"].as<std::string>();
      t.size = vec<3>(n["size"], where + ".size");
      if (n["inner_color"]) {
        t.inner_color = n["inner_color"].as<std::string>();
      }
      config.types.push_back(t);
    }

    const YAML::Node p = loadFile(perception_file);
    if (!p["colors"] || !p["colors"].IsMap()) {
      throw configError(perception_file, "missing 'colors' mapping");
    }
    for (const auto & entry : p["colors"]) {
      ColorClass c;
      c.name = entry.first.as<std::string>();
      for (const auto & r : entry.second) {
        const auto lo = vec<3>(r["lo"], "colors." + c.name + ".lo");
        const auto hi = vec<3>(r["hi"], "colors." + c.name + ".hi");
        c.ranges.push_back({cv::Scalar(lo[0], lo[1], lo[2]), cv::Scalar(hi[0], hi[1], hi[2])});
      }
      config.colors.push_back(c);
    }
    if (p["roi"]) {
      const auto x = vec<2>(p["roi"]["x"], "roi.x");
      const auto y = vec<2>(p["roi"]["y"], "roi.y");
      config.roi_min = {x[0], y[0]};
      config.roi_max = {x[1], y[1]};
    }
    if (p["min_area_px"]) {config.min_area_px = p["min_area_px"].as<double>();}
    if (p["top_band"]) {config.top_band = p["top_band"].as<double>();}
    if (p["max_size_error"]) {config.max_size_error = p["max_size_error"].as<double>();}
    if (p["max_height_error"]) {config.max_height_error = p["max_height_error"].as<double>();}
    if (p["top_height"]) {
      const auto h = vec<2>(p["top_height"], "top_height");
      config.min_top_height = h[0];
      config.max_top_height = h[1];
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("perception config: ") + e.what());
  }
  return config;
}

}  // namespace cell_perception
