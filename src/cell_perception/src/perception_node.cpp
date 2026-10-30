// This is the perception node, which answers /perception/detect for the DetectObjects skill. On
// each ~/detect request it triggers one frame from the overhead camera, runs the detector on it and
// returns the objects, filtered by the class and colour the request asks for. The camera is polled
// rather than streaming, so a detection always sees a frame rendered after the request, and the
// skill moves the arm to its observe pose first so the robot is out of the view. The objects also
// go out on ~/objects, as TF frames obj_<id>, and as an annotated image on ~/debug_image when
// something subscribes. color_noise can be changed while it runs, which is how cell_monitor injects
// camera noise as a fault. cell.launch.py starts it.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cell_interfaces/msg/detected_object_array.hpp"
#include "cell_interfaces/srv/detect_objects.hpp"
#include "cell_perception/detector.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace cell_perception
{

using namespace std::chrono_literals;
using cell_interfaces::msg::DetectedObject;
using cell_interfaces::msg::DetectedObjectArray;
using cell_interfaces::srv::DetectObjects;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class PerceptionNode : public rclcpp::Node
{
public:
  explicit PerceptionNode(const rclcpp::NodeOptions & options)
  : Node("perception", options),
    detector_(loadDetectorConfig(
        declare_parameter<std::string>(
          "objects_file", ament_index_cpp::get_package_share_directory("cell_description") +
          "/config/objects.yaml"),
        declare_parameter<std::string>(
          "perception_file", ament_index_cpp::get_package_share_directory("cell_perception") +
          "/config/perception.yaml")))
  {
    table_frame_ = declare_parameter<std::string>("table_frame", "table");
    frame_timeout_ = declare_parameter<double>("frame_timeout", 5.0);
    frame_attempts_ = static_cast<int>(declare_parameter<int64_t>("frame_attempts", 3));
    color_noise_ = declare_parameter<double>("color_noise", 0.0);  // std dev of a channel gain
    // cell_monitor turns the noise on for the runs that inject it as a fault, and off again
    param_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "color_noise") {
            color_noise_ = p.as_double();
            RCLCPP_INFO(get_logger(), "color_noise is now %.3f", color_noise_.load());
          }
        }
        return result;
      });
    rng_ = cv::RNG(static_cast<uint64_t>(declare_parameter<int64_t>("noise_seed", 1)));
    // This is added to camera_info's principal point. The MuJoCo plugin reports cx = W/2 and
    // cy = H/2, but with pixel centres at integer coordinates a symmetric frustum is centred at
    // W/2 - 0.5, so the plugin's cx and cy need a -0.5 px correction.
    principal_point_offset_ = declare_parameter<double>("principal_point_offset", -0.5);

    // images arrive while a request blocks waiting for them, so they get their own group
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    trigger_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = sensor_group_;

    // Reliable, not SensorDataQoS. Frames are requested one at a time, and best-effort delivery
    // of large messages (1.2 MB for a depth image) over fragmented UDP loses whole frames. The
    // VM's shared-memory transport is broken as well, so the camera topics need reliable QoS.
    const auto sensor_qos = rclcpp::QoS(2).reliable();
    color_sub_ = create_subscription<Image>(
      declare_parameter<std::string>("color_topic", "/camera/image_raw"), sensor_qos,
      [this](Image::ConstSharedPtr msg) {store(color_, color_count_, msg);}, sub_options);
    depth_sub_ = create_subscription<Image>(
      declare_parameter<std::string>("depth_topic", "/camera/depth"), sensor_qos,
      [this](Image::ConstSharedPtr msg) {store(depth_, depth_count_, msg);}, sub_options);
    info_sub_ = create_subscription<CameraInfo>(
      declare_parameter<std::string>("info_topic", "/camera/camera_info"), sensor_qos,
      [this](CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        info_ = msg;
      }, sub_options);
    trigger_client_ = create_client<std_srvs::srv::Trigger>(
      declare_parameter<std::string>("trigger_service", "/mujoco_camera_plugin/overhead/trigger"),
      rclcpp::ServicesQoS(), trigger_group_);

    objects_pub_ = create_publisher<DetectedObjectArray>("~/objects", 10);
    debug_pub_ = create_publisher<Image>("~/debug_image", 1);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    service_ = create_service<DetectObjects>(
      "~/detect",
      [this](
        const std::shared_ptr<DetectObjects::Request> req,
        std::shared_ptr<DetectObjects::Response> res) {detect(*req, *res);},
      rclcpp::ServicesQoS(), service_group_);

    RCLCPP_INFO(
      get_logger(), "Perception ready: %zu object types, %zu colours",
      detector_.config().types.size(), detector_.config().colors.size());
  }

private:
  void store(Image::ConstSharedPtr & slot, std::size_t & count, const Image::ConstSharedPtr & msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      slot = msg;
      ++count;
    }
    frame_cv_.notify_all();
  }

  // True once a render has been requested and the colour and depth pair that arrives after it
  // is in hand, with the reason in `error` when not. Arrival is counted rather than stamps
  // compared, because the plugin stamps a frame with the previous step's time. A render is asked
  // for up to frame_attempts times: in a lab benchmark of 20 runs, 8 detections out of a few
  // hundred got no matching pair within frame_timeout of their trigger, and since one failed look
  // ends a watch, those runs left their vials unsorted, so a missing frame is asked for again.
  bool captureFrame(
    Image::ConstSharedPtr & color, Image::ConstSharedPtr & depth,
    CameraInfo::ConstSharedPtr & info, std::string & error)
  {
    if (!trigger_client_->wait_for_service(2s)) {
      error = "camera trigger service not available";
      return false;
    }
    for (int attempt = 1; attempt <= frame_attempts_; ++attempt) {
      std::size_t color_before, depth_before;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        color_before = color_count_;
        depth_before = depth_count_;
      }
      auto trigger = trigger_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
      if (trigger.wait_for(2s) != std::future_status::ready || !trigger.get()->success) {
        error = "camera trigger failed";
        continue;
      }
      std::unique_lock<std::mutex> lock(mutex_);
      const bool ok = frame_cv_.wait_for(
        lock, std::chrono::duration<double>(frame_timeout_), [&]() {
          return color_count_ > color_before && depth_count_ > depth_before && info_ &&
                 color_->header.stamp == depth_->header.stamp;
        });
      if (ok) {
        color = color_;
        depth = depth_;
        info = info_;
        return true;
      }
      error = "no camera frame within " + std::to_string(frame_timeout_) + " s of the trigger";
      if (attempt < frame_attempts_) {
        RCLCPP_WARN(get_logger(), "%s (attempt %d of %d), triggering again", error.c_str(),
          attempt, frame_attempts_);
      }
    }
    error += " after " + std::to_string(frame_attempts_) + " attempts";
    return false;
  }

  void detect(const DetectObjects::Request & request, DetectObjects::Response & response)
  {
    const auto start = std::chrono::steady_clock::now();
    Image::ConstSharedPtr color_msg, depth_msg;
    CameraInfo::ConstSharedPtr info;
    // Average `frames` captures, each tinted by a random colour gain. A rendered frame is exact,
    // so color_noise stands in for a real camera's frame-to-frame white-balance drift, which
    // tints the whole image at once: per-pixel noise would average out over a vial's hundred top
    // pixels within one frame and never move a reading. Averaging is the second look a borderline
    // reading gets, and it divides the drift by the square root of the frame count.
    const int frames = std::max<int>(1, request.frames);
    cv::Mat sum;
    for (int i = 0; i < frames; ++i) {
      if (!captureFrame(color_msg, depth_msg, info, response.message)) {
        RCLCPP_WARN(get_logger(), "Detection failed: %s", response.message.c_str());
        return;
      }
      cv::Mat frame;
      cv_bridge::toCvShare(color_msg, "rgb8")->image.convertTo(frame, CV_32FC3);
      if (const double noise = color_noise_.load(); noise > 0.0) {
        cv::multiply(frame, cv::Scalar(1.0 + rng_.gaussian(noise),
          1.0 + rng_.gaussian(noise), 1.0 + rng_.gaussian(noise)), frame);
      }
      sum = sum.empty() ? frame : sum + frame;
    }
    cv::Mat rgb;
    sum.convertTo(rgb, CV_8UC3, 1.0 / frames);  // saturating, so noise cannot wrap around

    Eigen::Isometry3d table_T_camera;
    try {
      table_T_camera = tf2::transformToEigen(tf_buffer_->lookupTransform(
          table_frame_, color_msg->header.frame_id, tf2::TimePointZero, 1s));
    } catch (const tf2::TransformException & e) {
      response.message = std::string("no transform to the camera frame: ") + e.what();
      return;
    }

    const auto depth = cv_bridge::toCvShare(depth_msg, "32FC1")->image;
    const Intrinsics k{info->k[0], info->k[4], info->k[2] + principal_point_offset_,
      info->k[5] + principal_point_offset_};
    std::vector<Detection> detections;
    try {
      detections = detector_.detect(rgb, depth, k, table_T_camera);
    } catch (const std::exception & e) {
      response.message = std::string("detector failed: ") + e.what();
      return;
    }

    // stable ids within this frame, per type and ordered by x then y
    std::sort(detections.begin(), detections.end(), [](const Detection & a, const Detection & b) {
        if (a.type != b.type) {return a.type < b.type;}
        if (std::abs(a.pose.translation().x() - b.pose.translation().x()) > 1e-3) {
          return a.pose.translation().x() < b.pose.translation().x();
        }
        return a.pose.translation().y() < b.pose.translation().y();
      });

    DetectedObjectArray array;
    array.header.stamp = color_msg->header.stamp;
    array.header.frame_id = table_frame_;
    std::map<std::string, int> counts;
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    for (const auto & d : detections) {
      DetectedObject o;
      o.id = d.type + "_" + std::to_string(counts[d.type]++);
      o.type = d.type;
      o.object_class = d.object_class;
      o.color = d.color;
      o.pose.header = array.header;
      o.pose.pose = tf2::toMsg(d.pose);
      o.size.x = d.size.x();
      o.size.y = d.size.y();
      o.size.z = d.size.z();
      o.confidence = static_cast<float>(d.confidence);
      o.hue = static_cast<float>(d.hue);
      o.inner_area = static_cast<float>(d.inner_area);
      if ((!request.object_class.empty() && request.object_class != o.object_class) ||
        (!request.color.empty() && request.color != o.color))
      {
        continue;
      }
      auto tf = tf2::eigenToTransform(d.pose);
      tf.header = array.header;
      tf.child_frame_id = "obj_" + o.id;
      tfs.push_back(tf);
      array.objects.push_back(o);
    }

    response.success = true;
    response.detections = array;
    objects_pub_->publish(array);
    if (!tfs.empty()) {
      tf_broadcaster_->sendTransform(tfs);
    }
    if (debug_pub_->get_subscription_count() > 0) {
      auto debug = cv_bridge::CvImage(color_msg->header, "rgb8",
          Detector::drawDetections(rgb, detections)).toImageMsg();
      debug_pub_->publish(*debug);
    }
    const double ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    RCLCPP_INFO(
      get_logger(), "Detected %zu object(s) (%zu after filters) in %.0f ms", detections.size(),
      array.objects.size(), ms);
  }

  Detector detector_;
  std::string table_frame_;
  double frame_timeout_;
  int frame_attempts_ = 3;
  std::atomic<double> color_noise_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_handle_;
  cv::RNG rng_;
  double principal_point_offset_;

  std::mutex mutex_;
  std::condition_variable frame_cv_;
  Image::ConstSharedPtr color_, depth_;
  std::size_t color_count_ = 0;
  std::size_t depth_count_ = 0;
  CameraInfo::ConstSharedPtr info_;

  rclcpp::CallbackGroup::SharedPtr sensor_group_, trigger_group_, service_group_;
  rclcpp::Subscription<Image>::SharedPtr color_sub_, depth_sub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
  rclcpp::Publisher<DetectedObjectArray>::SharedPtr objects_pub_;
  rclcpp::Publisher<Image>::SharedPtr debug_pub_;
  rclcpp::Service<DetectObjects>::SharedPtr service_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace cell_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cell_perception::PerceptionNode>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
