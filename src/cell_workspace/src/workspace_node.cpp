// This is the workspace node, a thin ROS wrapper around the workspace model. It loads
// workspace.yaml and refuses to start if validate() finds a problem, then publishes a static TF
// frame for every tray, slot, bin and fixture (`tray_1_C4` and so on), latches RViz markers on
// ~/markers and serves ~/resolve_target, which turns a name such as "tray_1/C4" into a pose. The
// node is named "workspace", so these appear as /workspace/markers and
// /workspace/resolve_target. It is built as a composable component and also as the standalone
// workspace_node executable.
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "cell_interfaces/srv/resolve_target.hpp"
#include "cell_workspace/workspace.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace cell_workspace
{

class WorkspaceNode : public rclcpp::Node
{
public:
  using ResolveTarget = cell_interfaces::srv::ResolveTarget;

  explicit WorkspaceNode(const rclcpp::NodeOptions & options)
  : Node("workspace", options)
  {
    const auto path = declare_parameter<std::string>("workspace_file", defaultWorkspaceFile());
    workspace_ = loadWorkspaceFile(path);

    const auto errors = workspace_.validate();
    if (!errors.empty()) {
      for (const auto & e : errors) {
        RCLCPP_ERROR(get_logger(), "%s", e.c_str());
      }
      throw std::runtime_error("invalid workspace config: " + path);
    }

    publishFrames();
    publishMarkers();

    service_ = create_service<ResolveTarget>(
      "~/resolve_target",
      [this](
        const std::shared_ptr<ResolveTarget::Request> request,
        std::shared_ptr<ResolveTarget::Response> response) {resolve(*request, *response);});

    RCLCPP_INFO(
      get_logger(), "Loaded %s: %zu tray(s), %zu bin(s), %zu fixture(s), %zu TF frames",
      path.c_str(), workspace_.trays.size(), workspace_.bins.size(), workspace_.fixtures.size(),
      frame_count_);
  }

private:
  static std::string defaultWorkspaceFile()
  {
    return ament_index_cpp::get_package_share_directory("cell_description") +
           "/config/workspace.yaml";
  }

  void publishFrames()
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    const auto stamp = now();
    for (const auto & frame : workspace_.allFrames()) {
      auto tf = tf2::eigenToTransform(frame.pose);
      tf.header.stamp = stamp;
      tf.header.frame_id = workspace_.frame;
      tf.child_frame_id = frame.frame_id;
      transforms.push_back(tf);
    }
    frame_count_ = transforms.size();
    broadcaster_->sendTransform(transforms);
  }

  void publishMarkers()
  {
    using visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    const auto make = [&](
      const std::string & ns, int type, const Eigen::Isometry3d & pose,
      const Eigen::Vector3d & scale, const std::array<float, 4> & rgba) {
        Marker m;
        m.header.frame_id = workspace_.frame;  // zero stamp: RViz uses the latest transform
        m.ns = ns;
        m.id = id++;
        m.type = type;
        m.action = Marker::ADD;
        m.pose = tf2::toMsg(pose);
        m.scale.x = scale.x();
        m.scale.y = scale.y();
        m.scale.z = scale.z();
        m.color.r = rgba[0];
        m.color.g = rgba[1];
        m.color.b = rgba[2];
        m.color.a = rgba[3];
        return m;
      };
    const auto label = [&](const std::string & text, Eigen::Isometry3d pose, double height) {
        pose.translation().z() += height;
        Marker m = make(
          "labels", Marker::TEXT_VIEW_FACING, pose, {0.0, 0.0, 0.012}, {1.0f, 1.0f, 1.0f, 0.9f});
        m.text = text;
        return m;
      };

    for (const auto & t : workspace_.trays) {
      for (int r = 0; r < t.rows; ++r) {
        for (int c = 0; c < t.cols; ++c) {
          Eigen::Isometry3d pocket = t.slotPose(r, c);
          pocket.translate(Eigen::Vector3d(0.0, 0.0, -0.5 * t.slot_depth));
          array.markers.push_back(make(
            t.name, Marker::CUBE, pocket, {t.slot_size, t.slot_size, t.slot_depth},
            {0.35f, 0.55f, 0.75f, 0.5f}));
          array.markers.push_back(label(slotName(r, c), t.slotPose(r, c), 0.01));
        }
      }
    }
    for (const auto & b : workspace_.bins) {
      Eigen::Isometry3d box = b.pose();
      box.translation().z() = 0.5 * (b.center.z() + b.size.z());
      array.markers.push_back(make(
        b.name, Marker::CUBE, box, {b.size.x(), b.size.y(), b.size.z() - b.center.z()},
        {0.3f, 0.3f, 0.32f, 0.35f}));
      array.markers.push_back(label(b.name, b.pose(), b.size.z()));
    }
    for (const auto & f : workspace_.fixtures) {
      Eigen::Isometry3d block = f.hole_pose;
      block.translation().z() *= 0.5;
      array.markers.push_back(make(
        f.name, Marker::CUBE, block,
        {f.block_size, f.block_size, f.hole_pose.translation().z()}, {0.6f, 0.6f, 0.62f, 0.6f}));
      array.markers.push_back(label(f.name, f.hole_pose, 0.03));
    }

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/markers", rclcpp::QoS(1).transient_local());
    marker_pub_->publish(array);
  }

  void resolve(const ResolveTarget::Request & request, ResolveTarget::Response & response)
  {
    try {
      const auto pose = workspace_.resolve(request.name);
      response.pose.header.frame_id = workspace_.frame;
      response.pose.header.stamp = now();
      response.pose.pose = tf2::toMsg(pose);
      response.success = true;
    } catch (const std::exception & e) {
      response.success = false;
      response.message = e.what();
    }
  }

  Workspace workspace_;
  std::size_t frame_count_ = 0;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Service<ResolveTarget>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace cell_workspace

RCLCPP_COMPONENTS_REGISTER_NODE(cell_workspace::WorkspaceNode)
