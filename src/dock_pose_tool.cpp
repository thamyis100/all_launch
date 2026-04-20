#include "all_launch/dock_pose_tool.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"

namespace all_launch
{

DockPoseTool::DockPoseTool()
{
  shortcut_key_ = 'd';
  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/dock_pose",
    "Topic to publish dock target pose (PoseStamped).",
    getPropertyContainer());
  QObject::connect(
    topic_property_, &rviz_common::properties::Property::changed,
    this, &DockPoseTool::updateTopic);
}

void DockPoseTool::onInitialize()
{
  rviz_default_plugins::tools::PoseTool::onInitialize();

  setName("2D Dock Pose");
  setDescription("Set a 2D dock target pose and publish it as PoseStamped.");

  auto node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    setStatus("Failed to get ROS node abstraction");
    return;
  }

  node_ = node_abstraction->get_raw_node();
  updateTopic();
}

void DockPoseTool::onPoseSet(double x, double y, double theta)
{
  if (!publisher_ || !node_) {
    setStatus("Publisher not initialized");
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = node_->now();
  pose.header.frame_id = context_->getFixedFrame().toStdString();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientationAroundZAxis(theta);

  publisher_->publish(pose);
  setStatus("Dock pose published");
}

void DockPoseTool::updateTopic()
{
  if (!node_) {
    return;
  }

  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::QoS(10));
}

}  // namespace all_launch

PLUGINLIB_EXPORT_CLASS(all_launch::DockPoseTool, rviz_common::Tool)
