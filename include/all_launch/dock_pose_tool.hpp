#ifndef ALL_LAUNCH__DOCK_POSE_TOOL_HPP_
#define ALL_LAUNCH__DOCK_POSE_TOOL_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace all_launch
{

class DockPoseTool : public rviz_default_plugins::tools::PoseTool
{
public:
  DockPoseTool();
  ~DockPoseTool() override = default;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private:
  void updateTopic();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rviz_common::properties::StringProperty * topic_property_{nullptr};
};

}  // namespace all_launch

#endif  // ALL_LAUNCH__DOCK_POSE_TOOL_HPP_
