// Copyright 2026 Jeongmin Choi
// SPDX-License-Identifier: BSD-3-Clause
//
// NavSingleGoalTool: a click tool (PoseTool subclass) that publishes the
// selected goal pose to a private topic (default /nav_single/goal_pose).
// Kept fully separate from nav2's GoalTool (which uses an in-process Qt
// signal, not a topic) and the 2D Goal Pose tool, so clicking this tool
// never triggers /navigate_to_pose etc.

#ifndef ROS2_RVIZ_PLUGINS__NAV_SINGLE_GOAL_TOOL_HPP_
#define ROS2_RVIZ_PLUGINS__NAV_SINGLE_GOAL_TOOL_HPP_

#include <QObject>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_default_plugins/tools/pose/pose_tool.hpp"

namespace rviz_common
{
namespace properties
{
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace ros2_rviz_plugins
{

class NavSingleGoalTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  NavSingleGoalTool();
  ~NavSingleGoalTool() override;

  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

private Q_SLOTS:
  void updateTopic();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rviz_common::properties::StringProperty * topic_property_{nullptr};
};

}  // namespace ros2_rviz_plugins

#endif  // ROS2_RVIZ_PLUGINS__NAV_SINGLE_GOAL_TOOL_HPP_
