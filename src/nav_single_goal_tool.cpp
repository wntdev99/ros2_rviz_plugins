// Copyright 2026 Jeongmin Choi
// SPDX-License-Identifier: BSD-3-Clause

#include "ros2_rviz_plugins/nav_single_goal_tool.hpp"

#include <string>

#include "rviz_common/display_context.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace ros2_rviz_plugins
{

NavSingleGoalTool::NavSingleGoalTool()
: rviz_default_plugins::tools::PoseTool()
{
  // 'n' avoids clashing with nav2 GoalTool ('g') and 2D Goal Pose ('g').
  shortcut_key_ = 'n';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic", "/nav_single/goal_pose",
    "발행할 nav_single 목표 pose 토픽. NavSinglePanel 의 구독 토픽과 일치시킬 것.",
    getPropertyContainer(), SLOT(updateTopic()), this);
}

NavSingleGoalTool::~NavSingleGoalTool() = default;

void NavSingleGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Nav Single Goal");
  setIcon(rviz_common::loadPixmap(
      "package://ros2_rviz_plugins/icons/classes/NavSingleGoalTool.svg"));
  node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
  updateTopic();
}

void NavSingleGoalTool::updateTopic()
{
  if (!node_) {
    return;
  }
  publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::QoS(1));
}

void NavSingleGoalTool::onPoseSet(double x, double y, double theta)
{
  if (!publisher_) {
    return;
  }
  const std::string fixed_frame = context_->getFixedFrame().toStdString();

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = node_->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = orientationAroundZAxis(theta);

  publisher_->publish(pose);
}

}  // namespace ros2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(ros2_rviz_plugins::NavSingleGoalTool, rviz_common::Tool)
