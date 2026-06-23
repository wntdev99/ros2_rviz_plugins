// Copyright 2026 Jeongmin Choi
// SPDX-License-Identifier: BSD-3-Clause
//
// NavSinglePanel: an RViz panel that sends goals to the /nav_single action
// (w_behavior_tree_interfaces/action/NavSingle). It subscribes to the goal
// pose published by NavSingleGoalTool and triggers / cancels the action with
// the selected recovery_mode, pass_final_goal_tol, timeout_ms.
//
// Threading: rclcpp subscription / action callbacks run on RViz's ROS spin
// thread, so all UI updates are marshalled to the Qt main thread via signals
// (queued connection). Shared state (last_goal_, goal_handle_) is mutex-guarded.

#ifndef ROS2_RVIZ_PLUGINS__NAV_SINGLE_PANEL_HPP_
#define ROS2_RVIZ_PLUGINS__NAV_SINGLE_PANEL_HPP_

#include <mutex>
#include <string>

#include <QString>
#include <QWidget>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rviz_common/panel.hpp"
#include "w_behavior_tree_interfaces/action/nav_single.hpp"

class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QSpinBox;

namespace rviz_common
{
class Config;
}  // namespace rviz_common

namespace ros2_rviz_plugins
{

class NavSinglePanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  using NavSingle = w_behavior_tree_interfaces::action::NavSingle;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavSingle>;

  explicit NavSinglePanel(QWidget * parent = nullptr);
  ~NavSinglePanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

Q_SIGNALS:
  // Emitted from ROS-thread callbacks; received on the Qt main thread.
  void goalReceived(const QString & text);
  void feedbackReceived(const QString & text);
  void resultReceived(const QString & text);
  void goalResponded(bool accepted);

private Q_SLOTS:
  void onSendClicked();
  void onCancelClicked();
  void onGoalReceived(const QString & text);
  void onFeedbackReceived(const QString & text);
  void onResultReceived(const QString & text);
  void onGoalResponded(bool accepted);

private:
  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void setRunning(bool running);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavSingle>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  std::mutex goal_mutex_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_goal_{false};

  std::mutex handle_mutex_;
  GoalHandle::SharedPtr goal_handle_;

  std::string action_name_{"/nav_single"};
  std::string goal_topic_{"/nav_single/goal_pose"};

  // Widgets
  QLabel * goal_label_{nullptr};
  QComboBox * recovery_combo_{nullptr};
  QDoubleSpinBox * tol_spin_{nullptr};
  QSpinBox * timeout_spin_{nullptr};
  QPushButton * send_button_{nullptr};
  QPushButton * cancel_button_{nullptr};
  QLabel * status_label_{nullptr};
  QLabel * feedback_label_{nullptr};
};

}  // namespace ros2_rviz_plugins

#endif  // ROS2_RVIZ_PLUGINS__NAV_SINGLE_PANEL_HPP_
