// Copyright 2026 Jeongmin Choi
// SPDX-License-Identifier: BSD-3-Clause

#include "ros2_rviz_plugins/nav_single_panel.hpp"

#include <cmath>
#include <memory>
#include <string>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

#include "rviz_common/display_context.hpp"
#include "tf2/utils.h"
// getYaw() 내부의 fromMsg(geometry_msgs::msg::Quaternion, tf2::Quaternion&)
// 정의를 이 번역 단위에 포함시켜 dlopen 시 undefined-symbol 을 방지한다.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace ros2_rviz_plugins
{

NavSinglePanel::NavSinglePanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto * layout = new QVBoxLayout;

  goal_label_ = new QLabel("목표 pose 없음 — 'Nav Single Goal' 도구로 클릭하세요.");
  goal_label_->setWordWrap(true);
  layout->addWidget(goal_label_);

  // --- 주요 NavSingle 파라미터 ---
  auto * form = new QFormLayout;

  recovery_combo_ = new QComboBox;
  recovery_combo_->addItem("자동 (auto)", QString(""));
  recovery_combo_->addItem("stop_and_go", QString("stop_and_go"));
  recovery_combo_->addItem("avoidance", QString("avoidance"));
  form->addRow("recovery_mode", recovery_combo_);

  tol_spin_ = new QDoubleSpinBox;
  tol_spin_->setRange(0.0, 10.0);
  tol_spin_->setSingleStep(0.05);
  tol_spin_->setDecimals(2);
  tol_spin_->setValue(0.0);
  tol_spin_->setToolTip("pass_final_goal_tol (m). 0 이면 비활성(일반 navigation).");
  form->addRow("pass_final_goal_tol", tol_spin_);

  timeout_spin_ = new QSpinBox;
  timeout_spin_->setRange(0, 3600000);
  timeout_spin_->setSingleStep(1000);
  timeout_spin_->setValue(60000);
  timeout_spin_->setSuffix(" ms");
  form->addRow("timeout_ms", timeout_spin_);

  layout->addLayout(form);

  // --- 버튼 ---
  auto * btn_layout = new QHBoxLayout;
  send_button_ = new QPushButton("Send to /nav_single");
  cancel_button_ = new QPushButton("Cancel");
  cancel_button_->setEnabled(false);
  btn_layout->addWidget(send_button_);
  btn_layout->addWidget(cancel_button_);
  layout->addLayout(btn_layout);

  // --- 상태 / 피드백 ---
  status_label_ = new QLabel("대기 중");
  status_label_->setWordWrap(true);
  layout->addWidget(status_label_);

  feedback_label_ = new QLabel("");
  feedback_label_->setWordWrap(true);
  layout->addWidget(feedback_label_);

  layout->addStretch();
  setLayout(layout);

  connect(send_button_, &QPushButton::clicked, this, &NavSinglePanel::onSendClicked);
  connect(cancel_button_, &QPushButton::clicked, this, &NavSinglePanel::onCancelClicked);

  // ROS-thread → Qt main-thread 마샬링 (queued connection)
  connect(this, &NavSinglePanel::goalReceived, this, &NavSinglePanel::onGoalReceived);
  connect(this, &NavSinglePanel::feedbackReceived, this, &NavSinglePanel::onFeedbackReceived);
  connect(this, &NavSinglePanel::resultReceived, this, &NavSinglePanel::onResultReceived);
  connect(this, &NavSinglePanel::goalResponded, this, &NavSinglePanel::onGoalResponded);
}

NavSinglePanel::~NavSinglePanel() = default;

void NavSinglePanel::onInitialize()
{
  node_ = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  action_client_ = rclcpp_action::create_client<NavSingle>(node_, action_name_);

  goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic_, rclcpp::QoS(1),
    std::bind(&NavSinglePanel::goalPoseCallback, this, std::placeholders::_1));
}

void NavSinglePanel::goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    last_goal_ = *msg;
    has_goal_ = true;
  }
  const double yaw = tf2::getYaw(msg->pose.orientation);
  const QString text = QString("목표 수신 [%1]  x=%2  y=%3  yaw=%4 rad")
    .arg(QString::fromStdString(msg->header.frame_id))
    .arg(msg->pose.position.x, 0, 'f', 3)
    .arg(msg->pose.position.y, 0, 'f', 3)
    .arg(yaw, 0, 'f', 3);
  Q_EMIT goalReceived(text);
}

void NavSinglePanel::onSendClicked()
{
  bool has_goal = false;
  geometry_msgs::msg::PoseStamped goal_pose;
  {
    std::lock_guard<std::mutex> lk(goal_mutex_);
    has_goal = has_goal_;
    goal_pose = last_goal_;
  }
  if (!has_goal) {
    status_label_->setText("목표 pose가 없습니다. 먼저 'Nav Single Goal' 도구로 클릭하세요.");
    return;
  }
  if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    status_label_->setText("'/nav_single' 액션 서버를 찾을 수 없습니다 (서버 미가동?).");
    return;
  }

  NavSingle::Goal goal;
  goal.pose = goal_pose;
  goal.recovery_mode = recovery_combo_->currentData().toString().toStdString();
  goal.pass_final_goal_tol = static_cast<float>(tol_spin_->value());
  goal.timeout_ms = static_cast<uint32_t>(timeout_spin_->value());
  // replan_period_ms / goal_checker_name / progress_checker_name 는 서버 기본값 사용.

  rclcpp_action::Client<NavSingle>::SendGoalOptions options;

  options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle) {
      {
        std::lock_guard<std::mutex> lk(handle_mutex_);
        goal_handle_ = handle;
      }
      Q_EMIT goalResponded(static_cast<bool>(handle));
    };

  options.feedback_callback =
    [this](GoalHandle::SharedPtr, const std::shared_ptr<const NavSingle::Feedback> feedback) {
      const QString text = QString("거리 남음: %1 m   |   복구 횟수: %2")
        .arg(feedback->distance_remaining, 0, 'f', 2)
        .arg(feedback->number_of_recoveries);
      Q_EMIT feedbackReceived(text);
    };

  options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      QString text;
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          text = "결과: 성공 (SUCCEEDED)";
          break;
        case rclcpp_action::ResultCode::ABORTED:
          text = QString("결과: 중단 (ABORTED)  error_code=%1  %2")
            .arg(result.result ? result.result->error_code : 0)
            .arg(result.result ? QString::fromStdString(result.result->error_msg) : QString());
          break;
        case rclcpp_action::ResultCode::CANCELED:
          text = "결과: 취소됨 (CANCELED)";
          break;
        default:
          text = "결과: 알 수 없음 (UNKNOWN)";
          break;
      }
      Q_EMIT resultReceived(text);
    };

  action_client_->async_send_goal(goal, options);
  status_label_->setText("목표 전송 중...");
  feedback_label_->clear();
  setRunning(true);
}

void NavSinglePanel::onCancelClicked()
{
  GoalHandle::SharedPtr handle;
  {
    std::lock_guard<std::mutex> lk(handle_mutex_);
    handle = goal_handle_;
  }
  if (handle) {
    action_client_->async_cancel_goal(handle);
    status_label_->setText("취소 요청 전송...");
  } else {
    status_label_->setText("취소할 활성 목표가 없습니다.");
  }
}

void NavSinglePanel::onGoalReceived(const QString & text)
{
  goal_label_->setText(text);
}

void NavSinglePanel::onFeedbackReceived(const QString & text)
{
  feedback_label_->setText(text);
}

void NavSinglePanel::onResultReceived(const QString & text)
{
  status_label_->setText(text);
  setRunning(false);
  {
    std::lock_guard<std::mutex> lk(handle_mutex_);
    goal_handle_.reset();
  }
}

void NavSinglePanel::onGoalResponded(bool accepted)
{
  if (accepted) {
    status_label_->setText("목표 수락됨 — 주행 중...");
  } else {
    status_label_->setText("목표가 서버에 의해 거부되었습니다.");
    setRunning(false);
  }
}

void NavSinglePanel::setRunning(bool running)
{
  send_button_->setEnabled(!running);
  cancel_button_->setEnabled(running);
}

void NavSinglePanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}

void NavSinglePanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

}  // namespace ros2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>  // NOLINT
PLUGINLIB_EXPORT_CLASS(ros2_rviz_plugins::NavSinglePanel, rviz_common::Panel)
