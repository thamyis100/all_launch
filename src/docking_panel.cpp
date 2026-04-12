#include "all_launch/docking_panel.hpp"

#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStringList>
#include <QVBoxLayout>

#include <cmath>
#include <string>

#include "pluginlib/class_list_macros.hpp"

namespace all_launch
{

std::atomic_uint DockingPanel::panel_counter_{0};

DockingPanel::DockingPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  setupUi();

  const auto panel_id = panel_counter_.fetch_add(1);
  node_ = std::make_shared<rclcpp::Node>("rviz_docking_panel_" + std::to_string(panel_id));

  dock_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/dock_pose", 10);
  undock_pub_ = node_->create_publisher<std_msgs::msg::Empty>("/undock_trigger", 10);

  dock_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/dock_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      last_clicked_pose_ = *msg;
      have_last_clicked_pose_ = true;
      if (auto_sync_checkbox_->isChecked()) {
        updatePoseFields(*msg);
      }

      const double yaw_deg = yawFromQuaternion(msg->pose.orientation) * 180.0 / M_PI;
      last_clicked_value_label_->setText(
        QString("%1 | x=%2  y=%3  yaw=%4 deg")
        .arg(QString::fromStdString(msg->header.frame_id.empty() ? "map" : msg->header.frame_id))
        .arg(msg->pose.position.x, 0, 'f', 2)
        .arg(msg->pose.position.y, 0, 'f', 2)
        .arg(yaw_deg, 0, 'f', 1));
    });

  docking_status_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/docking/status_text", rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr msg) {
      const QString full_text = QString::fromStdString(msg->data);
      const QStringList lines = full_text.split('\n');
      const QString title = lines.isEmpty() ? QString("Docking: IDLE") : lines[0];
      const QString detail = lines.size() > 1 ? lines.mid(1).join("\n") : QString();

      QString color = "#bdbdbd";
      if (title.contains("SUCCESS")) {
        color = "#4caf50";
      } else if (title.contains("FAILED") || title.contains("REJECTED") || title.contains("MISSING")) {
        color = "#ef5350";
      } else if (title.contains("CONTROLLING") || title.contains("INITIAL_PERCEPTION") ||
        title.contains("NAV_TO_STAGING") || title.contains("WAIT_FOR_CHARGE"))
      {
        color = "#ffd54f";
      } else if (title.contains("ACCEPTED") || title.contains("REQUESTED")) {
        color = "#4fc3f7";
      }

      updateStatusTitle(title, color);
      updateStatusDetail(detail);
    });

  dock_source_sub_ = node_->create_subscription<std_msgs::msg::String>(
    "/detected_dock_pose_source", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      last_source_text_ = QString::fromStdString(msg->data);
      source_value_label_->setText(last_source_text_);
    });

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, &QTimer::timeout, this, &DockingPanel::spinOnce);
  spin_timer_->start(50);
}

void DockingPanel::onInitialize()
{
}

void DockingPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString frame{"map"};
  float x{0.0F};
  float y{0.0F};
  float yaw_deg{0.0F};
  bool auto_sync{true};

  config.mapGetString("DockFrame", &frame);
  config.mapGetFloat("DockX", &x);
  config.mapGetFloat("DockY", &y);
  config.mapGetFloat("DockYawDeg", &yaw_deg);
  config.mapGetBool("AutoSync", &auto_sync);

  frame_edit_->setText(frame);
  x_spin_->setValue(x);
  y_spin_->setValue(y);
  yaw_spin_->setValue(yaw_deg);
  auto_sync_checkbox_->setChecked(auto_sync);
}

void DockingPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("DockFrame", frame_edit_->text());
  config.mapSetValue("DockX", x_spin_->value());
  config.mapSetValue("DockY", y_spin_->value());
  config.mapSetValue("DockYawDeg", yaw_spin_->value());
  config.mapSetValue("AutoSync", auto_sync_checkbox_->isChecked());
}

void DockingPanel::onDockPressed()
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = frame_edit_->text().trimmed().isEmpty() ?
    std::string("map") : frame_edit_->text().trimmed().toStdString();
  msg.pose.position.x = x_spin_->value();
  msg.pose.position.y = y_spin_->value();
  msg.pose.position.z = 0.0;
  msg.pose.orientation = quaternionFromYaw(yaw_spin_->value() * M_PI / 180.0);
  dock_pose_pub_->publish(msg);
}

void DockingPanel::onUndockPressed()
{
  std_msgs::msg::Empty msg;
  undock_pub_->publish(msg);
}

void DockingPanel::onUseLastClickedPressed()
{
  if (have_last_clicked_pose_) {
    updatePoseFields(last_clicked_pose_);
  }
}

void DockingPanel::spinOnce()
{
  if (rclcpp::ok()) {
    rclcpp::spin_some(node_);
  }
}

void DockingPanel::setupUi()
{
  auto * root_layout = new QVBoxLayout;
  root_layout->setContentsMargins(8, 8, 8, 8);
  root_layout->setSpacing(8);

  auto * title = new QLabel("Docking");
  title->setStyleSheet("font-weight: 700; font-size: 16px;");
  root_layout->addWidget(title);

  auto * command_group = new QGroupBox("Commands");
  auto * command_layout = new QVBoxLayout;
  auto * button_row = new QHBoxLayout;
  dock_button_ = new QPushButton("Dock");
  undock_button_ = new QPushButton("Undock");
  dock_button_->setStyleSheet("font-weight: 700; min-height: 32px;");
  undock_button_->setStyleSheet("font-weight: 700; min-height: 32px;");
  button_row->addWidget(dock_button_);
  button_row->addWidget(undock_button_);
  command_layout->addLayout(button_row);
  command_group->setLayout(command_layout);
  root_layout->addWidget(command_group);

  auto * pose_group = new QGroupBox("Dock Pose");
  auto * pose_layout = new QVBoxLayout;
  auto * form_layout = new QFormLayout;

  frame_edit_ = new QLineEdit("map");
  form_layout->addRow("Frame", frame_edit_);

  x_spin_ = new QDoubleSpinBox;
  x_spin_->setRange(-1000.0, 1000.0);
  x_spin_->setDecimals(3);
  x_spin_->setSingleStep(0.05);
  form_layout->addRow("X (m)", x_spin_);

  y_spin_ = new QDoubleSpinBox;
  y_spin_->setRange(-1000.0, 1000.0);
  y_spin_->setDecimals(3);
  y_spin_->setSingleStep(0.05);
  form_layout->addRow("Y (m)", y_spin_);

  yaw_spin_ = new QDoubleSpinBox;
  yaw_spin_->setRange(-180.0, 180.0);
  yaw_spin_->setDecimals(1);
  yaw_spin_->setSingleStep(5.0);
  form_layout->addRow("Yaw (deg)", yaw_spin_);

  pose_layout->addLayout(form_layout);

  auto_sync_checkbox_ = new QCheckBox("Auto-sync from /dock_pose");
  auto_sync_checkbox_->setChecked(true);
  pose_layout->addWidget(auto_sync_checkbox_);

  use_last_clicked_button_ = new QPushButton("Use Last Clicked Dock Pose");
  pose_layout->addWidget(use_last_clicked_button_);

  last_clicked_value_label_ = new QLabel("No /dock_pose received yet");
  last_clicked_value_label_->setWordWrap(true);
  last_clicked_value_label_->setStyleSheet("color: #b0bec5;");
  pose_layout->addWidget(last_clicked_value_label_);

  pose_group->setLayout(pose_layout);
  root_layout->addWidget(pose_group);

  auto * status_group = new QGroupBox("Status");
  auto * status_layout = new QVBoxLayout;
  status_title_label_ = new QLabel("Docking: IDLE");
  status_title_label_->setStyleSheet(
    "font-weight: 700; color: #bdbdbd; padding: 4px 6px; background: #263238; border-radius: 4px;");
  status_detail_label_ = new QLabel("Awaiting /dock_pose");
  status_detail_label_->setWordWrap(true);
  status_detail_label_->setStyleSheet("color: #eceff1;");
  source_value_label_ = new QLabel("idle");
  source_value_label_->setStyleSheet("color: #4fc3f7;");

  auto * source_row = new QHBoxLayout;
  auto * source_label = new QLabel("Pose Source");
  source_label->setStyleSheet("font-weight: 600;");
  source_row->addWidget(source_label);
  source_row->addStretch();
  source_row->addWidget(source_value_label_);

  status_layout->addWidget(status_title_label_);
  status_layout->addWidget(status_detail_label_);
  status_layout->addLayout(source_row);
  status_group->setLayout(status_layout);
  root_layout->addWidget(status_group);
  root_layout->addStretch();

  setLayout(root_layout);

  connect(dock_button_, &QPushButton::clicked, this, &DockingPanel::onDockPressed);
  connect(undock_button_, &QPushButton::clicked, this, &DockingPanel::onUndockPressed);
  connect(
    use_last_clicked_button_, &QPushButton::clicked, this, &DockingPanel::onUseLastClickedPressed);
}

void DockingPanel::updatePoseFields(const geometry_msgs::msg::PoseStamped & pose)
{
  frame_edit_->setText(QString::fromStdString(pose.header.frame_id.empty() ? "map" : pose.header.frame_id));
  x_spin_->setValue(pose.pose.position.x);
  y_spin_->setValue(pose.pose.position.y);
  yaw_spin_->setValue(yawFromQuaternion(pose.pose.orientation) * 180.0 / M_PI);
}

void DockingPanel::updateStatusTitle(const QString & text, const QString & color)
{
  status_title_label_->setText(text);
  status_title_label_->setStyleSheet(QString(
    "font-weight: 700; color: %1; padding: 4px 6px; background: #263238; border-radius: 4px;")
    .arg(color));
}

void DockingPanel::updateStatusDetail(const QString & text)
{
  status_detail_label_->setText(text.isEmpty() ? QString("No feedback yet") : text);
}

double DockingPanel::yawFromQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion DockingPanel::quaternionFromYaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

}  // namespace all_launch

PLUGINLIB_EXPORT_CLASS(all_launch::DockingPanel, rviz_common::Panel)
