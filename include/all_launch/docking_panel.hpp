#ifndef ALL_LAUNCH__DOCKING_PANEL_HPP_
#define ALL_LAUNCH__DOCKING_PANEL_HPP_

#include <QTimer>
#include <QWidget>
#include <atomic>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/config.hpp"
#include "rviz_common/panel.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

class QCheckBox;
class QDoubleSpinBox;
class QHBoxLayout;
class QLabel;
class QLineEdit;
class QPushButton;
class QVBoxLayout;

namespace all_launch
{

class DockingPanel : public rviz_common::Panel
{
public:
  explicit DockingPanel(QWidget * parent = nullptr);
  ~DockingPanel() override = default;

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

private:
  void onDockPressed();
  void onUndockPressed();
  void onUseLastClickedPressed();
  void spinOnce();
  void setupUi();
  void updatePoseFields(const geometry_msgs::msg::PoseStamped & pose);
  void updateStatusTitle(const QString & text, const QString & color);
  void updateStatusDetail(const QString & text);
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q);
  static geometry_msgs::msg::Quaternion quaternionFromYaw(double yaw);

  static std::atomic_uint panel_counter_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr undock_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr docking_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr dock_source_sub_;

  QTimer * spin_timer_{nullptr};

  QLineEdit * frame_edit_{nullptr};
  QDoubleSpinBox * x_spin_{nullptr};
  QDoubleSpinBox * y_spin_{nullptr};
  QDoubleSpinBox * yaw_spin_{nullptr};
  QCheckBox * auto_sync_checkbox_{nullptr};
  QPushButton * use_last_clicked_button_{nullptr};
  QPushButton * dock_button_{nullptr};
  QPushButton * undock_button_{nullptr};
  QLabel * status_title_label_{nullptr};
  QLabel * status_detail_label_{nullptr};
  QLabel * source_value_label_{nullptr};
  QLabel * last_clicked_value_label_{nullptr};

  geometry_msgs::msg::PoseStamped last_clicked_pose_;
  bool have_last_clicked_pose_{false};
  QString last_source_text_{"idle"};
};

}  // namespace all_launch

#endif  // ALL_LAUNCH__DOCKING_PANEL_HPP_
