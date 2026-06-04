#ifndef PB2025_CHASSIS_BRIDGE__CHASSIS_BRIDGE_NODE_HPP_
#define PB2025_CHASSIS_BRIDGE__CHASSIS_BRIDGE_NODE_HPP_

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rmoss_base/uart_transporter.hpp"
#include "rmoss_base/fixed_packet.hpp"
#include "rmoss_base/fixed_packet_tool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rmoss_interfaces/msg/chassis_cmd.hpp"

namespace pb2025_chassis_bridge
{

class ChassisBridgeNode : public rclcpp::Node
{
public:
  explicit ChassisBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
  void chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg);
  void send_loop();
  uint8_t compute_checksum(const rmoss_base::FixedPacket<16> & packet);

  // transport
  std::shared_ptr<rmoss_base::UartTransporter> transporter_;
  std::shared_ptr<rmoss_base::FixedPacketTool<16>> packet_tool_;

  // subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<rmoss_interfaces::msg::ChassisCmd>::SharedPtr chassis_cmd_sub_;

  // timer
  rclcpp::TimerBase::SharedPtr send_timer_;

  // device path for reconnect
  std::string uart_device_;

  // cached target state (thread-safe with single-threaded executor)
  rclcpp::Time last_reconnect_{0};
  float target_vx_{0};
  float target_vy_{0};
  float target_wz_{0};
  uint8_t mode_{0};
};

}  // namespace pb2025_chassis_bridge

#endif  // PB2025_CHASSIS_BRIDGE__CHASSIS_BRIDGE_NODE_HPP_
