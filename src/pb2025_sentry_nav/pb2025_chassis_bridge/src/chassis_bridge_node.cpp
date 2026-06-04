#include "pb2025_chassis_bridge/chassis_bridge_node.hpp"

#include <chrono>
#include <cstring>

namespace pb2025_chassis_bridge
{

ChassisBridgeNode::ChassisBridgeNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("chassis_bridge", options)
{
  this->declare_parameter<std::string>("uart_device", "/dev/ttyUSB0");
  this->declare_parameter<int>("baud_rate", 115200);

  uart_device_ = this->get_parameter("uart_device").as_string();
  auto baud_rate = this->get_parameter("baud_rate").as_int();

  transporter_ = std::make_shared<rmoss_base::UartTransporter>(uart_device_, baud_rate);
  transporter_->open();
  if (!transporter_->is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open UART: %s", uart_device_.c_str());
  }

  packet_tool_ = std::make_shared<rmoss_base::FixedPacketTool<16>>(transporter_);

  using namespace std::placeholders;
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&ChassisBridgeNode::cmd_vel_cb, this, _1));

  chassis_cmd_sub_ = this->create_subscription<rmoss_interfaces::msg::ChassisCmd>(
    "robot_base/chassis_cmd", rclcpp::SensorDataQoS(),
    std::bind(&ChassisBridgeNode::chassis_cb, this, _1));

  // 20Hz send loop, matches typical MCU update rate
  send_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&ChassisBridgeNode::send_loop, this));

  RCLCPP_INFO(this->get_logger(), "ChassisBridgeNode started, UART: %s, baud: %d",
    uart_device_.c_str(), baud_rate);
}

void ChassisBridgeNode::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  target_vx_ = static_cast<float>(msg->linear.x);
  target_vy_ = static_cast<float>(msg->linear.y);
  target_wz_ = static_cast<float>(msg->angular.z);
  mode_ = rmoss_interfaces::msg::ChassisCmd::VELOCITY;  // 速度模式
}

void ChassisBridgeNode::chassis_cb(const rmoss_interfaces::msg::ChassisCmd::SharedPtr msg)
{
  target_vx_ = static_cast<float>(msg->twist.linear.x);
  target_vy_ = static_cast<float>(msg->twist.linear.y);
  target_wz_ = static_cast<float>(msg->twist.angular.z);
  mode_ = msg->type;  // VELOCITY / FOLLOW_GIMBAL / SWING / SPIN
}

void ChassisBridgeNode::send_loop()
{
  // 串口断开时每2秒尝试重连
  if (!transporter_->is_open()) {
    auto now = this->now();
    if ((now - last_reconnect_).seconds() > 2.0) {
      last_reconnect_ = now;
      RCLCPP_WARN(this->get_logger(), "UART disconnected, trying reconnect...");
      transporter_->open();
      if (transporter_->is_open()) {
        RCLCPP_INFO(this->get_logger(), "UART reconnected: %s", uart_device_.c_str());
      }
    }
    return;
  }

  rmoss_base::FixedPacket<16> packet;

  // 帧格式:
  // Byte  0:     0xFF (帧头)
  // Byte  1-4:   vx   (float32)
  // Byte  5-8:   vy   (float32)
  // Byte  9-12:  wz   (float32)
  // Byte  13:    mode (uint8: 1=VELOCITY,2=FOLLOW_GIMBAL,3=SWING,4=SPIN)
  // Byte  14:    checksum (XOR of bytes 1-13)
  // Byte  15:    0x0D (帧尾)

  uint8_t offset = 1;
  packet.load_data<float>(target_vx_, offset);       offset += sizeof(float);   // 1-4
  packet.load_data<float>(target_vy_, offset);       offset += sizeof(float);   // 5-8
  packet.load_data<float>(target_wz_, offset);       offset += sizeof(float);   // 9-12
  packet.load_data<uint8_t>(mode_, offset);                                       // 13

  uint8_t ck = compute_checksum(packet);
  packet.set_check_byte(ck);                                                     // 14

  packet_tool_->send_packet(packet);
}

uint8_t ChassisBridgeNode::compute_checksum(const rmoss_base::FixedPacket<16> & packet)
{
  uint8_t ck = 0;
  const uint8_t * buf = packet.buffer();
  for (int i = 1; i < 14; ++i) {  // 数据区: byte 1-13
    ck ^= buf[i];
  }
  return ck;
}

}  // namespace pb2025_chassis_bridge

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pb2025_chassis_bridge::ChassisBridgeNode)
