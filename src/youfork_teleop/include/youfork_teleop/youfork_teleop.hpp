#ifndef YOUFORK_TELEOP_YOUFORK_TELEOP_HPP_
#define YOUFORK_TELEOP_YOUFORK_TELEOP_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace youfork_teleop
{
class YouforkTeleop : public rclcpp::Node
{
public:
  YouforkTeleop();
  ~YouforkTeleop() = default;

private:
  void joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{nullptr};
  rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr
    set_joint_position_client_{nullptr};
  rclcpp::Clock clock{RCL_ROS_TIME};
  rclcpp::Time previous_time_{clock.now()};
};
}  // namespace youfork_teleop

#endif  // YOUFORK_TELEOP_YOUFORK_TELEOP_HPP_
