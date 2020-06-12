#include "youfork_teleop/youfork_teleop.hpp"

#include "geometry_msgs/msg/twist__struct.hpp"
#include "open_manipulator_msgs/srv/set_joint_position__struct.hpp"
#include "rclcpp/client.hpp"

namespace youfork_teleop
{
YouforkTeleop::YouforkTeleop() : Node("youfork_teleop")
{
  using namespace std::placeholders;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
  set_joint_position_client_ = create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path_from_present");
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", qos, std::bind(&YouforkTeleop::joy_callback, this, _1));
}

void YouforkTeleop::joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg)
{
  constexpr double kBaseLinearDelta = 0.1;
  constexpr double kBaseAngularDelta = 0.2;
  constexpr double kArmDelta = 0.1;
  bool base_enabled = msg->axes[3] < -0.9;
  bool arm_enabled = msg->axes[4] < -0.9;

  if (base_enabled) {
    geometry_msgs::msg::Twist::UniquePtr twist;

    twist->linear.x = msg->axes[1] * kBaseLinearDelta;
    twist->angular.z = msg->axes[0] * kBaseAngularDelta;
    twist_publisher_->publish(std::move(twist));
  }

  if (arm_enabled) {
    open_manipulator_msgs::srv::SetJointPosition::Request::UniquePtr set_joint_position_request;
    set_joint_position_request->joint_position.joint_name = {"joint1", "joint2", "joint3",
                                                             "joint4"};
    set_joint_position_request->joint_position.position = {0.0, 0.0, 0.0, 0.0};

    if (msg->axes[9] != 0.0) {
      set_joint_position_request->joint_position.position[0] = kArmDelta;
    }
    if (msg->axes[10] != 0.0) {
      set_joint_position_request->joint_position.position[1] = kArmDelta;
    }
    if (msg->buttons[0] != 0.0) {
      set_joint_position_request->joint_position.position[2] = -kArmDelta;
    } else if (msg->buttons[2] != 0.0) {
      set_joint_position_request->joint_position.position[2] = kArmDelta;
    }
    if (msg->buttons[1] != 0.0) {
      set_joint_position_request->joint_position.position[3] = -kArmDelta;
    } else if (msg->buttons[3] != 0.0) {
      set_joint_position_request->joint_position.position[3] = kArmDelta;
    }
    auto result = set_joint_position_client_->async_send_request(
      std::move(set_joint_position_request),
      [](rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future) {
        return future.get()->is_planned;
      });
  }
}

}  // namespace youfork_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<youfork_teleop::YouforkTeleop>());
  rclcpp::shutdown();

  return 0;
}
