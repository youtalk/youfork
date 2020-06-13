#include "youfork_teleop/youfork_teleop.hpp"

#include <chrono>
#include <memory>

namespace youfork_teleop
{
YouforkTeleop::YouforkTeleop() : Node("youfork_teleop")
{
  using namespace std::placeholders;

  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

  set_actuator_state_client_ =
    create_client<open_manipulator_msgs::srv::SetActuatorState>("set_actuator_state");
  if (!set_actuator_state_client_->wait_for_service()) {
    RCLCPP_FATAL(get_logger(), "Service not found: set_actuator_state");
    rclcpp::shutdown();
  }

  set_joint_position_client_ = create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path_from_present");
  if (!set_joint_position_client_->wait_for_service()) {
    RCLCPP_FATAL(get_logger(), "Service not found: goal_joint_space_path_from_present");
    rclcpp::shutdown();
  }

  previous_time_ = get_clock()->now();
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", qos, std::bind(&YouforkTeleop::joy_callback, this, _1));
}

void YouforkTeleop::joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg)
{
  using namespace std::chrono_literals;

  constexpr double kBaseLinearDelta = 0.1;   // [m/s]
  constexpr double kBaseAngularDelta = 0.2;  // [rad/s]
  constexpr double kArmDelta = 0.1;          // [rad/s]

  rclcpp::Time current_time = get_clock()->now();
  rclcpp::Duration dt = current_time - previous_time_;
  previous_time_ = current_time;

  bool base_enabled = msg->axes[3] < -0.9;
  bool arm_enabled = msg->axes[4] < -0.9;

  if (msg->buttons[9] == 1 || msg->buttons[8] == 1) {
    auto request = std::make_unique<open_manipulator_msgs::srv::SetActuatorState::Request>();
    request->set_actuator_state = msg->buttons[9] == 1;
    RCLCPP_INFO(get_logger(), "set_actuator_state: %d", request->set_actuator_state);

    auto result = set_actuator_state_client_->async_send_request(
      std::move(request),
      [](rclcpp::Client<open_manipulator_msgs::srv::SetActuatorState>::SharedFuture future) {
        return future.get()->is_planned;
      });
  }

  if (base_enabled) {
    auto & clock = *get_clock();
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "base_enabled");

    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    twist->linear.x = msg->axes[1] * kBaseLinearDelta;
    twist->angular.z = msg->axes[0] * kBaseAngularDelta;
    if (twist->linear.x != 0.0 && twist->angular.z != 0.0) {
      RCLCPP_INFO(
        get_logger(), "linear.x: %.3f, angular.z: %.3f", twist->linear.x, twist->angular.z);
    }
    twist_publisher_->publish(std::move(twist));
  }

  if (arm_enabled) {
    auto & clock = *get_clock();
    RCLCPP_INFO_THROTTLE(get_logger(), clock, 1000, "arm_enabled");

    auto set_joint_position_request =
      std::make_unique<open_manipulator_msgs::srv::SetJointPosition::Request>();
    set_joint_position_request->joint_position.joint_name = {"joint1", "joint2", "joint3",
                                                             "joint4"};
    set_joint_position_request->joint_position.position = {0.0, 0.0, 0.0, 0.0};
    if (msg->axes[9] != 0.0) {
      set_joint_position_request->joint_position.position[0] = kArmDelta * dt.seconds();
    }
    if (msg->axes[10] != 0.0) {
      set_joint_position_request->joint_position.position[1] = kArmDelta * dt.seconds();
    }
    if (msg->buttons[0] == 1) {
      set_joint_position_request->joint_position.position[2] = -kArmDelta * dt.seconds();
    } else if (msg->buttons[2] == 1) {
      set_joint_position_request->joint_position.position[2] = kArmDelta * dt.seconds();
    }
    if (msg->buttons[1] == 1) {
      set_joint_position_request->joint_position.position[3] = -kArmDelta * dt.seconds();
    } else if (msg->buttons[3] == 1) {
      set_joint_position_request->joint_position.position[3] = kArmDelta * dt.seconds();
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
