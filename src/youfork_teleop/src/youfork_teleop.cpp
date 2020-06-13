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

  set_arm_position_client_ = create_client<open_manipulator_msgs::srv::SetJointPosition>(
    "goal_joint_space_path_from_present");
  if (!set_arm_position_client_->wait_for_service()) {
    RCLCPP_FATAL(get_logger(), "Service not found: goal_joint_space_path_from_present");
    rclcpp::shutdown();
  }

  set_gripper_position_client_ =
    create_client<open_manipulator_msgs::srv::SetJointPosition>("goal_tool_control");
  if (!set_gripper_position_client_->wait_for_service()) {
    RCLCPP_FATAL(get_logger(), "Service not found: goal_tool_control");
    rclcpp::shutdown();
  }

  previous_time_ = get_clock()->now();
  joy_subscription_ = create_subscription<sensor_msgs::msg::Joy>(
    "joy", qos, std::bind(&YouforkTeleop::joy_callback, this, _1));
}

void YouforkTeleop::joy_callback(const sensor_msgs::msg::Joy::UniquePtr msg)
{
  using namespace std::chrono_literals;

  constexpr double kBaseLinearDelta = 0.2;   // [m/s]
  constexpr double kBaseAngularDelta = 1.0;  // [rad/s]
  constexpr double kArmDelta = 0.5;          // [rad/s]
  constexpr double kGripperDelta = 0.2;      // [rad/s]

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
      [this](rclcpp::Client<open_manipulator_msgs::srv::SetActuatorState>::SharedFuture future) {
        if (!future.get()->is_planned) {
          RCLCPP_WARN(get_logger(), "set_actuator_state: failure");
        }
        return future.get()->is_planned;
      });
  }

  if (base_enabled) {
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
    {
      auto request = std::make_unique<open_manipulator_msgs::srv::SetJointPosition::Request>();
      request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4"};
      request->joint_position.position = {0.0, 0.0, 0.0, 0.0};
      request->path_time = dt.seconds();
      if (msg->axes[9] != 0.0) {
        request->joint_position.position[0] = msg->axes[9] * kArmDelta * dt.seconds();
      }
      if (msg->axes[10] != 0.0) {
        request->joint_position.position[1] = -msg->axes[10] * kArmDelta * dt.seconds();
      }
      if (msg->buttons[0] == 1) {
        request->joint_position.position[2] = kArmDelta * dt.seconds();
      } else if (msg->buttons[2] == 1) {
        request->joint_position.position[2] = -kArmDelta * dt.seconds();
      }
      if (msg->buttons[1] == 1) {
        request->joint_position.position[3] = kArmDelta * dt.seconds();
      } else if (msg->buttons[3] == 1) {
        request->joint_position.position[3] = -kArmDelta * dt.seconds();
      }
      if (std::any_of(
            request->joint_position.position.begin(), request->joint_position.position.end(),
            [](double p) { return p != 0.0; })) {
        RCLCPP_INFO(
          get_logger(), "j1: %.3f, j2: %.3f, j3: %.3f, j4: %.3f",
          request->joint_position.position[0], request->joint_position.position[1],
          request->joint_position.position[2], request->joint_position.position[3]);
        auto result = set_arm_position_client_->async_send_request(
          std::move(request),
          [this](
            rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future) {
            if (!future.get()->is_planned) {
              RCLCPP_WARN(get_logger(), "set_arm_position: failure");
            }
            return future.get()->is_planned;
          });
      }
    }
    {
      auto request = std::make_unique<open_manipulator_msgs::srv::SetJointPosition::Request>();
      request->joint_position.joint_name = {"gripper"};
      request->joint_position.position = {0.0};
      request->path_time = dt.seconds();
      if (msg->buttons[10] == 1) {
        request->joint_position.position[0] = kGripperDelta * dt.seconds();
      } else if (msg->buttons[11] == 1) {
        request->joint_position.position[0] = -kGripperDelta * dt.seconds();
      }
      if (std::any_of(
            request->joint_position.position.begin(), request->joint_position.position.end(),
            [](double p) { return p != 0.0; })) {
        RCLCPP_INFO(get_logger(), "gripper: %.3f", request->joint_position.position[0]);
        auto result = set_gripper_position_client_->async_send_request(
          std::move(request),
          [this](
            rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedFuture future) {
            if (!future.get()->is_planned) {
              RCLCPP_WARN(get_logger(), "set_gripper_position: failure");
            }
            return future.get()->is_planned;
          });
      }
    }
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
