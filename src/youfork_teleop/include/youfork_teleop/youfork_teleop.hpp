#ifndef YOUFORK_TELEOP_YOUFORK_TELEOP_HPP_
#define YOUFORK_TELEOP_YOUFORK_TELEOP_HPP_

#include <open_manipulator_msgs/srv/set_joint_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace youfork_teleop
{
class YouforkTeleop : public rclcpp::Node
{
public:
  YouforkTeleop();
};
}  // namespace youfork_teleop

#endif  // YOUFORK_TELEOP_YOUFORK_TELEOP_HPP_
