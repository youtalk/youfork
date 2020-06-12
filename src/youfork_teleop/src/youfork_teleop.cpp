#include "youfork_teleop/youfork_teleop.hpp"

namespace youfork_teleop
{
YouforkTeleop::YouforkTeleop() : Node("youfork_teleop") {}
}  // namespace youfork_teleop

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<youfork_teleop::YouforkTeleop>());
  rclcpp::shutdown();

  return 0;
}
