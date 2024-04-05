#include "move_base/server/move_base_action_server.hpp"

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mb::MoveBaseActionServer>());

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
