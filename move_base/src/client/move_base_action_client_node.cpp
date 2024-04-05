#include "move_base/client/move_base_action_client.hpp"

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mb::MoveBaseActionClient>());

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
