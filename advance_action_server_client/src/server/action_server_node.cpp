#include <cstdlib>
#include "advance_action_server_client/server/action_server.hpp"

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<amb::MoveBaseActionServer>());

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
