#include <cstdlib>
#include <thread>
#include "advance_action_server_client/client/action_client.hpp"

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amb::MoveBaseActionClient>();

  std::thread spinner([node](){ rclcpp::spin(node); });

  // Wait for action server to be available
  node->waitForServer();

  node->sendGoal();
  node->sendGoal(1.0, 1.0);
  node->sendGoal(1.0, 1.0);
  node->sendGoal(1.0, 1.0);
  node->sendGoal(1.0, 1.0);
  node->sendGoal(1.0, 1.0);

  std::this_thread::sleep_for(std::chrono::seconds(3));

  auto goal_ids = node->getAllGoalIDs();

  node->cancelGoal(goal_ids[2]);
  node->cancelGoal(goal_ids[3]);

  std::this_thread::sleep_for(std::chrono::seconds(10));

  node->cancelGoal(goal_ids[0]);
  node->cancelGoal(goal_ids[1]);
  node->cancelGoal(goal_ids[4]);

  spinner.join();

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
