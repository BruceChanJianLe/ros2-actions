#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "move_base_msgs/action/move_base.hpp"
#include <memory>
#include <sstream>
#include <ostream>
#include <unordered_map>
#include <thread>
#include <mutex>

namespace amb
{
  class MoveBaseActionServer : public rclcpp::Node
  {
  public:
    explicit MoveBaseActionServer(
        const rclcpp::NodeOptions &opts = rclcpp::NodeOptions{});
    virtual ~MoveBaseActionServer();

    // Handling new goal
    auto handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal
        ) -> rclcpp_action::GoalResponse;

    // Handling goal cancellation
    auto handleCancel(
        const std::shared_ptr<const rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler
        ) -> rclcpp_action::CancelResponse;

    // Handle accepted goal
    auto handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler
        ) -> void;

    auto execute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler,
        std::shared_ptr<std::atomic<bool>> cancellation_flag
        ) ->void;

    inline auto uuidtos(
        const std::array<unsigned char, 16>& uuid
        ) -> std::string
    {
      std::ostringstream oss;
      for (size_t i = 0; i < uuid.size(); ++i)
      {
        oss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(uuid[i]);
        // Optional hyphen formatting
        if (i == 3 || i == 5 || i == 7 || i == 9)
        {
          oss << "-";
        }
      }
      return oss.str();}

  private:
    rclcpp_action::Server<move_base_msgs::action::MoveBase>::SharedPtr amb_ac_srv_;
    std::unordered_map<std::string, std::shared_ptr<std::atomic<bool>>> cancellation_flags_;
    std::unordered_map<std::string, std::shared_ptr<std::thread>> execution_threads_;
    std::mutex execution_mutex_;

    auto validateGoal(std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal) -> bool;
  };

} // amb
