#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <move_base_msgs/action/move_base.hpp>
#include "move_base/const_defs.hpp"

namespace mb
{
  class MoveBaseActionServer : public rclcpp::Node
  {
  public:
    explicit MoveBaseActionServer(const rclcpp::NodeOptions &opts = rclcpp::NodeOptions{});
    virtual ~MoveBaseActionServer();

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal
        );
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<const rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler
        );
    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler);

  private:
    rclcpp_action::Server<move_base_msgs::action::MoveBase>::SharedPtr mb_action_srv_;
  };
} // mb
