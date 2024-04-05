#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <move_base_msgs/action/move_base.hpp>
#include "move_base/const_defs.hpp"

#include <chrono>
#include <string>

namespace mb
{
  class MoveBaseActionClient : public rclcpp::Node
  {
  public:
    explicit MoveBaseActionClient(const rclcpp::NodeOptions opts = rclcpp::NodeOptions{});
    virtual ~MoveBaseActionClient();

  private:
    rclcpp_action::Client<move_base_msgs::action::MoveBase>::SharedPtr mb_action_clt_;
    rclcpp::TimerBase::SharedPtr timer_;

    // method to send move base goal
    void sendMoveBaseGoal();

    void goalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>);
    void feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>, const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback>);
    void resultCallback(const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult);
  };
} // mb
