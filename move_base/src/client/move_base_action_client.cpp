#include "move_base/client/move_base_action_client.hpp"

namespace mb
{
  MoveBaseActionClient::MoveBaseActionClient(const rclcpp::NodeOptions opts)
    : rclcpp::Node("move_base_action_client", opts)
  {
    mb_action_clt_ = rclcpp_action::create_client<move_base_msgs::action::MoveBase>(this, MB_SERVER_NAME);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]{ sendMoveBaseGoal(); }
        );

    RCLCPP_INFO_STREAM(get_logger(), "Move base action client started!");
  }

  MoveBaseActionClient::~MoveBaseActionClient()
  {
  }

  void MoveBaseActionClient::sendMoveBaseGoal()
  {
    // Timer will call itself
    timer_->cancel();

    // Wait for action server to be available
    if (!mb_action_clt_->wait_for_action_server())
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Move base action server not available after waiting.");
      rclcpp::shutdown();
    }

    // Prep goal
    auto goal_msg = move_base_msgs::action::MoveBase::Goal();
    goal_msg.goal.position.x = 1.0;
    goal_msg.goal.position.y = 1.0;

    RCLCPP_INFO_STREAM(get_logger(), "Sending move base goal to action server.");

    // Update corresponding feedback
    auto send_goal_options = rclcpp_action::Client<move_base_msgs::action::MoveBase>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handler){ goalResponseCallback(goal_handler); };
    send_goal_options.feedback_callback =
      [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>goal_handler, const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback> fd){ feedbackCallback(goal_handler, fd); };
    send_goal_options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult result){ resultCallback(result); };

    // Send request to action server
    mb_action_clt_->async_send_goal(goal_msg, send_goal_options);
  }

  void MoveBaseActionClient::goalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handler)
  {
    if (!goal_handler)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Move base goal was rejected!");
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "Move base goal was accepted, waiting for server result!");
    }
  }

  void MoveBaseActionClient::feedbackCallback([[maybe_unused]] std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handler, const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback> feedback)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Move Base Progress: " << feedback->progress << "%");
  }

  void MoveBaseActionClient::resultCallback(const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(get_logger(), "Move base goal was aborted.");
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(get_logger(), "Move base goal was cancelled.");
        break;

      default:
        RCLCPP_ERROR_STREAM(get_logger(), "Unknown result code!");
        break;
    }

    RCLCPP_INFO_STREAM(get_logger(), "Finished move base action with " << result.result->result_code << " result code.");
    rclcpp::shutdown();
  }
} // mb

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mb::MoveBaseActionClient)
