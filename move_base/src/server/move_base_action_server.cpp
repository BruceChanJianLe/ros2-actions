#include "move_base/server/move_base_action_server.hpp"

namespace mb
{
  MoveBaseActionServer::MoveBaseActionServer(const rclcpp::NodeOptions &opts)
    : rclcpp::Node("move_base_action_server", opts)
  {
    mb_action_srv_ = rclcpp_action::create_server<move_base_msgs::action::MoveBase>(
        this,
        MB_SERVER_NAME,
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal
        ){ return this->handleGoal(uuid, goal); },
        [this](const std::shared_ptr<const rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler){ return this->handleCancel(goal_handler); },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler){ this->handleAccepted(goal_handler); }
        );

    RCLCPP_INFO_STREAM(get_logger(), "Move base action server started!");
  }

  MoveBaseActionServer::~MoveBaseActionServer()
  {
  }

  rclcpp_action::GoalResponse MoveBaseActionServer::handleGoal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal
      )
  {
    RCLCPP_INFO_STREAM(get_logger(), "Received goal request with goal:\n" << goal->goal.position.x << ", " << goal->goal.position.y);
    (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse MoveBaseActionServer::handleCancel(
      const std::shared_ptr<const rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler
      )
  {
    RCLCPP_INFO_STREAM(get_logger(), "Received request to cancel goal!");
    // (void)goal_handler;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void MoveBaseActionServer::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler)
  {
    std::thread{[this, goal_handler]{ execute(goal_handler); }}.detach();
  }

  void MoveBaseActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Executing move base goal...");
    rclcpp::Rate loop_rate{1};

    const auto goal = goal_handler->get_goal();
    // Get feedback placeholder
    auto feedback = std::make_shared<move_base_msgs::action::MoveBase::Feedback>();
    auto &progress = feedback->progress;
    progress = 0.0;
    // Get result placeholder
    auto result = std::make_shared<move_base_msgs::action::MoveBase::Result>();

    for (int i = 0; (i <= 10) && rclcpp::ok(); ++i)
    {
      // Check if there is a cancel request
      if (goal_handler->is_canceling())
      {
        goal_handler->canceled(result);
        RCLCPP_INFO_STREAM(get_logger(), "Move base goal cancel at " << progress);
        break;
      }

      // Update progress
      result->result_code = i;
      progress = i * 10.0;
      // Publisher progress
      goal_handler->publish_feedback(feedback);
      RCLCPP_INFO_STREAM(get_logger(), "Published move base feedback " << progress << "%");

      loop_rate.sleep();
    }

    if (rclcpp::ok())
    {
      if (result->result_code == 10)
      {
        goal_handler->succeed(result);
        RCLCPP_INFO_STREAM(get_logger(), "Move base goal succeeded.");
      }
    }
  }
} // mb

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mb::MoveBaseActionServer)
