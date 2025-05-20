#include "advance_action_server_client/server/action_server.hpp"
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>

namespace amb
{
  MoveBaseActionServer::MoveBaseActionServer(const rclcpp::NodeOptions &opts)
    : rclcpp::Node("advance_move_base_action_server", opts)
  {
    amb_ac_srv_ = rclcpp_action::create_server<move_base_msgs::action::MoveBase>(
        this,
        "amb_service",
        [this](const auto &uuid, auto goal){ return handleGoal(uuid, goal); },
        [this](const auto gh){ return handleCancel(gh); },
        [this](const auto gh){ handleAccepted(gh); }
      );

    RCLCPP_INFO_STREAM(get_logger(), "Advance move base action server has started!");
  }

  MoveBaseActionServer::~MoveBaseActionServer()
  {
  }

  auto MoveBaseActionServer::handleGoal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal
      ) -> rclcpp_action::GoalResponse
  {
    RCLCPP_INFO_STREAM(get_logger(),
        "Received goal (" << uuidtos(uuid) <<
        ") request with goal:\n" << goal->goal.position.x <<
        ", " << goal->goal.position.y);

    RCLCPP_INFO_STREAM(get_logger(), "Goal validation successful!");
    if (validateGoal(goal))
    {
      RCLCPP_INFO_STREAM(get_logger(), "Goal validation successful!");
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    else
    {
      RCLCPP_WARN(get_logger(), "Goal rejected due to validation failure");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  auto MoveBaseActionServer::handleCancel(
      const std::shared_ptr<const rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> gh
      ) -> rclcpp_action::CancelResponse
  {
    // Signal to execution thread that it should stop
    auto goal_id = uuidtos(gh->get_goal_id());
    RCLCPP_INFO_STREAM(get_logger(), "Received request to cancel goal (" << goal_id << ")!");

    // Set cancellation flag for this goal
    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      if (cancellation_flags_.count(goal_id) > 0) {
        *(cancellation_flags_[goal_id]) = true;
        RCLCPP_INFO_STREAM(get_logger(), "Cancellation flag set for goal " << goal_id);
      }
    }

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  auto MoveBaseActionServer::handleAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> gh
      ) -> void
  {
    auto goal_id = uuidtos(gh->get_goal_id());
    RCLCPP_INFO(this->get_logger(), "Goal accepted: %s", goal_id.c_str());

    // Create cancellation flag for this goal
    std::shared_ptr<std::atomic<bool>> cancellation_flag = 
      std::make_shared<std::atomic<bool>>(false);

    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      cancellation_flags_[goal_id] = cancellation_flag;
    }

    // Start execution in a new thread
    std::shared_ptr<std::thread> execution_thread = std::make_shared<std::thread>(
        std::bind(&MoveBaseActionServer::execute, this, gh, cancellation_flag)
        );

    {
      std::lock_guard<std::mutex> lock(execution_mutex_);
      execution_threads_[goal_id] = execution_thread;
    }
  }

  auto MoveBaseActionServer::execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<move_base_msgs::action::MoveBase>> goal_handler,
      std::shared_ptr<std::atomic<bool>> cancellation_flag
      ) -> void
  {

  }

  auto MoveBaseActionServer::validateGoal(std::shared_ptr<const move_base_msgs::action::MoveBase::Goal> goal) -> bool
  {
    // Simple validation for demonstration purposes
    if (std::isnan(goal->goal.position.x) || 
        std::isnan(goal->goal.position.y) ||
        std::isnan(goal->goal.position.z)) {
      RCLCPP_WARN_STREAM(get_logger(), "Target pose contains NaN values");
      return false;
    }
    
    return true;
  }
} // amb
