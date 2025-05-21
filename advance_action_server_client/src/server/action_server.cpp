#include "advance_action_server_client/server/action_server.hpp"

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
    RCLCPP_INFO_STREAM(get_logger(), "Goal accepted: " << goal_id);

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
    // Obtain current goal and goal id
    const auto goal = goal_handler->get_goal();
    const auto goal_id = uuidtos(goal_handler->get_goal_id());
    // Place holder for feedback and result
    auto feedback = std::make_shared<move_base_msgs::action::MoveBase::Feedback>();
    auto result = std::make_shared<move_base_msgs::action::MoveBase::Result>();

    RCLCPP_INFO_STREAM(get_logger(), "Executing advance move base goal("
        << goal_id << ")");

    // Check if goal is still active
    if (goal_handler->is_canceling())
    {
      RCLCPP_INFO_STREAM(get_logger(), "Cancelling goal (" << goal_id << ") before processing.");
      result->result_code = move_base_msgs::action::MoveBase::Result::CANCELLED;
      goal_handler->canceled(result);
      stopExecution(goal_id);
      return;
    }

    // Timeout check
    const auto start_time = now();
    const auto feeback_period = rclcpp::Duration::from_seconds(1.0);
    const auto timeout_duration = rclcpp::Duration::from_seconds(60.0);

    rclcpp::Rate loop_rate{1};
    double remaining_distance{100.0};
    // Main work
    while (rclcpp::ok())
    {
      // Handle timeout
      if ((now() - start_time) > timeout_duration)
      {
        RCLCPP_ERROR_STREAM(get_logger(), "Time one for goal ("
            << goal_id << ")");
        result->result_code = move_base_msgs::action::MoveBase::Result::FAILED;
        goal_handler->abort(result);
        stopExecution(goal_id);
        return;
      }

      // Handle cancellation
      if (*cancellation_flag || goal_handler->is_canceling())
      {
        RCLCPP_INFO_STREAM(get_logger(), "Cancelling goal (" <<
            goal_id << ") during processing.");
        result->result_code = move_base_msgs::action::MoveBase::Result::CANCELLED;
        goal_handler->canceled(result);
        stopExecution(goal_id);
        return;
      }

      // Simulating task execution with regular feedback
      remaining_distance -= 5.0;
      feedback->progress = std::max(remaining_distance, 0.0);

      // Publish feedback
      goal_handler->publish_feedback(feedback);
      RCLCPP_INFO_STREAM(get_logger(), "Feedback goal (" << goal_id <<
          "), remaining distance: " << remaining_distance);

      // Mark sucession (not failing in this example)
      if (remaining_distance == 0.0)
      {
        RCLCPP_INFO_STREAM(get_logger(), "Goal (" << goal_id << ") succeeded.");
        result->result_code = move_base_msgs::action::MoveBase::Result::SUCCESS;
        goal_handler->succeed(result);
        stopExecution(goal_id);
        return;
      }

      // Sleep to not halt cpu resource (good practice)
      loop_rate.sleep();
    }
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

  auto MoveBaseActionServer::stopExecution(const std::string& goal_id) -> void
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);

    // Remove the cancellation flag
    if (cancellation_flags_.count(goal_id) > 0)
    {
      cancellation_flags_.erase(goal_id);
    }

    // Detach the thread so it can finish properly
    if (execution_threads_.count(goal_id) > 0)
    {
      if (execution_threads_[goal_id]->joinable())
      {
        execution_threads_[goal_id]->detach();
      }
      execution_threads_.erase(goal_id);
    }
  }

  auto MoveBaseActionServer::stopAllExecutions() -> void
  {
    std::lock_guard<std::mutex> lock(execution_mutex_);

    // Set all cancellation flags
    for (auto& [_, flag] : cancellation_flags_)
    {
      *(flag) = true;
    }

    // Join all threads
    for (auto& [_, thread] : execution_threads_)
    {
      if (thread->joinable())
      {
        thread->join();
      }
    }

    cancellation_flags_.clear();
    execution_threads_.clear();
  }
} // amb
