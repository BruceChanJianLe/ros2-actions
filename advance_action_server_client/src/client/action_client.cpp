#include "advance_action_server_client/client/action_client.hpp"

namespace amb
{
  MoveBaseActionClient::MoveBaseActionClient(const rclcpp::NodeOptions &opts)
    : rclcpp::Node("advance_move_base_action_client", opts)
  {
    amb_ac_clt_ = rclcpp_action::create_client<move_base_msgs::action::MoveBase>(
        this,
        "amb_service"
        );

    RCLCPP_INFO_STREAM(get_logger(), "Advance move base action client is ready!");
  }

  MoveBaseActionClient::~MoveBaseActionClient()
  {
  }

  auto MoveBaseActionClient::waitForServer() -> void
  {
    // Wait for action server to be available
    if (!amb_ac_clt_->wait_for_action_server())
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Advance move base action server not available after waiting.");
      rclcpp::shutdown();
    }
  }

  auto MoveBaseActionClient::getAllGoalIDs() -> std::vector<std::string>
  {
    std::vector<std::string> goal_ids;

    for (const auto &[goal_id, _] : lookup)
    {
      goal_ids.emplace_back(goal_id);
    }

    return goal_ids;
  }

  auto MoveBaseActionClient::sendGoal(const double x, const double y) -> void
  {
    // Prep goal
    auto goal_msg = move_base_msgs::action::MoveBase::Goal();
    goal_msg.goal.position.x = x;
    goal_msg.goal.position.y = y;

    RCLCPP_INFO_STREAM(get_logger(), "Sending advance move base goal to action server.");

    // Update corresponding feedback
    auto send_goal_options = rclcpp_action::Client<move_base_msgs::action::MoveBase>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      [this](const auto goal_handler){ goalResponseCallback(goal_handler); };
    send_goal_options.feedback_callback = [this](auto goal_handler, const auto fd){ goalFeedbackCallback(goal_handler, fd); };
    send_goal_options.result_callback =
      [this](const auto result){ goalResultCallback(result); };

    // Send request to action server
    amb_ac_clt_->async_send_goal(goal_msg, send_goal_options);
  }

  auto MoveBaseActionClient::cancelGoal(const std::string& goal_id) -> void
  {
    if (auto node = lookup.find(goal_id); !goal_id.empty() && node != lookup.end())
    {
      RCLCPP_INFO_STREAM(get_logger(), "Requesting to cancel (" << goal_id <<
          ") waiting for server response!");
      auto [_, goal_handler] = *node;

      std::thread([this, goal_handler, &goal_id](){
          auto future_cancel = amb_ac_clt_->async_cancel_goal(goal_handler);

          // Wait for cancel response with timeout
          if (future_cancel.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
          {
            RCLCPP_ERROR_STREAM(get_logger(),
                "Failed to cancel goal (" << goal_id << ")");
          }
          else
          {
            RCLCPP_INFO_STREAM(get_logger(),
                "Successfully cancel goal (" << goal_id << ")");
          }

          }).detach();
      lookup.erase(node);
    }
  }

  auto MoveBaseActionClient::goalResponseCallback(
      const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handle
      ) -> void
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (!goal_handle)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Move base goal was rejected!");
    }
    else
    {
      auto goal_id = uuidtos(goal_handle->get_goal_id());
      lookup.emplace(goal_id, goal_handle);
      RCLCPP_INFO_STREAM(get_logger(), "Goal (" << goal_id <<
          ") was accepted, waiting for server result!");
    }
  }

  auto MoveBaseActionClient::goalFeedbackCallback(
      std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handle,
      const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback> feedback
      ) -> void
  {
    RCLCPP_INFO_STREAM(get_logger(), "Goal (" << uuidtos(goal_handle->get_goal_id())
        << ") remaining distance: " << feedback->progress << ".");
  }

  auto MoveBaseActionClient::goalResultCallback(
      const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult goal_handle
      ) -> void
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    auto goal_id = uuidtos(goal_handle.goal_id);

    switch (goal_handle.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO_STREAM(get_logger(), "Goal (" << goal_id << ") succeeded");
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(this->get_logger(), "Goal (" << goal_id << ") was aborted");
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO_STREAM(this->get_logger(), "Goal (" << goal_id << ") was canceled");
        break;

      default:
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown result code");
        break;
    }

    lookup.erase(goal_id);
  }
} // amb
