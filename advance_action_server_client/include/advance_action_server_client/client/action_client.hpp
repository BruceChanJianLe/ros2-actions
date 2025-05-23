#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "move_base_msgs/action/move_base.hpp"
#include <limits>
#include <string>
#include <vector>

namespace amb
{
  class MoveBaseActionClient : public rclcpp::Node
  {
  public:
    explicit MoveBaseActionClient(
        const rclcpp::NodeOptions &opts = rclcpp::NodeOptions{}
        );
    virtual ~MoveBaseActionClient();

    auto waitForServer() -> void;
    auto getAllGoalIDs() -> std::vector<std::string>;

    auto sendGoal(
        const double x = std::numeric_limits<double>::quiet_NaN(),
        const double y = std::numeric_limits<double>::quiet_NaN()
        ) -> void;

    auto cancelGoal(const std::string& goal_id) -> void;

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
      return oss.str();
    }

  private:
    rclcpp_action::Client<move_base_msgs::action::MoveBase>::SharedPtr amb_ac_clt_;
    std::mutex goal_mutex_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>
> lookup;

    auto goalResponseCallback(
        const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>
        ) -> void;

    auto goalFeedbackCallback(
        std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>,
        const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback>
        ) -> void;

    auto goalResultCallback(
        const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult
        ) -> void;
  };
} // amb
