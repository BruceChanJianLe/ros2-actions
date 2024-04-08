#include "rqt_move_base_client/rqt_move_base_client.hpp"
#include <move_base_msgs/action/detail/move_base__struct.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <QStringList>
#include <qnamespace.h>
#include <QValidator>

PLUGINLIB_EXPORT_CLASS(rqt_plugin::RQTMoveBaseClient, rqt_gui_cpp::Plugin)

namespace rqt_plugin
{

  CustomWidget::CustomWidget(rclcpp::Node::SharedPtr node)
    : node_{node}
    , goal_{}
    , is_srv_online_{false}
  {
    // Extend the widget with all attributes and children from UI file
    ui_.setupUi(this);

    // Setup ui
    auto dv = new QDoubleValidator();
    ui_.lineEditGoalX->setValidator(dv);
    ui_.lineEditGoalY->setValidator(dv);

    // Get save value if available

    // ROS2 related declaration
    mb_action_clt_ = rclcpp_action::create_client<move_base_msgs::action::MoveBase>(node_, MB_SERVER_NAME);
    // Start short observation timer
    timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]{ waitForMoveBaseServer(); });

    // Update action client callbacks
    send_goal_options_ = rclcpp_action::Client<move_base_msgs::action::MoveBase>::SendGoalOptions();
    send_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handler){ goalResponseCallback(goal_handler); };
    send_goal_options_.feedback_callback =
      [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>goal_handler, const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback> fd){ feedbackCallback(goal_handler, fd); };
    send_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult result){ resultCallback(result); };
  }

  CustomWidget::~CustomWidget() {}

  void CustomWidget::on_lineEditGoalX_editingFinished()
  {
    goal_.position.x = ui_.lineEditGoalX->text().toDouble();
  }

  void CustomWidget::on_lineEditGoalY_editingFinished()
  {
    goal_.position.y = ui_.lineEditGoalY->text().toDouble();
  }

  void CustomWidget::on_pushButtonSendMoveBaseGoal_clicked()
  {
    // Wait for action server to be available
    if (!mb_action_clt_->wait_for_action_server(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Move base action server not available, try calling later.");
      ui_.labelActionStatus->setText(QString::fromStdString(std::string{STATUS_MSG} + std::string{"offline"}));
      // start timer to observe mb action server
      is_srv_online_ = false;
      // Start short observation timer
      timer_ = node_->create_wall_timer(std::chrono::seconds(1), [this]{ waitForMoveBaseServer(); });
    }
    else
    {
      move_base_msgs::action::MoveBase::Goal goal_msg;
      goal_msg.goal = goal_;
      mb_action_clt_->async_send_goal(goal_msg, send_goal_options_);
    }
  }

  void CustomWidget::waitForMoveBaseServer(const bool verify)
  {
    if (!is_srv_online_ || verify)
    {
      ui_.labelActionStatus->setText(QString::fromStdString(std::string{STATUS_MSG} + std::string{"offline"}));
      RCLCPP_INFO_STREAM(node_->get_logger(), "Waiting for move base action server not available after waiting.");
      if (!mb_action_clt_->wait_for_action_server(std::chrono::seconds(2)))
      {
        RCLCPP_WARN_STREAM(node_->get_logger(), "Move base action server not available at the moment, retrying.");
      }
      else
      {
        RCLCPP_INFO_STREAM(node_->get_logger(), "Move base action server is running.");
        is_srv_online_ = true;
      ui_.labelActionStatus->setText(QString::fromStdString(std::string{STATUS_MSG} + std::string{"online"}));
      }
    }
    else
    {
      // Stop timer
      timer_->cancel();
      // Start long observation timer
      timer_ = node_->create_wall_timer(std::chrono::seconds(10), [this]{ waitForMoveBaseServer(true); });
    }
  }

  void CustomWidget::on_pushButtonCancelMoveBaseGoal_clicked()
  {
    mb_action_clt_->async_cancel_all_goals();
  }

  void CustomWidget::goalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handler)
  {
    if (!goal_handler)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Move base goal was rejected!");
    }
    else
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Move base goal was accepted, waiting for server result!");
    }
  }

  void CustomWidget::feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>> goal_handler, const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback> feedback)
  {
    ui_.labelActionStatus->setText(QString::fromStdString(std::string{STATUS_MSG} + std::string{"running (" + std::to_string(feedback->progress) + "%"}));
  }

  void CustomWidget::resultCallback(const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Move base goal was aborted.");
        break;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Move base goal was cancelled.");
        break;

      default:
        RCLCPP_ERROR_STREAM(node_->get_logger(), "Unknown result code!");
        break;
    }

    RCLCPP_INFO_STREAM(node_->get_logger(), "Finished move base action with " << result.result->result_code << " result code.");
    ui_.labelActionStatus->setText(QString::fromStdString(std::string{STATUS_MSG} + std::string{"online"}));
  }

  RQTMoveBaseClient::RQTMoveBaseClient()
    : rqt_gui_cpp::Plugin()
    , widget_(0)
  {
  }

  void RQTMoveBaseClient::initPlugin(qt_gui_cpp::PluginContext& context)
  {
    // Access standalone command line arguments
    QStringList argv = context.argv();

    // Create custom QWidget
    widget_ = new CustomWidget(node_);

    // add widget to the user interface
    context.addWidget(widget_);
  }

  RQTMoveBaseClient::~RQTMoveBaseClient()
  {
    if(widget_)
    {
      delete widget_;
    }
  }

  void RQTMoveBaseClient::shutdownPlugin()
  {
    ;
  }

  void RQTMoveBaseClient::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
  {
    ;
  }

  void RQTMoveBaseClient::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
  {
    ;
  }

} // namespace rqt_plugin
