#ifndef PUSH_BUTTON_H__
#define PUSH_BUTTON_H__

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
// msgs
#include <move_base_msgs/action/move_base.hpp>
#include <move_base/client/move_base_action_client.hpp>
// Rqt
#include <rqt_gui_cpp/plugin.h>
// Qt
#include <QWidget>
// Custom UI
#include <rqt_move_base_client/ui_rqt_move_base_client.h>

namespace rqt_plugin
{

  static constexpr const char* STATUS_MSG {"Move Base Action Server: "};

  // To use autoconnect signals and slots
  class CustomWidget : public QWidget
  {
  Q_OBJECT
  public:
    explicit CustomWidget(rclcpp::Node::SharedPtr node);
    virtual ~CustomWidget();
    void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

  private slots:
      void on_lineEditGoalX_editingFinished();
      void on_lineEditGoalY_editingFinished();
      void on_pushButtonSendMoveBaseGoal_clicked();
      void on_pushButtonCancelMoveBaseGoal_clicked();

  protected:
      Ui::rqt_move_base_client ui_;
      rclcpp::Node::SharedPtr node_;
      geometry_msgs::msg::Pose goal_;
      bool is_srv_online_;
      rclcpp_action::Client<move_base_msgs::action::MoveBase>::SharedPtr mb_action_clt_;
      rclcpp::TimerBase::SharedPtr timer_;
      rclcpp_action::Client<move_base_msgs::action::MoveBase>::SendGoalOptions send_goal_options_;

      void waitForMoveBaseServer(const bool verify = false);
      void goalResponseCallback(const std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>);
      void feedbackCallback(std::shared_ptr<rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>>, const std::shared_ptr<const move_base_msgs::action::MoveBase::Feedback>);
      void resultCallback(const rclcpp_action::ClientGoalHandle<move_base_msgs::action::MoveBase>::WrappedResult);

      template <typename T>
      void actionClientGoalResponseCallback(const T t);
      template <typename T, typename U>
      void actionClientFeedbackCallback(T t, const U u);
      template <typename T>
      void actionClientResultCallback(const T t);
  };

  class RQTMoveBaseClient : public rqt_gui_cpp::Plugin
  {
    Q_OBJECT
    public:
      RQTMoveBaseClient();
      virtual ~RQTMoveBaseClient();

      virtual void initPlugin(qt_gui_cpp::PluginContext& context) override;
      virtual void shutdownPlugin() override;
      virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const override;
      virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) override;

    private:
      CustomWidget *widget_;
  };

}  // namespace rqt_plugin

#endif  // PUSH_BUTTON_H__
