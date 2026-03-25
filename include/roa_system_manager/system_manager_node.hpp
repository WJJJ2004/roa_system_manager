#pragma once

#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <roa_system_manager/packet_manager.hpp>
#include <roa_interfaces/msg/system_status.hpp>
#include <roa_interfaces/msg/motor_command.hpp>
#include <roa_interfaces/msg/motor_command_array.hpp>
#include <roa_interfaces/msg/rsu_target.hpp>

namespace system_manager
{

class SystemManagerNode : public rclcpp::Node
{
public:
  explicit SystemManagerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  enum class FsmState : uint8_t
  {
    BOOT = 0,
    RUN = 1,
    SAFE_HOLD = 2,
    ESTOP = 3
  };

  enum class BootStep : uint8_t
  {
    WAIT_CONTROLLER_SERVICE = 0,
    REQUEST_GET_STATE_FOR_CONFIGURE = 1,
    WAIT_GET_STATE_FOR_CONFIGURE = 2,
    REQUEST_CONFIGURE = 3,
    WAIT_CONFIGURE = 4,
    WAIT_CONTROLLER_READY = 5,
    REQUEST_INIT_POS = 6,
    WAIT_INIT_POS_DONE = 7,
    REQUEST_GET_STATE_FOR_ACTIVATE = 8,
    WAIT_GET_STATE_FOR_ACTIVATE = 9,
    REQUEST_ACTIVATE = 10,
    WAIT_ACTIVATE = 11,
    DONE = 12
  };

  enum class SafeHoldStep : uint8_t
  {
    REQUEST_DEACTIVATE = 0,
    WAIT_DEACTIVATE = 1,
    WAIT_RECOVERY = 2
  };

  struct StatusCache
  {
    bool received{false};
    bool ready{false};
    bool healthy{false};
    bool rt_ok{false};
    uint32_t error_code{0};

    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_rx_time{0, 0, RCL_ROS_TIME};
  };

  struct GetStateCallContext
  {
    bool pending{false};
    bool done{false};
    bool success{false};

    rclcpp::Time request_time{0, 0, RCL_ROS_TIME};

    uint8_t state_id{0};
    std::string label;
    std::string error_msg;
  };

  struct ChangeStateCallContext
  {
    bool pending{false};
    bool done{false};
    bool success{false};

    rclcpp::Time request_time{0, 0, RCL_ROS_TIME};

    uint8_t transition_id{0};
    std::string error_msg;
  };

private:
  bool shouldRepublishInitPos(const rclcpp::Time& now_time) const;
  void publishInitPosIfNeeded(const rclcpp::Time& now_time);
  void publish_init_pos();

  void declareAndLoadParams();
  void setupRosInterfaces();
  void setupTimer();

  void onControllerStatus(const roa_interfaces::msg::SystemStatus::SharedPtr msg);
  void onInitPosDone(const std_msgs::msg::Bool::SharedPtr msg);
  void onFsmTimer();
  void onBootSeqTimer();

  bool isStatusFresh(const StatusCache& cache, const rclcpp::Time& now_time, double timeout_sec) const;

  bool bootReady(const rclcpp::Time& now_time) const;
  bool hasRecoverableFault(const rclcpp::Time& now_time) const;
  bool hasCriticalFault() const;
  bool recovered(const rclcpp::Time& now_time) const;

  bool controllerLifecycleServicesReady() const;

  void resetGetStateContext();
  void resetChangeStateContext();

  void requestControllerGetStateAsync();
  void requestControllerChangeStateAsync(uint8_t transition_id);

  bool getStateTimedOut(const rclcpp::Time& now_time) const;
  bool changeStateTimedOut(const rclcpp::Time& now_time) const;

  void enterBootStep(BootStep step);
  void enterSafeHoldStep(SafeHoldStep step);

  void handleBoot(const rclcpp::Time& tnow);
  void handleRun(const rclcpp::Time& tnow);
  void handleSafeHold(const rclcpp::Time& tnow);

  void transitionTo(FsmState new_state, const std::string& reason);

  static const char* toString(FsmState state);

private:
  bool is_init_pos_done_{false};
  roa_controller_node::PacketManager::Command12Dof init_pos_{};

  rclcpp::Time init_pos_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_init_pos_pub_time_{0, 0, RCL_ROS_TIME};

  // parameters
  double fsm_period_sec_{0.02};               // 50 Hz
  double controller_status_timeout_sec_{0.3};
  double safe_hold_timeout_sec_{2.0};
  double lifecycle_service_timeout_sec_{1.0};
  double init_pos_pub_period_sec_{0.05};      // 20 Hz
  double init_pos_timeout_sec_{10.0};

  std::string controller_status_topic_{"/controller/status"};
  std::string controller_node_name_{"/roa_controller_node"};

  // top-level FSM
  FsmState current_state_{FsmState::BOOT};
  rclcpp::Time last_transition_time_{0, 0, RCL_ROS_TIME};
  std::string last_transition_reason_{"startup"};

  // internal orchestration substeps
  BootStep boot_step_{BootStep::WAIT_CONTROLLER_SERVICE};
  SafeHoldStep safe_hold_step_{SafeHoldStep::REQUEST_DEACTIVATE};

  rclcpp::Time boot_step_enter_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time safe_hold_step_enter_time_{0, 0, RCL_ROS_TIME};

  // orchestration progress flags
  bool controller_configured_{false};
  bool controller_activated_{false};

  // status cache
  StatusCache controller_status_;

  // async lifecycle request contexts
  GetStateCallContext get_state_ctx_;
  ChangeStateCallContext change_state_ctx_;

  // ROS interfaces
  rclcpp::Subscription<roa_interfaces::msg::SystemStatus>::SharedPtr controller_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr init_pos_sub_;
  rclcpp::TimerBase::SharedPtr fsm_timer_;
  rclcpp::TimerBase::SharedPtr boot_seq_timer_;


  // lifecycle clients
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr controller_change_state_client_;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr controller_get_state_client_;

  // command publisher
  rclcpp::Publisher<roa_interfaces::msg::MotorCommandArray>::SharedPtr init_pos_pub_;
  rclcpp::Publisher<roa_interfaces::msg::RsuTarget>::SharedPtr rsu_target_pub_;
  // rclcpp::Publisher<>
};

}  // namespace system_manager