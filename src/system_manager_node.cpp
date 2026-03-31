#include "roa_system_manager/system_manager_node.hpp"

#include <chrono>
#include <functional>
#include <sstream>
#include <utility>

using namespace std::chrono_literals;

namespace
{
constexpr float DEG_20 = 0.349066f;
constexpr float DEG_30 = 0.417960f  // non real value -> calcualted by rsu solver ;
constexpr float DEG_50 = 0.872665f;
}  // namespace

namespace system_manager
{

SystemManagerNode::SystemManagerNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("system_manager_node", options)
{
  declareAndLoadParams();
  setupRosInterfaces();
  setupTimer();

  last_transition_time_ = now();
  boot_step_enter_time_ = now();
  safe_hold_step_enter_time_ = now();

  RCLCPP_INFO(
    get_logger(),
    "system_manager_node started | state=%s controller=%s",
    toString(current_state_),
    controller_node_name_.c_str());
}

void SystemManagerNode::declareAndLoadParams()
{
  fsm_period_sec_ =
    this->declare_parameter<double>("fsm_period_sec", fsm_period_sec_);
  controller_status_timeout_sec_ =
    this->declare_parameter<double>("controller_status_timeout_sec", controller_status_timeout_sec_);
  safe_hold_timeout_sec_ =
    this->declare_parameter<double>("safe_hold_timeout_sec", safe_hold_timeout_sec_);
  lifecycle_service_timeout_sec_ =
    this->declare_parameter<double>("lifecycle_service_timeout_sec", lifecycle_service_timeout_sec_);
  init_pos_pub_period_sec_ =
    this->declare_parameter<double>("init_pos_pub_period_sec", init_pos_pub_period_sec_);
  init_pos_timeout_sec_ =
    this->declare_parameter<double>("init_pos_timeout_sec", init_pos_timeout_sec_);
  error_log_throttle_sec_ =
    this->declare_parameter<double>("error_log_throttle_sec", error_log_throttle_sec_);

  controller_status_topic_ =
    this->declare_parameter<std::string>("controller_status_topic", controller_status_topic_);
  controller_node_name_ =
    this->declare_parameter<std::string>("controller_node_name", controller_node_name_);

  // init pose preset
  init_pos_.left_hip_pitch   = -DEG_20;
  init_pos_.left_hip_roll    = 0.0f;
  init_pos_.left_hip_yaw     = 0.0f;
  init_pos_.left_knee_pitch  =  DEG_50;

  init_pos_.right_hip_pitch  =  DEG_20;
  init_pos_.right_hip_roll   = 0.0f;
  init_pos_.right_hip_yaw    = 0.0f;
  init_pos_.right_knee_pitch = -DEG_50;

  init_pos_.left_rsu_upper   =  DEG_30;
  init_pos_.left_rsu_lower   = -DEG_30;
  init_pos_.right_rsu_upper  = -DEG_30;
  init_pos_.right_rsu_lower  =  DEG_30;
}

void SystemManagerNode::setupRosInterfaces()
{
  auto status_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
  auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  controller_status_sub_ = this->create_subscription<roa_interfaces::msg::SystemStatus>(
    controller_status_topic_,
    status_qos,
    std::bind(&SystemManagerNode::onControllerStatus, this, std::placeholders::_1));

  init_pos_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "walk_initialized",
    cmd_qos,
    std::bind(&SystemManagerNode::onInitPosDone, this, std::placeholders::_1));

  init_pos_pub_ = this->create_publisher<roa_interfaces::msg::MotorCommandArray>(
    "/hardware_interface/command",
    cmd_qos);

  controller_change_state_client_ =
    this->create_client<lifecycle_msgs::srv::ChangeState>(
      controller_node_name_ + std::string("/change_state"));

  controller_get_state_client_ =
    this->create_client<lifecycle_msgs::srv::GetState>(
      controller_node_name_ + std::string("/get_state"));
}

void SystemManagerNode::setupTimer()
{
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(fsm_period_sec_));

  fsm_timer_ = this->create_wall_timer(
    period,
    std::bind(&SystemManagerNode::onFsmTimer, this));
}

void SystemManagerNode::onControllerStatus(
  const roa_interfaces::msg::SystemStatus::SharedPtr msg)
{
  controller_status_.received = true;
  controller_status_.ready = msg->ready;
  controller_status_.healthy = msg->healthy;
  controller_status_.rt_ok = msg->rt_ok;
  controller_status_.error_code = msg->error_code;
  controller_status_.stamp = rclcpp::Time(msg->header.stamp);
  controller_status_.last_rx_time = now();

  static constexpr uint32_t ERR_POLICY_NOT_LOADED = 1u << 0;
  static constexpr uint32_t ERR_IMU_STALE         = 1u << 1;
  static constexpr uint32_t ERR_MOTOR_STATE_STALE = 1u << 2;
  static constexpr uint32_t ERR_RSU_STATE_STALE   = 1u << 3;
  static constexpr uint32_t ERR_POLICY_STALE      = 1u << 4;

  const uint32_t err = msg->error_code;
  const auto now_time = now();

  if (err == 0u) {
    if (last_logged_error_code_ != 0u) {
      RCLCPP_INFO(get_logger(), "Controller error cleared");
      last_logged_error_code_ = 0u;
      last_error_log_time_ = now_time;
    }
    return;
  }

  const bool code_changed = (err != last_logged_error_code_);
  const bool throttle_elapsed =
    (last_error_log_time_.nanoseconds() == 0) ||
    ((now_time - last_error_log_time_).seconds() >= error_log_throttle_sec_);

  if (!code_changed && !throttle_elapsed) {
    return;
  }

  std::stringstream ss;
  ss << "[Controller Error] code=" << err << " | ";

  if (err & ERR_POLICY_NOT_LOADED) {
    ss << "POLICY_NOT_LOADED ";
  }
  if (err & ERR_IMU_STALE) {
    ss << "IMU_STALE ";
  }
  if (err & ERR_MOTOR_STATE_STALE) {
    ss << "MOTOR_STATE_STALE ";
  }
  if (err & ERR_RSU_STATE_STALE) {
    ss << "RSU_STATE_STALE ";
  }
  if (err & ERR_POLICY_STALE) {
    ss << "POLICY_STALE ";
  }

  RCLCPP_WARN(get_logger(), "%s", ss.str().c_str());

  last_logged_error_code_ = err;
  last_error_log_time_ = now_time;
}

void SystemManagerNode::onInitPosDone(const std_msgs::msg::Bool::SharedPtr msg)
{
  is_init_pos_done_ = msg->data;
}

bool SystemManagerNode::isStatusFresh(
  const StatusCache& cache,
  const rclcpp::Time& now_time,
  double timeout_sec) const
{
  if (!cache.received) {
    return false;
  }
  return (now_time - cache.last_rx_time).seconds() <= timeout_sec;
}

bool SystemManagerNode::bootReady(const rclcpp::Time& now_time) const
{
  const bool fresh = isStatusFresh(controller_status_, now_time, controller_status_timeout_sec_);
  return fresh && controller_status_.ready && controller_status_.healthy && controller_status_.rt_ok;
}

bool SystemManagerNode::hasRecoverableFault(const rclcpp::Time& now_time) const
{
  const bool stale = !isStatusFresh(controller_status_, now_time, controller_status_timeout_sec_);
  const bool rt_bad = controller_status_.received && !controller_status_.rt_ok;
  return stale || rt_bad;
}

bool SystemManagerNode::hasCriticalFault() const
{
  return controller_status_.received &&
         (!controller_status_.healthy || controller_status_.error_code != 0u);
}

bool SystemManagerNode::recovered(const rclcpp::Time& now_time) const
{
  return bootReady(now_time);
}

bool SystemManagerNode::controllerLifecycleServicesReady() const
{
  return controller_change_state_client_->service_is_ready() &&
         controller_get_state_client_->service_is_ready();
}

void SystemManagerNode::resetGetStateContext()
{
  get_state_ctx_ = GetStateCallContext{};
}

void SystemManagerNode::resetChangeStateContext()
{
  change_state_ctx_ = ChangeStateCallContext{};
}

void SystemManagerNode::requestControllerGetStateAsync()
{
  if (get_state_ctx_.pending) {
    return;
  }

  auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

  get_state_ctx_.pending = true;
  get_state_ctx_.done = false;
  get_state_ctx_.success = false;
  get_state_ctx_.request_time = now();
  get_state_ctx_.label.clear();
  get_state_ctx_.error_msg.clear();

  controller_get_state_client_->async_send_request(
    req,
    [this](rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) {
      get_state_ctx_.pending = false;
      get_state_ctx_.done = true;

      try {
        const auto resp = future.get();
        get_state_ctx_.success = true;
        get_state_ctx_.state_id = resp->current_state.id;
        get_state_ctx_.label = resp->current_state.label;
      } catch (const std::exception& e) {
        get_state_ctx_.success = false;
        get_state_ctx_.error_msg = e.what();
      }
    });
}

void SystemManagerNode::requestControllerChangeStateAsync(uint8_t transition_id)
{
  if (change_state_ctx_.pending) {
    return;
  }

  auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  req->transition.id = transition_id;

  change_state_ctx_.pending = true;
  change_state_ctx_.done = false;
  change_state_ctx_.success = false;
  change_state_ctx_.request_time = now();
  change_state_ctx_.transition_id = transition_id;
  change_state_ctx_.error_msg.clear();

  controller_change_state_client_->async_send_request(
    req,
    [this](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) {
      change_state_ctx_.pending = false;
      change_state_ctx_.done = true;

      try {
        const auto resp = future.get();
        change_state_ctx_.success = resp->success;
        if (!resp->success) {
          change_state_ctx_.error_msg = "change_state returned success=false";
        }
      } catch (const std::exception& e) {
        change_state_ctx_.success = false;
        change_state_ctx_.error_msg = e.what();
      }
    });
}

bool SystemManagerNode::getStateTimedOut(const rclcpp::Time& now_time) const
{
  return get_state_ctx_.pending &&
         ((now_time - get_state_ctx_.request_time).seconds() > lifecycle_service_timeout_sec_);
}

bool SystemManagerNode::changeStateTimedOut(const rclcpp::Time& now_time) const
{
  return change_state_ctx_.pending &&
         ((now_time - change_state_ctx_.request_time).seconds() > lifecycle_service_timeout_sec_);
}

void SystemManagerNode::enterBootStep(BootStep step)
{
  boot_step_ = step;
  boot_step_enter_time_ = now();

  if (step == BootStep::WAIT_CONTROLLER_READY) {
    controller_ready_wait_count_ = 0;
  }
}

void SystemManagerNode::enterSafeHoldStep(SafeHoldStep step)
{
  safe_hold_step_ = step;
  safe_hold_step_enter_time_ = now();
}

bool SystemManagerNode::shouldRepublishInitPos(const rclcpp::Time& now_time) const
{
  if (last_init_pos_pub_time_.nanoseconds() == 0) {
    return true;
  }

  return (now_time - last_init_pos_pub_time_).seconds() >= init_pos_pub_period_sec_;
}

void SystemManagerNode::publishInitPosIfNeeded(const rclcpp::Time& now_time)
{
  if (!shouldRepublishInitPos(now_time)) {
    return;
  }

  publish_init_pos();
  last_init_pos_pub_time_ = now_time;
}

void SystemManagerNode::publish_init_pos()
{
  auto msg = roa_controller_node::PacketManager::build(init_pos_, this->now(), "INIT POS HOLD");
  init_pos_pub_->publish(msg);
}

void SystemManagerNode::handleBoot(const rclcpp::Time& tnow)
{
  // 핵심 요구사항:
  // activate 성공 전까지는 BOOT 어디에 있든 100Hz로 init pose hold 발행
  if (!controller_activated_) {
    publishInitPosIfNeeded(tnow);
  }

  switch (boot_step_) {
    case BootStep::WAIT_CONTROLLER_SERVICE:
    {
      if (controllerLifecycleServicesReady()) {
        RCLCPP_INFO(get_logger(), "Controller lifecycle services are ready");
        enterBootStep(BootStep::REQUEST_GET_STATE_FOR_CONFIGURE);
      }
      return;
    }

    case BootStep::REQUEST_GET_STATE_FOR_CONFIGURE:
    {
      resetGetStateContext();
      requestControllerGetStateAsync();
      enterBootStep(BootStep::WAIT_GET_STATE_FOR_CONFIGURE);
      return;
    }

    case BootStep::WAIT_GET_STATE_FOR_CONFIGURE:
    {
      if (getStateTimedOut(tnow)) {
        transitionTo(FsmState::SAFE_HOLD, "get_state timeout before configure");
        return;
      }

      if (!get_state_ctx_.done) {
        return;
      }

      if (!get_state_ctx_.success) {
        transitionTo(FsmState::SAFE_HOLD, "get_state failed before configure");
        return;
      }

      if (get_state_ctx_.state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
        enterBootStep(BootStep::REQUEST_CONFIGURE);
      } else {
        controller_configured_ = true;
        enterBootStep(BootStep::WAIT_CONTROLLER_READY);
      }
      return;
    }

    case BootStep::REQUEST_CONFIGURE:
    {
      resetChangeStateContext();
      requestControllerChangeStateAsync(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
      enterBootStep(BootStep::WAIT_CONFIGURE);
      return;
    }

    case BootStep::WAIT_CONFIGURE:
    {
      if (changeStateTimedOut(tnow)) {
        transitionTo(FsmState::SAFE_HOLD, "configure timeout");
        return;
      }

      if (!change_state_ctx_.done) {
        return;
      }

      if (!change_state_ctx_.success) {
        transitionTo(FsmState::SAFE_HOLD, "configure failed");
        return;
      }

      controller_configured_ = true;
      enterBootStep(BootStep::WAIT_CONTROLLER_READY);
      return;
    }

    case BootStep::WAIT_CONTROLLER_READY:
    {
      if (bootReady(tnow)) {
        controller_ready_wait_count_ = 0;
        RCLCPP_INFO(get_logger(), "Controller status ready/healthy/rt_ok confirmed");
        enterBootStep(BootStep::WAIT_INIT_POS_DONE);
        return;
      }

      if (controller_ready_wait_count_ < 1000) {
        ++controller_ready_wait_count_;
      } else {
        RCLCPP_ERROR(
          get_logger(),
          "Controller status not ready after %d consecutive checks",
          controller_ready_wait_count_);
        transitionTo(FsmState::SAFE_HOLD, "controller safety check timeout");
      }
      return;
    }

    case BootStep::WAIT_INIT_POS_DONE:
    {
      if (is_init_pos_done_) {
        RCLCPP_INFO(get_logger(), "Init pose done confirmed");
        enterBootStep(BootStep::REQUEST_GET_STATE_FOR_ACTIVATE);
        return;
      }

      // if ((tnow - init_pos_start_time_).seconds() > init_pos_timeout_sec_) {
      //   transitionTo(FsmState::SAFE_HOLD, "init pose timeout");
      //   return;
      // }

      return;
    }

    case BootStep::REQUEST_GET_STATE_FOR_ACTIVATE:
    {
      resetGetStateContext();
      requestControllerGetStateAsync();
      enterBootStep(BootStep::WAIT_GET_STATE_FOR_ACTIVATE);
      return;
    }

    case BootStep::WAIT_GET_STATE_FOR_ACTIVATE:
    {
      if (getStateTimedOut(tnow)) {
        transitionTo(FsmState::SAFE_HOLD, "get_state timeout before activate");
        return;
      }

      if (!get_state_ctx_.done) {
        return;
      }

      if (!get_state_ctx_.success) {
        transitionTo(FsmState::SAFE_HOLD, "get_state failed before activate");
        return;
      }

      if (get_state_ctx_.state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
        enterBootStep(BootStep::REQUEST_ACTIVATE);
      } else if (get_state_ctx_.state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        controller_activated_ = true;
        enterBootStep(BootStep::WAIT_ACTIVATE);
      } else {
        transitionTo(FsmState::SAFE_HOLD, "controller not inactive before activate");
      }
      return;
    }

    case BootStep::REQUEST_ACTIVATE:
    {
      resetChangeStateContext();
      requestControllerChangeStateAsync(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
      enterBootStep(BootStep::WAIT_ACTIVATE);
      return;
    }

    case BootStep::WAIT_ACTIVATE:
    {
      if (change_state_ctx_.pending && changeStateTimedOut(tnow)) {
        transitionTo(FsmState::SAFE_HOLD, "activate timeout");
        return;
      }

      if (change_state_ctx_.pending) {
        return;
      }

      if (change_state_ctx_.done && !change_state_ctx_.success) {
        transitionTo(FsmState::SAFE_HOLD, "activate failed");
        return;
      }

      controller_activated_ = true;
      enterBootStep(BootStep::DONE);
      transitionTo(FsmState::RUN, "boot completed");
      return;
    }

    case BootStep::DONE:
    default:
      return;
  }
}

void SystemManagerNode::handleRun(const rclcpp::Time& tnow)
{
  if (hasCriticalFault()) {
    transitionTo(FsmState::ESTOP, "critical fault in RUN");
    return;
  }

  if (hasRecoverableFault(tnow)) {
    transitionTo(FsmState::SAFE_HOLD, "recoverable fault in RUN");
    return;
  }
}

void SystemManagerNode::handleSafeHold(const rclcpp::Time& tnow)
{
  switch (safe_hold_step_) {
    case SafeHoldStep::REQUEST_DEACTIVATE:
    {
      resetGetStateContext();
      requestControllerGetStateAsync();
      enterSafeHoldStep(SafeHoldStep::WAIT_DEACTIVATE);
      return;
    }

    case SafeHoldStep::WAIT_DEACTIVATE:
    {
      if (getStateTimedOut(tnow)) {
        transitionTo(FsmState::ESTOP, "get_state timeout in SAFE_HOLD");
        return;
      }

      if (!get_state_ctx_.done) {
        return;
      }

      if (!get_state_ctx_.success) {
        transitionTo(FsmState::ESTOP, "get_state failed in SAFE_HOLD");
        return;
      }

      if (get_state_ctx_.state_id == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        resetChangeStateContext();
        requestControllerChangeStateAsync(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
      }

      enterSafeHoldStep(SafeHoldStep::WAIT_RECOVERY);
      return;
    }

    case SafeHoldStep::WAIT_RECOVERY:
    {
      if (hasCriticalFault()) {
        transitionTo(FsmState::ESTOP, "critical fault in SAFE_HOLD");
        return;
      }

      if (recovered(tnow)) {
        transitionTo(FsmState::BOOT, "recovered from SAFE_HOLD");
        return;
      }

      if ((tnow - safe_hold_step_enter_time_).seconds() > safe_hold_timeout_sec_) {
        transitionTo(FsmState::ESTOP, "SAFE_HOLD timeout");
        return;
      }

      return;
    }

    default:
      return;
  }
}

void SystemManagerNode::transitionTo(FsmState new_state, const std::string& reason)
{
  if (current_state_ == new_state) {
    return;
  }

  current_state_ = new_state;
  last_transition_time_ = now();
  last_transition_reason_ = reason;

  RCLCPP_WARN(
    get_logger(),
    "FSM transition -> %s | reason=%s",
    toString(current_state_),
    last_transition_reason_.c_str());

  resetGetStateContext();
  resetChangeStateContext();

  if (new_state == FsmState::BOOT) {
    controller_configured_ = false;
    controller_activated_ = false;
    is_init_pos_done_ = false;
    controller_ready_wait_count_ = 0;
    // init_pos_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    last_init_pos_pub_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    enterBootStep(BootStep::WAIT_CONTROLLER_SERVICE);
  } else if (new_state == FsmState::SAFE_HOLD) {
    enterSafeHoldStep(SafeHoldStep::REQUEST_DEACTIVATE);
  }
}

const char* SystemManagerNode::toString(FsmState state)
{
  switch (state) {
    case FsmState::BOOT:
      return "BOOT";
    case FsmState::RUN:
      return "RUN";
    case FsmState::SAFE_HOLD:
      return "SAFE_HOLD";
    case FsmState::ESTOP:
      return "ESTOP";
    default:
      return "UNKNOWN";
  }
}

void SystemManagerNode::onFsmTimer()
{
  const auto tnow = now();

  switch (current_state_) {
    case FsmState::BOOT:
      handleBoot(tnow);
      break;
    case FsmState::RUN:
      handleRun(tnow);
      break;
    case FsmState::SAFE_HOLD:
      handleSafeHold(tnow);
      break;
    case FsmState::ESTOP:
      break;
    default:
      break;
  }
}

}  // namespace system_manager

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<system_manager::SystemManagerNode>(rclcpp::NodeOptions());

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}