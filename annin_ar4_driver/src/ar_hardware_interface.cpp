#include <annin_ar4_driver/ar_hardware_interface.hpp>
#include <sstream>


namespace annin_ar4_driver {

hardware_interface::CallbackReturn ARHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = info;
  init_variables();

  // init motor driver
  std::string serial_port = info_.hardware_parameters.at("serial_port");
  std::string ar_model = info_.hardware_parameters.at("ar_model");
  std::string velocity_control_p =
      info_.hardware_parameters.at("velocity_control_enabled");
  bool velocity_control_enabled =
      velocity_control_p == "True" || velocity_control_p == "true";
  int baud_rate = 9600;
  bool success = driver_.init(ar_model, serial_port, baud_rate,
                              info_.joints.size(), velocity_control_enabled);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // calibrate joints if needed
  bool calibrate = info_.hardware_parameters.at("calibrate") == "True";
  if (calibrate) {
    RCLCPP_INFO(logger_, "Running joint calibration...");
    if (!driver_.calibrateJoints()) {
      RCLCPP_INFO(logger_, "calibration failed.");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(logger_, "calibration succeeded.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ARHardwareInterface::init_variables() {
  int num_joints = info_.joints.size();
  actuator_pos_commands_.resize(num_joints);
  actuator_vel_commands_.resize(num_joints);
  actuator_positions_.resize(num_joints);
  actuator_velocities_.resize(num_joints);
  joint_positions_.resize(num_joints);
  joint_velocities_.resize(num_joints);
  joint_efforts_.resize(num_joints);
  joint_position_commands_.resize(num_joints);
  joint_velocity_commands_.resize(num_joints);
  joint_effort_commands_.resize(num_joints);
  joint_offsets_.resize(num_joints);
  for (int i = 0; i < num_joints; ++i) {
    joint_offsets_[i] =
        std::stod(info_.joints[i].parameters["position_offset"]);
  }
}

// --- Util for the service ---
bool ARHardwareInterface::nudgeJointSteps(int joint_idx, int steps) {
  RCLCPP_INFO(logger_, "Requested nudge: joint_idx=%d, steps=%d",
              joint_idx, steps);

  // Check range
  if (joint_idx < 0 || joint_idx >= static_cast<int>(info_.joints.size())) {
    RCLCPP_ERROR(logger_,
      "nudgeJointSteps: joint_idx %d out of range [0,%zu)",
      joint_idx, info_.joints.size());
    return false;
  }

  // Show current position before nudge (if we have it)
  if (joint_positions_.size() > static_cast<size_t>(joint_idx)) {
    RCLCPP_INFO(logger_,
      "Current joint[%d] position: %.6f rad (%.3f deg)",
      joint_idx,
      joint_positions_[joint_idx],
      radToDeg(joint_positions_[joint_idx]));
  } else {
    RCLCPP_WARN(logger_,
      "No current position available for joint[%d]",
      joint_idx);
  }

  // Execute nudge via driver
  bool ok = driver_.nudgeJointSteps(joint_idx, steps);
  RCLCPP_INFO(logger_, "Driver nudgeJointSteps() returned: %s",
              ok ? "SUCCESS" : "FAILURE");

  // Show new position after nudge
  if (ok && joint_positions_.size() > static_cast<size_t>(joint_idx)) {
    RCLCPP_INFO(logger_,
      "New joint[%d] position: %.6f rad (%.3f deg)",
      joint_idx,
      joint_positions_[joint_idx],
      radToDeg(joint_positions_[joint_idx]));
  }

  return ok;
}


// --- NEW: service handler for discrete calibration nudge ---
void ARHardwareInterface::handle_nudge_joint(const std::shared_ptr<my_robot_interfaces::srv::NudgeJoint::Request> req,
    std::shared_ptr<my_robot_interfaces::srv::NudgeJoint::Response> res) {
  const int joint_idx = static_cast<int>(req->joint_index);
  const int steps     = static_cast<int>(req->steps);

  // (Optional) safety gates
  if (driver_.isEStopped()) {
    res->success = false;
    res->message = "E-Stop active";
    RCLCPP_ERROR(logger_, "Refusing nudge: E-Stop active");
    return;
  }

  // Apply discrete nudge
  const bool ok = nudgeJointSteps(joint_idx, steps);
  res->success = ok;
  if (ok) {
    res->message = "Nudge applied";
    RCLCPP_INFO(logger_, "Nudged joint %d by %d steps", joint_idx, steps);
  } else {
    res->message = "Nudge failed";
    RCLCPP_ERROR(logger_, "Nudge failed for joint %d steps %d", joint_idx, steps);
  }
}

hardware_interface::CallbackReturn ARHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware- interface...");
  // Reset Estop (if any)
  bool success = driver_.resetEStop();
  if (!success) {
    RCLCPP_ERROR(logger_,
                 "Cannot activate. Hardware E-stop state cannot be reset.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");

  // Gracefully stop executor thread
  if (executor_) {
    executor_->cancel();
  }
  if (service_thread_.joinable()) {
    service_thread_.join();
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & ) {
  RCLCPP_INFO(logger_, "configuring node");

  // Node + executor (non-RT)
  node_ = std::make_shared<rclcpp::Node>("ar4_hardware_interface_node");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(node_);

  // Homing service (existing)
  homing_service_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/homing",
    std::bind(&ARHardwareInterface::handle_homing_request, this,
      std::placeholders::_1, std::placeholders::_2));

  // String topic (existing)
  string_subscription_ = node_->create_subscription<std_msgs::msg::String>(
    "~/homing_string", 10,
    std::bind(&ARHardwareInterface::handle_string_message, this, std::placeholders::_1));

  // NEW: discrete nudge service
  nudge_service_ = node_->create_service<my_robot_interfaces::srv::NudgeJoint>(
    "~/nudge_joint",
    std::bind(&ARHardwareInterface::handle_nudge_joint, this,
              std::placeholders::_1, std::placeholders::_2));

  // Start executor thread (once)
  service_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {
      executor_->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ARHardwareInterface::handle_string_message(const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(string_mutex_);
  last_received_string_ = msg->data;
}

void ARHardwareInterface::handle_homing_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(logger_, "handle_homing_request was called");
  std::string current_string;
  { std::lock_guard<std::mutex> lock(string_mutex_);
    current_string = last_received_string_;
  }
  RCLCPP_INFO(logger_, "handle_homing_request with: %s", current_string.c_str());

  if (driver_.calibrateSomeJoints(current_string)) {
    response->success = true;
    response->message = "Homing completed successfully";
  } else {
    response->success = false;
    response->message = "Homing failed";
  }

  RCLCPP_INFO(logger_, "completed handling the homing request: %s", current_string.c_str());
}

bool ARHardwareInterface::perform_homing() {
  is_homing_ = true;
  // ... your homing logic ...
  is_homing_ = false;
  is_homed_ = true;
  return is_homed_;
}

std::vector<hardware_interface::StateInterface>
ARHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position",
                                  &joint_positions_[i]);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                  &joint_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ARHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position",
                                    &joint_position_commands_[i]);
    command_interfaces.emplace_back(info_.joints[i].name, "velocity",
                                    &joint_velocity_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type ARHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  driver_.getJointPositions(actuator_positions_);
  driver_.getJointVelocities(actuator_velocities_);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // apply offsets, convert from deg to rad for moveit
    joint_positions_[i] = degToRad(actuator_positions_[i] + joint_offsets_[i]);
    joint_velocities_[i] = degToRad(actuator_velocities_[i]);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // convert from rad to deg, apply offsets
    actuator_pos_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
    actuator_vel_commands_[i] = radToDeg(joint_velocity_commands_[i]);
  }
  driver_.update(actuator_pos_commands_, actuator_vel_commands_,
                 actuator_positions_, actuator_velocities_);
  if (driver_.isEStopped()) {
    const char* logWarn =
        "Hardware in EStop state. To reset the EStop "
        "reactivate the hardware component using 'ros2 "
        "run annin_ar4_driver reset_estop.sh'.";
    RCLCPP_WARN(logger_, "%s", logWarn);
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace annin_ar4_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(annin_ar4_driver::ARHardwareInterface,
                       hardware_interface::SystemInterface)
