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
    // run calibration
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
  // resize vectors
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

// hardware_interface::CallbackReturn ARHardwareInterface::home_robot() {
//   RCLCPP_INFO(logger_, "Homing started...");

//   // if (!driver_.calibrateJoints()) {
//   //   RCLCPP_INFO(logger_, "calibration failed.");
//   //   return hardware_interface::CallbackReturn::ERROR;
//   // }

//   RCLCPP_INFO(logger_, "Homing completed.");
//   return hardware_interface::CallbackReturn::SUCCESS;
// }


hardware_interface::CallbackReturn ARHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware- interface...");
  //register_homing_service();
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
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & ) {
    // Existing configuration
    RCLCPP_INFO(logger_, "configuring node");
    // Create homing service
    node_ = std::make_shared<rclcpp::Node>("ar4_hardware_interface_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    
    homing_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/homing",
      std::bind(&ARHardwareInterface::handle_homing_request, this,
        std::placeholders::_1, std::placeholders::_2));
    
    // Create subscription to the string topic
    string_subscription_ = node_->create_subscription<std_msgs::msg::String>(
      "~/homing_string", 10,
      std::bind(&ARHardwareInterface::handle_string_message, this, std::placeholders::_1));


    // Start a thread for handling services
    service_thread_ = std::thread([this]() {
      while (rclcpp::ok()) {
        executor_->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  void ARHardwareInterface::handle_string_message(const std_msgs::msg::String::SharedPtr msg) 
  {
    std::lock_guard<std::mutex> lock(string_mutex_);
    last_received_string_ = msg->data;
    //RCLCPP_INFO(logger_, "last_received_string_: %s" , last_received_string_.c_str());
    // RCLCPP_INFO(node_->get_logger(), "Received string: %s", last_received_string_.c_str());
  }

  void ARHardwareInterface::handle_homing_request(const std::shared_ptr<std_srvs::srv::Trigger::Request> ,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)  {
    RCLCPP_INFO(logger_, "handle_homing_request was called");
    std::string current_string;
    // Get the latest string from the topic
    {
      std::lock_guard<std::mutex> lock(string_mutex_);
      current_string = last_received_string_;
    }
    RCLCPP_INFO(logger_, "handle_homing_request was called with: %s" , current_string.c_str());
    if (driver_.calibrateSomeJoints(current_string))
    {
      response->success = true;
      response->message = "Homing completed successfully";
    }
    else
    {
      response->success = false;
      response->message = "Homing failed";
    }

  // if (is_homing_) {
  //   response->success = false;
  //   response->message = "Homing already in progress";
  //   return;
  // }
  
  // if (trajectory_controller_active) {
  //   response->success = false;
  //   response->message = "Cannot home while trajectory controller is active";
  //   return;
  // }
  
  // response->success = perform_homing();
  // if (response->success) {
  //   response->message = "Homing completed successfully";
  // } else {
  //   response->message = "Homing failed";
  // }
  response->success = true;
  response->message = "Homing completed successfully";
  RCLCPP_INFO(logger_, "completed handling the homing request: %s", current_string.c_str());
}

bool ARHardwareInterface::perform_homing() {
  is_homing_ = true;
  
  // Implement your homing sequence here
  // This might involve:
  // 1. Moving the robot to a known position
  // 2. Reading limit switches
  // 3. Calibrating encoders
  // 4. Setting the reference position
  
  is_homing_ = false;
  is_homed_ = true;  // Set if homing was successful
  
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
  //RCLCPP_INFO(logger_, "write called");
  // if (homing_requested_ > 0) {
  //   home_robot();
  //   homing_requested_ = false; // Reset flag
  //   return hardware_interface::return_type::OK;
  // }

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // convert from rad to deg, apply offsets
    actuator_pos_commands_[i] =
        radToDeg(joint_position_commands_[i]) - joint_offsets_[i];
    actuator_vel_commands_[i] = radToDeg(joint_velocity_commands_[i]);
  }
  driver_.update(actuator_pos_commands_, actuator_vel_commands_,
                 actuator_positions_, actuator_velocities_);
  if (driver_.isEStopped()) {
    std::string logWarn =
        "Hardware in EStop state. To reset the EStop "
        "reactivate the hardware component using 'ros2 "
        "run annin_ar4_driver reset_estop.sh'.";
    RCLCPP_WARN(logger_, logWarn.c_str());

    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace annin_ar4_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(annin_ar4_driver::ARHardwareInterface,
                       hardware_interface::SystemInterface)
