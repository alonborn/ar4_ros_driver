#pragma once

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "annin_ar4_driver/teensy_driver.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

using namespace hardware_interface;

namespace annin_ar4_driver {
class ARHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ARHardwareInterface);

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;
        
  // New method for homing


  void handle_homing_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool perform_homing();

 private:

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service_;
  std::thread service_thread_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
  void handle_string_message(const std_msgs::msg::String::SharedPtr msg);

  bool is_homing_ = false;
  bool is_homed_ = false;
  
  std::mutex string_mutex_;
  std::string last_received_string_;

  rclcpp::Logger logger_ = rclcpp::get_logger("annin_ar4_driver");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Motor driver
  TeensyDriver driver_;
  std::vector<double> actuator_pos_commands_;
  std::vector<double> actuator_vel_commands_;
  std::vector<double> actuator_positions_;
  std::vector<double> actuator_velocities_;

  // Shared memory
  std::vector<double> joint_offsets_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_efforts_;
  std::vector<double> joint_position_commands_;
  std::vector<double> joint_velocity_commands_;
  std::vector<double> joint_effort_commands_;

  // Homing flag
  bool homing_requested_ = false;

  // Misc
  void init_variables();
  double degToRad(double deg) { return deg / 180.0 * M_PI; };
  double radToDeg(double rad) { return rad / M_PI * 180.0; };

};
}  // namespace annin_ar4_driver
