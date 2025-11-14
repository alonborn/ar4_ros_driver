#pragma once

#include <boost/scoped_ptr.hpp>
#include <chrono>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "annin_ar4_driver/teensy_driver.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"
#include "my_robot_interfaces/srv/nudge_joint.hpp"     // NEW
#include <my_robot_interfaces/srv/move_servo_to_angle.hpp>
#include <my_robot_interfaces/srv/set_speed_scale.hpp>

#include <rclcpp/qos.hpp>                            // for rclcpp::ServicesQoS



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

  void handle_set_speed_scale(
    const std::shared_ptr<my_robot_interfaces::srv::SetSpeedScale::Request> req,
    std::shared_ptr<my_robot_interfaces::srv::SetSpeedScale::Response> res);

  rclcpp::Service<my_robot_interfaces::srv::SetSpeedScale>::SharedPtr set_speed_scale_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_gripper_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_gripper_srv_;

  // Homing service handler
  void handle_homing_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool perform_homing();

  // NEW: direct API to nudge one joint by a number of motor steps (can be negative)
  // joint_idx in [0..N-1], steps e.g. +5 / -5
  bool nudgeJointSteps(int joint_idx, int steps);


  void handle_move_servo_to_angle(
    const std::shared_ptr<my_robot_interfaces::srv::MoveServoToAngle::Request> req,
    std::shared_ptr<my_robot_interfaces::srv::MoveServoToAngle::Response> res);

  // --- NEW: service handlers ---
  void handle_open_gripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response>);

  void handle_close_gripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response>);


 private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr homing_service_;
  rclcpp::Service<my_robot_interfaces::srv::MoveServoToAngle>::SharedPtr move_servo_srv_;

  std::thread service_thread_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
  void handle_string_message(const std_msgs::msg::String::SharedPtr msg);

   void handle_nudge_joint(
    const std::shared_ptr<my_robot_interfaces::srv::NudgeJoint::Request> req,
    std::shared_ptr<my_robot_interfaces::srv::NudgeJoint::Response> res);



  bool is_homing_ = false;
  bool is_homed_ = false;
  
  std::mutex string_mutex_;
  std::string last_received_string_;
  rclcpp::Service<my_robot_interfaces::srv::NudgeJoint>::SharedPtr nudge_service_;

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
