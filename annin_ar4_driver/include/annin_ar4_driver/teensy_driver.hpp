#ifndef TEENSY_DRIVER_H
#define TEENSY_DRIVER_H

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "math.h"
#include "time.h"

namespace annin_ar4_driver {

class TeensyDriver {
 public:
  bool init(std::string ar_model, std::string port, int baudrate,
            int num_joints, bool velocity_control_enabled);
  void update(std::vector<double>& pos_commands,
              std::vector<double>& vel_commands,
              std::vector<double>& joint_states,
              std::vector<double>& joint_velocities);
  void getJointPositions(std::vector<double>& joint_positions);
  void getJointVelocities(std::vector<double>& joint_velocities);
  bool calibrateJoints();
  bool calibrateSomeJoints(std::string joints);
  bool setGlobalSpeedScale(double scale);

  bool openGripper();
  bool closeGripper();
  bool moveServoToAngle(double angle_deg);

  bool resetEStop();
  bool isEStopped();
  
  // Nudge a single joint by a number of motor steps (can be negative)
  // joint_idx: 0..(num_joints_-1), steps: e.g., +5 or -5
  bool nudgeJointSteps(int joint_idx, int steps);
  
  TeensyDriver();

 private:
  bool initialised_;
  std::string ar_model_;
  std::string version_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  int num_joints_;
  std::vector<double> joint_positions_deg_;
  std::vector<double> joint_velocities_deg_;
  std::vector<int> enc_calibrations_;
  bool velocity_control_enabled_ = true;
  bool is_estopped_;

  rclcpp::Logger logger_ = rclcpp::get_logger("teensy_driver");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Comms with teensy
  bool exchange(std::string outMsg);  // exchange joint commands/state
  bool transmit(std::string outMsg, std::string& err);
  void receive(std::string& inMsg);
  bool sendCommand(std::string outMsg);  // send arbitrary commands

  void checkInit(std::string msg);

  template <typename T>
  void parseValuesToVector(const std::string msg, std::vector<T>& values);
  void updateEncoderCalibrations(std::string msg);
  void updateJointPositions(std::string msg);
  void updateJointVelocities(std::string msg);
  void updateEStopStatus(std::string msg);

  // Special exchange for "HM" that tolerates non-header text and waits for completion
  bool exchangeHM(const std::string& outMsg);

  std::mutex io_mutex_;  // protect serial access if called from multiple
    
};

}  // namespace annin_ar4_driver

#endif  // TEENSY_DRIVER
