#include "annin_ar4_driver/teensy_driver.hpp"

#include <chrono>
#include <stdexcept>
#include <thread>

#define FW_VERSION "2.0.0"

namespace annin_ar4_driver {

bool TeensyDriver::init(std::string ar_model, std::string port, int baudrate,
                        int num_joints, bool velocity_control_enabled) {
  // @TODO read version from config
  version_ = FW_VERSION;
  ar_model_ = ar_model;

  // establish connection with teensy board
  boost::system::error_code ec;
  serial_port_.open(port, ec);

  if (ec) {
    RCLCPP_WARN(logger_, "Failed to connect to serial port %s", port.c_str());
    return false;
  } else {
    serial_port_.set_option(boost::asio::serial_port_base::baud_rate(
        static_cast<uint32_t>(baudrate)));
    serial_port_.set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    RCLCPP_INFO(logger_, "Successfully connected to serial port %s",
                port.c_str());
  }

  initialised_ = false;
  std::string msg = "STA" + version_ + "B" + ar_model_ + "\n";

  while (!initialised_) {
    RCLCPP_INFO(logger_, "Waiting for response from Teensy on port %s",
                port.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    exchange(msg);
  }
  RCLCPP_INFO(logger_, "Successfully initialised driver on port %s",
              port.c_str());

  // initialise joint and encoder calibration
  num_joints_ = num_joints;
  joint_positions_deg_.resize(num_joints_);
  joint_velocities_deg_.resize(num_joints_);
  enc_calibrations_.resize(num_joints_);
  velocity_control_enabled_ = velocity_control_enabled;
  is_estopped_ = false;
  return true;
}

TeensyDriver::TeensyDriver() : serial_port_(io_service_) {}

// Update between hardware interface and hardware driver
void TeensyDriver::update(std::vector<double>& pos_commands,
                          std::vector<double>& vel_commands,
                          std::vector<double>& joint_positions,
                          std::vector<double>& joint_velocities) {
  // log pos_commands
  std::string logInfo = "Joint Pos Cmd: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(2) << pos_commands[i];
    logInfo += std::to_string(i) + ": " + jointPositionStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  // log vel_commands
  logInfo = "Joint Vel Cmd: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointVelocityStm;
    jointVelocityStm << std::fixed << std::setprecision(2) << vel_commands[i];
    logInfo += std::to_string(i) + ": " + jointVelocityStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  std::string outMsg = "";
  // construct update message
  if (velocity_control_enabled_) {
    outMsg += "MV";
    for (int i = 0; i < num_joints_; ++i) {
      outMsg += 'A' + i;
      outMsg += std::to_string(vel_commands[i]);
    }
  } else {
    outMsg += "MT";
    for (int i = 0; i < num_joints_; ++i) {
      outMsg += 'A' + i;
      outMsg += std::to_string(pos_commands[i]);
    }
  }
  outMsg += "\n";

  // run the communication with board
  exchange(outMsg);

  joint_positions = joint_positions_deg_;
  joint_velocities = joint_velocities_deg_;

  // print joint_positions
  logInfo = "Joint Pos: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointPositionStm;
    jointPositionStm << std::fixed << std::setprecision(2)
                     << joint_positions[i];
    logInfo += std::to_string(i) + ": " + jointPositionStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());

  // print joint_velocities
  logInfo = "Joint Vel: ";
  for (int i = 0; i < num_joints_; i++) {
    std::stringstream jointVelocityStm;
    jointVelocityStm << std::fixed << std::setprecision(2)
                     << joint_velocities[i];
    logInfo += std::to_string(i) + ": " + jointVelocityStm.str() + " | ";
  }
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
}

bool TeensyDriver::calibrateJoints() {
  std::string outMsg = "JC\n";
  return sendCommand(outMsg);
}

bool TeensyDriver::calibrateSomeJoints(std::string joints) {
  std::string outMsg = "JC";
  outMsg += joints;
  outMsg += "\n";
  RCLCPP_INFO(logger_, "Calibration message to be sent: %s", outMsg.c_str());
  return sendCommand(outMsg);
}

bool TeensyDriver::setGlobalSpeedScale(double scale)
{
    std::lock_guard<std::mutex> lock(io_mutex_);

    if (!initialised_) {
        RCLCPP_ERROR(logger_, "Cannot set speed scale: driver not initialised");
        return false;
    }

    if (scale <= 0.0 || scale > 3.0) {   // safe bounds
        RCLCPP_ERROR(logger_, "Invalid speed scale %.3f (must be 0 < scale <= 3)", scale);
        return false;
    }

    std::stringstream ss;
    ss << "SF " << scale << "\n";
    std::string cmd = ss.str();

    RCLCPP_INFO(logger_, "Sending speed scale command to Teensy: %s", cmd.c_str());

    std::string err;
    bool ok = transmit(cmd, err);

    if (!ok) {
        RCLCPP_ERROR(logger_, "Failed to send SF command: %s", err.c_str());
        return false;
    }

    return true;
}


void TeensyDriver::getJointPositions(std::vector<double>& joint_positions) {
  // get current joint positions
  std::string msg = "JP\n";
  exchange(msg);
  joint_positions = joint_positions_deg_;
}

bool TeensyDriver::resetEStop() {
  std::string msg = "RE\n";
  exchange(msg);
  return !is_estopped_;
}

bool TeensyDriver::isEStopped() { return is_estopped_; }

void TeensyDriver::getJointVelocities(std::vector<double>& joint_velocities) {
  // get current joint velocities
  std::string msg = "JV\n";
  exchange(msg);
  joint_velocities = joint_velocities_deg_;
}

bool TeensyDriver::sendCommand(std::string outMsg) { return exchange(outMsg); }

// Send msg to board and collect data
bool TeensyDriver::exchange(std::string outMsg) {
  std::string inMsg;
  std::string errTransmit = "";

  // RCLCPP_INFO(logger_, "Sending message: %s", outMsg.c_str());
  if (!transmit(outMsg, errTransmit)) {
    RCLCPP_ERROR(logger_, "Error in transmit: %s", errTransmit.c_str());
    return false;
  }

  while (true) {
    receive(inMsg);
    std::string header = inMsg.substr(0, 2);

    if (header == "DB") {
      // debug message
      RCLCPP_DEBUG(logger_, "Debug message: %s", inMsg.c_str());
    } else if (header == "WN") {
      // warning message
      RCLCPP_WARN(logger_, "Warning: %s", inMsg.c_str());
    } else {
      if (header == "ST") {
        // init acknowledgement
        checkInit(inMsg);
      } else if (header == "JC") {
        // encoder calibration values
        updateEncoderCalibrations(inMsg);
      } else if (header == "JP") {
        // encoder steps
        updateJointPositions(inMsg);
      } else if (header == "JV") {
        // encoder steps
        updateJointVelocities(inMsg);
      } else if (header == "ES") {
        // estop status
        updateEStopStatus(inMsg);
      } else if (header == "ER") {
        // error message
        RCLCPP_INFO(logger_, "ERROR message: %s", inMsg.c_str());
        return false;
      } else {
        // unknown header
        RCLCPP_WARN(logger_, "Unknown header %s", header.c_str());
        RCLCPP_WARN(logger_, "Unknown header (full header:) %s", inMsg.c_str());

        return false;
      }
      return true;
    }
  }
  return true;
}

bool TeensyDriver::transmit(std::string msg, std::string& err) {
  boost::system::error_code ec;
  const auto sendBuffer = boost::asio::buffer(msg.c_str(), msg.size());
  // RCLCPP_INFO(logger_, "before write to serial: %s", msg.c_str());
  boost::asio::write(serial_port_, sendBuffer, ec);
  // RCLCPP_INFO(logger_, "after write to serial: %s", msg.c_str());
  if (!ec) {
    return true;
  } else {
    err = ec.message();
    return false;
  }
}

void TeensyDriver::receive(std::string& inMsg) {
  char c;
  std::string msg = "";
  bool eol = false;
  while (!eol) {
    boost::asio::read(serial_port_, boost::asio::buffer(&c, 1));
    switch (c) {
      case '\r':
        break;
      case '\n':
        eol = true;
        break;
      default:
        msg += c;
    }
  }
  inMsg = msg;
}

void TeensyDriver::checkInit(std::string msg) {
  std::size_t ack_idx = msg.find("A", 2) + 1;
  std::size_t version_idx = msg.find("B", 2) + 1;
  std::size_t ar_model_matched_idx = msg.find("C", 2) + 1;
  std::size_t ar_model_idx = msg.find("D", 2) + 1;
  int ack = std::stoi(msg.substr(ack_idx, version_idx));
  int ar_model_matched =
      std::stoi(msg.substr(ar_model_matched_idx, ar_model_idx));
  if (!ack) {
    std::string version = msg.substr(version_idx);
    RCLCPP_ERROR(logger_, "Firmware version mismatch %s", version.c_str());
  }
  if (!ar_model_matched) {
    std::string ar_model = msg.substr(ar_model_idx);
    RCLCPP_ERROR(logger_, "Model mismatch %s", ar_model.c_str());
  }
  if (ack && ar_model_matched) {
    initialised_ = true;
  }
}

void TeensyDriver::updateJointPositions(const std::string msg) {
  parseValuesToVector(msg, joint_positions_deg_);
}

void TeensyDriver::updateJointVelocities(const std::string msg) {
  parseValuesToVector(msg, joint_velocities_deg_);
}

void TeensyDriver::updateEStopStatus(std::string msg) {
  is_estopped_ = msg.substr(2) == "1" ? true : false;
}

void TeensyDriver::updateEncoderCalibrations(const std::string msg) {
  parseValuesToVector(msg, enc_calibrations_);
}

template <typename T>
void TeensyDriver::parseValuesToVector(const std::string msg,
                                       std::vector<T>& values) {
  values.clear();
  size_t prevIdx = msg.find('A', 2) + 1;

  for (size_t i = 1;; ++i) {
    char currentIdentifier = 'A' + i;
    size_t currentIdx = msg.find(currentIdentifier, 2);

    try {
      if (currentIdx == std::string::npos) {
        if constexpr (std::is_same<T, int>::value) {
          values.push_back(std::stoi(msg.substr(prevIdx)));
        } else if constexpr (std::is_same<T, double>::value) {
          values.push_back(std::stod(msg.substr(prevIdx)));
        }
        break;
      }
      if constexpr (std::is_same<T, int>::value) {
        values.push_back(std::stoi(msg.substr(prevIdx, currentIdx - prevIdx)));
      } else if constexpr (std::is_same<T, double>::value) {
        values.push_back(std::stod(msg.substr(prevIdx, currentIdx - prevIdx)));
      }
    } catch (const std::invalid_argument&) {
      RCLCPP_WARN(logger_, "Invalid argument, can't parse %s", msg.c_str());
    }
    prevIdx = currentIdx + 1;
  }
}


bool TeensyDriver::moveServoToAngle(double angle_deg) {
  
  auto clamp = [](double v, double lo, double hi) {
      if (v < lo) return lo;
      if (v > hi) return hi;
      return v;
  };

  // clamp for safety
  if (std::isnan(angle_deg)) angle_deg = 0.0;
  angle_deg = clamp(angle_deg, 0.0, 180.0);

  std::ostringstream oss;
  // Example protocol: "SV" + integer degrees + newline
  oss << "SA" << static_cast<int>(std::round(angle_deg)) << "\n";

  // RCLCPP_INFO(logger_, "Requesting servo angle: %.2f deg (cmd='%s')",
  //             angle_deg, oss.str().c_str());

  std::string err;
  bool retval= transmit(oss.str(),err);

  // RCLCPP_INFO(logger_, "Request sent");
  return retval;
  // If your firmware just ACKs and you don’t need a state update parsed,
  // you could use transmit() like OG/CG optimization, but sendCommand()
  // is safer because it reads the response and error headers.
}



bool TeensyDriver::openGripper() {
  std::lock_guard<std::mutex> lk(io_mutex_);
  std::string err;
  RCLCPP_INFO(logger_, "Opening gripper...");
  bool retval =  transmit("OG\n",err);
  return retval;
}

bool TeensyDriver::closeGripper() {
  std::lock_guard<std::mutex> lk(io_mutex_);
  std::string err;
  RCLCPP_INFO(logger_, "Closing gripper...");
  bool retval =  transmit("CG\n",err);
  return retval;
}

bool TeensyDriver::nudgeJointSteps(int joint_idx, int steps) {
  RCLCPP_INFO(logger_, "Received nudge request: joint_idx=%d, steps=%d", joint_idx, steps);

  if (!serial_port_.is_open()) {
    RCLCPP_ERROR(logger_, "Serial port is not open; cannot send HM.");
    return false;
  }

  if (joint_idx < 0 || joint_idx >= num_joints_) {
    RCLCPP_ERROR(logger_, "HM joint_idx %d out of range [0,%d).", joint_idx, num_joints_);
    return false;
  }

  // Build "HM a b c d e f\n" with zeros except the selected joint
  std::array<int, 6> deltas{};
  deltas.fill(0);
  deltas[static_cast<size_t>(joint_idx)] = steps;

  std::ostringstream oss;
  oss << "HM ";
  for (size_t i = 0; i < deltas.size(); ++i) {
    oss << deltas[i];
    if (i + 1 < deltas.size()) oss << ' ';
  }
  oss << "\n";

  RCLCPP_DEBUG(logger_, "Constructed HM command: '%s'", oss.str().c_str());

  {
    std::lock_guard<std::mutex> lk(io_mutex_);
    RCLCPP_INFO(logger_, "Sending HM command to Teensy...");
    bool result = exchangeHM(oss.str());
    if (result) {
      RCLCPP_INFO(logger_, "HM command acknowledged by Teensy.");
    } else {
      RCLCPP_WARN(logger_, "HM command failed or no acknowledgment from Teensy.");
    }
    return result;
  }
}


// Send HM and read until we see an HM completion line or an error.
// Note: firmware prints free-form lines like:
//   "Moving joint 3 by 5 steps."
//   "HM: Manual move complete. New position set as home."
//   "EEPROM updated."
bool TeensyDriver::exchangeHM(const std::string& outMsg) {
  std::string err;
  RCLCPP_INFO(logger_, "About to write HM to serial: %s", outMsg.c_str());
  if (!transmit(outMsg, err)) {
    RCLCPP_ERROR(logger_, "HM transmit failed: %s", err.c_str());
    return false;
  }
  RCLCPP_INFO(logger_, "transmit completed");
  return true;
  
}


}  // namespace annin_ar4_driver
