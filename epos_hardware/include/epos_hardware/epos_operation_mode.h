#ifndef EPOS_HARDWARE_EPOS_OPERATION_MODE_H
#define EPOS_HARDWARE_EPOS_OPERATION_MODE_H

#include <epos_hardware/utils.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

namespace epos_hardware {

class EposOperationMode {
public:
  EposOperationMode();
  virtual ~EposOperationMode();

  // configure operation mode (e.g. register command handle or load parameters)
  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh,
                    const std::string &motor_name, epos_hardware::NodeHandle &epos_handle) = 0;

  // activate operation mode
  virtual void activate() = 0;

  // read something required for operation mode
  virtual void read() = 0;

  // write commands of operation mode
  virtual void write() = 0;
};

class EposProfilePositionMode : public EposOperationMode {
public:
  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh,
                    const std::string &motor_name, epos_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  epos_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  int encoder_resolution_;
  double position_cmd_;
};

class EposProfileVelocityMode : public EposOperationMode {
public:
  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh,
                    const std::string &motor_name, epos_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  epos_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  bool halt_velocity_;
  double velocity_cmd_;
};

class EposCurrentMode : public EposOperationMode {
public:
  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh,
                    const std::string &motor_name, epos_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  epos_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  double torque_constant_;
  double effort_cmd_;
};

class EposCyclicSynchronoustTorqueMode : public EposOperationMode {
public:
  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh,
                    const std::string &motor_name, epos_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  epos_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  double effort_cmd_;
};

} // namespace epos_hardware

#endif