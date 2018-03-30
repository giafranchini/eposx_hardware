#ifndef EPOS_HARDWARE_EPOS_H_
#define EPOS_HARDWARE_EPOS_H_

#include <list>
#include <map>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.h>
#include <epos_hardware/utils.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <sensor_msgs/BatteryState.h>

#include <boost/cstdint.hpp>

namespace epos_hardware {

class Epos {
public:
  Epos();
  virtual ~Epos();

  void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);
  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list);
  void read();
  void write();
  void updateDiagnostics();

  std::string motorName() const { return motor_name_; }

private:
  // subfunctions for init()
  void initHardwareInterface(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh);
  void initEposNodeHandle(ros::NodeHandle &motor_nh);
  void initFaultReaction(ros::NodeHandle &motor_nh);
  void initOperationMode(ros::NodeHandle &motor_nh);
  void initMotorParameter(ros::NodeHandle &motor_nh);
  void initSensorParameter(ros::NodeHandle &motor_nh);
  void initSafetyParameter(ros::NodeHandle &motor_nh);
  void initPositionRegulator(ros::NodeHandle &motor_nh);
  void initVelocityRegulator(ros::NodeHandle &motor_nh);
  void initCurrentRegulator(ros::NodeHandle &motor_nh);
  void initPositionProfile(ros::NodeHandle &motor_nh);
  void initVelocityProfile(ros::NodeHandle &motor_nh);
  void initDeviceError(ros::NodeHandle &motor_nh);
  void initDiagnostic(ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh);
  void initMiscParameters(ros::NodeHandle &motor_nh);

  // subfunctions for read()
  void readJointState();
  void readPowerSupply();
  void readDiagnostic();

  // subfunctions for write()
  void writePositionCommand();
  void writeVelocityCommand();
  void writeCurrentCommand();

  // callbacks for updateDiagnostics()
  void updateMotorDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void updateMotorOutputDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

  // current [A] <-> turque [mNm]
  double currentToTorque(const double current) { return current * torque_constant_; }
  double torqueToCurrent(const double torque) { return torque / torque_constant_; }

private:
  enum OperationMode { PROFILE_POSITION_MODE = 1, PROFILE_VELOCITY_MODE = 3, CURRENT_MODE = -3 };

  // TODO: describe member variables

  std::string motor_name_;

  bool has_init_;

  epos_hardware::NodeHandle epos_handle_;
  std::map< std::string, OperationMode > operation_mode_map_;
  OperationMode operation_mode_;

  diagnostic_updater::Updater diagnostic_updater_;

  // state: epos -> ros
  double position_;
  double velocity_;
  double effort_;
  double current_;
  sensor_msgs::BatteryState power_supply_state_;
  boost::uint16_t statusword_;
  std::vector< unsigned int > device_errors_;

  // command: ros -> epos
  double position_cmd_;
  double velocity_cmd_;
  double torque_cmd_;

  bool rw_ros_units_;
  int max_profile_velocity_;
  bool halt_velocity_;
  double torque_constant_;
  double nominal_current_;
  double max_current_;
  int encoder_resolution_;
};
} // namespace epos_hardware

#endif
