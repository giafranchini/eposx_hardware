#ifndef EPOS_HARDWARE_EPOS_H_
#define EPOS_HARDWARE_EPOS_H_

#include <list>
#include <map>
#include <string>
#include <vector>

#include <battery_state_interface/battery_state_interface.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <epos_hardware/utils.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

namespace epos_hardware {

class Epos {
public:
  typedef enum {
    PROFILE_POSITION_MODE = 1,
    PROFILE_VELOCITY_MODE = 3,
    CURRENT_MODE = -3
  } OperationMode;

  Epos(const std::string &name, ros::NodeHandle &nh, ros::NodeHandle &config_nh,
       hardware_interface::ActuatorStateInterface &asi,
       hardware_interface::VelocityActuatorInterface &avi,
       hardware_interface::PositionActuatorInterface &api,
       hardware_interface::EffortActuatorInterface &aei,
       battery_state_interface::BatteryStateInterface &bsi);
  ~Epos();
  bool init();
  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list);
  void read();
  void write();
  std::string name() { return name_; }
  std::string actuator_name() { return actuator_name_; }
  void update_diagnostics();

private:
  void buildMotorStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void buildMotorOutputStatus(diagnostic_updater::DiagnosticStatusWrapper &stat);

  double currentToTorque(double current) { return current * torque_constant_; }
  double torqueToCurrent(double torque) { return torque / torque_constant_; }

private:
  ros::NodeHandle config_nh_;
  diagnostic_updater::Updater diagnostic_updater_;
  std::string name_;
  std::string actuator_name_;
  uint64_t serial_number_;
  std::map< std::string, OperationMode > operation_mode_map_;
  OperationMode operation_mode_;
  epos_hardware::NodeHandle node_handle_;
  bool valid_;
  bool has_init_;
  bool rw_ros_units_;

  // epos -> ros
  double position_;
  double velocity_;
  double effort_;
  double current_;
  boost::uint16_t statusword_;
  std::vector< unsigned int > device_errors_;

  // ros -> epos
  double position_cmd_;
  double velocity_cmd_;
  double torque_cmd_;
  int max_profile_velocity_;
  bool halt_velocity_;
  double torque_constant_;
  double nominal_current_;
  double max_current_;
  int encoder_resolution_;

  std::string power_supply_name_;
  sensor_msgs::BatteryState power_supply_state_;
};
} // namespace epos_hardware

#endif
