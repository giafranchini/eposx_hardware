#ifndef EPOS_HARDWARE_EPOS_HARDWARE_H_
#define EPOS_HARDWARE_EPOS_HARDWARE_H_

#include <list>
#include <string>
#include <vector>

#include "epos_hardware/epos.h"
#include "epos_hardware/epos_manager.h"
#include "epos_hardware/utils.h"
#include <battery_state_interface/battery_state_interface.hpp>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>

#include <boost/scoped_ptr.hpp>

namespace epos_hardware {

class EposHardware : public hardware_interface::RobotHW {
public:
  EposHardware(ros::NodeHandle &nh, ros::NodeHandle &pnh,
               const std::vector< std::string > &motor_names);

  // TODO: use common methods standardized in RobotHW
  bool init();
  virtual void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                        const std::list< hardware_interface::ControllerInfo > &stop_list);
  void read();
  void write();
  void updateDiagnostics();

private:
  hardware_interface::ActuatorStateInterface asi;
  hardware_interface::VelocityActuatorInterface avi;
  hardware_interface::PositionActuatorInterface api;
  hardware_interface::EffortActuatorInterface aei;
  battery_state_interface::BatteryStateInterface bsi;

  EposManager epos_manager_;

  transmission_interface::RobotTransmissions robot_transmissions;
  boost::scoped_ptr< transmission_interface::TransmissionInterfaceLoader > transmission_loader;
};

} // namespace epos_hardware

#endif
