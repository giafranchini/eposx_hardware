#ifndef EPOS_HARDWARE_EPOS_MANAGER_H_
#define EPOS_HARDWARE_EPOS_MANAGER_H_

#include <list>
#include <string>
#include <vector>

#include "epos_hardware/epos.h"
#include "epos_hardware/utils.h"
#include <battery_state_interface/battery_state_interface.hpp>
#include <diagnostic_updater/diagnostic_updater.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>

namespace epos_hardware {

class EposManager {
public:
  EposManager(hardware_interface::ActuatorStateInterface &asi,
              hardware_interface::VelocityActuatorInterface &avi,
              hardware_interface::PositionActuatorInterface &api,
              hardware_interface::EffortActuatorInterface &aei,
              battery_state_interface::BatteryStateInterface &bsi, ros::NodeHandle &nh,
              ros::NodeHandle &pnh, const std::vector< std::string > &motor_names);
  bool init();
  void read();
  void write();
  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list);
  void update_diagnostics();
  std::vector< boost::shared_ptr< Epos > > motors() { return motors_; };

private:
  std::vector< boost::shared_ptr< Epos > > motors_;

  hardware_interface::ActuatorStateInterface *asi_;
  hardware_interface::VelocityActuatorInterface *avi_;
  hardware_interface::PositionActuatorInterface *api_;
  hardware_interface::EffortActuatorInterface *aei_;
  battery_state_interface::BatteryStateInterface *bsi_;
};

} // namespace epos_hardware

#endif
