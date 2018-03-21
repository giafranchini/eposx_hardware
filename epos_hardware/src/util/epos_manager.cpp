#include <epos_hardware/epos_manager.h>

#include <boost/foreach.hpp>

namespace epos_hardware {

EposManager::EposManager(ros::NodeHandle &nh, ros::NodeHandle &pnh,
                         const std::vector< std::string > &motor_names,
                         hardware_interface::ActuatorStateInterface &asi,
                         hardware_interface::VelocityActuatorInterface &avi,
                         hardware_interface::PositionActuatorInterface &api,
                         hardware_interface::EffortActuatorInterface &aei,
                         battery_state_interface::BatteryStateInterface &bsi) {
  BOOST_FOREACH (const std::string &motor_name, motor_names) {
    ROS_INFO_STREAM("Loading EPOS: " << motor_name);
    ros::NodeHandle motor_config_nh(pnh, motor_name);
    boost::shared_ptr< Epos > motor(
        new Epos(nh, motor_config_nh, motor_name, asi, avi, api, aei, bsi));
    motors_.push_back(motor);
  }
}

bool EposManager::init() {
  bool success = true;
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) {
    if (!motor->init()) {
      ROS_ERROR_STREAM("Could not configure motor: " << motor->motorName());
      success = false;
    }
  }
  return success;
}

void EposManager::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                           const std::list< hardware_interface::ControllerInfo > &stop_list) {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) {
    motor->doSwitch(start_list, stop_list);
  }
}

void EposManager::read() {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) { motor->read(); }
}

void EposManager::write() {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) { motor->write(); }
}

void EposManager::updateDiagnostics() {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) { motor->updateDiagnostics(); }
}

} // namespace epos_hardware
