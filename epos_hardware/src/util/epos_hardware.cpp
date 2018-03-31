#include <set>
#include <stdexcept>

#include <epos_hardware/epos_hardware.h>
#include <epos_hardware/utils.h>
#include <ros/console.h>

#include <boost/foreach.hpp>

namespace epos_hardware {

EposHardware::EposHardware() {}

EposHardware::~EposHardware() {}

//
// init()
//

bool EposHardware::init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh,
                        const std::vector< std::string > &motor_names) {
  try {
    initHardwareInterfaces();
    initMotors(root_nh, hw_nh, motor_names);
    initTransmissions(root_nh);
    // TODO: initJointLimitInterface
  } catch (const std::exception &error) {
    ROS_ERROR_STREAM(error.what());
    return false;
  }
  return true;
}

void EposHardware::initHardwareInterfaces() {
  // register hardware interfaces
  registerInterface(&asi);
  registerInterface(&avi);
  registerInterface(&api);
  registerInterface(&aei);
  registerInterface(&bsi);
  registerInterface(&edi);
}

void EposHardware::initMotors(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh,
                              const std::vector< std::string > &motor_names) {
  // register state/command/diagnostic handles to hardware interfaces
  // and configure motors
  epos_manager_.init(*this, root_nh, hw_nh, motor_names);
}

// helper function to populate actuator names registered in interfaces
template < typename ActuatorInterface >
void insertNames(std::set< std::string > &names, const ActuatorInterface &interface) {
  const std::vector< std::string > new_names(interface.getNames());
  names.insert(new_names.begin(), new_names.end());
}

void EposHardware::initTransmissions(ros::NodeHandle &root_nh) {
  // wait for URDF which contains transmission information
  std::string urdf_str;
  root_nh.getParam("robot_description", urdf_str);
  while (urdf_str.empty() && ros::ok()) {
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    root_nh.getParam("robot_description", urdf_str);
    ros::Duration(0.1).sleep();
  }

  // load transmission infomations from URDF
  transmission_interface::TransmissionParser trans_parser;
  std::vector< transmission_interface::TransmissionInfo > trans_infos;
  if (!trans_parser.parse(urdf_str, trans_infos)) {
    throw EposException("Failed to parse urdf");
  }

  // build a list of actuator names in this hardware
  std::set< std::string > hw_actuator_names;
  insertNames(hw_actuator_names, asi);
  insertNames(hw_actuator_names, api);
  insertNames(hw_actuator_names, avi);
  insertNames(hw_actuator_names, aei);

  // load all transmissions that are for the motors in this hardware
  transmission_loader.reset(
      new transmission_interface::TransmissionInterfaceLoader(this, &robot_transmissions));
  BOOST_FOREACH (const transmission_interface::TransmissionInfo &trans_info, trans_infos) {
    // check the transmission is for some of actuators in this hardware
    BOOST_FOREACH (const transmission_interface::ActuatorInfo &trans_actuator,
                   trans_info.actuators_) {
      if (hw_actuator_names.count(trans_actuator.name_) == 0) {
        ROS_INFO_STREAM("Skip loading " << trans_info.name_);
        continue;
      }
    }
    // load the transmission
    if (!transmission_loader->load(trans_info)) {
      throw EposException("Failed to load " + trans_info.name_);
    }
    ROS_INFO_STREAM("Loaded transmission: " << trans_info.name_);
  }
}

//
// doSwitch()
//

void EposHardware::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                            const std::list< hardware_interface::ControllerInfo > &stop_list) {
  epos_manager_.doSwitch(start_list, stop_list);
}

//
// read()
//

void EposHardware::read() {
  epos_manager_.read();
  if (robot_transmissions.get< transmission_interface::ActuatorToJointStateInterface >()) {
    robot_transmissions.get< transmission_interface::ActuatorToJointStateInterface >()->propagate();
  }
}

//
// write()
//

void EposHardware::write() {
  // TODO: enforce limits to joint command interfaces
  if (robot_transmissions.get< transmission_interface::JointToActuatorVelocityInterface >()) {
    robot_transmissions.get< transmission_interface::JointToActuatorVelocityInterface >()
        ->propagate();
  }
  if (robot_transmissions.get< transmission_interface::JointToActuatorPositionInterface >()) {
    robot_transmissions.get< transmission_interface::JointToActuatorPositionInterface >()
        ->propagate();
  }
  if (robot_transmissions.get< transmission_interface::JointToActuatorEffortInterface >()) {
    robot_transmissions.get< transmission_interface::JointToActuatorEffortInterface >()
        ->propagate();
  }
  epos_manager_.write();
}

//
// updateDiagnostics()
//

void EposHardware::updateDiagnostics() { epos_manager_.updateDiagnostics(); }

} // namespace epos_hardware
