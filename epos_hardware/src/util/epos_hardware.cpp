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
    initLowLevelInterfaces();
    initMotors(root_nh, hw_nh, motor_names);
    initTransmissions(root_nh);
    // TODO: initJointLimitInterface
  } catch (const std::exception &error) {
    ROS_ERROR_STREAM(error.what());
    return false;
  }
  return true;
}

void EposHardware::initLowLevelInterfaces() {
  registerInterface(&ator_state_iface_);
  registerInterface(&pos_ator_iface_);
  registerInterface(&vel_ator_iface_);
  registerInterface(&eff_ator_iface_);
  registerInterface(&bat_state_iface_);
  registerInterface(&epos_diag_iface_);
}

void EposHardware::initMotors(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh,
                              const std::vector< std::string > &motor_names) {
  // register state/command/diagnostic handles to hardware interfaces
  // and configure motors
  epos_manager_.init(*this, root_nh, hw_nh, motor_names);
}

// helper function to populate actuator names registered in interfaces
template < typename ActuatorInterface >
void insertNames(std::set< std::string > &names, const ActuatorInterface &ator_iface) {
  const std::vector< std::string > new_names(ator_iface.getNames());
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
  std::set< std::string > hw_ator_names;
  insertNames(hw_ator_names, ator_state_iface_);
  insertNames(hw_ator_names, pos_ator_iface_);
  insertNames(hw_ator_names, vel_ator_iface_);
  insertNames(hw_ator_names, eff_ator_iface_);

  // load all transmissions that are for the motors in this hardware
  trans_iface_loader_.reset(
      new transmission_interface::TransmissionInterfaceLoader(this, &robot_trans_));
  BOOST_FOREACH (const transmission_interface::TransmissionInfo &trans_info, trans_infos) {
    // check the transmission is for some of actuators in this hardware
    bool trans_has_non_epos_ator(false);
    BOOST_FOREACH (const transmission_interface::ActuatorInfo &trans_ator, trans_info.actuators_) {
      if (hw_ator_names.count(trans_ator.name_) == 0) {
        trans_has_non_epos_ator = true;
        break;
      }
    }
    if (trans_has_non_epos_ator) {
      ROS_INFO_STREAM("Skip loading " << trans_info.name_ << " because it has non epos actuator");
      continue;
    }
    // load the transmission
    if (!trans_iface_loader_->load(trans_info)) {
      throw EposException("Failed to load " + trans_info.name_);
    }
    ROS_INFO_STREAM("Loaded transmission: " << trans_info.name_);
  }
}

void EposHardware::initJointLimits() {
  // TODO:
  //   - bind all joints to joint limits
  //   - init each limits by URDF
  // NOTE: <how to access joint interfaces>
  //       transmission_loader->getData()->joint_interfaces.position_joint_interface;
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

// helper function to propagate between joint and actuator states/commands
template < typename BridgeInterface >
void propagate(transmission_interface::RobotTransmissions &robot_trans) {
  BridgeInterface *const bridge_iface(robot_trans.get< BridgeInterface >());
  if (bridge_iface) {
    bridge_iface->propagate();
  }
}

void EposHardware::read() {
  epos_manager_.read();

  propagate< transmission_interface::ActuatorToJointStateInterface >(robot_trans_);
}

//
// write()
//

void EposHardware::write() {
  // TODO:
  //   - update joint limit values from parameter server
  //     (must be fast. use cache or fetch in background)
  //   - enforce limits to joint commands

  propagate< transmission_interface::JointToActuatorVelocityInterface >(robot_trans_);
  propagate< transmission_interface::JointToActuatorPositionInterface >(robot_trans_);
  propagate< transmission_interface::JointToActuatorEffortInterface >(robot_trans_);

  epos_manager_.write();
}

//
// updateDiagnostics()
//

void EposHardware::updateDiagnostics() { epos_manager_.updateDiagnostics(); }

} // namespace epos_hardware
