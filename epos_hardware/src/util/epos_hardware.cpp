#include <epos_hardware/epos_hardware.h>

#include <boost/foreach.hpp>

namespace epos_hardware {

EposHardware::EposHardware() {}

EposHardware::~EposHardware() {}

bool EposHardware::init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh,
                        const std::vector< std::string > &motor_names) {
  // register hardware interfaces
  registerInterface(&asi);
  registerInterface(&avi);
  registerInterface(&api);
  registerInterface(&aei);
  registerInterface(&bsi);

  // init motors
  // - register state/command handles to hardware interfaces
  // - configure motors
  try {
    epos_manager_.init(*this, root_nh, hw_nh, motor_names);
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(error.what());
    return false;
  }

  //
  try {
    transmission_loader.reset(
        new transmission_interface::TransmissionInterfaceLoader(this, &robot_transmissions));
  } catch (const std::invalid_argument &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  } catch (const pluginlib::LibraryLoadException &ex) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. " << ex.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("Failed to create transmission interface loader. ");
    return false;
  }

  //
  std::string urdf_string;
  root_nh.getParam("robot_description", urdf_string);
  while (urdf_string.empty() && ros::ok()) {
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    root_nh.getParam("robot_description", urdf_string);
    ros::Duration(0.1).sleep();
  }

  //
  transmission_interface::TransmissionParser parser;
  std::vector< transmission_interface::TransmissionInfo > infos;
  if (!parser.parse(urdf_string, infos)) {
    ROS_ERROR("Error parsing URDF");
    return false;
  }

  // build a list of all loaded actuator names
  const std::vector< std::string > actuator_names(epos_manager_.motorNames());

  // Load all transmissions that are for the loaded motors
  BOOST_FOREACH (const transmission_interface::TransmissionInfo &info, infos) {
    // find loaded motors associated with the transmission
    std::size_t n_found(0);
    BOOST_FOREACH (const transmission_interface::ActuatorInfo &actuator, info.actuators_) {
      if (std::find(actuator_names.begin(), actuator_names.end(), actuator.name_) !=
          actuator_names.end()) {
        ++n_found;
      }
    }
    // no motors for the transmission found. skip.
    if (n_found == 0) {
      continue;
    }
    // some motors for the transmission found. not supported.
    if (n_found < info.actuators_.size()) {
      ROS_ERROR_STREAM(
          "Do not support transmissions that contain only some EPOS actuators: " << info.name_);
      continue;
    }
    // all motors for the transmission found. try load the transmission.
    if (!transmission_loader->load(info)) {
      ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
      return false;
    }
    ROS_INFO_STREAM("Loaded transmission: " << info.name_);
  }

  return true;
}

void EposHardware::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                            const std::list< hardware_interface::ControllerInfo > &stop_list) {
  epos_manager_.doSwitch(start_list, stop_list);
}

void EposHardware::read() {
  epos_manager_.read();
  if (robot_transmissions.get< transmission_interface::ActuatorToJointStateInterface >()) {
    robot_transmissions.get< transmission_interface::ActuatorToJointStateInterface >()->propagate();
  }
}

void EposHardware::write() {
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

void EposHardware::updateDiagnostics() { epos_manager_.updateDiagnostics(); }

} // namespace epos_hardware
