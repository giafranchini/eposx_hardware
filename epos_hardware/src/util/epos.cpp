#include <ios>
#include <limits>
#include <sstream>

#include <epos_hardware/epos.h>

#include <boost/bind.hpp>
#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace epos_hardware {

Epos::Epos(ros::NodeHandle &nh, ros::NodeHandle &config_nh, const std::string &motor_name,
           hardware_interface::ActuatorStateInterface &asi,
           hardware_interface::VelocityActuatorInterface &avi,
           hardware_interface::PositionActuatorInterface &api,
           hardware_interface::EffortActuatorInterface &aei,
           battery_state_interface::BatteryStateInterface &bsi)
    : config_nh_(config_nh), motor_name_(motor_name), diagnostic_updater_(nh, config_nh),
      has_init_(false), position_(0), velocity_(0), effort_(0), current_(0), position_cmd_(0),
      velocity_cmd_(0), torque_cmd_(0) {
  // register hardware interfaces so that ros-controllers can find this hardware
  {
    hardware_interface::ActuatorStateHandle state_handle(motor_name_, &position_, &velocity_,
                                                         &effort_);
    asi.registerHandle(state_handle);
    hardware_interface::ActuatorHandle position_handle(state_handle, &position_cmd_);
    api.registerHandle(position_handle);
    hardware_interface::ActuatorHandle velocity_handle(state_handle, &velocity_cmd_);
    avi.registerHandle(velocity_handle);
    hardware_interface::ActuatorHandle effort_handle(state_handle, &torque_cmd_);
    aei.registerHandle(effort_handle);
  }
  // if power_supply/name is given, additionally register power supply hardware
  std::string power_supply_name;
  if (config_nh_.getParam("power_supply/name", power_supply_name)) {
    battery_state_interface::BatteryStateHandle battery_state_handle(power_supply_name,
                                                                     &power_supply_state_);
    bsi.registerHandle(battery_state_handle);
  }
}

Epos::~Epos() {
  try {
    VCS_N0(SetDisableState, epos_handle_);
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(error.what());
  }
}

//
// init() and subfunctions
//

bool Epos::init() {
  try {
    initEposNodeHandle();
    ROS_INFO_STREAM(motor_name_ << " is ready to be initialized");

    ROS_INFO_STREAM("Disabling " << motor_name_ << " to start configuration");
    VCS_N0(SetDisableState, epos_handle_);

    initOperationMode();
    initFaultReaction();
    initMotorParameter();
    initSensorParameter();
    initSafetyParameter();
    initPositionRegulator();
    initVelocityRegulator();
    initCurrentRegulator();
    initPositionProfile();
    initVelocityProfile();
    initDeviceError();
    initDiagnostic();
    initMiscParameters();

    ROS_INFO_STREAM("Enabling " << motor_name_ << " to finalize configuration");
    VCS_N0(SetEnableState, epos_handle_);
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(error.what());
    return false;
  }

  has_init_ = true;
  return true;
}

void Epos::initEposNodeHandle() {
  // get serial number
  std::string serial_number_str;
  GET_PARAM_KV(config_nh_, "serial_number", serial_number_str);
  std::istringstream iss(serial_number_str);
  boost::uint64_t serial_number;
  iss >> std::hex >> serial_number;
  if (!iss) {
    throw EposException("Invalid serial number (" + serial_number_str + ")");
  }

  // create epos handle
  // TODO: device_name, protocol_stack_name, interface_name from parameters
  epos_handle_ = createNodeHandle("EPOS4", "MAXON SERIAL V2", "USB", serial_number);
  // TODO: proper protocol stack setting according to current protocol stack
  VCS(SetProtocolStackSettings, epos_handle_.ptr.get(), 1000000, 500);
}

void Epos::initOperationMode() {
  // load mapping from ros-controllers to epos operation modes
  std::map< std::string, std::string > str_map;
  if (config_nh_.getParam("operation_mode_map", str_map)) {
    for (std::map< std::string, std::string >::const_iterator str_pair = str_map.begin();
         str_pair != str_map.end(); ++str_pair) {
      if (str_pair->second == "profile_position") {
        operation_mode_map_[str_pair->first] = PROFILE_POSITION_MODE;
      } else if (str_pair->second == "profile_velocity") {
        operation_mode_map_[str_pair->first] = PROFILE_VELOCITY_MODE;
      } else if (str_pair->second == "current") {
        operation_mode_map_[str_pair->first] = CURRENT_MODE;
      } else {
        throw EposException("Invalid operation mode (" + str_pair->second + ")");
      }
    }
  }
}

void Epos::initFaultReaction() {
  // TODO: use corrent index and subindex according to device_name
  std::string fault_reaction_str;
  if (config_nh_.getParam("fault_reaction_option", fault_reaction_str)) {
    if (fault_reaction_str == "signal_only") {
      boost::int16_t data(-1);
      VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
    } else if (fault_reaction_str == "disable_drive") {
      boost::int16_t data(0);
      VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
    } else if (fault_reaction_str == "slow_down_ramp") {
      boost::int16_t data(1);
      VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
    } else if (fault_reaction_str == "slow_down_quickstop") {
      boost::int16_t data(2);
      VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
    } else {
      throw EposException("Invalid fault reaction option (" + fault_reaction_str + ")");
    }
  }
}

void Epos::initMotorParameter() {
  ros::NodeHandle motor_nh(config_nh_, "motor");
  // set motor type
  int type;
  GET_PARAM_V(motor_nh, type);
  VCS_NN(SetMotorType, epos_handle_, type);
  // set motor parameters
  nominal_current_ = 0.;
  max_current_ = 0.;
  if (type == 1 /* DC MOTOR */) {
    double thermal_time_constant;
    GET_PARAM_KV(motor_nh, "nominal_current", nominal_current_);
    GET_PARAM_KV(motor_nh, "max_output_current", max_current_);
    GET_PARAM_V(motor_nh, thermal_time_constant);
    VCS_NN(SetDcMotorParameter, epos_handle_,
           static_cast< int >(1000 * nominal_current_),     // A -> mA
           static_cast< int >(1000 * max_current_),         // A -> mA
           static_cast< int >(10 * thermal_time_constant)); // s -> 100ms
  } else if (type == 10 || type == 11 /*EC MOTOR*/) {
    double thermal_time_constant;
    int number_of_pole_pairs;
    GET_PARAM_KV(motor_nh, "nominal_current", nominal_current_);
    GET_PARAM_KV(motor_nh, "max_output_current", max_current_);
    GET_PARAM_V(motor_nh, thermal_time_constant);
    GET_PARAM_V(motor_nh, number_of_pole_pairs);
    VCS_NN(SetEcMotorParameter, epos_handle_,
           static_cast< int >(1000 * nominal_current_),    // A -> mA
           static_cast< int >(1000 * max_current_),        // A -> mA
           static_cast< int >(10 * thermal_time_constant), // s -> 100ms
           number_of_pole_pairs);
  } else {
    throw EposException("Invalid motor type (" + boost::lexical_cast< std::string >(type) + ")");
  }
  // set motor max speed
  double max_speed;
  if (motor_nh.getParam("max_speed", max_speed)) {
    // TODO: use correct index and subindex according to device_name
    boost::uint32_t data(max_speed);
    VCS_OBJ(SetObject, epos_handle_, 0x6080, 0x00, &data, 4);
  }
}

void Epos::initSensorParameter() {
  ros::NodeHandle sensor_nh(config_nh_, "sensor");
  // set sensor type
  int type;
  GET_PARAM_V(sensor_nh, type);
  VCS_NN(SetSensorType, epos_handle_, type);
  // set sensor parameters (TODO: support hall sensors)
  encoder_resolution_ = 0;
  if (type == 1 || type == 2 /* INC ENCODER */) {
    bool inverted_polarity;
    GET_PARAM_KV(sensor_nh, "resolution", encoder_resolution_);
    GET_PARAM_V(sensor_nh, inverted_polarity);
    VCS_NN(SetIncEncoderParameter, epos_handle_, encoder_resolution_, inverted_polarity);
    if (inverted_polarity) {
      encoder_resolution_ = -encoder_resolution_;
    }
  } else if (type == 4 || type == 5 /* SSI ABS ENCODER */) {
    int data_rate, number_of_multiturn_bits, number_of_singleturn_bits;
    bool inverted_polarity;
    GET_PARAM_V(sensor_nh, data_rate);
    GET_PARAM_V(sensor_nh, number_of_multiturn_bits);
    GET_PARAM_V(sensor_nh, number_of_singleturn_bits);
    GET_PARAM_V(sensor_nh, inverted_polarity);
    VCS_NN(SetSsiAbsEncoderParameter, epos_handle_, data_rate, number_of_multiturn_bits,
           number_of_singleturn_bits, inverted_polarity);
    if (inverted_polarity) {
      encoder_resolution_ = -(1 << number_of_singleturn_bits);
    } else {
      encoder_resolution_ = (1 << number_of_singleturn_bits);
    }
  } else {
    throw EposException("Invalid sensor type (" + boost::lexical_cast< std::string >(type) + ")");
  }
}

void Epos::initSafetyParameter() {
  ros::NodeHandle safety_nh(config_nh_, "safety");

  int max_following_error;
  GET_PARAM_V(safety_nh, max_following_error);
  VCS_NN(SetMaxFollowingError, epos_handle_, max_following_error);

  GET_PARAM_KV(safety_nh, "max_profile_velocity", max_profile_velocity_);
  VCS_NN(SetMaxProfileVelocity, epos_handle_, max_profile_velocity_);

  int max_acceleration;
  GET_PARAM_V(safety_nh, max_acceleration);
  VCS_NN(SetMaxAcceleration, epos_handle_, max_acceleration);
}

void Epos::initPositionRegulator() {
  ros::NodeHandle position_regulator_nh(config_nh_, "position_regulator");
  if (position_regulator_nh.hasParam("gain")) {
    ros::NodeHandle gain_nh(position_regulator_nh, "gain");
    int p, i, d;
    GET_PARAM_V(gain_nh, p);
    GET_PARAM_V(gain_nh, i);
    GET_PARAM_V(gain_nh, d);
    VCS_NN(SetPositionRegulatorGain, epos_handle_, p, i, d);
  }
  if (position_regulator_nh.hasParam("feed_forward")) {
    ros::NodeHandle feed_forward_nh(position_regulator_nh, "feed_forward");
    int velocity, acceleration;
    GET_PARAM_V(feed_forward_nh, velocity);
    GET_PARAM_V(feed_forward_nh, acceleration);
    VCS_NN(SetPositionRegulatorFeedForward, epos_handle_, velocity, acceleration);
  }
}

void Epos::initVelocityRegulator() {
  ros::NodeHandle velocity_regulator_nh(config_nh_, "velocity_regulator");
  if (velocity_regulator_nh.hasParam("gain")) {
    ros::NodeHandle gain_nh(velocity_regulator_nh, "gain");
    int p, i;
    GET_PARAM_V(gain_nh, p);
    GET_PARAM_V(gain_nh, i);
    VCS_NN(SetVelocityRegulatorGain, epos_handle_, p, i);
  }
  if (velocity_regulator_nh.hasParam("feed_forward")) {
    ros::NodeHandle feed_forward_nh(velocity_regulator_nh, "feed_forward");
    int velocity, acceleration;
    GET_PARAM_V(feed_forward_nh, velocity);
    GET_PARAM_V(feed_forward_nh, acceleration);
    VCS_NN(SetVelocityRegulatorFeedForward, epos_handle_, velocity, acceleration);
  }
}

void Epos::initCurrentRegulator() {
  ros::NodeHandle current_regulator_nh(config_nh_, "current_regulator");
  if (current_regulator_nh.hasParam("gain")) {
    ros::NodeHandle gain_nh(current_regulator_nh, "gain");
    int p, i;
    GET_PARAM_V(gain_nh, p);
    GET_PARAM_V(gain_nh, i);
    VCS_NN(SetCurrentRegulatorGain, epos_handle_, p, i);
  }
}

void Epos::initPositionProfile() {
  ros::NodeHandle position_profile_nh(config_nh_, "position_profile");
  if (position_profile_nh.hasParam("velocity")) {
    int velocity, acceleration, deceleration;
    GET_PARAM_V(position_profile_nh, velocity);
    GET_PARAM_V(position_profile_nh, acceleration);
    GET_PARAM_V(position_profile_nh, deceleration);
    VCS_NN(SetPositionProfile, epos_handle_, velocity, acceleration, deceleration);
  }
  if (position_profile_nh.hasParam("window")) {
    ros::NodeHandle window_nh(position_profile_nh, "window");
    int window;
    double time;
    GET_PARAM_V(window_nh, window);
    GET_PARAM_V(window_nh, time);
    VCS_NN(EnablePositionWindow, epos_handle_, window,
           static_cast< int >(1000 * time) /* s -> ms */);
  }
}

void Epos::initVelocityProfile() {
  ros::NodeHandle velocity_profile_nh(config_nh_, "velocity_profile");
  if (velocity_profile_nh.hasParam("acceleration")) {
    int acceleration, deceleration;
    GET_PARAM_V(velocity_profile_nh, acceleration);
    GET_PARAM_V(velocity_profile_nh, deceleration);
    VCS_NN(SetVelocityProfile, epos_handle_, acceleration, deceleration);
  }
  if (velocity_profile_nh.hasParam("window")) {
    ros::NodeHandle window_nh(velocity_profile_nh, "window");
    int window;
    double time;
    GET_PARAM_V(window_nh, window);
    GET_PARAM_V(window_nh, time);
    VCS_NN(EnableVelocityWindow, epos_handle_, window,
           static_cast< int >(1000 * time) /* s -> ms */);
  }
}

void Epos::initDeviceError() {
  unsigned char num_device_errors;
  VCS_NN(GetNbOfDeviceError, epos_handle_, &num_device_errors);
  for (int i = 1; i <= num_device_errors; ++i) {
    unsigned int device_error_code;
    VCS_NN(GetDeviceErrorCode, epos_handle_, i, &device_error_code);
    ROS_WARN_STREAM("EPOS Device Error: 0x" << std::hex << device_error_code);
  }

  if (config_nh_.param("clear_faults", false)) {
    VCS_N0(ClearFault, epos_handle_);
  }

  VCS_NN(GetNbOfDeviceError, epos_handle_, &num_device_errors);
  if (num_device_errors > 0) {
    throw EposException(boost::lexical_cast< std::string >(num_device_errors) +
                        " faults uncleared on the device");
  }
}

void Epos::initDiagnostic() {
  std::string serial_number;
  GET_PARAM_V(config_nh_, serial_number);
  diagnostic_updater_.setHardwareID("EPOS 0x" + serial_number);
  diagnostic_updater_.add(motor_name_ + ": Motor",
                          boost::bind(&Epos::updateMotorDiagnostic, this, _1));
  diagnostic_updater_.add(motor_name_ + ": Motor Output",
                          boost::bind(&Epos::updateMotorOutputDiagnostic, this, _1));
}

void Epos::initMiscParameters() {
  // constant whose unit is mNm/A
  config_nh_.param("torque_constant", torque_constant_, 0.);
  if (torque_constant_ == 0.) {
    for (std::map< std::string, OperationMode >::const_iterator mode = operation_mode_map_.begin();
         mode != operation_mode_map_.end(); ++mode) {
      if (mode->second == CURRENT_MODE) {
        throw EposException("Invalid torque_constant parameter which is required to use the "
                            "operation mode \'current\'");
      }
    }
  }

  // unit of outgoing states and incomming commands
  config_nh_.param("rw_ros_units", rw_ros_units_, false);

  // halt velocity if velocity command is zero
  config_nh_.param("halt_velocity", halt_velocity_, false);

  // constants in battery state
  power_supply_state_.power_supply_technology = config_nh_.param< int >(
      "power_supply/technology", sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
  power_supply_state_.location = config_nh_.param< std::string >("power_supply/location", "");
  power_supply_state_.serial_number =
      config_nh_.param< std::string >("power_supply/serial_number", "");
}

//
// doSwitch()
//

void Epos::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                    const std::list< hardware_interface::ControllerInfo > &stop_list) {
  // switch epos's operation mode according to starting controllers
  BOOST_FOREACH (const hardware_interface::ControllerInfo &starting_controller, start_list) {
    const std::map< std::string, OperationMode >::const_iterator mode_to_switch(
        operation_mode_map_.find(starting_controller.name));
    if (mode_to_switch == operation_mode_map_.end()) {
      continue;
    }
    try {
      // TODO: do VCS_N0(SetDisableState, epos_handle_); ??
      VCS_NN(SetOperationMode, epos_handle_, mode_to_switch->second);
      // TODO: do VCS_N0(SetEnableState, epos_handle_); ??
      operation_mode_ = mode_to_switch->second;
    } catch (const EposException &error) {
      ROS_ERROR_STREAM(error.what());
    }
  }
}

//
// read() and subfunctions
//

void Epos::read() {
  if (!has_init_) {
    return;
  }
  try {
    readJointState();
    readPowerSupply();
    readDiagnostic();
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(error.what());
  }
}

void Epos::readJointState() {
  int position_raw;
  int velocity_raw;
  short current_raw;
  VCS_NN(GetPositionIs, epos_handle_, &position_raw);
  VCS_NN(GetVelocityIs, epos_handle_, &velocity_raw);
  VCS_NN(GetCurrentIs, epos_handle_, &current_raw);
  if (rw_ros_units_) {
    // quad-counts of the encoder -> rad
    position_ = position_raw * M_PI / (2. * encoder_resolution_);
    // rpm -> rad/s
    velocity_ = velocity_raw * M_PI / 30.;
    // mA -> A
    current_ = current_raw / 1000.;
    // mNm -> Nm
    effort_ = currentToTorque(current_) / 1000.;
  } else {
    position_ = position_raw;
    velocity_ = velocity_raw;
    current_ = current_raw / 1000.0; // mA -> A
    effort_ = currentToTorque(current_);
  }
}

void Epos::readPowerSupply() {
  boost::uint16_t voltage10x(0);
  // TODO: use corrent index and subindex according to device_name
  VCS_OBJ(GetObject, epos_handle_, 0x2200, 0x01, &voltage10x, 2);
  // measured variables
  power_supply_state_.voltage = voltage10x / 10.;
  power_supply_state_.present = (voltage10x > 0);
  // unmeasured variables
  power_supply_state_.current = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_.charge = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_.capacity = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_.design_capacity = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_.percentage = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  power_supply_state_.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
}

void Epos::readDiagnostic() {
  // read statusword
  // TODO: use corrent index and subindex according to device_name
  VCS_OBJ(GetObject, epos_handle_, 0x6041, 0x00, &statusword_, 2);

  // read fault info
  unsigned char num_device_errors;
  VCS_NN(GetNbOfDeviceError, epos_handle_, &num_device_errors);
  device_errors_.resize(num_device_errors, 0);
  for (unsigned char i = 1; i <= num_device_errors; ++i) {
    VCS_NN(GetDeviceErrorCode, epos_handle_, i, &device_errors_[i]);
  }
}

//
// write() and subfunctions
//

void Epos::write() {
  if (!has_init_) {
    return;
  }
  try {
    switch (operation_mode_) {
    case PROFILE_POSITION_MODE:
      writePositionCommand();
      break;
    case PROFILE_VELOCITY_MODE:
      writeVelocityCommand();
      break;
    case CURRENT_MODE:
      writeCurrentCommand();
      break;
    default:
      throw EposException("Invalied operation mode (" +
                          boost::lexical_cast< std::string >(operation_mode_) + ")");
      break;
    }
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(error.what());
  }
}

void Epos::writePositionCommand() {
  if (isnan(position_cmd_)) {
    return;
  }
  int cmd;
  if (rw_ros_units_) {
    // rad -> quad-counts of the encoder
    cmd = static_cast< int >(position_cmd_ * 2. * encoder_resolution_ / M_PI);
  } else {
    cmd = static_cast< int >(position_cmd_);
  }
  VCS_NN(MoveToPosition, epos_handle_, cmd, true /* target position is absolute */,
         true /* overwrite old target position */);
}

void Epos::writeVelocityCommand() {
  if (isnan(velocity_cmd_)) {
    return;
  }
  int cmd;
  if (rw_ros_units_) {
    // rad/s -> rpm
    cmd = static_cast< int >(velocity_cmd_ * 30. / M_PI);
  } else {
    cmd = static_cast< int >(velocity_cmd_);
  }
  if (max_profile_velocity_ >= 0) {
    if (cmd < -max_profile_velocity_)
      cmd = -max_profile_velocity_;
    if (cmd > max_profile_velocity_)
      cmd = max_profile_velocity_;
  }

  if (cmd == 0 && halt_velocity_) {
    VCS_N0(HaltVelocityMovement, epos_handle_);
  } else {
    VCS_NN(MoveWithVelocity, epos_handle_, cmd);
  }
}

void Epos::writeCurrentCommand() {
  if (isnan(torque_cmd_)) {
    return;
  }
  int cmd;
  if (rw_ros_units_) {
    // Nm -> mNm & A -> mA
    cmd = static_cast< int >(torqueToCurrent(torque_cmd_ * 1000.) * 1000.);
  } else {
    // A -> mA
    cmd = static_cast< int >(torqueToCurrent(torque_cmd_) * 1000.);
  }
  VCS_NN(SetCurrentMust, epos_handle_, cmd);
}

//
// updateDiagnostics() and callbacks
//

void Epos::updateDiagnostics() { diagnostic_updater_.update(); }

#define STATUSWORD(b, v) ((v >> b) & 1)
#define READY_TO_SWITCH_ON (0)
#define SWITCHED_ON (1)
#define ENABLE (2)
#define FAULT (3)
#define VOLTAGE_ENABLED (4)
#define QUICKSTOP (5)
#define WARNING (7)
#define TARGET_REACHED (10)
#define CURRENT_LIMIT_ACTIVE (11)

void Epos::updateMotorDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("Motor Name", motor_name_);

  if (!has_init_) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "EPOS not initialized");
    return;
  }

  bool enabled = STATUSWORD(READY_TO_SWITCH_ON, statusword_) &&
                 STATUSWORD(SWITCHED_ON, statusword_) && STATUSWORD(ENABLE, statusword_);
  if (enabled) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enabled");
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disabled");
  }

  // Quickstop is enabled when bit is unset (only read quickstop when enabled)
  if (!STATUSWORD(QUICKSTOP, statusword_) && enabled) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Quickstop");
  }

  if (STATUSWORD(WARNING, statusword_)) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Warning");
  }

  if (STATUSWORD(FAULT, statusword_)) {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Fault");
  }

  stat.add< bool >("Enabled", STATUSWORD(ENABLE, statusword_));
  stat.add< bool >("Fault", STATUSWORD(FAULT, statusword_));
  stat.add< bool >("Voltage Enabled", STATUSWORD(VOLTAGE_ENABLED, statusword_));
  stat.add< bool >("Quickstop", STATUSWORD(QUICKSTOP, statusword_));
  stat.add< bool >("Warning", STATUSWORD(WARNING, statusword_));

  BOOST_FOREACH (const unsigned int &device_error, device_errors_) {
    std::stringstream error_msg;
    error_msg << "EPOS Device Error: 0x" << std::hex << device_error;
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
  }
}

void Epos::updateMotorOutputDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat) {
  std::string operation_mode_str;
  if (operation_mode_ == PROFILE_POSITION_MODE) {
    operation_mode_str = "Profile Position Mode";
    stat.add("Commanded Position",
             boost::lexical_cast< std::string >(position_cmd_) + " rotations");
  } else if (operation_mode_ == PROFILE_VELOCITY_MODE) {
    operation_mode_str = "Profile Velocity Mode";
    stat.add("Commanded Velocity", boost::lexical_cast< std::string >(velocity_cmd_) +
                                       (rw_ros_units_ ? " rad/s" : " rpm"));
  } else if (operation_mode_ == CURRENT_MODE) {
    operation_mode_str = "Current Mode";
    stat.add("Commanded Torque",
             boost::lexical_cast< std::string >(torque_cmd_) + (rw_ros_units_ ? " Nm" : " mNm"));
    stat.add("Commanded Current", boost::lexical_cast< std::string >(torqueToCurrent(
                                      rw_ros_units_ ? (torque_cmd_ * 1000.) : torque_cmd_)) +
                                      " A");
  } else {
    operation_mode_str = "Unknown Mode";
  }
  stat.add("Operation Mode", operation_mode_str);
  stat.add("Nominal Current", boost::lexical_cast< std::string >(nominal_current_) + " A");
  stat.add("Max Current", boost::lexical_cast< std::string >(max_current_) + " A");

  unsigned int error_code;
  if (has_init_) {
    stat.add("Position", boost::lexical_cast< std::string >(position_) +
                             (rw_ros_units_ ? "rad" : " quad-encoder-counts"));
    stat.add("Velocity",
             boost::lexical_cast< std::string >(velocity_) + (rw_ros_units_ ? "rad/s" : " rpm"));
    stat.add("Torque",
             boost::lexical_cast< std::string >(effort_) + (rw_ros_units_ ? " Nm" : " mNm"));
    stat.add("Current", boost::lexical_cast< std::string >(current_) + " A");

    stat.add< bool >("Target Reached", STATUSWORD(TARGET_REACHED, statusword_));
    stat.add< bool >("Current Limit Active", STATUSWORD(CURRENT_LIMIT_ACTIVE, statusword_));

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "EPOS operating in " + operation_mode_str);
    if (STATUSWORD(CURRENT_LIMIT_ACTIVE, statusword_))
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Current Limit Active");
    if (nominal_current_ > 0 && std::abs(current_) > nominal_current_) {
      stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
                         "Nominal Current Exceeded (Current: %f A)", current_);
    }

  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "EPOS not initialized");
  }
}
} // namespace epos_hardware
