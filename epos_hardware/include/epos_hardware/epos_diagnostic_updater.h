#ifndef EPOS_HARDWARE_EPOS_DIAGNOSTIC_UPDATER_H
#define EPOS_HARDWARE_EPOS_DIAGNOSTIC_UPDATER_H

#include <string>
#include <vector>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

#include <boost/cstdint.hpp>
#include <boost/scoped_ptr.hpp>

namespace epos_hardware {

class EposDiagnosticHandle {
public:
  EposDiagnosticHandle() : name_(), statusword_(NULL), device_errors_(NULL) {}
  EposDiagnosticHandle(const std::string &name, boost::uint16_t *const statusword,
                       std::vector< unsigned int > *const device_errors)
      : name_(name), statusword_(statusword), device_errors_(device_errors) {}
  virtual ~EposDiagnosticHandle() {}

  std::string getName() const { return name_; }
  boost::uint16_t getStatusword() const { return *statusword_; }
  const boost::uint16_t *getStatuswordPtr() const { return statusword_; }
  std::vector< unsigned int > getDeviceErrors() const { return *device_errors_; }
  const std::vector< unsigned int > *getDeviceErrorsPtr() const { return device_errors_; }

private:
  std::string name_;
  const boost::uint16_t *statusword_;
  const std::vector< unsigned int > *device_errors_;
};

class EposDiagnosticInterface
    : public hardware_interface::HardwareResourceManager< EposDiagnosticHandle > {};

class EposDiagnosticUpdater {
public:
  EposDiagnosticUpdater();
  virtual ~EposDiagnosticUpdater();

  void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);
  void update();

private:
  void updateMotorDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void updateMotorOutputDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
  std::string motor_name_;

  boost::scoped_ptr< diagnostic_updater::Updater > diagnostic_updater_;

  bool rw_ros_units_;
  double torque_constant_, nominal_current_, max_output_current_;

  const double *position_, *velocity_, *effort_;
  const double *position_cmd_, *velocity_cmd_, *effort_cmd_;
  const boost::uint16_t *statusword_;
  const std::vector< unsigned int > *device_errors_;
};

} // namespace epos_hardware

#endif