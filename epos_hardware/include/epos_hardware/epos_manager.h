#ifndef EPOS_HARDWARE_EPOS_MANAGER_H_
#define EPOS_HARDWARE_EPOS_MANAGER_H_

#include <list>
#include <string>
#include <vector>

#include <epos_hardware/epos.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

#include <boost/shared_ptr.hpp>

namespace epos_hardware {

class EposManager {
public:
  EposManager();
  virtual ~EposManager();

  void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
            const std::vector< std::string > &motor_names);
  void read();
  void write();
  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list);
  void updateDiagnostics();
  std::vector< std::string > motorNames() const;

private:
  std::vector< boost::shared_ptr< Epos > > motors_;
};

} // namespace epos_hardware

#endif
