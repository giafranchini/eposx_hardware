eposx_hardware
============
# eposx_hardware is ...
* ROS interface for Maxon EPOS motor drivers
* forked from [RIVeR-Lab/epos_hardware](https://github.com/RIVeR-Lab/epos_hardware), with a lot of enhancements and refactors
* difference from the original package
  * supports all EPOS drivers (EPOS/EPOS2/EPOS4)
  * supports all phisycal interface to EPOS (USB/RS232/CANOpen)
  * supports online switching of EPOS control modes
  * supports reading status of EPOS's power supply
  * supports joint limit interface
  * uses the latest offical command library from Maxon in backend (as of Jun. 2018)
  * refactored codes including an useful C++ wrapper of backend C library

# Node: epos_hardware_node
will be described soon

# Commandline tool: list_nodes
will be described soon

# Commandline tool: get_state
will be described soon

# Examples
* see [eposx_hardware/launch](eposx_hardware/launch)
