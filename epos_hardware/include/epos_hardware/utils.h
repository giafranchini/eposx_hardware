#ifndef EPOS_HARDWARE_UTILS_H_
#define EPOS_HARDWARE_UTILS_H_

#include <stdexcept>
#include <string>
#include <vector>

#include "epos_library/Definitions.h"

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

namespace epos_hardware {

//
// exception which this c++ wrapper may throw
//

class EposException : public std::runtime_error {
public:
  EposException(const std::string &what_arg);
  EposException(const std::string &what_arg, const unsigned int error_code);
  virtual ~EposException() throw();

  bool hasErrorCode() const;
  unsigned int getErrorCode() const;

  static std::string toErrorInfo(const unsigned int error_code);

public:
  static unsigned int bytes_transferred;
  static unsigned int error_code;

private:
  bool has_error_code_;
  unsigned int error_code_;
};

//
// information of device (node chain)
//

class DeviceInfo {
public:
  DeviceInfo();
  DeviceInfo(const std::string &device_name, const std::string &protocol_stack_name,
             const std::string &interface_name, const std::string &port_name);
  virtual ~DeviceInfo();

public:
  std::string device_name;
  std::string protocol_stack_name;
  std::string interface_name;
  std::string port_name;
};

//
// handle of device (node chain) which finalizes itself on destruction
//

class DeviceHandle {
public:
  DeviceHandle();
  DeviceHandle(const DeviceInfo &device_info);
  virtual ~DeviceHandle();

private:
  static boost::shared_ptr< void > makePtr(const DeviceInfo &device_info);
  static void *openDevice(const DeviceInfo &device_info);
  static void closeDevice(void *ptr);

public:
  boost::shared_ptr< void > ptr;
};

//
// information of node
//

class NodeInfo : public DeviceInfo {
public:
  NodeInfo();
  NodeInfo(const DeviceInfo &device_info, const unsigned short node_id);
  virtual ~NodeInfo();

public:
  unsigned short node_id;
  boost::uint64_t serial_number;
  unsigned short hardware_version;
  unsigned short software_version;
  unsigned short application_number;
  unsigned short application_version;
};

//
// handle of node
//

class NodeHandle : public DeviceHandle {
public:
  NodeHandle();
  NodeHandle(const NodeInfo &node_info);
  NodeHandle(const DeviceHandle &device_handle, unsigned short node_id);
  virtual ~NodeHandle();

public:
  unsigned short node_id;
};

//
// enumeration functions
//

std::vector< std::string > getDeviceNameList();

std::vector< std::string > getProtocolStackNameList(const std::string &device_name);

std::vector< std::string > getInterfaceNameList(const std::string &device_name,
                                                const std::string &protocol_stack_name);

std::vector< std::string > getPortNameList(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name);

std::vector< unsigned int > getBaudrateList(const std::string &device_name,
                                            const std::string &protocol_stack_name,
                                            const std::string &interface_name,
                                            const std::string &port_name);

std::vector< DeviceInfo > enumerateDevices(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name);

std::vector< NodeInfo > enumerateNodes(const std::string &device_name,
                                       const std::string &protocol_stack_name,
                                       const std::string &interface_name);

std::vector< NodeInfo > enumerateNodes(const DeviceInfo &device_info);

NodeHandle createNodeHandle(const std::string &device_name, const std::string &protocol_stack_name,
                            const std::string &interface_name, const boost::uint64_t serial_number);

} // namespace epos_hardware

//
// useful macros
//

// boolean value in VCS_xxx functions
#define VCS_FALSE 0

// call a VCS_xxx function in a if statement
#define IF_VCS(func, ...)                                                                          \
  if (VCS_##func(__VA_ARGS__, &epos_hardware::EposException::error_code) != VCS_FALSE)

// call a VCS_xxx function or die
#define VCS(func, ...)                                                                             \
  do {                                                                                             \
    unsigned int error_code;                                                                       \
    if (VCS_##func(__VA_ARGS__, &error_code) == VCS_FALSE) {                                       \
      throw EposException(#func, error_code);                                                      \
    }                                                                                              \
  } while (false)

// call a VCS_xxx function with epos_hardware::NodeHandlePtr in a if statement (no more arguments)
#define IF_VCS_N0(func, epos_node_handle)                                                          \
  if (VCS_##func(epos_node_handle.ptr.get(), epos_node_handle.node_id,                             \
                 &epos_hardware::EposException::error_code) != VCS_FALSE)

// call a VCS_xxx function with epos_hardware::NodeHandlePtr or die (no more arguments)
#define VCS_N0(func, epos_node_handle)                                                             \
  do {                                                                                             \
    unsigned int error_code;                                                                       \
    if (VCS_##func(epos_node_handle.ptr.get(), epos_node_handle.node_id, &error_code) ==           \
        VCS_FALSE) {                                                                               \
      throw EposException(#func, error_code);                                                      \
    }                                                                                              \
  } while (false)

// call a VCS_xxx function with epos_hardware::NodeHandlePtr in a if statement
#define IF_VCS_NN(func, epos_node_handle, ...)                                                     \
  if (VCS_##func(epos_node_handle.ptr(), epos_node_handle.node_id, __VA_ARGS__,                    \
                 &epos_hardware::EposException::error_code) != VCS_FALSE)

// call a VCS_xxx function with epos_hardware::NodeHandlePtr or die
#define VCS_NN(func, epos_node_handle, ...)                                                        \
  do {                                                                                             \
    unsigned int error_code;                                                                       \
    if (VCS_##func(epos_node_handle.ptr.get(), epos_node_handle.node_id, __VA_ARGS__,              \
                   &error_code) == VCS_FALSE) {                                                    \
      throw EposException(#func, error_code);                                                      \
    }                                                                                              \
  } while (false)

// call a VCS_XxxObject function with epos_hardware::NodeHandlePtr in a if statement
#define IF_VCS_OBJ(func, epos_node_handle, index, subindex, data, length)                          \
  if (VCS_##func(epos_node_handle.ptr.get(), epos_node_handle.node_id, index, subindex, data,      \
                 length, &epos_hardware::EposException::bytes_transferred,                         \
                 &epos_hardware::EposException::error_code) != VCS_FALSE)

// call a VCS_XxxObject function with epos_hardware::NodeHandlePtr or die
#define VCS_OBJ(func, epos_node_handle, index, subindex, data, length)                             \
  do {                                                                                             \
    unsigned int bytes_transferred;                                                                \
    unsigned int error_code;                                                                       \
    if (VCS_##func(epos_node_handle.ptr.get(), epos_node_handle.node_id, index, subindex, data,    \
                   length, &bytes_transferred, &error_code) == VCS_FALSE) {                        \
      throw EposException(#func, error_code);                                                      \
    }                                                                                              \
  } while (false)

#endif
