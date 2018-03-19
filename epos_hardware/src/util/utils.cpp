#include "epos_hardware/utils.h"

#include <ios>
#include <map>
#include <sstream>

#include <boost/foreach.hpp>
#include <boost/weak_ptr.hpp>

namespace epos_hardware {

//
// EposException
//

EposException::EposException(const std::string &what_arg)
    : std::runtime_error(what_arg), has_error_code_(false), error_code_(0) {}

EposException::EposException(const std::string &what_arg, const unsigned int error_code)
    : std::runtime_error(what_arg + " (" + toErrorInfo(error_code) + ")"), has_error_code_(true),
      error_code_(error_code) {}

EposException::~EposException() throw() {}

bool EposException::hasErrorCode() const { return has_error_code_; }

unsigned int EposException::getErrorCode() const { return error_code_; }

std::string EposException::toErrorInfo(const unsigned int error_code) {
  std::ostringstream oss;
  oss << "0x" << std::hex << error_code;
  char error_info[1024];
  if (VCS_GetErrorInfo(error_code, error_info, 1024) != VCS_FALSE) {
    oss << ": " << error_info;
  }
  return oss.str();
}

//
// DeviceHandle
//

DeviceHandle::DeviceHandle(void *ptr) : ptr(ptr) {}

DeviceHandle::~DeviceHandle() {
  unsigned int error_code;
  if (VCS_CloseDevice(ptr, &error_code) == VCS_FALSE) {
    throw EposException("CloseDevice", error_code);
  }
}

//
// NodeHandle
//

NodeHandle::NodeHandle(DeviceHandlePtr device_handle, unsigned short node_id)
    : device_handle(device_handle), node_id(node_id) {}

NodeHandle::~NodeHandle() {}

//
// enumeration functions
//

std::vector< std::string > getDeviceNameList() {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > device_names;
  VCS(GetDeviceNameSelection, true, buffer, 1024, &end_of_selection);
  device_names.push_back(buffer);
  while (end_of_selection != VCS_FALSE) {
    VCS(GetDeviceNameSelection, false, buffer, 1024, &end_of_selection);
    device_names.push_back(buffer);
  }
  return device_names;
}

std::vector< std::string > getProtocolStackNameList(const std::string &device_name) {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > protocol_stack_names;
  VCS(GetProtocolStackNameSelection, const_cast< char * >(device_name.c_str()), true, buffer, 1024,
      &end_of_selection);
  protocol_stack_names.push_back(buffer);
  while (end_of_selection != VCS_FALSE) {
    VCS(GetProtocolStackNameSelection, const_cast< char * >(device_name.c_str()), false, buffer,
        1024, &end_of_selection);
    protocol_stack_names.push_back(buffer);
  }
  return protocol_stack_names;
}

std::vector< std::string > getInterfaceNameList(const std::string &device_name,
                                                const std::string &protocol_stack_name) {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > interface_names;
  VCS(GetInterfaceNameSelection, const_cast< char * >(device_name.c_str()),
      const_cast< char * >(protocol_stack_name.c_str()), true, buffer, 1024, &end_of_selection);
  interface_names.push_back(buffer);
  while (end_of_selection != VCS_FALSE) {
    VCS(GetInterfaceNameSelection, const_cast< char * >(device_name.c_str()),
        const_cast< char * >(protocol_stack_name.c_str()), false, buffer, 1024, &end_of_selection);
    interface_names.push_back(buffer);
  }
  return interface_names;
}

std::vector< std::string > getPortNameList(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name) {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > port_names;
  VCS(GetPortNameSelection, const_cast< char * >(device_name.c_str()),
      const_cast< char * >(protocol_stack_name.c_str()),
      const_cast< char * >(interface_name.c_str()), true, buffer, 1024, &end_of_selection);
  port_names.push_back(buffer);
  while (end_of_selection != VCS_FALSE) {
    VCS(GetPortNameSelection, const_cast< char * >(device_name.c_str()),
        const_cast< char * >(protocol_stack_name.c_str()),
        const_cast< char * >(interface_name.c_str()), false, buffer, 1024, &end_of_selection);
    port_names.push_back(buffer);
  }
  return port_names;
}

std::vector< unsigned int > getBaudrateList(const std::string &device_name,
                                            const std::string &protocol_stack_name,
                                            const std::string &interface_name,
                                            const std::string &port_name) {
  unsigned int baudrate;
  int end_of_selection; // BOOL
  std::vector< unsigned int > baudrates;
  VCS(GetBaudrateSelection, const_cast< char * >(device_name.c_str()),
      const_cast< char * >(protocol_stack_name.c_str()),
      const_cast< char * >(interface_name.c_str()), const_cast< char * >(port_name.c_str()), true,
      &baudrate, &end_of_selection);
  baudrates.push_back(baudrate);
  while (end_of_selection != VCS_FALSE) {
    VCS(GetBaudrateSelection, const_cast< char * >(device_name.c_str()),
        const_cast< char * >(protocol_stack_name.c_str()),
        const_cast< char * >(interface_name.c_str()), const_cast< char * >(port_name.c_str()),
        false, &baudrate, &end_of_selection);
    baudrates.push_back(baudrate);
  }
  return baudrates;
}

std::vector< NodeInfo > enumerateNodes(const std::string &device_name,
                                       const std::string &protocol_stack_name,
                                       const std::string &interface_name,
                                       const std::string &port_name) {
  std::vector< NodeInfo > node_infos;

  // get the device
  DeviceHandlePtr device_handle(
      createDeviceHandle(device_name, protocol_stack_name, interface_name, port_name));
  if (!device_handle) {
    return node_infos;
  }

  // try access all possible nodes on the device
  NodeInfo node_info;
  node_info.device_name = device_name;
  node_info.protocol_stack_name = protocol_stack_name;
  node_info.interface_name = interface_name;
  node_info.port_name = port_name;
  for (unsigned short i = 1; i < 127; ++i) {
    try {
      VCS(GetVersion, device_handle->ptr, i, &node_info.hardware_version,
          &node_info.software_version, &node_info.application_number,
          &node_info.application_version);
      // TODO: change index based on protocol_stack_name("epos2" or "epos4")
      unsigned int bytes_read;
      VCS(GetObject, device_handle->ptr, i, 0x2100, 0x01, &node_info.serial_number, 8, &bytes_read);
    } catch (const EposException &) {
      // node does not exist
      continue;
    }
    node_info.node_id = i;
    node_infos.push_back(node_info);
  }
  return node_infos;
}

std::vector< NodeInfo > enumerateNodes(const std::string &device_name,
                                       const std::string &protocol_stack_name,
                                       const std::string &interface_name) {
  std::vector< NodeInfo > node_infos;
  const std::vector< std::string > port_names(
      getPortNameList(device_name, protocol_stack_name, interface_name));
  BOOST_FOREACH (const std::string &port_name, port_names) {
    const std::vector< NodeInfo > this_node_infos(
        enumerateNodes(device_name, protocol_stack_name, interface_name, port_name));
    node_infos.insert(node_infos.end(), this_node_infos.begin(), this_node_infos.end());
  }
  return node_infos;
}

//
// factory functions
//

// global storage of opened devices
std::map< std::string, boost::weak_ptr< DeviceHandle > > existing_device_handles;

DeviceHandlePtr createDeviceHandle(const std::string &device_name,
                                   const std::string &protocol_stack_name,
                                   const std::string &interface_name,
                                   const std::string &port_name) {
  const std::string key(device_name + '/' + protocol_stack_name + '/' + interface_name + '/' +
                        port_name);
  DeviceHandlePtr device_handle(existing_device_handles[key].lock());
  if (!device_handle) {
    unsigned int error_code;
    void *raw_handle(VCS_OpenDevice(const_cast< char * >(device_name.c_str()),
                                    const_cast< char * >(protocol_stack_name.c_str()),
                                    const_cast< char * >(interface_name.c_str()),
                                    const_cast< char * >(port_name.c_str()), &error_code));
    if (!raw_handle) {
      throw EposException("OpenDevice", error_code);
    }
    device_handle.reset(new DeviceHandle(raw_handle));
    existing_device_handles[key] = device_handle;
  }
  return device_handle;
}

NodeHandlePtr createNodeHandle(const std::string &device_name,
                               const std::string &protocol_stack_name,
                               const std::string &interface_name,
                               const boost::uint64_t serial_number) {
  const std::vector< NodeInfo > node_infos(
      enumerateNodes(device_name, protocol_stack_name, interface_name));
  BOOST_FOREACH (const NodeInfo &node_info, node_infos) {
    if (node_info.serial_number == serial_number) {
      return createNodeHandle(node_info);
    }
  }
  return NodeHandlePtr();
}

NodeHandlePtr createNodeHandle(const NodeInfo &node_info) {
  DeviceHandlePtr device_handle(createDeviceHandle(node_info.device_name,
                                                   node_info.protocol_stack_name,
                                                   node_info.interface_name, node_info.port_name));
  if (!device_handle) {
    return NodeHandlePtr();
  }
  return NodeHandlePtr(new NodeHandle(device_handle, node_info.node_id));
}
} // namespace epos_hardware
