#include <ios>
#include <iostream>

#include <epos_hardware/utils.h>

#include <boost/foreach.hpp>

namespace eh = epos_hardware;

int main(int argc, char *argv[]) {
  bool skip_rs232 = true;
  if (argc == 2) {
    if (!strcmp(argv[1], "--rs232"))
      skip_rs232 = false;
    else {
      std::cerr << "unknown option: " << argv[1] << std::endl;
      return 1;
    }
  } else if (argc > 1) {
    std::cerr << "unknown options" << std::endl;
    return 1;
  }

  std::cout << "Listing Devices:" << std::endl;

  const std::vector< std::string > device_names(eh::getDeviceNameList());
  BOOST_FOREACH (const std::string &device_name, device_names) {
    std::cout << device_name << std::endl;

    const std::vector< std::string > protocol_stack_names(
        eh::getProtocolStackNameList(device_name));
    BOOST_FOREACH (const std::string &protocol_stack_name, protocol_stack_names) {
      std::cout << "\t" << protocol_stack_name << std::endl;

      const std::vector< std::string > interface_names(
          eh::getInterfaceNameList(device_name, protocol_stack_name));
      BOOST_FOREACH (const std::string &interface_name, interface_names) {
        std::cout << "\t\t" << interface_name << std::endl;
        if (skip_rs232 && interface_name == "RS232") {
          std::cout << "\t\t\tSkipping RS232" << std::endl;
          continue;
        }

        const std::vector< std::string > port_names(
            eh::getPortNameList(device_name, protocol_stack_name, interface_name));
        BOOST_FOREACH (const std::string &port_name, port_names) {
          std::cout << "\t\t\t" << port_name << std::endl;

          const std::vector< unsigned int > baudrates(
              eh::getBaudrateList(device_name, protocol_stack_name, interface_name, port_name));
          std::cout << "\t\t\t\tBaudrates:" << std::endl;
          BOOST_FOREACH (unsigned int baudrate, baudrates) {
            std::cout << "\t\t\t\t\t" << std::dec << baudrate << std::endl;
          }

          const std::vector< eh::NodeInfo > nodes(eh::enumerateNodes(
              eh::DeviceInfo(device_name, protocol_stack_name, interface_name, port_name)));
          std::cout << "\t\t\t\tNodes:" << std::endl;
          BOOST_FOREACH (const eh::NodeInfo &node, nodes) {
            std::cout << "\t\t\t\t\tNode Id: " << std::dec << node.node_id << std::endl;
            std::cout << "\t\t\t\t\t\tSerial Number: 0x" << std::hex << node.serial_number
                      << std::endl;
            std::cout << "\t\t\t\t\t\tHardware Version: 0x" << std::hex << node.hardware_version
                      << std::endl;
            std::cout << "\t\t\t\t\t\tSoftware Version: 0x" << std::hex << node.software_version
                      << std::endl;
            std::cout << "\t\t\t\t\t\tApplication Number: 0x" << std::hex << node.application_number
                      << std::endl;
            std::cout << "\t\t\t\t\t\tApplication Version: 0x" << std::hex
                      << node.application_version << std::endl;
          }
        }
      }
    }
  }

  return 0;
}
