#include <ios>
#include <iostream>
#include <sstream>
#include <string>

#include <epos_hardware/utils.h>
#include <epos_library/Definitions.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

namespace eh = epos_hardware;
namespace bpo = boost::program_options;

int main(int argc, char *argv[]) {
  std::string device_name, protocol_stack_name, interface_name, serial_number_str;
  unsigned short max_node_id;
  try {
    // define available options
    bpo::options_description options;
    bool show_help;
    options.add(
        boost::make_shared< bpo::option_description >("help", bpo::bool_switch(&show_help)));
    options.add(boost::make_shared< bpo::option_description >(
        "device", bpo::value(&device_name)->default_value("EPOS4")));
    options.add(boost::make_shared< bpo::option_description >(
        "protocol-stack", bpo::value(&protocol_stack_name)->default_value("MAXON SERIAL V2")));
    options.add(boost::make_shared< bpo::option_description >(
        "interface", bpo::value(&interface_name)->default_value("USB")));
    options.add(boost::make_shared< bpo::option_description >("serial-number",
                                                              bpo::value(&serial_number_str)));
    options.add(boost::make_shared< bpo::option_description >(
        "max-node-id", bpo::value(&max_node_id)->default_value(8)));
    // parse the command line
    bpo::variables_map args;
    bpo::store(bpo::parse_command_line(argc, argv, options), args);
    bpo::notify(args);
    // show help if requested
    if (show_help) {
      std::cout << "Available options:\n" << options << std::endl;
      return 0;
    }
  } catch (const bpo::error &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }

  if (serial_number_str.empty()) {
    std::cerr << "Error: no serial number specified from command line" << std::endl;
    return 1;
  }
  boost::uint64_t serial_number;
  {
    std::istringstream iss(serial_number_str);
    iss >> std::hex >> serial_number;
    if (!iss) {
      std::cerr << "Error: Invalid serial number (" << serial_number_str << ")" << std::endl;
      return 1;
    }
  }

  std::cout << "Searching for " << device_name << " 0x" << std::hex << serial_number << " on "
            << interface_name << " (" << protocol_stack_name << ")" << std::endl;

  try {
    eh::NodeHandle epos_handle(eh::createNodeHandle(device_name, protocol_stack_name,
                                                    interface_name, serial_number, max_node_id));

    int position;
    VCS_NN(GetPositionIs, epos_handle, &position);
    std::cout << "Position: " << std::dec << position << std::endl;

    int velocity;
    VCS_NN(GetVelocityIs, epos_handle, &velocity);
    std::cout << "Velocity: " << std::dec << velocity << std::endl;

    short current;
    VCS_NN(GetCurrentIs, epos_handle, &current);
    std::cout << "Current: " << std::dec << current << std::endl;
  } catch (const eh::EposException &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }

  return 0;
}
