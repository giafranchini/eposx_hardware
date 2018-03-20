#include <ios>
#include <iostream>
#include <sstream>

#include <epos_hardware/utils.h>
#include <epos_library/Definitions.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>

namespace eh = epos_hardware;

int main(int argc, char *argv[]) {
  boost::uint64_t serial_number;
  if (argc == 2) {
    std::stringstream ss;
    ss << argv[1];
    ss >> std::hex >> serial_number;
    if (!ss) {
      std::cerr << "Expected a serial number" << std::endl;
      return 1;
    }
  } else {
    std::cerr << "Expected exactly one argument that is a serial number" << std::endl;
    return 1;
  }

  std::cout << "Searching for USB EPOS4: 0x" << std::hex << serial_number << std::endl;

  try {
    eh::NodeHandle handle(eh::createNodeHandle("EPOS4", "MAXON SERIAL V2", "USB", serial_number));

    int position;
    VCS_NN(GetPositionIs, handle, &position);
    std::cout << "Position: " << std::dec << position << std::endl;

    int velocity;
    VCS_NN(GetVelocityIs, handle, &velocity);
    std::cout << "Velocity: " << std::dec << velocity << std::endl;

    short current;
    VCS_NN(GetCurrentIs, handle, &current);
    std::cout << "Current: " << std::dec << current << std::endl;
  } catch (const eh::EposException &error) {
    std::cerr << error.what() << std::endl;
    return 1;
  }

  return 0;
}
