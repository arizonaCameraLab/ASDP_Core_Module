/*
 * Copyright (C) 2024-2025: Arizona Board of Regents on Behalf of the University of Arizona
 */

// This is the Core Module implementation.

#include <cstdlib>
#include <iostream>
#include <chrono>
#include <ASDP_Core_API.h>
#include <Core_Module.h>

using namespace asdp;

static std::string VERSION = "1.2.0";

void Usage(const char* programName, int code)
{
  std::cerr << "Usage: " << programName << " [--version] [--verbosity V] NICName" << std::endl;
  std::cerr << "  --version: Print out the version number and exit" << std::endl;
  std::cerr << "  --verbosity V: Set the verbosity level to V (default 0)" << std::endl;
  std::cerr << "  NICName: The IP address or DNS name of the NIC to listen on" << std::endl;
  std::exit(code);
}

int main(int argc, char** argv)
{
  std::string NICName;
  size_t realParams = 0;
  int verbosity = 0;

  // Parse the command line arguments, with the first non-flag argument being the
  // name of the IP address to broadcast on.  There is a --serial flag to specify
  // the serial number of the server, which defaults to 1.
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--version") {
      std::cout << "Core Module version: " << VERSION + "-" + BUILD_TYPE << " using Core API "
        << asdp::Core::GetVersion() << std::endl;
      return 0;
    } else if (std::string(argv[i]) == "--verbosity") {
      if (i + 1 < argc) {
        verbosity = std::stoi(argv[i + 1]);
        ++i;
      } else {
        std::cerr << "--verbosity flag requires an argument" << std::endl;
        return 1;
      }
    } else if (argv[i][0] == '-' ) {
      std::cerr << "Unknown flag: " << argv[i] << std::endl;
      return 1;
    } else switch (realParams++) {
      case 0:
        NICName = argv[i];
        break;
      default:
        Usage(argv[0], 2);
    }
  }
  if (realParams != 1) {
    Usage(argv[0], 2);
  }

  // Open a Core Module, specifying the NIC to listen and broadcast on.
  // Do this inside a block so that the module is destroyed when it goes out of scope.
  {
    if (verbosity > 0) {
      std::cout << "Initializing Core module (please wait)..." << std::endl;
    }
    Core_Module_Server server(NICName, 10102, 10101, 9000-28, verbosity);
    if (server.GetConstructorStatus() != OKAY) {
      std::cerr << "Failed to open module: " << ErrorMessage(server.GetConstructorStatus()) << std::endl;
      return 3;
    }
    if (verbosity > 0) {
      std::cout << "Opened Core module listening on " << NICName << std::endl;
    }
    std::string ret = server.run();
    if (ret.size() > 0) {
      std::cerr << "Server failed: " << ret << std::endl;
      return 4;
    }
    // Watch the module.  Print an error message if it fails.
    do {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } while (server.GetCurrentStatus() == OKAY);
    std::cerr << "Module failed: " << ErrorMessage(server.GetCurrentStatus()) << std::endl;
    return server.GetCurrentStatus();
  }

  return 0;
}
