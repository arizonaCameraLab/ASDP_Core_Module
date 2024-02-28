#include <BspInclude.hpp>
#include <iostream>

int main(int argc, char** argv) {

  // Get and print the BSP version
  std::cout << "BSP version: " << Bsp::GetVersion() << std::endl;

  return 0;
}
