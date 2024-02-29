#include <BspInclude.hpp>
#include <iostream>

int main(int argc, char** argv) {

  // Get and print the BSP version
  std::cout << "BSP version: " << Bsp::GetVersion() << std::endl;

  // Get and print the number of various types of BSP objects.
  std::cout << "Number of internal triggers: "
    << Bsp::Timing::GetNumberOfInternalTriggers() << std::endl;
  std::cout << "Number of external triggers: "
    << Bsp::Timing::GetNumberOfExternalTriggers() << std::endl;

  std::cout << "Number of temperature sensors per camera: "
    << Bsp::Temperature::GetNumberOfTemperatureSensorsPerCamera() << std::endl;
  std::cout << "Number of system temperature sensors: "
    << Bsp::Temperature::GetNumberOfSystemTemperatureSensors() << std::endl;

  std::vector<Bsp::Pose::pose_report_s> poseDeviceConfigurations = Bsp::Pose::GetPoseDeviceConfigurations();
  std::cout << "Number of pose device configurations: "
    << poseDeviceConfigurations.size() << std::endl;
  for (size_t i = 0; i < poseDeviceConfigurations.size(); i++) {
    std::cout << " Pose device Orientation: "
      << poseDeviceConfigurations[i].ReturnsOrientation
      << " Position: " << poseDeviceConfigurations[i].ReturnsPosition
      << std::endl;
  }

  return 0;
}
