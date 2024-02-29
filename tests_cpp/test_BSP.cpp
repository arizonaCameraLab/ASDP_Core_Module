#include <BspInclude.hpp>
#include <iostream>
#include <chrono>
#include <vector>
#include <thread>

int main(int argc, char** argv) {

  // Get and print the BSP version
  std::cout << "BSP version: " << Bsp::GetVersion() << std::endl;

  // Get and print the number of various types of BSP objects.
  std::cout << "Number of internal triggers: "
    << Bsp::Timing::GetNumberOfInternalTriggers() << std::endl;
  std::cout << "Number of external triggers: "
    << Bsp::Timing::GetNumberOfExternalTriggers() << std::endl;

  uint16_t NumTempsPerCamera = Bsp::Temperature::GetNumberOfTemperatureSensorsPerCamera();
  std::cout << "Number of temperature sensors per camera: " << NumTempsPerCamera << std::endl;
  uint16_t NumSystemTemps = Bsp::Temperature::GetNumberOfSystemTemperatureSensors();
  std::cout << "Number of system temperature sensors: " << NumSystemTemps << std::endl;

  std::vector<Bsp::Pose::pose_report_s> PoseDeviceConfigurations = Bsp::Pose::GetPoseDeviceConfigurations();
  std::cout << "Number of pose device configurations: "
    << PoseDeviceConfigurations.size() << std::endl;
  for (size_t i = 0; i < PoseDeviceConfigurations.size(); i++) {
    std::cout << " Pose device Orientation: "
      << PoseDeviceConfigurations[i].ReturnsOrientation
      << " Position: " << PoseDeviceConfigurations[i].ReturnsPosition
      << std::endl;
  }

  uint16_t NumCameras = Bsp::Camera::GetNumberOfCameras();
  std::cout << "Number of cameras: " << NumCameras << std::endl;

  // Read from all of the temperature sensors and make sure that they are reporting.
  std::vector<uint32_t> SystemTemperatureSensors(NumSystemTemps, 0);
  std::vector< std::vector<uint32_t> > CameraTemperatureSensors(NumCameras);
  for (size_t i = 0; i < NumCameras; i++) {
    CameraTemperatureSensors[i].resize(NumTempsPerCamera, 0);
  }
  auto start = std::chrono::steady_clock::now();
  auto now = start;
  while (now - start < std::chrono::seconds(1)) {
    auto temps = Bsp::Temperature::GetNewTemperatures();
    for (auto t : temps) {
      if (t.CameraIndex == 0) {
        SystemTemperatureSensors[t.SensorIndex - 1]++;
      } else {
        CameraTemperatureSensors[t.CameraIndex - 1][t.SensorIndex - 1]++;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    now = std::chrono::steady_clock::now();
  }
  for (auto &s : SystemTemperatureSensors) {
    if (s == 0) {
      std::cout << "System temperature sensor not reporting." << std::endl;
      return 1;
    }
  }
  for (auto& v : CameraTemperatureSensors) {
    for (auto& s : v) {
      if (s == 0) {
        std::cout << "Camera temperature sensor not reporting." << std::endl;
        return 1;
      }
    }
  }
  std::cout << "Got reports from all temperature sensors" << std::endl;

  // Read from all of the pose reporters and make sure they are reporting.
  size_t NumPosers = PoseDeviceConfigurations.size();
  std::vector<uint32_t> poseReporters(NumPosers, 0);
  start = std::chrono::steady_clock::now();
  now = start;
  while (now - start < std::chrono::seconds(1)) {
    auto poses = Bsp::Pose::GetNewPoses();
    for (auto &p : poses) {
      poseReporters[p.DeviceID - 1]++;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    now = std::chrono::steady_clock::now();
  }
  for (auto& p : poseReporters) {
    if (p == 0) {
      std::cout << "Pose reporter not reporting." << std::endl;
      return 2;
    }
  }
  std::cout << "Got reports from all pose reporters" << std::endl;

  return 0;
}
