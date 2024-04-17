#include <BspInclude.hpp>
#include <ASDP_Core_API.h>
#include <iostream>
#include <chrono>
#include <vector>
#include <thread>

int main(int argc, char** argv)
{
  std::cout << "Note: This can only test the UDP sending for systems that can send to localhost" << std::endl;

  // Put all of this into a block so that objects will be deleted when they go out of scope.
  {
    // Get and print the BSP version and serial number.
    std::cout << "BSP version: " << Bsp::GetVersion() << std::endl;
    std::cout << "BSP serial number: " << Bsp::GetSerialNumber() << std::endl;

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
    std::vector<std::string> fields = { "type", "width", "height", "minTriggerPeriod", "maxTriggerPeriod" };
    for (size_t c = 1; c <= NumCameras; c++) {
      std::map<std::string, std::string> status = Bsp::Camera::GetStatus(c);
      for (auto f : fields) {
        if (status.find(f) == status.end()) {
          std::cerr << "Camera " << c << " field " << f << " is unspecified" << std::endl;
          return 1;
        }
      }
    }

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
        }
        else {
          CameraTemperatureSensors[t.CameraIndex - 1][t.SensorIndex - 1]++;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      now = std::chrono::steady_clock::now();
    }
    for (auto& s : SystemTemperatureSensors) {
      if (s == 0) {
        std::cerr << "System temperature sensor not reporting." << std::endl;
        return 1;
      }
    }
    for (auto& v : CameraTemperatureSensors) {
      for (auto& s : v) {
        if (s == 0) {
          std::cerr << "Camera temperature sensor not reporting." << std::endl;
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
      for (auto& p : poses) {
        poseReporters[p.DeviceID - 1]++;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      now = std::chrono::steady_clock::now();
    }
    for (auto& p : poseReporters) {
      if (p == 0) {
        std::cerr << "Pose reporter not reporting." << std::endl;
        return 2;
      }
    }
    std::cout << "Got reports from all pose reporters" << std::endl;

    // Ensure that we can't configure invalid triggers.
    if (Bsp::Timing::ConfigureTrigger(0, 3, 1, 1.0 / 60, 0.5)) {
      std::cerr << "Configured trigger 0, which should not be allowed" << std::endl;
      return 3;
    }
    uint32_t badTrigger = Bsp::Timing::GetNumberOfInternalTriggers() + 1;
    if (Bsp::Timing::ConfigureTrigger(badTrigger, 3, 1, 1.0 / 60, 0.5)) {
      std::cerr << "Configured trigger " << badTrigger << ", which should not be allowed" << std::endl;
      return 3;
    }

    // Ensure that we can't configure invalid cameras.
    if (Bsp::Camera::SetupCamera(0, 1)) {
      std::cerr << "Configured camera 0, which should not be allowed" << std::endl;
      return 4;
    }
    uint32_t badCamera = Bsp::Camera::GetNumberOfCameras() + 1;
    if (Bsp::Camera::SetupCamera(badCamera, 1)) {
      std::cerr << "Configured camera " << badCamera << ", which should not be allowed" << std::endl;
      return 4;
    }

    // Try all triggers.
    for (size_t trig = 1; trig <= Bsp::Timing::GetNumberOfInternalTriggers(); trig++) {

      // Configure trigger to fire at 60fps using a periodic software trigger from external trigger 1.
      if (!Bsp::Timing::ConfigureTrigger(trig, 3, 1, 1.0 / 60, 0.5)) {
        std::cerr << "Could not configure trigger " << trig << std::endl;
        return 5;
      }
      std::cout << "Trigger " << trig << " configured" << std::endl;

      // Check all cameras for streaming
      for (size_t cam = 1; cam <= NumCameras; cam++) {
        // Set up the camera to use trigger.
        if (!Bsp::Camera::SetupCamera(cam, trig)) {
          std::cerr << "Could not set up camera " << cam << std::endl;
          return 4;
        }
        std::cout << " Camera " << cam << " set up" << std::endl;

        // Set up a UDP receiver on localhost so that we can get streamed video from camera.
        asdp::ReceiverUDP receiver;
        if (receiver.GetConstructorStatus() != asdp::OKAY) {
          std::cerr << "Error constructing ReceiverUDP: " << ErrorMessage(receiver.GetConstructorStatus()) << std::endl;
          return 5;
        }
        uint16_t port;
        asdp::Status status = receiver.GetPort(port);
        if (status != asdp::OKAY) {
          std::cerr << "Error getting port from ReceiverUDP: " << ErrorMessage(status) << std::endl;
          return 6;
        }

        // Configure the UDP output stream to talk with our receiver, sending every full frame starting immediately.
        Bsp::UdpInserter::endpoint_s endpoint;
        endpoint.IP = (127 << 24) + 1;
        endpoint.Port = port;
        Bsp::UdpInserter::roi_s roi;
        roi.start_x = 0;
        roi.end_x = 1280 - 1;
        roi.end_y = 1024 - 1;
        roi.start_y = 0;
        if (!Bsp::UdpInserter::EnableStream(cam, endpoint, roi, 0, 0, 0)) {
          std::cerr << "Could not enable streaming for camera " << cam << std::endl;
          std::cerr << "Note: This can only test the UDP sending for systems that can send to localhost" << std::endl;
          return 7;
        }

        // Trigger the camera to start sending frames.
        if (!Bsp::Timing::SoftwareTrigger(trig, 0, 0)) {
          std::cerr << "Could not fire software trigger " << trig << std::endl;
          return 7;
        }

        // Receive frames for a second and then make sure we're getting about 60fps.
        start = std::chrono::steady_clock::now();
        auto end = start;
        size_t numStartFrames = 0, numEndFrames = 0;
        size_t sequenceNumber = 0;
        do {

          // Get packets and count how many are begin-frame messages with proper sequence numbers.
          std::shared_ptr<asdp::StreamPacket> receiveStreamPacket;
          size_t offset = 0;
          status = receiver.ReceiveStreamPacket(1.0, receiveStreamPacket, offset);
          if (status != asdp::OKAY) {
            std::cerr << "Error receiving StreamPacket: " << ErrorMessage(status) << std::endl;
            return 8;
          }
          uint32_t packetSequenceNumber;
          status = receiveStreamPacket->GetSequenceNumber(packetSequenceNumber);
          if (status != asdp::OKAY) {
            std::cerr << "Error getting sequence number from StreamPacket: " << ErrorMessage(status) << std::endl;
            return 8;
          }
          if (packetSequenceNumber != sequenceNumber++) {
            std::cerr << " Bad sequence number: " << packetSequenceNumber << ", expected " << sequenceNumber - 1 << std::endl;
            std::cerr << "  (Presumably dropped network packets, consider re-running)" << std::endl;
          }
          std::shared_ptr<asdp::Message> message;
          status = receiveStreamPacket->GetNextMessage(message);
          if (status != asdp::OKAY) {
            std::cerr << "Error getting first message from packet: " << ErrorMessage(status) << std::endl;
            return 8;
          }
          asdp::MessageID rID;
          status = message->GetType(rID);
          if (status != asdp::OKAY) {
            std::cerr << "Error getting type from packet: " << ErrorMessage(status) << std::endl;
            return 8;
          }
          if (rID == asdp::FRAME_BEGIN) {
            numStartFrames++;
          }
          if (rID == asdp::FRAME_END) {
            numEndFrames++;
          }

          end = std::chrono::steady_clock::now();
        } while ((end - start) < std::chrono::seconds(1));
        std::cout << "  Got " << numStartFrames << " start frames and " << numEndFrames << " end frames" << std::endl;
        if (numStartFrames < 58 || numStartFrames > 62) {
          std::cerr << "Did not get 58-62 start frames" << std::endl;
          std::cerr << "Note: This can only test the UDP sending for systems that can send to localhost" << std::endl;
          return 8;
        }

        // Disable the stream
        if (!Bsp::UdpInserter::DisableStream(cam, endpoint)) {
          std::cerr << "Could not disable streaming for camera " << cam << std::endl;
          return 9;
        }

        // Set up the camera to use trigger 0 (disabled).
        if (!Bsp::Camera::SetupCamera(cam, 0)) {
          std::cerr << "Could not disable camera " << cam << std::endl;
          return 10;
        }
        std::cout << " Camera " << cam << " disabled" << std::endl;
      }

      // Disable the trigger
      if (!Bsp::Timing::ConfigureTrigger(trig, 0, 1, 1.0 / 60, 0.5)) {
        std::cerr << "Could not disable trigger " << trig << std::endl;
        return 11;
      }
      std::cout << "Trigger " << trig << " disabled" << std::endl;
    }

  } // End of block to ensure objects go out of scope.


  return 0;
}
