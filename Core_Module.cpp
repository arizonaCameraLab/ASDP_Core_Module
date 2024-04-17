/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "Core_Module.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <filesystem>

using namespace asdp;
using namespace Bsp::Temperature;
using namespace Bsp::Timing;
using namespace Bsp::Pose;
using namespace Bsp::Camera;
using namespace Bsp::UdpInserter;

Core_Module_Server::Core_Module_Server(const std::string& NicName, uint16_t sendPort, uint16_t listenPort,
  uint32_t maxPayloadSize, int verbosity)
  : CoreServerBase(Bsp::GetSerialNumber(), NicName, sendPort, listenPort, maxPayloadSize, verbosity)
  , m_numCameras(Bsp::Camera::GetNumberOfCameras())
  , m_poseDeviceConfigurations(Bsp::Pose::GetPoseDeviceConfigurations())
{
  // Fill in our state with the current values from the BSP.
  m_numTemperaturesPerCamera = GetNumberOfTemperatureSensorsPerCamera();
  m_numSystemTemperatures = GetNumberOfSystemTemperatureSensors();
  for (size_t i = 1; i <= GetNumberOfInternalTriggers(); i++) {
    TriggerInfo ti;
    ti.ID = i;
    ti.mode = 0;
    ti.externalID = 0;
    ti.period = 0;
    ti.offset = 0;
    ti.trackingFactor = 1;
    m_triggers.push_back(ti);
  }

  // Find out about our pose devices
  bool havePoseOrientation = false;
  bool havePosePosition = false;
  for (auto const& p : GetPoseDeviceConfigurations()) {
    if (p.ReturnsOrientation) {
      havePoseOrientation = true;
    }
    if (p.ReturnsPosition) {
      havePosePosition = true;
    }
  }

  // Update our feature list.
  if (m_numTemperaturesPerCamera > 0 || m_numSystemTemperatures > 0) {
    m_features.push_back(TEMPERATURE_API_AVAILABLE);
  }
  if (havePoseOrientation) {
    m_features.push_back(POSE_API_ORIENTATION_AVAILABLE);
  }
  if (havePosePosition) {
    m_features.push_back(POSE_API_POSITION_AVAILABLE);
  }

  // Configure all cameras to be triggered off of the first trigger.
  // Fill in the camera information.
  /// @todo Enable config-file-based setting of the camera trigger configuration.
  for (size_t c = 1; c <= m_numCameras; c++) {
    // Configure the camera for trigger 1
    if (!SetupCamera(c, 1)) {
      m_error = "Error setting up camera " + std::to_string(c) + " for trigger 1";
      continue;
    }

    // Get the rest of the camera information
    // Store the camera information.
    CameraInfo ci;
    ci.trigger = 1;
    std::map<std::string, std::string> camInfo = GetStatus(c);
    if (camInfo.find("type") != camInfo.end()) {
      ci.type = atoi(camInfo["type"].c_str());
    }
    if (camInfo.find("width") != camInfo.end()) {
      ci.width = atoi(camInfo["width"].c_str());
    }
    if (camInfo.find("height") != camInfo.end()) {
      ci.height = atoi(camInfo["height"].c_str());
    }
    if (camInfo.find("minTriggerPeriod") != camInfo.end()) {
      ci.minTriggerPeriod = atof(camInfo["minTriggerPeriod"].c_str());
    }
    if (camInfo.find("maxTriggerPeriod") != camInfo.end()) {
      ci.maxTriggerPeriod = atof(camInfo["maxTriggerPeriod"].c_str());
    }

    m_cameras.push_back(ci);
  }
}

Status Core_Module_Server::GetCurrentStatus() const
{
  if (m_error != "") {
    return INTERNAL_EXCEPTION;
  }
  return OKAY;
}

void Core_Module_Server::doEveryLoop()
{
  // Check the temperatures and see if there are any new ones. If so, send them to any clients that are
  // receiving them.
  // We ignore any errors here; the parent class watches to see if a client dies.
  std::vector<temp_s> temps = GetNewTemperatures();
  for (auto const& temp: temps) {
    for (auto& client : m_clients) {
      if (client.m_streamingTemperatures) {
        std::shared_ptr<StreamPacket> packet;
        client.m_writer->GetCurrentPacket(packet);
        Time timeCode = { temp.Seconds, temp.Microseconds };
        uint16_t cameraID = temp.CameraIndex;
        uint16_t sensorID = temp.SensorIndex;
        float temperatureCelcius = temp.Temperature;
        MessageTemperature message(*packet, timeCode, cameraID, sensorID, temperatureCelcius);
      }
    }
  }

  // Check the poses and see if there are any new ones. If so, send them to any clients that are
  // receiving them.
  // We ignore any errors here; the parent class watches to see if a client dies.
  std::vector<pose_s> poses = GetNewPoses();
  for (auto const& pose: poses) {
    for (auto& client : m_clients) {
      if (client.m_streamingPoses) {
        std::shared_ptr<StreamPacket> packet;
        client.m_writer->GetCurrentPacket(packet);
        Time timeCode = { pose.Seconds, pose.Microseconds };
        MessagePose message(*packet, timeCode, pose.Longitude, pose.Latitude, pose.Altitude,
          pose.Rotation, pose.Velocity, pose.RotationVelocity);
      }
    }
  }

  // Flush the writers for all clients in case we have new data for them.
  // We ignore any errors here; the parent class watches to see if a client dies.
  for (auto& client : m_clients) {
    client.m_writer->Flush();
  }
}

void Core_Module_Server::sendInternalErrorMessage(OpCode opCode, ClientState& client)
{
  if (m_verbosity >= 1) {
    std::cout << "Internal failure occurred"<< std::endl;
  }

  std::shared_ptr<StreamWriter> writer = client.m_writer;
  uint8_t priority = 0;
  EventID type = INTERNAL_ERROR;
  if (client.m_eventVerbosity < priority) {
    return;
  }
  std::shared_ptr<StreamPacket> packet;
  Status status = writer->GetCurrentPacket(packet);
  if (status != OKAY) {
    m_error = "Error getting current packet from StreamWriter: " + ErrorMessage(status);
    return;
  }
  Time timeCode;
  m_timer->GetCoreTime(timeCode);
  MessageEvent message(*packet, timeCode, priority, type, std::to_string(opCode));
  if (message.GetConstructorStatus() != OKAY) {
    m_error = "Error constructing MessageEvent: " + ErrorMessage(message.GetConstructorStatus());
    return;
  }

  // Send the packet immediately.
  status = writer->Flush();
  if (status != OKAY) {
    m_error = "Error flushing StreamWriter: " + ErrorMessage(status);
    return;
  }
}

void Core_Module_Server::doReset(const CommandPacketReset& command, ClientState& client)
{
  m_error = "@todo implement doReset";
}

void Core_Module_Server::doConfigureTrigger(const CommandPacketConfigureTrigger& command, ClientState& client)
{
  // Get the trigger info
  TriggerInfo ti;
  Status status = command.GetConfiguration(ti);
  if (status != OKAY) {
    sendInternalErrorMessage(CONFIGURE_TRIGGER, client);
    m_error = "Error getting trigger configuration: " + ErrorMessage(status);
    return;
  }

  // Configure the trigger.
  if (!(ConfigureTrigger(ti.ID, ti.mode, ti.externalID, ti.period, ti.trackingFactor))) {
    sendInternalErrorMessage(CONFIGURE_TRIGGER, client);
    m_error = "Error configuring trigger";
    return;
  }

  m_error = "";
}

void Core_Module_Server::doSoftwareTrigger(const CommandPacketSoftwareTrigger& command, ClientState& client)
{
  // Get the ID and time of the trigger
  uint8_t triggerID;
  Status status = command.GetID(triggerID);
  if (status != OKAY) {
    sendInternalErrorMessage(SOFTWARE_TRIGGER, client);
    m_error = "Error getting trigger ID: " + ErrorMessage(status);
    return;
  }
  Time timeCode;
  status = command.GetInitialTime(timeCode);
  if (status != OKAY) {
    sendInternalErrorMessage(SOFTWARE_TRIGGER, client);
    m_error = "Error getting trigger initial time: " + ErrorMessage(status);
    return;
  }

  // Send the trigger
  if (!(SoftwareTrigger(triggerID, timeCode.seconds, timeCode.microseconds))) {
    sendInternalErrorMessage(SOFTWARE_TRIGGER, client);
    m_error = "Error sending software trigger";
    return;
  }

  m_error = "";
}

void Core_Module_Server::doStreamSubregion(const CommandPacketStreamSubregion& command, ClientState& client)
{
  m_error = "@todo implement doStreamSubregion";
}

void Core_Module_Server::doCancelSubregion(const CommandPacketCancelSubregion& command, ClientState& client)
{
  m_error = "@todo implement doCancelSubregion";
}
