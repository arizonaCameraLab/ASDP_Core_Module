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

static std::string to_lower(const std::string& input) {
  std::string result = input;
  std::transform(result.begin(), result.end(), result.begin(),
    [](unsigned char c) { return std::tolower(c); });
  return result;
}

Core_Module_Server::Core_Module_Server(const std::string& NicName, uint16_t sendPort, uint16_t listenPort,
  uint32_t maxPayloadSize, int verbosity)
  : CoreServerBase(Bsp::GetSerialNumber(), NicName, sendPort, listenPort, maxPayloadSize, verbosity)
  , m_poseDeviceConfigurations(Bsp::Pose::GetPoseDeviceConfigurations())
  , m_clockSynchronizer(std::make_shared<ClockSynchronizer>(m_timer))
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
  bool haveNUCFlag = false;
  bool haveOnCameraNUC = false;
  for (uint16_t c = 1; c <= Bsp::Camera::GetNumberOfCameras(); c++) {

    // Get the rest of the camera information
    // Store the camera information.
    CameraInfo ci;
    ci.trigger = 1;
    std::map<std::string, std::string> camInfo = GetStatus(c);
    if (camInfo.find("type") != camInfo.end()) {
      ci.type = atoi(camInfo["type"].c_str());
      if (ci.type == 0) {
        // Ignore cameras that are not present. This is a valid state for a camera in the BSP.
        continue;
      }
    }
    if (camInfo.find("width") != camInfo.end()) {
      ci.width = atoi(camInfo["width"].c_str());
    }
    if (camInfo.find("height") != camInfo.end()) {
      ci.height = atoi(camInfo["height"].c_str());
    }
    if (camInfo.find("minTriggerPeriod") != camInfo.end()) {
      ci.minTriggerPeriod = std::stod(camInfo["minTriggerPeriod"]);
    }
    if (camInfo.find("maxTriggerPeriod") != camInfo.end()) {
      ci.maxTriggerPeriod = std::stod(camInfo["maxTriggerPeriod"]);
    }
    if (camInfo.find("nucFlagAvailable") != camInfo.end()) {
      std::string lowerValue = to_lower(camInfo["nucFlagAvailable"]);
      if ((lowerValue == "true") || (lowerValue == "t") || (lowerValue == "1")) {
        haveNUCFlag = true;
        if (!RegisterNUCFlagStateCallback(NUCFlagStateCallback, this)) {
          m_error = "Error registering NUC flag state callback";
        }
      }
    }
    if (camInfo.find("onCameraNUCAvailable") != camInfo.end()) {
      std::string lowerValue = to_lower(camInfo["onCameraNUCAvailable"]);
      if ((lowerValue == "true") || (lowerValue == "t") || (lowerValue == "1")) {
        haveOnCameraNUC = true;
        if (!RegisterOnCameraNUCStateCallback(OnCameraNUCStateCallback, this)) {
          m_error = "Error registering on-camera NUC state callback";
        }
      }
    }

    // Configure the camera for trigger 1
    if (!SetupCamera(c, 1)) {
      m_error = "Error setting up camera " + std::to_string(c) + " for trigger 1";
      continue;
    }

    // We have a complete and ready camera, so add it to our list of cameras.
    m_cameras.push_back(ci);

    // Add the camera to both of our mappings between API ID and BSP ID.
    m_API_IDtoBSP_ID[m_cameras.size()] = c;
    m_BSP_IDtoAPI_ID[c] = m_cameras.size();
  }

  // Add features discovered during the camera handling.
  if (haveNUCFlag) {
    m_features.push_back(NUC_FLAG_AVAILABLE);
  }
  if (haveOnCameraNUC) {
    m_features.push_back(ON_CAMERA_NUC_AVAILABLE);
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
  // We ignore any write errors here; the parent class watches to see if a client dies.
  std::vector<temp_s> temps = GetNewTemperatures();
  for (auto const& temp: temps) {
    for (auto& client : m_clients) {
      if (client.m_streamingTemperatures) {
        std::lock_guard<std::mutex> lock(m_writerMutex);
        std::shared_ptr<StreamPacket> packet;
        client.m_writer->GetCurrentPacket(packet);
        Time timeCode = { temp.Seconds, temp.Microseconds };
        uint16_t cameraID = m_BSP_IDtoAPI_ID[temp.CameraIndex];
        uint16_t sensorID = temp.SensorIndex;
        float temperatureCelsius = temp.Temperature;
        MessageTemperature message(*packet, timeCode, cameraID, sensorID, temperatureCelsius);
        // If we've had an error constructing the message, try flushing the writer to see if that helps.
        // We may be trying to put too much data in the packet.
        Status ret = message.GetConstructorStatus();
        while (ret == WRITE_PAST_END) {
          client.m_writer->Flush();
          client.m_writer->GetCurrentPacket(packet);
          MessageTemperature message(*packet, timeCode, cameraID, sensorID, temperatureCelsius);
          ret = message.GetConstructorStatus();
        }
        if (ret != OKAY) {
          // If we still have an error, just give up on this one.
          if (m_verbosity >= 1) {
            std::cout << "Error constructing MessageTemperature after flush: " << ErrorMessage(ret) << std::endl;
            continue;
          }
        }
      }
    }
  }

  // Check the poses and see if there are any new ones. If so, send them to any clients that are
  // receiving them.
  // We ignore any write errors here; the parent class watches to see if a client dies.
  std::vector<pose_s> poses = GetNewPoses();
  for (auto const& pose: poses) {
    for (auto& client : m_clients) {
      if (client.m_streamingPoses) {
        std::lock_guard<std::mutex> lock(m_writerMutex);
        std::shared_ptr<StreamPacket> packet;
        client.m_writer->GetCurrentPacket(packet);
        Time timeCode = { pose.Seconds, pose.Microseconds };
        MessagePose message(*packet, timeCode, pose.Latitude, pose.Longitude, pose.Altitude,
          pose.Rotation, pose.Velocity, pose.RotationVelocity);
        // If we've had an error constructing the message, try flushing the writer to see if that helps.
        // We may be trying to put too much data in the packet.
        Status ret = message.GetConstructorStatus();
        while (ret == WRITE_PAST_END) {
          client.m_writer->Flush();
          client.m_writer->GetCurrentPacket(packet);
          MessagePose message(*packet, timeCode, pose.Latitude, pose.Longitude, pose.Altitude,
            pose.Rotation, pose.Velocity, pose.RotationVelocity);
          ret = message.GetConstructorStatus();
        }
        if (ret != OKAY) {
          // If we still have an error, just give up on this one.
          if (m_verbosity >= 1) {
            std::cout << "Error constructing MessagePose after flush: " << ErrorMessage(ret) << std::endl;
            continue;
          }
        }
      }
    }
  }

  // Flush the writers for all clients in case we have new data for them.
  // We ignore any errors here; the parent class watches to see if a client dies.
  for (auto& client : m_clients) {
    std::lock_guard<std::mutex> lock(m_writerMutex);
    client.m_writer->Flush();
  }

  // Provide an updated time code to the clock synchronizer if it is time to do so.
  // We see if it has been at least 10ms since the last time we sent a time code.
  auto now = std::chrono::steady_clock::now();
  if (now - m_lastClockSync >= std::chrono::milliseconds(10)) {
    m_lastClockSync = now;
    Time timeCode;
    Bsp::Timing::GetCurrentSystemTime(timeCode.seconds, timeCode.microseconds);
    // Clear all history because the clock can drift.  We always want to use the most recent time code
    // to synchronize the clocks because the Timer does not have an adjustable rate of time flow.
    m_clockSynchronizer->ClearHistory();
    m_clockSynchronizer->AddDataPoint(timeCode);
  }
}

void Core_Module_Server::sendInternalErrorMessage(OpCode opCode, ClientState& client)
{
  if (m_verbosity >= 1) {
    std::cout << "Internal failure occurred"<< std::endl;
  }

  std::lock_guard<std::mutex> lock(m_writerMutex);
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

void Core_Module_Server::sendInvalidOperationErrorMessage(OpCode opCode, ClientState& client)
{
  if (m_verbosity >= 1) {
    std::cout << "Invalid operation occurred" << std::endl;
  }

  std::lock_guard<std::mutex> lock(m_writerMutex);
  std::shared_ptr<StreamWriter> writer = client.m_writer;
  uint8_t priority = 0;
  EventID type = INVALID_OPERATION;
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
  // Reconfigure all of our triggers to be disabled.
  for (size_t i = 1; i <= GetNumberOfInternalTriggers(); i++) {
    ConfigureTrigger(i, 0, 0, 0, 1);
  }

  // Destroy all of our clients.
  while (!m_clients.empty()) {
    m_clients.pop_back();
  }
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
    sendInvalidOperationErrorMessage(CONFIGURE_TRIGGER, client);
    return;
  }
}

void Core_Module_Server::doSetNUCFlagState(const CommandPacketSetNUCFlagState& command, ClientState& client)
{
  uint32_t cameraID;
  Status status = command.GetID(cameraID);
  if (status != OKAY) {
    sendInternalErrorMessage(SET_NUC_FLAG_STATE, client);
    m_error = "Error getting camera ID: " + ErrorMessage(status);
    return;
  }
  uint16_t bspID = m_API_IDtoBSP_ID[cameraID];
  uint32_t flipped;
  status = command.GetState(flipped);
  if (status != OKAY) {
    sendInternalErrorMessage(SET_NUC_FLAG_STATE, client);
    m_error = "Error getting flag state: " + ErrorMessage(status);
    return;
  }
  if (!(SetNUCFlagState(bspID, flipped != 0))) {
    sendInvalidOperationErrorMessage(SET_NUC_FLAG_STATE, client);
    return;
  }
}

void Core_Module_Server::doStartOnCameraNUC(const CommandPacketStartOnCameraNUC& command, ClientState& client)
{
  uint32_t cameraID;
  Status status = command.GetID(cameraID);
  if (status != OKAY) {
    sendInternalErrorMessage(START_ON_CAMERA_NUC, client);
    m_error = "Error getting camera ID: " + ErrorMessage(status);
    return;
  }
  uint16_t bspID = m_API_IDtoBSP_ID[cameraID];
  if (!(StartOnCameraNUC(bspID))) {
    sendInvalidOperationErrorMessage(START_ON_CAMERA_NUC, client);
    return;
  }
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
    sendInvalidOperationErrorMessage(SOFTWARE_TRIGGER, client);
    return;
  }
}

void Core_Module_Server::doStreamSubregion(const CommandPacketStreamSubregion& command, ClientState& client)
{
  // Read the enpoint and subregion descriptions from the command packet.
  StreamEndpoint endpoint;
  Status status = command.GetEndpoint(endpoint);
  if (status != OKAY) {
    sendInternalErrorMessage(STREAM_SUBREGION, client);
    return;
  }
  SubregionDescription subregion;
  status = command.GetRegionDescription(subregion);
  if (status != OKAY) {
    sendInternalErrorMessage(STREAM_SUBREGION, client);
    return;
  }
  uint16_t bspID = m_API_IDtoBSP_ID[subregion.cameraID];

  // See if we already have a stream going to this endpoint for this camera.  If so, first cancel it.
  // The API calls for replacing its parameters, which we do here by the two steps of disabling and enabling.
  Bsp::UdpInserter::endpoint_s endpointS = { endpoint.IP, endpoint.port };
  auto streams = m_ongoingStreams.find(client.m_client.get());
  if (streams != m_ongoingStreams.end()) {
    auto existingStream = std::find_if(streams->second.begin(), streams->second.end(),
      [subregion, &endpointS](const OngoingStreamInfo& info) {
        return (info.cameraIndex == subregion.cameraID) && (endpointS == info.endpoint);
      });
    if (existingStream != streams->second.end()) {
      // We have an existing stream to this endpoint for this camera.  Cancel it first.
      if (!DisableStream(bspID, { endpoint.IP, endpoint.port })) {
        sendInvalidOperationErrorMessage(STREAM_SUBREGION, client);
        return;
      }
      streams->second.erase(existingStream);
    }
  }

  // Request the streaming
  roi_s roi;
  roi.start_x = subregion.left;
  roi.start_y = subregion.top;
  roi.end_x = subregion.right;
  roi.end_y = subregion.bottom;
  if (!EnableStream(bspID, { endpoint.IP, endpoint.port }, roi, subregion.skipFrames,
      subregion.startTimeSeconds, subregion.startTimeMicroseconds)) {
    sendInvalidOperationErrorMessage(STREAM_SUBREGION, client);
    return;
  }

  // Keep track of ongoing streams
  OngoingStreamInfo streamInfo;
  streamInfo.cameraIndex = subregion.cameraID;
  streamInfo.endpoint = { endpoint.IP, endpoint.port };
  m_ongoingStreams[client.m_client.get()].push_back(streamInfo);
}

void Core_Module_Server::doCancelSubregion(const CommandPacketCancelSubregion& command, ClientState& client)
{
  // Read the camera index and endpoint from the command packet.
  uint32_t cameraID;
  Status status = command.GetCamera(cameraID);
  if (status != OKAY) {
    sendInternalErrorMessage(CANCEL_SUBREGION, client);
    return;
  }
  StreamEndpoint endpoint;
  status = command.GetEndpoint(endpoint);
  if (status != OKAY) {
    sendInternalErrorMessage(CANCEL_SUBREGION, client);
    return;
  }
  uint16_t bspID = m_API_IDtoBSP_ID[cameraID];

  // Request the cancelation of the streaming
  if (!DisableStream(bspID, { endpoint.IP, endpoint.port })) {
    sendInvalidOperationErrorMessage(CANCEL_SUBREGION, client);
    return;
  }

  // Remove this from the ongoing-streams list
  Bsp::UdpInserter::endpoint_s endpointS = { endpoint.IP, endpoint.port };
  m_ongoingStreams[client.m_client.get()].remove_if([cameraID, endpoint, &endpointS](const OngoingStreamInfo& info) {
    return (info.cameraIndex == cameraID) && (endpointS == info.endpoint);
  });
}

void Core_Module_Server::clientBeingRemoved(ClientState& client)
{
  // Remove any ongoing streams that this client was responsible for
  auto streams = m_ongoingStreams.find(client.m_client.get());
  if (streams != m_ongoingStreams.end()) {
    for (auto const& stream : streams->second) {
      DisableStream(stream.cameraIndex, stream.endpoint);
    }
    m_ongoingStreams.erase(streams);
  }
}

void Core_Module_Server::NUCFlagStateCallback(uint16_t Index, bool State, uint32_t Seconds, uint32_t Microseconds,
  void* UserData)
{
  Core_Module_Server* me = static_cast<Core_Module_Server*>(UserData);
  if (me == nullptr) {
    return;
  }
  // Send the NUC flag state to all clients.
  for (auto& client : me->m_clients) {
    std::lock_guard<std::mutex> lock(me->m_writerMutex);
    std::shared_ptr<StreamPacket> packet;
    Status status = client.m_writer->GetCurrentPacket(packet);
    if (status != OKAY) {
      me->m_error = "Error getting current packet from StreamWriter: " + ErrorMessage(status);
      return;
    }
    Time timeCode = { Seconds, Microseconds };
    int priority = 1;
    std::string params = std::to_string(Index) + "," + (State ? "1" : "0");
    MessageEvent message(*packet, timeCode, priority, NUC_FLAG_STATE, params.c_str());
    if (message.GetConstructorStatus() != OKAY) {
      me->m_error = "Error constructing MessageEvent: " + ErrorMessage(message.GetConstructorStatus());
      return;
    }
    status = client.m_writer->Flush();
    if (status != OKAY) {
      me->m_error = "Error flushing StreamWriter: " + ErrorMessage(status);
      return;
    }
  }
}

void Core_Module_Server::OnCameraNUCStateCallback(uint16_t Index, bool State, uint32_t Seconds, uint32_t Microseconds,
  void* UserData)
{
  Core_Module_Server* me = static_cast<Core_Module_Server*>(UserData);
  if (me == nullptr) {
    return;
  }
  // Send the NUC flag state to all clients.
  for (auto& client : me->m_clients) {
    std::lock_guard<std::mutex> lock(me->m_writerMutex);
    std::shared_ptr<StreamPacket> packet;
    Status status = client.m_writer->GetCurrentPacket(packet);
    if (status != OKAY) {
      me->m_error = "Error getting current packet from StreamWriter: " + ErrorMessage(status);
      return;
    }
    Time timeCode = { Seconds, Microseconds };
    int priority = 1;
    std::string params = std::to_string(Index) + "," + (State ? "1" : "0");
    MessageEvent message(*packet, timeCode, priority, ON_CAMERA_NUC_STATE, params.c_str());
    if (message.GetConstructorStatus() != OKAY) {
      me->m_error = "Error constructing MessageEvent: " + ErrorMessage(message.GetConstructorStatus());
      return;
    }
    status = client.m_writer->Flush();
    if (status != OKAY) {
      me->m_error = "Error flushing StreamWriter: " + ErrorMessage(status);
      return;
    }
  }
}
