/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#include "Core_Module.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <filesystem>

using namespace asdp;

Core_Module_Server::Core_Module_Server(const std::string& NicName, uint16_t sendPort, uint16_t listenPort,
  uint32_t maxPayloadSize, int verbosity)
  : CoreServerBase(Bsp::GetSerialNumber(), NicName, sendPort, listenPort, maxPayloadSize, verbosity)
  , m_numCameras(Bsp::Camera::GetNumberOfCameras())
  , m_numInternalTriggers(Bsp::Timing::GetNumberOfInternalTriggers())
  , m_numExternalTriggers(Bsp::Timing::GetNumberOfExternalTriggers())
  , m_numTempsPerCamera(Bsp::Temperature::GetNumberOfTemperatureSensorsPerCamera())
  , m_numSystemTemps(Bsp::Temperature::GetNumberOfSystemTemperatureSensors())
  , m_poseDeviceConfigurations(Bsp::Pose::GetPoseDeviceConfigurations())
{

  /// @todo
}

Status Core_Module_Server::GetCurrentStatus() const
{
  Status status = NOT_IMPLEMENTED;

  /// @todo
  return status;
}

void Core_Module_Server::doEveryLoop()
{
  /// @todo
}

void Core_Module_Server::doReset(const CommandPacketReset& command, ClientState& client)
{
  m_error = "@todo implement doReset";
}

void Core_Module_Server::doConfigureTrigger(const CommandPacketConfigureTrigger& command, ClientState& client)
{
  m_error = "@todo implement doConfigureTrigger";
}

void Core_Module_Server::doSoftwareTrigger(const CommandPacketSoftwareTrigger& command, ClientState& client)
{
  m_error = "@todo implement doSoftwareTrigger";
}

void Core_Module_Server::doStreamSubregion(const CommandPacketStreamSubregion& command, ClientState& client)
{
  m_error = "@todo implement doStreamSubregion";
}

void Core_Module_Server::doCancelSubregion(const CommandPacketCancelSubregion& command, ClientState& client)
{
  m_error = "@todo implement doCancelSubregion";
}
