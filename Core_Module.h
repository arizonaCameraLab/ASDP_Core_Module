/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#pragma once
#include <ASDP_Core_API.h>
#include <ASDP_ClockSynchronizer.h>
#include <BspInclude.hpp>
#include <map>
#include <list>

namespace asdp {

/// @brief Per-serial-number server module that advertises on a specific port.
class Core_Module_Server : public CoreServerBase {
public:

  /// @brief Constructor
  /// @param NicName The name of the network interface to listen on for incoming connections.
  /// @param sendPort The port to broadcast notifications on.
  /// @param listenPort The port to listen on for incoming connections.
  /// @param maxPayloadSize The maximum size of a payload that can be sent or received.
  /// @param verbosity The verbosity level of the server, 0 for no verbosity, higher for more verbosity.
  Core_Module_Server(const std::string &NicName, uint16_t sendPort = 10102, uint16_t listenPort = 10101,
    uint32_t maxPayloadSize = 9000 - 28, int verbosity = 0);

  /// @brief Get the current status of the object.
  Status GetCurrentStatus() const;


protected:
  //=============================================================================
  // Additional state information for the server module.
  uint16_t m_numCameras; ///< The number of cameras that are connected to the server.
  std::vector<Bsp::Pose::pose_report_s> m_poseDeviceConfigurations; ///< The configurations for the pose devices.

  //=============================================================================
  // Additional helper functions for the server module.
  void sendInternalErrorMessage(OpCode opCode, ClientState& client);
  void sendInvalidOperationErrorMessage(OpCode opCode, ClientState& client);

  //=============================================================================
  // Clock synchronization.
  std::shared_ptr<asdp::ClockSynchronizer> m_clockSynchronizer; ///< The clock synchronizer for the server.
  std::chrono::steady_clock::time_point m_lastClockSync; ///< The last time the clock was synchronized.

  //=============================================================================
  // Override methods to implement the commands as needed.

  /// @brief Handle operations that must be done every loop, like checking if our thread should stop.
  void doEveryLoop() override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doReset(const CommandPacketReset&, ClientState& client) override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doSetNUCFlagState(const CommandPacketSetNUCFlagState& command, ClientState& client) override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doStartOnCameraNUC(const CommandPacketStartOnCameraNUC& command, ClientState& client) override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doConfigureTrigger(const CommandPacketConfigureTrigger&, ClientState& client) override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doSoftwareTrigger(const CommandPacketSoftwareTrigger&, ClientState& client) override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doStreamSubregion(const CommandPacketStreamSubregion&, ClientState& client) override;

  /// @brief Implement the specified command.
  /// @param command The command packet to implement.
  /// @param client The client that the command is coming from.
  void doCancelSubregion(const CommandPacketCancelSubregion&, ClientState& client) override;

  /// @brief Handle things that have to happen when a client is removed.
  /// @details This enforces clean shutdown even if the client does not close all of the
  /// existing streams.
  void clientBeingRemoved(ClientState& client) override;

  struct OngoingStreamInfo {
    uint16_t cameraIndex;
    Bsp::UdpInserter::endpoint_s endpoint;
  };
  std::map< ClientState *, std::list<OngoingStreamInfo> > m_ongoingStreams; ///< The streams that are currently ongoing.

  /// @todo Override the other methods as needed.
};

} // namespace asdp
