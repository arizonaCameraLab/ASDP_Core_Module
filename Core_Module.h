/*
 * Copyright (C) 2024: Arizona Board of Regents on Behalf of the University of Arizona
 */

#pragma once
#include <ASDP_Core_API.h>
#include <ASDP_ClockSynchronizer.h>
#include <BspInclude.hpp>
#include <map>
#include <list>
#include <mutex>

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

  /// @brief Get the mapping from API camera ID to BSP camera ID.
  /// @return A map from API camera ID to BSP camera ID.
  std::map<uint16_t, uint16_t> GetAPItoBSPMapping() const { return m_API_IDtoBSP_ID; }

protected:
  //=============================================================================
  // Additional state information for the server module.
  std::map<uint16_t, uint16_t> m_API_IDtoBSP_ID; ///< Map from API camera ID to BSP ID.
  std::map<uint16_t, uint16_t> m_BSP_IDtoAPI_ID; ///< Map from BSP camera ID to API ID.
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
  // Callbacks from the BSP library.
  static void NUCFlagStateCallback(uint16_t Index, bool State, uint32_t Seconds, uint32_t Microseconds, void* UserData);
  static void OnCameraNUCStateCallback(uint16_t Index, bool State, uint32_t Seconds, uint32_t Microseconds, void* UserData);

  //=============================================================================
  // Mutex to protect writing to the clients. Some of the writing is done in the callbacks
  // from the BSP library, which are in a different thread than the main loop.
  /// @todo This could become a map from ClientState* to mutex, to allow more concurrency.
  std::mutex m_writerMutex; ///< Mutex to protect writing to the clients.

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
  /// The streams that are currenly ongoing.  We look them up by the raw pointer to the ClientInfo
  /// rather than the ClientState because these get moved around the the m_clients vector.
  std::map< ClientInfo*, std::list<OngoingStreamInfo> > m_ongoingStreams;

  /// @todo Override the other methods as needed.
};

} // namespace asdp
