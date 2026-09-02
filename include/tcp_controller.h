#pragma once

#include <Arduino.h>
#include <WiFiNINA.h>

/**
 * @file tcp_controller.h
 * @brief Standalone TCP client used to stream CAN frames to a remote host.
 *
 * This class wraps a WiFiNINA TCP client so the sketch can push encoded CAN
 * frames to a configurable server and receive commands back over the same
 * connection. It uses only the standard <WiFiNINA.h> API and depends on the
 * WiFi link already being established by the caller.
 */

/**
 * @class tcp_controller
 * @brief A small, reconnect-capable TCP client for CAN-bridge traffic.
 *
 * A TCP connection is connection-oriented, so unlike the SPI CAN driver the
 * send/receive helpers must tolerate a disconnected peer. This class tracks a
 * single client, transparently attempts to (re)connect, and exposes blocking
 * send and receive primitives that report success or failure.
 */
class tcp_controller
{
public:
  /**
   * @brief Construct a controller that will dial the given server.
   * @param host  IP address or hostname of the remote server.
   * @param port  Remote TCP port.
   */
  tcp_controller(const char *host, uint16_t port);

  /**
   * @brief Establish (or re-establish) the TCP connection.
   *        No-op if already connected.
   * @return true if connected after the attempt.
   */
  bool connect();

  /**
   * @brief Send the given buffer over the socket.
   * @param data Pointer to the bytes to transmit.
   * @param len  Number of bytes to send.
   * @return true if all bytes were written to the socket.
   */
  bool send(const uint8_t *data, size_t len);

  /**
   * @brief Block up to @p timeout_ms for data, then copy it to the caller.
   * @param out_data Buffer to receive the payload.
   * @param out_len  In: capacity of @p out_data.
   *                 Out: number of bytes actually received (0 on none/timeout).
   * @param timeout_ms How long to wait for at least one byte.
   * @return true if at least one byte was received.
   */
  bool recv(uint8_t *out_data, size_t &out_len, uint32_t timeout_ms);

  /**
   * @brief Check whether the socket is currently open.
   * @return true if connected and not closed by the peer.
   */
  bool isConnected() const;

  /**
   * @brief Close the socket and mark the client disconnected.
   */
  void disconnect();

  /**
   * @brief Return a descriptive status string.
   */
  const char *status() const;

private:
  // ---------------------------------------------------------------------
  // Timeouts
  // ---------------------------------------------------------------------
  static constexpr uint32_t CONNECT_TIMEOUT_MS = 5000;
  static constexpr uint32_t RECV_CHUNK_MS = 10; // Poll interval for available().

  // ---------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------
  const char *_host;
  uint16_t _port;
  WiFiClient _client;
  bool _connected = false;
};
