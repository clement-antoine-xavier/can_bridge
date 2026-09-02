#pragma once

#include <Arduino.h>
#include <SPI.h>

/**
 * @file can_controller.h
 * @brief Standalone driver for the Microchip MCP2515 CAN controller.
 *
 * This class drives the MCP2515 over SPI using only the standard Arduino
 * <SPI.h> and core APIs. It is tailored to the Arduino MKR CAN Shield
 * (MCP2515 + TJA1049 transceiver) but works with any MCP2515 wired to SPI.
 *
 * No external CAN libraries are used.
 */

/**
 * @class can_controller
 * @brief A compact, production-oriented driver for the MCP2515.
 *
 * The MCP2515 is a standalone CAN 2.0B controller. It is configured over SPI
 * and exposes three transmit buffers (TXB0/1/2) and two receive buffers
 * (RXB0/1). This class wraps the SPI register access, bit-timing setup,
 * message transmission and reception, and basic error handling.
 */
class can_controller
{
public:
  /**
   * @brief Initialize SPI, reset the MCP2515, configure bit timing and
   *        filters, and enter normal operation mode.
   * @param baudrate Desired CAN bit rate in bits/s (125000, 250000, 500000,
   *                 1000000 supported).
   * @param cs_pin   SPI chip-select pin (defaults to the board's SPI SS).
   * @return true on success, false if the controller did not enter normal mode.
   */
  bool begin(uint32_t baudrate, uint8_t cs_pin = PIN_SPI_SS);

  /**
   * @brief Transmit a CAN frame using the first free TX buffer.
   * @param data     Payload bytes (up to 8).
   * @param len      Number of payload bytes (0..8).
   * @param id       CAN identifier (11-bit standard or 29-bit extended).
   * @param extended true for a 29-bit extended identifier.
   * @param rtr      true to send a Remote Transmission Request frame.
   * @return true if the frame was loaded and transmission was requested.
   */
  bool sendFrame(const uint8_t *data, uint8_t len, uint32_t id = 0,
                 bool extended = false, bool rtr = false);

  /**
   * @brief Receive a frame from the first RX buffer that has data.
   * @param out_data    Buffer to receive the payload (at least 8 bytes).
   * @param out_len     Receives the payload length.
   * @param out_id      Receives the CAN identifier.
   * @param out_extended Receives whether the identifier was extended.
   * @param out_rtr     Receives whether the frame was a remote request.
   * @return true if a frame was read, false if no message was pending.
   */
  bool recvFrame(uint8_t *out_data, uint8_t &out_len, uint32_t &out_id,
                 bool &out_extended, bool &out_rtr);

  /**
   * @brief Non-blocking housekeeping: clear error flags and update state.
   * @return true if the controller is in normal mode and error-free.
   */
  bool poll();

  /**
   * @brief Check whether at least one RX buffer holds a pending message.
   * @return true if a message is available to read.
   */
  bool isRxAvailable() const;

  /**
   * @brief Send the SPI RESET command and reinitialize to a known state.
   * @return true on success.
   */
  bool reset();

private:
  // ---------------------------------------------------------------------
  // MCP2515 SPI command opcodes
  // ---------------------------------------------------------------------
  static constexpr uint8_t CMD_RESET = 0xC0;       // Reset the device.
  static constexpr uint8_t CMD_READ = 0x03;        // Read from a register.
  static constexpr uint8_t CMD_WRITE = 0x02;       // Write to a register.
  static constexpr uint8_t CMD_RTS = 0x80;         // Request to Send (TX).
  static constexpr uint8_t CMD_READ_STATUS = 0xA0; // Read status bits.
  static constexpr uint8_t CMD_BIT_MODIFY = 0x05;  // Masked bit modify.

  // ---------------------------------------------------------------------
  // MCP2515 register addresses
  // ---------------------------------------------------------------------
  static constexpr uint8_t REG_CANSTAT = 0x0E; // Status (mode bits).
  static constexpr uint8_t REG_CANCTRL = 0x0F; // Control (mode + CLKOUT).
  static constexpr uint8_t REG_CNF3 = 0x28;    // Bit timing: phase seg 2.
  static constexpr uint8_t REG_CNF2 = 0x29;    // Bit timing: phase seg 1.
  static constexpr uint8_t REG_CNF1 = 0x2A;    // Bit timing: prescaler.
  static constexpr uint8_t REG_CANINTE = 0x2B; // Interrupt enable.
  static constexpr uint8_t REG_CANINTF = 0x2C; // Interrupt flags.
  static constexpr uint8_t REG_EFLG = 0x2D;    // Error flags.

  // Transmit buffers (base addresses for TXBnCTRL).
  static constexpr uint8_t REG_TXB0CTRL = 0x30;

  // Receive buffers (base addresses for RXBnCTRL).
  static constexpr uint8_t REG_RXB0CTRL = 0x60;
  static constexpr uint8_t REG_RXB1CTRL = 0x70;

  // Acceptance masks (RXMnSIDH base addresses).
  static constexpr uint8_t REG_RXM0SIDH = 0x20;

  // ---------------------------------------------------------------------
  // Important bit definitions
  // ---------------------------------------------------------------------
  // CANCTRL: REQOP[2:0] request operation mode.
  static constexpr uint8_t CANCTRL_REQOP_NORMAL = 0x00;
  static constexpr uint8_t CANCTRL_REQOP_CONFIG = 0x80;
  static constexpr uint8_t CANCTRL_REQOP_LISTEN = 0x60;
  static constexpr uint8_t CANCTRL_REQOP_MASK = 0xE0;

  // CANSTAT: OPMOD[2:0] reports the current operation mode.
  static constexpr uint8_t CANSTAT_OPMOD_MASK = 0xE0;
  static constexpr uint8_t CANSTAT_OPMOD_NORMAL = 0x00;
  static constexpr uint8_t CANSTAT_OPMOD_CONFIG = 0x80;

  // CANINTF: interrupt flags.
  static constexpr uint8_t CANINTF_RX0IF = 0x01; // RXB0 full.
  static constexpr uint8_t CANINTF_RX1IF = 0x02; // RXB1 full.
  static constexpr uint8_t CANINTF_TX0IF = 0x04; // TXB0 empty.
  static constexpr uint8_t CANINTF_TX1IF = 0x08; // TXB1 empty.
  static constexpr uint8_t CANINTF_TX2IF = 0x10; // TXB2 empty.
  static constexpr uint8_t CANINTF_ERRIF = 0x20; // Error.
  static constexpr uint8_t CANINTF_MERRF = 0x40; // Message error.
  static constexpr uint8_t CANINTF_WAKIF = 0x80; // Wake-up.

  // TXBnCTRL: TXREQ bit requests transmission.
  static constexpr uint8_t TXBCTRL_TXREQ = 0x08;
  static constexpr uint8_t TXBCTRL_TXP = 0x03; // Priority bits.

  // RXBnCTRL: RXM[1:0] acceptance filter mode.
  static constexpr uint8_t RXBCTRL_RXM_MASK = 0x60;
  static constexpr uint8_t RXBCTRL_RXM_ALL = 0x00; // Accept all messages.

  // DLC register: RTR bit (bit 6) + data length code (bits 3:0).
  static constexpr uint8_t DLC_RTR = 0x40;

  // ---------------------------------------------------------------------
  // SPI / timing constants
  // ---------------------------------------------------------------------
  static constexpr uint8_t MAX_DATA_LEN = 8;
  static constexpr uint32_t TX_TIMEOUT_MS = 10;   // Wait for a free TX buffer.
  static constexpr uint32_t MODE_TIMEOUT_MS = 10; // Wait for mode change.
  static constexpr uint32_t SPI_CLOCK = 10000000; // SPI clock for MCP2515.

  // ---------------------------------------------------------------------
  // SPI register access helpers
  // ---------------------------------------------------------------------
  uint8_t readRegister(uint8_t addr) const;
  void writeRegister(uint8_t addr, uint8_t data);
  void modifyRegister(uint8_t addr, uint8_t mask, uint8_t data);
  uint8_t readStatus();

  // ---------------------------------------------------------------------
  // MCP2515 command helpers
  // ---------------------------------------------------------------------
  void requestToSend(uint8_t buffer_index);
  bool setMode(uint8_t reqop);
  bool waitForMode(uint8_t reqop);

  // ---------------------------------------------------------------------
  // Configuration helpers
  // ---------------------------------------------------------------------
  bool setBitTiming(uint32_t baudrate);
  void configureAcceptAll();
  bool waitForTxBuffer(uint8_t &buffer_index);

  // ---------------------------------------------------------------------
  // State
  // ---------------------------------------------------------------------
  uint8_t _cs_pin = PIN_SPI_SS;
  bool _initialized = false;
};
