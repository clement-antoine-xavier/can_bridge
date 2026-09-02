#include "../include/can_controller.h"

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool can_controller::begin(uint32_t baudrate, uint8_t cs_pin)
{
  _cs_pin = cs_pin;

  SPI.begin();

  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);

  if (!reset())
  {
    return false;
  }

  // Enter configuration mode before touching CNF / filter registers.
  if (!setMode(CANCTRL_REQOP_CONFIG))
  {
    return false;
  }

  if (!setBitTiming(baudrate))
  {
    return false;
  }

  configureAcceptAll();

  // Switch to normal operation mode.
  if (!setMode(CANCTRL_REQOP_NORMAL))
  {
    return false;
  }

  _initialized = true;
  return true;
}

bool can_controller::sendFrame(const uint8_t *data, uint8_t len,
                               uint32_t id, bool extended, bool rtr)
{
  if (!_initialized)
  {
    return false;
  }
  if (len > MAX_DATA_LEN)
  {
    len = MAX_DATA_LEN;
  }

  uint8_t bufferIndex = 0;
  if (!waitForTxBuffer(bufferIndex))
  {
    return false; // No TX buffer became free within the timeout.
  }

  const uint8_t base = REG_TXB0CTRL + (bufferIndex * 0x10);

  // Build the full TX buffer image in RAM: SIDH, SIDL, EID8, EID0, DLC,
  // then up to 8 data bytes. Writing it in a single SPI burst avoids the
  // per-register transaction overhead of the naive approach.
  uint8_t buffer[1 + 4 + MAX_DATA_LEN];
  uint8_t n = 0;

  if (extended)
  {
    // 29-bit extended ID: SIDH/SIDL hold the upper 11 bits, EID8/EID0 the rest.
    buffer[n++] = (uint8_t)(id >> 21);           // SIDH
    buffer[n++] = (uint8_t)((id >> 13) & 0xE0) | // SIDL (upper 3)
                  ((id >> 16) & 0x03) | 0x08;    // EXIDE + EID[17:16]
    buffer[n++] = (uint8_t)(id >> 8);            // EID8
    buffer[n++] = (uint8_t)id;                   // EID0
  }
  else
  {
    // 11-bit standard ID: SIDH/SIDL hold the identifier, EXIDE cleared.
    buffer[n++] = (uint8_t)(id >> 3);          // SIDH
    buffer[n++] = (uint8_t)((id & 0x07) << 5); // SIDL
    buffer[n++] = 0x00;                        // EID8
    buffer[n++] = 0x00;                        // EID0
  }

  // DLC register: RTR bit (bit 6) + data length code (bits 3:0).
  uint8_t dlc = len & 0x0F;
  if (rtr)
  {
    dlc |= DLC_RTR;
  }
  buffer[n++] = dlc;

  // Payload bytes (skipped for RTR frames, which carry no data).
  if (!rtr && len > 0)
  {
    for (uint8_t i = 0; i < len; ++i)
    {
      buffer[n++] = data[i];
    }
  }

  // Write the whole buffer in one transaction.
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_WRITE);
  SPI.transfer(base + 0x01);
  for (uint8_t i = 0; i < n; ++i)
  {
    SPI.transfer(buffer[i]);
  }
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();

  // Request transmission.
  requestToSend(bufferIndex);
  return true;
}

bool can_controller::recvFrame(uint8_t *out_data, uint8_t &out_len,
                               uint32_t &out_id, bool &out_extended,
                               bool &out_rtr)
{
  if (!_initialized)
  {
    return false;
  }

  uint8_t intf = readRegister(REG_CANINTF);

  uint8_t base = 0;
  uint8_t flag = 0;
  if (intf & CANINTF_RX0IF)
  {
    base = REG_RXB0CTRL;
    flag = CANINTF_RX0IF;
  }
  else if (intf & CANINTF_RX1IF)
  {
    base = REG_RXB1CTRL;
    flag = CANINTF_RX1IF;
  }
  else
  {
    return false; // No pending message.
  }

  // Read the whole RX buffer (SIDH, SIDL, EID8, EID0, DLC, then up to 8 data
  // bytes) in a single SPI burst to minimize per-register transaction cost.
  uint8_t buffer[4 + 1 + MAX_DATA_LEN];
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer(base + 0x01);
  for (uint8_t i = 0; i < sizeof(buffer); ++i)
  {
    buffer[i] = SPI.transfer(0x00);
  }
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();

  // Parse the identifier.
  uint8_t sidh = buffer[0];
  uint8_t sidl = buffer[1];
  uint8_t eid8 = buffer[2];
  uint8_t eid0 = buffer[3];

  out_extended = (sidl & 0x08) != 0; // EXIDE bit.
  if (out_extended)
  {
    out_id = ((uint32_t)sidh << 21) |
             ((uint32_t)(sidl & 0xE0) << 13) |
             ((uint32_t)(sidl & 0x03) << 16) |
             ((uint32_t)eid8 << 8) |
             eid0;
  }
  else
  {
    out_id = ((uint32_t)sidh << 3) | (sidl >> 5);
  }

  // DLC register: RTR bit (bit 6) + data length code (bits 3:0).
  uint8_t dlc = buffer[4];
  out_rtr = (dlc & DLC_RTR) != 0;
  out_len = dlc & 0x0F;
  if (out_len > MAX_DATA_LEN)
  {
    out_len = MAX_DATA_LEN;
  }

  // Copy the payload (RTR frames carry no data bytes).
  if (!out_rtr && out_len > 0)
  {
    for (uint8_t i = 0; i < out_len; ++i)
    {
      out_data[i] = buffer[5 + i];
    }
  }

  // Clear the interrupt flag for this buffer.
  modifyRegister(REG_CANINTF, flag, 0x00);
  return true;
}

bool can_controller::poll()
{
  if (!_initialized)
  {
    return false;
  }

  // Clear transient error flags so they do not accumulate. EFLG is a
  // read/write register; bits are cleared by writing 0 to them.
  modifyRegister(REG_EFLG, 0xFF, 0x00);

  // Clear the error interrupt flag if set.
  uint8_t intf = readRegister(REG_CANINTF);
  if (intf & CANINTF_ERRIF)
  {
    modifyRegister(REG_CANINTF, CANINTF_ERRIF, 0x00);
  }

  // Report healthy if we are in normal mode.
  return (readRegister(REG_CANSTAT) & CANSTAT_OPMOD_MASK) == CANSTAT_OPMOD_NORMAL;
}

bool can_controller::isRxAvailable() const
{
  if (!_initialized)
  {
    return false;
  }
  uint8_t intf = readRegister(REG_CANINTF);
  return (intf & (CANINTF_RX0IF | CANINTF_RX1IF)) != 0;
}

bool can_controller::reset()
{
  // RESET command: reinitializes the MCP2515 to its power-on state.
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_RESET);
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();

  // Give the controller time to complete the reset sequence.
  delay(10);

  // Verify the reset took effect: after reset the device is in
  // configuration mode (OPMOD = 100).
  if ((readRegister(REG_CANSTAT) & CANSTAT_OPMOD_MASK) != CANSTAT_OPMOD_CONFIG)
  {
    _initialized = false;
    return false;
  }

  _initialized = false;
  return true;
}

// ---------------------------------------------------------------------------
// SPI register access helpers
// ---------------------------------------------------------------------------

uint8_t can_controller::readRegister(uint8_t addr) const
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_READ);
  SPI.transfer(addr);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
  return value;
}

void can_controller::writeRegister(uint8_t addr, uint8_t data)
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_WRITE);
  SPI.transfer(addr);
  SPI.transfer(data);
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}

void can_controller::modifyRegister(uint8_t addr, uint8_t mask,
                                    uint8_t data)
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_BIT_MODIFY);
  SPI.transfer(addr);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}

uint8_t can_controller::readStatus()
{
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(CMD_READ_STATUS);
  uint8_t status = SPI.transfer(0x00);
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
  return status;
}

// ---------------------------------------------------------------------------
// MCP2515 command helpers
// ---------------------------------------------------------------------------

void can_controller::requestToSend(uint8_t buffer_index)
{
  // RTS command: bit 2 selects TXB0, bit 1 TXB1, bit 0 TXB2.
  uint8_t cmd = CMD_RTS | (0x01 << buffer_index);
  SPI.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(cmd);
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}

bool can_controller::setMode(uint8_t reqop)
{
  // Write the requested mode into the REQOP bits of CANCTRL.
  modifyRegister(REG_CANCTRL, CANCTRL_REQOP_MASK, reqop);
  return waitForMode(reqop);
}

bool can_controller::waitForMode(uint8_t reqop)
{
  uint32_t start = millis();
  while ((millis() - start) < MODE_TIMEOUT_MS)
  {
    uint8_t opmod = readRegister(REG_CANSTAT) & CANSTAT_OPMOD_MASK;
    if (opmod == reqop)
    {
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// Configuration helpers
// ---------------------------------------------------------------------------

bool can_controller::setBitTiming(uint32_t baudrate)
{
  // Bit timing values assume an 8 MHz oscillator on the MKR CAN Shield.
  // The MCP2515 divides the oscillator by 2 to produce the TQ clock, so the
  // effective TQ frequency is 4 MHz. CNF1[5:0] is the prescaler (minus 1).
  //
  //   TQ/bit = (BRP + 1) * (SYNC + PS1 + PS2 + 1)
  //
  // The values below use a 1 TQ sync segment, PS1 = 3 TQ, PS2 = 3 TQ
  // (sample point at ~87.5%) for 500k/1M, and PS1 = 7 TQ, PS2 = 7 TQ
  // (sample point at ~87.5%) for 125k/250k.
  uint8_t cnf1, cnf2, cnf3;
  switch (baudrate)
  {
  case 1000000:
    // 4 TQ/bit: BRP=0, PS1=1, PS2=1.
    cnf1 = 0x00;
    cnf2 = 0x88; // PS1=1 (bits 5:3), PRSEG=1 (bits 2:0).
    cnf3 = 0x02; // PS2=1 (bits 2:0).
    break;
  case 500000:
    // 8 TQ/bit: BRP=0, PS1=3, PS2=3.
    cnf1 = 0x00;
    cnf2 = 0x99; // PS1=3 (bits 5:3), PRSEG=1 (bits 2:0).
    cnf3 = 0x03; // PS2=3 (bits 2:0).
    break;
  case 250000:
    // 16 TQ/bit: BRP=0, PS1=7, PS2=7.
    cnf1 = 0x00;
    cnf2 = 0xB9; // PS1=7 (bits 5:3), PRSEG=1 (bits 2:0).
    cnf3 = 0x07; // PS2=7 (bits 2:0).
    break;
  case 125000:
    // 16 TQ/bit with BRP=1: BRP=1, PS1=7, PS2=7.
    cnf1 = 0x01;
    cnf2 = 0xB9; // PS1=7 (bits 5:3), PRSEG=1 (bits 2:0).
    cnf3 = 0x07; // PS2=7 (bits 2:0).
    break;
  default:
    return false; // Unsupported baud rate.
  }

  writeRegister(REG_CNF1, cnf1);
  writeRegister(REG_CNF2, cnf2);
  writeRegister(REG_CNF3, cnf3);
  return true;
}

void can_controller::configureAcceptAll()
{
  // Configure both RX buffers to accept all messages (RXM = 00). With RXM
  // set to "accept all", the acceptance masks and filters are ignored, so
  // there is no need to program them.
  modifyRegister(REG_RXB0CTRL, RXBCTRL_RXM_MASK, RXBCTRL_RXM_ALL);
  modifyRegister(REG_RXB1CTRL, RXBCTRL_RXM_MASK, RXBCTRL_RXM_ALL);
}

bool can_controller::waitForTxBuffer(uint8_t &buffer_index)
{
  uint32_t start = millis();
  while ((millis() - start) < TX_TIMEOUT_MS)
  {
    // READ STATUS returns all three TXREQ bits in a single transaction.
    // Bit 2 = TX0REQ, bit 3 = TX1REQ, bit 4 = TX2REQ.
    uint8_t status = readStatus();
    if (!(status & 0x04))
    {
      buffer_index = 0;
      return true;
    }
    if (!(status & 0x08))
    {
      buffer_index = 1;
      return true;
    }
    if (!(status & 0x10))
    {
      buffer_index = 2;
      return true;
    }
  }
  return false;
}
