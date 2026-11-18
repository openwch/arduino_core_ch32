// CAN1 for the CH32V30x and CH32V4x7; needs a CAN transceiver on the pins.
// Pins PA11/PA12 by default, or PB8/PB9 and PD0/PD1 with setPins() before begin().
#pragma once

#include <Arduino.h>

#if !defined(CH32V30x) && !defined(CH32V4x7)
#error "This CAN library supports the CH32V30x and CH32V4x7"
#endif

class CANClass : public Stream {
public:
  // pins: rx/tx pairs PA11/PA12, PB8/PB9 or PD0/PD1; returns 0 for any other pair
  int setPins(uint32_t rx, uint32_t tx);
  int begin(long baudRate);
  void end();
  // TX is fed back into RX inside the chip: a test without transceiver or bus
  int loopback();

  int beginPacket(long id, int dlc = -1, bool rtr = false);
  int beginExtendedPacket(long id, int dlc = -1, bool rtr = false);
  // 1 when the frame was sent (acknowledged by another node, or looped back)
  int endPacket();

  int parsePacket();
  long packetId() { return _rxId; }
  bool packetExtended() { return _rxExtended; }
  bool packetRtr() { return _rxRtr; }
  int packetDlc() { return _rxDlc; }

  // only frames with (id & mask) == (filter id & mask) get through; default: all frames
  int filter(long id, long mask = 0x7ff);
  int filterExtended(long id, long mask = 0x1fffffff);

  size_t write(uint8_t byte) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  int available() override;
  int read() override;
  int peek() override;
  void flush() override {}
  using Print::write;

private:
  int init(uint8_t mode);
  void acceptAll();

  uint8_t _pins = 0;            // 0 PA11/PA12, 1 PB8/PB9, 2 PD0/PD1
  long _baudRate = 0;
  bool _started = false;

  bool _txStarted = false;
  long _txId = 0;
  bool _txExtended = false;
  bool _txRtr = false;
  int _txDlc = -1;
  uint8_t _txLength = 0;
  uint8_t _txData[8];

  long _rxId = -1;
  bool _rxExtended = false;
  bool _rxRtr = false;
  int _rxDlc = 0;
  uint8_t _rxLength = 0;
  uint8_t _rxIndex = 0;
  uint8_t _rxData[8];
};

extern CANClass CAN;
