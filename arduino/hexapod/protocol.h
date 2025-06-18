/*
 * protocol.h — Pi <-> Arduino wire contract (C mirror of pi/comms/protocol.py).
 *
 * Frame layout (both directions):
 *   [0xAA][0x55][TYPE][LEN][PAYLOAD ...][CRC8]
 *   CRC8 (poly 0x07, init 0x00) covers TYPE, LEN, and PAYLOAD (not the sync
 *   bytes). Payloads are little-endian, which the AVR is natively, so no
 *   byte-swapping is needed.
 *
 * Keep these definitions byte-for-byte identical to protocol.py.
 */
#ifndef HEXAPOD_PROTOCOL_H
#define HEXAPOD_PROTOCOL_H

#include <Arduino.h>

static const uint8_t SYNC0 = 0xAA;
static const uint8_t SYNC1 = 0x55;

static const uint8_t MSG_SERVO     = 0x01;  // Pi  -> Arduino : 18× uint16 us
static const uint8_t MSG_TELEMETRY = 0x02;  // Arduino -> Pi  : dist,roll,pitch,contacts

static const uint8_t  SERVO_PAYLOAD_LEN     = 36;  // 18 servos × uint16
static const uint8_t  TELEMETRY_PAYLOAD_LEN = 7;
static const uint16_t DISTANCE_NO_ECHO      = 0xFFFF;

// CRC-8 (poly 0x07, init 0x00). crc8_update() folds in one byte at a time so the
// receiver can checksum a frame as it streams in, with no extra buffering.
static inline uint8_t crc8_update(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (uint8_t i = 0; i < 8; i++) {
    if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
    else            crc = (uint8_t)(crc << 1);
  }
  return crc;
}

static inline uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) crc = crc8_update(crc, data[i]);
  return crc;
}

#endif  // HEXAPOD_PROTOCOL_H
