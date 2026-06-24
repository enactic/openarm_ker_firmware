// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include <math.h>

// #define DEVICE_ID 1// Device ID (set in range 0-31)

// ================================
// pin assign
// ================================
#define CLK  16
#define DATA_PIN 14
#define CS   13

#define LED 9

#define TX 7
#define RX 6
#define EN 5

#define SERIAL_BAUD 2000000UL
#define SSC_INTERVAL_MS 20UL      // Max SSC read interval [ms]
#define CHAIN_MAX_ID 16             // Last ID in chain (wraps back to ID=1 after this)

#ifndef DEVICE_ID
#define DEVICE_ID 1 // Default value
#endif

// ================================
// fast I/O wrapper
// Assumes digitalWriteFast / digitalReadFast are available in megaTinyCore
// Falls back to standard functions if undefined in the environment
// ================================
#ifndef digitalWriteFast
  #define digitalWriteFast(pin, val) digitalWrite((pin), (val))
#endif

#ifndef digitalReadFast
  #define digitalReadFast(pin) digitalRead((pin))
#endif

#ifndef pinModeFast
  #define pinModeFast(pin, mode) pinMode((pin), (mode))
#endif

// ================================
// TLE5012B SSC command
// RW (Bit 15): 1 (Read)
// Lock (Bit 14-11): 0000B (default value for access to addresses 00H-04H)
// UPD (Bit 10): normally 0 (read latest value)
// ADDR (Bit 9-4): 000010B (AVAL register address 02H)
// ND (Bit 3-0): 0001B (read 1 word)
// ================================

static const uint16_t CMD_READ_AVAL_FAST = 0x8021;
// static const uint16_t CMD_READ_AVAL_FAST = 0x8200;



// RS485 transmit buffer
static uint8_t tx_data[4];

// Packet receive state
static uint8_t r_id = 0, r_cmd = 0;
static uint32_t r_data = 0;

// Device configuration
static uint8_t device_id = DEVICE_ID;

// Latest sensor data (value sent via RS485)
static uint32_t latest_angle_21bit = 0;

// SSC read timing
static uint32_t last_ssc_time = 0;
static bool do_ssc_read = true;  // Read immediately on first iteration

// ================================
// CRC-8 0x07 calculation
// ================================
uint8_t crc8_0x07(const uint8_t *data, size_t len) {
  const uint8_t polynomial = 0x07;
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = (uint8_t)(crc << 1) ^ polynomial;
      } else {
        crc = (uint8_t)(crc << 1);
      }
    }
  }
  return crc;
}

// ================================
// Optimized to reduce floating-point operations
// ================================

// ================================
// 4-byte packet builder
// ID   : 0-31
// CMD  : 0-3
// data : 0-2^21-1
// ================================
void makePacket21(uint8_t ID, uint8_t CMD, uint32_t data) {
  // Clamp to valid range
  ID   &= 0x1F;          // 5bit
  CMD  &= 0x03;          // 2bit
  uint32_t data21 = data & 0x1FFFFF;  // 21bit

  // Lower 7 bits of header = ID(5bit) + CMD(2bit)
  uint8_t header7 = (ID << 2) | CMD;  // 7bit
  uint8_t header  = 0x80 | header7;   // Set MSB=1

  // Split data into 3 bytes of 7 bits each (MSB always 0)
  uint8_t d0 =  data21        & 0x7F;    // bits [6:0]
  uint8_t d1 = (data21 >> 7)  & 0x7F;    // bits [13:7]
  uint8_t d2 = (data21 >> 14) & 0x7F;    // bits [20:14]

  tx_data[0] = header; // MSB=1
  tx_data[1] = d0;     // MSB=0
  tx_data[2] = d1;     // MSB=0
  tx_data[3] = d2;     // MSB=0
}

// ================================
// RS485 transmit
// ================================
void send485() {
  digitalWrite(LED, HIGH);
  digitalWrite(EN, HIGH);

  // Send payload
  Serial.write(tx_data[0]);
  Serial.write(tx_data[1]);
  Serial.write(tx_data[2]);
  Serial.write(tx_data[3]);

  // Push all bytes to the UART hardware
  Serial.flush();

  // Disable RS-485 driver and turn off LED
  digitalWrite(LED, LOW);
  digitalWrite(EN, LOW);
}

// ================================
// 4-byte packet receiver (non-blocking)
// Returns true on success
// Packet format: [H:1xxxxxxx] [d0:0xxxxxxx] [d1:0xxxxxxx] [d2:0xxxxxxx]
// Lower 7 bits of H = (ID<<2) | CMD
// ================================
bool receivePacket21(uint8_t &ID, uint8_t &CMD, uint32_t &data) {
  enum { WAIT_HEADER, GET_D0, GET_D1, GET_D2 };
  static uint8_t state = WAIT_HEADER;
  static uint8_t header = 0;
  static uint8_t d0 = 0, d1 = 0;

  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    switch (state) {
      case WAIT_HEADER:
        if (b & 0x80) {
          header = b;
          state = GET_D0;
        }
        break;
      case GET_D0:
        if ((b & 0x80) == 0) {
          d0 = b;
          state = GET_D1;
        } else {
          state = WAIT_HEADER;
        }
        break;
      case GET_D1:
        if ((b & 0x80) == 0) {
          d1 = b;
          state = GET_D2;
        } else {
          state = WAIT_HEADER;
        }
        break;
      case GET_D2:
        if ((b & 0x80) == 0) {
          uint8_t d2 = b;
          uint8_t header7 = header & 0x7F;
          ID = header7 >> 2;
          CMD = header7 & 0x03;
          data = (uint32_t)d0 | ((uint32_t)d1 << 7) | ((uint32_t)d2 << 14);
          state = WAIT_HEADER;
          return true;
        } else {
          state = WAIT_HEADER;
        }
        break;
    }
  }

  return false;
}

// ================================
// TLE5012B SSC read functions
// ================================

// Tiny wait (removed: eliminated unnecessary overhead)
// static inline void tinyWait() {
//   __asm__ __volatile__("nop\n\t""nop\n\t");
// }

static inline void clkLow()  { digitalWriteFast(CLK, LOW); }
static inline void clkHigh() { digitalWriteFast(CLK, HIGH); }
static inline void csLow()   { digitalWriteFast(CS, LOW); }
static inline void csHigh()  { digitalWriteFast(CS, HIGH); }

static inline void dataOut() { pinModeFast(DATA_PIN, OUTPUT); }
static inline void dataIn()  { pinModeFast(DATA_PIN, INPUT); }

static inline void dataWrite(uint8_t v) {
  digitalWriteFast(DATA_PIN, v ? HIGH : LOW);
}

static inline uint8_t dataRead() {
  return digitalReadFast(DATA_PIN) ? 1 : 0;
}

// SSC:
// Sensor outputs data on rising edge
// Master reads on falling edge
// Optimized: pinMode calls moved outside bit loop

static inline void sscWrite16(uint16_t v) {
  dataOut();  // Set OUTPUT once at the start
  for (int8_t i = 15; i >= 0; --i) {
    dataWrite((v >> i) & 0x01);
    clkHigh();
    clkLow();
  }
}

static inline uint16_t sscRead16() {
  dataIn();  // Set INPUT once at the start
  uint16_t v = 0;
  for (int8_t i = 15; i >= 0; --i) {
    v <<= 1;
    clkHigh();
    clkLow();
    v |= dataRead();
  }
  return v;
}

static inline uint16_t tleReadAvalRawFast() {
  csLow();

  sscWrite16(CMD_READ_AVAL_FAST);

  // DATA line release
  dataIn();

  uint16_t raw = sscRead16();

  csHigh();

  return raw;
}

// Treat as 15-bit signed value
static inline int16_t signExtend15(uint16_t raw) {
  uint16_t v15 = raw & 0x7FFF;
  if (v15 & 0x4000) {
    return (int16_t)(v15 | 0x8000);
  }
  return (int16_t)v15;
}

// For debugging (unused)
// static inline float rawToDeg(uint16_t raw) {
//   int16_t aval = signExtend15(raw);
//   return (float)aval * 360.0f / 32768.0f;
// }

void setup() {
  // GPIO init
  pinModeFast(CS, OUTPUT);
  pinModeFast(CLK, OUTPUT);
  pinModeFast(DATA_PIN, INPUT);

  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);  // EN = LOW -> receive mode

  // Serial init (RS485 TX/RX)
  Serial.begin(SERIAL_BAUD);

  // TLE5012B SSC init
  csHigh();
  clkLow();
}

void loop() {
  // ========================================
  // 1. RS485 receive check (highest priority, non-blocking)
  // ========================================
  if (receivePacket21(r_id, r_cmd, r_data)) {
    // 1-1. Command addressed to this device
    if (r_id == device_id) {
      uint8_t cmd_only = r_cmd & 0x03;

      if (cmd_only == 1) {
        // CMD=1: send cached data (reply with cmd=1)
        makePacket21(device_id, r_cmd, latest_angle_21bit);
        send485();
        do_ssc_read = true;  // Trigger SSC read immediately after send
      }
      // cmd=0, cmd=2, cmd=3: do nothing
    }
    // 1-2. Chain read (prev device -> this device -> next device)
    else {
      uint8_t prev_id = (device_id - 1) & 0x1F;
      // If this device is the last ID in chain, SSC trigger comes from ID=1 (wrap-around)
      // uint8_t fowd_id = (device_id >= CHAIN_MAX_ID) ? 1 : (device_id + 1);

      if (r_id == prev_id && (r_cmd & 0x03) == 2) {
        // Received CMD=2 from previous device -> reply immediately (cmd=2)
        makePacket21(device_id, r_cmd, latest_angle_21bit);
        send485();
        do_ssc_read = true;  // Trigger SSC read immediately after send
      }

      // if (r_id == fowd_id && (r_cmd & 0x03) == 2) {
      //   // Next device (or wrap-around ID=1) sent CMD=2 -> trigger SSC read
      //   // Run SSC between our own output and next turn to avoid collision with send loop
      //   do_ssc_read = true;
      // }
    }


  }

  // ========================================
  // 2. TLE5012B SSC read
  //    Condition a: SSC_INTERVAL_MS elapsed since last read
  //    Condition b: immediately after RS485 send (do_ssc_read flag)
  // ========================================
  if (do_ssc_read || (millis() - last_ssc_time >= SSC_INTERVAL_MS)) {
    uint16_t raw = tleReadAvalRawFast();
    // Scale 15-bit value (0-0x7FFF) to 21-bit (0-0x1FFFFF) via bit replication
    //   Place upper 15 bits at [20:6] with <<6, fill lower 6 bits with upper 6 bits (>>9)
    //   Full scale: 0x0000->0x000000, 0x7FFF->0x1FFFFF
    uint16_t raw15 = raw & 0x7FFF;
    latest_angle_21bit = ((uint32_t)raw15 << 6) | (raw15 >> 9);
    last_ssc_time = millis();
    do_ssc_read = false;
  }
}
