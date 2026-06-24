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

#pragma once

#include <Arduino.h>

#define JOINTNUM 16

class RSNexus {
public:
    RSNexus(HardwareSerial& serial, int en_pin);

    void begin(unsigned long baud, int rx_pin, int tx_pin);

    void requestPacket(uint8_t id, uint8_t cmd = 2, uint32_t data = 0);

    bool readPacket();


    float getAngleDeg(uint8_t id) const;
    float getAngleRad(uint8_t id) const;
    uint32_t getRaw(uint8_t id) const;
    bool isValid(uint8_t id) const;
    uint32_t getLastSeen(uint8_t id) const;

    void getAllAngles(float* buffer);

private:
    HardwareSerial& _serial;
    const int _en_pin;

    uint32_t _raw_angle[JOINTNUM];
    float    _angle_deg[JOINTNUM];
    float    _angle_rad[JOINTNUM];
    bool     _valid[JOINTNUM];
    uint32_t _last_seen[JOINTNUM];

    enum State { WAIT_HEADER, GET_D0, GET_D1, GET_D2 };
    State _state = WAIT_HEADER;
    uint8_t _packet[4];

    void _setTxMode(bool tx);
};
