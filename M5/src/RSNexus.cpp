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

#include "RSNexus.h"
#include <driver/gpio.h>

RSNexus::RSNexus(HardwareSerial& serial, int en_pin) : _serial(serial), _en_pin(en_pin) {
    for(int i=0; i<JOINTNUM; i++) {
        _raw_angle[i] = 0;
        _angle_deg[i] = 0.0f;
        _angle_rad[i] = 0.0f;
        _valid[i] = false;
        _last_seen[i] = 0;
    }
}

void RSNexus::begin(unsigned long baud, int rx, int tx) {
    _serial.begin(baud, SERIAL_8N1, rx, tx);
    _serial.setTimeout(0);

    pinMode(_en_pin, OUTPUT_OPEN_DRAIN);
    gpio_set_pull_mode((gpio_num_t)_en_pin, GPIO_PULLUP_ONLY);
    _setTxMode(false);
}

void RSNexus::_setTxMode(bool tx) {
    digitalWrite(_en_pin, tx ? HIGH : LOW);
    if (tx) delayMicroseconds(2);
}

void RSNexus::requestPacket(uint8_t id, uint8_t cmd, uint32_t data) {
    uint8_t header = 0x80 | ((id & 0x1F) << 2) | (cmd & 0x03);
    uint8_t d0 = data & 0x7F;
    uint8_t d1 = (data >> 7) & 0x7F;
    uint8_t d2 = (data >> 14) & 0x7F;

    _setTxMode(true);
    _serial.write(header);
    _serial.write(d0);
    _serial.write(d1);
    _serial.write(d2);
    _serial.flush();
    delayMicroseconds(2);
    _setTxMode(false);
}

bool RSNexus::readPacket() {
    bool new_data = false;
    while (_serial.available()) {
        uint8_t b = _serial.read();
        switch (_state) {
            case WAIT_HEADER:
                if (b & 0x80) { _packet[0] = b; _state = GET_D0; }
                break;
            case GET_D0:
                if (!(b & 0x80)) { _packet[1] = b; _state = GET_D1; }
                else _state = WAIT_HEADER;
                break;
            case GET_D1:
                if (!(b & 0x80)) { _packet[2] = b; _state = GET_D2; }
                else _state = WAIT_HEADER;
                break;
            case GET_D2:
                if (!(b & 0x80)) {
                    uint8_t received_id = (_packet[0] & 0x7C) >> 2;
                    uint32_t raw = ((uint32_t)_packet[1]) |
                                   ((uint32_t)_packet[2] << 7) |
                                   ((uint32_t)b << 14);

                    if (received_id >= 1 && received_id <= JOINTNUM) {
                        uint8_t idx = received_id - 1;

                        _raw_angle[idx] = raw;
                        _angle_deg[idx] = (raw / (float)(1 << 21)) * 360.0f;
                        _angle_rad[idx] = _angle_deg[idx] * (PI / 180.0f);
                        _valid[idx]     = true;
                        _last_seen[idx] = millis();

                        new_data = true;
                    }
                    _state = WAIT_HEADER;
                } else _state = WAIT_HEADER;
                break;
        }
    }
    return new_data;
}


float RSNexus::getAngleDeg(uint8_t id) const {
    if (id < 1 || id > JOINTNUM) return 0.0f;
    return _angle_deg[id - 1];
}

float RSNexus::getAngleRad(uint8_t id) const {
    if (id < 1 || id > JOINTNUM) return 0.0f;
    return _angle_rad[id - 1];
}

uint32_t RSNexus::getRaw(uint8_t id) const {
    if (id < 1 || id > JOINTNUM) return 0;
    return _raw_angle[id - 1];
}

bool RSNexus::isValid(uint8_t id) const {
    if (id < 1 || id > JOINTNUM) return false;
    return _valid[id - 1];
}

uint32_t RSNexus::getLastSeen(uint8_t id) const {
    if (id < 1 || id > JOINTNUM) return 0;
    return _last_seen[id - 1];
}

void RSNexus::getAllAngles(float* buffer) {
    for (int i = 0; i < JOINTNUM; i++) {
        buffer[i] = _angle_deg[i];
    }
}
