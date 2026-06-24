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
#include <math.h>
#include "Common.h"

class AngleProcessor {
private:
    bool  _invert;
    float _mech_min;   // Mechanical lower limit (deg)
    float _mech_max;   // Mechanical upper limit (deg)
    float _last_raw;
    float _unwrapped;
    bool  _inited;
    static constexpr float MAX_TURN_ANGLE = 360.0f;

public:
    // Constructor
    // invert   : Invert sensor direction
    // mech_min : Mechanical lower limit (deg) - used to correct +/-360 init jump
    // mech_max : Mechanical upper limit (deg)
    AngleProcessor(bool invert = false, float mech_min = -180.0f, float mech_max = 180.0f)
        : _invert(invert)
        , _mech_min(mech_min)
        , _mech_max(mech_max)
        , _last_raw(0.0f)
        , _unwrapped(0.0f)
        , _inited(false)
    {}

    // Main processing
    // raw_enc_deg  : Raw angle from sensor (deg, 0-360)
    // jig_angle_offset : Jig-specific angle offset (deg)
    // valid              : Whether sensor data is valid (if false, return previous value)
    float process(float raw_enc_deg, float jig_angle_offset, bool valid = true) {

        // --- 1. Invert ---
        if (_invert) {
            raw_enc_deg = MAX_TURN_ANGLE - fmod(raw_enc_deg, MAX_TURN_ANGLE);
        }

        // --- 2. Unwrap ---
        if (!_inited) {

            float delta = raw_enc_deg - jig_angle_offset;
            if (delta > MAX_TURN_ANGLE / 2.0f)  delta -= MAX_TURN_ANGLE;
            if (delta <= -MAX_TURN_ANGLE / 2.0f) delta += MAX_TURN_ANGLE;

            // _unwrapped = jig_angle_offset + delta;
            _unwrapped = delta;
            _last_raw = raw_enc_deg;
            _inited    = true;
        } else if(valid) {
            float diff = raw_enc_deg - _last_raw;
            if (diff > MAX_TURN_ANGLE / 2.0f)  diff -= MAX_TURN_ANGLE;
            if (diff < -MAX_TURN_ANGLE / 2.0f) diff += MAX_TURN_ANGLE;
            _unwrapped += diff;
            _last_raw   = raw_enc_deg;
        }

        // --- 3. Output ---
        return _unwrapped;
    }

    // Reset processor state (call after zero reset)
    void reset() {
        _last_raw  = 0.0f;
        _unwrapped = 0.0f;
        _inited    = false;
    }

    // Returns current cumulative angle (for zero reset saving)
    float getCumulative() const { return _unwrapped; }
};
