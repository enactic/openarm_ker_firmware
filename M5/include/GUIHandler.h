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

#include <M5Unified.h>
#include "Common.h"

#define STOP_BTN_X  90
#define STOP_BTN_Y  200
#define STOP_BTN_W  140
#define STOP_BTN_H  40

struct GUICommand {
    enum class Type {
        NONE,
        START,
        STOP,
        ZERO_ALL,
        ZERO_MASK,
        TOGGLE_JUMP_DETECT,
    };
    Type     type = Type::NONE;
    uint16_t mask = 0;
};

class GUIHandler {
public:
    GUIHandler();
    void init();

    void setJumpStopped(bool val)       { _jump_stopped   = val; }
    void setJumpDetectEnabled(bool val) { _jump_detect_on = val; }

    // Returns command from user interaction
    GUICommand tick(const SensorSnapshot& snapshot, AppMode current_mode);

private:
    M5Canvas* _canvas;
    bool      _sensor_selected[NUM_SENSORS];
    bool     _confirm_pending = false;
    uint16_t _pending_mask   = 0;
    bool     _jump_stopped   = false;
    bool     _jump_detect_on = true;

    void       drawBars(const SensorSnapshot& snapshot);
    void       drawButtons(bool any_selected, AppMode current_mode);
    void       drawConfirmDialog();
    GUICommand handleTouch();
};
