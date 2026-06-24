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

#include "GUIHandler.h"

GUIHandler::GUIHandler() {
    for (int i = 0; i < NUM_SENSORS; i++) _sensor_selected[i] = false;
}

void GUIHandler::init() {
    _canvas = new M5Canvas(&M5.Display);
    _canvas->createSprite(M5.Display.width(), M5.Display.height());
    _canvas->setTextDatum(bottom_center);
    _canvas->setFont(&fonts::Font2);
}

GUICommand GUIHandler::tick(const SensorSnapshot& snapshot, AppMode current_mode) {
    _canvas->fillSprite(BLACK);

    bool any_selected = false;
    for (int i = 0; i < NUM_SENSORS; i++) if (_sensor_selected[i]) any_selected = true;

    drawBars(snapshot);
    drawButtons(any_selected, current_mode);
    if (_confirm_pending) drawConfirmDialog();

    _canvas->pushSprite(0, 0);
    return handleTouch();
}
void GUIHandler::drawBars(const SensorSnapshot& snapshot) {
    _canvas->setTextColor(TFT_WHITE);

    const int y_off        = 40;
    const int area_h       = 160;
    int bar_width          = _canvas->width() / (NUM_SENSORS + 1);
    int spacing            = (_canvas->width() - (bar_width * NUM_SENSORS)) / (NUM_SENSORS + 1);
    int max_bar_height     = area_h - 20;

    uint16_t SKY_BLUE = _canvas->color565(135, 206, 235);

    for (int i = 0; i < NUM_SENSORS; i++) {
        float angle = snapshot.sensors[i].angle;
        int   bar_height;

        if (isnan(angle) || snapshot.sensors[i].error) {
            bar_height = max_bar_height;
            angle      = NAN;
        } else {
            if (angle < -180.0f) angle = -180.0f;
            if (angle >  180.0f) angle =  180.0f;
            float normalized = (angle - (-180.0f)) / (360.0f);
            bar_height = (int)(normalized * (float)max_bar_height);
        }

        int x_pos = spacing + i * (bar_width + spacing);

        if (isnan(angle)) {
            _canvas->fillRect(x_pos, y_off + area_h - max_bar_height - 15,
                              bar_width, max_bar_height, RED);
            _canvas->setTextColor(TFT_WHITE);
            _canvas->drawCentreString("ERR", x_pos + bar_width / 2,
                                      y_off + area_h - max_bar_height / 2 - 8);
        } else {
            uint32_t color = _sensor_selected[i] ? TFT_ORANGE : SKY_BLUE;
            _canvas->fillRect(x_pos, y_off + area_h - bar_height - 15,
                              bar_width, bar_height, color);
        }
        _canvas->setTextColor(TFT_WHITE);
        _canvas->drawNumber(i + 1, x_pos + bar_width / 2, y_off + area_h - 5);
    }
}

void GUIHandler::drawButtons(bool any_selected, AppMode current_mode) {
    const int btn_h  = 40;
    const int btn_y  = _canvas->height() - btn_h;
    const int btn_w  = 140;
    const int btn1_x = 20;
    const int btn2_x = 320 - btn_w - 20;


    // Zero Reset Button
    _canvas->fillRoundRect(btn1_x, btn_y, btn_w, btn_h, 8, TFT_DARKGREY);
    _canvas->setTextColor(TFT_WHITE);
    _canvas->setTextDatum(middle_center);
    _canvas->setFont(&fonts::Font2);
    if (any_selected)
        _canvas->drawCentreString("Confirm Zero", btn1_x + btn_w / 2, btn_y + btn_h / 2 - 5);
    else
        _canvas->drawCentreString("Zero Reset (All)", btn1_x + btn_w / 2, btn_y + btn_h / 2 - 5);

    // Start / Stop Button
    if (current_mode == AppMode::STANDBY) {
        _canvas->fillRoundRect(btn2_x, btn_y, btn_w, btn_h, 8, TFT_GREEN);
        _canvas->setTextColor(TFT_WHITE);
        _canvas->setFont(&fonts::Font2);
        _canvas->drawCentreString("START", btn2_x + btn_w / 2, btn_y + btn_h / 2 - 5);
    } else {
        _canvas->fillRoundRect(btn2_x, btn_y, btn_w, btn_h, 8, TFT_RED);
        _canvas->setTextColor(TFT_WHITE);
        _canvas->setFont(&fonts::Font2);
        _canvas->drawCentreString("STOP", btn2_x + btn_w / 2, btn_y + btn_h / 2 - 5);
    }

    // Status Bar
    _canvas->setFont(&fonts::Font2);
    _canvas->setTextDatum(middle_center);
    if (_jump_stopped) {
        uint16_t dark_red = _canvas->color565(80, 0, 0);
        _canvas->fillRect(0, 20, 320, 18, dark_red);
        _canvas->setTextColor(TFT_RED, dark_red);
        _canvas->drawString("STREAMING STOPPED: Jump Detected", 160, 29);
    } else {
        _canvas->fillRect(0, 20, 320, 18, BLACK);
        _canvas->setTextColor(TFT_WHITE, BLACK);
        if (current_mode == AppMode::STANDBY)
            _canvas->drawString("STANDBY", 160, 29);
        else
            _canvas->drawString("STREAMING", 160, 29);
    }

    // Jump Detect toggle (top-right, y=4..20)
    uint16_t jd_color = _jump_detect_on ? _canvas->color565(0, 120, 0) : TFT_DARKGREY;
    _canvas->fillRoundRect(262, 4, 56, 16, 4, jd_color);
    _canvas->setTextColor(TFT_WHITE, jd_color);
    _canvas->setFont(&fonts::Font2);
    _canvas->drawCentreString(_jump_detect_on ? "JD: ON" : "JD: OFF", 290, 5);
}

void GUIHandler::drawConfirmDialog() {
    const int dlg_x = 40,  dlg_y = 75,  dlg_w = 240, dlg_h = 100;
    const int yes_x = 55,  btn_y = 130, btn_w = 90,  btn_h = 32;
    const int can_x = 175;

    _canvas->fillRoundRect(dlg_x, dlg_y, dlg_w, dlg_h, 10, TFT_DARKGREY);
    _canvas->drawRoundRect(dlg_x, dlg_y, dlg_w, dlg_h, 10, TFT_WHITE);

    _canvas->setFont(&fonts::Font2);
    _canvas->setTextDatum(middle_center);
    _canvas->setTextColor(TFT_WHITE, TFT_DARKGREY);
    _canvas->drawCentreString("Are you sure?", 160, dlg_y + 27);

    _canvas->fillRoundRect(yes_x, btn_y, btn_w, btn_h, 6, TFT_GREEN);
    _canvas->setTextColor(TFT_WHITE, TFT_GREEN);
    _canvas->drawCentreString("YES", yes_x + btn_w / 2, btn_y + btn_h / 2 - 4);

    _canvas->fillRoundRect(can_x, btn_y, btn_w, btn_h, 6, TFT_RED);
    _canvas->setTextColor(TFT_WHITE, TFT_RED);
    _canvas->drawCentreString("CANCEL", can_x + btn_w / 2, btn_y + btn_h / 2 - 4);
}

GUICommand GUIHandler::handleTouch() {
    GUICommand cmd;  // type = NONE

    m5::touch_detail_t tp = M5.Touch.getDetail(0);
    if (!tp.wasReleased()) return cmd;

    const int btn_h   = 40;
    const int btn_y   = _canvas->height() - btn_h;
    const int btn_w   = 140;
    const int btn1_x  = 20;
    const int btn2_x  = 320 - btn_w - 20;
    const int bar_y   = 40;
    const int bar_h   = 160;
    const int bar_w   = _canvas->width() / (NUM_SENSORS + 1);
    const int spacing = (_canvas->width() - (bar_w * NUM_SENSORS)) / (NUM_SENSORS + 1);

    bool zero_btn  = (tp.x >= btn1_x && tp.x <= btn1_x + btn_w && tp.y >= btn_y);
    bool start_btn = (tp.x >= btn2_x && tp.x <= btn2_x + btn_w && tp.y >= btn_y);
    bool bar_area  = (tp.y >= bar_y && tp.y < bar_y + bar_h);
    bool jd_btn    = (tp.x >= 262 && tp.x <= 318 && tp.y >= 4  && tp.y <= 20);

    if (_confirm_pending) {
        const int yes_x = 55,  btn_y = 130, btn_w = 90, btn_h = 32;
        bool yes_hit = (tp.x >= yes_x && tp.x <= yes_x + btn_w && tp.y >= btn_y && tp.y <= btn_y + btn_h);
        if (yes_hit) {
            if (_pending_mask != 0) {
                cmd.type = GUICommand::Type::ZERO_MASK;
                cmd.mask = _pending_mask;
            } else {
                cmd.type = GUICommand::Type::ZERO_ALL;
            }
        }
        _pending_mask    = 0;
        _confirm_pending = false;
        return cmd;
    }

    if (zero_btn) {
        uint16_t mask = 0;
        for (int i = 0; i < NUM_SENSORS; i++) {
            if (_sensor_selected[i]) mask |= (1 << i);
            _sensor_selected[i] = false;
        }
        _pending_mask    = mask;  // 0 = ZERO_ALL, non-zero = ZERO_MASK
        _confirm_pending = true;
    } else if (start_btn) {
        cmd.type = GUICommand::Type::START;
    } else if (bar_area) {
        bool hit = false;
        for (int i = 0; i < NUM_SENSORS; i++) {
            int x_pos = spacing + i * (bar_w + spacing);
            if (tp.x >= x_pos && tp.x < x_pos + bar_w) {
                _sensor_selected[i] = !_sensor_selected[i];
                hit = true;
                break;
            }
        }
        if (!hit) {
            for (int i = 0; i < NUM_SENSORS; i++) _sensor_selected[i] = false;
        }
    } else if (jd_btn) {
        cmd.type = GUICommand::Type::TOGGLE_JUMP_DETECT;
    } else {
        for (int i = 0; i < NUM_SENSORS; i++) _sensor_selected[i] = false;
    }

    return cmd;
}
