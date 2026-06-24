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
#include <atomic>

// USE_USB is defined via build flags in platformio.ini - do not hardcode here

// =====================================================
// RS-485
// =====================================================
#define RS485_EN_PIN 5
#define RS485_BAUD   2000000UL
#define RS485_TX     43
#define RS485_RX     44
#define BULK 0
// =====================================================
// I2C
// =====================================================
#define I2C_SDA_PIN 2
#define I2C_SCL_PIN 1

// =====================================================
// Packet header
// =====================================================
#define PACKET_HEADER_0  0xA5
#define PACKET_HEADER_1  0x5A

// =====================================================
// Constants
// =====================================================
#define NUM_SENSORS 16
#define MARGIN_DEG          25.0f   // Margin for mechanical limit correction
#define JUMP_THRESHOLD_DEG   30.0f  // Angle deviation to consider a jump
#define JUMP_CONFIRM_FRAMES  20      // Consecutive frames before confirming as zero error

// =====================================================
// Sensor data
// =====================================================
struct SensorData {
    float angle;
    float raw_deg;
    bool  error;
};

struct SensorSnapshot {
    SensorData sensors[NUM_SENSORS];
    uint32_t   timestamp;
    int16_t    encoder_value;
    uint8_t    encoder_button;
    uint16_t   joy_x;
    uint16_t   joy_y;
    uint8_t    joy_button;
};

// =====================================================
// Encoder config (one entry per channel)
// All sensor-specific settings in one place.
// To add/modify a channel: edit only this table.
//
// invert      : Reverse sensor direction
// mech_min    : Mechanical lower limit (deg)
// mech_max    : Mechanical upper limit (deg)
// joint_offset: Mechanical offset to robot coordinate (deg)
// =====================================================
struct EncoderConfig {
    bool  invert;
    float mech_min;
    float mech_max;
    float mech_joint_offset;
    bool  skip_jump_detect;
};

const EncoderConfig ENCODER_CONFIG[NUM_SENSORS] = {

    // invert   mech_min   mech_max   joint_offset  skip_jd   ch    joint
    {false,    -80.00f,   160.00f,    0.0f,         false},  // ch1   right joint1
    {true,     -10.00f,   160.00f,    0.0f,         false},  // ch2   right joint2
    {false,    -90.00f,    90.00f,     -90.0f,      false},  // ch3   right joint3
    {true,       0.00f,   140.00f,   134.95f,       false},  // ch4   right joint4
    {false,    -90.00f,    90.00f,     0.0f,        false},  // ch5   right joint5
    {true,     -45.00f,    45.00f,    44.95f,       false},  // ch6   right joint6
    {true,     -90.00f,    90.00f,     0.00f,       false},  // ch7   right joint7
    {false,    -90.00f,     5.73f,     0.00f,       true},   // ch8   right gripper

    {false,   -160.00f,    80.00f,     -20.92f,     false},  // ch9   left  joint1
    {true,    -160.00f,    10.00f,     0.00f,       false},  // ch10  left  joint2
    {false,    -90.00f,    90.00f,    73.41f,       false},  // ch11  left  joint3
    {false,      0.00f,   140.00f,    126.75f,      false},  // ch12  left  joint4
    {false,    -90.00f,    90.00f,     -26.47f,     false},  // ch13  left  joint5
    {true,      -45.00f,    45.00f,    -39.83f,     false},  // ch14  left  joint6
    {true,     -90.00f,    90.00f,      0.00f,      false},  // ch15  left  joint7
    {false,     -5.73f,    60.00f,      0.00f,      true},   // ch16  left  gripper

    // debug right using mech stop
    // {false,    -80.00f,   160.00f,    -80.0f},        // ch1   right joint1
    // {true,     -10.00f,   160.00f,    -10.0f},        // ch2   right joint2
    // {false,    -90.00f,    90.00f,     -90.0f},        // ch3   right joint3
    // {true,       0.00f,   140.00f,    0.0f},        // ch4   right joint4
    // {false,    -90.00f,    90.00f,     -90.0f},        // ch5   right joint5
    // {true,     -45.00f,    45.00f,    -45.0f},        // ch6   right joint6
    // {true,     -90.00f,    90.00f,     -90.00f},        // ch7   right joint7
    // {false,    -90.00f,     5.73f,      0.00f},        // ch8   right gripper

    // debug left using mech stop
    // {false,   -160.00f,    80.00f,     80.00f},        // ch9   left  joint1
    // {true,    -160.00f,    10.00f,     10.00f},        // ch10  left  joint2
    // {false,    -90.00f,    90.00f,    -90.00f},        // ch11  left  joint3
    // {false,      0.00f,   140.00f,     0.00f},        // ch12  left  joint4
    // {false,    -90.00f,    90.00f,     -90.00f},        // ch13  left  joint5
    // {true,      -45.00f,     45.00f,     -45.00f},       // subtracted 90 from original offset to correct +/-360 init jump
    // {true,     -90.00f,    90.00f,      -90.00f},        // ch15  left  joint7
    // {false,     -5.73f,    60.00f,      0.00f},        // ch16  left  gripper


};

// const EncoderConfig ENCODER_CONFIG[NUM_SENSORS] = {
//     // invert   mech_min   mech_max   joint_offset        ch    joint
//     {false,    -80.00f,   200.00f,    -54.99f},        // ch1   right joint1
//     {true,     -10.00f,   190.00f,    -19.99f},        // ch2   right joint2
//     {false,    -90.00f,    90.00f,     68.13f},        // ch3   right joint3
//     {true,       0.00f,   140.00f,   -121.60f},        // ch4   right joint4
//     {false,    -90.00f,    90.00f,    -64.64f},        // ch5   right joint5
//     {true,     -45.00f,    45.00f,    -22.76f},        // ch6   right joint6
//     {true,     -90.00f,    90.00f,    -89.99f},        // ch7   right joint7
//     {false,    -90.00f,     5.73f,      0.00f},        // ch8   right gripper
//     {false,   -200.00f,    80.00f,     54.99f},        // ch9   left  joint1
//     {true,    -190.00f,    10.00f,     19.99f},        // ch10  left  joint2
//     {false,    -90.00f,    90.00f,    -68.13f},        // ch11  left  joint3
//     {false,      0.00f,   140.00f,   -121.60f},        // ch12  left  joint4
//     {false,    -90.00f,    90.00f,     64.64f},        // ch13  left  joint5
//     {true,     -45.00f,    45.00f,     22.76f},        // ch14  left  joint6
//     {true,     -90.00f,    90.00f,     89.99f},        // ch15  left  joint7
//     {false,     -5.73f,    90.00f,      0.00f},        // ch16  left  gripper
// };

// =====================================================
// Application mode
// =====================================================
enum class AppMode {
    STANDBY,  // GUI active, no streaming
    STREAM,   // Streaming active, minimal GUI
};

// =====================================================
// System state (managed in main.cpp)
// =====================================================
struct SystemState {
    std::atomic<AppMode>  mode           { AppMode::STREAM };
    std::atomic<bool>     zero_all       { false };
    std::atomic<uint16_t> zero_mask      { 0 };
    std::atomic<bool>     ping_requested { false };
    std::atomic<bool>     jump_detected       { false };
    std::atomic<bool>     jump_detect_enabled { true };
    std::atomic<bool>     reset_jump_state{false};
    // Only for Core0. We don't need "atomic" here.
    float jig_angle_offsets[NUM_SENSORS] = {};
};
