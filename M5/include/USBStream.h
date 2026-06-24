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

// include/USBStream.h
#pragma once

#include <USB.h>
#include <USBVendor.h>
#include <Arduino.h>

#include "Common.h"
#include "Meta.h"

#ifndef STREAM_TYPES_DEFINED
#define STREAM_TYPES_DEFINED

enum class Type {
    UINT32,
    UINT16,
    UINT8,
    INT32,
    INT16,
    FLOAT,
    BOOL,
};

struct FieldDef {
    const char* key;
    Type        type;
    size_t      count;
    size_t      offset;
};

using CommandCallback = bool (*)(const uint8_t* buf, size_t len);
#endif

class USBStream {
public:
    static const size_t MAX_FIELDS = 32;
    static const size_t MAX_BUF    = 512;

    USBStream();
    size_t getBinTotal() const { return _bin_total; }

    bool add(const char* key, Type type, size_t count = 1);

    void begin(uint16_t    vid          = USB_VID,
               uint16_t    pid          = USB_PID,
               const char* manufacturer = MANUFACTURER,
               const char* product      = PRODUCT);

    bool mounted();
    bool hasError() const { return _error; }

    void set(const char* key, uint32_t value);
    void set(const char* key, uint16_t value);
    void set(const char* key, uint8_t  value);
    void set(const char* key, int32_t  value);
    void set(const char* key, int16_t  value);
    void set(const char* key, float    value);
    void set(const char* key, bool     value);

    void set(const char* key, const uint32_t* array, size_t len);
    void set(const char* key, const uint16_t* array, size_t len);
    void set(const char* key, const int32_t*  array, size_t len);
    void set(const char* key, const int16_t*  array, size_t len);
    void set(const char* key, const float*    array, size_t len);
    void set(const char* key, const bool*     array, size_t len);

    bool   send();
    size_t recv(uint8_t* buf, size_t len);
    void   onCommand(CommandCallback cb);

    void sendPingResponse(const SensorSnapshot& snapshot,
                          const char* fw_version,
                          const char* hw_version,
                          const char* last_updated);

private:
    USBVendor       _vendor;
    bool            _error;
    CommandCallback _on_command;

    FieldDef _fields[MAX_FIELDS];
    size_t   _field_count;
    size_t   _bin_total;
    uint8_t  _bin_buf[MAX_BUF];

    FieldDef*   _find(const char* key);
    uint8_t     _checksum() const;
    static size_t      _typeSize(Type type);
    static const char* _typeName(Type type);
};
