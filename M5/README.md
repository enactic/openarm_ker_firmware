# OpenArm KER M5

OpenArm KER (Kinematically Equivalent Replica) M5 is a 16-channel joint angle DAQ system for the OpenArm robot.
It reads magnetic encoders via RS-485 and streams joint angles to a PC over USB or serial.

---

## Hardware

- M5Stack CoreS3-SE
- RS-485 x16ch TLE5012B encoder (GPIO43/44, EN: GPIO5)
- I2C scroll encoder (GPIO1/2) — optional

### Optional I2C peripherals

For workflows that require direct control on the hand/gripper section of the leader arm (e.g. complex menu navigation without a foot pedal), additional M5Stack units can be attached via the Grove port.

The following units are verified to work:

| Unit | Use case |
|---|---|
| [Unit-JoyStick2](https://docs.m5stack.com/en/unit/Unit-JoyStick2) | Multi-axis input, menu navigation |
| [UNIT-Scroll](https://docs.m5stack.com/en/unit/UNIT-Scroll) | Rotary encoder + button — supported out of the box |

> [!WARNING]
> **Signal noise & cable length**
> I2C over long Grove cables is susceptible to signal noise and voltage drops, which can cause communication instabilities or packet loss during data collection. Keep cables as short as practical.

To add a unit not yet supported:

1. Add its library to `lib_deps` in `platformio.ini`
2. Initialize the device in `sensorTask()` before the loop
3. Read the device in the loop and store to shared variables

---

## Directory Structure

```
.
├── include/
│   ├── Common.h          # Pin/sensor config, system state — edit here to add devices
│   ├── AngleProcessor.h  # Unwrapping & offset logic
│   ├── GUIHandler.h      # Touchscreen GUI
│   ├── RSNexus.h         # RS-485 packet handling
│   ├── USBStream.h       # USB-OTG vendor stream
│   └── SerialStream.h    # Serial stream (drop-in replacement for USBStream)
├── src/
│   ├── GUIHandler.cpp
│   ├── RSNexus.cpp
│   ├── USBStream.cpp
│   └── SerialStream.cpp
│   └── main.cpp
└── platformio.ini
```

---

## GUI

The touchscreen GUI provides sensor monitoring and control.

| Area | Action |
|---|---|
| Bar chart | Tap a bar to select that sensor (turns orange) |
| **Zero Reset (All)** | Tap → confirmation dialog → YES to zero all sensors |
| **Confirm Zero** | Shown when sensors are selected; tap to zero selected sensors only |
| **START / STOP** | Begin or end streaming |
| **JD: ON / OFF** (top-right) | Toggle jump detection on/off |

**Status bar** (below the title area):

| State | Display |
|---|---|
| Normal standby | `STANDBY` (white) |
| Streaming | `STREAMING` (white) |
| Stopped by jump | `STREAMING STOPPED: Jump Detected` (red) — persists until START is pressed |

### Jump Detection

During streaming, if any joint angle deviates more than `JUMP_THRESHOLD_DEG` (30°) from its last known good value for `JUMP_CONFIRM_FRAMES` consecutive frames, the system:

1. Stops streaming immediately and returns to standby
2. Shows a 3-second overlay: **"Jump Detected / Recalibrate zero position"**
3. Keeps the warning in the status bar until the user presses START again

This typically indicates that a zero calibration was recorded at the wrong robot position.
Use **JD: ON / OFF** in the top-right corner to disable detection temporarily if needed.

---



## Stream Schema

Data is streamed in binary format. The schema is sent automatically on connection — the PC side parses field names and types dynamically, so no code changes are needed when fields are added.

| Field | Type | Count |
|---|---|---|
| `timestamp` | `uint32` | 1 |
| `angles` | `float` | NUM_SENSORS |
| `errors` | `bool` | NUM_SENSORS |

### Adding a new field

**1. Register the field in `setup()`:**

```cpp
stream.add("field_name", Type::FLOAT);      // single value
stream.add("field_name", Type::FLOAT, N);   // array of N values
```

Available types: `FLOAT`, `UINT32`, `INT16`, `UINT8`, `BOOL`

**2. Set the value in `loop()` before `stream.send()`:**

```cpp
float accel[3] = { ax, ay, az };
stream.set("imu_accel", accel, 3);
stream.send();
```

---

## Setup

### 1. udev rules (run once)

**USB vendor mode (normal operation):**

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0666"' | sudo tee /etc/udev/rules.d/99-m5stack.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Serial/flash mode — put M5Stack into boot mode first (hold RST 3 seconds):**

```bash
SERIAL=$(udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT | cut -d= -f2)
sudo tee -a /etc/udev/rules.d/99-m5stack.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", ATTRS{serial}=="$SERIAL", MODE="0666", SYMLINK+="m5_ker_485"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 2. Verify connection

After rebooting normally, the device should appear as a USB vendor device:

```bash
lsusb | grep 303a
# 303a:4002 should appear
```

### 3. Install PlatformIO

```bash
uv venv --seed
uv pip install platformio
```

### 4. Flash firmware

Put M5Stack into boot mode (hold RST 3 seconds until green LED), then:

```bash
uv run pio run -e usb --target upload     # USB-OTG mode (default)
uv run pio run -e serial --target upload  # Serial mode
```

Press RST once to reboot.

---


A `[env:serial]` environment is also available as an alternative if needed:

```bash
pio run -e serial -t upload
```

### 5. PC receiver

https://github.com/enactic/openarm_ker

