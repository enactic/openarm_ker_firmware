# OpenArm KER Encoder

ATtiny1616 firmware. Reads a TLE5012B magnetic angle sensor over SSC and streams it over RS485 (chainable, multi-device).

## Hardware

- **MCU**: ATtiny1616 (20MHz, 2KB RAM / 16KB Flash)
- **Sensor**: TLE5012B (15-bit angle, SSC)
- **Bus**: RS485 @ 2Mbps, 4-byte packets, up to 21-bit angle output
- **Device ID**: 0–31 (set at build time via `DEVICE_ID`)

## Writer

A USB-serial UPDI programmer (Adafruit UPDI Friend, serialUPDI). It appears as e.g. `/dev/ttyUSB0` (CH340E).

- **Standard** — [Switch Science #9535](https://www.switch-science.com/products/9535) / [Adafruit 5879](https://www.adafruit.com/product/5879)
- **High-Voltage** — [DigiKey 5893](https://www.digikey.jp/ja/products/detail/adafruit-industries-llc/5893/22596412) / [Adafruit 5893](https://www.adafruit.com/product/5893). Adds a 12V pulse for chips whose UPDI pin is fused as reset/GPIO, and for unbricking.

The standard version is enough for normal use; the HV version is handy if you ever need to recover a bricked chip.

## Flashing

Set up PlatformIO in a uv venv (first time only):

```bash
uv venv
source .venv/bin/activate
uv pip install platformio
```

Build and upload. The Device ID is set with the build flag — just change the number:

```bash
PLATFORMIO_BUILD_FLAGS="-DDEVICE_ID=3" platformio run -t upload --upload-port /dev/ttyUSB0
```

The example above flashes **ID=3**. Change `DEVICE_ID` to flash a different ID

## Hardware design

They are distributed from [enactic/openarm_hardware](https://github.com/enactic/openarm_hardware).
