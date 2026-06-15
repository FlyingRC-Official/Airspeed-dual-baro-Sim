# AirSpeed ESP32

ESP32/ESP32-S3 firmware that emulates an MS4525 differential pressure airspeed sensor as an I2C slave for ArduPilot bring-up.

Version 1 does not read real barometer sensors. It returns fake, serial-adjustable MS4525 frames so a flight controller running ArduPilot can detect the device and read plausible airspeed data.

## Hardware Defaults

- Primary target: ESP32-S3 DevKitC, PlatformIO env `esp32-s3-devkitc-1`
- Fallback target: classic ESP32 DevKit, PlatformIO env `esp32dev`
- I2C slave address: `0x28`
- ESP32-S3 I2C pins: SDA `GPIO8`, SCL `GPIO9`
- Classic ESP32 I2C pins: SDA `GPIO21`, SCL `GPIO22`
- Serial monitor: `115200`

Connect SDA, SCL, and GND to the flight controller I2C bus. Use 3.3 V-compatible pullups.

## Build

```sh
pio run -e esp32-s3-devkitc-1
```

Fallback classic ESP32 build:

```sh
pio run -e esp32dev
```

## Upload And Monitor

```sh
pio run -e esp32-s3-devkitc-1 -t upload
pio device monitor -b 115200
```

If the monitor prints `waiting for download`, the ESP32-S3 is in ROM download mode. Press reset on the board or run:

```sh
python3 -m esptool --chip esp32s3 --port /dev/cu.usbmodem101 run
```

## Serial Commands

- `p <pa>`: set fake differential pressure in Pascals
- `t <c>`: set fake temperature in Celsius
- `z`: set pressure to zero
- `r on`: enable a slow fake pressure ramp
- `r off`: disable the pressure ramp
- `s`: print current state and raw MS4525 frame bytes
- `h`: print help

Default boot state is `0 Pa`, `25 C`, ramp off.

## ArduPilot Setup

Set `ARSPD_TYPE=1` for `I2C-MS4525D0`. Leave bus autodetection enabled first; set `ARSPD_BUS` only if autodetect fails. Reboot the flight controller and check the GCS messages for an MS4525 found message. Then use `p <pa>` in the ESP32 serial monitor and confirm ArduPilot telemetry changes.
