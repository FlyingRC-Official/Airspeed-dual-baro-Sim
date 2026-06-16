# AirSpeed ESP32

ESP32/ESP32-S3 firmware that emulates an MS4525 differential pressure airspeed sensor as an I2C slave for ArduPilot bring-up.

Version 2 can read two SPA06-003 barometers over a second I2C bus and expose their differential pressure as an MS4525-compatible airspeed sensor. If the barometers are not available, it falls back to the fake 0-100-0 km/h ramp used by V1.

## Hardware Defaults

- Primary target: ESP32-S3 DevKitC, PlatformIO env `esp32-s3-devkitc-1`
- Fallback target: classic ESP32 DevKit, PlatformIO env `esp32dev`
- Experimental STM32 target: STM32G031G8U6, PlatformIO env `stm32g031g8u6`
- I2C slave address: `0x28`
- ESP32-S3 flight-controller I2C slave pins: SDA `GPIO8`, SCL `GPIO9`
- ESP32-S3 barometer I2C master pins: SDA `GPIO4`, SCL `GPIO5`
- ESP32-S3 WS2812 debug LED pin: `GPIO48`
- Classic ESP32 flight-controller I2C slave pins: SDA `GPIO21`, SCL `GPIO22`
- Classic ESP32 barometer I2C master pins: SDA `GPIO4`, SCL `GPIO5`
- STM32G031G8U6 flight-controller I2C slave pins: I2C1 SCL `PB6`, SDA `PB7`
- STM32G031G8U6 barometer I2C master pins: I2C2 SCL `PA11`, SDA `PA12`
- STM32G031G8U6 WS2812 debug LED pin: `PA8` by default
- Barometer 1 address: `0x76`
- Barometer 2 address: `0x77`
- Serial monitor: `115200`

Connect SDA, SCL, and GND to the flight controller I2C bus. Use 3.3 V-compatible pullups.

Connect both SPA06-003 barometers to the barometer I2C bus. Put one sensor at `0x76` and the other at `0x77`.

The WS2812 debug LED shows firmware state:

- Purple blink: waiting for flight-controller I2C reads
- Green heartbeat: using live dual-barometer differential pressure
- Red/orange heartbeat: barometer input enabled but not fully healthy
- Amber heartbeat: fallback fake ramp is active
- Blue heartbeat: manual/static pressure mode

## Build

```sh
pio run -e esp32-s3-devkitc-1
```

Fallback classic ESP32 build:

```sh
pio run -e esp32dev
```

Experimental STM32G031G8U6 build:

```sh
pio run -e stm32g031g8u6
```

The STM32G031G8U6 port is a lean STM32Cube/HAL firmware. It keeps the MS4525 I2C slave, dual SPA06 barometer input, startup auto-zero, and WS2812 state LED, but does not include the ESP32 serial command interface.

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
- `r on`: enable the fake airspeed ramp
- `r off`: disable the pressure ramp
- `b on`: enable real dual-barometer input
- `b off`: disable real dual-barometer input
- `c`: zero the current barometer differential pressure
- `s`: print current state and raw MS4525 frame bytes
- `h`: print help

Default boot state enables real barometers. The firmware automatically zeroes the first valid barometer differential pressure reading. If either barometer is missing, it falls back to the fake ramp, which rises from `0 km/h` to `100 km/h` in 10 seconds, then falls from `100 km/h` to `0 km/h` in 10 seconds, repeating continuously.

## ArduPilot Setup

Set `ARSPD_TYPE=1` for `I2C-MS4525D0`. Leave bus autodetection enabled first; set `ARSPD_BUS` only if autodetect fails. Reboot the flight controller and check the GCS messages for an MS4525 found message. Then use `p <pa>` in the ESP32 serial monitor and confirm ArduPilot telemetry changes.
