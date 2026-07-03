# FlyingRC Airspeed Dual Barometer Simulator

STM32G031G8U6 firmware for a dual absolute-barometer airspeed module. The board
reads two SPA06-003 barometers, calculates the static pressure difference, and
emulates an MS4525 differential-pressure airspeed sensor on the flight
controller I2C bus.

The firmware is intended for FlyingRC STM32 production hardware. It is not a
general PC simulator. "Simulator" here means the board presents an
MS4525-compatible I2C interface to ArduPilot, INAV, and other flight stacks that
already support MS4525 pitot sensors.

## Features

- STM32G031G8U6 production firmware based on STM32Cube/HAL.
- Dual SPA06-003 barometer input on a dedicated I2C master bus.
- MS4525-compatible 4-byte I2C slave frame at address `0x28`.
- DPS310-compatible virtual barometer at address `0x77` for flight-controller
  external-baro detection.
- Startup zero-offset calibration for the barometer pressure difference.
- Append-only Flash storage for calibration records with CRC32 validation.
- WS2812 status LED output for factory and field diagnosis.
- PWLINK2/CMSIS-DAP batch flashing page with calibration readback.
- ST-Link upload target for development.

## Repository Scope

This repository contains the STM32 hardware firmware and supporting production
tools only.

Main files:

| Path | Purpose |
| --- | --- |
| `src/stm32/main.c` | STM32G031 firmware, SPA06 driver, MS4525 emulation, Flash calibration |
| `src/stm32/ws2812.c` | WS2812 debug LED driver |
| `boards/stm32g031g8u6.json` | PlatformIO board definition |
| `stm32g031g8u6_app.ld` | Application linker script, leaving the last 4 KB for calibration |
| `platformio.ini` | Build, upload, and test environments |
| `tools/pwlink2_flash_ui.py` | Local batch flashing and calibration readback web UI |

## Hardware Defaults

| Function | Default |
| --- | --- |
| MCU | STM32G031G8U6 |
| Flight-controller I2C address | `0x28` |
| Virtual DPS310 I2C address | `0x77` |
| Flight-controller I2C pins | I2C1 SCL `PB6`, SDA `PB7` |
| Barometer I2C pins | I2C2 SCL `PA11`, SDA `PA12` |
| Barometer 1 address | `0x76` |
| Barometer 2 address | `0x77` |
| WS2812 debug LED | `PA8` |
| MS4525 pressure range | `1.0 psi` |

Connect SDA, SCL, and GND from the flight controller to the flight-controller
I2C bus. Use 3.3 V-compatible pullups. Connect both SPA06-003 sensors to the
barometer I2C bus and set one sensor to `0x76` and the other to `0x77`.
On flight controllers that already have a real DPS310 at `0x76`, keep the
virtual DPS310 at `0x77` to avoid an I2C address collision.

## How It Works

1. At boot, the STM32 initializes I2C1 as a dual-address I2C slave and I2C2 as
   the SPA06 barometer master bus.
2. The firmware reads both SPA06-003 sensors continuously.
3. The signed pressure difference is calculated as:

   ```text
   diff_pa = BARO_DIFF_SIGN * (baro1_pressure_pa - baro2_pressure_pa)
   output_pa = diff_pa - diff_offset_pa
   ```

4. `output_pa` is encoded into the standard MS4525 4-byte pressure and
   temperature frame.
5. The flight controller sees a normal MS4525 airspeed sensor on I2C address
   `0x28`.
6. The flight controller also sees a DPS310-compatible virtual barometer on I2C
   address `0x77`. Before static-side detection locks, this virtual barometer
   reports the average of both absolute barometers. After local zero offset is
   available and the corrected pressure difference is stable, the lower-pressure
   sensor is locked as the static pressure source.

The pressure sign can be changed at build time with `BARO_DIFF_SIGN`.

## Calibration Storage

The firmware reserves the last two 2 KB Flash pages for calibration:

| Page | Address Range |
| --- | --- |
| Page 0 | `0x0800F000` to `0x0800F7FF` |
| Page 1 | `0x0800F800` to `0x0800FFFF` |

Calibration records are append-only. At boot, the firmware scans both pages,
checks the magic value and CRC32, and loads the valid record with the highest
sequence number.

If a valid record is found, the saved `diff_offset_pa` is used immediately. If
no valid record is found, the board must remain still at power-up. The firmware
captures about 3 seconds of dual-barometer samples, requires at least 32 samples
and standard deviation no higher than 5 Pa, writes one Flash record, and then
uses that offset.

The linker script limits the application image to the first 60 KB of Flash, so
normal firmware uploads do not overwrite the calibration pages. A full-chip
erase or explicit erase of `0x0800F000` to `0x0800FFFF` will remove the saved
calibration.

## LED Status

| LED State | Meaning |
| --- | --- |
| Purple blink | Waiting for flight-controller I2C reads |
| Green heartbeat | Both barometers are healthy and live differential pressure is used |
| Red/orange heartbeat | Partial or degraded barometer input |
| Red heartbeat | No usable barometer input |
| Blue flash | Flight-controller I2C read activity |

## Prerequisites

- PlatformIO Core.
- STM32 PlatformIO platform packages.
- PWLINK2 or another CMSIS-DAP probe for the production upload target.
- ST-Link for the alternate development upload target.
- Python 3 for the batch flashing UI.

The batch UI finds tools in this order: command-line argument, environment
variable, `PATH`, then the default PlatformIO package location under
`~/.platformio`.

Override tool paths when needed:

```sh
PIO=/path/to/pio python3 tools/pwlink2_flash_ui.py
python3 tools/pwlink2_flash_ui.py --pio /path/to/pio
python3 tools/pwlink2_flash_ui.py --openocd /path/to/openocd --openocd-scripts /path/to/scripts
```

## Build

Production PWLINK2/CMSIS-DAP build:

```sh
pio run -e stm32g031g8u6-pwlink2
```

ST-Link development build:

```sh
pio run -e stm32g031g8u6
```

LED and GPIO test builds:

```sh
pio run -e stm32g031g8u6-pwlink2-ledtest
pio run -e stm32g031g8u6-pwlink2-gpiotest
pio run -e stm32g031g8u6-pwlink2-arduino-ledtest
```

## Upload

Upload with PWLINK2/CMSIS-DAP:

```sh
pio run -e stm32g031g8u6-pwlink2 -t upload
```

Upload with ST-Link:

```sh
pio run -e stm32g031g8u6 -t upload
```

## Batch Flashing With PWLINK2

Start the local production page:

```sh
python3 tools/pwlink2_flash_ui.py
```

Then open:

```text
http://127.0.0.1:8765
```

Default behavior is unlimited batch mode. Move the pogo pins to the current
board, keep the board still, and click the flash button. The page uploads the
firmware, waits for startup calibration, dumps the calibration Flash area, and
records:

- Result.
- `offset Pa`.
- `stddev Pa`.
- Sample count.
- Mean temperature.
- Calibration dump path.

Use a fixed count only when required:

```sh
python3 tools/pwlink2_flash_ui.py --count 12
```

The normal flash button preserves existing calibration records. Use
`Retry and recalibrate current board` when a board must be recalibrated. That
flow erases `0x0800F000` to `0x0800FFFF` first, then uploads the firmware and
waits for a fresh calibration record.

## Factory Calibration Guidance

During first power-up or forced recalibration:

1. Keep the board mechanically still.
2. Avoid airflow across either barometer port.
3. Wait at least 3 to 5 seconds after reset.
4. Check the UI readback.

Suggested production checks:

| Check | Typical Result | Action |
| --- | --- | --- |
| `stddev Pa` | `0.3` to `1.0` | Good fixture stability |
| `abs(offset Pa)` | `0` to `8` | Normal |
| `abs(offset Pa)` | `8` to `15` | Usable, mark and observe |
| `abs(offset Pa)` | `> 20` | Recheck assembly, leakage, port stress, or sensor damage |

If `stddev Pa` is low but a board repeatedly calibrates near the same larger
offset, the value is likely a real sensor or assembly offset rather than noise.
The saved offset compensates it, but the board should be tracked as a boundary
sample for production consistency.

After calibration, static airspeed reported by the flight controller should
normally be below about `1` to `2 m/s` when the board is still and protected from
airflow.

## ArduPilot Setup

Use the MS4525 driver:

```text
ARSPD_TYPE = 1
```

Leave bus autodetection enabled first. Set `ARSPD_BUS` only if autodetection
does not find the sensor. Reboot the flight controller and confirm that the GCS
reports an MS4525 airspeed sensor.

ArduPilot performs its own airspeed offset handling at the flight-controller
level. The STM32 board also stores its local dual-barometer static pressure
offset, so the flight controller receives an already zeroed MS4525-style
differential-pressure frame.

## INAV Setup

Select the MS4525 pitot/airspeed sensor type. Keep the airspeed module still and
shielded from airflow during INAV's initial pitot calibration window.

INAV performs calibration inside the flight controller. The STM32 board does not
need a special I2C calibration command from INAV; it exposes a normal
MS4525-compatible sensor frame.

## Build-Time Options

The main options are set in `platformio.ini`:

| Option | Default | Purpose |
| --- | --- | --- |
| `I2C_SLAVE_ADDRESS` | `0x28` | MS4525-compatible slave address |
| `MS4525_PSI_RANGE` | `1.0f` | Encoded pressure range |
| `BARO1_I2C_ADDRESS` | `0x76` | First SPA06 address |
| `BARO2_I2C_ADDRESS` | `0x77` | Second SPA06 address |
| `BARO_DIFF_SIGN` | `1` | Pressure-difference polarity |
| `BARO_AUTOZERO` | `1` | Startup local zero handling |
| `VIRTUAL_DPS310_ENABLED` | `1` | Enable the flight-controller-side virtual DPS310 |
| `VIRTUAL_DPS310_I2C_ADDRESS` | `0x77` | Virtual DPS310 slave address |
| `STATIC_DETECT_MIN_DIFF_PA` | `20.0f` | Pressure difference needed to identify static side |
| `STATIC_DETECT_LOCK_SAMPLES` | `10U` | Consecutive samples required before static-side lock |
| `DEBUG_LED_ENABLED` | `1` | WS2812 status LED |

## License

MIT License. See `LICENSE`.
