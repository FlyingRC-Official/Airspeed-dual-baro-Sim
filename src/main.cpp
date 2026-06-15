#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>

#include "spa06_003.h"

#ifndef MONITOR_BAUD
#define MONITOR_BAUD 115200
#endif

#ifndef I2C_SLAVE_ADDRESS
#define I2C_SLAVE_ADDRESS 0x28
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 8
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 9
#endif

#ifndef I2C_BUS_HZ
#define I2C_BUS_HZ 100000
#endif

#ifndef MS4525_PSI_RANGE
#define MS4525_PSI_RANGE 1.0f
#endif

#ifndef USE_BAROMETERS
#define USE_BAROMETERS 1
#endif

#ifndef BARO_I2C_SDA_PIN
#define BARO_I2C_SDA_PIN 4
#endif

#ifndef BARO_I2C_SCL_PIN
#define BARO_I2C_SCL_PIN 5
#endif

#ifndef BARO_I2C_HZ
#define BARO_I2C_HZ 400000
#endif

#ifndef BARO1_I2C_ADDRESS
#define BARO1_I2C_ADDRESS 0x76
#endif

#ifndef BARO2_I2C_ADDRESS
#define BARO2_I2C_ADDRESS 0x77
#endif

#ifndef BARO_DIFF_SIGN
#define BARO_DIFF_SIGN 1
#endif

#ifndef BARO_AUTOZERO
#define BARO_AUTOZERO 1
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S3) && ARDUINO_USB_MODE
#define CONSOLE USBSerial
#else
#define CONSOLE Serial
#endif

namespace {

constexpr float kPsiToPa = 6894.757f;
constexpr float kRawFullScale = 16383.0f;
constexpr float kTempRawFullScale = 2047.0f;
constexpr float kAirDensityKgM3 = 1.225f;
constexpr float kRampMaxSpeedKmh = 100.0f;
constexpr uint32_t kRampHalfPeriodMs = 10000;
constexpr uint32_t kRampFullPeriodMs = kRampHalfPeriodMs * 2;

portMUX_TYPE frameMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t responseFrame[4] = {0};
volatile uint32_t requestCount = 0;
volatile uint32_t receiveCount = 0;
volatile uint32_t measurementCommandCount = 0;
volatile uint8_t lastReceiveLength = 0;
volatile uint8_t lastReceiveByte = 0;

Spa06Barometer barometer1;
Spa06Barometer barometer2;
BarometerReading barometerReading1;
BarometerReading barometerReading2;

float fakePressurePa = 0.0f;
float fakeTemperatureC = 25.0f;
bool rampEnabled = true;
bool barometersEnabled = USE_BAROMETERS != 0;
bool barometer1Online = false;
bool barometer2Online = false;
bool barometerOffsetValid = false;
float barometerDiffOffsetPa = 0.0f;
uint32_t barometerSampleCount = 0;
uint32_t barometerFailureCount = 0;
uint32_t lastBarometerSampleMs = 0;
uint32_t lastRampMs = 0;
char commandBuffer[96] = {0};
size_t commandLength = 0;

int16_t clampInt16(float value, int16_t low, int16_t high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return static_cast<int16_t>(lroundf(value));
}

int16_t pressurePaToRaw(float pressurePa) {
  const float pressurePsi = pressurePa / kPsiToPa;
  const float raw = (0.1f * kRawFullScale) +
                    ((MS4525_PSI_RANGE - pressurePsi) * (0.8f * kRawFullScale)) /
                        (2.0f * MS4525_PSI_RANGE);

  return clampInt16(raw, 1, 0x3FFE);
}

int16_t temperatureCToRaw(float temperatureC) {
  const float raw = ((temperatureC + 50.0f) * kTempRawFullScale) / 200.0f;
  return clampInt16(raw, 1, 0x7FE);
}

void updateResponseFrame() {
  const uint16_t pressureRaw = static_cast<uint16_t>(pressurePaToRaw(fakePressurePa));
  const uint16_t temperatureRaw = static_cast<uint16_t>(temperatureCToRaw(fakeTemperatureC));

  const uint8_t nextFrame[4] = {
      static_cast<uint8_t>((pressureRaw >> 8) & 0x3F),
      static_cast<uint8_t>(pressureRaw & 0xFF),
      static_cast<uint8_t>((temperatureRaw >> 3) & 0xFF),
      static_cast<uint8_t>((temperatureRaw & 0x07) << 5),
  };

  portENTER_CRITICAL(&frameMux);
  for (size_t i = 0; i < sizeof(nextFrame); i++) {
    responseFrame[i] = nextFrame[i];
  }
  portEXIT_CRITICAL(&frameMux);
}

void copyResponseFrame(uint8_t *destination) {
  portENTER_CRITICAL_ISR(&frameMux);
  for (size_t i = 0; i < 4; i++) {
    destination[i] = responseFrame[i];
  }
  portEXIT_CRITICAL_ISR(&frameMux);
}

void onI2CRequest() {
  uint8_t frame[4];
  copyResponseFrame(frame);
  Wire.write(frame, sizeof(frame));
  requestCount++;
}

void onI2CReceive(int length) {
  receiveCount++;
  lastReceiveLength = static_cast<uint8_t>(min(length, 255));

  bool first = true;
  while (Wire.available()) {
    const uint8_t value = Wire.read();
    if (first) {
      lastReceiveByte = value;
      first = false;
    }
    if (value == 0x00) {
      measurementCommandCount++;
    }
  }
}

void printHelp() {
  CONSOLE.println();
  CONSOLE.println(F("Commands:"));
  CONSOLE.println(F("  p <pa>   set fake differential pressure in Pascals"));
  CONSOLE.println(F("  t <c>    set fake temperature in Celsius"));
  CONSOLE.println(F("  z        set pressure to 0 Pa"));
  CONSOLE.println(F("  r on     enable 0-100-0 km/h fake airspeed ramp"));
  CONSOLE.println(F("  r off    disable slow fake pressure ramp"));
  CONSOLE.println(F("  b on     enable real dual-barometer input"));
  CONSOLE.println(F("  b off    disable real dual-barometer input"));
  CONSOLE.println(F("  c        zero current barometer differential pressure"));
  CONSOLE.println(F("  s        print current state"));
  CONSOLE.println(F("  h        print this help"));
  CONSOLE.println();
}

void printStatus() {
  uint8_t frame[4];
  copyResponseFrame(frame);

  CONSOLE.println();
  CONSOLE.println(F("MS4525 emulator status"));
  CONSOLE.printf("  pressure_pa: %.2f\n", fakePressurePa);
  CONSOLE.printf("  temperature_c: %.2f\n", fakeTemperatureC);
  CONSOLE.printf("  ramp: %s\n", rampEnabled ? "on" : "off");
  CONSOLE.printf("  barometers: %s\n", barometersEnabled ? "enabled" : "disabled");
  CONSOLE.printf("  baro1: %s addr=0x%02X pressure_pa=%.2f temp_c=%.2f\n",
                 barometer1Online ? "online" : "offline",
                 BARO1_I2C_ADDRESS,
                 barometerReading1.pressurePa,
                 barometerReading1.temperatureC);
  CONSOLE.printf("  baro2: %s addr=0x%02X pressure_pa=%.2f temp_c=%.2f\n",
                 barometer2Online ? "online" : "offline",
                 BARO2_I2C_ADDRESS,
                 barometerReading2.pressurePa,
                 barometerReading2.temperatureC);
  CONSOLE.printf("  baro_diff_offset_pa: %.2f valid=%s\n",
                 barometerDiffOffsetPa,
                 barometerOffsetValid ? "yes" : "no");
  CONSOLE.printf("  baro_samples: %lu failures=%lu\n",
                 static_cast<unsigned long>(barometerSampleCount),
                 static_cast<unsigned long>(barometerFailureCount));
  CONSOLE.printf("  ramp_profile: 0-%.0f-0 km/h over %lu seconds\n",
                 kRampMaxSpeedKmh,
                 static_cast<unsigned long>(kRampFullPeriodMs / 1000));
  CONSOLE.printf("  i2c_address: 0x%02X\n", I2C_SLAVE_ADDRESS);
  CONSOLE.printf("  sda_pin: %d\n", I2C_SDA_PIN);
  CONSOLE.printf("  scl_pin: %d\n", I2C_SCL_PIN);
  CONSOLE.printf("  baro_sda_pin: %d\n", BARO_I2C_SDA_PIN);
  CONSOLE.printf("  baro_scl_pin: %d\n", BARO_I2C_SCL_PIN);
  CONSOLE.printf("  bus_hz: %u\n", static_cast<unsigned>(I2C_BUS_HZ));
  CONSOLE.printf("  frame: %02X %02X %02X %02X\n", frame[0], frame[1], frame[2], frame[3]);
  CONSOLE.printf("  i2c_requests: %lu\n", static_cast<unsigned long>(requestCount));
  CONSOLE.printf("  i2c_receives: %lu\n", static_cast<unsigned long>(receiveCount));
  CONSOLE.printf("  measure_commands: %lu\n", static_cast<unsigned long>(measurementCommandCount));
  CONSOLE.printf("  last_receive_len: %u\n", static_cast<unsigned>(lastReceiveLength));
  CONSOLE.printf("  last_receive_first_byte: 0x%02X\n", static_cast<unsigned>(lastReceiveByte));
  CONSOLE.println();
}

void setPressure(float pressurePa) {
  fakePressurePa = pressurePa;
  updateResponseFrame();
}

void setTemperature(float temperatureC) {
  fakeTemperatureC = temperatureC;
  updateResponseFrame();
}

float speedKmhToPressurePa(float speedKmh) {
  const float speedMs = speedKmh / 3.6f;
  return 0.5f * kAirDensityKgM3 * speedMs * speedMs;
}

void setBarometerOffsetToCurrentReading() {
  if (!barometerReading1.valid || !barometerReading2.valid) {
    return;
  }

  const float signedDiffPa =
      (barometerReading1.pressurePa - barometerReading2.pressurePa) * static_cast<float>(BARO_DIFF_SIGN);
  barometerDiffOffsetPa = signedDiffPa;
  barometerOffsetValid = true;
}

void initBarometers() {
#if USE_BAROMETERS
  Wire1.begin(BARO_I2C_SDA_PIN, BARO_I2C_SCL_PIN, BARO_I2C_HZ);
  barometer1Online = barometer1.begin(Wire1, BARO1_I2C_ADDRESS);
  barometer2Online = barometer2.begin(Wire1, BARO2_I2C_ADDRESS);

  CONSOLE.printf("Barometer bus SDA=%d SCL=%d hz=%u\n",
                 BARO_I2C_SDA_PIN,
                 BARO_I2C_SCL_PIN,
                 static_cast<unsigned>(BARO_I2C_HZ));
  CONSOLE.printf("Barometer 1 %s at 0x%02X\n", barometer1Online ? "found" : "missing", BARO1_I2C_ADDRESS);
  CONSOLE.printf("Barometer 2 %s at 0x%02X\n", barometer2Online ? "found" : "missing", BARO2_I2C_ADDRESS);
#endif
}

void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }

  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "h") == 0 || strcmp(line, "help") == 0) {
    printHelp();
    return;
  }

  if (strcmp(line, "s") == 0 || strcmp(line, "status") == 0) {
    printStatus();
    return;
  }

  if (strcmp(line, "z") == 0) {
    barometersEnabled = false;
    rampEnabled = false;
    setPressure(0.0f);
    CONSOLE.println(F("pressure_pa=0.00"));
    return;
  }

  float value = 0.0f;
  if (sscanf(line, "p %f", &value) == 1) {
    barometersEnabled = false;
    rampEnabled = false;
    setPressure(value);
    CONSOLE.printf("pressure_pa=%.2f\n", fakePressurePa);
    return;
  }

  if (sscanf(line, "t %f", &value) == 1) {
    setTemperature(value);
    CONSOLE.printf("temperature_c=%.2f\n", fakeTemperatureC);
    return;
  }

  if (strcmp(line, "r on") == 0) {
    barometersEnabled = false;
    rampEnabled = true;
    lastRampMs = millis();
    CONSOLE.println(F("ramp=on"));
    return;
  }

  if (strcmp(line, "r off") == 0) {
    rampEnabled = false;
    CONSOLE.println(F("ramp=off"));
    return;
  }

  if (strcmp(line, "b on") == 0) {
    barometersEnabled = true;
    CONSOLE.println(F("barometers=on"));
    return;
  }

  if (strcmp(line, "b off") == 0) {
    barometersEnabled = false;
    CONSOLE.println(F("barometers=off"));
    return;
  }

  if (strcmp(line, "c") == 0 || strcmp(line, "zero") == 0) {
    setBarometerOffsetToCurrentReading();
    CONSOLE.printf("baro_diff_offset_pa=%.2f valid=%s\n",
                   barometerDiffOffsetPa,
                   barometerOffsetValid ? "yes" : "no");
    return;
  }

  CONSOLE.print(F("Unknown command: "));
  CONSOLE.println(line);
  printHelp();
}

void pollSerialCommands() {
  while (CONSOLE.available()) {
    const char c = static_cast<char>(CONSOLE.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      commandBuffer[commandLength] = '\0';
      handleCommand(commandBuffer);
      commandLength = 0;
      return;
    }
    if (commandLength < sizeof(commandBuffer) - 1) {
      commandBuffer[commandLength++] = c;
    }
  }
}

void updateRamp() {
  if (!rampEnabled) {
    return;
  }

  const uint32_t now = millis();
  if (now - lastRampMs < 50) {
    return;
  }
  lastRampMs = now;

  const uint32_t rampTimeMs = now % kRampFullPeriodMs;
  const float speedKmh = rampTimeMs <= kRampHalfPeriodMs
                             ? kRampMaxSpeedKmh * static_cast<float>(rampTimeMs) /
                                   static_cast<float>(kRampHalfPeriodMs)
                             : kRampMaxSpeedKmh *
                                   static_cast<float>(kRampFullPeriodMs - rampTimeMs) /
                                   static_cast<float>(kRampHalfPeriodMs);
  setPressure(speedKmhToPressurePa(speedKmh));
}

bool updateBarometers() {
#if !USE_BAROMETERS
  return false;
#else
  if (!barometersEnabled) {
    return false;
  }

  const uint32_t now = millis();
  if (now - lastBarometerSampleMs < 50) {
    return barometerReading1.valid && barometerReading2.valid;
  }
  lastBarometerSampleMs = now;

  const bool read1 = barometer1Online && barometer1.read(barometerReading1);
  const bool read2 = barometer2Online && barometer2.read(barometerReading2);

  if (!read1 || !read2) {
    barometerFailureCount++;
    return barometerReading1.valid && barometerReading2.valid && (now - barometerReading1.updatedMs) < 500 &&
           (now - barometerReading2.updatedMs) < 500;
  }

  barometerSampleCount++;
  const float signedDiffPa =
      (barometerReading1.pressurePa - barometerReading2.pressurePa) * static_cast<float>(BARO_DIFF_SIGN);
#if BARO_AUTOZERO
  if (!barometerOffsetValid) {
    barometerDiffOffsetPa = signedDiffPa;
    barometerOffsetValid = true;
  }
#endif

  setPressure(signedDiffPa - barometerDiffOffsetPa);
  setTemperature((barometerReading1.temperatureC + barometerReading2.temperatureC) * 0.5f);
  return true;
#endif
}

} // namespace

void setup() {
  CONSOLE.begin(MONITOR_BAUD);
  delay(500);

  updateResponseFrame();
  initBarometers();

  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  const bool i2cStarted = Wire.begin(static_cast<uint8_t>(I2C_SLAVE_ADDRESS),
                                    I2C_SDA_PIN,
                                    I2C_SCL_PIN,
                                    I2C_BUS_HZ);

#if CONFIG_IDF_TARGET_ESP32
  uint8_t initialFrame[4];
  copyResponseFrame(initialFrame);
  Wire.slaveWrite(initialFrame, sizeof(initialFrame));
#endif

  CONSOLE.println();
  CONSOLE.println(F("FlyingRC AirSpeed ESP32 MS4525 emulator"));
  CONSOLE.printf("I2C slave %s at 0x%02X, SDA=%d, SCL=%d, bus_hz=%u\n",
                 i2cStarted ? "started" : "failed",
                 I2C_SLAVE_ADDRESS,
                 I2C_SDA_PIN,
                 I2C_SCL_PIN,
                 static_cast<unsigned>(I2C_BUS_HZ));
  printStatus();
  printHelp();
}

void loop() {
  pollSerialCommands();
  if (!updateBarometers()) {
    updateRamp();
  }
  delay(5);
}
