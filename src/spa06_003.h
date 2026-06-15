#pragma once

#include "barometer.h"

class Spa06Barometer : public BarometerDriver {
public:
  bool begin(TwoWire &wire, uint8_t address) override;
  bool read(BarometerReading &reading) override;
  const char *name() const override { return "SPA06-003"; }
  uint8_t address() const override { return _address; }

private:
  static constexpr uint8_t kRegPressure = 0x00;
  static constexpr uint8_t kRegTemperature = 0x03;
  static constexpr uint8_t kRegPressureConfig = 0x06;
  static constexpr uint8_t kRegTemperatureConfig = 0x07;
  static constexpr uint8_t kRegMeasureConfig = 0x08;
  static constexpr uint8_t kRegConfig = 0x09;
  static constexpr uint8_t kRegReset = 0x0C;
  static constexpr uint8_t kRegId = 0x0D;
  static constexpr uint8_t kRegCoef = 0x10;

  static constexpr uint8_t kExpectedId = 0x11;
  static constexpr uint8_t kOversampling8x = 3;
  static constexpr uint8_t kRate16Hz = 4;
  static constexpr float kScale8x = 7864320.0f;

  bool readCoefficients();
  bool waitReady(uint8_t mask, uint32_t timeoutMs);
  bool readBytes(uint8_t reg, uint8_t *data, size_t length);
  bool read8(uint8_t reg, uint8_t &value);
  bool write8(uint8_t reg, uint8_t value);
  int32_t readSigned24(uint8_t reg);
  static int16_t signExtend12(uint16_t value);
  static int32_t signExtend20(uint32_t value);

  TwoWire *_wire = nullptr;
  uint8_t _address = 0;

  int16_t _c0 = 0;
  int16_t _c1 = 0;
  int32_t _c00 = 0;
  int32_t _c10 = 0;
  int16_t _c01 = 0;
  int16_t _c11 = 0;
  int16_t _c20 = 0;
  int16_t _c21 = 0;
  int16_t _c30 = 0;
  int16_t _c31 = 0;
  int16_t _c40 = 0;
};
