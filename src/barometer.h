#pragma once

#include <Arduino.h>
#include <Wire.h>

struct BarometerReading {
  float pressurePa = 0.0f;
  float temperatureC = 0.0f;
  uint32_t updatedMs = 0;
  bool valid = false;
};

class BarometerDriver {
public:
  virtual ~BarometerDriver() = default;

  virtual bool begin(TwoWire &wire, uint8_t address) = 0;
  virtual bool read(BarometerReading &reading) = 0;
  virtual const char *name() const = 0;
  virtual uint8_t address() const = 0;
};
