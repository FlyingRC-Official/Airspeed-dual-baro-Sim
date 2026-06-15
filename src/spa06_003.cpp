#include "spa06_003.h"

bool Spa06Barometer::begin(TwoWire &wire, uint8_t address) {
  _wire = &wire;
  _address = address;

  uint8_t id = 0;
  if (!read8(kRegId, id) || id != kExpectedId) {
    return false;
  }

  if (!write8(kRegReset, 0x09)) {
    return false;
  }
  delay(10);

  if (!waitReady(0xC0, 100)) {
    return false;
  }

  if (!readCoefficients()) {
    return false;
  }

  const uint8_t config = static_cast<uint8_t>((kRate16Hz << 4) | kOversampling8x);
  if (!write8(kRegPressureConfig, config)) {
    return false;
  }
  if (!write8(kRegTemperatureConfig, config)) {
    return false;
  }
  if (!write8(kRegConfig, 0x00)) {
    return false;
  }

  return write8(kRegMeasureConfig, 0x07);
}

bool Spa06Barometer::read(BarometerReading &reading) {
  uint8_t status = 0;
  if (!read8(kRegMeasureConfig, status)) {
    return false;
  }
  if ((status & 0x30) != 0x30) {
    return false;
  }

  const int32_t rawPressure = readSigned24(kRegPressure);
  const int32_t rawTemperature = readSigned24(kRegTemperature);
  if (rawPressure == INT32_MIN || rawTemperature == INT32_MIN) {
    return false;
  }

  const float tRawSc = static_cast<float>(rawTemperature) / kScale8x;
  const float pRawSc = static_cast<float>(rawPressure) / kScale8x;
  const float p2 = pRawSc * pRawSc;
  const float p3 = p2 * pRawSc;
  const float p4 = p3 * pRawSc;

  reading.temperatureC = static_cast<float>(_c0) * 0.5f + static_cast<float>(_c1) * tRawSc;
  reading.pressurePa = static_cast<float>(_c00) + static_cast<float>(_c10) * pRawSc +
                       static_cast<float>(_c20) * p2 + static_cast<float>(_c30) * p3 +
                       static_cast<float>(_c40) * p4 +
                       tRawSc * (static_cast<float>(_c01) + static_cast<float>(_c11) * pRawSc +
                                 static_cast<float>(_c21) * p2 + static_cast<float>(_c31) * p3);
  reading.updatedMs = millis();
  reading.valid = true;
  return true;
}

bool Spa06Barometer::readCoefficients() {
  uint8_t data[21] = {0};
  if (!readBytes(kRegCoef, data, sizeof(data))) {
    return false;
  }

  _c0 = signExtend12((static_cast<uint16_t>(data[0]) << 4) | (data[1] >> 4));
  _c1 = signExtend12((static_cast<uint16_t>(data[1] & 0x0F) << 8) | data[2]);
  _c00 = signExtend20((static_cast<uint32_t>(data[3]) << 12) |
                      (static_cast<uint32_t>(data[4]) << 4) | (data[5] >> 4));
  _c10 = signExtend20((static_cast<uint32_t>(data[5] & 0x0F) << 16) |
                      (static_cast<uint32_t>(data[6]) << 8) | data[7]);
  _c01 = static_cast<int16_t>((static_cast<uint16_t>(data[8]) << 8) | data[9]);
  _c11 = static_cast<int16_t>((static_cast<uint16_t>(data[10]) << 8) | data[11]);
  _c20 = static_cast<int16_t>((static_cast<uint16_t>(data[12]) << 8) | data[13]);
  _c21 = static_cast<int16_t>((static_cast<uint16_t>(data[14]) << 8) | data[15]);
  _c30 = static_cast<int16_t>((static_cast<uint16_t>(data[16]) << 8) | data[17]);
  _c31 = signExtend12((static_cast<uint16_t>(data[18]) << 4) | (data[19] >> 4));
  _c40 = signExtend12((static_cast<uint16_t>(data[19] & 0x0F) << 8) | data[20]);

  return true;
}

bool Spa06Barometer::waitReady(uint8_t mask, uint32_t timeoutMs) {
  const uint32_t startMs = millis();
  do {
    uint8_t status = 0;
    if (read8(kRegMeasureConfig, status) && (status & mask) == mask) {
      return true;
    }
    delay(5);
  } while (millis() - startMs < timeoutMs);

  return false;
}

bool Spa06Barometer::readBytes(uint8_t reg, uint8_t *data, size_t length) {
  if (_wire == nullptr || length == 0) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  const uint8_t received = _wire->requestFrom(static_cast<int>(_address), static_cast<int>(length));
  if (received != length) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    data[i] = _wire->read();
  }
  return true;
}

bool Spa06Barometer::read8(uint8_t reg, uint8_t &value) {
  return readBytes(reg, &value, 1);
}

bool Spa06Barometer::write8(uint8_t reg, uint8_t value) {
  if (_wire == nullptr) {
    return false;
  }

  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  return _wire->endTransmission() == 0;
}

int32_t Spa06Barometer::readSigned24(uint8_t reg) {
  uint8_t data[3] = {0};
  if (!readBytes(reg, data, sizeof(data))) {
    return INT32_MIN;
  }

  int32_t value = (static_cast<int32_t>(data[0]) << 16) | (static_cast<int32_t>(data[1]) << 8) |
                  static_cast<int32_t>(data[2]);
  if (value & 0x800000) {
    value |= 0xFF000000;
  }
  return value;
}

int16_t Spa06Barometer::signExtend12(uint16_t value) {
  value &= 0x0FFF;
  if (value & 0x0800) {
    value |= 0xF000;
  }
  return static_cast<int16_t>(value);
}

int32_t Spa06Barometer::signExtend20(uint32_t value) {
  value &= 0x000FFFFF;
  if (value & 0x00080000) {
    value |= 0xFFF00000;
  }
  return static_cast<int32_t>(value);
}
