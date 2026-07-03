#include "stm32g0xx_hal.h"
#include "ws2812.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef I2C_SLAVE_ADDRESS
#define I2C_SLAVE_ADDRESS 0x28
#endif

#ifndef MS4525_PSI_RANGE
#define MS4525_PSI_RANGE 1.0f
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

#ifndef VIRTUAL_DPS310_ENABLED
#define VIRTUAL_DPS310_ENABLED 1
#endif

#ifndef VIRTUAL_DPS310_I2C_ADDRESS
#define VIRTUAL_DPS310_I2C_ADDRESS 0x77
#endif

#ifndef STATIC_DETECT_MIN_DIFF_PA
#define STATIC_DETECT_MIN_DIFF_PA 20.0f
#endif

#ifndef STATIC_DETECT_LOCK_SAMPLES
#define STATIC_DETECT_LOCK_SAMPLES 10U
#endif

#ifndef DEBUG_LED_ENABLED
#define DEBUG_LED_ENABLED 0
#endif

#ifndef DEBUG_LED_GPIO_PORT
#define DEBUG_LED_GPIO_PORT GPIOA
#endif

#ifndef DEBUG_LED_GPIO_PIN
#define DEBUG_LED_GPIO_PIN GPIO_PIN_8
#endif

#ifndef DEBUG_LED_TEST
#define DEBUG_LED_TEST 0
#endif

#ifndef DEBUG_LED_GPIO_TEST
#define DEBUG_LED_GPIO_TEST 0
#endif

#define SPA06_REG_PRESSURE 0x00
#define SPA06_REG_TEMPERATURE 0x03
#define SPA06_REG_PRESSURE_CONFIG 0x06
#define SPA06_REG_TEMPERATURE_CONFIG 0x07
#define SPA06_REG_MEASURE_CONFIG 0x08
#define SPA06_REG_CONFIG 0x09
#define SPA06_REG_RESET 0x0C
#define SPA06_REG_ID 0x0D
#define SPA06_REG_COEF 0x10
#define SPA06_EXPECTED_ID 0x11
#define SPA06_OVERSAMPLING_8X 3
#define SPA06_RATE_16HZ 4
#define SPA06_SCALE_8X 7864320.0f

#define I2C_TIMING_100KHZ_64MHZ 0x30303D5BU
#define MS4525_FRAME_LEN 4U

#if VIRTUAL_DPS310_ENABLED
#define DPS310_REG_PRESSURE 0x00
#define DPS310_REG_TEMPERATURE 0x03
#define DPS310_REG_PRESSURE_CONFIG 0x06
#define DPS310_REG_TEMPERATURE_CONFIG 0x07
#define DPS310_REG_MEASURE_CONFIG 0x08
#define DPS310_REG_CONFIG 0x09
#define DPS310_REG_RESET 0x0C
#define DPS310_REG_ID 0x0D
#define DPS310_REG_COEF 0x10
#define DPS310_REG_COEF_SRCE 0x28
#define DPS310_EXPECTED_ID 0x10
#define DPS310_COEF_LEN 18U
#define DPS310_DATA_LEN 6U
#define DPS310_SCALE_16X 253952.0f
#define DPS310_READY_STATUS 0xF0U
#define DPS310_MEAS_CTRL_MASK 0x07U
#define DPS310_RESET_COMMAND 0x09U
#define DPS310_COEF_SRCE_TMP_EXT 0x80U
#define DPS310_SYNTH_C00_PA 100000.0f
#define DPS310_SYNTH_C10_PA 2500.0f
#define DPS310_SYNTH_C1_TEMP 100.0f
#define STATIC_SOURCE_UNKNOWN 0U
#define STATIC_SOURCE_BARO1 1U
#define STATIC_SOURCE_BARO2 2U
#endif

#define CAL_FLASH_PAGE_SIZE 0x800U
#define CAL_FLASH_PAGE0_ADDR 0x0800F000U
#define CAL_FLASH_PAGE1_ADDR 0x0800F800U
#define CAL_FLASH_END_ADDR 0x08010000U
#define CAL_RECORD_MAGIC 0x46524153U
#define CAL_RECORD_VERSION 1U
#define CAL_CAPTURE_MS 3000U
#define CAL_MIN_SAMPLES 32U
#define CAL_MAX_STDDEV_PA 5.0f

typedef struct {
  int16_t c0;
  int16_t c1;
  int32_t c00;
  int32_t c10;
  int16_t c01;
  int16_t c11;
  int16_t c20;
  int16_t c21;
  int16_t c30;
  int16_t c31;
  int16_t c40;
  uint8_t address;
  bool online;
  bool valid;
  float pressure_pa;
  float temperature_c;
  uint32_t updated_ms;
} Spa06;

typedef struct {
  uint16_t version;
  uint16_t length;
  uint32_t sequence;
  float diff_offset_pa;
  float baro1_mean_pa;
  float baro2_mean_pa;
  float temperature_mean_c;
  float diff_stddev_pa;
  uint32_t sample_count;
  uint32_t flags;
  uint32_t reserved;
  uint32_t crc32;
  uint32_t magic;
} AirspeedCalRecord;

typedef char cal_record_must_be_doubleword_aligned[(sizeof(AirspeedCalRecord) % 8U) == 0U ? 1 : -1];

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

static Spa06 baro1 = {.address = BARO1_I2C_ADDRESS};
static Spa06 baro2 = {.address = BARO2_I2C_ADDRESS};

static volatile uint8_t response_frame[MS4525_FRAME_LEN] = {0};
static uint8_t tx_frame[MS4525_FRAME_LEN] = {0};
static volatile uint32_t request_count = 0;
static volatile uint32_t receive_count = 0;

typedef enum {
  I2C1_SLAVE_NONE = 0,
  I2C1_SLAVE_MS4525,
  I2C1_SLAVE_DPS310
} I2c1SlaveDevice;

static volatile I2c1SlaveDevice i2c1_active_slave = I2C1_SLAVE_NONE;
static volatile uint8_t i2c1_ms4525_tx_index = 0;

static float pressure_pa = 0.0f;
static float temperature_c = 25.0f;
static float diff_offset_pa = 0.0f;
static bool offset_valid = false;
static uint32_t startup_cal_start_ms = 0;
static uint32_t startup_cal_count = 0;
static float startup_cal_diff_sum = 0.0f;
static float startup_cal_diff_sq_sum = 0.0f;
static float startup_cal_baro1_sum = 0.0f;
static float startup_cal_baro2_sum = 0.0f;
static float startup_cal_temp_sum = 0.0f;
static uint32_t last_led_update_ms = 0;
static uint32_t last_fc_request_ms = 0;
static uint32_t last_led_request_count = 0;
static uint32_t last_fc_flash_ms = 0;
static uint32_t fc_flash_until_ms = 0;

#if VIRTUAL_DPS310_ENABLED
static const uint8_t dps310_coefficients[DPS310_COEF_LEN] = {
    0x00, 0x00, 0x64, 0x18, 0x6A, 0x00, 0x09, 0xC4, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static volatile uint8_t dps310_data_frame[DPS310_DATA_LEN] = {0};
static volatile uint8_t dps310_reg_pointer = DPS310_REG_ID;
static volatile uint8_t dps310_rx_count = 0;
static volatile uint8_t dps310_pressure_config = 0;
static volatile uint8_t dps310_temperature_config = 0;
static volatile uint8_t dps310_measure_config = 0;
static volatile uint8_t dps310_config = 0;
static volatile uint8_t dps310_reset = 0;
static uint8_t static_pressure_source = STATIC_SOURCE_UNKNOWN;
static uint8_t static_pressure_candidate = STATIC_SOURCE_UNKNOWN;
static uint32_t static_pressure_candidate_count = 0;
#endif

static int16_t sign_extend_12(uint16_t value) {
  value &= 0x0FFFU;
  if (value & 0x0800U) {
    value |= 0xF000U;
  }
  return (int16_t)value;
}

static int32_t sign_extend_20(uint32_t value) {
  value &= 0x000FFFFFU;
  if (value & 0x00080000U) {
    value |= 0xFFF00000U;
  }
  return (int32_t)value;
}

static int32_t sign_extend_24(uint32_t value) {
  value &= 0x00FFFFFFU;
  if (value & 0x00800000U) {
    value |= 0xFF000000U;
  }
  return (int32_t)value;
}

static int16_t clamp_i16(float value, int16_t low, int16_t high) {
  if (value < (float)low) {
    return low;
  }
  if (value > (float)high) {
    return high;
  }
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static int32_t clamp_i32(float value, int32_t low, int32_t high) {
  if (value < (float)low) {
    return low;
  }
  if (value > (float)high) {
    return high;
  }
  return (int32_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

static float abs_float(float value) {
  return (value < 0.0f) ? -value : value;
}

static float sqrt_approx(float value) {
  if (value <= 0.0f) {
    return 0.0f;
  }

  float x = value;
  for (uint32_t i = 0; i < 6U; i++) {
    x = 0.5f * (x + (value / x));
  }
  return x;
}

static uint32_t crc32_update(uint32_t crc, uint8_t data) {
  crc ^= data;
  for (uint32_t bit = 0; bit < 8U; bit++) {
    crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
  }
  return crc;
}

static uint32_t crc32_calc(const void *data, uint32_t length) {
  const uint8_t *bytes = (const uint8_t *)data;
  uint32_t crc = 0xFFFFFFFFU;
  for (uint32_t i = 0; i < length; i++) {
    crc = crc32_update(crc, bytes[i]);
  }
  return crc ^ 0xFFFFFFFFU;
}

static bool cal_record_slot_empty(uint32_t address) {
  const uint32_t *word = (const uint32_t *)address;
  for (uint32_t i = 0; i < (sizeof(AirspeedCalRecord) / 4U); i++) {
    if (word[i] != 0xFFFFFFFFU) {
      return false;
    }
  }
  return true;
}

static bool cal_record_valid(const AirspeedCalRecord *record) {
  if (record->magic != CAL_RECORD_MAGIC ||
      record->version != CAL_RECORD_VERSION ||
      record->length != sizeof(AirspeedCalRecord) ||
      record->sample_count == 0U ||
      record->diff_offset_pa < -10000.0f ||
      record->diff_offset_pa > 10000.0f) {
    return false;
  }

  const uint32_t crc = crc32_calc(record, sizeof(AirspeedCalRecord) - 8U);
  return crc == record->crc32;
}

static uint32_t cal_page_start(uint32_t address) {
  return (address >= CAL_FLASH_PAGE1_ADDR) ? CAL_FLASH_PAGE1_ADDR : CAL_FLASH_PAGE0_ADDR;
}

static uint32_t cal_other_page(uint32_t page_start) {
  return (page_start == CAL_FLASH_PAGE0_ADDR) ? CAL_FLASH_PAGE1_ADDR : CAL_FLASH_PAGE0_ADDR;
}

static bool cal_find_empty_slot(uint32_t page_start, uint32_t *empty_addr) {
  const uint32_t page_end = page_start + CAL_FLASH_PAGE_SIZE;
  for (uint32_t addr = page_start; addr + sizeof(AirspeedCalRecord) <= page_end;
       addr += sizeof(AirspeedCalRecord)) {
    if (cal_record_slot_empty(addr)) {
      *empty_addr = addr;
      return true;
    }
  }
  return false;
}

static bool cal_find_latest_record(const AirspeedCalRecord **record, uint32_t *record_addr) {
  const AirspeedCalRecord *best = NULL;
  uint32_t best_addr = 0U;

  for (uint32_t addr = CAL_FLASH_PAGE0_ADDR; addr + sizeof(AirspeedCalRecord) <= CAL_FLASH_END_ADDR;
       addr += sizeof(AirspeedCalRecord)) {
    const AirspeedCalRecord *candidate = (const AirspeedCalRecord *)addr;
    if (!cal_record_valid(candidate)) {
      continue;
    }
    if (best == NULL || candidate->sequence > best->sequence) {
      best = candidate;
      best_addr = addr;
    }
  }

  if (best == NULL) {
    return false;
  }

  *record = best;
  *record_addr = best_addr;
  return true;
}

static bool cal_erase_page(uint32_t page_start) {
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t page_error = 0U;

  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Page = (page_start - FLASH_BASE) / FLASH_PAGE_SIZE;
  erase.NbPages = 1U;

  return HAL_FLASHEx_Erase(&erase, &page_error) == HAL_OK;
}

static bool cal_program_record(uint32_t address, const AirspeedCalRecord *record) {
  const uint32_t *word = (const uint32_t *)record;
  const uint32_t word_count = sizeof(AirspeedCalRecord) / 4U;

  for (uint32_t i = 0; i < word_count - 2U; i += 2U) {
    const uint64_t doubleword = (uint64_t)word[i] | ((uint64_t)word[i + 1U] << 32);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address + (i * 4U), doubleword) != HAL_OK) {
      return false;
    }
  }

  const uint64_t commit_doubleword =
      (uint64_t)word[word_count - 2U] | ((uint64_t)word[word_count - 1U] << 32);
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                           address + ((word_count - 2U) * 4U),
                           commit_doubleword) == HAL_OK;
}

static bool cal_save_record(float offset_pa,
                            float baro1_mean_pa,
                            float baro2_mean_pa,
                            float temp_mean_c,
                            float diff_stddev_pa,
                            uint32_t sample_count) {
  const AirspeedCalRecord *latest = NULL;
  uint32_t latest_addr = 0U;
  const bool have_latest = cal_find_latest_record(&latest, &latest_addr);
  const uint32_t active_page = have_latest ? cal_page_start(latest_addr) : CAL_FLASH_PAGE0_ADDR;
  uint32_t write_addr = 0U;
  bool erase_old_page_after_write = false;
  const uint32_t old_page = active_page;

  if (!cal_find_empty_slot(active_page, &write_addr)) {
    const uint32_t target_page = cal_other_page(active_page);
    if (HAL_FLASH_Unlock() != HAL_OK) {
      return false;
    }
    if (!cal_erase_page(target_page)) {
      HAL_FLASH_Lock();
      return false;
    }
    HAL_FLASH_Lock();
    write_addr = target_page;
    erase_old_page_after_write = true;
  }

  AirspeedCalRecord record = {
      .version = CAL_RECORD_VERSION,
      .length = sizeof(AirspeedCalRecord),
      .sequence = have_latest ? latest->sequence + 1U : 1U,
      .diff_offset_pa = offset_pa,
      .baro1_mean_pa = baro1_mean_pa,
      .baro2_mean_pa = baro2_mean_pa,
      .temperature_mean_c = temp_mean_c,
      .diff_stddev_pa = diff_stddev_pa,
      .sample_count = sample_count,
      .flags = 0U,
      .reserved = 0U,
      .crc32 = 0U,
      .magic = CAL_RECORD_MAGIC,
  };
  record.crc32 = crc32_calc(&record, sizeof(AirspeedCalRecord) - 8U);

  if (HAL_FLASH_Unlock() != HAL_OK) {
    return false;
  }

  const bool programmed = cal_program_record(write_addr, &record);
  const bool verified = programmed && cal_record_valid((const AirspeedCalRecord *)write_addr);

  if (verified && erase_old_page_after_write) {
    (void)cal_erase_page(old_page);
  }

  HAL_FLASH_Lock();
  return verified;
}

static void cal_load_flash_offset(void) {
  const AirspeedCalRecord *record = NULL;
  uint32_t record_addr = 0U;
  if (!cal_find_latest_record(&record, &record_addr)) {
    return;
  }

  (void)record_addr;
  diff_offset_pa = record->diff_offset_pa;
  offset_valid = true;
}

static void startup_cal_reset(uint32_t now) {
  startup_cal_start_ms = now;
  startup_cal_count = 0U;
  startup_cal_diff_sum = 0.0f;
  startup_cal_diff_sq_sum = 0.0f;
  startup_cal_baro1_sum = 0.0f;
  startup_cal_baro2_sum = 0.0f;
  startup_cal_temp_sum = 0.0f;
}

static void startup_cal_add_sample(float signed_diff, float baro1_pa, float baro2_pa, float temp_c) {
  startup_cal_count++;
  startup_cal_diff_sum += signed_diff;
  startup_cal_diff_sq_sum += signed_diff * signed_diff;
  startup_cal_baro1_sum += baro1_pa;
  startup_cal_baro2_sum += baro2_pa;
  startup_cal_temp_sum += temp_c;
}

static bool startup_cal_finish_if_ready(uint32_t now) {
  if (startup_cal_start_ms == 0U) {
    startup_cal_reset(now);
    return false;
  }

  if ((now - startup_cal_start_ms) < CAL_CAPTURE_MS || startup_cal_count < CAL_MIN_SAMPLES) {
    return false;
  }

  const float count = (float)startup_cal_count;
  const float mean = startup_cal_diff_sum / count;
  float variance = (startup_cal_diff_sq_sum / count) - (mean * mean);
  if (variance < 0.0f) {
    variance = 0.0f;
  }
  const float stddev = sqrt_approx(variance);

  if (stddev > CAL_MAX_STDDEV_PA) {
    startup_cal_reset(now);
    return false;
  }

  diff_offset_pa = mean;
  offset_valid = true;
  const bool saved = cal_save_record(mean,
                                     startup_cal_baro1_sum / count,
                                     startup_cal_baro2_sum / count,
                                     startup_cal_temp_sum / count,
                                     stddev,
                                     startup_cal_count);
  if (!saved) {
    startup_cal_reset(now);
  }
  return offset_valid;
}

static bool i2c_mem_read(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t reg, uint8_t *data, uint16_t len) {
  const uint16_t dev = (uint16_t)address << 1;
  if (HAL_I2C_Master_Transmit(i2c, dev, &reg, 1, 20) != HAL_OK) {
    return false;
  }
  return HAL_I2C_Master_Receive(i2c, dev, data, len, 20) == HAL_OK;
}

static bool i2c_write8(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t reg, uint8_t value) {
  const uint8_t data[2] = {reg, value};
  return HAL_I2C_Master_Transmit(i2c, (uint16_t)address << 1, (uint8_t *)data, sizeof(data), 20) == HAL_OK;
}

static bool i2c_read8(I2C_HandleTypeDef *i2c, uint8_t address, uint8_t reg, uint8_t *value) {
  return i2c_mem_read(i2c, address, reg, value, 1);
}

static void debug_led_init(void) {
#if DEBUG_LED_ENABLED
#if DEBUG_LED_GPIO_TEST
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = DEBUG_LED_GPIO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DEBUG_LED_GPIO_PORT, &gpio);
  DEBUG_LED_GPIO_PORT->BRR = DEBUG_LED_GPIO_PIN;
#else
  ws2812_init();
#endif
#endif
}


static void set_debug_led(uint8_t red, uint8_t green, uint8_t blue) {
#if DEBUG_LED_ENABLED
  ws2812_write_rgb(red, green, blue);
#else
  (void)red;
  (void)green;
  (void)blue;
#endif
}

#if DEBUG_LED_ENABLED && DEBUG_LED_TEST
static void run_debug_led_test(void) {
  while (1) {
    set_debug_led(0, 0, 0);
    HAL_Delay(1000);
    set_debug_led(24, 0, 0);
    HAL_Delay(1000);
    set_debug_led(0, 24, 0);
    HAL_Delay(1000);
    set_debug_led(0, 0, 24);
    HAL_Delay(1000);
  }
}
#endif

#if DEBUG_LED_ENABLED && DEBUG_LED_GPIO_TEST
static void run_debug_led_gpio_test(void) {
  while (1) {
    DEBUG_LED_GPIO_PORT->BRR = DEBUG_LED_GPIO_PIN;
    HAL_Delay(1000);
    DEBUG_LED_GPIO_PORT->BSRR = DEBUG_LED_GPIO_PIN;
    HAL_Delay(1000);
  }
}
#endif

static bool spa06_wait_ready(Spa06 *baro, uint8_t mask, uint32_t timeout_ms) {
  const uint32_t start = HAL_GetTick();
  do {
    uint8_t status = 0;
    if (i2c_read8(&hi2c2, baro->address, SPA06_REG_MEASURE_CONFIG, &status) && ((status & mask) == mask)) {
      return true;
    }
    HAL_Delay(5);
  } while ((HAL_GetTick() - start) < timeout_ms);

  return false;
}

static bool spa06_read_coefficients(Spa06 *baro) {
  uint8_t data[21] = {0};
  if (!i2c_mem_read(&hi2c2, baro->address, SPA06_REG_COEF, data, sizeof(data))) {
    return false;
  }

  baro->c0 = sign_extend_12(((uint16_t)data[0] << 4) | (data[1] >> 4));
  baro->c1 = sign_extend_12(((uint16_t)(data[1] & 0x0F) << 8) | data[2]);
  baro->c00 = sign_extend_20(((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4));
  baro->c10 = sign_extend_20(((uint32_t)(data[5] & 0x0F) << 16) | ((uint32_t)data[6] << 8) | data[7]);
  baro->c01 = (int16_t)(((uint16_t)data[8] << 8) | data[9]);
  baro->c11 = (int16_t)(((uint16_t)data[10] << 8) | data[11]);
  baro->c20 = (int16_t)(((uint16_t)data[12] << 8) | data[13]);
  baro->c21 = (int16_t)(((uint16_t)data[14] << 8) | data[15]);
  baro->c30 = (int16_t)(((uint16_t)data[16] << 8) | data[17]);
  baro->c31 = sign_extend_12(((uint16_t)data[18] << 4) | (data[19] >> 4));
  baro->c40 = sign_extend_12(((uint16_t)(data[19] & 0x0F) << 8) | data[20]);
  return true;
}

static bool spa06_begin(Spa06 *baro, uint8_t address) {
  uint8_t id = 0;
  baro->address = address;
  baro->online = false;
  baro->valid = false;

  if (!i2c_read8(&hi2c2, baro->address, SPA06_REG_ID, &id) || id != SPA06_EXPECTED_ID) {
    return false;
  }
  if (!i2c_write8(&hi2c2, baro->address, SPA06_REG_RESET, 0x09)) {
    return false;
  }
  HAL_Delay(10);
  if (!spa06_wait_ready(baro, 0xC0, 1000)) {
    return false;
  }
  if (!spa06_read_coefficients(baro)) {
    return false;
  }

  const uint8_t config = (uint8_t)((SPA06_RATE_16HZ << 4) | SPA06_OVERSAMPLING_8X);
  if (!i2c_write8(&hi2c2, baro->address, SPA06_REG_PRESSURE_CONFIG, config)) {
    return false;
  }
  if (!i2c_write8(&hi2c2, baro->address, SPA06_REG_TEMPERATURE_CONFIG, config)) {
    return false;
  }
  if (!i2c_write8(&hi2c2, baro->address, SPA06_REG_CONFIG, 0x00)) {
    return false;
  }
  if (!i2c_write8(&hi2c2, baro->address, SPA06_REG_MEASURE_CONFIG, 0x07)) {
    return false;
  }

  baro->online = true;
  return true;
}

static bool spa06_read(Spa06 *baro) {
  uint8_t status = 0;
  uint8_t data[6] = {0};
  if (!baro->online) {
    return false;
  }
  if (!i2c_read8(&hi2c2, baro->address, SPA06_REG_MEASURE_CONFIG, &status) || ((status & 0x30U) != 0x30U)) {
    return false;
  }
  if (!i2c_mem_read(&hi2c2, baro->address, SPA06_REG_PRESSURE, data, sizeof(data))) {
    return false;
  }

  const int32_t raw_pressure = sign_extend_24(((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2]);
  const int32_t raw_temperature = sign_extend_24(((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5]);
  const float t_raw_sc = (float)raw_temperature / SPA06_SCALE_8X;
  const float p_raw_sc = (float)raw_pressure / SPA06_SCALE_8X;
  const float p2 = p_raw_sc * p_raw_sc;
  const float p3 = p2 * p_raw_sc;
  const float p4 = p3 * p_raw_sc;

  baro->temperature_c = (float)baro->c0 * 0.5f + (float)baro->c1 * t_raw_sc;
  baro->pressure_pa = (float)baro->c00 + (float)baro->c10 * p_raw_sc + (float)baro->c20 * p2 +
                      (float)baro->c30 * p3 + (float)baro->c40 * p4 +
                      t_raw_sc * ((float)baro->c01 + (float)baro->c11 * p_raw_sc +
                                  (float)baro->c21 * p2 + (float)baro->c31 * p3);
  baro->updated_ms = HAL_GetTick();
  baro->valid = true;
  return true;
}

static void update_response_frame(void) {
  const float pressure_psi = pressure_pa / 6894.757f;
  const uint16_t pressure_raw =
      (uint16_t)clamp_i16((0.5f * 16383.0f) - (pressure_psi * 0.4f * 16383.0f / MS4525_PSI_RANGE), 1, 0x3FFE);
  const uint16_t temperature_raw =
      (uint16_t)clamp_i16(((temperature_c + 50.0f) * 2047.0f) / 200.0f, 1, 0x07FE);

  __disable_irq();
  response_frame[0] = (uint8_t)((pressure_raw >> 8) & 0x3FU);
  response_frame[1] = (uint8_t)(pressure_raw & 0xFFU);
  response_frame[2] = (uint8_t)((temperature_raw >> 3) & 0xFFU);
  response_frame[3] = (uint8_t)((temperature_raw & 0x07U) << 5);
  __enable_irq();
}

#if VIRTUAL_DPS310_ENABLED
static void dps310_reset_registers(void) {
  dps310_pressure_config = 0;
  dps310_temperature_config = 0;
  dps310_measure_config = 0;
  dps310_config = 0;
  dps310_reset = 0;
}

static uint8_t dps310_read_register(uint8_t reg) {
  if (reg < DPS310_DATA_LEN) {
    return dps310_data_frame[reg];
  }

  if (reg >= DPS310_REG_COEF && reg < (DPS310_REG_COEF + DPS310_COEF_LEN)) {
    return dps310_coefficients[reg - DPS310_REG_COEF];
  }

  switch (reg) {
    case DPS310_REG_PRESSURE_CONFIG:
      return dps310_pressure_config;
    case DPS310_REG_TEMPERATURE_CONFIG:
      return dps310_temperature_config;
    case DPS310_REG_MEASURE_CONFIG:
      return (uint8_t)(DPS310_READY_STATUS | (dps310_measure_config & DPS310_MEAS_CTRL_MASK));
    case DPS310_REG_CONFIG:
      return dps310_config;
    case DPS310_REG_RESET:
      return dps310_reset;
    case DPS310_REG_ID:
      return DPS310_EXPECTED_ID;
    case DPS310_REG_COEF_SRCE:
      return DPS310_COEF_SRCE_TMP_EXT;
    default:
      return 0;
  }
}

static void dps310_write_register(uint8_t reg, uint8_t value) {
  switch (reg) {
    case DPS310_REG_PRESSURE_CONFIG:
      dps310_pressure_config = value;
      break;
    case DPS310_REG_TEMPERATURE_CONFIG:
      dps310_temperature_config = value;
      break;
    case DPS310_REG_MEASURE_CONFIG:
      dps310_measure_config = (uint8_t)(value & DPS310_MEAS_CTRL_MASK);
      break;
    case DPS310_REG_CONFIG:
      dps310_config = value;
      break;
    case DPS310_REG_RESET:
      dps310_reset = value;
      if (value == DPS310_RESET_COMMAND) {
        dps310_reset_registers();
      }
      break;
    default:
      break;
  }
}

static void update_static_pressure_source(void) {
  if (static_pressure_source != STATIC_SOURCE_UNKNOWN || !baro1.valid || !baro2.valid) {
    return;
  }

#if BARO_AUTOZERO
  if (!offset_valid) {
    static_pressure_candidate = STATIC_SOURCE_UNKNOWN;
    static_pressure_candidate_count = 0;
    return;
  }
#endif

  const float raw_diff_pa = baro1.pressure_pa - baro2.pressure_pa;
  const float raw_offset_pa = diff_offset_pa / (float)BARO_DIFF_SIGN;
  const float corrected_diff_pa = raw_diff_pa - raw_offset_pa;
  if (abs_float(corrected_diff_pa) < STATIC_DETECT_MIN_DIFF_PA) {
    static_pressure_candidate = STATIC_SOURCE_UNKNOWN;
    static_pressure_candidate_count = 0;
    return;
  }

  const uint8_t candidate = (corrected_diff_pa <= 0.0f) ? STATIC_SOURCE_BARO1 : STATIC_SOURCE_BARO2;
  if (candidate == static_pressure_candidate) {
    static_pressure_candidate_count++;
  } else {
    static_pressure_candidate = candidate;
    static_pressure_candidate_count = 1;
  }

  if (static_pressure_candidate_count >= STATIC_DETECT_LOCK_SAMPLES) {
    static_pressure_source = static_pressure_candidate;
  }
}

static float virtual_static_pressure_pa(void) {
  if (static_pressure_source == STATIC_SOURCE_BARO1 && baro1.valid) {
    return baro1.pressure_pa;
  }
  if (static_pressure_source == STATIC_SOURCE_BARO2 && baro2.valid) {
    return baro2.pressure_pa;
  }
  if (baro1.valid && baro2.valid) {
    return (baro1.pressure_pa + baro2.pressure_pa) * 0.5f;
  }
  if (baro1.valid) {
    return baro1.pressure_pa;
  }
  if (baro2.valid) {
    return baro2.pressure_pa;
  }
  return DPS310_SYNTH_C00_PA;
}

static float virtual_static_temperature_c(void) {
  if (static_pressure_source == STATIC_SOURCE_BARO1 && baro1.valid) {
    return baro1.temperature_c;
  }
  if (static_pressure_source == STATIC_SOURCE_BARO2 && baro2.valid) {
    return baro2.temperature_c;
  }
  if (baro1.valid && baro2.valid) {
    return (baro1.temperature_c + baro2.temperature_c) * 0.5f;
  }
  if (baro1.valid) {
    return baro1.temperature_c;
  }
  if (baro2.valid) {
    return baro2.temperature_c;
  }
  return 25.0f;
}

static void update_virtual_dps310_frame(void) {
  uint8_t frame[DPS310_DATA_LEN] = {0};
  const float static_pressure_pa = virtual_static_pressure_pa();
  const float static_temperature_c = virtual_static_temperature_c();
  const int32_t pressure_raw =
      clamp_i32(((static_pressure_pa - DPS310_SYNTH_C00_PA) / DPS310_SYNTH_C10_PA) * DPS310_SCALE_16X,
                -8388608, 8388607);
  const int32_t temperature_raw =
      clamp_i32((static_temperature_c / DPS310_SYNTH_C1_TEMP) * DPS310_SCALE_16X, -8388608, 8388607);
  const uint32_t pressure_bits = (uint32_t)pressure_raw & 0x00FFFFFFU;
  const uint32_t temperature_bits = (uint32_t)temperature_raw & 0x00FFFFFFU;

  frame[0] = (uint8_t)(pressure_bits >> 16);
  frame[1] = (uint8_t)(pressure_bits >> 8);
  frame[2] = (uint8_t)pressure_bits;
  frame[3] = (uint8_t)(temperature_bits >> 16);
  frame[4] = (uint8_t)(temperature_bits >> 8);
  frame[5] = (uint8_t)temperature_bits;

  __disable_irq();
  for (uint32_t i = 0; i < DPS310_DATA_LEN; i++) {
    dps310_data_frame[i] = frame[i];
  }
  __enable_irq();
}
#endif

static void update_barometers(void) {
  static uint32_t last_sample_ms = 0;
  const uint32_t now = HAL_GetTick();
  if ((now - last_sample_ms) < 50U) {
    return;
  }
  last_sample_ms = now;

  if (!spa06_read(&baro1) || !spa06_read(&baro2)) {
    pressure_pa = 0.0f;
    update_response_frame();
#if VIRTUAL_DPS310_ENABLED
    update_virtual_dps310_frame();
#endif
    return;
  }

#if VIRTUAL_DPS310_ENABLED
  update_static_pressure_source();
  update_virtual_dps310_frame();
#endif

  const float signed_diff = (baro1.pressure_pa - baro2.pressure_pa) * (float)BARO_DIFF_SIGN;
#if BARO_AUTOZERO
  if (!offset_valid) {
    const float avg_temp_c = (baro1.temperature_c + baro2.temperature_c) * 0.5f;
    startup_cal_add_sample(signed_diff, baro1.pressure_pa, baro2.pressure_pa, avg_temp_c);
    if (!startup_cal_finish_if_ready(now)) {
      pressure_pa = 0.0f;
      temperature_c = avg_temp_c;
      update_response_frame();
      return;
    }
  }
#endif
  pressure_pa = signed_diff - diff_offset_pa;
  temperature_c = (baro1.temperature_c + baro2.temperature_c) * 0.5f;
  update_response_frame();
}

static void update_debug_led(void) {
#if DEBUG_LED_ENABLED
  const uint32_t now = HAL_GetTick();
  if ((now - last_led_update_ms) < 100U) {
    return;
  }
  last_led_update_ms = now;

  const bool request_seen = request_count != 0U;
  const bool request_pulse = request_count != last_led_request_count;
  if (request_pulse) {
    last_fc_request_ms = now;
    if ((now - last_fc_flash_ms) >= 1000U) {
      last_fc_flash_ms = now;
      fc_flash_until_ms = now + 150U;
    }
  }
  last_led_request_count = request_count;
  const bool recent_request = request_seen && ((now - last_fc_request_ms) < 2000U);
  const bool led_on = ((now / 500U) % 2U) == 0U;
  const bool fc_flash_active = (int32_t)(fc_flash_until_ms - now) > 0;

  uint8_t red = 0;
  uint8_t green = 0;
  uint8_t blue = 0;

  if (!recent_request) {
    red = led_on ? 10U : 0U;
    blue = led_on ? 24U : 0U;
  } else if (baro1.valid && baro2.valid) {
    green = led_on ? 24U : 2U;
  } else if (baro1.online || baro2.online) {
    red = led_on ? 28U : 4U;
    green = led_on ? 8U : 0U;
  } else {
    red = led_on ? 28U : 4U;
  }

  if (fc_flash_active) {
    red = 0U;
    green = 0U;
    blue = 24U;
  }

  set_debug_led(red, green, blue);
#endif
}

static void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSIDiv = RCC_HSI_DIV1;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = RCC_PLLM_DIV1;
  osc.PLL.PLLN = 8;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = RCC_PLLQ_DIV2;
  osc.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    while (1) {
    }
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
    while (1) {
    }
  }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = I2C_TIMING_100KHZ_64MHZ;
  hi2c1.Init.OwnAddress1 = (uint32_t)I2C_SLAVE_ADDRESS << 1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
#if VIRTUAL_DPS310_ENABLED
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_ENABLE;
  hi2c1.Init.OwnAddress2 = (uint32_t)VIRTUAL_DPS310_I2C_ADDRESS << 1;
#else
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
#endif
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    while (1) {
    }
  }
  HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
}

static void MX_I2C2_Init(void) {
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = I2C_TIMING_100KHZ_64MHZ;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    while (1) {
    }
  }
  HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  if (hi2c->Instance == I2C1) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_NVIC_SetPriority(I2C1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(I2C1_IRQn);
  } else if (hi2c->Instance == I2C2) {
    __HAL_RCC_I2C2_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF6_I2C2;
    HAL_GPIO_Init(GPIOA, &gpio);
  }
}

static void i2c1_slave_irq_enable(void) {
  I2C1->ICR = I2C_ICR_ADDRCF | I2C_ICR_NACKCF | I2C_ICR_STOPCF | I2C_ICR_BERRCF | I2C_ICR_ARLOCF | I2C_ICR_OVRCF;
  I2C1->CR1 |= I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_NACKIE | I2C_CR1_STOPIE | I2C_CR1_ERRIE;
}

static void i2c1_handle_address_match(uint32_t isr) {
  const uint16_t addr_match = (uint16_t)((isr & I2C_ISR_ADDCODE) >> 16U);
  const bool master_read = (isr & I2C_ISR_DIR) != 0U;
  i2c1_active_slave = I2C1_SLAVE_NONE;

  if (addr_match == ((uint16_t)I2C_SLAVE_ADDRESS << 1)) {
    i2c1_active_slave = I2C1_SLAVE_MS4525;
  }
#if VIRTUAL_DPS310_ENABLED
  else if (addr_match == ((uint16_t)VIRTUAL_DPS310_I2C_ADDRESS << 1)) {
    i2c1_active_slave = I2C1_SLAVE_DPS310;
  }
#endif

  if (master_read) {
    if (i2c1_active_slave == I2C1_SLAVE_MS4525) {
      for (uint32_t i = 0; i < MS4525_FRAME_LEN; i++) {
        tx_frame[i] = response_frame[i];
      }
      i2c1_ms4525_tx_index = 0;
    }
    if (i2c1_active_slave != I2C1_SLAVE_NONE) {
      request_count++;
    }
  } else {
#if VIRTUAL_DPS310_ENABLED
    if (i2c1_active_slave == I2C1_SLAVE_DPS310) {
      dps310_rx_count = 0;
    }
#endif
    if (i2c1_active_slave != I2C1_SLAVE_NONE) {
      receive_count++;
    }
  }

  I2C1->ICR = I2C_ICR_ADDRCF;
}

static void i2c1_handle_rx_byte(uint8_t value) {
#if VIRTUAL_DPS310_ENABLED
  if (i2c1_active_slave == I2C1_SLAVE_DPS310) {
    if (dps310_rx_count == 0U) {
      dps310_reg_pointer = value;
      dps310_rx_count = 1U;
    } else {
      dps310_write_register(dps310_reg_pointer, value);
      dps310_reg_pointer++;
      dps310_rx_count++;
    }
  } else
#endif
  {
    (void)value;
  }
}

static uint8_t i2c1_next_tx_byte(void) {
  if (i2c1_active_slave == I2C1_SLAVE_MS4525) {
    if (i2c1_ms4525_tx_index < MS4525_FRAME_LEN) {
      return tx_frame[i2c1_ms4525_tx_index++];
    }
    return 0xFFU;
  }

#if VIRTUAL_DPS310_ENABLED
  if (i2c1_active_slave == I2C1_SLAVE_DPS310) {
    const uint8_t value = dps310_read_register(dps310_reg_pointer);
    dps310_reg_pointer++;
    return value;
  }
#endif

  return 0xFFU;
}

void I2C1_IRQHandler(void) {
  uint32_t isr = I2C1->ISR;

  if ((isr & I2C_ISR_ADDR) != 0U) {
    i2c1_handle_address_match(isr);
    isr = I2C1->ISR;
  }

  if ((isr & I2C_ISR_RXNE) != 0U) {
    i2c1_handle_rx_byte((uint8_t)I2C1->RXDR);
    isr = I2C1->ISR;
  }

  if ((isr & I2C_ISR_TXIS) != 0U) {
    I2C1->TXDR = i2c1_next_tx_byte();
    isr = I2C1->ISR;
  }

  uint32_t clear_flags = 0;
  if ((isr & I2C_ISR_NACKF) != 0U) {
    clear_flags |= I2C_ICR_NACKCF;
  }
  if ((isr & I2C_ISR_STOPF) != 0U) {
    clear_flags |= I2C_ICR_STOPCF;
    i2c1_active_slave = I2C1_SLAVE_NONE;
  }
  if ((isr & I2C_ISR_BERR) != 0U) {
    clear_flags |= I2C_ICR_BERRCF;
  }
  if ((isr & I2C_ISR_ARLO) != 0U) {
    clear_flags |= I2C_ICR_ARLOCF;
  }
  if ((isr & I2C_ISR_OVR) != 0U) {
    clear_flags |= I2C_ICR_OVRCF;
  }
  if (clear_flags != 0U) {
    I2C1->ICR = clear_flags;
  }
}

void I2C2_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler(&hi2c2);
  HAL_I2C_ER_IRQHandler(&hi2c2);
}

void SysTick_Handler(void) {
  HAL_IncTick();
}

int _close(int file) {
  (void)file;
  return -1;
}

int _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

int _read(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len) {
  (void)file;
  (void)ptr;
  return len;
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  debug_led_init();
#if DEBUG_LED_GPIO_TEST
  run_debug_led_gpio_test();
#endif
#if DEBUG_LED_TEST
  run_debug_led_test();
#endif
  set_debug_led(0, 0, 8);
  MX_I2C2_Init();

  baro1.online = spa06_begin(&baro1, BARO1_I2C_ADDRESS);
  baro2.online = spa06_begin(&baro2, BARO2_I2C_ADDRESS);
  cal_load_flash_offset();
  update_response_frame();
#if VIRTUAL_DPS310_ENABLED
  update_virtual_dps310_frame();
#endif

  MX_I2C1_Init();
  i2c1_slave_irq_enable();

  while (1) {
    update_barometers();
    update_debug_led();
  }
}
