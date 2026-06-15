#include "stm32g0xx_hal.h"

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

#ifndef DEBUG_LED_ENABLED
#define DEBUG_LED_ENABLED 0
#endif

#ifndef DEBUG_LED_GPIO_PORT
#define DEBUG_LED_GPIO_PORT GPIOA
#endif

#ifndef DEBUG_LED_GPIO_PIN
#define DEBUG_LED_GPIO_PIN GPIO_PIN_8
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

#define I2C_TIMING_100KHZ_16MHZ 0x00303D5BU
#define MS4525_FRAME_LEN 4U

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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

static Spa06 baro1 = {.address = BARO1_I2C_ADDRESS};
static Spa06 baro2 = {.address = BARO2_I2C_ADDRESS};

static volatile uint8_t response_frame[MS4525_FRAME_LEN] = {0};
static uint8_t tx_frame[MS4525_FRAME_LEN] = {0};
static uint8_t rx_byte = 0;
static volatile uint32_t request_count = 0;
static volatile uint32_t receive_count = 0;

static float pressure_pa = 0.0f;
static float temperature_c = 25.0f;
static float diff_offset_pa = 0.0f;
static bool offset_valid = false;
static uint32_t last_led_update_ms = 0;
static uint32_t last_fc_request_ms = 0;
static uint32_t last_led_request_count = 0;

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
  GPIO_InitTypeDef gpio = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = DEBUG_LED_GPIO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DEBUG_LED_GPIO_PORT, &gpio);
  DEBUG_LED_GPIO_PORT->BRR = DEBUG_LED_GPIO_PIN;
#endif
}

static void ws2812_delay(uint32_t cycles) {
  while (cycles--) {
    __NOP();
  }
}

static void ws2812_send_bit(bool one) {
#if DEBUG_LED_ENABLED
  DEBUG_LED_GPIO_PORT->BSRR = DEBUG_LED_GPIO_PIN;
  if (one) {
    ws2812_delay(7);
    DEBUG_LED_GPIO_PORT->BRR = DEBUG_LED_GPIO_PIN;
    ws2812_delay(4);
  } else {
    ws2812_delay(2);
    DEBUG_LED_GPIO_PORT->BRR = DEBUG_LED_GPIO_PIN;
    ws2812_delay(9);
  }
#else
  (void)one;
#endif
}

static void ws2812_send_byte(uint8_t value) {
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1) {
    ws2812_send_bit((value & mask) != 0);
  }
}

static void set_debug_led(uint8_t red, uint8_t green, uint8_t blue) {
#if DEBUG_LED_ENABLED
  __disable_irq();
  ws2812_send_byte(green);
  ws2812_send_byte(red);
  ws2812_send_byte(blue);
  __enable_irq();
#else
  (void)red;
  (void)green;
  (void)blue;
#endif
}

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
    return;
  }

  const float signed_diff = (baro1.pressure_pa - baro2.pressure_pa) * (float)BARO_DIFF_SIGN;
#if BARO_AUTOZERO
  if (!offset_valid) {
    diff_offset_pa = signed_diff;
    offset_valid = true;
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
  }
  last_led_request_count = request_count;
  const bool recent_request = request_seen && ((now - last_fc_request_ms) < 2000U);
  const bool led_on = ((now / 500U) % 2U) == 0U;
  const bool short_pulse = ((now / 100U) % 10U) == 0U;

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

  if (request_pulse || short_pulse) {
    blue = blue > 12U ? blue : 12U;
  }

  set_debug_led(red, green, blue);
#endif
}

static void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSIDiv = RCC_HSI_DIV1;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    while (1) {
    }
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) {
    while (1) {
    }
  }
}

static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = I2C_TIMING_100KHZ_16MHZ;
  hi2c1.Init.OwnAddress1 = (uint32_t)I2C_SLAVE_ADDRESS << 1;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
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
  hi2c2.Init.Timing = I2C_TIMING_100KHZ_16MHZ;
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

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t transfer_direction, uint16_t addr_match_code) {
  (void)addr_match_code;
  if (hi2c->Instance != I2C1) {
    return;
  }

  if (transfer_direction == I2C_DIRECTION_TRANSMIT) {
    receive_count++;
    HAL_I2C_Slave_Seq_Receive_IT(&hi2c1, &rx_byte, 1, I2C_FIRST_AND_LAST_FRAME);
  } else {
    __disable_irq();
    for (uint32_t i = 0; i < MS4525_FRAME_LEN; i++) {
      tx_frame[i] = response_frame[i];
    }
    __enable_irq();
    request_count++;
    HAL_I2C_Slave_Seq_Transmit_IT(&hi2c1, tx_frame, MS4525_FRAME_LEN, I2C_FIRST_AND_LAST_FRAME);
  }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    HAL_I2C_EnableListen_IT(&hi2c1);
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    HAL_I2C_EnableListen_IT(&hi2c1);
  }
}

void I2C1_IRQHandler(void) {
  HAL_I2C_EV_IRQHandler(&hi2c1);
  HAL_I2C_ER_IRQHandler(&hi2c1);
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
  set_debug_led(8, 8, 8);
  MX_I2C2_Init();

  baro1.online = spa06_begin(&baro1, BARO1_I2C_ADDRESS);
  baro2.online = spa06_begin(&baro2, BARO2_I2C_ADDRESS);
  update_response_frame();

  MX_I2C1_Init();
  HAL_I2C_EnableListen_IT(&hi2c1);

  while (1) {
    update_barometers();
    update_debug_led();
  }
}
