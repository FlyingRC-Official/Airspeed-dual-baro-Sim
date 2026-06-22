#include "ws2812.h"

#include "stm32g0xx_hal.h"
#include "stm32g0xx_ll_gpio.h"

#define WS2812_GPIO_PORT GPIOA
#define WS2812_GPIO_PIN GPIO_PIN_8

void ws2812_init(void) {
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = WS2812_GPIO_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(WS2812_GPIO_PORT, &gpio);
  WS2812_GPIO_PORT->BRR = WS2812_GPIO_PIN;
}

void ws2812_write_rgb(uint8_t red, uint8_t green, uint8_t blue) {
  const uint8_t pixels[] = {green, red, blue, 0};
  HAL_Delay(1);

  const uint32_t hclk_hz = HAL_RCC_GetHCLKFreq();
  const uint32_t bit_ticks = hclk_hz / 800000U;
  const uint32_t zero_threshold = bit_ticks - (hclk_hz / 2500000U);
  const uint32_t one_threshold = bit_ticks - (hclk_hz / 1250000U);
  const uint32_t save_load = SysTick->LOAD;
  const uint32_t save_val = SysTick->VAL;

  __disable_irq();
  SysTick->LOAD = bit_ticks - 1U;
  SysTick->VAL = 0U;

  const uint8_t *ptr = pixels;
  const uint8_t *end = pixels + sizeof(pixels);
  uint8_t value = *ptr++;
  uint8_t mask = 0x80U;
  for (;;) {
    LL_GPIO_SetOutputPin(WS2812_GPIO_PORT, WS2812_GPIO_PIN);
    const uint32_t threshold = (value & mask) ? one_threshold : zero_threshold;
    while (SysTick->VAL > threshold) {
    }
    LL_GPIO_ResetOutputPin(WS2812_GPIO_PORT, WS2812_GPIO_PIN);
    if ((mask >>= 1U) == 0U) {
      if (ptr >= end) {
        break;
      }
      value = *ptr++;
      mask = 0x80U;
    }
    while (SysTick->VAL <= threshold) {
    }
  }

  WS2812_GPIO_PORT->BRR = WS2812_GPIO_PIN;
  SysTick->LOAD = save_load;
  SysTick->VAL = save_val;
  __enable_irq();

  HAL_Delay(1);
}
