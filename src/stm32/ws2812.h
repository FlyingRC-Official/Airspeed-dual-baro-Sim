#pragma once

#include <stdint.h>

void ws2812_init(void);
void ws2812_write_rgb(uint8_t red, uint8_t green, uint8_t blue);

