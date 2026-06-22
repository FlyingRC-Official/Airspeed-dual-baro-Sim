#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#ifndef DEBUG_LED_PIN
#define DEBUG_LED_PIN PA8
#endif

Adafruit_NeoPixel pixel(1, DEBUG_LED_PIN, NEO_GRBW + NEO_KHZ800);

static void show(uint8_t red, uint8_t green, uint8_t blue, uint8_t white = 0) {
  pixel.clear();
  pixel.setPixelColor(0, pixel.Color(red, green, blue, white));
  pixel.show();
}

void setup() {
  pinMode(DEBUG_LED_PIN, OUTPUT);
  digitalWrite(DEBUG_LED_PIN, LOW);
  delay(100);
  pixel.begin();
  pixel.setBrightness(24);
  show(0, 0, 0, 0);
}

void loop() {
  show(0, 0, 0, 0);
  delay(1000);
  show(255, 0, 0, 0);
  delay(1000);
  show(0, 255, 0, 0);
  delay(1000);
  show(0, 0, 255, 0);
  delay(1000);
}

