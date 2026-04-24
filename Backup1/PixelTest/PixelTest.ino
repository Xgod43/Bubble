#include <FastLED.h>

#define LED_PIN     6
#define NUM_LEDS    4
#define BRIGHTNESS  10
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

CRGB leds[NUM_LEDS];

void fillAll(const CRGB& c) {
  for (int i = 0; i < NUM_LEDS; i++) leds[i] = c;
  FastLED.show();
}

void setup() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  fillAll(CRGB::Black);
}

void loop() {
  //fillAll(CRGB::Red);   delay(500);
  //fillAll(CRGB::Green); delay(500);
  //fillAll(CRGB::Blue);  delay(500);
  fillAll(CRGB::White); delay(500);
  //fillAll(CRGB::Black); delay(500);
}
