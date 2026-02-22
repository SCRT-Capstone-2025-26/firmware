#ifndef LED_H
#define LED_H

#include <Arduino.h>

struct RGB {
    uint8_t r, g, b;

    RGB();
    RGB(uint8_t r, uint8_t g, uint8_t b);
};

enum LEDs {
    LED_SD       = 0,
    LED_RADIO    = 1,
    LED_IMU      = 2,
    LED_BARO     = 3,
    // Watchdog used is the MAGN LED
    // This code doesn't use the MAGN though
    LED_WATCHDOG = 4,
    LED_STATUS   = 5
};

#define LED_COUNT 6

extern RGB leds[LED_COUNT];

void led_init();

void led_show();

void __not_in_flash_func(write_leds)(RGB* values, int count);

#endif

