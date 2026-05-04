#pragma once

// ---------------------------------------------------------------
// Pin definitions for ESP32-C6 Matter Valve Controller
// ---------------------------------------------------------------

// I2C -- AS5600 magnetic angle sensor
#define PIN_I2C_SDA     6
#define PIN_I2C_SCL     7

// WS2812 RGB LED (onboard)
#define PIN_LED_RGB     8

// Onboard BOOT button -- factory reset (active low)
#define PIN_BTN_BOOT    9

// ST7789 1.69" TFT (240x280) over SPI2
#define PIN_TFT_CLK     18
#define PIN_TFT_MOSI    19
#define PIN_TFT_CS      20
#define PIN_TFT_DC      21
#define PIN_TFT_RST     22
#define PIN_TFT_BL      23   // backlight, LEDC PWM

// H-bridge motor drive (two PWM channels, no separate enable)
#define PIN_MOTOR_IN1   4    // forward PWM
#define PIN_MOTOR_IN2   5    // reverse PWM

// Zero-position button (active low, internal pullup)
#define PIN_BTN_ZERO    3
