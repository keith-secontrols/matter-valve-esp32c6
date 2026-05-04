#pragma once

#include <esp_err.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// AS5600 magnetic angle sensor driver
// I2C address: 0x36 (fixed)
// Reads the 12-bit raw angle register (0-4095 = 0-360 degrees)

#define AS5600_I2C_ADDR   0x36
#define AS5600_REG_ANGLE  0x0E   // RAW_ANGLE high byte (followed by low byte)

typedef struct {
    int      i2c_port;   // I2C port number (e.g. I2C_NUM_0)
    int      sda_gpio;
    int      scl_gpio;
    uint32_t clk_hz;     // I2C clock, e.g. 100000
} as5600_config_t;

// Initialise I2C and verify the sensor is present
esp_err_t as5600_init(const as5600_config_t *cfg);

// Read raw 12-bit angle (0-4095)
esp_err_t as5600_read_raw(uint16_t *raw_out);

// Read angle in degrees (0.0 - 359.9)
esp_err_t as5600_read_degrees(float *degrees_out);

#ifdef __cplusplus
}
#endif
