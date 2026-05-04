#include "as5600.h"

#include <driver/i2c_master.h>
#include <esp_log.h>

static const char *TAG = "as5600";

static i2c_master_bus_handle_t  s_bus    = NULL;
static i2c_master_dev_handle_t  s_device = NULL;

esp_err_t as5600_init(const as5600_config_t *cfg)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port        = cfg->i2c_port,
        .sda_io_num      = cfg->sda_gpio,
        .scl_io_num      = cfg->scl_gpio,
        .clk_source      = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %d", err);
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = AS5600_I2C_ADDR,
        .scl_speed_hz    = cfg->clk_hz,
    };
    err = i2c_master_bus_add_device(s_bus, &dev_cfg, &s_device);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add AS5600 device: %d", err);
        return err;
    }

    // Probe to confirm sensor is present
    err = i2c_master_probe(s_bus, AS5600_I2C_ADDR, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "AS5600 not found at 0x%02X", AS5600_I2C_ADDR);
        return err;
    }

    ESP_LOGI(TAG, "AS5600 found on SDA=%d SCL=%d", cfg->sda_gpio, cfg->scl_gpio);
    return ESP_OK;
}

esp_err_t as5600_read_raw(uint16_t *raw_out)
{
    uint8_t reg = AS5600_REG_ANGLE;
    uint8_t buf[2];

    esp_err_t err = i2c_master_transmit_receive(s_device, &reg, 1, buf, 2, 100);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %d", err);
        return err;
    }

    *raw_out = ((uint16_t)(buf[0] & 0x0F) << 8) | buf[1];
    return ESP_OK;
}

esp_err_t as5600_read_degrees(float *degrees_out)
{
    uint16_t raw;
    esp_err_t err = as5600_read_raw(&raw);
    if (err != ESP_OK) return err;

    *degrees_out = raw * 360.0f / 4096.0f;
    return ESP_OK;
}
