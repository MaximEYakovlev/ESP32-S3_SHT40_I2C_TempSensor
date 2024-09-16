#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"

#define I2C_MASTER_SCL_IO           2    
#define I2C_MASTER_SDA_IO           3    
#define I2C_MASTER_NUM              I2C_NUM_0 
#define I2C_MASTER_FREQ_HZ          100000   
#define I2C_MASTER_TX_BUF_DISABLE   0    
#define I2C_MASTER_RX_BUF_DISABLE   0     
#define I2C_MASTER_TIMEOUT_MS       1000

#define SHT40_SENSOR_ADDR           0x44

#define SHT40_MEASURE_CMD           0xFD

static const char *TAG = "SHT40_APP";

static esp_err_t i2c_master_init(void) {
    int i2c_master_port = I2C_MASTER_NUM;
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C configuration failed: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(err));
    }

    return err;
}

static esp_err_t sht40_read_data(float *temperature) {
    uint8_t data[6];
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SHT40_MEASURE_CMD, true);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send measurement command: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT40_SENSOR_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, sizeof(data), I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data from sensor: %s", esp_err_to_name(err));
        return err;
    }

    uint16_t temp_raw = (data[0] << 8) | data[1];
    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);

    return ESP_OK;
}

void app_main(void) {
    ESP_LOGI(TAG, "Initializing I2C...");
    esp_err_t err = i2c_master_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed. Exiting...");
        return;
    }

    ESP_LOGI(TAG, "SHT40 initialized successfully");

    while (1) {
        float temperature;
        err = sht40_read_data(&temperature);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature data");
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
