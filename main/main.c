#include <stdio.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "sht4x.h"

#define I2C_MASTER_SCL_IO           2    // SCL pin
#define I2C_MASTER_SDA_IO           3    // SDA pin
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

static const char *TAG = "SHT40_APP";

// I2C initialization
static void i2c_master_init() {
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret;

    // Initialize the I2C driver
    ret = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "I2C initialized successfully");
}

void app_main(void) {
    // Initialize I2C
    i2c_master_init();

    // Initialize SHT40 sensor
    sht4x_t sht40;
    esp_err_t ret = sht4x_init_desc(&sht40, I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "SHT40 initialized successfully");
    } else {
        ESP_LOGE(TAG, "SHT40 initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    // Read temperature from SHT40
    float temperature;
    while (1) {
        ret = sht4x_measure(&sht40, &temperature, NULL); // Read only temperature
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Temperature: %.2f °C", temperature);
        } else {
            ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1-second delay between readings
    }
}
