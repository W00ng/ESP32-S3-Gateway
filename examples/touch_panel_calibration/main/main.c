/* 
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "i2c_bus.h"
#include "bsp_i2c.h"
#include "bsp_lcd.h"
#include "ns2009.h"


static const char *TAG = "main";

void led_task(void *arg)
{
    gpio_config_t led_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << GPIO_LED_STATUS
    };
    gpio_config(&led_gpio_config);

    while (1)
    {
        gpio_set_level(GPIO_LED_STATUS, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        gpio_set_level(GPIO_LED_STATUS, 0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* Initialize I2C 400KHz */
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));

    /* LCD init */
    ESP_ERROR_CHECK(bsp_lcd_init());

    /* Touch IC init */
    ESP_ERROR_CHECK(ns2009_init());

    ESP_LOGI(TAG, "main init done");

    xTaskCreate(led_task, "led_task", 1024 * 2, NULL, 2, NULL);

    touch_panel_calibration();

    uint16_t x, y;
    while (1)
    {
        if(ESP_OK == ns2009_get_pos(&x, &y)){
            lcd_draw_point(lcd_panel, x, y, COLOR_BLACK);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}
