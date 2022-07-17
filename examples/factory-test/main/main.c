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
#include "bsp_sdcard.h"
// #include "esp_netif.h"
// #include "esp_eth.h"
// #include "esp_event.h"
// #include "driver/spi_master.h"

static const char *TAG = "main";

// #define ETH_SPI_HOST         (SPI2_HOST)
// #define ETH_SPI_CLOCK_MHZ    (12)
// static esp_eth_handle_t eth_handle = NULL;
// static void initialize_ethernet(void)
// {
//     // Install GPIO ISR handler to be able to service SPI Eth modlues interrupts
//     ESP_ERROR_CHECK(gpio_install_isr_service(0));

//     // Init SPI bus
//     spi_device_handle_t spi_handle = NULL;
//     spi_bus_config_t buscfg = {
//         .miso_io_num = GPIO_SPI_MISO,
//         .mosi_io_num = GPIO_SPI_MOSI,
//         .sclk_io_num = GPIO_SPI_CLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//     };
//     ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

//     // Configure SPI interface and Ethernet driver for specific SPI module
//     spi_eth_module_config_t spi_eth_module_config;
//     spi_eth_module_config.spi_cs_gpio = GPIO_SPI_CS;
//     spi_eth_module_config.int_gpio = GPIO_W5500_INT;
//     spi_eth_module_config.phy_reset_gpio = GPIO_W5500_RST;
//     spi_eth_module_config.phy_addr = 1;

//     spi_device_interface_config_t devcfg = {
//         .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
//         .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
//         .mode = 0,
//         .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
//         .spics_io_num = spi_eth_module_config.spi_cs_gpio,
//         .queue_size = 20
//     };
//     ESP_ERROR_CHECK(spi_bus_add_device(ETH_SPI_HOST, &devcfg, &spi_handle));

//     // w5500 ethernet driver is based on spi driver
//     eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);

//     // Set remaining GPIO numbers and configuration used by the SPI module
//     w5500_config.int_gpio_num = spi_eth_module_config.int_gpio;

//     // Init MAC and PHY configs to default
//     eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
//     eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
//     phy_config.phy_addr = spi_eth_module_config.phy_addr;
//     phy_config.reset_gpio_num = spi_eth_module_config.phy_reset_gpio;

//     esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
//     esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);
//     esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);

//     ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &eth_handle));

//     /* The SPI Ethernet module might not have a burned factory MAC address, we cat to set it manually.
//     02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
//     */
//     ESP_ERROR_CHECK(esp_eth_ioctl(eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
//         0x02, 0x00, 0x00, 0x12, 0x34, 0x56
//     }));

//     // Initialize TCP/IP network interface (should be called only once in application)
//     ESP_ERROR_CHECK(esp_netif_init());

//     // Create instance(s) of esp-netif for SPI Ethernet(s)
//     esp_netif_config_t netif_cfg = ESP_NETIF_DEFAULT_ETH();
//     esp_netif_t *eth_netif = esp_netif_new(&netif_cfg);
//     // attach Ethernet driver to TCP/IP stack
//     ESP_ERROR_CHECK(esp_netif_attach(eth_netif, esp_eth_new_netif_glue(eth_handle)));

//     // Register user defined event handers
//     ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, &eth_event_handler, NULL));
//     ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &got_ip_event_handler, NULL));

//     /* start Ethernet driver state machine */
//     ESP_ERROR_CHECK(esp_eth_start(eth_handle));
// }

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

    /* Init sdcard */
    // ESP_ERROR_CHECK(bsp_sdcard_init_default());
    bsp_sdcard_init_default();

    ESP_LOGI(TAG, "main init done");

    xTaskCreate(led_task, "led_task", 1024 * 2, NULL, 2, NULL);
}


