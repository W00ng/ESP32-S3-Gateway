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
#include "esp_netif.h"
#include "esp_eth.h"
#include "esp_event.h"
#include "driver/spi_master.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"

static const char *TAG = "main";

#define ETH_SPI_HOST         (SPI2_HOST)
#define ETH_SPI_CLOCK_MHZ    (12)
#define WIFI_SSID            "ESP32-ETH2AP"
#define WIFI_PASSWORD        "12345678"
#define WIFI_MAX_STA_CONN    (4)
#define WIFI_CHANNEL         (1)   // 1~13

typedef struct {
    uint8_t spi_cs_gpio;
    uint8_t int_gpio;
    int8_t phy_reset_gpio;
    uint8_t phy_addr;
}spi_eth_module_config_t;

static esp_eth_handle_t s_eth_handle = NULL;
static QueueHandle_t flow_control_queue = NULL;
static bool s_sta_is_connected = false;
static bool s_ethernet_is_connected = false;
static uint8_t s_eth_mac[6];

#define FLOW_CONTROL_QUEUE_TIMEOUT_MS (100)
#define FLOW_CONTROL_QUEUE_LENGTH (40)
#define FLOW_CONTROL_WIFI_SEND_TIMEOUT_MS (100)

typedef struct {
    void *packet;
    uint16_t length;
} flow_control_msg_t;

// Forward packets from Wi-Fi to Ethernet
static esp_err_t pkt_wifi2eth(void *buffer, uint16_t len, void *eb)
{
    if (s_ethernet_is_connected) {
        if (esp_eth_transmit(s_eth_handle, buffer, len) != ESP_OK) {
            ESP_LOGE(TAG, "Ethernet send packet failed");
        }
    }
    esp_wifi_internal_free_rx_buffer(eb);
    return ESP_OK;
}

// Forward packets from Ethernet to Wi-Fi
// Note that, Ethernet works faster than Wi-Fi on ESP32,
// so we need to add an extra queue to balance their speed difference.
static esp_err_t pkt_eth2wifi(esp_eth_handle_t eth_handle, uint8_t *buffer, uint32_t len, void *priv)
{
    esp_err_t ret = ESP_OK;
    flow_control_msg_t msg = {
        .packet = buffer,
        .length = len
    };
    if (xQueueSend(flow_control_queue, &msg, pdMS_TO_TICKS(FLOW_CONTROL_QUEUE_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "send flow control message failed or timeout");
        free(buffer);
        ret = ESP_FAIL;
    }
    return ret;
}

// This task will fetch the packet from the queue, and then send out through Wi-Fi.
// Wi-Fi handles packets slower than Ethernet, we might add some delay between each transmitting.
static void eth2wifi_flow_control_task(void *args)
{
    flow_control_msg_t msg;
    int res = 0;
    uint32_t timeout = 0;
    while (1) {
        if (xQueueReceive(flow_control_queue, &msg, pdMS_TO_TICKS(FLOW_CONTROL_QUEUE_TIMEOUT_MS)) == pdTRUE) {
            timeout = 0;
            if (s_sta_is_connected && msg.length) {
                do {
                    vTaskDelay(pdMS_TO_TICKS(timeout));
                    timeout += 2;
                    res = esp_wifi_internal_tx(WIFI_IF_AP, msg.packet, msg.length);
                } while (res && timeout < FLOW_CONTROL_WIFI_SEND_TIMEOUT_MS);
                if (res != ESP_OK) {
                    ESP_LOGE(TAG, "WiFi send packet failed: %d", res);
                }
            }
            free(msg.packet);
        }
    }
    vTaskDelete(NULL);
}

// Event handler for Ethernet
static void eth_event_handler(void *arg, esp_event_base_t event_base,
                              int32_t event_id, void *event_data)
{
    switch (event_id) {
    case ETHERNET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Up");
        s_ethernet_is_connected = true;
        esp_eth_ioctl(s_eth_handle, ETH_CMD_G_MAC_ADDR, s_eth_mac);
        esp_wifi_set_mac(WIFI_IF_AP, s_eth_mac);
        ESP_ERROR_CHECK(esp_wifi_start());
        break;
    case ETHERNET_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Link Down");
        s_ethernet_is_connected = false;
        ESP_ERROR_CHECK(esp_wifi_stop());
        break;
    case ETHERNET_EVENT_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case ETHERNET_EVENT_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
}

// Event handler for Wi-Fi
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    static uint8_t s_con_cnt = 0;
    switch (event_id) {
    case WIFI_EVENT_AP_STACONNECTED:
        ESP_LOGI(TAG, "Wi-Fi AP got a station connected");
        if (!s_con_cnt) {
            s_sta_is_connected = true;
            esp_wifi_internal_reg_rxcb(WIFI_IF_AP, pkt_wifi2eth);
        }
        s_con_cnt++;
        break;
    case WIFI_EVENT_AP_STADISCONNECTED:
        ESP_LOGI(TAG, "Wi-Fi AP got a station disconnected");
        s_con_cnt--;
        if (!s_con_cnt) {
            s_sta_is_connected = false;
            esp_wifi_internal_reg_rxcb(WIFI_IF_AP, NULL);
        }
        break;
    default:
        break;
    }
}

static void initialize_ethernet(void)
{
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    spi_device_handle_t spi_handle = NULL;
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_SPI_MISO,
        .mosi_io_num = GPIO_SPI_MOSI,
        .sclk_io_num = GPIO_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(ETH_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_eth_module_config_t spi_eth_module_config;
    spi_eth_module_config.spi_cs_gpio = GPIO_SPI_CS;
    spi_eth_module_config.int_gpio = GPIO_W5500_INT;
    spi_eth_module_config.phy_reset_gpio = GPIO_W5500_RST;
    spi_eth_module_config.phy_addr = 1;   // 0~31

    spi_device_interface_config_t devcfg = {
        .command_bits = 16, // Actually it's the address phase in W5500 SPI frame
        .address_bits = 8,  // Actually it's the control phase in W5500 SPI frame
        .mode = 0,
        .clock_speed_hz = ETH_SPI_CLOCK_MHZ * 1000 * 1000,
        .spics_io_num = spi_eth_module_config.spi_cs_gpio,
        .queue_size = 20
    };
    ESP_ERROR_CHECK(spi_bus_add_device(ETH_SPI_HOST, &devcfg, &spi_handle));

    /* w5500 ethernet driver is based on spi driver */
    eth_w5500_config_t w5500_config = ETH_W5500_DEFAULT_CONFIG(spi_handle);
    w5500_config.int_gpio_num = spi_eth_module_config.int_gpio;

    eth_mac_config_t mac_config = ETH_MAC_DEFAULT_CONFIG();
    eth_phy_config_t phy_config = ETH_PHY_DEFAULT_CONFIG();
    phy_config.phy_addr = spi_eth_module_config.phy_addr;
    phy_config.reset_gpio_num = spi_eth_module_config.phy_reset_gpio;
    esp_eth_mac_t *mac = esp_eth_mac_new_w5500(&w5500_config, &mac_config);
    esp_eth_phy_t *phy = esp_eth_phy_new_w5500(&phy_config);

    esp_eth_config_t eth_config = ETH_DEFAULT_CONFIG(mac, phy);
    eth_config.stack_input = pkt_eth2wifi;
    ESP_ERROR_CHECK(esp_eth_driver_install(&eth_config, &s_eth_handle));

    /* The SPI Ethernet module might doesn't have a burned factory MAC address, we cat to set it manually.
       02:00:00 is a Locally Administered OUI range so should not be used except when testing on a LAN under your control.
    */
    ESP_ERROR_CHECK(esp_eth_ioctl(s_eth_handle, ETH_CMD_S_MAC_ADDR, (uint8_t[]) {
        0x02, 0x00, 0x00, 0x12, 0x34, 0x56
    }));

    bool eth_promiscuous = true;
    esp_eth_ioctl(s_eth_handle, ETH_CMD_S_PROMISCUOUS, &eth_promiscuous);

    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ESP_EVENT_ANY_ID, eth_event_handler, NULL));
    esp_eth_start(s_eth_handle);
}

static void initialize_wifi(void)
{
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .password = WIFI_PASSWORD,
            .max_connection = WIFI_MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .channel = WIFI_CHANNEL // default: channel 1
        },
    };
    if (strlen(WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
}

static esp_err_t initialize_flow_control(void)
{
    flow_control_queue = xQueueCreate(FLOW_CONTROL_QUEUE_LENGTH, sizeof(flow_control_msg_t));
    if (!flow_control_queue) {
        ESP_LOGE(TAG, "create flow control queue failed");
        return ESP_FAIL;
    }
    BaseType_t ret = xTaskCreate(eth2wifi_flow_control_task, "flow_ctl", 2048, NULL, (tskIDLE_PRIORITY + 2), NULL);
    if (ret != pdTRUE) {
        ESP_LOGE(TAG, "create flow control task failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void app_main(void)
{
    /* Initialize I2C 400KHz */
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(initialize_flow_control());
    initialize_wifi();
    initialize_ethernet();
}

