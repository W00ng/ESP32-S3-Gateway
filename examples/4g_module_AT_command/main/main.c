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
#include "bsp_i2c.h"
#include "esp_usbh_cdc.h"

static const char *TAG = "main";

// config MODEM_USB_IN_EP_ADDR
//     default 0x81 if MODEM_TARGET_ML302_DNLM
//     default 0x84 if MODEM_TARGET_AIR724UG_NFM
//     default 0x86 if  MODEM_TARGET_EC600N_CNLA_N05
//     default 0x81 if  MODEM_TARGET_EC600N_CNLC_N06
//     default 0x81 if MODEM_TARGET_A7600C1
//     default 0x81 if MODEM_TARGET_USER

// config MODEM_USB_OUT_EP_ADDR
//     default 0x01 if MODEM_TARGET_ML302_DNLM
//     default 0x03 if MODEM_TARGET_AIR724UG_NFM
//     default 0x0f if MODEM_TARGET_EC600N_CNLA_N05
//     default 0x0a if MODEM_TARGET_EC600N_CNLC_N06
//     default 0x0a if MODEM_TARGET_A7600C1
//     default 0x01 if MODEM_TARGET_USER

/* ringbuffer size */
#define IN_RINGBUF_SIZE  (1024 * 1)
#define OUT_RINGBUF_SIZE (1024 * 1)

/* bulk endpoint address */
#define EXAMPLE_BULK_IN_EP_ADDR  (0x81)
#define EXAMPLE_BULK_OUT_EP_ADDR (0x0a)
/* bulk endpoint max package size */
#define EXAMPLE_BULK_EP_MPS 64
/* bulk endpoint transfer interval */
#define EXAMPLE_BULK_EP_INTERVAL 0

/* choose if use user endpoint descriptors */
#define EXAMPLE_CONFIG_USER_EP_DESC

#ifdef EXAMPLE_CONFIG_USER_EP_DESC
/*
the basic demo skip the standred get descriptors process,
users need to get params from cdc device descriptors from PC side,
eg. run `lsusb -v` in linux, then hardcode the related params below
*/
static usb_ep_desc_t bulk_out_ep_desc = {
    .bLength = sizeof(usb_ep_desc_t),
    .bDescriptorType = USB_B_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = EXAMPLE_BULK_OUT_EP_ADDR,
    .bmAttributes = USB_BM_ATTRIBUTES_XFER_BULK,
    .wMaxPacketSize = EXAMPLE_BULK_EP_MPS,
    .bInterval = EXAMPLE_BULK_EP_INTERVAL,
};

static usb_ep_desc_t bulk_in_ep_desc = {
    .bLength = sizeof(usb_ep_desc_t),
    .bDescriptorType = USB_B_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = EXAMPLE_BULK_IN_EP_ADDR,
    .bmAttributes = USB_BM_ATTRIBUTES_XFER_BULK,
    .wMaxPacketSize = EXAMPLE_BULK_EP_MPS,
    .bInterval = EXAMPLE_BULK_EP_INTERVAL,
};
#endif

static void usb_receive_task(void *param)
{
    size_t data_len = 0;
    uint8_t buf[IN_RINGBUF_SIZE];

    while (1) {
        /* Polling USB receive buffer to get data */
        usbh_cdc_get_buffered_data_len(&data_len);

        if (data_len == 0) {
            vTaskDelay(1);
            continue;
        }

        usbh_cdc_read_bytes(buf, data_len, 10);
        ESP_LOGI(TAG, "Receive len=%d: %.*s", data_len, data_len, buf);
    }
}

static void usb_connect_callback(void *arg)
{
    ESP_LOGI(TAG, "Device Connected!");
}

static void usb_disconnect_callback(void *arg)
{
    ESP_LOGW(TAG, "Device Disconnected!");
}

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
    ESP_LOGI(TAG, "Force reset 4g board");
    gpio_config_t io_config = {
            .pin_bit_mask = BIT64(GPIO_MODEM_RESET),
            .mode = GPIO_MODE_OUTPUT
    };
    gpio_config(&io_config);
    gpio_set_level(GPIO_MODEM_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(500));
    gpio_set_level(GPIO_MODEM_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Initialize I2C 400KHz */
    ESP_ERROR_CHECK(bsp_i2c_init(I2C_NUM_0, 400000));
    // xTaskCreate(led_task, "led_task", 1024 * 2, NULL, 2, NULL);

    /* @brief install usbh cdc driver with bulk endpoint configs and size of internal ringbuffer*/
#ifdef EXAMPLE_CONFIG_USER_EP_DESC
    ESP_LOGI(TAG, "using user bulk endpoint descriptor");
    static usbh_cdc_config_t config = {
    /* use user endpoint descriptor */
        .bulk_in_ep = &bulk_in_ep_desc,
        .bulk_out_ep = &bulk_out_ep_desc,
#else
    ESP_LOGI(TAG, "using default bulk endpoint descriptor");
    static usbh_cdc_config_t config = {
    /* use default endpoint descriptor with user address */
        .bulk_in_ep_addr = EXAMPLE_BULK_IN_EP_ADDR,
        .bulk_out_ep_addr = EXAMPLE_BULK_OUT_EP_ADDR,
#endif
        .rx_buffer_size = IN_RINGBUF_SIZE,
        .tx_buffer_size = OUT_RINGBUF_SIZE,
        .conn_callback = usb_connect_callback,
        .disconn_callback = usb_disconnect_callback,
    };
    /* install USB host CDC driver */
    esp_err_t ret = usbh_cdc_driver_install(&config);
    assert(ret == ESP_OK);
    /* Waitting for USB device connected */
    ret = usbh_cdc_wait_connect(portMAX_DELAY);
    assert(ret == ESP_OK);
    /* Create a task for USB data processing */
    xTaskCreate(usb_receive_task, "usb_rx", 4096, NULL, 2, NULL);

    /* Repeatly sent AT through USB */
    char buff[5][32] = {"AT\r\n","AT+CPIN?\r\n","AT+CSQ\r\n","AT+CREG?\r\n","AT+CPSI?\r\n"};
    while (1) {
        for (size_t i = 0; i < 5; i++)
        {
            int len = usbh_cdc_write_bytes((uint8_t *)buff[i], strlen(buff[i]));
            ESP_LOGI(TAG, "Send len=%d: %s", len, buff[i]);
            vTaskDelay(pdMS_TO_TICKS(2000));            
        }
    }
    // char buff[32] = "AT+CPIN?\r\n";
    // while (1) {
    //     int len = usbh_cdc_write_bytes((uint8_t *)buff, strlen(buff));
    //     ESP_LOGI(TAG, "Send len=%d: %s", len, buff);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
}

