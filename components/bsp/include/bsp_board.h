
#ifndef _BSP_BOARD_H_
#define _BSP_BOARD_H_

#include "driver/gpio.h"

/**
 * @brief ESP32-S3 LCD GPIO defination and config
 * 
 */
#define LCD_WIDTH           (320)
#define LCD_HEIGHT          (240)
#define LCD_BL_ON_LEVEL     (0)
#define LCD_BL_OFF_LEVEL    !LCD_BL_ON_LEVEL
#define LCD_CMD_BITS        (8)
#define LCD_PARAM_BITS      (8)
#define LCD_SPI_CLOCK_HZ    (40 * 1000 * 1000)
#define LCD_SPI_HOST        (SPI3_HOST)

#define GPIO_LCD_BL     (GPIO_NUM_21)
#define GPIO_LCD_RST    (GPIO_NUM_38)
#define GPIO_LCD_SCK    (GPIO_NUM_39)
#define GPIO_LCD_DC     (GPIO_NUM_40)
#define GPIO_LCD_CS     (GPIO_NUM_41)
#define GPIO_LCD_SDA    (GPIO_NUM_42)

/**
 * @brief ESP32-S3 LED GPIO defineation
 * 
 */
#define GPIO_LED_STATUS    (GPIO_NUM_3)

/**
 * @brief ESP32-S3 I2C GPIO defineation
 * 
 */
#define FUNC_I2C_EN     (1)
#define GPIO_I2C_SCL    (GPIO_NUM_2)
#define GPIO_I2C_SDA    (GPIO_NUM_1)

/**
 * @brief ESP32-S3 SPI GPIO defination
 * 
 */
#define FUNC_SPI_EN      (1)
#define GPIO_SPI_MOSI    (GPIO_NUM_6)
#define GPIO_SPI_MISO    (GPIO_NUM_7)
#define GPIO_SPI_CLK     (GPIO_NUM_15)
#define GPIO_SPI_CS      (GPIO_NUM_16)

/**
 * @brief ESP32-S3 UART GPIO defination
 * 
 */
#define GPIO_UART_TXD0    (GPIO_NUM_43)
#define GPIO_UART_RXD0    (GPIO_NUM_44)
#define GPIO_UART_TXD1    (GPIO_NUM_47)
#define GPIO_UART_RXD1    (GPIO_NUM_48)

/**
 * @brief ESP32-S3 SDMMC GPIO defination
 * 
 */
#define SDMMC_BUS_WIDTH (1)
#define GPIO_SDMMC_CLK    (GPIO_NUM_18)
#define GPIO_SDMMC_CMD    (GPIO_NUM_8)
#define GPIO_SDMMC_D0     (GPIO_NUM_17)
#define GPIO_SDMMC_D1     (GPIO_NUM_NC)
#define GPIO_SDMMC_D2     (GPIO_NUM_NC)
#define GPIO_SDMMC_D3     (GPIO_NUM_NC)
#define GPIO_SDMMC_DET    (GPIO_NUM_NC)

/**
 * @brief ESP32-S3 EXTRA GPIO defination
 * 
 */
#define GPIO_W5500_RST    (GPIO_NUM_4)
#define GPIO_W5500_INT    (GPIO_NUM_5)

#define GPIO_MODEM_RESET  (GPIO_NUM_45)

#endif /* CONFIG_ESP32_S3_DEVKIT_BOARD */

