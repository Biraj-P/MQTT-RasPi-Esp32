#ifndef DEV_CONFIG_H_
#define DEV_CONFIG_H_

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "DEV_CONFIG"

#define I2C_MASTER_SCL_IO 7
#define I2C_MASTER_SDA_IO 6
#define I2C_MASTER_FREQ_HZ 400000
#define I2C_MASTER_NUM I2C_NUM_0

#define SPI_MOSI_PIN 11
#define SPI_MISO_PIN 12
#define SPI_CLK_PIN 10
#define SPI_CS_PIN 9

#define LCD_BL_PIN 2
#define BAT_ADC_CHANNEL 1

#endif