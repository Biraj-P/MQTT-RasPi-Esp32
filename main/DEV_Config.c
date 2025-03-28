#include "DEV_Config.h"
#include "esp_log.h"

static spi_device_handle_t spi;

void DEV_Digital_Write(gpio_num_t Pin, uint8_t Value)
{
    gpio_set_level(Pin, Value);
}

uint8_t DEV_Digital_Read(gpio_num_t Pin)
{
    return gpio_get_level(Pin);
}

void DEV_SPI_WriteByte(uint8_t Value)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &Value
    };
    spi_device_transmit(spi, &t);
}

void DEV_I2C_Write_Byte(uint8_t addr, uint8_t reg, uint8_t Value)
{
    uint8_t data[] = {reg, Value};
    i2c_master_write_to_device(I2C_MASTER_NUM, addr, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
}

uint8_t DEV_I2C_Read_Byte(uint8_t addr, uint8_t reg)
{
    uint8_t value;
    i2c_master_write_read_device(I2C_MASTER_NUM, addr, &reg, 1, &value, 1, 1000 / portTICK_PERIOD_MS);
    return value;
}

uint16_t DEV_ADC_Read(void)
{
    return adc1_get_raw(BAT_ADC_CHANNEL);
}

void DEV_GPIO_Mode(gpio_num_t Pin, gpio_mode_t Mode)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << Pin),
        .mode = Mode,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void DEV_Set_PWM(uint8_t Value)
{
    if (Value > 100)
    {
        ESP_LOGE(TAG, "Invalid PWM Value");
    }
    else
    {
        // Implement PWM if needed
    }
}

void DEV_Delay_ms(uint32_t xms)
{
    vTaskDelay(xms / portTICK_PERIOD_MS);
}

void DEV_GPIO_Init(void)
{
    DEV_GPIO_Mode(SPI_CS_PIN, GPIO_MODE_OUTPUT);
    DEV_GPIO_Mode(LCD_BL_PIN, GPIO_MODE_OUTPUT);
    DEV_Digital_Write(SPI_CS_PIN, 1);
    DEV_Digital_Write(LCD_BL_PIN, 1);
}

uint8_t DEV_Module_Init(void)
{
    ESP_LOGI(TAG, "Initializing Module");
    DEV_GPIO_Init();

    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size = 1
    };
    spi_bus_add_device(SPI2_HOST, &devcfg, &spi);

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    return 0;
}

void DEV_Module_Exit(void)
{
    spi_bus_remove_device(spi);
    spi_bus_free(SPI2_HOST);
    i2c_driver_delete(I2C_MASTER_NUM);
}
