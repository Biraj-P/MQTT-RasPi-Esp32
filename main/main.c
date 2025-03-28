#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "nvs_flash.h"
#include "esp_mac.h"
#include "QMI8658.h"

#include "my_data.h"

//static const char *TAG = "MQTT_TCP";

static void wifi_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case WIFI_EVENT_STA_START:
        printf("WiFi connecting ... \n");
        break;
    case WIFI_EVENT_STA_CONNECTED:
        printf("WiFi connected ... \n");
        break;
    case WIFI_EVENT_STA_DISCONNECTED:
        printf("WiFi lost connection ... \n");
        break;
    case IP_EVENT_STA_GOT_IP:
        printf("WiFi got IP ... \n\n");
        break;
    default:
        break;
    }
}

void wifi_connection()
{
    //WiFi/LwIP Init Phase
    esp_netif_init();                       // Initialize the underlying TCP/IP stack
    esp_event_loop_create_default();        // event loop
    esp_netif_create_default_wifi_sta();     // Wifi station
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();    // @note Always use WIFI_INIT_CONFIG_DEFAULT macro to initialize the configuration to default values
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_initiation));        //Initialize WiFi Allocate resource for WiFi driver, such as WiFi control structure, RX/TX buffer, WiFi NVS structure etc

    //WiFi Configuration Phase
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);
    wifi_config_t wifi_configuration = {
        .sta = {
            .ssid = SSID,
            .password = PASS,
            .scan_method = WIFI_ALL_CHANNEL_SCAN,
            .failure_retry_cnt = 5,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        }
    };
    
    esp_wifi_set_mode(WIFI_MODE_STA); 
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_configuration));

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    // Wi-Fi Start Phase
    esp_wifi_start();
    // Wi-Fi Connect Phase
    esp_wifi_connect();
}

#define SENSOR_READ_INTERVAL_MS 20 // 50Hz = 20ms interval

esp_mqtt_client_handle_t mqtt_client;

/** @todo ADD sensor function
*/
typedef struct {
    u_int32_t timestamp;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} SensorData;

SensorData read_sensor_data() {
    SensorData data;
    float acc[3], gyro[3];

    // Call function to populate acceleration and gyroscope data
    QMI8658_read_xyz(acc, gyro, NULL);

    // Assign values to the structure
    data.timestamp = esp_log_timestamp();
    data.acc_x = acc[0];
    data.acc_y = acc[1];
    data.acc_z = acc[2];

    data.gyro_x = gyro[0];
    data.gyro_y = gyro[1];
    data.gyro_z = gyro[2];

    return data;
}

/**
 * @brief Send sensor data over MQTT
 */
void mqtt_publish_sensor_data() {
    char payload[128];

    while (1) {
        SensorData sensor = read_sensor_data();
        
        snprintf(payload, sizeof(payload),
                 "{\"timestamp\": %ld, \"acc_x\": %.2f, \"acc_y\": %.2f, \"acc_z\": %.2f, \"gyro_x\": %.2f, \"gyro_y\": %.2f, \"gyro_z\": %.2f}",
                 sensor.timestamp, sensor.acc_x, sensor.acc_y, sensor.acc_z, sensor.gyro_x, sensor.gyro_y, sensor.gyro_z);

        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, payload, 0, 0, 0);
        ESP_LOGI(TAG, "Published: %s", payload);

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS)); // 50Hz (every 20ms)
    }
}


/**
 * @brief MQTT event handler
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    //esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            xTaskCreate(mqtt_publish_sensor_data, "mqtt_publish_task", 4096, NULL, 5, NULL);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT disconnected");
            break;
        default:
            break;
    }
}

/**
 * @brief Initialize MQTT client
 */
void init_mqtt() {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

/**
 * @brief Main function
 */
void app_main() {
    // Initialize I2C
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    // Initialize sensor
    if (QMI8658_init()) {
        ESP_LOGI(TAG, "Sensor initialized successfully");
    } else {
        ESP_LOGE(TAG, "Sensor initialization failed");
        return;
    }
    nvs_flash_init();
    wifi_connection();
    init_mqtt();
}


