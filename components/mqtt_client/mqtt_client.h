#ifndef __MQTT_CLIENT_H__
#define __MQTT_CLIENT_H__

#include "esp_event.h"
#include "mqtt_client.h"
#include "freertos/queue.h"

// MQTT配置
#define MQTT_BROKER_URI "mqtt://broker.emqx.io:1883"
#define MQTT_TOPIC_PUBLISH "greenhouse/sensor/data"
#define MQTT_TOPIC_SUBSCRIBE "greenhouse/control"
#define MQTT_CLIENT_ID "esp32_greenhouse_%s"

// 初始化函数
esp_err_t mqtt_client_init(void);

// 发布传感器数据
esp_err_t mqtt_publish_sensor_data(float temp, float hum, uint16_t light, uint16_t soil);

// 设置命令回调
typedef void (*mqtt_command_callback_t)(const char *topic, const char *data);
void mqtt_set_command_callback(mqtt_command_callback_t callback);

#endif