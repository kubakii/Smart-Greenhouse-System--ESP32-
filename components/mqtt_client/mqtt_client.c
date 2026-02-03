#include "mqtt_client.h"
#include "uart_manager.h"
#include "esp_log.h"
#include "cJSON.h"
#include "wifi_manager.h"

static const char *TAG = "MQTT_CLIENT";
static esp_mqtt_client_handle_t mqtt_client = NULL;
static mqtt_command_callback_t command_callback = NULL;

// MQTT事件处理
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                              int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT Connected");
            // 订阅控制主题
            esp_mqtt_client_subscribe(event->client, MQTT_TOPIC_SUBSCRIBE, 0);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT Disconnected");
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT data received: topic=%.*s, data=%.*s",
                    event->topic_len, event->topic,
                    event->data_len, event->data);
            
            // 处理控制命令
            if (command_callback != NULL) {
                char topic[64] = {0};
                char data[256] = {0};
                
                strncpy(topic, event->topic, event->topic_len);
                strncpy(data, event->data, event->data_len);
                
                command_callback(topic, data);
            }
            break;
            
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error");
            break;
            
        default:
            break;
    }
}

// 解析MQTT命令并发送到STM32
static void mqtt_command_handler(const char *topic, const char *data) {
    cJSON *root = cJSON_Parse(data);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return;
    }
    
    uart_command_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    
    cJSON *type = cJSON_GetObjectItem(root, "type");
    cJSON *value = cJSON_GetObjectItem(root, "value");
    cJSON *param = cJSON_GetObjectItem(root, "param");
    
    if (type != NULL && value != NULL) {
        const char *cmd_type = cJSON_GetStringValue(type);
        
        if (strcmp(cmd_type, "motor") == 0) {
            cmd.type = CMD_MOTOR_SPEED;
            cmd.value = cJSON_GetNumberValue(value);
        } else if (strcmp(cmd_type, "led") == 0) {
            cmd.type = CMD_LED_CONTROL;
            cmd.value = cJSON_GetNumberValue(value);
        } else if (strcmp(cmd_type, "pump") == 0) {
            cmd.type = CMD_WATER_PUMP;
            if (param != NULL) {
                strncpy(cmd.param, cJSON_GetStringValue(param), sizeof(cmd.param) - 1);
            }
        }
        
        // 发送到STM32
        uart_send_command(&cmd);
        ESP_LOGI(TAG, "MQTT command sent to STM32: type=%d, value=%d", cmd.type, cmd.value);
    }
    
    cJSON_Delete(root);
}

// 初始化MQTT客户端
esp_err_t mqtt_client_init(void) {
    // 等待WiFi连接
    if (!wifi_is_connected()) {
        ESP_LOGE(TAG, "WiFi not connected");
        return ESP_FAIL;
    }
    
    // 配置MQTT客户端
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = "esp32_greenhouse",
        .session.keepalive = 60,
    };
    
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return ESP_FAIL;
    }
    
    // 注册事件处理器
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    
    // 启动MQTT客户端
    esp_mqtt_client_start(mqtt_client);
    
    // 设置命令回调
    mqtt_set_command_callback(mqtt_command_handler);
    
    ESP_LOGI(TAG, "MQTT client initialized");
    return ESP_OK;
}

// 发布传感器数据
esp_err_t mqtt_publish_sensor_data(float temp, float hum, uint16_t light, uint16_t soil) {
    if (mqtt_client == NULL) {
        return ESP_FAIL;
    }
    
    // 创建JSON数据
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "temperature", temp);
    cJSON_AddNumberToObject(root, "humidity", hum);
    cJSON_AddNumberToObject(root, "light", light);
    cJSON_AddNumberToObject(root, "soil_moisture", soil);
    cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000);
    
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL) {
        cJSON_Delete(root);
        return ESP_FAIL;
    }
    
    // 发布数据
    int msg_id = esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_PUBLISH, 
                                        json_str, 0, 1, 0);
    
    free(json_str);
    cJSON_Delete(root);
    
    if (msg_id < 0) {
        ESP_LOGE(TAG, "Failed to publish MQTT message");
        return ESP_FAIL;
    }
    
    ESP_LOGD(TAG, "Published sensor data to MQTT");
    return ESP_OK;
}

// 设置命令回调
void mqtt_set_command_callback(mqtt_command_callback_t callback) {
    command_callback = callback;
}