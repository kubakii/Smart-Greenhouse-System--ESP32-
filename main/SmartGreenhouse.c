#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "uart.h"
#include "lvgl_driver.h"
#include "mqtt_client.h"
#include "wifi_manager.h"

static const char *TAG = "MAIN";

// 全局队列
static QueueHandle_t sensor_data_queue = NULL;

// 传感器数据处理任务
static void sensor_data_task(void *pvParameter) {
    ESP_LOGI(TAG, "Sensor Data Task started");
    
    sensor_data_t sensor_data;
    
    while (1) {
        // 从串口接收队列获取数据
        QueueHandle_t uart_queue = get_uart_receive_queue();
        if (uart_queue != NULL && xQueueReceive(uart_queue, &sensor_data, portMAX_DELAY) == pdTRUE) {
            // 更新LVGL显示
            update_temperature_display(sensor_data.temperature);
            update_humidity_display(sensor_data.humidity);
            update_light_display(sensor_data.light_level);
            update_soil_display(sensor_data.soil_moisture);
            
            // 发送到MQTT
            mqtt_publish_sensor_data(sensor_data.temperature,
                                    sensor_data.humidity,
                                    sensor_data.light_level,
                                    sensor_data.soil_moisture);
            
            ESP_LOGI(TAG, "Sensor data processed: Temp=%.1f, Hum=%.1f", 
                    sensor_data.temperature, sensor_data.humidity);
        }
    }
}

// 应用初始化
static void app_init(void) {
    ESP_LOGI(TAG, "Initializing application...");
    
    // 1. 初始化WiFi
    wifi_manager_init();
    
    // 2. 初始化串口管理器
    uart_manager_init();
    
    // 3. 初始化LVGL驱动
    lvgl_driver_init();
    
    // 4. 初始化MQTT客户端
    mqtt_client_init();
    
    // 创建传感器数据处理队列
    sensor_data_queue = xQueueCreate(20, sizeof(sensor_data_t));
    
    ESP_LOGI(TAG, "Application initialized");
}

void app_main(void) {
    // 初始化应用
    app_init();
    
    // 创建任务
    xTaskCreate(lvgl_task, "lvgl_task", 8192, NULL, 20, NULL);
    xTaskCreate(sensor_data_task, "sensor_task", 4096, NULL, 16, NULL);
    
    // 主循环
    while (1) {
        // 系统状态监控
        static uint32_t last_print = 0;
        uint32_t now = esp_timer_get_time() / 1000000; // seconds
        
        if (now - last_print >= 10) {
            ESP_LOGI(TAG, "System running... Free heap: %d bytes", esp_get_free_heap_size());
            last_print = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}