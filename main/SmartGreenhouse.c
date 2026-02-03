/**
 * @file main.c
 * @brief 智能温室控制系统 - ESP32主程序（纯C语言版本）
 * 
 * 任务架构：
 * 1. 串口发送任务 (优先级14) - 发送控制命令到STM32
 * 2. 串口接收任务 (优先级18) - 接收STM32传感器数据
 * 3. LVGL显示任务 (优先级20) - 触摸屏UI界面
 * 4. WiFi/MQTT任务 (优先级16) - 网络通信
 * 5. 传感器数据处理任务 (优先级15) - 数据处理和分发
 * 6. 系统监控任务 (优先级10) - 系统状态监控
 * 
 * 系统特性：
 * - 实时传感器数据显示
 * - 触摸屏控制界面
 * - 远程MQTT监控
 * - 自动重连机制
 * - 错误恢复处理
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

// 项目组件头文件
#include "uart.h"
#include "lvgl_driver.h"
#include "mqtt_client.h"
#include "wifi_manager.h"

// 标签定义
static const char *TAG = "MAIN";

// 系统配置
#define SYSTEM_VERSION "1.0.0"
#define SYSTEM_NAME "SmartGreenhouse-ESP32"

// 任务优先级定义
#define TASK_PRIORITY_LVGL       20  // LVGL显示任务（最高）
#define TASK_PRIORITY_UART_RX    18  // 串口接收任务
#define TASK_PRIORITY_MQTT       16  // WiFi/MQTT任务
#define TASK_PRIORITY_DATA_PROC  15  // 数据处理任务
#define TASK_PRIORITY_UART_TX    14  // 串口发送任务
#define TASK_PRIORITY_SYS_MON    10  // 系统监控任务

// 任务堆栈大小
#define STACK_SIZE_LVGL          8192
#define STACK_SIZE_UART_RX       4096
#define STACK_SIZE_UART_TX       2048
#define STACK_SIZE_MQTT          4096
#define STACK_SIZE_DATA_PROC     4096
#define STACK_SIZE_SYS_MON       3072

// 事件组标志位
#define EVENT_WIFI_CONNECTED     BIT0
#define EVENT_MQTT_CONNECTED     BIT1
#define EVENT_STM32_CONNECTED    BIT2
#define EVENT_DATA_RECEIVED      BIT3
#define EVENT_UI_UPDATE          BIT4
#define EVENT_SYSTEM_ERROR       BIT5

// 系统状态结构
typedef struct {
    uint32_t uptime_ms;
    uint32_t sensor_read_count;
    uint32_t mqtt_publish_count;
    uint32_t uart_command_count;
    uint32_t display_fps;
    uint32_t wifi_rssi;
    uint32_t last_stm32_heartbeat;
    bool wifi_connected;
    bool mqtt_connected;
    bool stm32_connected;
    bool system_ready;
    char error_message[64];
} system_status_t;

// 温室控制参数
typedef struct {
    uint16_t motor_speed;        // 电机速度 (0-1000)
    uint8_t led_state;           // LED状态 (0/1)
    uint8_t fan_state;           // 风扇状态 (0/1)
    uint8_t pump_state;          // 水泵状态 (0/1)
    float target_temperature;    // 目标温度
    float target_humidity;       // 目标湿度
} control_params_t;

// LVGL UI组件句柄（全局访问）
typedef struct {
    lv_obj_t *temp_label;
    lv_obj_t *hum_label;
    lv_obj_t *light_label;
    lv_obj_t *soil_label;
    lv_obj_t *motor_slider;
    lv_obj_t *led_switch;
    lv_obj_t *fan_switch;
    lv_obj_t *pump_switch;
    lv_obj_t *status_label;
    lv_obj_t *wifi_icon;
    lv_obj_t *mqtt_icon;
    lv_obj_t *stm32_icon;
    lv_obj_t *motor_value_label;
} ui_components_t;

// ==================== 全局变量 ====================

static EventGroupHandle_t system_event_group = NULL;
static QueueHandle_t sensor_data_queue = NULL;
static QueueHandle_t control_cmd_queue = NULL;
static SemaphoreHandle_t system_mutex = NULL;
static TaskHandle_t task_handles[6] = {NULL};
static system_status_t g_system_status = {0};
static control_params_t g_control_params = {
    .motor_speed = 500,
    .led_state = 0,
    .fan_state = 0,
    .pump_state = 0,
    .target_temperature = 25.0,
    .target_humidity = 60.0
};
static ui_components_t g_ui_components = {0};

// ==================== 静态函数声明 ====================

// 系统初始化函数
static esp_err_t system_init_nvs(void);
static esp_err_t system_init_resources(void);

// WiFi回调函数
static void wifi_connected_callback(void* arg);
static void wifi_disconnected_callback(void* arg, uint8_t reason);
static void wifi_got_ip_callback(void* arg, esp_netif_ip_info_t* ip_info);

// 串口接收回调包装器
static void uart_data_received_callback(sensor_data_t *data);

// 任务函数
static void uart_send_task(void *arg);
static void uart_receive_task(void *arg);
static void wifi_mqtt_task(void *arg);
static void sensor_data_process_task(void *arg);
static void system_monitor_task(void *arg);

// UI回调函数
static void motor_slider_event_cb(lv_event_t *e);
static void led_switch_event_cb(lv_event_t *e);
static void fan_switch_event_cb(lv_event_t *e);
static void pump_switch_event_cb(lv_event_t *e);

// UI更新函数
static void ui_update_timer_callback(lv_timer_t *timer);
static void update_ui_display(void);
static void create_greenhouse_ui(void);

// 应用程序函数
static void app_init(void);
static void app_create_tasks(void);

// ==================== 系统初始化函数实现 ====================

/**
 * @brief 初始化非易失性存储
 */
static esp_err_t system_init_nvs(void) {
    ESP_LOGI(TAG, "Initializing NVS...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition truncated, erasing...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");
    return ESP_OK;
}

/**
 * @brief 初始化系统资源
 */
static esp_err_t system_init_resources(void) {
    ESP_LOGI(TAG, "Initializing system resources...");
    
    // 创建事件组
    system_event_group = xEventGroupCreate();
    if (!system_event_group) {
        ESP_LOGE(TAG, "Failed to create system event group");
        return ESP_ERR_NO_MEM;
    }
    
    // 创建互斥锁
    system_mutex = xSemaphoreCreateMutex();
    if (!system_mutex) {
        ESP_LOGE(TAG, "Failed to create system mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 创建队列
    sensor_data_queue = xQueueCreate(20, sizeof(sensor_data_t));
    control_cmd_queue = xQueueCreate(10, sizeof(uart_command_t));
    
    if (!sensor_data_queue || !control_cmd_queue) {
        ESP_LOGE(TAG, "Failed to create system queues");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化系统状态
    memset(&g_system_status, 0, sizeof(g_system_status));
    g_system_status.system_ready = false;
    
    ESP_LOGI(TAG, "System resources initialized successfully");
    return ESP_OK;
}

// ==================== WiFi回调函数实现 ====================

/**
 * @brief WiFi连接成功回调
 */
static void wifi_connected_callback(void* arg) {
    ESP_LOGI(TAG, "WiFi connected!");
    xEventGroupSetBits(system_event_group, EVENT_WIFI_CONNECTED);
    
    // 更新UI状态
    if (g_ui_components.wifi_icon) {
        lv_label_set_text(g_ui_components.wifi_icon, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_color(g_ui_components.wifi_icon, 
                                   lv_color_hex(0x00FF00), 0);
    }
}

/**
 * @brief WiFi断开连接回调
 */
static void wifi_disconnected_callback(void* arg, uint8_t reason) {
    ESP_LOGW(TAG, "WiFi disconnected, reason: %d", reason);
    xEventGroupClearBits(system_event_group, EVENT_WIFI_CONNECTED);
    
    // 更新UI状态
    if (g_ui_components.wifi_icon) {
        lv_label_set_text(g_ui_components.wifi_icon, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_color(g_ui_components.wifi_icon, 
                                   lv_color_hex(0xFF0000), 0);
    }
}

/**
 * @brief WiFi获取IP回调
 */
static void wifi_got_ip_callback(void* arg, esp_netif_ip_info_t* ip_info) {
    ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&ip_info->ip));
    
    // 可以在这里启动MQTT连接
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_system_status.wifi_connected = true;
        xSemaphoreGive(system_mutex);
    }
}

// ==================== 串口接收回调包装器 ====================

/**
 * @brief 串口接收回调包装器
 */
static void uart_data_received_callback(sensor_data_t *data) {
    // 将数据发送到传感器队列
    if (xQueueSend(sensor_data_queue, data, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Sensor data queue full, dropping data");
    } else {
        // 设置数据接收事件
        xEventGroupSetBits(system_event_group, EVENT_DATA_RECEIVED);
        
        // 更新STM32连接状态
        if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            g_system_status.stm32_connected = true;
            g_system_status.last_stm32_heartbeat = esp_timer_get_time() / 1000;
            xSemaphoreGive(system_mutex);
        }
        
        // 更新UI中的STM32连接状态
        if (g_ui_components.stm32_icon) {
            lv_label_set_text(g_ui_components.stm32_icon, LV_SYMBOL_USB);
            lv_obj_set_style_text_color(g_ui_components.stm32_icon, 
                                      lv_color_hex(0x00FF00), 0);
        }
    }
}

// ==================== 串口发送任务实现 ====================

/**
 * @brief 串口发送任务 - 发送控制命令到STM32
 * @param arg 任务参数
 */
static void uart_send_task(void *arg) {
    ESP_LOGI(TAG, "UART Send Task started");
    
    uart_command_t cmd;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // 从控制命令队列接收指令
        if (xQueueReceive(control_cmd_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGD(TAG, "Sending command: type=%d, value=%d", cmd.type, cmd.value);
            
            // 发送命令到STM32
            esp_err_t ret = uart_send_command(&cmd);
            if (ret == ESP_OK) {
                // 更新统计
                if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_system_status.uart_command_count++;
                    xSemaphoreGive(system_mutex);
                }
                
                ESP_LOGI(TAG, "Command sent successfully: type=%d, value=%d", 
                        cmd.type, cmd.value);
            } else {
                ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
            }
        }
        
        // 固定周期发送心跳包（可选）
        static uint32_t last_heartbeat = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        
        if (now - last_heartbeat > 10000) { // 每10秒发送一次心跳
            uart_command_t heartbeat = {
                .type = CMD_GET_STATUS,
                .value = 0
            };
            
            if (xQueueSend(control_cmd_queue, &heartbeat, 0) == pdTRUE) {
                ESP_LOGD(TAG, "Heartbeat sent to STM32");
            }
            
            last_heartbeat = now;
        }
        
        // 任务延迟
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    }
}

// ==================== 串口接收任务实现 ====================

/**
 * @brief 串口接收任务 - 接收STM32传感器数据
 * @param arg 任务参数
 */
static void uart_receive_task(void *arg) {
    ESP_LOGI(TAG, "UART Receive Task started");
    
    // 设置串口接收回调
    uart_set_receive_callback(uart_data_received_callback);
    
    // 任务主循环（主要是等待事件）
    while (1) {
        // 检查STM32连接超时
        uint32_t now = esp_timer_get_time() / 1000;
        if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (g_system_status.stm32_connected && 
                (now - g_system_status.last_stm32_heartbeat > 30000)) { // 30秒超时
                ESP_LOGW(TAG, "STM32 connection timeout");
                g_system_status.stm32_connected = false;
                
                // 更新UI
                if (g_ui_components.stm32_icon) {
                    lv_label_set_text(g_ui_components.stm32_icon, LV_SYMBOL_USB);
                    lv_obj_set_style_text_color(g_ui_components.stm32_icon, 
                                              lv_color_hex(0xFF0000), 0);
                }
            }
            xSemaphoreGive(system_mutex);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== WiFi/MQTT任务实现 ====================

/**
 * @brief WiFi/MQTT任务 - 处理网络连接和数据传输
 * @param arg 任务参数
 */
static void wifi_mqtt_task(void *arg) {
    ESP_LOGI(TAG, "WiFi/MQTT Task started");
    
    // 初始化WiFi管理器
    ESP_LOGI(TAG, "Initializing WiFi...");
    esp_err_t wifi_ret = wifi_manager_init();
    
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "WiFi initialization failed: %s", esp_err_to_name(wifi_ret));
        strncpy(g_system_status.error_message, "WiFi init failed", 
                sizeof(g_system_status.error_message) - 1);
        xEventGroupSetBits(system_event_group, EVENT_SYSTEM_ERROR);
    } else {
        // 设置WiFi回调
        wifi_manager_set_callbacks(
            wifi_connected_callback,
            wifi_disconnected_callback,
            wifi_got_ip_callback,
            NULL
        );
        
        // 启用自动重连
        wifi_manager_set_auto_reconnect(true);
        
        // 启动WiFi连接
        wifi_ret = wifi_manager_start();
        if (wifi_ret != ESP_OK) {
            ESP_LOGE(TAG, "WiFi connection failed: %s", esp_err_to_name(wifi_ret));
        }
    }
    
    TickType_t last_wake_time = xTaskGetTickCount();
    bool mqtt_initialized = false;
    
    while (1) {
        // 等待WiFi连接成功
        EventBits_t bits = xEventGroupWaitBits(system_event_group,
                                              EVENT_WIFI_CONNECTED,
                                              pdFALSE, pdFALSE,
                                              pdMS_TO_TICKS(100));
        
        if (bits & EVENT_WIFI_CONNECTED) {
            // WiFi已连接，初始化MQTT
            if (!mqtt_initialized) {
                ESP_LOGI(TAG, "WiFi connected, initializing MQTT...");
                
                esp_err_t mqtt_ret = mqtt_client_init();
                if (mqtt_ret != ESP_OK) {
                    ESP_LOGW(TAG, "MQTT initialization failed, retrying later...");
                } else {
                    mqtt_initialized = true;
                    ESP_LOGI(TAG, "MQTT initialized successfully");
                    xEventGroupSetBits(system_event_group, EVENT_MQTT_CONNECTED);
                }
            }
            
            // 更新WiFi信号强度
            int8_t rssi = wifi_manager_get_rssi();
            if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_system_status.wifi_rssi = (uint32_t)rssi;
                xSemaphoreGive(system_mutex);
            }
        }
        
        // 定期检查连接状态
        static uint32_t last_status_check = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        
        if (now - last_status_check > 10000) { // 每10秒检查一次
            wifi_manager_print_status();
            last_status_check = now;
        }
        
        // 任务延迟
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

// ==================== 传感器数据处理任务实现 ====================

/**
 * @brief 传感器数据处理任务 - 处理数据并分发到UI和MQTT
 * @param arg 任务参数
 */
static void sensor_data_process_task(void *arg) {
    ESP_LOGI(TAG, "Sensor Data Process Task started");
    
    sensor_data_t sensor_data;
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // 从队列接收传感器数据（带超时）
        if (xQueueReceive(sensor_data_queue, &sensor_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            // 更新统计
            if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_system_status.sensor_read_count++;
                xSemaphoreGive(system_mutex);
            }
            
            // 更新LVGL显示
            if (g_ui_components.temp_label) {
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%.1f°C", sensor_data.temperature);
                lv_label_set_text(g_ui_components.temp_label, buffer);
            }
            
            if (g_ui_components.hum_label) {
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%.1f%%", sensor_data.humidity);
                lv_label_set_text(g_ui_components.hum_label, buffer);
            }
            
            if (g_ui_components.light_label) {
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%d", sensor_data.light_level);
                lv_label_set_text(g_ui_components.light_label, buffer);
            }
            
            if (g_ui_components.soil_label) {
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%d", sensor_data.soil_moisture);
                lv_label_set_text(g_ui_components.soil_label, buffer);
            }
            
            // 发布到MQTT
            if (mqtt_publish_sensor_data(sensor_data.temperature,
                                       sensor_data.humidity,
                                       sensor_data.light_level,
                                       sensor_data.soil_moisture) == ESP_OK) {
                // 更新统计
                if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_system_status.mqtt_publish_count++;
                    xSemaphoreGive(system_mutex);
                }
                
                // 更新MQTT连接状态
                if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_system_status.mqtt_connected = true;
                    xSemaphoreGive(system_mutex);
                }
                
                // 更新UI中的MQTT图标
                if (g_ui_components.mqtt_icon) {
                    lv_label_set_text(g_ui_components.mqtt_icon, LV_SYMBOL_CHARGE);
                    lv_obj_set_style_text_color(g_ui_components.mqtt_icon, 
                                              lv_color_hex(0x00FF00), 0);
                }
            } else {
                // MQTT发布失败
                if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    g_system_status.mqtt_connected = false;
                    xSemaphoreGive(system_mutex);
                }
                
                // 更新UI中的MQTT图标
                if (g_ui_components.mqtt_icon) {
                    lv_label_set_text(g_ui_components.mqtt_icon, LV_SYMBOL_CHARGE);
                    lv_obj_set_style_text_color(g_ui_components.mqtt_icon, 
                                              lv_color_hex(0xFF0000), 0);
                }
            }
            
            // 调试输出（每5秒一次）
            static uint32_t last_log_time = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            
            if (now - last_log_time > 5000) {
                ESP_LOGI(TAG, "Sensor: Temp=%.1f°C, Hum=%.1f%%, Light=%d, Soil=%d",
                        sensor_data.temperature, sensor_data.humidity,
                        sensor_data.light_level, sensor_data.soil_moisture);
                last_log_time = now;
            }
            
            // 设置UI更新事件
            xEventGroupSetBits(system_event_group, EVENT_UI_UPDATE);
        }
        
        // 固定周期执行
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(50));
    }
}

// ==================== 系统监控任务实现 ====================

/**
 * @brief 系统监控任务 - 监控系统状态和资源使用
 * @param arg 任务参数
 */
static void system_monitor_task(void *arg) {
    ESP_LOGI(TAG, "System Monitor Task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t last_heap_print = 0;
    
    while (1) {
        // 更新系统运行时间
        g_system_status.uptime_ms = esp_timer_get_time() / 1000;
        
        // 更新显示帧率
        g_system_status.display_fps = lvgl_driver_get_fps();
        
        // 定期输出系统状态
        uint32_t now = esp_timer_get_time() / 1000;
        
        if (now - last_heap_print > 30000) { // 每30秒输出一次
            uint32_t free_heap = esp_get_free_heap_size();
            uint32_t min_free_heap = esp_get_minimum_free_heap_size();
            
            ESP_LOGI(TAG, "=== System Status ===");
            ESP_LOGI(TAG, "Uptime: %"PRIu32" seconds", g_system_status.uptime_ms / 1000);
            ESP_LOGI(TAG, "Heap: Free=%"PRIu32"B, Min=%"PRIu32"B", free_heap, min_free_heap);
            ESP_LOGI(TAG, "Tasks: %d sensors, %d MQTT, %d commands",
                    g_system_status.sensor_read_count,
                    g_system_status.mqtt_publish_count,
                    g_system_status.uart_command_count);
            ESP_LOGI(TAG, "Display FPS: %"PRIu32, g_system_status.display_fps);
            ESP_LOGI(TAG, "WiFi RSSI: %"PRId32" dBm", g_system_status.wifi_rssi);
            ESP_LOGI(TAG, "Connections: WiFi=%s, MQTT=%s, STM32=%s",
                    g_system_status.wifi_connected ? "OK" : "NO",
                    g_system_status.mqtt_connected ? "OK" : "NO",
                    g_system_status.stm32_connected ? "OK" : "NO");
            
            last_heap_print = now;
        }
        
        // 检查系统错误
        EventBits_t error_bits = xEventGroupGetBits(system_event_group);
        if (error_bits & EVENT_SYSTEM_ERROR) {
            ESP_LOGE(TAG, "System error detected: %s", g_system_status.error_message);
            
            // 可以在这里实现错误恢复逻辑
            // 例如：重启WiFi、重新初始化组件等
            
            // 清除错误标志
            xEventGroupClearBits(system_event_group, EVENT_SYSTEM_ERROR);
        }
        
        // 检查任务运行状态
        for (int i = 0; i < sizeof(task_handles)/sizeof(task_handles[0]); i++) {
            if (task_handles[i]) {
                TaskStatus_t task_status;
                vTaskGetInfo(task_handles[i], &task_status, pdTRUE, eInvalid);
                
                // 检查堆栈高水位线
                if (task_status.usStackHighWaterMark < 128) {
                    ESP_LOGW(TAG, "Task %s low stack: %d", 
                            task_status.pcTaskName, task_status.usStackHighWaterMark);
                }
            }
        }
        
        // 任务延迟
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1000));
    }
}

// ==================== UI回调函数实现 ====================

/**
 * @brief 电机速度滑动条回调
 */
static void motor_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    // 更新全局控制参数
    g_control_params.motor_speed = value;
    
    // 更新电机值标签
    if (g_ui_components.motor_value_label) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%d", value);
        lv_label_set_text(g_ui_components.motor_value_label, buffer);
    }
    
    // 发送控制命令到STM32
    uart_command_t cmd = {
        .type = CMD_MOTOR_SPEED,
        .value = value
    };
    
    if (xQueueSend(control_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue motor command");
    } else {
        ESP_LOGI(TAG, "Motor speed set to: %d", value);
    }
}

/**
 * @brief LED开关回调
 */
static void led_switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    
    // 更新全局控制参数
    g_control_params.led_state = state ? 1 : 0;
    
    // 发送控制命令到STM32
    uart_command_t cmd = {
        .type = CMD_LED_CONTROL,
        .value = state ? 1 : 0
    };
    
    if (xQueueSend(control_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue LED command");
    } else {
        ESP_LOGI(TAG, "LED %s", state ? "ON" : "OFF");
    }
}

/**
 * @brief 风扇开关回调
 */
static void fan_switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    
    // 更新全局控制参数
    g_control_params.fan_state = state ? 1 : 0;
    
    // 发送控制命令到STM32
    uart_command_t cmd = {
        .type = CMD_FAN_CONTROL,
        .value = state ? 1 : 0
    };
    
    if (xQueueSend(control_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue fan command");
    } else {
        ESP_LOGI(TAG, "Fan %s", state ? "ON" : "OFF");
    }
}

/**
 * @brief 水泵开关回调
 */
static void pump_switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    
    // 更新全局控制参数
    g_control_params.pump_state = state ? 1 : 0;
    
    // 发送控制命令到STM32
    uart_command_t cmd = {
        .type = CMD_WATER_PUMP,
        .value = state ? 1 : 0
    };
    
    if (xQueueSend(control_cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue pump command");
    } else {
        ESP_LOGI(TAG, "Water pump %s", state ? "ON" : "OFF");
    }
}

// ==================== UI更新函数实现 ====================

/**
 * @brief UI更新定时器回调函数
 */
static void ui_update_timer_callback(lv_timer_t *timer) {
    // 更新状态栏
    if (g_ui_components.status_label) {
        char status_text[64];
        snprintf(status_text, sizeof(status_text), 
                "WiFi:%s MQTT:%s STM32:%s",
                g_system_status.wifi_connected ? "OK" : "NO",
                g_system_status.mqtt_connected ? "OK" : "NO",
                g_system_status.stm32_connected ? "OK" : "NO");
        lv_label_set_text(g_ui_components.status_label, status_text);
    }
    
    // 更新控制部件状态（确保UI与实际情况同步）
    if (g_ui_components.motor_slider) {
        lv_slider_set_value(g_ui_components.motor_slider, 
                          g_control_params.motor_speed, LV_ANIM_OFF);
    }
    
    if (g_ui_components.led_switch) {
        if (g_control_params.led_state) {
            lv_obj_add_state(g_ui_components.led_switch, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(g_ui_components.led_switch, LV_STATE_CHECKED);
        }
    }
    
    if (g_ui_components.fan_switch) {
        if (g_control_params.fan_state) {
            lv_obj_add_state(g_ui_components.fan_switch, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(g_ui_components.fan_switch, LV_STATE_CHECKED);
        }
    }
    
    if (g_ui_components.pump_switch) {
        if (g_control_params.pump_state) {
            lv_obj_add_state(g_ui_components.pump_switch, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(g_ui_components.pump_switch, LV_STATE_CHECKED);
        }
    }
    
    // 更新电机值标签
    if (g_ui_components.motor_value_label) {
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%d", g_control_params.motor_speed);
        lv_label_set_text(g_ui_components.motor_value_label, buffer);
    }
}

/**
 * @brief 更新UI显示（兼容旧接口）
 */
static void update_ui_display(void) {
    // 空函数，实际功能在定时器回调中实现
    // 保留此函数是为了兼容之前的代码调用
}

// ==================== UI创建函数实现 ====================

/**
 * @brief 创建温室控制UI界面（使用正确的LVGL符号）
 */
static void create_greenhouse_ui(void) {
    ESP_LOGI(TAG, "Creating greenhouse UI...");
    
    // 获取当前屏幕
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    
    // ========== 顶部状态栏 ==========
    lv_obj_t *status_bar = lv_obj_create(scr);
    lv_obj_set_size(status_bar, LV_PCT(100), 40);
    lv_obj_set_style_bg_color(status_bar, lv_color_hex(0x2D2D2D), 0);
    lv_obj_set_style_border_width(status_bar, 0, 0);
    lv_obj_align(status_bar, LV_ALIGN_TOP_MID, 0, 0);
    
    // 系统名称
    lv_obj_t *sys_name = lv_label_create(status_bar);
    lv_label_set_text(sys_name, SYSTEM_NAME);
    lv_obj_set_style_text_color(sys_name, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(sys_name, &lv_font_montserrat_14, 0);
    lv_obj_align(sys_name, LV_ALIGN_LEFT_MID, 10, 0);
    
    // 连接状态图标（使用正确的LVGL符号）
    g_ui_components.wifi_icon = lv_label_create(status_bar);
    lv_label_set_text(g_ui_components.wifi_icon, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(g_ui_components.wifi_icon, lv_color_hex(0xFF0000), 0);
    lv_obj_align(g_ui_components.wifi_icon, LV_ALIGN_RIGHT_MID, -80, 0);
    
    g_ui_components.mqtt_icon = lv_label_create(status_bar);
    lv_label_set_text(g_ui_components.mqtt_icon, LV_SYMBOL_CHARGE);
    lv_obj_set_style_text_color(g_ui_components.mqtt_icon, lv_color_hex(0xFF0000), 0);
    lv_obj_align(g_ui_components.mqtt_icon, LV_ALIGN_RIGHT_MID, -50, 0);
    
    g_ui_components.stm32_icon = lv_label_create(status_bar);
    lv_label_set_text(g_ui_components.stm32_icon, LV_SYMBOL_USB);
    lv_obj_set_style_text_color(g_ui_components.stm32_icon, lv_color_hex(0xFF0000), 0);
    lv_obj_align(g_ui_components.stm32_icon, LV_ALIGN_RIGHT_MID, -20, 0);
    
    // ========== 标题 ==========
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "智能温室控制系统");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 50);
    
    // ========== 传感器数据显示区域 ==========
    lv_obj_t *sensor_container = lv_obj_create(scr);
    lv_obj_set_size(sensor_container, LV_PCT(90), 160);
    lv_obj_set_style_bg_color(sensor_container, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_border_width(sensor_container, 1, 0);
    lv_obj_set_style_border_color(sensor_container, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(sensor_container, 10, 0);
    lv_obj_align(sensor_container, LV_ALIGN_TOP_MID, 0, 100);
    
    // 温度显示
    lv_obj_t *temp_container = lv_obj_create(sensor_container);
    lv_obj_set_size(temp_container, LV_PCT(45), 70);
    lv_obj_set_style_bg_color(temp_container, lv_color_hex(0x252525), 0);
    lv_obj_set_style_border_width(temp_container, 0, 0);
    lv_obj_set_style_radius(temp_container, 8, 0);
    lv_obj_align(temp_container, LV_ALIGN_TOP_LEFT, 10, 10);
    
    // 使用Unicode温度符号（℃）或者简单的"T"
    lv_obj_t *temp_icon = lv_label_create(temp_container);
    lv_label_set_text(temp_icon, "T");  // 或者使用 "℃" 或 "°C"
    lv_obj_set_style_text_color(temp_icon, lv_color_hex(0xFF6B6B), 0);
    lv_obj_set_style_text_font(temp_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(temp_icon, LV_ALIGN_LEFT_MID, 10, 0);
    
    lv_obj_t *temp_title = lv_label_create(temp_container);
    lv_label_set_text(temp_title, "温度");
    lv_obj_set_style_text_color(temp_title, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(temp_title, temp_icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    
    g_ui_components.temp_label = lv_label_create(temp_container);
    lv_label_set_text(g_ui_components.temp_label, "25.6°C");
    lv_obj_set_style_text_color(g_ui_components.temp_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(g_ui_components.temp_label, &lv_font_montserrat_18, 0);
    lv_obj_align(g_ui_components.temp_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // 湿度显示
    lv_obj_t *hum_container = lv_obj_create(sensor_container);
    lv_obj_set_size(hum_container, LV_PCT(45), 70);
    lv_obj_set_style_bg_color(hum_container, lv_color_hex(0x252525), 0);
    lv_obj_set_style_border_width(hum_container, 0, 0);
    lv_obj_set_style_radius(hum_container, 8, 0);
    lv_obj_align(hum_container, LV_ALIGN_TOP_RIGHT, -10, 10);
    
    // 使用Unicode水滴符号或简单的"H"
    lv_obj_t *hum_icon = lv_label_create(hum_container);
    lv_label_set_text(hum_icon, "H");  // 或者使用 Unicode 水滴符号
    lv_obj_set_style_text_color(hum_icon, lv_color_hex(0x4ECDC4), 0);
    lv_obj_set_style_text_font(hum_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(hum_icon, LV_ALIGN_LEFT_MID, 10, 0);
    
    lv_obj_t *hum_title = lv_label_create(hum_container);
    lv_label_set_text(hum_title, "湿度");
    lv_obj_set_style_text_color(hum_title, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(hum_title, hum_icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    
    g_ui_components.hum_label = lv_label_create(hum_container);
    lv_label_set_text(g_ui_components.hum_label, "60.5%");
    lv_obj_set_style_text_color(g_ui_components.hum_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(g_ui_components.hum_label, &lv_font_montserrat_18, 0);
    lv_obj_align(g_ui_components.hum_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // 光照显示
    lv_obj_t *light_container = lv_obj_create(sensor_container);
    lv_obj_set_size(light_container, LV_PCT(45), 70);
    lv_obj_set_style_bg_color(light_container, lv_color_hex(0x252525), 0);
    lv_obj_set_style_border_width(light_container, 0, 0);
    lv_obj_set_style_radius(light_container, 8, 0);
    lv_obj_align(light_container, LV_ALIGN_BOTTOM_LEFT, 10, -10);
    
    // 使用LVGL的光照符号
    lv_obj_t *light_icon = lv_label_create(light_container);
    lv_label_set_text(light_icon, LV_SYMBOL_EYE_OPEN);  // 使用眼睛符号表示光照监测
    lv_obj_set_style_text_color(light_icon, lv_color_hex(0xFFE66D), 0);
    lv_obj_set_style_text_font(light_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(light_icon, LV_ALIGN_LEFT_MID, 10, 0);
    
    lv_obj_t *light_title = lv_label_create(light_container);
    lv_label_set_text(light_title, "光照");
    lv_obj_set_style_text_color(light_title, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(light_title, light_icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    
    g_ui_components.light_label = lv_label_create(light_container);
    lv_label_set_text(g_ui_components.light_label, "1024");
    lv_obj_set_style_text_color(g_ui_components.light_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(g_ui_components.light_label, &lv_font_montserrat_18, 0);
    lv_obj_align(g_ui_components.light_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // 土壤湿度显示
    lv_obj_t *soil_container = lv_obj_create(sensor_container);
    lv_obj_set_size(soil_container, LV_PCT(45), 70);
    lv_obj_set_style_bg_color(soil_container, lv_color_hex(0x252525), 0);
    lv_obj_set_style_border_width(soil_container, 0, 0);
    lv_obj_set_style_radius(soil_container, 8, 0);
    lv_obj_align(soil_container, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
    
    // 使用简单的"S"表示土壤
    lv_obj_t *soil_icon = lv_label_create(soil_container);
    lv_label_set_text(soil_icon, "S");  // 或者使用其他符号
    lv_obj_set_style_text_color(soil_icon, lv_color_hex(0x9B5DE5), 0);
    lv_obj_set_style_text_font(soil_icon, &lv_font_montserrat_20, 0);
    lv_obj_align(soil_icon, LV_ALIGN_LEFT_MID, 10, 0);
    
    lv_obj_t *soil_title = lv_label_create(soil_container);
    lv_label_set_text(soil_title, "土壤");
    lv_obj_set_style_text_color(soil_title, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(soil_title, soil_icon, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    
    g_ui_components.soil_label = lv_label_create(soil_container);
    lv_label_set_text(g_ui_components.soil_label, "512");
    lv_obj_set_style_text_color(g_ui_components.soil_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(g_ui_components.soil_label, &lv_font_montserrat_18, 0);
    lv_obj_align(g_ui_components.soil_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    // ========== 控制区域 ==========
    lv_obj_t *control_container = lv_obj_create(scr);
    lv_obj_set_size(control_container, LV_PCT(90), 140);
    lv_obj_set_style_bg_color(control_container, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_border_width(control_container, 1, 0);
    lv_obj_set_style_border_color(control_container, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(control_container, 10, 0);
    lv_obj_align(control_container, LV_ALIGN_TOP_MID, 0, 270);
    
    // 电机速度控制
    lv_obj_t *motor_label = lv_label_create(control_container);
    lv_label_set_text(motor_label, "电机速度:");
    lv_obj_set_style_text_color(motor_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(motor_label, LV_ALIGN_TOP_LEFT, 10, 10);
    
    g_ui_components.motor_slider = lv_slider_create(control_container);
    lv_slider_set_range(g_ui_components.motor_slider, 0, 1000);
    lv_slider_set_value(g_ui_components.motor_slider, 500, LV_ANIM_OFF);
    lv_obj_set_width(g_ui_components.motor_slider, 150);
    lv_obj_align_to(g_ui_components.motor_slider, motor_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    lv_obj_add_event_cb(g_ui_components.motor_slider, motor_slider_event_cb, 
                       LV_EVENT_VALUE_CHANGED, NULL);
    
    g_ui_components.motor_value_label = lv_label_create(control_container);
    lv_label_set_text(g_ui_components.motor_value_label, "500");
    lv_obj_set_style_text_color(g_ui_components.motor_value_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align_to(g_ui_components.motor_value_label, g_ui_components.motor_slider, 
                   LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    
    // LED控制
    lv_obj_t *led_label = lv_label_create(control_container);
    lv_label_set_text(led_label, "补光灯:");
    lv_obj_set_style_text_color(led_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(led_label, LV_ALIGN_TOP_LEFT, 10, 50);
    
    g_ui_components.led_switch = lv_switch_create(control_container);
    lv_obj_align_to(g_ui_components.led_switch, led_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    lv_obj_add_event_cb(g_ui_components.led_switch, led_switch_event_cb, 
                       LV_EVENT_VALUE_CHANGED, NULL);
    
    // 风扇控制
    lv_obj_t *fan_label = lv_label_create(control_container);
    lv_label_set_text(fan_label, "通风扇:");
    lv_obj_set_style_text_color(fan_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(fan_label, LV_ALIGN_TOP_RIGHT, -150, 10);
    
    g_ui_components.fan_switch = lv_switch_create(control_container);
    lv_obj_align_to(g_ui_components.fan_switch, fan_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    lv_obj_add_event_cb(g_ui_components.fan_switch, fan_switch_event_cb, 
                       LV_EVENT_VALUE_CHANGED, NULL);
    
    // 水泵控制
    lv_obj_t *pump_label = lv_label_create(control_container);
    lv_label_set_text(pump_label, "水泵:");
    lv_obj_set_style_text_color(pump_label, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(pump_label, LV_ALIGN_TOP_RIGHT, -150, 50);
    
    g_ui_components.pump_switch = lv_switch_create(control_container);
    lv_obj_align_to(g_ui_components.pump_switch, pump_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    lv_obj_add_event_cb(g_ui_components.pump_switch, pump_switch_event_cb, 
                       LV_EVENT_VALUE_CHANGED, NULL);
    
    // ========== 状态显示区域 ==========
    lv_obj_t *status_container = lv_obj_create(scr);
    lv_obj_set_size(status_container, LV_PCT(90), 50);
    lv_obj_set_style_bg_color(status_container, lv_color_hex(0x1A1A1A), 0);
    lv_obj_set_style_border_width(status_container, 1, 0);
    lv_obj_set_style_border_color(status_container, lv_color_hex(0x333333), 0);
    lv_obj_set_style_radius(status_container, 10, 0);
    lv_obj_align(status_container, LV_ALIGN_BOTTOM_MID, 0, -10);
    
    g_ui_components.status_label = lv_label_create(status_container);
    lv_label_set_text(g_ui_components.status_label, "初始化中...");
    lv_obj_set_style_text_color(g_ui_components.status_label, lv_color_hex(0xAAAAAA), 0);
    lv_obj_center(g_ui_components.status_label);
    
    // 创建UI更新定时器（使用函数指针方式）
    lv_timer_create(ui_update_timer_callback, 1000, NULL); // 每秒更新一次
    
    ESP_LOGI(TAG, "Greenhouse UI created successfully");
}

// ==================== 应用程序函数实现 ====================

/**
 * @brief 应用程序初始化
 */
static void app_init(void) {
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   Smart Greenhouse Control System");
    ESP_LOGI(TAG, "   Version: %s", SYSTEM_VERSION);
    ESP_LOGI(TAG, "   ESP32 Chip ID: 0x%08X", (uint32_t)esp_efuse_get_chip_ver());
    ESP_LOGI(TAG, "==========================================");
    
    // 初始化NVS
    ESP_ERROR_CHECK(system_init_nvs());
    
    // 初始化系统资源
    ESP_ERROR_CHECK(system_init_resources());
    
    // 初始化串口管理器
    ESP_LOGI(TAG, "Initializing UART Manager...");
    ESP_ERROR_CHECK(uart_manager_init());
    
    // 初始化LVGL驱动
    ESP_LOGI(TAG, "Initializing LVGL Driver...");
    lvgl_driver_config_t lvgl_config = {
        .width = 240,
        .height = 320,
        .rotation = 0,
        .use_double_buffer = true,
        .use_full_refresh = false,
        .touch_enabled = true,
        .touch_threshold = 45,
        .flush_callback = NULL,
        .task_callback = NULL,
        .user_data = NULL
    };
    
    ESP_ERROR_CHECK(lvgl_driver_init(&lvgl_config));
    
    // 启动LVGL任务（在lvgl_driver内部）
    lvgl_driver_start_task();
    
    // 创建温室UI界面
    create_greenhouse_ui();
    
    // 标记系统准备就绪
    if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        g_system_status.system_ready = true;
        xSemaphoreGive(system_mutex);
    }
    
    ESP_LOGI(TAG, "System initialization completed");
    ESP_LOGI(TAG, "Free heap: %"PRIu32" bytes", esp_get_free_heap_size());
}

/**
 * @brief 创建系统任务
 */
static void app_create_tasks(void) {
    ESP_LOGI(TAG, "Creating system tasks...");
    
    // 1. 串口发送任务
    xTaskCreate(uart_send_task, "uart_send", STACK_SIZE_UART_TX, NULL, 
                TASK_PRIORITY_UART_TX, &task_handles[0]);
    
    // 2. 串口接收任务
    xTaskCreate(uart_receive_task, "uart_recv", STACK_SIZE_UART_RX, NULL, 
                TASK_PRIORITY_UART_RX, &task_handles[1]);
    
    // 3. LVGL显示任务（已在lvgl_driver中创建）
    // 我们不需要额外创建，但这里记录任务句柄位置
    
    // 4. WiFi/MQTT任务
    xTaskCreate(wifi_mqtt_task, "wifi_mqtt", STACK_SIZE_MQTT, NULL, 
                TASK_PRIORITY_MQTT, &task_handles[3]);
    
    // 5. 传感器数据处理任务
    xTaskCreate(sensor_data_process_task, "data_proc", STACK_SIZE_DATA_PROC, NULL, 
                TASK_PRIORITY_DATA_PROC, &task_handles[4]);
    
    // 6. 系统监控任务
    xTaskCreate(system_monitor_task, "sys_mon", STACK_SIZE_SYS_MON, NULL, 
                TASK_PRIORITY_SYS_MON, &task_handles[5]);
    
    ESP_LOGI(TAG, "All system tasks created");
}

// ==================== 主函数实现 ====================

/**
 * @brief 应用程序主入口
 */
void app_main(void) {
    // 应用程序初始化
    app_init();
    
    // 创建系统任务
    app_create_tasks();
    
    // 主循环（低优先级运行）
    ESP_LOGI(TAG, "=== System Started Successfully ===");
    ESP_LOGI(TAG, "Entering main loop...");
    
    while (1) {
        // 主任务保持空闲，低优先级运行
        vTaskDelay(pdMS_TO_TICKS(10000));
        
        // 定期输出系统状态
        static uint32_t last_main_log = 0;
        uint32_t now = esp_timer_get_time() / 1000;
        
        if (now - last_main_log > 30000) { // 每30秒输出一次
            ESP_LOGI(TAG, "Main loop heartbeat - System running normally");
            
            // 输出任务状态
            char task_list[512];
            vTaskList(task_list);
            ESP_LOGD(TAG, "Task List:\n%s", task_list);
            
            last_main_log = now;
        }
        
        // 检查系统是否准备就绪
        if (xSemaphoreTake(system_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (!g_system_status.system_ready) {
                ESP_LOGW(TAG, "System not ready, waiting for initialization...");
            }
            xSemaphoreGive(system_mutex);
        }
    }
}