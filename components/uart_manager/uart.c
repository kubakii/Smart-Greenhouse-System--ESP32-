#include "uart.h"

static const char *TAG = "UART_MANAGER";
static QueueHandle_t uart_send_queue = NULL;
static QueueHandle_t uart_receive_queue = NULL;
static TaskHandle_t uart_send_task_handle = NULL;
static TaskHandle_t uart_receive_task_handle = NULL;
static data_received_callback_t data_callback = NULL;

// 解析STM32发送的数据
static void parse_sensor_data(const char *data, sensor_data_t *sensor_data) {
    // 解析格式: "TEMP:25.6,HUM:60.5,LIGHT:1024,SOIL:512"
    char temp_str[32], hum_str[32], light_str[32], soil_str[32];
    
    if (sscanf(data, "TEMP:%f,HUM:%f,LIGHT:%hu,SOIL:%hu",
               &sensor_data->temperature,
               &sensor_data->humidity,
               &sensor_data->light_level,
               &sensor_data->soil_moisture) == 4) {
        sensor_data->timestamp = esp_timer_get_time() / 1000; // ms
        ESP_LOGD(TAG, "Parsed: Temp=%.1f, Hum=%.1f, Light=%d, Soil=%d",
                sensor_data->temperature, sensor_data->humidity,
                sensor_data->light_level, sensor_data->soil_moisture);
    }
}

// 串口发送任务
static void uart_send_task(void *arg) {
    uart_command_t cmd;
    char tx_buffer[128];
    
    ESP_LOGI(TAG, "UART Send Task started");
    
    while (1) {
        // 从队列接收命令
        if (xQueueReceive(uart_send_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            // 根据命令类型格式化数据
            switch (cmd.type) {
                case CMD_MOTOR_SPEED:
                    snprintf(tx_buffer, sizeof(tx_buffer), "MOTOR,%d\n", cmd.value);
                    break;
                case CMD_LED_CONTROL:
                    snprintf(tx_buffer, sizeof(tx_buffer), "LED,%d\n", cmd.value);
                    break;
                case CMD_FAN_CONTROL:
                    snprintf(tx_buffer, sizeof(tx_buffer), "FAN,%d\n", cmd.value);
                    break;
                case CMD_WATER_PUMP:
                    snprintf(tx_buffer, sizeof(tx_buffer), "PUMP,%s\n", cmd.param);
                    break;
                case CMD_GET_STATUS:
                    snprintf(tx_buffer, sizeof(tx_buffer), "STATUS\n");
                    break;
                default:
                    continue;
            }
            
            // 发送数据
            uart_write_bytes(UART_NUM, tx_buffer, strlen(tx_buffer));
            ESP_LOGD(TAG, "Sent: %s", tx_buffer);
        }
    }
}

// 串口接收任务
static void uart_receive_task(void *arg) {
    ESP_LOGI(TAG, "UART Receive Task started");
    
    uint8_t *data = (uint8_t *) malloc(RD_BUF_SIZE + 1);
    uint8_t rx_buffer[128];
    int rx_length = 0;
    
    while (1) {
        // 读取串口数据
        int len = uart_read_bytes(UART_NUM, rx_buffer, sizeof(rx_buffer) - 1, 20 / portTICK_PERIOD_MS);
        
        if (len > 0) {
            rx_buffer[len] = '\0'; // 添加字符串结束符
            
            // 处理完整的数据帧（假设以换行符结束）
            for (int i = 0; i < len; i++) {
                if (rx_buffer[i] == '\n' || rx_length >= RD_BUF_SIZE) {
                    data[rx_length] = '\0';
                    
                    // 解析数据
                    sensor_data_t sensor_data;
                    parse_sensor_data((char *)data, &sensor_data);
                    
                    // 调用回调函数
                    if (data_callback != NULL) {
                        data_callback(&sensor_data);
                    }
                    
                    // 发送到接收队列（供其他任务使用）
                    if (uart_receive_queue != NULL) {
                        xQueueSend(uart_receive_queue, &sensor_data, 0);
                    }
                    
                    rx_length = 0;
                } else {
                    data[rx_length++] = rx_buffer[i];
                }
            }
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    free(data);
}

// 初始化串口管理器
esp_err_t uart_manager_init(void) {
    esp_err_t ret = ESP_OK;
    
    // 配置串口参数
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    // 安装UART驱动程序
    ret = uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed");
        return ret;
    }
    
    ret = uart_param_config(UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed");
        return ret;
    }
    
    ret = uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed");
        return ret;
    }
    
    // 创建队列
    uart_send_queue = xQueueCreate(10, sizeof(uart_command_t));
    uart_receive_queue = xQueueCreate(20, sizeof(sensor_data_t));
    
    if (uart_send_queue == NULL || uart_receive_queue == NULL) {
        ESP_LOGE(TAG, "Queue creation failed");
        return ESP_FAIL;
    }
    
    // 创建任务
    xTaskCreate(uart_send_task, "uart_send", 4096, NULL, 14, &uart_send_task_handle);
    xTaskCreate(uart_receive_task, "uart_recv", 4096, NULL, 18, &uart_receive_task_handle);
    
    ESP_LOGI(TAG, "UART Manager initialized");
    return ESP_OK;
}

// 发送命令到STM32
esp_err_t uart_send_command(uart_command_t *cmd) {
    if (uart_send_queue == NULL || cmd == NULL) {
        return ESP_FAIL;
    }
    
    if (xQueueSend(uart_send_queue, cmd, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Send queue full");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

// 设置接收回调
void uart_set_receive_callback(data_received_callback_t callback) {
    data_callback = callback;
}

// 获取发送队列
QueueHandle_t get_uart_send_queue(void) {
    return uart_send_queue;
}

// 获取接收队列
QueueHandle_t get_uart_receive_queue(void) {
    return uart_receive_queue;
}