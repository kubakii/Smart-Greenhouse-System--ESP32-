#ifndef __UART_MANAGER_H__
#define __UART_MANAGER_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"

#define UART_NUM UART_NUM_1
#define UART_TX_PIN GPIO_NUM_17
#define UART_RX_PIN GPIO_NUM_16
#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

// 命令定义
typedef enum {
    CMD_MOTOR_SPEED = 0,
    CMD_LED_CONTROL,
    CMD_FAN_CONTROL,
    CMD_WATER_PUMP,
    CMD_GET_STATUS
} command_type_t;

typedef struct {
    command_type_t type;
    int16_t value;
    char param[32];
} uart_command_t;

// 数据定义
typedef struct {
    float temperature;
    float humidity;
    uint16_t light_level;
    uint16_t soil_moisture;
    uint32_t timestamp;
} sensor_data_t;

// 初始化函数
esp_err_t uart_manager_init(void);

// 发送数据到STM32
esp_err_t uart_send_command(uart_command_t *cmd);
esp_err_t uart_send_raw(const char *data, size_t len);

// 接收数据回调类型
typedef void (*data_received_callback_t)(sensor_data_t *data);

// 设置接收回调
void uart_set_receive_callback(data_received_callback_t callback);

// 获取发送队列（供其他任务使用）
QueueHandle_t get_uart_send_queue(void);

#endif