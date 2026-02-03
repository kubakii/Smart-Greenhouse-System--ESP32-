#ifndef __WIFI_MANAGER_H__
#define __WIFI_MANAGER_H__

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi配置宏定义（可以通过menuconfig配置）
#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "Greenhouse_WIFI"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "greenhouse123"
#endif

#ifndef CONFIG_WIFI_MAX_RETRY
#define CONFIG_WIFI_MAX_RETRY 5
#endif

// WiFi事件组标志位
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
#define WIFI_DISCONNECTED_BIT BIT2
#define WIFI_GOT_IP_BIT    BIT3
#define WIFI_LOST_IP_BIT   BIT4

// WiFi状态枚举
typedef enum {
    WIFI_STATE_UNINIT = 0,     // 未初始化
    WIFI_STATE_INIT,           // 已初始化
    WIFI_STATE_STARTED,        // WiFi已启动
    WIFI_STATE_CONNECTING,     // 连接中
    WIFI_STATE_CONNECTED,      // 已连接
    WIFI_STATE_GOT_IP,         // 已获取IP
    WIFI_STATE_DISCONNECTED,   // 已断开
    WIFI_STATE_ERROR           // 错误状态
} wifi_manager_state_t;

// WiFi配置结构体
typedef struct {
    char ssid[32];              // WiFi SSID
    char password[64];          // WiFi密码
    bool auto_connect;          // 是否自动连接
    uint8_t max_retry;          // 最大重试次数
    uint8_t retry_count;        // 当前重试次数
    wifi_auth_mode_t auth_mode; // 认证模式
    wifi_manager_state_t state; // WiFi状态
} wifi_manager_config_t;

// WiFi信息结构体
typedef struct {
    char ssid[32];              // 当前连接的SSID
    uint8_t bssid[6];           // BSSID (MAC地址)
    int8_t rssi;                // 信号强度
    wifi_second_chan_t second;  // 二级信道
    char ip_addr[16];           // IP地址
    char gateway[16];           // 网关地址
    char netmask[16];           // 子网掩码
    char dns1[16];              // 主DNS
    char dns2[16];              // 备用DNS
} wifi_manager_info_t;

// WiFi事件回调函数类型
typedef void (*wifi_event_callback_t)(int32_t event_id, void* event_data);
typedef void (*wifi_connected_callback_t)(void* arg);
typedef void (*wifi_disconnected_callback_t)(void* arg, uint8_t reason);
typedef void (*wifi_got_ip_callback_t)(void* arg, esp_netif_ip_info_t* ip_info);

// WiFi管理器句柄
typedef struct {
    wifi_manager_config_t config;           // 配置
    wifi_manager_info_t info;               // 连接信息
    EventGroupHandle_t wifi_event_group;    // WiFi事件组
    esp_netif_t* esp_netif;                // 网络接口
    bool initialized;                       // 初始化标志
    bool connected;                         // 连接标志
    TaskHandle_t reconnect_task_handle;     // 重连任务句柄
    
    // 回调函数
    wifi_connected_callback_t connected_cb;
    wifi_disconnected_callback_t disconnected_cb;
    wifi_got_ip_callback_t got_ip_cb;
    void* callback_arg;                     // 回调参数
} wifi_manager_handle_t;

/**
 * @brief 初始化WiFi管理器
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_init(void);

/**
 * @brief 使用默认配置启动WiFi连接
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_start(void);

/**
 * @brief 使用自定义配置启动WiFi连接
 * @param ssid WiFi SSID
 * @param password WiFi密码
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_start_with_config(const char* ssid, const char* password);

/**
 * @brief 停止WiFi连接
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_stop(void);

/**
 * @brief 重启WiFi连接
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_restart(void);

/**
 * @brief 检查WiFi是否已连接
 * @return true 已连接
 * @return false 未连接
 */
bool wifi_is_connected(void);

/**
 * @brief 获取WiFi连接状态
 * @return wifi_manager_state_t WiFi状态
 */
wifi_manager_state_t wifi_manager_get_state(void);

/**
 * @brief 获取WiFi连接信息
 * @param info WiFi信息输出指针
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_get_info(wifi_manager_info_t* info);

/**
 * @brief 获取IP地址字符串
 * @return const char* IP地址字符串
 */
const char* wifi_manager_get_ip_address(void);

/**
 * @brief 获取信号强度
 * @return int8_t RSSI值 (负值)
 */
int8_t wifi_manager_get_rssi(void);

/**
 * @brief 扫描可用的WiFi网络
 * @param ap_list 接入点列表输出
 * @param max_aps 最大接入点数
 * @param num_aps 实际扫描到的接入点数输出
 * @return esp_err_t 错误码
 */
esp_err_t wifi_manager_scan(wifi_ap_record_t* ap_list, uint16_t max_aps, uint16_t* num_aps);

/**
 * @brief 设置连接回调函数
 * @param connected_cb 连接成功回调
 * @param disconnected_cb 断开连接回调
 * @param got_ip_cb 获取IP回调
 * @param arg 回调参数
 */
void wifi_manager_set_callbacks(wifi_connected_callback_t connected_cb,
                                wifi_disconnected_callback_t disconnected_cb,
                                wifi_got_ip_callback_t got_ip_cb,
                                void* arg);

/**
 * @brief 设置WiFi自动重连
 * @param enable 是否启用自动重连
 */
void wifi_manager_set_auto_reconnect(bool enable);

/**
 * @brief 获取WiFi管理器实例句柄
 * @return wifi_manager_handle_t* WiFi管理器句柄
 */
wifi_manager_handle_t* wifi_manager_get_instance(void);

/**
 * @brief 等待WiFi连接
 * @param timeout_ms 超时时间（毫秒）
 * @return true 连接成功
 * @return false 连接失败或超时
 */
bool wifi_manager_wait_for_connected(uint32_t timeout_ms);

/**
 * @brief 打印WiFi状态信息
 */
void wifi_manager_print_status(void);

/**
 * @brief 获取默认WiFi配置
 * @return wifi_manager_config_t 默认配置
 */
wifi_manager_config_t wifi_manager_get_default_config(void);

/**
 * @brief WiFi诊断信息
 * @param buffer 输出缓冲区
 * @param buffer_size 缓冲区大小
 * @return int 实际写入的字符数
 */
int wifi_manager_get_diagnostics(char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // __WIFI_MANAGER_H__