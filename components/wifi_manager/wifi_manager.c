#include "wifi_manager.h"

// 日志标签
static const char* TAG = "WIFI_MANAGER";

// WiFi管理器全局实例
static wifi_manager_handle_t g_wifi_manager = {0};

// 默认配置
static const wifi_manager_config_t default_config = {
    .ssid = CONFIG_WIFI_SSID,
    .password = CONFIG_WIFI_PASSWORD,
    .auto_connect = true,
    .max_retry = CONFIG_WIFI_MAX_RETRY,
    .retry_count = 0,
    .auth_mode = WIFI_AUTH_WPA2_PSK,
    .state = WIFI_STATE_UNINIT
};

// 静态函数声明
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data);
static void wifi_reconnect_task(void* arg);
static void wifi_start_reconnect_task(void);
static void wifi_stop_reconnect_task(void);
static esp_err_t wifi_connect_sta(void);
static void wifi_update_connection_info(void);
static void wifi_save_config_to_nvs(void);
static void wifi_load_config_from_nvs(void);

/**
 * @brief WiFi事件处理函数
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    wifi_manager_handle_t* wifi = &g_wifi_manager;
    
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                ESP_LOGI(TAG, "WiFi STA started");
                wifi->config.state = WIFI_STATE_STARTED;
                break;
                
            case WIFI_EVENT_STA_STOP:
                ESP_LOGI(TAG, "WiFi STA stopped");
                wifi->config.state = WIFI_STATE_INIT;
                break;
                
            case WIFI_EVENT_STA_CONNECTED: {
                wifi_event_sta_connected_t* event = (wifi_event_sta_connected_t*) event_data;
                ESP_LOGI(TAG, "Connected to %s (BSSID: %02X:%02X:%02X:%02X:%02X:%02X)",
                event->ssid,
                event->bssid[0], event->bssid[1], event->bssid[2],
                event->bssid[3], event->bssid[4], event->bssid[5]);
                
                
                // 保存连接信息
                strncpy(wifi->info.ssid, (char*)event->ssid, sizeof(wifi->info.ssid) - 1);
                memcpy(wifi->info.bssid, event->bssid, sizeof(wifi->info.bssid));
                wifi->info.rssi = 0; // 初始化为0，稍后更新
                
                wifi->config.state = WIFI_STATE_CONNECTED;
                wifi->connected = true;
                wifi->config.retry_count = 0; // 重置重试计数
                
                // 设置事件标志
                if (wifi->wifi_event_group) {
                    xEventGroupSetBits(wifi->wifi_event_group, WIFI_CONNECTED_BIT);
                }
                
                // 调用连接回调
                if (wifi->connected_cb) {
                    wifi->connected_cb(wifi->callback_arg);
                }
                break;
            }
                
            case WIFI_EVENT_STA_DISCONNECTED: {
                wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
                ESP_LOGW(TAG, "Disconnected from AP: %s, reason: %d", 
                        event->ssid, event->reason);
                
                wifi->config.state = WIFI_STATE_DISCONNECTED;
                wifi->connected = false;
                
                // 设置事件标志
                if (wifi->wifi_event_group) {
                    xEventGroupSetBits(wifi->wifi_event_group, WIFI_DISCONNECTED_BIT);
                }
                
                // 递增重试计数
                wifi->config.retry_count++;
                
                // 检查是否超过最大重试次数
                if (wifi->config.retry_count > wifi->config.max_retry) {
                    ESP_LOGE(TAG, "Max retry count reached, giving up");
                    if (wifi->wifi_event_group) {
                        xEventGroupSetBits(wifi->wifi_event_group, WIFI_FAIL_BIT);
                    }
                } else {
                    ESP_LOGI(TAG, "Retry %d/%d to connect to AP", 
                            wifi->config.retry_count, wifi->config.max_retry);
                    // 自动重连
                    if (wifi->config.auto_connect) {
                        esp_err_t err = esp_wifi_connect();
                        if (err != ESP_OK) {
                            ESP_LOGE(TAG, "Failed to initiate reconnection: %s", esp_err_to_name(err));
                        }
                    }
                }
                
                // 调用断开连接回调
                if (wifi->disconnected_cb) {
                    wifi->disconnected_cb(wifi->callback_arg, event->reason);
                }
                break;
            }
                
            case WIFI_EVENT_STA_AUTHMODE_CHANGE:
                ESP_LOGI(TAG, "Authentication mode changed");
                break;
                
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                
                // 更新IP信息
                esp_netif_ip_info_t* ip_info = &event->ip_info;
                wifi->config.state = WIFI_STATE_GOT_IP;
                
                // 保存IP信息
                sprintf(wifi->info.ip_addr, IPSTR, IP2STR(&ip_info->ip));
                sprintf(wifi->info.gateway, IPSTR, IP2STR(&ip_info->gw));
                sprintf(wifi->info.netmask, IPSTR, IP2STR(&ip_info->netmask));
                
                // 获取DNS信息
                esp_netif_dns_info_t dns_info;
                if (esp_netif_get_dns_info(wifi->esp_netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK) {
                    sprintf(wifi->info.dns1, IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
                }
                if (esp_netif_get_dns_info(wifi->esp_netif, ESP_NETIF_DNS_BACKUP, &dns_info) == ESP_OK) {
                    sprintf(wifi->info.dns2, IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
                }
                
                // 更新信号强度
                wifi_update_connection_info();
                
                // 设置事件标志
                if (wifi->wifi_event_group) {
                    xEventGroupSetBits(wifi->wifi_event_group, WIFI_GOT_IP_BIT);
                }
                
                // 调用获取IP回调
                if (wifi->got_ip_cb) {
                    wifi->got_ip_cb(wifi->callback_arg, ip_info);
                }
                
                // 保存配置到NVS
                wifi_save_config_to_nvs();
                break;
            }
                
            case IP_EVENT_STA_LOST_IP:
                ESP_LOGW(TAG, "Lost IP address");
                wifi->config.state = WIFI_STATE_CONNECTED; // 回到连接状态
                if (wifi->wifi_event_group) {
                    xEventGroupSetBits(wifi->wifi_event_group, WIFI_LOST_IP_BIT);
                }
                break;
                
            default:
                break;
        }
    }
}

/**
 * @brief WiFi自动重连任务
 */
static void wifi_reconnect_task(void* arg) {
    ESP_LOGI(TAG, "WiFi reconnect task started");
    
    while (1) {
        // 等待断开连接事件
        EventBits_t bits = xEventGroupWaitBits(g_wifi_manager.wifi_event_group,
                                              WIFI_DISCONNECTED_BIT | WIFI_FAIL_BIT,
                                              pdFALSE, pdFALSE, portMAX_DELAY);
        
        if (bits & WIFI_FAIL_BIT) {
            ESP_LOGE(TAG, "WiFi connection failed permanently, stopping reconnect task");
            break;
        }
        
        if (bits & WIFI_DISCONNECTED_BIT) {
            // 清除断开连接标志
            xEventGroupClearBits(g_wifi_manager.wifi_event_group, WIFI_DISCONNECTED_BIT);
            
            // 延迟一段时间后尝试重连
            vTaskDelay(pdMS_TO_TICKS(3000));
            
            if (g_wifi_manager.config.auto_connect) {
                ESP_LOGI(TAG, "Attempting to reconnect...");
                esp_err_t err = esp_wifi_connect();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Reconnection attempt failed: %s", esp_err_to_name(err));
                }
            }
        }
        
        // 防止任务占用过多CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    vTaskDelete(NULL);
}

/**
 * @brief 启动WiFi重连任务
 */
static void wifi_start_reconnect_task(void) {
    if (g_wifi_manager.reconnect_task_handle == NULL) {
        xTaskCreate(wifi_reconnect_task, "wifi_reconnect", 4096, NULL, 5, 
                   &g_wifi_manager.reconnect_task_handle);
    }
}

/**
 * @brief 停止WiFi重连任务
 */
static void wifi_stop_reconnect_task(void) {
    if (g_wifi_manager.reconnect_task_handle) {
        vTaskDelete(g_wifi_manager.reconnect_task_handle);
        g_wifi_manager.reconnect_task_handle = NULL;
    }
}

/**
 * @brief 连接到WiFi STA模式
 */
static esp_err_t wifi_connect_sta(void) {
    // 设置WiFi为STA模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // 配置WiFi连接参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    // 复制SSID和密码
    strncpy((char*)wifi_config.sta.ssid, g_wifi_manager.config.ssid, 
            sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, g_wifi_manager.config.password, 
            sizeof(wifi_config.sta.password));
    
    ESP_LOGI(TAG, "Connecting to SSID: %s", g_wifi_manager.config.ssid);
    
    // 应用配置
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // 开始连接
    esp_err_t ret = esp_wifi_connect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initiate WiFi connection: %s", esp_err_to_name(ret));
        return ret;
    }
    
    g_wifi_manager.config.state = WIFI_STATE_CONNECTING;
    ESP_LOGI(TAG, "WiFi connection initiated");
    
    return ESP_OK;
}

/**
 * @brief 更新WiFi连接信息
 */
static void wifi_update_connection_info(void) {
    wifi_ap_record_t ap_info;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_info);
    
    if (err == ESP_OK) {
        g_wifi_manager.info.rssi = ap_info.rssi;
        g_wifi_manager.config.auth_mode = ap_info.authmode;
        
        ESP_LOGD(TAG, "RSSI: %d dBm, Channel: %d", 
                g_wifi_manager.info.rssi, ap_info.primary);
    }
}

/**
 * @brief 保存WiFi配置到NVS
 */
static void wifi_save_config_to_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for writing: %s", esp_err_to_name(err));
        return;
    }
    
    // 保存SSID
    err = nvs_set_str(nvs_handle, "ssid", g_wifi_manager.config.ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID to NVS: %s", esp_err_to_name(err));
    }
    
    // 保存密码
    err = nvs_set_str(nvs_handle, "password", g_wifi_manager.config.password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password to NVS: %s", esp_err_to_name(err));
    }
    
    // 保存自动连接设置
    err = nvs_set_u8(nvs_handle, "auto_connect", g_wifi_manager.config.auto_connect ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save auto_connect to NVS: %s", esp_err_to_name(err));
    }
    
    // 提交更改
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to commit NVS changes: %s", esp_err_to_name(err));
    }
    
    nvs_close(nvs_handle);
    ESP_LOGD(TAG, "WiFi configuration saved to NVS");
}

/**
 * @brief 从NVS加载WiFi配置
 */
static void wifi_load_config_from_nvs(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READONLY, &nvs_handle);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No WiFi configuration found in NVS, using defaults");
        return;
    }
    
    size_t required_size;
    
    // 加载SSID
    err = nvs_get_str(nvs_handle, "ssid", NULL, &required_size);
    if (err == ESP_OK && required_size > 0 && required_size <= sizeof(g_wifi_manager.config.ssid)) {
        nvs_get_str(nvs_handle, "ssid", g_wifi_manager.config.ssid, &required_size);
        ESP_LOGI(TAG, "Loaded SSID from NVS: %s", g_wifi_manager.config.ssid);
    }
    
    // 加载密码
    err = nvs_get_str(nvs_handle, "password", NULL, &required_size);
    if (err == ESP_OK && required_size > 0 && required_size <= sizeof(g_wifi_manager.config.password)) {
        nvs_get_str(nvs_handle, "password", g_wifi_manager.config.password, &required_size);
        ESP_LOGI(TAG, "Loaded password from NVS");
    }
    
    // 加载自动连接设置
    uint8_t auto_connect = 1;
    err = nvs_get_u8(nvs_handle, "auto_connect", &auto_connect);
    if (err == ESP_OK) {
        g_wifi_manager.config.auto_connect = (auto_connect != 0);
        ESP_LOGI(TAG, "Loaded auto_connect from NVS: %s", 
                g_wifi_manager.config.auto_connect ? "true" : "false");
    }
    
    nvs_close(nvs_handle);
    ESP_LOGD(TAG, "WiFi configuration loaded from NVS");
}

/**
 * @brief 初始化WiFi管理器
 */
esp_err_t wifi_manager_init(void) {
    if (g_wifi_manager.initialized) {
        ESP_LOGW(TAG, "WiFi manager already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing WiFi manager...");
    
    // 初始化管理器结构
    memset(&g_wifi_manager, 0, sizeof(g_wifi_manager));
    g_wifi_manager.config = default_config;
    
    // 加载NVS中的配置
    wifi_load_config_from_nvs();
    
    // 创建事件组
    g_wifi_manager.wifi_event_group = xEventGroupCreate();
    if (!g_wifi_manager.wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化TCP/IP协议栈
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 创建默认的STA网络接口
    g_wifi_manager.esp_netif = esp_netif_create_default_wifi_sta();
    if (!g_wifi_manager.esp_netif) {
        ESP_LOGE(TAG, "Failed to create default WiFi STA network interface");
        return ESP_FAIL;
    }
    
    // 初始化WiFi配置
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 注册WiFi事件处理器
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_LOST_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    
    // 设置WiFi为STA模式
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // 设置存储模式
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    
    // 启动WiFi
    ESP_ERROR_CHECK(esp_wifi_start());
    
    g_wifi_manager.initialized = true;
    g_wifi_manager.config.state = WIFI_STATE_INIT;
    
    ESP_LOGI(TAG, "WiFi manager initialized successfully");
    return ESP_OK;
}

/**
 * @brief 使用默认配置启动WiFi连接
 */
esp_err_t wifi_manager_start(void) {
    if (!g_wifi_manager.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    return wifi_connect_sta();
}

/**
 * @brief 使用自定义配置启动WiFi连接
 */
esp_err_t wifi_manager_start_with_config(const char* ssid, const char* password) {
    if (!g_wifi_manager.initialized) {
        ESP_LOGE(TAG, "WiFi manager not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!ssid || strlen(ssid) == 0) {
        ESP_LOGE(TAG, "Invalid SSID");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 更新配置
    strncpy(g_wifi_manager.config.ssid, ssid, sizeof(g_wifi_manager.config.ssid) - 1);
    if (password) {
        strncpy(g_wifi_manager.config.password, password, 
                sizeof(g_wifi_manager.config.password) - 1);
    }
    
    // 重置重试计数
    g_wifi_manager.config.retry_count = 0;
    
    ESP_LOGI(TAG, "Starting WiFi connection with SSID: %s", ssid);
    return wifi_connect_sta();
}

/**
 * @brief 停止WiFi连接
 */
esp_err_t wifi_manager_stop(void) {
    if (!g_wifi_manager.initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    ESP_LOGI(TAG, "Stopping WiFi connection...");
    
    // 停止重连任务
    wifi_stop_reconnect_task();
    
    // 断开连接
    esp_err_t ret = esp_wifi_disconnect();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disconnect WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 停止WiFi
    ret = esp_wifi_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    g_wifi_manager.connected = false;
    g_wifi_manager.config.state = WIFI_STATE_INIT;
    
    ESP_LOGI(TAG, "WiFi stopped");
    return ESP_OK;
}

/**
 * @brief 重启WiFi连接
 */
esp_err_t wifi_manager_restart(void) {
    ESP_LOGI(TAG, "Restarting WiFi connection...");
    
    esp_err_t ret = wifi_manager_stop();
    if (ret != ESP_OK) {
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to restart WiFi: %s", esp_err_to_name(ret));
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
    
    return wifi_manager_start();
}

/**
 * @brief 检查WiFi是否已连接
 */
bool wifi_is_connected(void) {
    return g_wifi_manager.connected && (g_wifi_manager.config.state >= WIFI_STATE_CONNECTED);
}

/**
 * @brief 获取WiFi连接状态
 */
wifi_manager_state_t wifi_manager_get_state(void) {
    return g_wifi_manager.config.state;
}

/**
 * @brief 获取WiFi连接信息
 */
esp_err_t wifi_manager_get_info(wifi_manager_info_t* info) {
    if (!info) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!g_wifi_manager.connected) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memcpy(info, &g_wifi_manager.info, sizeof(wifi_manager_info_t));
    
    // 更新信号强度
    wifi_update_connection_info();
    info->rssi = g_wifi_manager.info.rssi;
    
    return ESP_OK;
}

/**
 * @brief 获取IP地址字符串
 */
const char* wifi_manager_get_ip_address(void) {
    if (!g_wifi_manager.connected || g_wifi_manager.config.state < WIFI_STATE_GOT_IP) {
        return NULL;
    }
    
    return g_wifi_manager.info.ip_addr;
}

/**
 * @brief 获取信号强度
 */
int8_t wifi_manager_get_rssi(void) {
    if (!g_wifi_manager.connected) {
        return 0;
    }
    
    wifi_update_connection_info();
    return g_wifi_manager.info.rssi;
}

/**
 * @brief 扫描可用的WiFi网络
 */
esp_err_t wifi_manager_scan(wifi_ap_record_t* ap_list, uint16_t max_aps, uint16_t* num_aps) {
    if (!ap_list || !num_aps || max_aps == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 配置扫描参数
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300
    };
    
    // 开始扫描
    esp_err_t ret = esp_wifi_scan_start(&scan_config, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi scan: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 获取扫描结果
    uint16_t ap_count = 0;
    ret = esp_wifi_scan_get_ap_num(&ap_count);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get scan result count: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 限制返回数量
    if (ap_count > max_aps) {
        ap_count = max_aps;
    }
    
    // 获取AP记录
    ret = esp_wifi_scan_get_ap_records(&ap_count, ap_list);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get scan records: %s", esp_err_to_name(ret));
        return ret;
    }
    
    *num_aps = ap_count;
    
    ESP_LOGI(TAG, "WiFi scan completed, found %d networks", ap_count);
    return ESP_OK;
}

/**
 * @brief 设置连接回调函数
 */
void wifi_manager_set_callbacks(wifi_connected_callback_t connected_cb,
                                wifi_disconnected_callback_t disconnected_cb,
                                wifi_got_ip_callback_t got_ip_cb,
                                void* arg) {
    g_wifi_manager.connected_cb = connected_cb;
    g_wifi_manager.disconnected_cb = disconnected_cb;
    g_wifi_manager.got_ip_cb = got_ip_cb;
    g_wifi_manager.callback_arg = arg;
}

/**
 * @brief 设置WiFi自动重连
 */
void wifi_manager_set_auto_reconnect(bool enable) {
    g_wifi_manager.config.auto_connect = enable;
    
    if (enable && !g_wifi_manager.reconnect_task_handle) {
        wifi_start_reconnect_task();
    } else if (!enable && g_wifi_manager.reconnect_task_handle) {
        wifi_stop_reconnect_task();
    }
    
    ESP_LOGI(TAG, "WiFi auto-reconnect %s", enable ? "enabled" : "disabled");
}

/**
 * @brief 获取WiFi管理器实例句柄
 */
wifi_manager_handle_t* wifi_manager_get_instance(void) {
    return &g_wifi_manager;
}

/**
 * @brief 等待WiFi连接
 */
bool wifi_manager_wait_for_connected(uint32_t timeout_ms) {
    if (!g_wifi_manager.wifi_event_group) {
        return false;
    }
    
    // 等待连接成功或失败
    EventBits_t bits = xEventGroupWaitBits(g_wifi_manager.wifi_event_group,
                                          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                          pdFALSE, pdFALSE, 
                                          pdMS_TO_TICKS(timeout_ms));
    
    if (bits & WIFI_CONNECTED_BIT) {
        // 继续等待获取IP
        bits = xEventGroupWaitBits(g_wifi_manager.wifi_event_group,
                                  WIFI_GOT_IP_BIT | WIFI_FAIL_BIT,
                                  pdFALSE, pdFALSE,
                                  pdMS_TO_TICKS(5000));
        
        return (bits & WIFI_GOT_IP_BIT) != 0;
    }
    
    return false;
}

/**
 * @brief 打印WiFi状态信息
 */
void wifi_manager_print_status(void) {
    if (!g_wifi_manager.initialized) {
        ESP_LOGI(TAG, "WiFi manager not initialized");
        return;
    }
    
    const char* state_strings[] = {
        "UNINIT",
        "INIT",
        "STARTED",
        "CONNECTING",
        "CONNECTED",
        "GOT_IP",
        "DISCONNECTED",
        "ERROR"
    };
    
    ESP_LOGI(TAG, "=== WiFi Status ===");
    ESP_LOGI(TAG, "State: %s", state_strings[g_wifi_manager.config.state]);
    ESP_LOGI(TAG, "Connected: %s", g_wifi_manager.connected ? "Yes" : "No");
    
    if (g_wifi_manager.connected) {
        ESP_LOGI(TAG, "SSID: %s", g_wifi_manager.info.ssid);
        ESP_LOGI(TAG, "BSSID: %02X:%02X:%02X:%02X:%02X:%02X",
         g_wifi_manager.info.bssid[0],
         g_wifi_manager.info.bssid[1],
         g_wifi_manager.info.bssid[2],
         g_wifi_manager.info.bssid[3],
         g_wifi_manager.info.bssid[4],
         g_wifi_manager.info.bssid[5]);
        ESP_LOGI(TAG, "RSSI: %d dBm", g_wifi_manager.info.rssi);
        
        if (g_wifi_manager.config.state >= WIFI_STATE_GOT_IP) {
            ESP_LOGI(TAG, "IP Address: %s", g_wifi_manager.info.ip_addr);
            ESP_LOGI(TAG, "Gateway: %s", g_wifi_manager.info.gateway);
            ESP_LOGI(TAG, "Netmask: %s", g_wifi_manager.info.netmask);
            ESP_LOGI(TAG, "DNS1: %s", g_wifi_manager.info.dns1[0] ? g_wifi_manager.info.dns1 : "N/A");
            ESP_LOGI(TAG, "DNS2: %s", g_wifi_manager.info.dns2[0] ? g_wifi_manager.info.dns2 : "N/A");
        }
    }
    
    ESP_LOGI(TAG, "Auto-connect: %s", g_wifi_manager.config.auto_connect ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "Retry count: %d/%d", g_wifi_manager.config.retry_count, g_wifi_manager.config.max_retry);
    ESP_LOGI(TAG, "==========================");
}

/**
 * @brief 获取默认WiFi配置
 */
wifi_manager_config_t wifi_manager_get_default_config(void) {
    return default_config;
}

/**
 * @brief WiFi诊断信息
 */
int wifi_manager_get_diagnostics(char* buffer, size_t buffer_size) {
    if (!buffer || buffer_size == 0) {
        return 0;
    }
    
    const char* state_strings[] = {
        "UNINIT", "INIT", "STARTED", "CONNECTING", 
        "CONNECTED", "GOT_IP", "DISCONNECTED", "ERROR"
    };
    
    int written = 0;
    
    written += snprintf(buffer + written, buffer_size - written,
                       "WiFi Diagnostics:\n");
    written += snprintf(buffer + written, buffer_size - written,
                       "  State: %s\n", state_strings[g_wifi_manager.config.state]);
    written += snprintf(buffer + written, buffer_size - written,
                       "  Connected: %s\n", g_wifi_manager.connected ? "Yes" : "No");
    written += snprintf(buffer + written, buffer_size - written,
                       "  Auto-reconnect: %s\n", 
                       g_wifi_manager.config.auto_connect ? "Enabled" : "Disabled");
    written += snprintf(buffer + written, buffer_size - written,
                       "  Retry: %d/%d\n", 
                       g_wifi_manager.config.retry_count, 
                       g_wifi_manager.config.max_retry);
    
    if (g_wifi_manager.connected) {
        written += snprintf(buffer + written, buffer_size - written,
                           "  SSID: %s\n", g_wifi_manager.info.ssid);
        written += snprintf(buffer + written, buffer_size - written,
                           "  RSSI: %d dBm\n", g_wifi_manager.info.rssi);
        
        if (g_wifi_manager.config.state >= WIFI_STATE_GOT_IP) {
            written += snprintf(buffer + written, buffer_size - written,
                               "  IP: %s\n", g_wifi_manager.info.ip_addr);
        }
    }
    
    return written;
}