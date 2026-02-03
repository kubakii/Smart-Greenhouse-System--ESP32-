#include "lvgl_driver.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

static const char* TAG = "LVGL_DRIVER";

// 全局驱动句柄
static lvgl_driver_handle_t g_lvgl_driver = {0};

// 默认配置
const lvgl_driver_config_t lvgl_driver_default_config = {
    .width = LVGL_DISP_HOR_RES,
    .height = LVGL_DISP_VER_RES,
    .rotation = 0,
    .use_double_buffer = true,
    .use_full_refresh = false,
    .touch_enabled = true,
    .touch_threshold = 45,
    .flush_callback = NULL,
    .task_callback = NULL,
    .user_data = NULL
};

// 性能统计
typedef struct {
    uint32_t flush_count;
    uint32_t last_flush_time;
    uint32_t fps;
    uint32_t frame_start_time;
    uint32_t mem_total;
    uint32_t mem_used;
    uint32_t mem_max_used;
} lvgl_perf_stats_t;

static lvgl_perf_stats_t g_perf_stats = {0};

// 静态函数声明
static void lvgl_display_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p);
static void lvgl_display_flush_ready(lv_disp_drv_t* disp_drv);
static void lvgl_touch_read(lv_indev_drv_t* indev_drv, lv_indev_data_t* data);
static void lvgl_task(void* arg);
static void lvgl_initialize_display(const lvgl_driver_config_t* config);
static void lvgl_initialize_touch(const lvgl_driver_config_t* config);
static void lvgl_initialize_buffers(const lvgl_driver_config_t* config);






/**
 * @brief LVGL显示刷新回调函数
 */
static void lvgl_display_flush(lv_disp_drv_t* disp_drv, const lv_area_t* area, lv_color_t* color_p) {
    // 记录帧开始时间
    g_perf_stats.frame_start_time = esp_timer_get_time() / 1000;
    
    // 获取用户数据
    lvgl_display_driver_t* display = (lvgl_display_driver_t*)disp_drv->user_data;
    if (!display) {
        ESP_LOGE(TAG, "Display driver user data is NULL");
        return;
    }
    
    // 更新显示区域
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;
    
    // 调用ST7789的刷新函数
    st7789_flush(x1, x2, y1, y2, (void*)color_p);
    
    // 调用用户回调（如果存在）
    if (g_lvgl_driver.display.disp_drv.flush_cb) {
        g_lvgl_driver.display.disp_drv.flush_cb(&g_lvgl_driver.display.disp_drv, area, color_p);
    }
    
    // 性能统计
    g_perf_stats.flush_count++;
    g_perf_stats.last_flush_time = esp_timer_get_time() / 1000;
    
    // 通知LVGL刷新完成
    lv_disp_flush_ready(disp_drv);
}

/**
 * @brief LVGL触摸读取回调函数
 */
static void lvgl_touch_read(lv_indev_drv_t* indev_drv, lv_indev_data_t* data) {
    lvgl_touch_driver_t* touch = (lvgl_touch_driver_t*)indev_drv->user_data;
    if (!touch) {
        ESP_LOGE(TAG, "Touch driver user data is NULL");
        return;
    }
    
    int16_t x = 0, y = 0;
    bool pressed = false;
    uint8_t gesture = 0;
    
    // 读取触摸数据
    esp_err_t ret = ft6336u_read_simple(&x, &y, &gesture, &pressed);
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Failed to read touch data");
        // 使用上一次的数据
        data->point.x = touch->last_x;
        data->point.y = touch->last_y;
        data->state = touch->is_pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
        return;
    }
    
    // 更新最后触摸点
    touch->last_x = x;
    touch->last_y = y;
    touch->is_pressed = pressed;
    
    // 设置触摸数据
    data->point.x = x;
    data->point.y = y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    
    // 调试输出
    static uint32_t last_log_time = 0;
    uint32_t now = esp_timer_get_time() / 1000;
    if (now - last_log_time > 1000) { // 每秒输出一次
        ESP_LOGD(TAG, "Touch: x=%d, y=%d, pressed=%d, gesture=0x%02X", x, y, pressed, gesture);
        last_log_time = now;
    }
}

/**
 * @brief 初始化显示设备
 */
static void lvgl_initialize_display(const lvgl_driver_config_t* config) {
    ESP_LOGI(TAG, "Initializing ST7789 display...");
    
    // 配置ST7789
    st7789_cfg_t st_config = {
        .mosi = PIN_LCD_MOSI,
        .clk = PIN_LCD_CLK,
        .cs = PIN_LCD_CS,
        .dc = PIN_LCD_DC,
        .rst = PIN_LCD_RST,
        .bl = PIN_LCD_BL,
        .spi_freq = LVGL_SPI_FREQ,
        .width = config->width,
        .height = config->height,
        .rotation = config->rotation,
        .color_mode = 1,  // 16-bit color mode
        .bgr_order = false,
        .done_cb = NULL,
        .cb_param = NULL
    };
    
    // 初始化ST7789
    esp_err_t ret = st7789_init(&st_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ST7789: %s", esp_err_to_name(ret));
        return;
    }
    
    // 保存配置
    g_lvgl_driver.display.st_config = st_config;
    
    ESP_LOGI(TAG, "ST7789 initialized successfully");
}

/**
 * @brief 初始化触摸设备
 */
static void lvgl_initialize_touch(const lvgl_driver_config_t* config) {
    if (!config->touch_enabled) {
        ESP_LOGI(TAG, "Touch disabled by configuration");
        return;
    }
    
    ESP_LOGI(TAG, "Initializing FT6336U touch...");
    
    // 配置FT6336U
    ft6336u_cfg_t ft_config = {
        .scl = PIN_TOUCH_SCL,
        .sda = PIN_TOUCH_SDA,
        .int_pin = PIN_TOUCH_INT,
        .i2c_freq = LVGL_I2C_FREQ,
        .x_limit = config->width,
        .y_limit = config->height,
        .use_int = true
    };
    
    // 初始化FT6336U
    esp_err_t ret = ft6336u_init(&ft_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize FT6336U: %s", esp_err_to_name(ret));
        return;
    }
    
    // 设置触摸阈值
    ft6336u_set_touch_threshold(config->touch_threshold);
    
    // 保存配置
    g_lvgl_driver.touch.ft_config = ft_config;
    
    ESP_LOGI(TAG, "FT6336U initialized successfully");
}

/**
 * @brief 初始化显示缓冲区
 */
static void lvgl_initialize_buffers(const lvgl_driver_config_t* config) {
    ESP_LOGI(TAG, "Initializing LVGL buffers...");
    
    // 计算缓冲区大小
    uint32_t buf_size = LVGL_DISP_HOR_RES * LVGL_BUFFER_SIZE;
    ESP_LOGI(TAG, "Buffer size: %d pixels (%d KB)", 
             buf_size, (buf_size * sizeof(lv_color_t)) / 1024);
    
    // 分配缓冲区内存（优先使用外部RAM）
    g_lvgl_driver.display.buf1 = (lv_color_t*)heap_caps_malloc(
        buf_size * sizeof(lv_color_t), 
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    
    if (!g_lvgl_driver.display.buf1) {
        ESP_LOGW(TAG, "Failed to allocate buffer in SPIRAM, using internal RAM");
        g_lvgl_driver.display.buf1 = (lv_color_t*)malloc(buf_size * sizeof(lv_color_t));
    }
    
    if (!g_lvgl_driver.display.buf1) {
        ESP_LOGE(TAG, "Failed to allocate display buffer");
        return;
    }
    
    // 如果使用双缓冲区
    if (config->use_double_buffer) {
        g_lvgl_driver.display.buf2 = (lv_color_t*)heap_caps_malloc(
            buf_size * sizeof(lv_color_t), 
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        
        if (!g_lvgl_driver.display.buf2) {
            ESP_LOGW(TAG, "Failed to allocate second buffer in SPIRAM, using internal RAM");
            g_lvgl_driver.display.buf2 = (lv_color_t*)malloc(buf_size * sizeof(lv_color_t));
        }
        
        if (!g_lvgl_driver.display.buf2) {
            ESP_LOGW(TAG, "Failed to allocate second buffer, falling back to single buffer");
            free(g_lvgl_driver.display.buf1);
            g_lvgl_driver.display.buf1 = NULL;
            return;
        }
        
        // 初始化双缓冲区
        lv_disp_draw_buf_init(&g_lvgl_driver.display.draw_buf,
                             g_lvgl_driver.display.buf1,
                             g_lvgl_driver.display.buf2,
                             buf_size);
    } else {
        // 初始化单缓冲区
        lv_disp_draw_buf_init(&g_lvgl_driver.display.draw_buf,
                             g_lvgl_driver.display.buf1,
                             NULL,
                             buf_size);
    }
    
    g_lvgl_driver.display.buf_size = buf_size;
    
    // 内存统计
    g_perf_stats.mem_total = esp_get_free_heap_size();
    g_perf_stats.mem_used = buf_size * sizeof(lv_color_t) * 
                           (config->use_double_buffer ? 2 : 1);
    g_perf_stats.mem_max_used = g_perf_stats.mem_used;
    
    ESP_LOGI(TAG, "LVGL buffers initialized: %s buffer, %d KB",
             config->use_double_buffer ? "double" : "single",
             g_perf_stats.mem_used / 1024);
}

/**
 * @brief LVGL任务函数
 */
static void lvgl_task(void* arg) {
    ESP_LOGI(TAG, "LVGL task started");
    
    TickType_t last_wake_time = xTaskGetTickCount();
    
    while (1) {
        // 保护LVGL操作
        if (xSemaphoreTake(g_lvgl_driver.mutex, portMAX_DELAY) == pdTRUE) {
            // 调用LVGL定时器处理
            lv_timer_handler();
            
            // 计算FPS
            static uint32_t last_fps_time = 0;
            static uint32_t frame_count = 0;
            uint32_t now = esp_timer_get_time() / 1000;
            
            frame_count++;
            if (now - last_fps_time >= 1000) { // 每秒计算一次
                g_perf_stats.fps = frame_count;
                frame_count = 0;
                last_fps_time = now;
                
                // 更新内存使用统计
                uint32_t current_used = g_perf_stats.mem_used;
                if (current_used > g_perf_stats.mem_max_used) {
                    g_perf_stats.mem_max_used = current_used;
                }
                
                // 调试输出
                ESP_LOGD(TAG, "LVGL FPS: %d, Mem: %d/%d KB", 
                         g_perf_stats.fps,
                         g_perf_stats.mem_used / 1024,
                         g_perf_stats.mem_total / 1024);
            }
            
            xSemaphoreGive(g_lvgl_driver.mutex);
        }
        
        // 延迟指定的时间
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(LVGL_TICK_HANDLER_DELAY));
        
        // 调用用户回调（如果存在）
        if (g_lvgl_driver.display.disp_drv.user_data) {
            lvgl_driver_config_t* config = (lvgl_driver_config_t*)g_lvgl_driver.display.disp_drv.user_data;
            if (config && config->task_callback) {
                config->task_callback(config->user_data);
            }
        }
    }
}

/**
 * @brief 初始化LVGL驱动
 */
esp_err_t lvgl_driver_init(const lvgl_driver_config_t* config) {
    if (g_lvgl_driver.initialized) {
        ESP_LOGW(TAG, "LVGL driver already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing LVGL driver...");
    
    // 使用默认配置或用户配置
    lvgl_driver_config_t driver_config;
    if (config) {
        driver_config = *config;
    } else {
        driver_config = lvgl_driver_default_config;
    }
    
    // 创建互斥锁
    g_lvgl_driver.mutex = xSemaphoreCreateMutex();
    if (!g_lvgl_driver.mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化LVGL库
    lv_init();
    ESP_LOGI(TAG, "LVGL version: %s", lv_version_info());
    
    // 初始化硬件
    lvgl_initialize_display(&driver_config);
    lvgl_initialize_touch(&driver_config);
    lvgl_initialize_buffers(&driver_config);
    
    if (!g_lvgl_driver.display.buf1) {
        ESP_LOGE(TAG, "Failed to initialize display buffers");
        return ESP_FAIL;
    }
    
    // 初始化显示驱动
    lv_disp_drv_init(&g_lvgl_driver.display.disp_drv);
    
    g_lvgl_driver.display.disp_drv.hor_res = driver_config.width;
    g_lvgl_driver.display.disp_drv.ver_res = driver_config.height;
    g_lvgl_driver.display.disp_drv.draw_buf = &g_lvgl_driver.display.draw_buf;
    g_lvgl_driver.display.disp_drv.flush_cb = lvgl_display_flush;
    g_lvgl_driver.display.disp_drv.full_refresh = driver_config.use_full_refresh;
    g_lvgl_driver.display.disp_drv.user_data = (void*)&driver_config;
    
    // 注册显示驱动
    g_lvgl_driver.display.disp = lv_disp_drv_register(&g_lvgl_driver.display.disp_drv);
    if (!g_lvgl_driver.display.disp) {
        ESP_LOGE(TAG, "Failed to register display driver");
        return ESP_FAIL;
    }
    
    // 初始化触摸驱动（如果启用）
    if (driver_config.touch_enabled) {
        lv_indev_drv_init(&g_lvgl_driver.touch.indev_drv);
        
        g_lvgl_driver.touch.indev_drv.type = LV_INDEV_TYPE_POINTER;
        g_lvgl_driver.touch.indev_drv.read_cb = lvgl_touch_read;
        g_lvgl_driver.touch.indev_drv.user_data = &g_lvgl_driver.touch;
        
        // 注册输入设备
        g_lvgl_driver.touch.indev = lv_indev_drv_register(&g_lvgl_driver.touch.indev_drv);
        if (!g_lvgl_driver.touch.indev) {
            ESP_LOGW(TAG, "Failed to register touch input device");
        } else {
            ESP_LOGI(TAG, "Touch input device registered");
        }
    }
    
    // 设置主题
    lv_theme_t* theme = lv_theme_default_init(g_lvgl_driver.display.disp,
                                             lv_palette_main(LV_PALETTE_BLUE),
                                             lv_palette_main(LV_PALETTE_RED),
                                             LV_THEME_DEFAULT_DARK,
                                             &lv_font_montserrat_14);
    lv_disp_set_theme(g_lvgl_driver.display.disp, theme);
    
    // 设置初始化完成标志
    g_lvgl_driver.initialized = true;
    
    ESP_LOGI(TAG, "LVGL driver initialized successfully");
    ESP_LOGI(TAG, "Screen: %dx%d, Rotation: %d, Buffer: %s",
             driver_config.width, driver_config.height,
             driver_config.rotation,
             driver_config.use_double_buffer ? "double" : "single");
    
    return ESP_OK;
}

/**
 * @brief 启动LVGL任务
 */
void lvgl_driver_start_task(void) {
    if (!g_lvgl_driver.initialized) {
        ESP_LOGE(TAG, "LVGL driver not initialized");
        return;
    }
    
    if (g_lvgl_driver.task_handle) {
        ESP_LOGW(TAG, "LVGL task already running");
        return;
    }
    
    ESP_LOGI(TAG, "Starting LVGL task...");
    
    // 创建LVGL任务
    xTaskCreate(lvgl_task,
                "lvgl_task",
                LVGL_TASK_STACK_SIZE,
                NULL,
                LVGL_TASK_PRIORITY,
                &g_lvgl_driver.task_handle);
    
    if (!g_lvgl_driver.task_handle) {
        ESP_LOGE(TAG, "Failed to create LVGL task");
        return;
    }
    
    ESP_LOGI(TAG, "LVGL task started with priority %d", LVGL_TASK_PRIORITY);
}

/**
 * @brief 停止LVGL任务
 */
void lvgl_driver_stop_task(void) {
    if (g_lvgl_driver.task_handle) {
        vTaskDelete(g_lvgl_driver.task_handle);
        g_lvgl_driver.task_handle = NULL;
        ESP_LOGI(TAG, "LVGL task stopped");
    }
}

/**
 * @brief 获取LVGL显示对象
 */
lv_disp_t* lvgl_driver_get_display(void) {
    return g_lvgl_driver.display.disp;
}

/**
 * @brief 获取LVGL输入设备
 */
lv_indev_t* lvgl_driver_get_indev(void) {
    return g_lvgl_driver.touch.indev;
}

/**
 * @brief 设置屏幕旋转
 */
void lvgl_driver_set_rotation(uint8_t rotation) {
    if (rotation > 3) rotation = 0;
    
    // 设置ST7789旋转
    st7789_set_rotation(rotation);
    
    // 更新显示分辨率
    if (rotation == 1 || rotation == 3) {
        g_lvgl_driver.display.disp_drv.hor_res = LVGL_DISP_VER_RES;
        g_lvgl_driver.display.disp_drv.ver_res = LVGL_DISP_HOR_RES;
    } else {
        g_lvgl_driver.display.disp_drv.hor_res = LVGL_DISP_HOR_RES;
        g_lvgl_driver.display.disp_drv.ver_res = LVGL_DISP_VER_RES;
    }
    
    // 通知LVGL分辨率改变
    lv_disp_drv_update(g_lvgl_driver.display.disp, &g_lvgl_driver.display.disp_drv);
    
    ESP_LOGI(TAG, "Screen rotation set to %d", rotation);
}

/**
 * @brief 设置背光亮度
 */
void lvgl_driver_set_backlight(uint8_t brightness) {
    st7789_set_brightness(brightness);
    ESP_LOGD(TAG, "Backlight set to %d", brightness);
}

/**
 * @brief 读取触摸点
 */
esp_err_t lvgl_driver_read_touch(int16_t* x, int16_t* y, bool* pressed) {
    if (!g_lvgl_driver.initialized || !g_lvgl_driver.touch.indev) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint8_t gesture = 0;
    return ft6336u_read_simple(x, y, &gesture, pressed);
}

/**
 * @brief 获取当前帧率
 */
uint32_t lvgl_driver_get_fps(void) {
    return g_perf_stats.fps;
}

/**
 * @brief 获取内存使用情况
 */
void lvgl_driver_get_mem_usage(uint32_t* total, uint32_t* used, uint32_t* max_used) {
    if (total) *total = g_perf_stats.mem_total;
    if (used) *used = g_perf_stats.mem_used;
    if (max_used) *max_used = g_perf_stats.mem_max_used;
}

/**
 * @brief 创建测试UI
 */
void lvgl_driver_create_test_ui(void) {
    if (!g_lvgl_driver.initialized) {
        ESP_LOGE(TAG, "LVGL not initialized");
        return;
    }
    
    ESP_LOGI(TAG, "Creating test UI...");
    
    // 获取当前屏幕
    lv_obj_t* scr = lv_scr_act();
    
    // 创建标题标签
    lv_obj_t* title_label = lv_label_create(scr);
    lv_label_set_text(title_label, "LVGL Test Interface");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_24, 0);
    lv_obj_align(title_label, LV_ALIGN_TOP_MID, 0, 20);
    
    // 创建按钮
    lv_obj_t* btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 120, 50);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, -50);
    
    lv_obj_t* btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Click Me!");
    lv_obj_center(btn_label);
    
    // 按钮点击事件
    lv_obj_add_event_cb(btn, [](lv_event_t* e) {
        static uint8_t cnt = 0;
        lv_obj_t* btn = lv_event_get_target(e);
        lv_obj_t* label = lv_obj_get_child(btn, 0);
        
        cnt++;
        lv_label_set_text_fmt(label, "Clicked: %d", cnt);
        
        ESP_LOGI(TAG, "Button clicked %d times", cnt);
    }, LV_EVENT_CLICKED, NULL);
    
    // 创建滑动条
    lv_obj_t* slider = lv_slider_create(scr);
    lv_slider_set_range(slider, 0, 100);
    lv_slider_set_value(slider, 50, LV_ANIM_OFF);
    lv_obj_set_width(slider, 200);
    lv_obj_align(slider, LV_ALIGN_CENTER, 0, 50);
    
    lv_obj_t* slider_label = lv_label_create(scr);
    lv_label_set_text_fmt(slider_label, "Value: %d", lv_slider_get_value(slider));
    lv_obj_align_to(slider_label, slider, LV_ALIGN_OUT_TOP_MID, 0, -10);
    
    // 滑动条值改变事件
    lv_obj_add_event_cb(slider, [](lv_event_t* e) {
        lv_obj_t* slider = lv_event_get_target(e);
        int32_t value = lv_slider_get_value(slider);
        
        lv_obj_t* label = lv_event_get_user_data(e);
        lv_label_set_text_fmt(label, "Value: %d", value);
        
        ESP_LOGD(TAG, "Slider value changed to %d", value);
    }, LV_EVENT_VALUE_CHANGED, slider_label);
    
    // 创建触摸测试区域
    lv_obj_t* touch_area = lv_obj_create(scr);
    lv_obj_set_size(touch_area, 100, 100);
    lv_obj_set_style_bg_color(touch_area, lv_palette_main(LV_PALETTE_BLUE), 0);
    lv_obj_set_style_radius(touch_area, 10, 0);
    lv_obj_align(touch_area, LV_ALIGN_BOTTOM_RIGHT, -20, -20);
    
    lv_obj_t* touch_label = lv_label_create(touch_area);
    lv_label_set_text(touch_label, "Touch");
    lv_obj_center(touch_label);
    
    // 触摸事件
    lv_obj_add_event_cb(touch_area, [](lv_event_t* e) {
        lv_obj_t* obj = lv_event_get_target(e);
        
        switch (lv_event_get_code(e)) {
            case LV_EVENT_PRESSED:
                lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_RED), 0);
                ESP_LOGI(TAG, "Touch area pressed");
                break;
                
            case LV_EVENT_RELEASED:
                lv_obj_set_style_bg_color(obj, lv_palette_main(LV_PALETTE_BLUE), 0);
                ESP_LOGI(TAG, "Touch area released");
                break;
                
            default:
                break;
        }
    }, LV_EVENT_ALL, NULL);
    
    // 创建FPS显示标签
    lv_obj_t* fps_label = lv_label_create(scr);
    lv_label_set_text(fps_label, "FPS: 0");
    lv_obj_align(fps_label, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_set_style_text_color(fps_label, lv_palette_main(LV_PALETTE_GREEN), 0);
    
    // 更新FPS的定时器
    lv_timer_create([](lv_timer_t* timer) {
        lv_obj_t* label = timer->user_data;
        uint32_t fps = lvgl_driver_get_fps();
        lv_label_set_text_fmt(label, "FPS: %d", fps);
    }, 1000, fps_label);
    
    ESP_LOGI(TAG, "Test UI created successfully");
}