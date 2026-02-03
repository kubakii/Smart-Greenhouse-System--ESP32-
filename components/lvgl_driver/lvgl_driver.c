#include "lvgl_driver.h"
#include "uart.h"

static const char *TAG = "LVGL_DRIVER";
static lv_disp_t *disp = NULL;
static lv_indev_t *indev = NULL;
static QueueHandle_t ui_command_queue = NULL;

// UI组件
static lv_obj_t *temp_label;
static lv_obj_t *hum_label;
static lv_obj_t *light_label;
static lv_obj_t *soil_label;
static lv_obj_t *motor_slider;
static lv_obj_t *led_switch;
static lv_obj_t *fan_switch;
static lv_obj_t *pump_button;

// 刷新显示回调
static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

// 触摸读取回调
static void lvgl_touch_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    esp_lcd_touch_handle_t touch_handle = (esp_lcd_touch_handle_t)drv->user_data;
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;
    
    esp_lcd_touch_read_data(touch_handle);
    bool pressed = esp_lcd_touch_get_coordinates(touch_handle, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);
    
    if (pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PRESSED;
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// 电机速度滑动回调
static void motor_slider_event_cb(lv_event_t *e) {
    lv_obj_t *slider = lv_event_get_target(e);
    int32_t value = lv_slider_get_value(slider);
    
    // 发送控制命令
    uart_command_t cmd = {
        .type = CMD_MOTOR_SPEED,
        .value = value
    };
    uart_send_command(&cmd);
    
    ESP_LOGI(TAG, "Motor speed set to: %d", value);
}

// LED开关回调
static void led_switch_event_cb(lv_event_t *e) {
    lv_obj_t *sw = lv_event_get_target(e);
    bool state = lv_obj_has_state(sw, LV_STATE_CHECKED);
    
    uart_command_t cmd = {
        .type = CMD_LED_CONTROL,
        .value = state ? 1 : 0
    };
    uart_send_command(&cmd);
    
    ESP_LOGI(TAG, "LED %s", state ? "ON" : "OFF");
}

// 创建UI界面
static void create_ui(void) {
    // 创建主容器
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(cont, 10, 0);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    // 标题
    lv_obj_t *title = lv_label_create(cont);
    lv_label_set_text(title, "智能大棚控制系统");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    
    // 传感器数据显示区域
    lv_obj_t *sensor_cont = lv_obj_create(cont);
    lv_obj_set_size(sensor_cont, LV_PCT(90), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_all(sensor_cont, 10, 0);
    lv_obj_set_flex_flow(sensor_cont, LV_FLEX_FLOW_ROW_WRAP);
    
    // 温度显示
    lv_obj_t *temp_cont = lv_obj_create(sensor_cont);
    lv_obj_set_size(temp_cont, LV_PCT(45), 80);
    lv_obj_t *temp_title = lv_label_create(temp_cont);
    lv_label_set_text(temp_title, "温度");
    lv_obj_align(temp_title, LV_ALIGN_TOP_MID, 0, 5);
    
    temp_label = lv_label_create(temp_cont);
    lv_label_set_text(temp_label, "25.6°C");
    lv_obj_set_style_text_font(temp_label, &lv_font_montserrat_20, 0);
    lv_obj_align(temp_label, LV_ALIGN_CENTER, 0, 0);
    
    // 其他传感器显示类似...
    
    // 控制区域
    lv_obj_t *control_cont = lv_obj_create(cont);
    lv_obj_set_size(control_cont, LV_PCT(90), LV_SIZE_CONTENT);
    
    // 电机速度控制
    lv_obj_t *motor_label = lv_label_create(control_cont);
    lv_label_set_text(motor_label, "电机速度:");
    lv_obj_align(motor_label, LV_ALIGN_TOP_LEFT, 10, 10);
    
    motor_slider = lv_slider_create(control_cont);
    lv_slider_set_range(motor_slider, 0, 1000);
    lv_slider_set_value(motor_slider, 500, LV_ANIM_OFF);
    lv_obj_set_width(motor_slider, 200);
    lv_obj_align_to(motor_slider, motor_label, LV_ALIGN_OUT_RIGHT_MID, 20, 0);
    lv_obj_add_event_cb(motor_slider, motor_slider_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // LED开关
    led_switch = lv_switch_create(control_cont);
    lv_obj_align(led_switch, LV_ALIGN_TOP_LEFT, 10, 50);
    lv_obj_t *led_label = lv_label_create(control_cont);
    lv_label_set_text(led_label, "补光灯:");
    lv_obj_align_to(led_label, led_switch, LV_ALIGN_OUT_LEFT_MID, -10, 0);
    lv_obj_add_event_cb(led_switch, led_switch_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    
    // 其他控制类似...
}

// 初始化LVGL驱动
esp_err_t lvgl_driver_init(void) {
    esp_err_t ret = ESP_OK;
    
    // 初始化LVGL
    lv_init();
    
    // 初始化SPI总线
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCLK,
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t)
    };
    
    ret = spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed");
        return ret;
    }
    
    // 配置LCD
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "New panel IO failed");
        return ret;
    }
    
    // 创建LCD面板
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_RGB,
        .bits_per_pixel = 16,
    };
    
    ret = esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "New panel failed");
        return ret;
    }
    
    // 初始化面板
    esp_lcd_panel_reset(panel_handle);
    esp_lcd_panel_init(panel_handle);
    esp_lcd_panel_invert_color(panel_handle, true);
    esp_lcd_panel_mirror(panel_handle, false, true);
    esp_lcd_panel_set_gap(panel_handle, 0, 0);
    
    // 背光控制
    gpio_set_direction(PIN_NUM_BK_LIGHT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_NUM_BK_LIGHT, LCD_BK_LIGHT_ON_LEVEL);
    
    // 注册LVGL显示驱动
    lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf1[LCD_H_RES * 40];
    static lv_color_t buf2[LCD_H_RES * 40];
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_H_RES * 40);
    
    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = panel_handle;
    disp = lv_disp_drv_register(&disp_drv);
    
    // 初始化触摸屏
    esp_lcd_touch_handle_t touch_handle = NULL;
    esp_lcd_touch_config_t touch_config = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = PIN_NUM_TOUCH_IRQ,
    };
    
    esp_lcd_touch_new_spi_i2c_bus(LCD_HOST, &touch_config, &touch_handle);
    
    // 注册LVGL输入设备
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = lvgl_touch_cb;
    indev_drv.user_data = touch_handle;
    indev = lv_indev_drv_register(&indev_drv);
    
    // 创建UI
    create_ui();
    
    // 创建UI命令队列
    ui_command_queue = xQueueCreate(10, sizeof(uart_command_t));
    
    ESP_LOGI(TAG, "LVGL driver initialized");
    return ESP_OK;
}

// LVGL任务
void lvgl_driver_task(void *pvParameter) {
    ESP_LOGI(TAG, "LVGL Task started");
    
    while (1) {
        // LVGL定时处理
        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 更新显示
void update_temperature_display(float temp) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%.1f°C", temp);
    lv_label_set_text(temp_label, buffer);
}

