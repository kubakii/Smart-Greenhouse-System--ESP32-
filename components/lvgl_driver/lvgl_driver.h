#ifndef __LVGL_DRIVER_H__
#define __LVGL_DRIVER_H__

#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include "toucholed/st7789.h"
#include "toucholed/ft6336u.h"

#ifdef __cplusplus
extern "C" {
#endif

// 屏幕配置
#define LVGL_DISP_HOR_RES       240
#define LVGL_DISP_VER_RES       320
#define LVGL_BUFFER_SIZE        40  // 缓冲区行数

// 颜色格式
#define LV_COLOR_DEPTH          16
#define LV_COLOR_16_SWAP        1   // 交换RGB565的字节顺序

// SPI和I2C配置
#define LVGL_SPI_HOST           SPI2_HOST
#define LVGL_SPI_FREQ           40000000  // 40MHz
#define LVGL_I2C_PORT           I2C_NUM_0
#define LVGL_I2C_FREQ           400000    // 400kHz

// 引脚定义（根据您的实际连接修改）
// ST7789引脚
#define PIN_LCD_MOSI            GPIO_NUM_21
#define PIN_LCD_CLK             GPIO_NUM_23
#define PIN_LCD_CS              GPIO_NUM_5
#define PIN_LCD_DC              GPIO_NUM_19
#define PIN_LCD_RST             GPIO_NUM_18
#define PIN_LCD_BL              GPIO_NUM_22

// FT6336U引脚
#define PIN_TOUCH_SDA           GPIO_NUM_27
#define PIN_TOUCH_SCL           GPIO_NUM_25
#define PIN_TOUCH_INT           GPIO_NUM_14

// 任务配置
#define LVGL_TASK_PRIORITY      20
#define LVGL_TASK_STACK_SIZE    8192
#define LVGL_TICK_HANDLER_DELAY 5  // ms

// 回调函数类型
typedef void (*lvgl_event_callback_t)(lv_event_t* e);
typedef void (*lvgl_user_data_callback_t)(void* user_data);

// LVGL驱动配置结构体
typedef struct {
    // 显示配置
    uint16_t width;
    uint16_t height;
    uint8_t rotation;
    bool use_double_buffer;
    bool use_full_refresh;
    
    // 触摸配置
    bool touch_enabled;
    uint16_t touch_threshold;
    
    // 回调函数
    lvgl_event_callback_t flush_callback;
    lvgl_user_data_callback_t task_callback;
    void* user_data;
} lvgl_driver_config_t;

// LVGL触摸数据结构
typedef struct {
    lv_indev_drv_t indev_drv;
    lv_indev_t* indev;
    ft6336u_cfg_t ft_config;
    bool is_pressed;
    int16_t last_x;
    int16_t last_y;
} lvgl_touch_driver_t;

// LVGL显示数据结构
typedef struct {
    lv_disp_drv_t disp_drv;
    lv_disp_t* disp;
    lv_disp_draw_buf_t draw_buf;
    st7789_cfg_t st_config;
    lv_color_t* buf1;
    lv_color_t* buf2;
    uint32_t buf_size;
} lvgl_display_driver_t;

// 全局驱动句柄
typedef struct {
    lvgl_display_driver_t display;
    lvgl_touch_driver_t touch;
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;
    bool initialized;
} lvgl_driver_handle_t;

/**
 * @brief 初始化LVGL驱动
 * @param config 配置参数（可为NULL使用默认配置）
 * @return esp_err_t 错误码
 */
esp_err_t lvgl_driver_init(const lvgl_driver_config_t* config);

/**
 * @brief 启动LVGL任务
 * @note 此任务会定期调用lv_timer_handler()
 */
void lvgl_driver_start_task(void);

/**
 * @brief 停止LVGL任务
 */
void lvgl_driver_stop_task(void);

/**
 * @brief 获取LVGL显示对象
 * @return lv_disp_t* 显示对象指针
 */
lv_disp_t* lvgl_driver_get_display(void);

/**
 * @brief 获取LVGL输入设备
 * @return lv_indev_t* 输入设备指针
 */
lv_indev_t* lvgl_driver_get_indev(void);

/**
 * @brief 设置屏幕旋转
 * @param rotation 旋转角度（0-3）
 */
void lvgl_driver_set_rotation(uint8_t rotation);

/**
 * @brief 设置背光亮度
 * @param brightness 亮度值（0-255）
 */
void lvgl_driver_set_backlight(uint8_t brightness);

/**
 * @brief 获取触摸点坐标
 * @param x X坐标输出
 * @param y Y坐标输出
 * @param pressed 按下状态输出
 * @return esp_err_t 错误码
 */
esp_err_t lvgl_driver_read_touch(int16_t* x, int16_t* y, bool* pressed);

/**
 * @brief 获取当前帧率
 * @return uint32_t 帧率（FPS）
 */
uint32_t lvgl_driver_get_fps(void);

/**
 * @brief 获取内存使用情况
 * @param total 总内存输出
 * @param used 已使用内存输出
 * @param max_used 最大使用内存输出
 */
void lvgl_driver_get_mem_usage(uint32_t* total, uint32_t* used, uint32_t* max_used);

/**
 * @brief 创建测试UI（用于验证驱动是否正常工作）
 */
void lvgl_driver_create_test_ui(void);

/**
 * @brief 默认配置
 */
extern const lvgl_driver_config_t lvgl_driver_default_config;

#ifdef __cplusplus
}
#endif

#endif // __LVGL_DRIVER_H__