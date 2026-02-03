#ifndef _FT6336U_DRIVER_H_
#define _FT6336U_DRIVER_H_

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"

// FT6336U I2C地址
#define FT6336U_I2C_ADDR         0x38

// FT6336U 寄存器地址定义 (根据PDF第9页寄存器映射)
#define FT6336U_REG_DEVICE_MODE  0x00  // 设备模式寄存器
#define FT6336U_REG_GEST_ID      0x01  // 手势ID寄存器
#define FT6336U_REG_TD_STATUS    0x02  // 触摸状态寄存器
#define FT6336U_REG_TOUCH1_XH    0x03  // 第一个触摸点X高字节
#define FT6336U_REG_TOUCH1_XL    0x04  // 第一个触摸点X低字节
#define FT6336U_REG_TOUCH1_YH    0x05  // 第一个触摸点Y高字节
#define FT6336U_REG_TOUCH1_YL    0x06  // 第一个触摸点Y低字节
#define FT6336U_REG_TOUCH1_WEIGHT 0x07 // 第一个触摸点权重
#define FT6336U_REG_TOUCH1_MISC  0x08  // 第一个触摸点其他信息

// 手势ID定义 (根据PDF第14页)
#define FT6336U_GESTURE_NONE            0x00  // 无手势
#define FT6336U_GESTURE_MOVE_UP         0x10  // 向上移动
#define FT6336U_GESTURE_MOVE_RIGHT      0x14  // 向右移动
#define FT6336U_GESTURE_MOVE_DOWN       0x18  // 向下移动
#define FT6336U_GESTURE_MOVE_LEFT       0x1C  // 向左移动
#define FT6336U_GESTURE_SINGLE_CLICK    0x20  // 单次点击
#define FT6336U_GESTURE_DOUBLE_CLICK    0x22  // 双击
#define FT6336U_GESTURE_ZOOM_IN         0x48  // 放大
#define FT6336U_GESTURE_ZOOM_OUT        0x49  // 缩小

// 触摸事件标志 (根据PDF第15页)
#define FT6336U_EVENT_PUT_DOWN          0x00  // 按下
#define FT6336U_EVENT_PUT_UP            0x01  // 抬起
#define FT6336U_EVENT_CONTACT           0x02  // 接触中

// FT6336U 配置结构体
typedef struct 
{
    gpio_num_t  scl;         // SCL管脚
    gpio_num_t  sda;         // SDA管脚
    gpio_num_t  int_pin;     // 中断管脚 (根据PDF第5页接口定义，脚位14: T_IRQ/INT)
    uint32_t    i2c_freq;    // I2C频率 (默认400kHz)
    uint16_t    x_limit;     // X方向触摸边界 (根据PDF，分辨率可配置)
    uint16_t    y_limit;     // Y方向触摸边界
    bool        use_int;     // 是否使用中断模式
} ft6336u_cfg_t;

// 触摸点数据结构体
typedef struct {
    uint16_t x;              // X坐标
    uint16_t y;              // Y坐标
    uint8_t  touch_id;       // 触摸点ID (0-9)
    uint8_t  event;          // 触摸事件
    uint8_t  weight;         // 触摸权重
    uint8_t  area;           // 触摸区域
} ft6336u_touch_point_t;

// 触摸数据完整结构体
typedef struct {
    uint8_t gesture_id;                  // 手势ID
    uint8_t touch_points;                // 触摸点数量
    ft6336u_touch_point_t points[10];    // 最多支持10点触摸
} ft6336u_touch_data_t;

/** 
 * FT6336U初始化
 * @param cfg 配置参数
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_init(ft6336u_cfg_t* cfg);

/** 
 * 读取触摸数据
 * @param touch_data 触摸数据输出
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_read_touch_data(ft6336u_touch_data_t* touch_data);

/** 
 * 读取单点触摸坐标（简化接口）
 * @param x X坐标输出
 * @param y Y坐标输出
 * @param gesture 手势ID输出
 * @param pressed 按下状态输出
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_read_simple(int16_t* x, int16_t* y, uint8_t* gesture, bool* pressed);

/** 
 * 设置工作模式
 * @param mode 0:正常模式, 1:测试模式, 2:系统信息模式
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_set_mode(uint8_t mode);

/** 
 * 设置触摸阈值
 * @param threshold 阈值值 (默认180/4=45)
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_set_touch_threshold(uint8_t threshold);

/** 
 * 设置屏幕分辨率
 * @param width 宽度
 * @param height 高度
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_set_resolution(uint16_t width, uint16_t height);

/** 
 * 使能/禁用中断
 * @param enable true:使能, false:禁用
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_enable_interrupt(bool enable);

/** 
 * 读取芯片ID
 * @param chip_id 芯片ID输出
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_read_chip_id(uint8_t* chip_id);

/** 
 * 进入睡眠模式
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_enter_sleep(void);

/** 
 * 退出睡眠模式
 * @return esp_err_t 错误码
 */
esp_err_t ft6336u_exit_sleep(void);

#endif // _FT6336U_DRIVER_H_