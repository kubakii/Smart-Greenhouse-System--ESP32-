#ifndef _ST7789_DRIVER_H_
#define _ST7789_DRIVER_H_

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/semphr.h"

// ST7789 命令定义         (SAVE)
#define ST7789_NOP        0x00     // 空操作
#define ST7789_SWRESET    0x01     // 软件复位
#define ST7789_RDDID      0x04     // 读显示ID
#define ST7789_RDDST      0x09     // 读显示状态
#define ST7789_SLPIN      0x10     // 进入睡眠模式
#define ST7789_SLPOUT     0x11     // 退出睡眠模式
#define ST7789_PTLON      0x12     // 部分显示模式开
#define ST7789_NORON      0x13     // 正常显示模式开
#define ST7789_INVOFF     0x20     // 关闭反色
#define ST7789_INVON      0x21     // 开启反色
#define ST7789_DISPOFF    0x28     // 关闭显示
#define ST7789_DISPON     0x29     // 开启显示
#define ST7789_CASET      0x2A     // 设置列地址
#define ST7789_RASET      0x2B     // 设置行地址
#define ST7789_RAMWR      0x2C     // 写显存
#define ST7789_RAMRD      0x2E     // 读显存
#define ST7789_PTLAR      0x30     // 设置部分区域
#define ST7789_VSCRDEF    0x33     // 设置垂直滚动定义
#define ST7789_TEOFF      0x34     // Tearing效果关闭
#define ST7789_TEON       0x35     // Tearing效果开启
#define ST7789_MADCTL     0x36     // 内存数据访问控制
#define ST7789_VSCSAD     0x37     // 设置垂直滚动起始地址
#define ST7789_COLMOD     0x3A     // 设置颜色模式
#define ST7789_WRMEMC     0x3C     // 写内存继续
#define ST7789_RDMEMC     0x3E     // 读内存继续
#define ST7789_STE        0x44     // 设置Tearing扫描线
#define ST7789_GSCAN      0x45     // 获取扫描线
#define ST7789_WRDISBV    0x51     // 写显示亮度
#define ST7789_RDDISBV    0x52     // 读显示亮度
#define ST7789_WRCTRLD    0x53     // 写CTRL显示
#define ST7789_RDCTRLD    0x54     // 读CTRL显示
#define ST7789_WRCACE     0x55     // 写内容自适应对比增强
#define ST7789_RDCABC     0x56     // 读内容自适应对比增强
#define ST7789_WRCABCMB   0x5E     // 写CABC最小亮度
#define ST7789_RDCABCMB   0x5F     // 读CABC最小亮度
#define ST7789_RDID1      0xDA     // 读ID1
#define ST7789_RDID2      0xDB     // 读ID2
#define ST7789_RDID3      0xDC     // 读ID3

// 屏幕方向定义
#define ST7789_MADCTL_MY  0x80     // 行地址顺序
#define ST7789_MADCTL_MX  0x40     // 列地址顺序
#define ST7789_MADCTL_MV  0x20     // 行列交换
#define ST7789_MADCTL_ML  0x10     // 垂直刷新顺序
#define ST7789_MADCTL_RGB 0x00     // RGB顺序
#define ST7789_MADCTL_BGR 0x08     // BGR顺序
#define ST7789_MADCTL_MH  0x04     // 水平刷新顺序

// 颜色模式
#define ST7789_COLOR_MODE_65K  0x50  // 65K色模式
#define ST7789_COLOR_MODE_262K 0x60  // 262K色模式
#define ST7789_COLOR_MODE_12BIT 0x03 // 12位/像素
#define ST7789_COLOR_MODE_16BIT 0x05 // 16位/像素
#define ST7789_COLOR_MODE_18BIT 0x06 // 18位/像素
#define ST7789_COLOR_MODE_16M   0x07 // 24位/像素

// 回调函数类型定义
typedef void (*st7789_flush_done_cb_t)(void* param);

// ST7789 配置结构体
typedef struct {
    gpio_num_t  mosi;           // MOSI管脚
    gpio_num_t  clk;            // SCLK管脚
    gpio_num_t  cs;             // CS管脚
    gpio_num_t  dc;             // DC管脚（命令/数据选择）
    gpio_num_t  rst;            // 复位管脚
    gpio_num_t  bl;             // 背光管脚
    uint32_t    spi_freq;       // SPI频率（默认40MHz）
    uint16_t    width;          // 屏幕宽度（240）
    uint16_t    height;         // 屏幕高度（320）
    uint8_t     rotation;       // 旋转方向（0-3）
    uint8_t     color_mode;     // 颜色模式
    bool        bgr_order;      // BGR顺序（true:BGR, false:RGB）
    st7789_flush_done_cb_t done_cb;  // 刷新完成回调
    void*       cb_param;       // 回调参数
} st7789_cfg_t;

// 颜色定义（16位RGB565）
typedef uint16_t st7789_color_t;
#define ST7789_BLACK       0x0000
#define ST7789_BLUE        0x001F
#define ST7789_RED         0xF800
#define ST7789_GREEN       0x07E0
#define ST7789_CYAN        0x07FF
#define ST7789_MAGENTA     0xF81F
#define ST7789_YELLOW      0xFFE0
#define ST7789_WHITE       0xFFFF
#define ST7789_GRAY        0x8410
#define ST7789_ORANGE      0xFD20

// 从RGB888转换为RGB565
#define RGB888_TO_RGB565(r, g, b) ((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

/** 
 * ST7789初始化
 * @param cfg 配置参数
 * @return esp_err_t 错误码
 */
esp_err_t st7789_init(st7789_cfg_t* cfg);

/** 
 * 刷新显示区域
 * @param x1 起始X坐标
 * @param x2 结束X坐标
 * @param y1 起始Y坐标
 * @param y2 结束Y坐标
 * @param data 显示数据（RGB565格式）
 */
void st7789_flush(int x1, int x2, int y1, int y2, void* data);

/** 
 * 控制背光
 * @param enable 是否使能背光
 */
void st7789_set_backlight(bool enable);

/** 
 * 设置屏幕方向
 * @param rotation 旋转方向（0-3）
 */
void st7789_set_rotation(uint8_t rotation);

/** 
 * 设置显示窗口
 * @param x1 起始X坐标
 * @param x2 结束X坐标
 * @param y1 起始Y坐标
 * @param y2 结束Y坐标
 */
void st7789_set_window(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2);

/** 
 * 填充颜色
 * @param x1 起始X坐标
 * @param x2 结束X坐标
 * @param y1 起始Y坐标
 * @param y2 结束Y坐标
 * @param color 填充颜色
 */
void st7789_fill_rect(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2, st7789_color_t color);

/** 
 * 绘制像素点
 * @param x X坐标
 * @param y Y坐标
 * @param color 颜色
 */
void st7789_draw_pixel(uint16_t x, uint16_t y, st7789_color_t color);

/** 
 * 绘制水平线
 * @param x 起始X坐标
 * @param y Y坐标
 * @param w 宽度
 * @param color 颜色
 */
void st7789_draw_hline(uint16_t x, uint16_t y, uint16_t w, st7789_color_t color);

/** 
 * 绘制垂直线
 * @param x X坐标
 * @param y 起始Y坐标
 * @param h 高度
 * @param color 颜色
 */
void st7789_draw_vline(uint16_t x, uint16_t y, uint16_t h, st7789_color_t color);

/** 
 * 绘制矩形
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param w 宽度
 * @param h 高度
 * @param color 颜色
 */
void st7789_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, st7789_color_t color);

/** 
 * 填充矩形
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param w 宽度
 * @param h 高度
 * @param color 颜色
 */
void st7789_fill_rect_simple(uint16_t x, uint16_t y, uint16_t w, uint16_t h, st7789_color_t color);

/** 
 * 绘制位图
 * @param x 起始X坐标
 * @param y 起始Y坐标
 * @param w 宽度
 * @param h 高度
 * @param bitmap 位图数据
 */
void st7789_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* bitmap);

/** 
 * 设置反色
 * @param invert 是否反色
 */
void st7789_set_invert(bool invert);

/** 
 * 设置睡眠模式
 * @param sleep 是否进入睡眠
 */
void st7789_set_sleep(bool sleep);

/** 
 * 设置显示开关
 * @param on 是否开启显示
 */
void st7789_set_display_on(bool on);

/** 
 * 设置亮度
 * @param brightness 亮度（0-255）
 */
void st7789_set_brightness(uint8_t brightness);

/** 
 * 获取屏幕宽度
 * @return uint16_t 宽度
 */
uint16_t st7789_get_width(void);

/** 
 * 获取屏幕高度
 * @return uint16_t 高度
 */
uint16_t st7789_get_height(void);

#endif // _ST7789_DRIVER_H_