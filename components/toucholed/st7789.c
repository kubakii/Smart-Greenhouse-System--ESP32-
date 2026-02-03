#include "st7789.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "ST7789";

// 全局变量
static spi_device_handle_t spi;
static st7789_cfg_t st_config;
static SemaphoreHandle_t spi_mutex = NULL;
static bool st_initialized = false;
static uint16_t screen_width = 240;
static uint16_t screen_height = 320;

// 发送命令
static void st7789_send_cmd(uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .user = (void*)0,  // DC=0表示命令
    };
    
    if (spi_mutex) xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    gpio_set_level(st_config.dc, 0);  // DC=0:命令
    ret = spi_device_polling_transmit(spi, &t);
    
    if (spi_mutex) xSemaphoreGive(spi_mutex);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send command 0x%02X failed: %s", cmd, esp_err_to_name(ret));
    }
}

// 发送数据
static void st7789_send_data(uint8_t* data, uint32_t len) {
    esp_err_t ret;
    
    if (len == 0) return;
    
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = data,
        .user = (void*)1,  // DC=1表示数据
    };
    
    if (spi_mutex) xSemaphoreTake(spi_mutex, portMAX_DELAY);
    
    gpio_set_level(st_config.dc, 1);  // DC=1:数据
    
    if (len > 4092) {
        // 大数据分块传输
        uint32_t offset = 0;
        while (offset < len) {
            uint32_t chunk_len = len - offset;
            if (chunk_len > 4092) chunk_len = 4092;
            
            t.length = chunk_len * 8;
            t.tx_buffer = data + offset;
            
            ret = spi_device_polling_transmit(spi, &t);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Send data chunk failed: %s", esp_err_to_name(ret));
                break;
            }
            offset += chunk_len;
        }
    } else {
        ret = spi_device_polling_transmit(spi, &t);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Send data failed: %s", esp_err_to_name(ret));
        }
    }
    
    if (spi_mutex) xSemaphoreGive(spi_mutex);
}

// 发送单字节数据
static void st7789_send_byte(uint8_t data) {
    st7789_send_data(&data, 1);
}

// 发送双字节数据
static void st7789_send_word(uint16_t data) {
    uint8_t buffer[2] = {data >> 8, data & 0xFF};
    st7789_send_data(buffer, 2);
}

// 硬件复位
static void st7789_hardware_reset(void) {
    if (st_config.rst >= 0) {
        gpio_set_level(st_config.rst, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(st_config.rst, 1);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else {
        // 软件复位
        st7789_send_cmd(ST7789_SWRESET);
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

// 初始化序列
static void st7789_init_sequence(void) {
    ESP_LOGI(TAG, "Starting ST7789 initialization sequence");
    
    // 软件复位
    st7789_send_cmd(ST7789_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));
    
    // 退出睡眠模式
    st7789_send_cmd(ST7789_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(255));
    
    // 设置颜色模式
    st7789_send_cmd(ST7789_COLMOD);
    st7789_send_byte(st_config.color_mode ? ST7789_COLOR_MODE_16BIT : ST7789_COLOR_MODE_65K);
    
    // 设置内存数据访问控制
    st7789_set_rotation(st_config.rotation);
    
    // 关闭反色
    st7789_send_cmd(ST7789_INVOFF);
    
    // 设置显示区域
    uint16_t x_offset = 0;
    uint16_t y_offset = 0;
    
    // 根据方向调整偏移
    if (st_config.rotation == 1 || st_config.rotation == 3) {
        x_offset = (320 - 240) / 2;
        y_offset = (240 - 320) / 2;
    }
    
    // 设置列地址
    st7789_send_cmd(ST7789_CASET);
    st7789_send_word(x_offset);
    st7789_send_word(x_offset + screen_width - 1);
    
    // 设置行地址
    st7789_send_cmd(ST7789_RASET);
    st7789_send_word(y_offset);
    st7789_send_word(y_offset + screen_height - 1);
    
    // 开启正常显示模式
    st7789_send_cmd(ST7789_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 开启显示
    st7789_send_cmd(ST7789_DISPON);
    vTaskDelay(pdMS_TO_TICKS(10));
    
    // 开启背光
    if (st_config.bl >= 0) {
        st7789_set_backlight(true);
    }
    
    ESP_LOGI(TAG, "ST7789 initialization completed");
}

esp_err_t st7789_init(st7789_cfg_t* cfg) {
    esp_err_t ret;
    
    if (st_initialized) {
        ESP_LOGW(TAG, "ST7789 already initialized");
        return ESP_OK;
    }
    
    if (!cfg) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 保存配置
    st_config = *cfg;
    screen_width = cfg->width;
    screen_height = cfg->height;
    
    // 创建互斥锁
    spi_mutex = xSemaphoreCreateMutex();
    if (!spi_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 配置GPIO
    gpio_config_t io_conf = {0};
    
    // 配置DC引脚
    if (cfg->dc >= 0) {
        io_conf.pin_bit_mask = (1ULL << cfg->dc);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(cfg->dc, 0);
    }
    
    // 配置RST引脚
    if (cfg->rst >= 0) {
        io_conf.pin_bit_mask = (1ULL << cfg->rst);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(cfg->rst, 1);
    }
    
    // 配置背光引脚
    if (cfg->bl >= 0) {
        io_conf.pin_bit_mask = (1ULL << cfg->bl);
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&io_conf);
        gpio_set_level(cfg->bl, 0);  // 初始关闭背光
    }
    
    // 配置SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = cfg->mosi,
        .miso_io_num = -1,  // 不使用MISO
        .sclk_io_num = cfg->clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092,
    };
    
    ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置SPI设备
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->spi_freq ? cfg->spi_freq : 40000000,  // 默认40MHz
        .mode = 0,  // SPI模式0
        .spics_io_num = cfg->cs,
        .queue_size = 7,
        .flags = SPI_DEVICE_NO_DUMMY,
        .pre_cb = NULL,
    };
    
    ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 硬件复位
    st7789_hardware_reset();
    
    // 发送初始化序列
    st7789_init_sequence();
    
    st_initialized = true;
    ESP_LOGI(TAG, "ST7789 initialized successfully");
    
    return ESP_OK;
}

void st7789_flush(int x1, int x2, int y1, int y2, void* data) {
    if (!st_initialized || !data) return;
    
    // 限制坐标范围
    if (x1 < 0) x1 = 0;
    if (y1 < 0) y1 = 0;
    if (x2 >= screen_width) x2 = screen_width - 1;
    if (y2 >= screen_height) y2 = screen_height - 1;
    
    if (x1 > x2 || y1 > y2) return;
    
    // 设置显示窗口
    st7789_set_window(x1, x2, y1, y2);
    
    // 发送数据
    uint32_t data_len = (x2 - x1 + 1) * (y2 - y1 + 1) * 2;  // 每个像素2字节
    st7789_send_cmd(ST7789_RAMWR);
    st7789_send_data((uint8_t*)data, data_len);
    
    // 调用回调函数（如果设置）
    if (st_config.done_cb) {
        st_config.done_cb(st_config.cb_param);
    }
}

void st7789_set_backlight(bool enable) {
    if (st_config.bl >= 0) {
        gpio_set_level(st_config.bl, enable ? 1 : 0);
    }
}

void st7789_set_rotation(uint8_t rotation) {
    if (rotation > 3) rotation = 0;
    st_config.rotation = rotation;
    
    uint8_t madctl = 0;
    
    switch (rotation) {
        case 0:  // 0度
            madctl = ST7789_MADCTL_MX | ST7789_MADCTL_MY;
            screen_width = st_config.width;
            screen_height = st_config.height;
            break;
        case 1:  // 90度
            madctl = ST7789_MADCTL_MV | ST7789_MADCTL_MY;
            screen_width = st_config.height;
            screen_height = st_config.width;
            break;
        case 2:  // 180度
            madctl = 0;
            screen_width = st_config.width;
            screen_height = st_config.height;
            break;
        case 3:  // 270度
            madctl = ST7789_MADCTL_MV | ST7789_MADCTL_MX;
            screen_width = st_config.height;
            screen_height = st_config.width;
            break;
    }
    
    // 设置BGR顺序
    if (st_config.bgr_order) {
        madctl |= ST7789_MADCTL_BGR;
    }
    
    st7789_send_cmd(ST7789_MADCTL);
    st7789_send_byte(madctl);
}

void st7789_set_window(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2) {
    // 设置列地址
    st7789_send_cmd(ST7789_CASET);
    st7789_send_word(x1);
    st7789_send_word(x2);
    
    // 设置行地址
    st7789_send_cmd(ST7789_RASET);
    st7789_send_word(y1);
    st7789_send_word(y2);
}

void st7789_fill_rect(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2, st7789_color_t color) {
    if (!st_initialized) return;
    
    // 限制坐标范围
    if (x1 >= screen_width || y1 >= screen_height) return;
    if (x2 >= screen_width) x2 = screen_width - 1;
    if (y2 >= screen_height) y2 = screen_height - 1;
    
    uint32_t pixel_count = (x2 - x1 + 1) * (y2 - y1 + 1);
    
    // 设置显示窗口
    st7789_set_window(x1, x2, y1, y2);
    
    // 发送填充命令
    st7789_send_cmd(ST7789_RAMWR);
    
    // 发送填充数据
    uint8_t color_high = color >> 8;
    uint8_t color_low = color & 0xFF;
    
    // 分块发送，避免大内存分配
    const uint32_t CHUNK_SIZE = 512;  // 一次发送512像素
    uint8_t chunk[CHUNK_SIZE * 2];
    
    // 准备填充数据
    for (uint32_t i = 0; i < CHUNK_SIZE * 2; i += 2) {
        chunk[i] = color_high;
        chunk[i + 1] = color_low;
    }
    
    uint32_t remaining = pixel_count;
    while (remaining > 0) {
        uint32_t send_count = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        st7789_send_data(chunk, send_count * 2);
        remaining -= send_count;
    }
}

void st7789_draw_pixel(uint16_t x, uint16_t y, st7789_color_t color) {
    if (x >= screen_width || y >= screen_height) return;
    
    st7789_set_window(x, x, y, y);
    st7789_send_cmd(ST7789_RAMWR);
    st7789_send_word(color);
}

void st7789_draw_hline(uint16_t x, uint16_t y, uint16_t w, st7789_color_t color) {
    if (y >= screen_height) return;
    if (x + w > screen_width) w = screen_width - x;
    
    st7789_fill_rect(x, x + w - 1, y, y, color);
}

void st7789_draw_vline(uint16_t x, uint16_t y, uint16_t h, st7789_color_t color) {
    if (x >= screen_width) return;
    if (y + h > screen_height) h = screen_height - y;
    
    st7789_fill_rect(x, x, y, y + h - 1, color);
}

void st7789_draw_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, st7789_color_t color) {
    if (x >= screen_width || y >= screen_height) return;
    
    // 限制尺寸
    if (x + w > screen_width) w = screen_width - x;
    if (y + h > screen_height) h = screen_height - y;
    
    // 绘制四条边
    st7789_draw_hline(x, y, w, color);               // 上边
    st7789_draw_hline(x, y + h - 1, w, color);       // 下边
    st7789_draw_vline(x, y + 1, h - 2, color);       // 左边
    st7789_draw_vline(x + w - 1, y + 1, h - 2, color); // 右边
}

void st7789_fill_rect_simple(uint16_t x, uint16_t y, uint16_t w, uint16_t h, st7789_color_t color) {
    if (x >= screen_width || y >= screen_height) return;
    
    // 限制尺寸
    if (x + w > screen_width) w = screen_width - x;
    if (y + h > screen_height) h = screen_height - y;
    
    st7789_fill_rect(x, x + w - 1, y, y + h - 1, color);
}

void st7789_draw_bitmap(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* bitmap) {
    if (!bitmap) return;
    
    // 限制坐标范围
    if (x >= screen_width || y >= screen_height) return;
    
    uint16_t draw_w = w;
    uint16_t draw_h = h;
    
    if (x + draw_w > screen_width) draw_w = screen_width - x;
    if (y + draw_h > screen_height) draw_h = screen_height - y;
    
    // 设置显示窗口
    st7789_set_window(x, x + draw_w - 1, y, y + draw_h - 1);
    
    // 发送数据
    st7789_send_cmd(ST7789_RAMWR);
    st7789_send_data((uint8_t*)bitmap, draw_w * draw_h * 2);
}

void st7789_set_invert(bool invert) {
    st7789_send_cmd(invert ? ST7789_INVON : ST7789_INVOFF);
}

void st7789_set_sleep(bool sleep) {
    st7789_send_cmd(sleep ? ST7789_SLPIN : ST7789_SLPOUT);
    if (!sleep) {
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

void st7789_set_display_on(bool on) {
    st7789_send_cmd(on ? ST7789_DISPON : ST7789_DISPOFF);
}

void st7789_set_brightness(uint8_t brightness) {
    // 注意：ST7789的亮度控制需要通过PWM控制背光引脚
    if (st_config.bl >= 0) {
        // 这里使用简单的GPIO控制，实际可以使用PWM
        gpio_set_level(st_config.bl, brightness > 0 ? 1 : 0);
    }
}

uint16_t st7789_get_width(void) {
    return screen_width;
}

uint16_t st7789_get_height(void) {
    return screen_height;
}