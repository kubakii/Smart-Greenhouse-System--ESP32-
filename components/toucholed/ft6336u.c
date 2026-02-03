#include "ft6336u.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "FT6336U";
static i2c_port_t i2c_port = I2C_NUM_0;
static ft6336u_cfg_t ft_config;
static bool ft_initialized = false;

// I2C写寄存器
static esp_err_t ft6336u_write_register(uint8_t reg, uint8_t value) {
    uint8_t write_buf[2] = {reg, value};
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, sizeof(write_buf), true);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write register 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// I2C读寄存器
static esp_err_t ft6336u_read_register(uint8_t reg, uint8_t* value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, value, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read register 0x%02X failed: %s", reg, esp_err_to_name(ret));
    }
    return ret;
}

// I2C连续读寄存器
static esp_err_t ft6336u_read_registers(uint8_t start_reg, uint8_t* buffer, uint8_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, start_reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (FT6336U_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    
    if (length > 1) {
        i2c_master_read(cmd, buffer, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buffer + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read registers from 0x%02X failed: %s", start_reg, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t ft6336u_init(ft6336u_cfg_t* cfg) {
    esp_err_t ret = ESP_OK;
    
    if (ft_initialized) {
        ESP_LOGW(TAG, "FT6336U already initialized");
        return ESP_OK;
    }
    
    if (!cfg) {
        ESP_LOGE(TAG, "Config is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    // 保存配置
    ft_config = *cfg;
    
    // 配置I2C
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = cfg->sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = cfg->scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq ? cfg->i2c_freq : 400000,
        .clk_flags = 0,
    };
    
    ret = i2c_param_config(i2c_port, &i2c_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置中断引脚（如果使用）
    if (cfg->use_int && cfg->int_pin >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << cfg->int_pin),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE,
        };
        gpio_config(&io_conf);
    }
    
    // 读取设备ID验证连接
    uint8_t device_id;
    ret = ft6336u_read_register(FT6336U_REG_DEVICE_MODE, &device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read device ID");
        return ret;
    }
    
    // 设置设备为正常操作模式
    device_id &= ~0x07;  // 清除模式位
    device_id |= 0x00;   // 设置为正常操作模式
    ret = ft6336u_write_register(FT6336U_REG_DEVICE_MODE, device_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set device mode");
        return ret;
    }
    
    // 设置触摸阈值（默认值）
    ret = ft6336u_set_touch_threshold(45);  // 默认180/4=45
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set touch threshold");
        return ret;
    }
    
    // 设置屏幕分辨率（根据显示模块）
    if (cfg->x_limit > 0 && cfg->y_limit > 0) {
        ret = ft6336u_set_resolution(cfg->x_limit, cfg->y_limit);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to set resolution, using default");
        }
    }
    
    // 使能中断
    if (cfg->use_int) {
        ret = ft6336u_enable_interrupt(true);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to enable interrupt");
            return ret;
        }
    }
    
    ft_initialized = true;
    ESP_LOGI(TAG, "FT6336U initialized successfully");
    
    return ESP_OK;
}

esp_err_t ft6336u_read_touch_data(ft6336u_touch_data_t* touch_data) {
    if (!touch_data || !ft_initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    uint8_t data_buffer[0x40] = {0};  // 读取足够的数据
    
    // 从0x00开始读取数据
    ret = ft6336u_read_registers(0x00, data_buffer, sizeof(data_buffer));
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 解析手势ID
    touch_data->gesture_id = data_buffer[FT6336U_REG_GEST_ID];
    
    // 解析触摸点数量
    uint8_t td_status = data_buffer[FT6336U_REG_TD_STATUS];
    touch_data->touch_points = td_status & 0x0F;  // 低4位为触摸点数量
    
    // 最大支持10个触摸点
    if (touch_data->touch_points > 10) {
        touch_data->touch_points = 10;
    }
    
    // 解析每个触摸点数据
    for (int i = 0; i < touch_data->touch_points; i++) {
        uint8_t base_reg = FT6336U_REG_TOUCH1_XH + i * 6;  // 每个触摸点占用6个寄存器
        
        // 读取触摸点数据
        uint8_t touch_xh = data_buffer[base_reg];
        uint8_t touch_xl = data_buffer[base_reg + 1];
        uint8_t touch_yh = data_buffer[base_reg + 2];
        uint8_t touch_yl = data_buffer[base_reg + 3];
        uint8_t touch_weight = data_buffer[base_reg + 4];
        uint8_t touch_misc = data_buffer[base_reg + 5];
        
        // 解析X坐标（12位）
        touch_data->points[i].x = ((touch_xh & 0x0F) << 8) | touch_xl;
        
        // 解析Y坐标（12位）
        touch_data->points[i].y = ((touch_yh & 0x0F) << 8) | touch_yl;
        
        // 解析触摸点ID（4位）
        touch_data->points[i].touch_id = (touch_yh >> 4) & 0x0F;
        
        // 解析事件（2位）
        touch_data->points[i].event = (touch_xh >> 6) & 0x03;
        
        // 解析权重
        touch_data->points[i].weight = touch_weight;
        
        // 解析区域（4位）
        touch_data->points[i].area = (touch_misc >> 4) & 0x0F;
    }
    
    return ESP_OK;
}

esp_err_t ft6336u_read_simple(int16_t* x, int16_t* y, uint8_t* gesture, bool* pressed) {
    ft6336u_touch_data_t touch_data;
    esp_err_t ret = ft6336u_read_touch_data(&touch_data);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 输出手势
    if (gesture) {
        *gesture = touch_data.gesture_id;
    }
    
    // 输出第一个触摸点坐标
    if (x && y && touch_data.touch_points > 0) {
        *x = touch_data.points[0].x;
        *y = touch_data.points[0].y;
    }
    
    // 判断按下状态
    if (pressed && touch_data.touch_points > 0) {
        *pressed = (touch_data.points[0].event != FT6336U_EVENT_PUT_UP);
    }
    
    return ESP_OK;
}

esp_err_t ft6336u_set_mode(uint8_t mode) {
    uint8_t device_mode;
    esp_err_t ret = ft6336u_read_register(FT6336U_REG_DEVICE_MODE, &device_mode);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 设置模式位
    device_mode &= ~0x07;  // 清除模式位
    device_mode |= (mode & 0x07);
    
    return ft6336u_write_register(FT6336U_REG_DEVICE_MODE, device_mode);
}

esp_err_t ft6336u_set_touch_threshold(uint8_t threshold) {
    // 寄存器0x80: ID_G_THGROUP
    return ft6336u_write_register(0x80, threshold);
}

esp_err_t ft6336u_set_resolution(uint16_t width, uint16_t height) {
    esp_err_t ret;
    
    // 设置X分辨率（寄存器0x98-0x99）
    ret = ft6336u_write_register(0x98, (width >> 8) & 0xFF);  // ID_G_MAX_X_HIGH
    if (ret != ESP_OK) return ret;
    
    ret = ft6336u_write_register(0x99, width & 0xFF);         // ID_G_MAX_X_LOW
    if (ret != ESP_OK) return ret;
    
    // 设置Y分辨率（寄存器0x9A-0x9B）
    ret = ft6336u_write_register(0x9A, (height >> 8) & 0xFF); // ID_G_MAX_Y_HIGH
    if (ret != ESP_OK) return ret;
    
    return ft6336u_write_register(0x9B, height & 0xFF);       // ID_G_MAX_Y_LOW
}

esp_err_t ft6336u_enable_interrupt(bool enable) {
    // 寄存器0xA4: ID_G_MODE
    return ft6336u_write_register(0xA4, enable ? 0x00 : 0x01);
}

esp_err_t ft6336u_read_chip_id(uint8_t* chip_id) {
    // 寄存器0xA3: ID_G_CIPHER
    return ft6336u_read_register(0xA3, chip_id);
}

esp_err_t ft6336u_enter_sleep(void) {
    // 寄存器0xA5: ID_G_PMODE (设置为休眠模式)
    return ft6336u_write_register(0xA5, 0x03);
}

esp_err_t ft6336u_exit_sleep(void) {
    // 寄存器0xA5: ID_G_PMODE (设置为活动模式)
    return ft6336u_write_register(0xA5, 0x00);
}