/**
 * @file mpr121.c
 *
 * ESP-IDF driver for MPR121 Capacitive Touch Sensor
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_idf_lib_helpers.h>
#include "mpr121.h"

#include <math.h>
#include <esp_log.h>

static const char *TAG = "MPR121";

#define I2C_FREQ_HZ 400000 // 400 kHz

// Register addresses.
// Datasheet
// https://cdn-shop.adafruit.com/datasheets/MPR121.pdf
#define MPR121_I2CADDR_DEFAULT 0x5A
#define MPR121_TOUCHSTATUS_L 0x00
#define MPR121_TOUCHSTATUS_H 0x01
#define MPR121_FILTDATA_0L 0x04
#define MPR121_FILTDATA_0H 0x05
#define MPR121_BASELINE_0 0x1E
#define MPR121_MHDR 0x2B
#define MPR121_NHDR 0x2C
#define MPR121_NCLR 0x2D
#define MPR121_FDLR 0x2E
#define MPR121_MHDF 0x2F
#define MPR121_NHDF 0x30
#define MPR121_NCLF 0x31
#define MPR121_FDLF 0x32
#define MPR121_NHDT 0x33
#define MPR121_NCLT 0x34
#define MPR121_FDLT 0x35
#define MPR121_TOUCHTH_0 0x41
#define MPR121_RELEASETH_0 0x42
#define MPR121_DEBOUNCE 0x5B
#define MPR121_CONFIG1 0x5C
#define MPR121_CONFIG2 0x5D
#define MPR121_CHARGECURR_0 0x5F
#define MPR121_CHARGETIME_1 0x6C
#define MPR121_ECR 0x5E
#define MPR121_AUTOCONFIG0 0x7B
#define MPR121_AUTOCONFIG1 0x7C
#define MPR121_UPLIMIT 0x7D
#define MPR121_LOWLIMIT 0x7E
#define MPR121_TARGETLIMIT 0x7F
#define MPR121_GPIODIR 0x76
#define MPR121_GPIOEN 0x77
#define MPR121_GPIOSET 0x78
#define MPR121_GPIOCLR 0x79
#define MPR121_GPIOTOGGLE 0x7A
#define MPR121_SOFTRESET 0x80
#define MAX_I2C_RETRIES = 5

// commands
#define MPR121_SOFTRESET_CMD 0x63
#define MPR121_RESET_PERIOD  1      // reset time in ms

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)


static inline esp_err_t read_reg_8_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t *data)
{
    return i2c_dev_read_reg(dev, reg, data, 1);
}

static inline esp_err_t write_reg_8_nolock(i2c_dev_t *dev, uint8_t reg, uint8_t data)
{
    return i2c_dev_write_reg(dev, reg, &data, 1);
}

// TODO should I removed this or should I be reading with the lock? PM
static esp_err_t read_reg_8(i2c_dev_t *dev, uint8_t reg, uint8_t *data)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_8_nolock(dev, reg, data));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

static inline esp_err_t read_reg_16_nolock(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    CHECK(i2c_dev_read_reg(dev, reg, val, 2));
    *val = (*val << 8) | (*val >> 8);
    return ESP_OK;
}

static esp_err_t read_reg_16(i2c_dev_t *dev, uint8_t reg, uint16_t *val)
{
    I2C_DEV_TAKE_MUTEX(dev);
    I2C_DEV_CHECK(dev, read_reg_16_nolock(dev, reg, val));
    I2C_DEV_GIVE_MUTEX(dev);

    return ESP_OK;
}

esp_err_t mpr121_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    // I2C Address verification is not needed because the device may have a custom factory address

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t mpr121_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t mpr121_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    // https://github.com/adafruit/Adafruit_Python_MPR121/blob/master/Adafruit_MPR121/MPR121.py
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_SOFTRESET, MPR121_SOFTRESET_CMD));
    //This 1ms delay here probably isn't necessary but can't hurt.
    vTaskDelay(pdMS_TO_TICKS(MPR121_RESET_PERIOD));

    // MPR121 must be put in Stop Mode to write to most registers
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_ECR, 0));

    // Check CDT, SFI, ESI configuration is at default values.
    uint8_t c = 0;
    I2C_DEV_CHECK(dev, read_reg_8_nolock(dev, MPR121_CONFIG2, &c));
    if (c != 0x24)
    {
        I2C_DEV_GIVE_MUTEX(dev);
        ESP_LOGE(TAG, "CDT CONFIG %02x is wrong, should be 0x24", c);
        return ESP_ERR_NOT_FOUND;
    }

    CHECK(mpr121_set_thresholds(dev, 12, 6));

    // Configure baseline filtering control registers.
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_MHDR, 0x01));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_NHDR, 0x01));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_NCLR, 0x0E));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_FDLR, 0x00));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_MHDF, 0x01));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_NHDF, 0x05));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_NCLF, 0x01));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_FDLF, 0x00));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_NHDT, 0x00));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_NCLT, 0x00));
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_FDLT, 0x00));

    // Set other configuration registers.
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_DEBOUNCE, 0));

    // See section 5.8 of the datasheet
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_CONFIG1, 0x10)); // default, 16uA charge current
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_CONFIG2, 0x20)); // 0.5uS encoding, 1ms period

    // The docs says 0x2C but adafriut as 0x8F... I am not yet sure what the value is PM.
    // See section 5.11 of the datasheet

    // 10 00 1111 0x8F
    // 00 10 1100 0x2C
    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_ECR, 0x8F));

    return ESP_OK;
}

esp_err_t mpr121_set_thresholds(i2c_dev_t *dev, uint8_t touch, uint8_t release)
{

    uint8_t i;
    for (i = 0; i < 12; i++)
    {
        I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_TOUCHTH_0 + 2 * i, touch));
        I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, MPR121_RELEASETH_0 + 2 * i, release));
    }

    return ESP_OK;
}


esp_err_t mpr121_get_touched(i2c_dev_t *dev, uint16_t *touched)
{
    CHECK_ARG(dev && touched);

    uint16_t temp;
    CHECK(read_reg_16(dev, MPR121_TOUCHSTATUS_L, &temp));

    *touched = (temp << 8 & 0xFF00) | (temp >> 8 & 0x00FF);

    return ESP_OK;
}
