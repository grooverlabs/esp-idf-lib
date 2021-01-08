/**
 * @file ht16k33.c
 *
 * ESP-IDF driver for HT16K33 Display
 *
 * Copyright (C) 2020 Paul Maseberg <pdmaseberg@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <stdio.h>
#include <esp_log.h>
//#include "driver/i2c.h"
//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
#include "esp_system.h"

#include <esp_idf_lib_helpers.h>
#include "ht16k33_7seg.h"

// TODO RTH
//#define I2C_MASTER_SDA_IO          21
//#define I2C_MASTER_SCL_IO          22
//#define I2C_MASTER_NUM             I2C_NUM_0
//#define I2C_MASTER_TX_BUF_ENABLE   0
//#define I2C_MASTER_RX_BUF_ENABLE   0

#define I2C_FREQ_HZ 400000 // 400 kHz

#define HT16K33_ADDR               0x70

#define HT16K33_BLINK_CMD          0x80
#define HT16k33_BRIGHTNESS_CMD     0xE0
#define HT16K33_BLINK_DISPLAYON    0x01
#define HT16K33_7SEG_DIGITS        5
#define HT16K33_OSCILLATOR_CMD     0x21


static uint16_t _display_buffer[8];
static uint8_t _position = 0;

static char *TAG = "HT16K33";

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

static const uint8_t _number_table[] =
{
    0x3F, /* 0 */
    0x06, /* 1 */
    0x5B, /* 2 */
    0x4F, /* 3 */
    0x66, /* 4 */
    0x6D, /* 5 */
    0x7D, /* 6 */
    0x07, /* 7 */
    0x7F, /* 8 */
    0x6F, /* 9 */
    0x77, /* a */
    0x7C, /* b */
    0x39, /* C */
    0x5E, /* d */
    0x79, /* E */
    0x71, /* F */
};

static inline esp_err_t write_reg_8_nolock(i2c_dev_t *dev, uint8_t data)
{
    return i2c_dev_write(dev, NULL, 0, &data, 1);
}

// Usefully info about the chip
// https://www.partsnotincluded.com/controlling-led-matrix-with-the-ht16k33/
esp_err_t ht16k33_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->port = port;
    dev->addr = addr;
    dev->cfg.sda_io_num = sda_gpio;
    dev->cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->cfg.master.clk_speed = I2C_FREQ_HZ;
#endif

    return i2c_dev_create_mutex(dev);
}

esp_err_t ht16k33_free_desc(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(dev);
}

esp_err_t ht16k33_init(i2c_dev_t *dev)
{
    CHECK_ARG(dev);

    ESP_LOGD(TAG, "ht16k33_init");

    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev,  HT16K33_OSCILLATOR_CMD));

    CHECK(ht16k33_set_blink_rate(dev, HT16K33_BLINK_OFF));
    CHECK(ht16k33_set_brightness(dev, 10));

    return ESP_OK;
}

esp_err_t ht16k33_write_display(i2c_dev_t *dev)
{
    uint8_t i;
    uint8_t outreg;
    uint8_t data[16];

    for (i = 0; i < 8; i++)
    {
        data[2 * i] = _display_buffer[i] & 0xFF;
        data[2 * i + 1] = _display_buffer[i] >> 8;
    }

    // start at address 0x0
    outreg = 0;
    I2C_DEV_CHECK(dev, i2c_dev_write(dev, &outreg, 1, data, 16));

    return ESP_OK;
}

esp_err_t ht16k33_set_blink_rate(i2c_dev_t *dev, uint8_t b)
{
    ESP_LOGD(TAG, "set_blink_rate %d", b);


    // Page 10 or ht16K33v110.pdf
    // 0b00 : Blinking off
    // ob01 : 2 Hz
    // ob10 : 1 Hz
    // ob11 : 0.5 Hz

    // HT16K33_BLINK_CMD          0x80
    // HT16K33_BLINK_DISPLAYON    0x01

    // TODO check b is less then or equal to 2
    if (b > 3) b = 0; // turn off if not sure

    // HT16K33_ADDR               0x70
    //i2c_master_write_byte(cmd, (HT16K33_ADDR << 1) | I2C_MASTER_WRITE, 1);
    //i2c_master_write_byte(cmd, data_cmd, 1);

    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (b << 1)));
    return ESP_OK;

}

esp_err_t ht16k33_set_brightness(i2c_dev_t *dev, uint8_t b)
{
    ESP_LOGD(TAG, "set brighness %d", b);

    // TODO check b is less then or equal to 2
    if (b > 15) b = 15;

    //i2c_master_write_cmd(I2C_MASTER_NUM, HT16k33_BRIGHTNESS_CMD | b);

    I2C_DEV_CHECK(dev, write_reg_8_nolock(dev,  HT16k33_BRIGHTNESS_CMD | b));
    return ESP_OK;
}

void ht16k33_write_digit_raw(uint8_t d, uint8_t bitmask)
{
    ESP_LOGD(TAG, "write_digit_raw, d %d bitmask %d", d, bitmask);
    if (d > 4) return;
    _display_buffer[d] = bitmask;
}

void ht16k33_write_digit_num(uint8_t d, uint8_t num, uint8_t dot)
{
    ESP_LOGD(TAG, "write_digit_num %d, %d, %d", d, num,
             _number_table[num] | (dot << 7));

    if (d > 4) return;

    ht16k33_write_digit_raw(d, _number_table[num] | (dot << 7));
}

size_t write(uint8_t c)
{
    uint8_t r = 0;

    ESP_LOGD(TAG, "write %c %d", c, c);

    if (c == '\n') _position = 0;
    if (c == '\r') _position = 0;

    if ((c >= '0') && (c <= '9'))
    {
        ht16k33_write_digit_num(_position, c - '0', 0);
        r = 1;
    }

    _position++;
    if (_position == 2) _position++;

    return r;
}

void ht16k33_clear(i2c_dev_t *dev)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
        _display_buffer[i] = 0;
}

static void _print_error(void)
{
    uint8_t i;
    ESP_LOGE(TAG, "Print Error");
    for (i = 0; i < HT16K33_7SEG_DIGITS; ++i)
        ht16k33_write_digit_raw(i, (i == 2 ? 0x00 : 0x40));
}

void print_float(double n, uint8_t fracDigits, uint8_t base)
{

    uint8_t numericDigits = 4;   // available digits on display
    uint8_t i;
    uint8_t isNegative = 0;  // true if the number is negative
    double toIntFactor = 1.0;
    uint32_t displayNumber;
    uint32_t tooBig = 1;

    // is the number negative?
    if (n < 0)
    {

        isNegative = 1;  // need to draw sign later
        --numericDigits;    // the sign will take up one digit
        n *= -1;            // pretend the number is positive
    }

    // calculate the factor required to shift all fractional digits
    // into the integer part of the number
    for (i = 0; i < fracDigits; ++i) toIntFactor *= base;

    // create integer containing digits to display by applying
    // shifting factor and rounding adjustment
    displayNumber = n * toIntFactor + 0.5;

    // calculate upper bound on displayNumber given
    // available digits on display
    for (i = 0; i < numericDigits; ++i) tooBig *= base;

    // if displayNumber is too large, try fewer fractional digits
    while (displayNumber >= tooBig)
    {
        --fracDigits;
        toIntFactor /= base;
        displayNumber = n * toIntFactor + 0.5;
    }

    // did toIntFactor shift the decimal off the display?
    if (toIntFactor < 1)
        _print_error();
    else
    {
        // otherwise, display the number
        int8_t displayPos = 4;

        if (displayNumber)  //if displayNumber is not 0
        {
            for (i = 0; displayNumber || i <= fracDigits; ++i)
            {
                uint8_t displayDecimal = (fracDigits != 0 && i == fracDigits);
                ht16k33_write_digit_num(displayPos--, displayNumber % base, displayDecimal);
                if (displayPos == 2) ht16k33_write_digit_raw(displayPos--, 0x00);
                displayNumber /= base;
            }
        }
        else
            ht16k33_write_digit_num(displayPos--, 0, 0);

        // display negative sign if negative
        if (isNegative) ht16k33_write_digit_raw(displayPos--, 0x40);

        // clear remaining display positions
        while (displayPos >= 0) ht16k33_write_digit_raw(displayPos--, 0x00);
    }
}

void ht16k33_draw_colon(uint8_t state)
{
    if (state)
        _display_buffer[2] = 0x2;
    else
        _display_buffer[2] = 0;
}

void print_number(long n, uint8_t base)
{
    print_float(n, 0, base);
}

void _print(unsigned long n, uint8_t base)
{
    if (base == 0) write(n);
    else print_number(n, base);
}

void _print_char(char c, uint8_t base)
{
    ESP_LOGD(TAG, "print char");
    _print((long) c, base);
}

void _print_uchar(unsigned char b, uint8_t base)
{
    ESP_LOGD(TAG, "print uchar");
    _print((unsigned long) b, base);
}

void _print_int(int n, uint8_t base)
{
    ESP_LOGD(TAG, "print int");
    _print((long) n, base);
}

void _print_uint(unsigned int n, uint8_t base)
{
    ESP_LOGD(TAG, "print uint");
    _print((unsigned long) n, base);
}

void _print_double(double n, uint8_t digits)
{
    ESP_LOGD(TAG, "print double");
    print_float(n, digits, 10);
}

void _println(void)
{
    ESP_LOGD(TAG, "println");
    _position = 0;
}
