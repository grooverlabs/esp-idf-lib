/**
 * @file mpr121.h
 * @defgroup mpr121 mpr121
 * @{
 *
 * ESP-IDF driver for MPR121 Capacitive Touch Sensor
 *
 * Copyright (C) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __MPR121_H__
#define __MPR121_H__

#include <stdbool.h>
#include <i2cdev.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MPR121_I2C_ADDR_00 0x5A //!< I2C address, pin ADDR = VSS
#define MPR121_I2C_ADDR_01 0x5B //!< I2C address, pin ADDR = VDD
#define MPR121_I2C_ADDR_10 0x5C //!< I2C address, pin ADDR = SDA
#define MPR121_I2C_ADDR_11 0x5D //!< I2C address, pin AdDR = SCL

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param addr Device I2C address
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t mpr121_init_desc(i2c_dev_t *dev, uint8_t addr, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mpr121_free_desc(i2c_dev_t *dev);

/**
 * @brief Init device
 *
 * Set device configuration to default, clear lock bits
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t mpr121_init(i2c_dev_t *dev);
/**
 * @brief Set the touch and release threshold for all inputs to the provided values.
 *
 * @param dev Device descriptor
 * @param touch
 * @param release
 * @return `ESP_OK` on success
 */
esp_err_t mpr121_set_thresholds(i2c_dev_t *dev, uint8_t touch, uint8_t release);

/**
 * @brief Get which pin was touch if any
 *
 * @param dev Device descriptor
 * @param touched
 * @return `ESP_OK` on success
 */
esp_err_t mpr121_get_touched(i2c_dev_t *dev, uint16_t *touched);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __MPR121_H__ */
