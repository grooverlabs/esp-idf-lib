#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <string.h>
#include <esp_err.h>
#include <mpr121.h>

/* float is used in printf(). you need non-default configuration in
 * sdkconfig for ESP8266, which is enabled by default for this
 * example. see sdkconfig.defaults.esp8266
 */

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define SDA_GPIO 4
#define SCL_GPIO 5
#else
#define SDA_GPIO 15
#define SCL_GPIO 4
#endif

#if defined(CONFIG_IDF_TARGET_ESP32S2)
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define ADDR MPR121_I2C_ADDR_00

void task(void *pvParamters)
{
    esp_err_t res;

    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(mpr121_init_desc(&dev, ADDR, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(mpr121_init(&dev));

    while (1)
    {
        // Get the values and do something with them.

        uint16_t touches;
        if ((res = mpr121_get_touched(&dev, &touches)) != ESP_OK)
            printf("Could not get results: %d (%s)", res, esp_err_to_name(res));
        //else
        //printf("Touched: %02x\n", touches);

        uint16_t pin;
        for (pin = 0; pin < 12; pin++)
        {
            if ((touches & (1 << pin)) > 0)
            {
                printf("Touched pin: %d\n", pin);
            }
        }

        //printf("I am alive\n");
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreatePinnedToCore(task, "mpr121_test", configMINIMAL_STACK_SIZE * 8, NULL, 5, NULL, APP_CPU_NUM);
}
