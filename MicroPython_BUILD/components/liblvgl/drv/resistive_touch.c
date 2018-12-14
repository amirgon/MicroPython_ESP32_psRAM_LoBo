/*
 * resistive_touch.c
 *
 *  Created on: Dec 1, 2018
 *      Author: amirgon
 */

#include <string.h>
#include "resistive_touch.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

static const char TAG[] = "[RTCH]";

#define INVALID_MEASUREMENT INT32_MIN

#define RTCH_TASK_STACK_SIZE (8*1024)
#define RTCH_TASK_PRIORITY (CONFIG_MICROPY_TASK_PRIORITY+1)

#ifndef RTCH_MAX_TOUCH_SAMPLES
#define RTCH_MAX_TOUCH_SAMPLES 16
#endif

#ifndef RTCH_SAMPLE_WAIT_MS
#define RTCH_SAMPLE_WAIT_MS 1
#endif

#ifndef RTCH_TOUCH_WAIT_MS
#define RTCH_TOUCH_WAIT_MS 10
#endif

#ifndef RTCH_INIT_ADC_WAIT_MS
#define RTCH_INIT_ADC_WAIT_MS 20
#endif

#ifndef CONCAT3
#define _CONCAT3(a,b,c) a ## b ## c
#define CONCAT3(a,b,c) _CONCAT3(a,b,c)
#endif

#define GPIO_TO_ADC_ELEMENT(x) [x] = CONCAT3(ADC1_GPIO, x, _CHANNEL)
static const int gpio_to_adc[] = {
        GPIO_TO_ADC_ELEMENT(36),
        GPIO_TO_ADC_ELEMENT(37),
        GPIO_TO_ADC_ELEMENT(38),
        GPIO_TO_ADC_ELEMENT(39),
        GPIO_TO_ADC_ELEMENT(32),
        GPIO_TO_ADC_ELEMENT(33),
        GPIO_TO_ADC_ELEMENT(34),
        GPIO_TO_ADC_ELEMENT(35),
};

static rtch_config_t rtch_config;
static rtch_info_t rtch_info;

static xTaskHandle rtch_task_handle;
static SemaphoreHandle_t rtch_info_mutex;
static esp_err_t esp_res = ESP_OK;


static int compare_int(const void *_a, const void *_b)
{
    int *a = (int*)_a;
    int *b = (int*)_b;
    return *a - *b;
}

static int measure_axis(
        gpio_num_t plus,
        gpio_num_t minus,
        gpio_num_t measure,
        gpio_num_t ignore)
{
    // Set GPIOs:

    // - Disable touch rail
    gpio_set_level(rtch_config.touch_rail, 1);

    // - Float "ignore" and "measure"
    gpio_pad_select_gpio(ignore);
    gpio_set_direction(ignore, GPIO_MODE_DISABLE);
    gpio_set_pull_mode(ignore, GPIO_FLOATING);

    gpio_pad_select_gpio(measure);
    gpio_set_direction(measure, GPIO_MODE_DISABLE);
    gpio_set_pull_mode(measure, GPIO_FLOATING);

    // - Set "plus" to 1, "minus" to 0
    esp_res = gpio_config(&(gpio_config_t){
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<plus) | (1ULL<<minus)
    });
    gpio_set_level(plus, 1);
    gpio_set_level(minus, 0);

    // Init ADC

    adc1_channel_t adc_channel = gpio_to_adc[measure];

    adc_gpio_init(ADC_UNIT_1, adc_channel);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(adc_channel,ADC_ATTEN_DB_11);

    vTaskDelay(RTCH_INIT_ADC_WAIT_MS / portTICK_RATE_MS);

    // Collect ADC samples and sort them

    static int samples[RTCH_MAX_TOUCH_SAMPLES];
    int sample_count = rtch_config.touch_samples;
    for (int i=0; i<sample_count; i++)
    {
        //vTaskDelay(RTCH_SAMPLE_WAIT_MS / portTICK_RATE_MS);
        samples[i] = adc1_get_raw(adc_channel);
    }
    qsort(samples, sample_count, sizeof(samples[0]), compare_int);

    // Make sure samples are close to each other

    int prevSample = INVALID_MEASUREMENT;
    for (int i=0; i<sample_count; i++)
    {
        //ESP_LOGD(TAG, "RAW Sample %d: [%d]", i, samples[i]);
        int sample = samples[i];
        if (prevSample != INVALID_MEASUREMENT &&
                abs(sample - prevSample) > rtch_config.touch_samples_threshold) {
            return INVALID_MEASUREMENT;
        }
        prevSample = sample;
    }

    // return median

    return samples[sample_count / 2];
}

static void enable_touch_sense()
{
    // Configure all touch pins to high impedance (input)

    esp_res = gpio_config(&(gpio_config_t){
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask =
            (1ULL<<rtch_config.xp) |
            (1ULL<<rtch_config.yp) |
            (1ULL<<rtch_config.xm) |
            (1ULL<<rtch_config.ym)
    });

    // Enable touch rail

    esp_res = gpio_config(&(gpio_config_t){
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<rtch_config.touch_rail),
    });
    esp_res = gpio_set_level(rtch_config.touch_rail, 0);

    // Configure touch sense and configure interrupt

    esp_res = gpio_config(&(gpio_config_t){
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1,
        .pin_bit_mask = (1ULL<<rtch_config.touch_sense),
    });

    // Wait for touch rail to stabilize

    vTaskDelay(RTCH_TOUCH_WAIT_MS / portTICK_RATE_MS);
}

static void IRAM_ATTR rtch_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Disable gpio interrupt

    gpio_intr_disable(rtch_config.touch_sense);

    // Notify the task

    xTaskNotifyFromISR(
            (TaskHandle_t)arg,
            0,
            eNoAction,
            &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}


static void rtch_task(void* arg)
{
    for( ;; )
    {
        // Enable Interrupt

        //gpio_intr_enable(rtch_config.touch_sense);
        gpio_isr_handler_add(rtch_config.touch_sense, rtch_isr_handler, xTaskGetCurrentTaskHandle());

        // Wait for interrupt

        ESP_LOGD(TAG, "Waiting for interrupt...");

        xTaskNotifyWait(
                0x00,      /* Don't clear any notification bits on entry. */
                0x00,      /* Don't clear any notification bits on exit. */
                NULL,      /* Notified value ignored. */
                portMAX_DELAY );  /* Block indefinitely. */

        ESP_LOGD(TAG, "Touched!");

        // Touch detected. Loop until untouched

        while(gpio_get_level(rtch_config.touch_sense))
        {
            ESP_LOGD(TAG, "Measuring...");

            // measure X and Y
            int x = measure_axis(
                    rtch_config.xp,
                    rtch_config.xm,
                    rtch_config.yp,
                    rtch_config.ym);

            int y = measure_axis(
                    rtch_config.yp,
                    rtch_config.ym,
                    rtch_config.xp,
                    rtch_config.xm);

            ESP_LOGD(TAG, "RAW: [%d, %d]", x, y);

            // If measurements valid, calculate calibrated X and Y
            if (x != INVALID_MEASUREMENT && y != INVALID_MEASUREMENT)
            {
                int xleft   = (rtch_config.cal_x >> 16) & 0x3FFF;
                int xright  = rtch_config.cal_x & 0x3FFF;
                int ytop    = (rtch_config.cal_y >> 16) & 0x3FFF;
                int ybottom = rtch_config.cal_y & 0x3FFF;
                x = ((x - xleft) * rtch_config.screen_width) / (xright - xleft);
                y = ((y - ytop) * rtch_config.screen_height) / (ybottom - ytop);
                if (x >= 0 && y >= 0 && x < rtch_config.screen_width && y < rtch_config.screen_height)
                {
                    ESP_LOGD(TAG, "[%d, %d]", x, y);

                    // Update touch info
                    xSemaphoreTake(rtch_info_mutex, portMAX_DELAY);
                    rtch_info.touched = true;
                    rtch_info.x = x;
                    rtch_info.y = y;
                    xSemaphoreGive(rtch_info_mutex);
                } else {
                    ESP_LOGD(TAG, "Overflow measurement [%d, %d]", x, y);
                }
            } else {
                ESP_LOGD(TAG, "Invalid measurement");
            }

            enable_touch_sense();
        }

        ESP_LOGD(TAG, "Untouched!");

        // Untouched. Update touch info
        xSemaphoreTake(rtch_info_mutex, portMAX_DELAY);
        rtch_info.touched = false;
        xSemaphoreGive(rtch_info_mutex);
    }
}

bool rtch_init(const rtch_config_t *info)
{
    rtch_config = *info;
    //esp_log_level_set(TAG, ESP_LOG_DEBUG);
    //esp_log_level_set("gpio", ESP_LOG_DEBUG);
    //esp_log_level_set("RTC_MODULE", ESP_LOG_DEBUG);

    BaseType_t xReturned = xTaskCreate(rtch_task, "RTCH Task", RTCH_TASK_STACK_SIZE, NULL, RTCH_TASK_PRIORITY, &rtch_task_handle);

    if (xReturned != pdPASS){
        ESP_LOGE(TAG, "Failed createing RTCH task!");
        vTaskDelete(rtch_task_handle);
        return false;
    }

    rtch_info_mutex = xSemaphoreCreateMutex();

    adc_power_on();
    enable_touch_sense();

    // Install Interrupt

    esp_res = gpio_install_isr_service(0);

    ESP_LOGD(TAG, "RTCH Initialized");

    return true;
}

void rtch_get_info(rtch_info_t *info)
{
    xSemaphoreTake(rtch_info_mutex, portMAX_DELAY);
    *info = rtch_info;
    xSemaphoreGive(rtch_info_mutex);
}
