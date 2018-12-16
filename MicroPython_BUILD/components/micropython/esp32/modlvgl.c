/*
 * lv_micropy.c
 *
 *  Created on: Nov 17, 2018
 *      Author: amirgon
 *
 *  TODO: Move FreeRTOS/ESP32/ILI specific code to other files
 */


#include "py/obj.h"
#include "py/runtime.h"
#include "../../liblvgl/lvgl/lvgl.h"
#include "../../liblvgl/drv/ili9341.h"
#include "../../liblvgl/drv/disp_spi.h"
#include "../../liblvgl/drv/resistive_touch.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Micropython function implementation
 */

static const char TAG[] = "[LVGL]";

STATIC mp_obj_t mp_lv_task_handler(mp_obj_t arg)
{
    lv_task_handler();
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_lv_task_handler_obj, mp_lv_task_handler);

static void lv_task(void* param)
{
    while(1)
    {
        vTaskDelay(5);
        mp_sched_schedule((mp_obj_t)&mp_lv_task_handler_obj, mp_const_none);
    }
}

static void lv_tick_task(void)
{
    lv_tick_inc(portTICK_RATE_MS);
}


static lv_disp_drv_t disp_drv;
static lv_disp_t *disp;
static TaskHandle_t lvglTaskHandle;

static bool touch_read(lv_indev_data_t *data)
{
    static rtch_info_t touch_info;
    rtch_get_info(&touch_info);
    data->point = (lv_point_t){touch_info.x, touch_info.y};
    data->state = touch_info.touched? LV_INDEV_STATE_PR: LV_INDEV_STATE_REL;
    return false;
}

void lv_mp_init()
{
    BaseType_t xReturned;

    lv_init();

    disp_spi_init();
    ili9431_init();
    rtch_init(&(rtch_config_t){
        .xp = 32,  // X+
        .yp = 33,  // Y+
        .xm = 25,  // X-
        .ym = 26,  // Y-

        .touch_rail = 27, // pfet! 0 to enable.
        .touch_sense = 33, // Probably Y+ or Y-, when touch rail is enabled

        .screen_width = ILI9341_HOR_RES,
        .screen_height = ILI9341_VER_RES,
        .cal_x = 29821466,
        .cal_y = 14944964,
        .touch_samples = 9,
        .touch_samples_threshold= 500
    });

    lv_indev_drv_register(&(lv_indev_drv_t){
        .type = LV_INDEV_TYPE_POINTER,
        .read = touch_read
    });

    lv_disp_drv_init(&disp_drv);
    disp_drv.disp_flush = ili9431_flush;
    disp = lv_disp_drv_register(&disp_drv);

    esp_register_freertos_tick_hook(lv_tick_task);

    xReturned = xTaskCreate(lv_task, "LVGL Task", 4096, NULL, CONFIG_MICROPY_TASK_PRIORITY, &lvglTaskHandle);
    if (xReturned != pdPASS){
        vTaskDelete(lvglTaskHandle);
        ESP_LOGE(TAG, "Failed creating LVGL task!");
    }
}


