/*
 * lv_micropy.c
 *
 *  Created on: Nov 17, 2018
 *      Author: amirgon
 *
 *  TODO: Move FreeRTOS/ESP32/ILI specific code to other files
 */


#include "py/obj.h"
#include "../../liblvgl/lvgl/lvgl.h"

#include "../../liblvgl/drv/ili9341.h"
#include "../../liblvgl/drv/disp_spi.h"
#include "esp_freertos_hooks.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/*
 * Micropython objects definitions
 */
#include "../../liblvgl/lvgl/micropython/lv_mpy.c"

/*
STATIC mp_obj_t display_close(mp_obj_t self_in);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(display_close_obj, display_close);

STATIC const mp_rom_map_elem_t display_locals_dict_table[] = {
        {MP_OBJ_NEW_QSTR(MP_QSTR_close), (mp_obj_t)&display_close_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&display_close_obj},
};
STATIC MP_DEFINE_CONST_DICT(display_locals_dict, display_locals_dict_table);


STATIC void display_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind );

STATIC mp_obj_t display_make_new(const mp_obj_type_t *type,
                                        size_t n_args,
                                        size_t n_kw,
                                        const mp_obj_t *all_args);

STATIC const mp_obj_type_t display_type = {
    { &mp_type_type },
    .name = MP_QSTR_display,
    .print = display_print,
    .make_new = display_make_new,
    .locals_dict = (mp_obj_dict_t*)&display_locals_dict,
};

STATIC mp_obj_t init();

STATIC MP_DEFINE_CONST_FUN_OBJ_0(init_obj, init);

STATIC const mp_rom_map_elem_t lvgl_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_lvgl) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___init__), (mp_obj_t)&init_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_display), (mp_obj_t)&display_type},
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_lvgl_globals,
    lvgl_globals_table
);

const mp_obj_module_t mp_module_lvgl = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_lvgl_globals,
};

*/


/*
 * Micropython function implementation
 */

static const char TAG[] = "[LVGL]";

static void lv_tick_task(void)
{
    lv_tick_inc(portTICK_RATE_MS);
}

static void lv_task(void* param)
{
    while(1)
    {
        vTaskDelay(5);
        lv_task_handler();
    }
}

static lv_disp_drv_t disp_drv;
static lv_disp_t *disp;
static TaskHandle_t lvglTaskHandle;

void lv_mp_init()
{
    BaseType_t xReturned;

    lv_init();

    disp_spi_init();
    ili9431_init();

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


STATIC void display_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind )
{

}

STATIC mp_obj_t display_make_new(const mp_obj_type_t *type,
                                        size_t n_args,
                                        size_t n_kw,
                                        const mp_obj_t *all_args)
{

    /*Create a style for the Preloader*/
    static lv_style_t style;
    lv_style_copy(&style, &lv_style_plain);
    style.line.width = 10;                         /*10 px thick arc*/
    style.line.color = LV_COLOR_HEX3(0x258);       /*Blueish arc color*/

    style.body.border.color = LV_COLOR_HEX3(0xBBB); /*Gray background color*/
    style.body.border.width = 10;
    style.body.padding.hor = 0;

    /*Create a Preloader object*/
    lv_obj_t * preload = lv_preload_create(lv_scr_act(), NULL);
    lv_obj_set_size(preload, 100, 100);
    lv_obj_align(preload, NULL, LV_ALIGN_CENTER, 0, 0);
    lv_preload_set_style(preload, LV_PRELOAD_STYLE_MAIN, &style);


    /*Create a Label on the currently active screen*/
    lv_obj_t * label1 =  lv_label_create(lv_scr_act(), NULL);

    /*Modify the Label's text*/
    lv_label_set_text(label1, "Hello world!");

    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, 0);

    return mp_const_none;
}

STATIC mp_obj_t display_close(mp_obj_t self_in)
{
    return mp_const_none;
}
