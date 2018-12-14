/*
 * resistive_touch.h
 *
 *  Created on: Dec 9, 2018
 *      Author: amirgon
 */

#ifndef COMPONENTS_LIBLVGL_DRV_RESISTIVE_TOUCH_H_
#define COMPONENTS_LIBLVGL_DRV_RESISTIVE_TOUCH_H_

#include "driver/gpio.h"

typedef struct _rtch_config_t
{
    gpio_num_t xp;  // X+
    gpio_num_t yp;  // Y+
    gpio_num_t xm;  // X-
    gpio_num_t ym;  // Y-
    gpio_num_t touch_rail; // pfet! 0 to enable.
    gpio_num_t touch_sense; // Probably Y+ or Y-, when touch rail is enabled

    uint32_t screen_width;
    uint32_t screen_height;
    uint32_t cal_x;
    uint32_t cal_y;
    uint32_t touch_samples; // number of samples to take on every touch measurement
    uint32_t touch_samples_threshold; // max distance between touch sample measurements for a valid touch reading
} rtch_config_t;

typedef struct _rtch_info_t
{
    int x;
    int y;
    bool touched;
} rtch_info_t;


bool rtch_init(const rtch_config_t *info);
//TODO: add deinit function

void rtch_get_info(rtch_info_t *info);



#endif /* COMPONENTS_LIBLVGL_DRV_RESISTIVE_TOUCH_H_ */
