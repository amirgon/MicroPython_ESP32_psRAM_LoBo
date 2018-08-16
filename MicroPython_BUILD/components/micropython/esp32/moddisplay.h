#ifndef MICROPY_INCLUDED_ESP32_MODDISPLAY_H
#define MICROPY_INCLUDED_ESP32_MODDISPLAY_H

#include "py/obj.h"
#include "py/objint.h"
#include "py/runtime.h"

#include "machine_hw_spi.h"
#include "tft/tftspi.h"
#include "tft/tft.h"


typedef struct _display_tft_obj_t {
    mp_obj_base_t base;
    machine_hw_spi_obj_t *spi;
    display_config_t dconfig;
    exspi_device_handle_t disp_spi_dev;
    exspi_device_handle_t ts_spi_dev;
    exspi_device_handle_t *disp_spi;
    exspi_device_handle_t *ts_spi;
    uint32_t tp_calx;
    uint32_t tp_caly;
} display_tft_obj_t;


/*
 * tftspi.c low level driver uses some global variables
 * Here we set those variables so that multiple displays can be used
 */
//-------------------------------------------------
int TFT_setupDevice(display_tft_obj_t* disp_dev);

//--------------------------------------
 color_t intToColor(uint32_t cint);

#endif
