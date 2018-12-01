/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * Development of the code in this file was sponsored by Microbric Pty Ltd
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2016 Damien P. George
 *
 * Copyright (c) 2017 Boris Lovosevic (External SPIRAM support)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * ######################################################
 * This is only the wrapper to the main MicroPython file:
 * components/micropython/esp32/main.c
 * ######################################################
 */


extern void micropython_entry(void);

/* startup_task before micropython is started and before spiram is initialized.
 * This allows it to start very fast after waking from deep sleep or reset.
 * Need to configure SPIRAM_BOOT_INIT to "NO" so it will not be configure when on cpu_start.c
 * Instead we confgire spiram here, after calling to startup_task
 */
extern void startup(void);

#include <stdint.h>
#include "esp_attr.h"
#include "esp_err.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_spiram.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"

static bool s_spiram_okay=true;
static const char* TAG = "app_main_start";

//===================
void app_main(void) {

    //startup();

    // If spiram was not configured during boot time, configure it now

#if !CONFIG_SPIRAM_BOOT_INIT
    esp_spiram_init_cache();
    if (esp_spiram_init() != ESP_OK) {
#if CONFIG_SPIRAM_IGNORE_NOTFOUND
        ESP_EARLY_LOGI(TAG, "Failed to init external RAM; continuing without it.");
        s_spiram_okay = false;
#else
        ESP_EARLY_LOGE(TAG, "Failed to init external RAM!");
        abort();
#endif
    }


#if CONFIG_SPIRAM_MEMTEST
    if (s_spiram_okay) {
        bool ext_ram_ok=esp_spiram_test();
        if (!ext_ram_ok) {
            ESP_EARLY_LOGE(TAG, "External RAM failed memory test!");
            abort();
        }
    }
#endif

    if (s_spiram_okay) {
#if (CONFIG_SPIRAM_USE_CAPS_ALLOC || CONFIG_SPIRAM_USE_MALLOC)
        esp_err_t r=esp_spiram_add_to_heapalloc();
        if (r != ESP_OK) {
            ESP_EARLY_LOGE(TAG, "External RAM could not be added to heap!");
            abort();
        }
#if CONFIG_SPIRAM_USE_MALLOC
        heap_caps_malloc_extmem_enable(CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL);
#endif
#endif
    }

#endif

    micropython_entry();
}

