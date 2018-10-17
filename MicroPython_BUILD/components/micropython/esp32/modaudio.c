/*
 * This file is part of the MicroPython ESP32 project, https://github.com/amirgon/MicroPython_ESP32_psRAM_LoBo
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 amirgon (https://github.com/amirgon)
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

#include <string.h>
#include "esp_task_wdt.h"
#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "moddisplay.h"
#include "esp_heap_caps.h"
#include "soc/rtc_cntl_reg.h"

static const char TAG[] = "[AUDIO]";

#define LCD_INIT_TASK_PRIORITY      (CONFIG_MICROPY_TASK_PRIORITY + 2)
#define RECORDING_TASK_PRIORITY     (31)
#define RECORDING_TASK_STACK_SIZE   (1024*16)
#define LCD_INIT_STACK_SIZE         (1024*4)
#define I2S_NUM                     (I2S_NUM_0)
#define I2S_ADC_UNIT                (ADC_UNIT_1)
#define BITS_PER_SAMPLE             (16) // there is an assumption it's 16 bit so we use int16_t to store samples!
#define BYTES_PER_SAMPLE            (BITS_PER_SAMPLE/8)
#define SAMPLE_TYPECODE             'H'
#define DMA_BUF_SIZE                1024
#define ALPHA_SHIFT                 8
#define DEFAULT_DISPLAY_FACTOR      3
#define DEFAULT_FRAMERATE           25
#define LINE_ZCR_INDICATOR_WIDTH    5

// PINS

#define GPIO_TOUCH_RAIL         27 // REVERSED LOGIC! 1 means touch rail is disabled
#define GPIO_TOUCH_RAIL_ENABLE  0
#define GPIO_TOUCH_RAIL_DISABLE 1
#define GPIO_TOUCH              33

#define ABS(x) ((x)>=0? (x): -(x))
#define SWAP(type, var1, var2) do { type t = var2; var2 = var1; var1 = t; } while(0)

/*
 * Globals
 */

static volatile bool display_initialized = false;

/*
 * Recording Object Definition
 */

STATIC mp_obj_t audio_recording_close(mp_obj_t self_in);
STATIC mp_obj_t audio_recording_data(mp_obj_t self_in);
STATIC mp_obj_t audio_recording_wait(mp_obj_t self_in);
STATIC mp_obj_t audio_recording_aborted(mp_obj_t self_in);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_recording_close_obj, audio_recording_close);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_recording_data_obj, audio_recording_data);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_recording_wait_obj, audio_recording_wait);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_recording_aborted_obj, audio_recording_aborted);


STATIC const mp_rom_map_elem_t recording_locals_dict_table[] = {
        {MP_OBJ_NEW_QSTR(MP_QSTR_close), (mp_obj_t)&audio_recording_close_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&audio_recording_close_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR_data), (mp_obj_t)&audio_recording_data_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR_wait), (mp_obj_t)&audio_recording_wait_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR_aborted), (mp_obj_t)&audio_recording_aborted_obj},
};
STATIC MP_DEFINE_CONST_DICT(recording_locals_dict, recording_locals_dict_table);

STATIC void recording_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind );

STATIC mp_obj_t recording_make_new(const mp_obj_type_t *type,
										size_t n_args,
										size_t n_kw,
										const mp_obj_t *all_args);

STATIC const mp_obj_type_t audio_recording_type = {
    { &mp_type_type },
    .name = MP_QSTR_recording,
    .print = recording_print,
    .make_new = recording_make_new,
    .locals_dict = (mp_obj_dict_t*)&recording_locals_dict,
};

/*
 * Viewport object definition
 */

STATIC const mp_rom_map_elem_t viewport_locals_dict_table[] = {};

STATIC void viewport_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind );

STATIC mp_obj_t viewport_make_new(const mp_obj_type_t *type,
                                        size_t n_args,
                                        size_t n_kw,
                                        const mp_obj_t *all_args);

STATIC MP_DEFINE_CONST_DICT(viewport_locals_dict, viewport_locals_dict_table);

STATIC const mp_obj_type_t viewport_type = {
    { &mp_type_type },
    .name = MP_QSTR_viewport,
    .print = viewport_print,
    .make_new = viewport_make_new,
    .locals_dict = (mp_obj_dict_t*)&viewport_locals_dict,
};

/*
 * Audio module definitions
 */

STATIC mp_obj_t init();
STATIC mp_obj_t getAutoRecording();
STATIC mp_obj_t getAutoDisplay();

STATIC MP_DEFINE_CONST_FUN_OBJ_0(init_obj, init);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(getAutoRecording_obj, getAutoRecording);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(getAutoDisplay_obj, getAutoDisplay);

STATIC const mp_rom_map_elem_t audio_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_audio) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___init__), (mp_obj_t)&init_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_recording), (mp_obj_t)&audio_recording_type},
    { MP_OBJ_NEW_QSTR(MP_QSTR_viewport), (mp_obj_t)&viewport_type},
    { MP_OBJ_NEW_QSTR(MP_QSTR_getAutoRecording), (mp_obj_t)&getAutoRecording_obj},
    { MP_OBJ_NEW_QSTR(MP_QSTR_getAutoDisplay), (mp_obj_t)&getAutoDisplay_obj},
};

STATIC MP_DEFINE_CONST_DICT (
    mp_module_audio_globals,
    audio_globals_table
);


const mp_obj_module_t mp_module_audio = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_audio_globals,
};


/*
 * Module Implementation
 */

STATIC mp_obj_t init(){
    esp_log_level_set("I2S", ESP_LOG_DEBUG);
    esp_log_level_set("RTC_MODULE", ESP_LOG_DEBUG);
    esp_log_level_set("AUDIO", ESP_LOG_VERBOSE);
	return mp_const_none;
}


/*
 * Viewport implementation
 */

typedef struct viewport_t {
    mp_obj_base_t base;
    int16_t x0,y0,x1,y1;
    uint8_t displayFactor; // how much to shift-right each sample before drawing it
    color_t color;
    uint32_t frameRate;
} viewport_t;

STATIC mp_obj_t viewport_make_new(const mp_obj_type_t *type,
                                        size_t n_args,
                                        size_t n_kw,
                                        const mp_obj_t *all_args)
{
    // parse args
    enum {
        ARG_x0,
        ARG_y0,
        ARG_x1,
        ARG_y1,
        ARG_displayFactor,
        ARG_color,
        ARG_frameRate,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_x0, MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_y0, MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_x1, MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_y1, MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_displayFactor, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_DISPLAY_FACTOR}},
        { MP_QSTR_color, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1}},
        { MP_QSTR_frameRate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DEFAULT_FRAMERATE}},
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    viewport_t *self = m_new_obj(viewport_t);
    // give it a type
    self->base.type = &viewport_type;
    // set the members
    self->x0 = args[ARG_x0].u_int;
    self->x1 = args[ARG_x1].u_int;
    self->y0 = args[ARG_y0].u_int;
    self->y1 = args[ARG_y1].u_int;
    self->displayFactor = args[ARG_displayFactor].u_int;
    self->color = _fg;
    if (args[ARG_color].u_int >= 0) {
        self->color = intToColor(args[ARG_color].u_int);
    }
    self->frameRate = args[ARG_frameRate].u_int;

    return MP_OBJ_FROM_PTR(self);
}

STATIC void viewport_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind )
{
    // get a ptr to the C-struct of the object
    viewport_t *self = MP_OBJ_TO_PTR(self_in);

    // print the object
    mp_printf (print, "viewport(x0=%u, y0=%u, x1=%u, y1=%u, displayFactor=%u, color=%X, frameRate=%u)",
                self->x0, self->y0, self->x1, self->y1, self->displayFactor, self->color, self->frameRate);
}


/*
 * Recording implementation
 */

typedef struct audio_recording_t {
    mp_obj_base_t base;
    uint32_t freq; // In Hz
    uint32_t len;  // In bytes
    void *data;
    uint32_t alpha; // alpha scaled up ^ ALPHA_SHIFT, for low pass filter.
    uint32_t threshold; // minimum volume threshold
    uint32_t maxSilence; // Number of milliseconds of silence before cutting recording
    TaskHandle_t recordingTask;
    SemaphoreHandle_t done;
    bool aborted; // OUT: whether recording was aborted

    viewport_t *waveViewport;
    viewport_t *volumeViewport;

} audio_recording_t;


STATIC mp_obj_t audio_recording_data(mp_obj_t self_in)
{
    audio_recording_t *self = self_in;
    return self->data?
                mp_obj_new_memoryview(SAMPLE_TYPECODE, self->len / BYTES_PER_SAMPLE, self->data):
                mp_const_none;
}

STATIC mp_obj_t audio_recording_wait(mp_obj_t self_in)
{
    audio_recording_t *self = self_in;
    xSemaphoreTake(self->done, portMAX_DELAY);
    return mp_const_none;
}

STATIC mp_obj_t audio_recording_aborted(mp_obj_t self_in)
{
    audio_recording_t *self = self_in;
    return self->aborted? mp_const_true: mp_const_false;
}


static void recordingTask(void *self_in)
{
    audio_recording_t *self = self_in;
    self->aborted = false;
    size_t wr_size = self->len;
    void *wr_ptr = self->data;
    size_t bytes_read;
    size_t iter = 0;

    uint32_t displayLineRate = 0;
    uint32_t displayLineSampleCount = 0;
    uint32_t displayVolumeSampleCount = 0;
    uint32_t samplesToDisplayLine = 1;
    uint32_t samplesToDisplayVolume = 1;
    int16_t x = 0;
    int16_t mid_y = 0;
    int16_t displayLineMin = INT16_MAX;
    int16_t displayLineMax = INT16_MIN;
    int16_t prevDisplayLineMin = 0;
    int16_t prevDisplayLineMax= 0;
    int64_t displayVolAcc = 0;
    int16_t lastVol = 0;
    uint32_t lineZCR = 0;
    bool lineZCR_enable = false;
    uint32_t dmaBufSize = DMA_BUF_SIZE;
    uint16_t prevProgress = 0;

    if (self->waveViewport){
        displayLineRate = self->waveViewport->frameRate * (self->waveViewport->x1 - self->waveViewport->x0 + 1);
        displayLineSampleCount = self->freq / displayLineRate;
        x = self->waveViewport->x0;
        mid_y = (self->waveViewport->y0 + self->waveViewport->y1) / 2;
        dmaBufSize = MIN(dmaBufSize, self->freq / self->waveViewport->frameRate);
    }

    if (self->volumeViewport){
        displayVolumeSampleCount = self->freq / self->volumeViewport->frameRate;
        dmaBufSize = MIN(dmaBufSize, self->freq / self->volumeViewport->frameRate);
    }

    int32_t postSilenceQuota = 0;
    int32_t preSilenceQuota = (self->maxSilence * self->freq) / 1000;
    bool recordingStarted = false;

    static const gpio_config_t config_touch_rail = {
        .pin_bit_mask = 1ULL << GPIO_TOUCH_RAIL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_touch_rail);
    gpio_set_level(GPIO_MODE_OUTPUT, GPIO_TOUCH_RAIL_ENABLE);

    static const gpio_config_t config_touch = {
        .pin_bit_mask = 1ULL << GPIO_TOUCH,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&config_touch);

    i2s_adc_enable(I2S_NUM);

    while (wr_size > 0) {
        if (gpio_get_level(GPIO_TOUCH)){
            self->aborted = true;
            break;
        }
        esp_task_wdt_reset();

        //read data from I2S bus, in this case, from ADC.
        i2s_read(I2S_NUM, wr_ptr, MIN(dmaBufSize, wr_size), &bytes_read, portMAX_DELAY);

        const uint32_t samples_read = (bytes_read / BYTES_PER_SAMPLE);

        //TODO: average on fast memory (not psram). malloc DMA (in advance) according to I2S buffer size

        // Find average
        int64_t acc = 0;

        for (int16_t *p = wr_ptr; ((void*)p) < (wr_ptr+bytes_read); p++) {
            acc+=*p;
        }

        // move average to 0
        const int16_t avr = acc / samples_read;

        int64_t volAcc = 0;

        // Handle first sample specially, without applying the filter
        *((int16_t*) wr_ptr) -= avr;

        // Handle the rest of the data
        for (int16_t *p = wr_ptr+BYTES_PER_SAMPLE; ((void*)p) < (wr_ptr+bytes_read); p++) {

            // Average each sample

            const int16_t currentSample = *p - avr;

            // Apply low pass filter

            const int16_t prevSample = *(p-1);
            *p = prevSample + ((self->alpha*(currentSample - prevSample))>>ALPHA_SHIFT);

            // Count zero crossing (actually zero crossing followed by threshold crossing)

            lineZCR_enable = lineZCR_enable || (prevSample < 0 && *p >= 0);
            if (lineZCR_enable && prevSample < self->threshold && *p >= self->threshold) {
                lineZCR++;
                lineZCR_enable = false;
            }

            // Accumulate volume data for volume threshold

            volAcc += ABS(*p);

            // Display the waveform, synced to zero crossing

            if (lineZCR > 0) {

                // Accumulate wave data.

                displayLineMin = MIN(displayLineMin, *p);
                displayLineMax = MAX(displayLineMax, *p);

                // Display wave data

                if (display_initialized && self->waveViewport && --samplesToDisplayLine == 0){

                    samplesToDisplayLine = displayLineSampleCount;

                    const int16_t min = MAX(self->waveViewport->y0,
                            MIN(prevDisplayLineMax, mid_y + (displayLineMin >> self->waveViewport->displayFactor)));
                    const int16_t max = MIN(self->waveViewport->y1,
                            MAX(prevDisplayLineMin, mid_y + (displayLineMax >> self->waveViewport->displayFactor)));

                    TFT_drawLine(x, self->waveViewport->y0, x, self->waveViewport->y1 - LINE_ZCR_INDICATOR_WIDTH, _bg);
                    TFT_drawPixel(x, mid_y, TFT_RED, 1);
                    if (min <= self->waveViewport->y1 && max >= self->waveViewport->y0) {
                        TFT_drawLine(x, min, x, max, self->waveViewport->color);
                    }

                    if (x == self->waveViewport->x1) {
                        x = self->waveViewport->x0;
                        const int16_t progress = ((self->len - wr_size)  * (self->waveViewport->x1 - self->waveViewport->x0)) / self->len;
                        TFT_fillRect(self->waveViewport->x0 + prevProgress, self->waveViewport->y1-LINE_ZCR_INDICATOR_WIDTH,
                                progress - prevProgress,LINE_ZCR_INDICATOR_WIDTH, TFT_YELLOW);
                        prevProgress = progress;

                        // Following are attempts to visualize the zero corssing
/*
                        if (lineZCR  > 0) {
                            int w = (self->waveViewport->x1 - self->waveViewport->x0 + 1)/lineZCR;
                            if ( 1 < w && w <= (self->waveViewport->x1 - self->waveViewport->x0) / 2){
                                color_t color = TFT_RED;
                                for (int16_t x=self->waveViewport->x0; x < self->waveViewport->x1; x+=w){
                                    if (x+w > self->waveViewport->x1)
                                        TFT_fillRect(x, self->waveViewport->y1-LINE_ZCR_INDICATOR_WIDTH,
                                                self->waveViewport->x1-x+1, LINE_ZCR_INDICATOR_WIDTH, color);
                                    else TFT_fillRect(x, self->waveViewport->y1-LINE_ZCR_INDICATOR_WIDTH,
                                            w, LINE_ZCR_INDICATOR_WIDTH, color);
                                    SWAP(uint8_t, color.r, color.g);
                                }
                            }
                        } else TFT_fillRect(self->waveViewport->x0, self->waveViewport->y1-LINE_ZCR_INDICATOR_WIDTH,
                                self->waveViewport->x1 - self->waveViewport->x0 + 1, LINE_ZCR_INDICATOR_WIDTH, _bg);
*/


                        /*
                        if (lineZCR  > 0) {
                            //int w = (self->waveViewport->x1 - self->waveViewport->x0 + 1) - lineZCR;
                            int w = lineZCR<<2;
                            w = MAX(w, 1);
                            w = MIN(w, self->waveViewport->x1 - self->waveViewport->x0 + 1);
                            TFT_fillRect(self->waveViewport->x0, self->waveViewport->y1-LINE_ZCR_INDICATOR_WIDTH,
                                                 w, LINE_ZCR_INDICATOR_WIDTH, self->waveViewport->color);
                            TFT_fillRect(self->waveViewport->x0+w+1, self->waveViewport->y1-LINE_ZCR_INDICATOR_WIDTH,
                                    self->waveViewport->x1 - self->waveViewport->x0 - w - 1, LINE_ZCR_INDICATOR_WIDTH, _bg);
                        }
                        */

                        lineZCR = 0;
                    }
                    else x++;

                    prevDisplayLineMin = min;
                    prevDisplayLineMax = max;
                    displayLineMin = INT16_MAX;
                    displayLineMax = INT16_MIN;
                }
            }

            // Display the volume

            // Accumulate volume data for display

            displayVolAcc += ABS(*p);

            // Display volume data

            if (display_initialized && self->volumeViewport && --samplesToDisplayVolume == 0) {
                samplesToDisplayVolume = displayVolumeSampleCount;

                const int16_t w = self->volumeViewport->x1 - self->volumeViewport->x0;
                const int16_t h = self->volumeViewport->y1 - self->volumeViewport->y0;
                const int16_t vol = (displayVolAcc / displayVolumeSampleCount) >> self->volumeViewport->displayFactor;
                const int16_t vol_y0 = self->volumeViewport->y0 + h - vol;

                if (vol < lastVol && lastVol < h)
                    TFT_fillRect(self->volumeViewport->x0, self->volumeViewport->y0, w, h - lastVol, _bg);
                if (h > vol && vol > lastVol)
                    TFT_fillRect(self->volumeViewport->x0, vol_y0, w,
                        self->volumeViewport->y1 - vol_y0, self->volumeViewport->color);

                lastVol = vol;
                displayVolAcc = 0;
            }

        }

        //ESP_LOGD("AUDIO", "recordingTask:bytes_read = %zu, wr_ptr = %p, wr_size = %zu, avr = %d", bytes_read, wr_ptr, wr_size, avr);

        // Calculate volume for current buffer

        const uint32_t vol = volAcc / samples_read;

        // Handle recording start and stop. Take into account silence quota before and after recording

        if ((!recordingStarted) &&  vol < self->threshold){
            // If recording wasn't started yet, and volume is below threshold, don't start recording.
            // Make sure we do record some silence before recording starts, up to silence quota

            if (preSilenceQuota > 0){ // record silence at the beginning until quota
                wr_ptr += bytes_read;
                wr_size -= bytes_read;
                preSilenceQuota -= samples_read;
            } else {
                // Enough silence was recorded on the beginning.
                // A new buffer of silence was recorded, so shift it to omit the oldest silence buffer recorded
                // so we always have the most recent silence quota before wr_ptr
                memmove(self->data, self->data + bytes_read, wr_ptr - self->data);
            }
        } else {
            // Recording either started, or should be started now.

            if (vol >= self->threshold){
                // volume is high enough so it's time to start recording (or continue if already started).
                // reset the post silence quota
                recordingStarted = true;
                postSilenceQuota = (self->maxSilence * self->freq) / 1000;
            } else {
                // recording started but volume is low again.
                // decrease post silence quota. If quota is over, stop recording.
                postSilenceQuota -= samples_read;
                if (postSilenceQuota <= 0) {
                    // If recording stopped due to silence, set its length correctly to the actual data recorded.
                    self->len -= wr_size;
                    break; // stop recording
                }
            }

            wr_ptr += bytes_read;
            wr_size -= bytes_read;
        }

        iter++;
    }

    i2s_adc_disable(I2S_NUM);
    ESP_LOGD("AUDIO", "i2s_read was called %d times, buffer size is %d bytes. Recording length is %d bytes.", iter, dmaBufSize, self->len);
    xSemaphoreGive(self->done);
    vTaskDelete(NULL);
}

STATIC mp_obj_t recording_make_new(const mp_obj_type_t *type,
										size_t n_args,
										size_t n_kw,
										const mp_obj_t *all_args) {
    // parse args

    enum {
        ARG_channel,
        ARG_freq,
        ARG_seconds,

        ARG_alpha,
        ARG_threshold,
        ARG_maxSilence,

        ARG_display,
        ARG_waveViewport,
        ARG_volumeViewport,
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = ADC1_CHANNEL_0} },
        { MP_QSTR_freq, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 16000} },
        { MP_QSTR_seconds, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 10}},

        { MP_QSTR_alpha, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1UL<<ALPHA_SHIFT}},
        { MP_QSTR_threshold, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_maxSilence, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1000}},

        { MP_QSTR_display, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = NULL}},
        { MP_QSTR_waveViewport, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = NULL}},
        { MP_QSTR_volumeViewport, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = NULL}},

    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    audio_recording_t *self = m_new_obj(audio_recording_t);
	// give it a type
	self->base.type = &audio_recording_type;
	// set the members
	self->freq = args[ARG_freq].u_int;
	self->len = self->freq * args[ARG_seconds].u_int * BYTES_PER_SAMPLE;
	self->data = heap_caps_malloc(self->len, MALLOC_CAP_DEFAULT);
	if (!self->data){
	    mp_raise_msg(&mp_type_Exception, "Memory allocation failed!");
	}

	self->alpha = args[ARG_alpha].u_int;
	self->threshold = args[ARG_threshold].u_int;
	self->maxSilence = args[ARG_maxSilence].u_int;

	display_tft_obj_t *display = args[ARG_display].u_obj;
	if (display == mp_const_none) display = NULL;
	if (display) {
	    if (TFT_setupDevice(display)) {
	        mp_raise_msg(&mp_type_Exception, "Display needs to be initialized first!");
	    }
	}
	self->waveViewport = args[ARG_waveViewport].u_obj;
	if (self->waveViewport == mp_const_none) self->waveViewport = NULL;
	if (self->waveViewport && !display) {
	    mp_raise_msg(&mp_type_Exception, "Cannot set waveViewport without setting display!");
	}
	self->volumeViewport = args[ARG_volumeViewport].u_obj;
	if (self->volumeViewport == mp_const_none) self->volumeViewport = NULL;
    if (self->volumeViewport && !display) {
        mp_raise_msg(&mp_type_Exception, "Cannot set volumeViewport without setting display!");
    }

	i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate =  self->freq,
        .bits_per_sample = BITS_PER_SAMPLE,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 2,
        .dma_buf_len = DMA_BUF_SIZE
	};
	//install and start i2s driver
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    //init DAC pad
    i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
    //init ADC pad
    i2s_set_adc_mode(I2S_ADC_UNIT, args[ARG_channel].u_int);

    self->done = xSemaphoreCreateBinary();
    self->recordingTask = NULL;
    BaseType_t xReturned = xTaskCreate(recordingTask, "Recording Task", RECORDING_TASK_STACK_SIZE, self, RECORDING_TASK_PRIORITY, &self->recordingTask);
    if (xReturned != pdPASS){
        vTaskDelete(self->recordingTask);
        mp_raise_msg(&mp_type_Exception, "Failed creating recording task!");
    }

    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t audio_recording_close(mp_obj_t self_in) {
    audio_recording_t *self = self_in;
    i2s_driver_uninstall(I2S_NUM);
    if (self->data){
        vSemaphoreDelete(self->done);
        heap_caps_free(self->data);
        self->data = NULL;
    }
    return mp_const_none;
}

STATIC void recording_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind ) {
    // get a ptr to the C-struct of the object
	audio_recording_t *self = MP_OBJ_TO_PTR(self_in);

    // print the object
	if (self->data){
	    mp_printf (print, "recording(freq=%u, len=%u", self->freq, self->len);
	    if (self->aborted) mp_printf (print, ", ABORTED!");
        if (self->waveViewport){
            mp_printf (print, ", waveViewport=");
            viewport_print(print, self->waveViewport, kind);
        }
        if (self->volumeViewport){
            mp_printf (print, ", volumeViewport=");
            viewport_print(print, self->volumeViewport, kind);
        }

	    mp_printf (print, ")");
	} else {
	    mp_printf (print, "recording(closed)");
	}
}

/*
 * Startup code which automatically starts display and recording as fast as possible.
 * This is done before spiram is initialized since initialization takes some time.
 * After MP is up, it can access the already active display and recording that were
 * automatically started with "getAutoRecording" and "getAutoDispay" functions.
 */

static display_tft_obj_t autoDisplay = {
    .base = {.type = &display_tft_type},
    .spi = NULL,
    .dconfig = {
            .speed = 4000000,
            .rdspeed = 4000000,
            .type = DISP_TYPE_ILI9341,
            .host = HSPI_HOST,
            .miso = 19,
            .mosi = 18,
            .sck = 5,
            .cs = 13,
            .dc = 12,
            .tcs = -1,
            .rst = 4,
            .bckl = -1,
            .bckl_on = -1,
            .color_bits = 24,
            .gamma = 0,
            .width = DEFAULT_TFT_DISPLAY_WIDTH,
            .height = DEFAULT_TFT_DISPLAY_HEIGHT,
            .invrot = 0,
            .bgr = 0,
            .touch = TOUCH_TYPE_NONE
    },
    .disp_spi_dev = {
            .handle = NULL,
            .cs = -1,
            .dc = -1,
            .selected = 0
    },
    .ts_spi_dev = {
            .handle = NULL,
            .cs = -1,
            .dc = -1,
            .selected = 0
    },
    .disp_spi = &autoDisplay.disp_spi_dev,
    .ts_spi = &autoDisplay.ts_spi_dev
};

static void lcdInitTask(void *params)
{
    esp_err_t ret;

    // ================================
    // ==== Initialize the Display ====

    esp_log_level_set("[TFTSPI]", ESP_LOG_VERBOSE);

    disp_spi = autoDisplay.disp_spi;
    ts_spi = autoDisplay.ts_spi;

    ret = TFT_display_init(&autoDisplay.dconfig);
    if (ret != ESP_OK) {
           mp_raise_msg(&mp_type_OSError, "Error initializing display");
    }

    autoDisplay.dconfig.speed = spi_set_speed(disp_spi, autoDisplay.dconfig.speed);
    max_rdclock = find_rd_speed();
    autoDisplay.dconfig.rdspeed = max_rdclock;

    font_rotate = 0;
    text_wrap = 0;
    font_transparent = 0;
    font_forceFixed = 0;
    gray_scale = 0;
    TFT_setRotation(PORTRAIT);
    TFT_setGammaCurve(0);
    TFT_setFont(DEFAULT_FONT, NULL);
    TFT_resetclipwin();
    _fg = intToColor(iTFT_GREEN);
    bcklOn(&autoDisplay.dconfig);
    // TFT_print("ZTL", CENTER, (_height/2));
    display_initialized = true;
    vTaskDelete(NULL);
}

STATIC mp_obj_t getAutoDisplay(void)
{
    display_tft_obj_t *self = m_new_obj(display_tft_obj_t);
    memcpy(self, &autoDisplay, sizeof(*self));
    return MP_OBJ_FROM_PTR(self);
}

static TaskHandle_t lcdInitTaskHandle;

static viewport_t autoWaveViewport = {
        .base = {.type = &viewport_type},
        .x0= 70, .y0= 110, .x1 = 170, .y1 = 210,
        .displayFactor = 5,
        .color = {255,255,255},
        .frameRate = 10
};

static viewport_t autoVolumeVieport = {
        .base = {.type = &viewport_type},
        .x0= 175, .y0= 110, .x1 = 200, .y1 = 210,
        .displayFactor = 2,
        .color = {255,255,255},
        .frameRate = 10
};

static audio_recording_t autoRecording = {
        .base = {.type = &audio_recording_type},
        .freq = 16000,
        .len = 0, // Initialized below to maximum available block
        .data = NULL, // Initialized below
        .alpha = 256, // no low pass filter
        .threshold = 100,
        .maxSilence = 1000,
        .recordingTask = NULL, // initialized by xTaskCreate
        .done = NULL,
        .waveViewport = &autoWaveViewport,
        .volumeViewport = &autoVolumeVieport
    };

STATIC mp_obj_t getAutoRecording(void)
{
    audio_recording_t *self = m_new_obj(audio_recording_t);
    memcpy(self, &autoRecording, sizeof(*self));
    return MP_OBJ_FROM_PTR(self);
}

// Called very soon after deepsleep wakeup or reset.
// spiram is not configured yet! (to save time)
void startup(void)
{
    BaseType_t xReturned;

    // First allocate memory for the recording, in order to allocate the biggest block

    portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;
    taskENTER_CRITICAL(&myMutex);
    autoRecording.len = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
    autoRecording.data = heap_caps_malloc(autoRecording.len, MALLOC_CAP_DEFAULT);
    taskEXIT_CRITICAL(&myMutex);

    // Start initializing the display

    xReturned = xTaskCreate(lcdInitTask, "LCD init Task", LCD_INIT_STACK_SIZE, NULL, LCD_INIT_TASK_PRIORITY, &lcdInitTaskHandle);
    if (xReturned != pdPASS){
        vTaskDelete(lcdInitTaskHandle);
        ESP_LOGE(TAG, "Failed creating Auto Recording task!");
    }

    // Power audio

    gpio_set_direction(23, GPIO_MODE_OUTPUT);
    gpio_set_level(23, 1);

    // Start recording

    if (autoRecording.data == NULL){
        ESP_LOGE(TAG, "Failed allocating %d bytes for Auto Recording!", autoRecording.len);
    } else {
        ESP_LOGI(TAG, "Allocated %d bytes for Auto Recording (%d ms)", autoRecording.len, (1000*autoRecording.len) / (autoRecording.freq * BYTES_PER_SAMPLE));
        i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
            .sample_rate =  autoRecording.freq,
            .bits_per_sample = BITS_PER_SAMPLE,
            .communication_format = I2S_COMM_FORMAT_I2S_MSB,
            .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
            .intr_alloc_flags = 0,
            .dma_buf_count = 2,
            .dma_buf_len = DMA_BUF_SIZE
        };
        //install and start i2s driver
        i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
        //init DAC pad
        i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);
        //init ADC pad
        i2s_set_adc_mode(I2S_ADC_UNIT, ADC1_CHANNEL_0);

        autoRecording.done = xSemaphoreCreateBinary();

        // Turn on "recording" led
        gpio_set_direction(21, GPIO_MODE_OUTPUT);
        gpio_set_level(21, 1);

        xReturned = xTaskCreate(recordingTask, "Auto Recording Task", RECORDING_TASK_STACK_SIZE, &autoRecording, RECORDING_TASK_PRIORITY, &autoRecording.recordingTask);
        if (xReturned != pdPASS){
            vTaskDelete(autoRecording.recordingTask);
            ESP_LOGE(TAG, "Failed creating Auto Recording task!");
        }

        xSemaphoreTake(autoRecording.done, portMAX_DELAY);
        //vTaskDelay(10 / portTICK_RATE_MS);

        //ESP_LOGI(TAG, "Before RTC_CNTL_DG_PAD_FORCE_HOLD");
        //taskENTER_CRITICAL(&myMutex);
        //esp_sleep_enable_ext0_wakeup(27, 1);
        //SET_PERI_REG_MASK(RTC_CNTL_DIG_ISO_REG, RTC_CNTL_DG_PAD_FORCE_HOLD);
        //esp_deep_sleep_start();
        //taskEXIT_CRITICAL(&myMutex);
        //ESP_LOGI(TAG, "After RTC_CNTL_DG_PAD_FORCE_HOLD");
    }
}
