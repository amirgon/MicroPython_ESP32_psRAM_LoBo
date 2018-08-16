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

#include "py/nlr.h"
#include "py/obj.h"
#include "py/runtime.h"
#include "py/binary.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "moddisplay.h"

#define RECORDING_TASK_PRIORITY     (CONFIG_MICROPY_TASK_PRIORITY)
#define I2S_NUM                     (I2S_NUM_0)
#define I2S_ADC_UNIT                (ADC_UNIT_1)
#define BITS_PER_SAMPLE             (16) // there is an assumption it's 16 bit so we use int16_t to store samples!
#define BYTES_PER_SAMPLE            (BITS_PER_SAMPLE/8)
#define SAMPLE_TYPECODE             'H'
#define DMA_BUF_SIZE                1024
#define ALPHA_SHIFT                 8
#define DISPLAY_FACTOR              3


typedef struct audio_recording_t {
	mp_obj_base_t base;
	uint32_t freq; // In Hz
	uint32_t len;  // In bytes
	void *data;
	uint32_t alpha; // alpha scaled up ^ ALPHA_SHIFT, for low pass filter.
	TaskHandle_t recordingTask;

	display_tft_obj_t *display;
	int16_t x0,y0,x1,y1;
	uint32_t frameRate;
	uint8_t displayFactor; // how much to shift-right each sample before drawing it
	color_t color;

} audio_recording_t;

STATIC mp_obj_t audio_recording_close(mp_obj_t self_in);
STATIC mp_obj_t audio_recording_data(mp_obj_t self_in);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_recording_close_obj, audio_recording_close);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(audio_recording_data_obj, audio_recording_data);

STATIC const mp_rom_map_elem_t recording_locals_dict_table[] = {
        {MP_OBJ_NEW_QSTR(MP_QSTR_close), (mp_obj_t)&audio_recording_close_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&audio_recording_close_obj},
        {MP_OBJ_NEW_QSTR(MP_QSTR_data), (mp_obj_t)&audio_recording_data_obj},
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


STATIC mp_obj_t init(){
    esp_log_level_set("I2S", ESP_LOG_DEBUG);
    esp_log_level_set("RTC_MODULE", ESP_LOG_DEBUG);
    esp_log_level_set("AUDIO", ESP_LOG_VERBOSE);
	return mp_const_none;
}

STATIC mp_obj_t audio_recording_data(mp_obj_t self_in)
{
    audio_recording_t *self = self_in;
    return self->data?
                mp_obj_new_memoryview(SAMPLE_TYPECODE, self->len / BYTES_PER_SAMPLE, self->data):
                mp_const_none;
}

static void recordingTask(void *self_in)
{
    audio_recording_t *self = self_in;
    size_t wr_size = self->len;
    void *wr_ptr = self->data;
    int32_t alpha = self->alpha;
    size_t bytes_read;
    size_t iter = 0;

    bool display = self->display != NULL;
    uint32_t displayLineRate = self->frameRate * (self->x1 - self->x0 + 1);
    uint32_t displayLineSampleCount = self->freq / displayLineRate;
    uint32_t samplesToDisplayLine = 1;
    uint16_t x = self->x0;
    uint16_t mid_y = (self->y0 + self->y1) / 2;
    int16_t displayLineMin = INT16_MAX;
    int16_t displayLineMax = INT16_MIN;

    i2s_adc_enable(I2S_NUM);
    while (wr_size > 0) {
        vTaskDelay(1);

        //read data from I2S bus, in this case, from ADC.
        i2s_read(I2S_NUM, wr_ptr, MIN(DMA_BUF_SIZE, wr_size), &bytes_read, portMAX_DELAY);

        //TODO: average on fast memory (not psram). malloc DMA (in advance) according to I2S buffer size

        // Find average
        int64_t acc = 0;
        for (int16_t *p = wr_ptr; ((void*)p) < (wr_ptr+bytes_read); p++) acc+=*p;

        // move average to 0
        int16_t avr = acc / (bytes_read / BYTES_PER_SAMPLE);

        // Handle first sample specially, without applying the filter
        *((int16_t*) wr_ptr) -= avr;

        // Handle the rest of the data
        for (int16_t *p = wr_ptr+BYTES_PER_SAMPLE; ((void*)p) < (wr_ptr+bytes_read); p++) {

            // Average

            int16_t currentSample = *p - avr;

            // Apply low pass filter

            int16_t prevSample = *(p-1);
            *p = prevSample + ((alpha*(currentSample - prevSample))>>ALPHA_SHIFT);

            // Display the waveform

            displayLineMin = MIN(displayLineMin, *p);
            displayLineMax = MAX(displayLineMax, *p);

            if (display && --samplesToDisplayLine == 0){
                samplesToDisplayLine = displayLineSampleCount;

                TFT_drawLine(x, self->y0, x, self->y1, _bg);
                TFT_drawLine(x, MAX(self->y0, mid_y + (displayLineMin>>self->displayFactor)),
                             x, MIN(self->y1, mid_y + (displayLineMax>>self->displayFactor)),
                             self->color);

                if (x == self->x1) x = self->x0; else x++;
                displayLineMin = INT16_MAX;
                displayLineMax = INT16_MIN;
            }
        }

        //ESP_LOGD("AUDIO", "recordingTask:bytes_read = %zu, wr_ptr = %p, wr_size = %zu, avr = %d", bytes_read, wr_ptr, wr_size, avr);

        wr_ptr += bytes_read;
        wr_size -= bytes_read;
        iter++;
    }
    i2s_adc_disable(I2S_NUM);
    ESP_LOGD("AUDIO", "i2s_read was called %d times", iter);
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

        ARG_display,
        ARG_x0,
        ARG_y0,
        ARG_x1,
        ARG_y1,
        ARG_frameRate,
        ARG_displayFactor,
        ARG_color
    };

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_channel, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = ADC1_CHANNEL_0} },
        { MP_QSTR_freq, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 16000} },
        { MP_QSTR_seconds, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 10}},

        { MP_QSTR_alpha, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 1UL<<ALPHA_SHIFT}},

        { MP_QSTR_display, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_obj = NULL}},
        { MP_QSTR_x0, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_y0, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_x1, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_y1, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        { MP_QSTR_frameRate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 10}},
        { MP_QSTR_displayFactor, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = DISPLAY_FACTOR}},
        { MP_QSTR_color, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = -1}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    audio_recording_t *self = m_new_obj(audio_recording_t);
	// give it a type
	self->base.type = &audio_recording_type;
	// set the member number with the first argument of the constructor
	self->freq = args[ARG_freq].u_int;
	self->len = self->freq * args[ARG_seconds].u_int * BYTES_PER_SAMPLE;
	self->data = m_malloc(self->len);
	if (!self->data){
	    mp_raise_msg(&mp_type_Exception, "Memory allocation failed!");
	}

	self->alpha = args[ARG_alpha].u_int;

	self->display =  args[ARG_display].u_obj;
	self->x0 = args[ARG_x0].u_int;
	self->x1 = args[ARG_x1].u_int;
	self->y0 = args[ARG_y0].u_int;
	self->y1 = args[ARG_y1].u_int;
	self->frameRate = args[ARG_frameRate].u_int;
	self->displayFactor = args[ARG_displayFactor].u_int;
	self->color = _fg;
	if (self->display) {
	    if (TFT_setupDevice(self->display)) {
	        mp_raise_msg(&mp_type_Exception, "Display need to be initialized first!");
	    }
	}
	if (args[ARG_color].u_int >= 0) {
	    self->color = intToColor(args[ARG_color].u_int);
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

    self->recordingTask = NULL;
    BaseType_t xReturned = xTaskCreate(recordingTask, "Recording Task", 4096, self, RECORDING_TASK_PRIORITY, &self->recordingTask);
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
        m_free(self->data);
        self->data = NULL;
    }
    return mp_const_none;
}

STATIC void recording_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind ) {
    // get a ptr to the C-struct of the object
	audio_recording_t *self = MP_OBJ_TO_PTR(self_in);
    // print the number
	if (self->data){
	    mp_printf (print, "recording(freq=%u, len=%u", self->freq, self->len);
	    if (self->display){
	        mp_printf (print, ", display=true, x0=%u, y0=%u, x1=%u, y1=%u, frameRate=%u, displayFactor=%u, color=%u",
	                self->x0, self->y0, self->x1, self->y1, self->frameRate, self->displayFactor, self->color);
	    }
	    mp_printf (print, ")");
	} else {
	    mp_printf (print, "recording(closed)");
	}
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(init_obj, init);

STATIC const mp_rom_map_elem_t audio_globals_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__), MP_OBJ_NEW_QSTR(MP_QSTR_audio) },
	{ MP_OBJ_NEW_QSTR(MP_QSTR___init__), (mp_obj_t)&init_obj},
	{ MP_OBJ_NEW_QSTR(MP_QSTR_recording), (mp_obj_t)&audio_recording_type},
};

STATIC MP_DEFINE_CONST_DICT (
	mp_module_audio_globals,
    audio_globals_table
);


const mp_obj_module_t mp_module_audio = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_audio_globals,
};

