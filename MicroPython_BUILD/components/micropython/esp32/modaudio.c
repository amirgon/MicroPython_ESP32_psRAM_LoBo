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

typedef struct audio_recording_t {
	mp_obj_base_t base;
	uint32_t freq;
	uint32_t len;
	void *data;
} audio_recording_t;


STATIC const mp_rom_map_elem_t recording_locals_dict_table[] = { };
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
	return mp_const_none;
}

STATIC mp_obj_t recording_make_new(const mp_obj_type_t *type,
										size_t n_args,
										size_t n_kw,
										const mp_obj_t *all_args) {
    // parse args
    enum { ARG_freq, ARG_len};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_freq, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 16000} },
        { MP_QSTR_len, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 10}},
    };
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    audio_recording_t *self = m_new_obj(audio_recording_t);
	// give it a type
	self->base.type = &audio_recording_type;
	// set the member number with the first argument of the constructor
	self->freq = args[ARG_freq].u_int;
	self->len = args[ARG_len].u_int;
	return MP_OBJ_FROM_PTR(self);
}

STATIC void recording_print( const mp_print_t *print,
                                  mp_obj_t self_in,
                                  mp_print_kind_t kind ) {
    // get a ptr to the C-struct of the object
	audio_recording_t *self = MP_OBJ_TO_PTR(self_in);
    // print the number
	mp_printf (print, "recording(freq=%u, len=%u)", self->freq, self->len);
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

