/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Marc Delling
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

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include "py/runtime.h"
#include "modmachine.h"

#if MICROPY_PY_MACHINE

#define N_LEDS 3

typedef struct _machine_led_obj_t {
    mp_obj_base_t base;
    int id;
    bool state;
} machine_led_obj_t;

static const struct gpio_dt_spec leds[N_LEDS] = {
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0}),
    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0}),
};

STATIC void machine_led_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_led_obj_t *self = self_in;
    mp_printf(print, "Led(%u, on=%p)", self->id, self->state);
}

mp_obj_t mp_led_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum {ARG_id, ARG_on};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_id, MP_ARG_INT | MP_ARG_REQUIRED, {.u_int = -1} },
        { MP_QSTR_on, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    int led_id = args[ARG_id].u_int;
    bool state = args[ARG_on].u_bool;

    if (led_id < 0 || led_id > N_LEDS) {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid led"));
    }

    if (!device_is_ready(leds[led_id].port)) {
        mp_raise_ValueError(MP_ERROR_TEXT("device not ready"));
    }

    int err = gpio_pin_configure_dt(&leds[led_id], GPIO_OUTPUT_INACTIVE);
    if (err != 0) {
        mp_raise_ValueError(MP_ERROR_TEXT("not configured"));
    }
    gpio_pin_set_dt(&leds[led_id], state);

    machine_led_obj_t *led = mp_obj_malloc(machine_led_obj_t, &machine_led_type);
    led->id = led_id;
    led->state = state;

    return (mp_obj_t)led;
}

STATIC mp_obj_t machine_led_obj_toggle(mp_obj_t self_in) {
    machine_led_obj_t *self = self_in;
    self->state = !self->state;
    gpio_pin_set_dt(&leds[self->id], self->state);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(machine_led_toggle_obj, machine_led_obj_toggle);

STATIC mp_obj_t machine_led_obj_set(mp_obj_t self_in, mp_obj_t state_in) {
    machine_led_obj_t *self = self_in;
    self->state = mp_obj_is_true(state_in);
    gpio_pin_set_dt(&leds[self->id], self->state);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(machine_led_set_obj, machine_led_obj_set);

STATIC const mp_rom_map_elem_t machine_led_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set),    MP_ROM_PTR(&machine_led_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_toggle), MP_ROM_PTR(&machine_led_toggle_obj) },
};
STATIC MP_DEFINE_CONST_DICT(machine_led_locals_dict, machine_led_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_led_type,
    MP_QSTR_Led,
    MP_TYPE_FLAG_NONE,
    make_new, mp_led_make_new,
    print, machine_led_print,
    locals_dict, &machine_led_locals_dict
);

#endif // MICROPY_PY_MACHINE
