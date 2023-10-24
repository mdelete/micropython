/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright 2023 Marc Delling
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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/byteorder.h>

#include "py/runtime.h"
#include "py/gc.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "py/objstr.h"
#include "modmachine.h"

#if MICROPY_PY_MACHINE_CAN

#define CAN_THREAD_STACK_SIZE    512
#define CAN_THREAD_PRIORITY      2
#define DEFAULT_CAN_BAUDRATE    (125000)

CAN_MSGQ_DEFINE(can_msgq, 2);
K_THREAD_STACK_DEFINE(can_rx_thread_stack, CAN_THREAD_STACK_SIZE);

/*
checkout zepyhrproject 3.4
checkout microphthon 1.20

    cd zephyrproject; mkdir nrf52840dongle_nrf52840_can; cd nrf52840dongle_nrf52840_can;

put board to zephyrproject/zephyr/boards/arm/nrf52840dongle_nrf52840_can

    $ west build -b nrf52840dongle_nrf52840_can ../../micropython/ports/zephyr -p

nrf52840dk:

    $ west flash

nrf52840dongle:

    $ nrfutil pkg generate --hw-version 52 --sd-req=0x00 --application build/zephyr/zephyr.hex --application-version 1 zephyr.zip
    $ nrfutil dfu usb-serial -pkg zephyr.zip -p /dev/ttyACM0

 * https://devicetree-specification.readthedocs.io/en/latest/chapter2-devicetree-basics.html

example:

from machine import CAN

def callback(obj):
	print("obj:", obj)

c = CAN(loopback=True, on_message=callback)

c.send(0x12, b'\x01\x02\x00\x00\x00')

*/

static struct k_thread can_rx_thread_data;
static struct can_bus_err_cnt current_err_cnt;
static enum can_state current_state;
static k_tid_t rx_tid;
static atomic_t received_messages = ATOMIC_INIT(0);

struct can_config {
    uint32_t baudrate;
    bool loopback;
};

typedef struct _machine_hard_can_obj_t {
    mp_obj_base_t base;
    mp_obj_t callback;
    const struct device *dev;
    struct can_config config;
} machine_hard_can_obj_t;

/*
static void can_rx_callback_function(const struct device *dev, struct can_frame *frame, void *user_data)
{
    machine_hard_can_obj_t *self = user_data;

    mp_obj_t tuple[3];
    tuple[0] = mp_obj_new_int(frame->id);
    tuple[1] = mp_obj_new_int(frame->flags);
    tuple[2] = mp_obj_new_bytes(frame->data, can_dlc_to_bytes(frame->dlc));
    mp_obj_t obj = mp_obj_new_tuple(3, tuple);

    if (self->callback != mp_const_none) {
        //mp_call_function_1_protected(self->callback, obj);
        mp_sched_schedule(self->callback, obj);
    }
}
*/

STATIC void can_rx_thread(void *arg1, void *arg2, void *arg3)
{
    machine_hard_can_obj_t *self = arg1;
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct can_frame msg;

    while (1) {
        k_msgq_get(&can_msgq, &msg, K_FOREVER);
        atomic_inc(&received_messages);

        mp_obj_t tuple[3];
        tuple[0] = mp_obj_new_int(msg.id);
        tuple[1] = mp_obj_new_int(msg.flags);
        tuple[2] = mp_obj_new_bytes(msg.data, can_dlc_to_bytes(msg.dlc));
        mp_obj_t obj = mp_obj_new_tuple(3, tuple);

        //gc_lock();
        if (self->callback != mp_const_none) {
            //mp_sched_lock();
            mp_sched_schedule(self->callback, obj);
            //mp_call_function_1_protected(ctx->callback, obj); // MP_OBJ_FROM_PTR(&mp_builtin_abs_obj)
            //mp_sched_unlock();
        }
        //gc_unlock();
    }
}

// https://docs.zephyrproject.org/apidoc/latest/group__can__interface.html#gac7ec472c26c564dd7067c49f67c8d2f7
STATIC const char* can_state_to_str(enum can_state state) {
    switch (state) {
    case CAN_STATE_ERROR_ACTIVE:
        return "error-active";
    case CAN_STATE_ERROR_WARNING:
        return "error-warning";
    case CAN_STATE_ERROR_PASSIVE:
        return "error-passive";
    case CAN_STATE_BUS_OFF:
        return "bus-off";
    case CAN_STATE_STOPPED:
        return "bus-stopped";
    default:
        return "unknown";
    }
}

// https://docs.zephyrproject.org/apidoc/latest/group__can__interface.html#ga446ee31913699de3c80be05d837b5fd5
STATIC const char* can_error_to_str(int err) {
    switch (err) {
    case EINVAL:
        return "EINVAL";
    case ENOTSUP:
        return "ENOTSUP";
    case ENETDOWN:
        return "ENETDOWN";
    case ENETUNREACH:
        return "ENETUNREACH";
    case EBUSY:
        return "EBUSY";
    case EIO:
        return "EIO";
    case EAGAIN:
        return "EAGAIN";
    default:
        return "ERROR";
    }
}

STATIC void state_change_callback(const struct device *dev, enum can_state state, struct can_bus_err_cnt err_cnt, void *user_data) {
    current_state = state;
    current_err_cnt = err_cnt;
}

STATIC void machine_hard_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_hard_can_obj_t *self = self_in;
    mp_printf(print, "CAN(\"%s\", baudrate=%u, loopback=%s, on_message=",
        self->dev->name,
        self->config.baudrate,
        self->config.loopback ? "True" : "False"
    );
    mp_obj_print_helper(print, self->callback, PRINT_REPR); // FIXME: don't print <function> but callbacks' name
    mp_printf(print, ")");
}

STATIC mp_obj_t machine_hard_can_on_message(mp_obj_t self_in, mp_obj_t obj_in) {

    machine_hard_can_obj_t *self = self_in;

    if (rx_tid > 0) {
        k_thread_abort(rx_tid);
    }

    if (mp_obj_is_fun(obj_in)) {
        self->callback = obj_in;

        rx_tid = k_thread_create(&can_rx_thread_data, can_rx_thread_stack, K_THREAD_STACK_SIZEOF(can_rx_thread_stack), can_rx_thread, self, NULL, NULL, CAN_THREAD_PRIORITY, 0, K_NO_WAIT);

        if (!rx_tid) {
            mp_raise_ValueError(MP_ERROR_TEXT("create can rx thread failed"));
        }
    } else {
        self->callback = mp_const_none;

        if (obj_in != mp_const_none) {
            mp_raise_ValueError(MP_ERROR_TEXT("callback is not a function"));
        }
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(can_obj_on_mesg_obj, machine_hard_can_on_message);

STATIC mp_obj_t machine_hard_can_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum {ARG_baudrate, ARG_loopback, ARG_on_message};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = DEFAULT_CAN_BAUDRATE} },
        { MP_QSTR_loopback, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_on_message, MP_ARG_OBJ | MP_ARG_REQUIRED,  {.u_obj = mp_const_none} },
    };

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all_kw_array(n_args, n_kw, all_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    struct can_config cfg = {
        .baudrate = args[ARG_baudrate].u_int,
        .loopback = args[ARG_loopback].u_bool,
    };

    const struct device *dev = DEVICE_DT_GET_OR_NULL(DT_CHOSEN(zephyr_canbus));

    if (!device_is_ready(dev)) {
        mp_raise_ValueError(MP_ERROR_TEXT("device not supported"));
    }

    // FIXME: customizable filters
    const struct can_filter filter = {
        .flags = CAN_FILTER_DATA
    };

    machine_hard_can_obj_t *self = mp_obj_malloc(machine_hard_can_obj_t, &machine_can_type);
    self->dev = dev;
    self->config = cfg;

    (void)machine_hard_can_on_message(self, args[ARG_on_message].u_obj);

    int ret = can_add_rx_filter_msgq(dev, &can_msgq, &filter);
    //int ret = can_add_rx_filter(dev, can_rx_callback_function, self, &filter);
    if (ret == -ENOSPC) {
        mp_raise_ValueError(MP_ERROR_TEXT("no filter available"));
    }

    can_set_mode(dev, args[ARG_loopback].u_bool ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL);
    can_set_state_change_callback(dev, state_change_callback, NULL);
    can_start(dev);

    return MP_OBJ_FROM_PTR(self);
}

STATIC void can_tx_callback(const struct device *dev, int error, void *user_data)
{
    if (error != 0) {
        mp_raise_ValueError(can_error_to_str(-error));
    }
}

#define mp_obj_is_bytes(o) (mp_obj_is_obj(o) && MP_OBJ_TYPE_GET_SLOT_OR_NULL(((mp_obj_base_t *)MP_OBJ_TO_PTR(o))->type, binary_op) == mp_obj_str_binary_op)

STATIC mp_obj_t machine_hard_can_send(mp_obj_t self_in, mp_obj_t canid_in, mp_obj_t obj_in) {
    machine_hard_can_obj_t *self = self_in;

    int ret;
    struct can_frame frame;

    if (mp_obj_is_small_int(canid_in)) {
        uint32_t canid = mp_obj_get_int(canid_in);
        if (canid > 0x1FFFFFFF) {
            mp_raise_ValueError(MP_ERROR_TEXT("invalid can id"));
        } else if (canid > 0x1FF) {
            frame.flags |= CAN_FRAME_IDE;
        } else {
            frame.flags = 0;
        }
        frame.id = canid;
    }
    else {
        mp_raise_ValueError(MP_ERROR_TEXT("invalid can id"));
    }

    // FIXME: obj_in may be int8 int16 int32 float int64 double byte[8]
    if (mp_obj_is_float(obj_in)) {
        float f = mp_obj_get_float(obj_in);
        memcpy(frame.data, &f, sizeof(float)); // FIXME: real32 net float?
        frame.dlc = can_bytes_to_dlc(sizeof(float));
    } else if (mp_obj_is_small_int(obj_in)) {
        int32_t val = (int32_t) mp_obj_get_int(obj_in);
        sys_put_be32(val, frame.data);
        frame.dlc = can_bytes_to_dlc(sizeof(int32_t));
    } else if (mp_obj_is_int(obj_in)) {
        uint64_t val = mp_obj_get_int(obj_in);
        sys_put_be64(val, frame.data);
        frame.dlc = can_bytes_to_dlc(sizeof(uint64_t));
    } else if (mp_obj_is_bytes(obj_in)) {
        GET_STR_DATA_LEN(obj_in, s, l);
        if(l > 8) {
            mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
        } else {
            memcpy(frame.data, s, l);
            frame.dlc = can_bytes_to_dlc(l);
        }
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("unsupported payload"));
    }

    //ret = can_send(self->dev, &frame, K_MSEC(25), NULL, NULL);
    ret = can_send(self->dev, &frame, K_FOREVER, can_tx_callback, NULL);

    if (ret < 0) {
        mp_raise_ValueError(can_error_to_str(-ret));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(can_obj_send_obj, machine_hard_can_send);

STATIC mp_obj_t machine_hard_can_status(mp_obj_t self_in) {

    int err;
    machine_hard_can_obj_t *self = self_in;

    err = can_get_state(self->dev, &current_state, &current_err_cnt);
    if (err != 0) {
        mp_raise_OSError(err);
    }

    const char* state = can_state_to_str(current_state);

    if (current_state == CAN_STATE_BUS_OFF) {
        if (can_recover(self->dev, K_MSEC(100)) != 0) {
            mp_raise_ValueError(MP_ERROR_TEXT("timeout trying to recover from bus-off"));
        }
    }

    return mp_obj_new_str(state, strlen(state));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(can_obj_status_obj, machine_hard_can_status);

STATIC mp_obj_t machine_hard_can_errors(mp_obj_t self) {
    mp_obj_t tuple[3];
    tuple[0] = mp_obj_new_int(atomic_get(&received_messages));
    tuple[1] = mp_obj_new_int(current_err_cnt.tx_err_cnt);
    tuple[2] = mp_obj_new_int(current_err_cnt.rx_err_cnt);
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(can_obj_errors_obj, machine_hard_can_errors);

STATIC const mp_rom_map_elem_t can_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_send),       MP_ROM_PTR(&can_obj_send_obj)    },
    { MP_ROM_QSTR(MP_QSTR_status),     MP_ROM_PTR(&can_obj_status_obj)  },
    { MP_ROM_QSTR(MP_QSTR_errors),     MP_ROM_PTR(&can_obj_errors_obj)  },
    { MP_ROM_QSTR(MP_QSTR_on_message), MP_ROM_PTR(&can_obj_on_mesg_obj) },
};
STATIC MP_DEFINE_CONST_DICT(mp_machine_can_locals_dict, can_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    machine_can_type,
    MP_QSTR_CAN,
    MP_TYPE_FLAG_NONE,
    make_new, machine_hard_can_make_new,
    print, machine_hard_can_print,
    locals_dict, &mp_machine_can_locals_dict
);

#endif // MICROPY_PY_MACHINE_CAN
