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
#include <zephyr/sys/barrier.h>

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

#ifdef CONFIG_CANOPEN_LSS
#define LSS_STEP_TIMEOUT (K_MSEC(125))
K_MSGQ_DEFINE(sdo_msgq, sizeof(struct can_frame), 1, 1);
K_SEM_DEFINE(lss_sem_ident_slave, 1, 1);
K_SEM_DEFINE(lss_sem_cfg_node_id, 1, 1);
static const struct can_frame lss_master_tpl = { .id = 0x7e5, .res0 = 0, .dlc = 8, .flags = 0, 0, .data = { 0, 0, 0, 0, 0, 0, 0, 0 } };
static const struct can_frame nmt_master_tpl = { .id = 0x000, .res0 = 0, .dlc = 2, .flags = 0, 0, .data = { 0, 0, 0, 0, 0, 0, 0, 0 } };
#endif

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

enum nmt_state {
    nmt_operational = 0x01,       // pdo works
    nmt_stop = 0x02,              // only nmt works
    nmt_pre_operational = 0x80,   // everything except pdo works
    nmt_reset_node = 0x81,        // clear config and reset node
    nmt_reset_communcation = 0x82 // reset node
};

STATIC void can_tx_callback(const struct device *dev, int error, void *user_data)
{
    if (error != 0) {
        mp_raise_ValueError(can_error_to_str(-error));
    }
}

#ifdef CONFIG_CANOPEN_LSS
STATIC void nmt_set_state (const struct device *dev, enum nmt_state state, uint8_t canid)
{
    struct can_frame frame = nmt_master_tpl;

    frame.data[0] = state;
    frame.data[1] = canid;

    can_send(dev, &frame, K_FOREVER, can_tx_callback, NULL);
}

STATIC bool lss_fast_scan_send(const struct device *dev, uint32_t addr_number, uint8_t bit_check, uint8_t addr_part, uint8_t addr_next)
{
    int err;
    struct can_frame frame = lss_master_tpl;

    frame.data[0] = 0x51;
	UNALIGNED_PUT(addr_number, (uint32_t *)&frame.data[1]);
    frame.data[5] = bit_check;
    frame.data[6] = addr_part;
    frame.data[7] = addr_next;

    k_sem_take(&lss_sem_ident_slave, K_MSEC(25));

    can_send(dev, &frame, K_FOREVER, can_tx_callback, NULL);

    err = k_sem_take(&lss_sem_ident_slave, LSS_STEP_TIMEOUT);
    if (err == 0) {
        return true;
    }

    return false;
}

enum lss_global_state {
	waiting_state = 0x00,
	configuration_state = 0x01,
};

STATIC void lss_switch_state_global (const struct device *dev, enum lss_global_state state)
{
    struct can_frame frame = lss_master_tpl;

    frame.data[0] = 0x04;
    frame.data[1] = (state == configuration_state) ? 1 : 0;

    can_send(dev, &frame, K_FOREVER, can_tx_callback, NULL);
}

STATIC bool lss_configure_node (const struct device *dev, uint8_t canid)
{
    int err;
    bool ret = false;
    struct can_frame frame = lss_master_tpl;

    frame.data[0] = 0x11;
    frame.data[1] = canid;

    can_send(dev, &frame, K_FOREVER, can_tx_callback, NULL);

    err = k_sem_take(&lss_sem_cfg_node_id, LSS_STEP_TIMEOUT);
    if (err == 0) {
        ret = true;
    }

    lss_switch_state_global(dev, waiting_state);
    return ret;
}

STATIC bool lss_fast_scan (const struct device *dev, mp_obj_t* components[5])
{
    uint8_t lss_sub;
    uint8_t bit_checked;
    uint32_t lss_number;
    uint8_t id = mp_obj_get_int(components[4]);

    if (id < 1 || id > 0x7f)
        mp_raise_ValueError(MP_ERROR_TEXT("invalid can id"));

    if (!lss_fast_scan_send(dev, 0, 0x80U, 0, 0))
        return false;

    for (lss_sub = 0; lss_sub < 3; lss_sub++)
    {
        if (!mp_obj_is_small_int(components[lss_sub]))
            break;
        lss_number = mp_obj_get_int(components[lss_sub]);
        //mp_printf(&mp_plat_print, "lss_number: %d\n", lss_number);
        if (!lss_fast_scan_send(dev, lss_number, 0, lss_sub, lss_sub+1))
            return false;
    }
    //mp_printf(&mp_plat_print, "lss_sub: %d\n", lss_sub);
    for (; lss_sub < 4; lss_sub++)
    {
        lss_number = 0;
        bit_checked = 0x1FU;
        while (true)
        {
            if (!lss_fast_scan_send(dev, lss_number, bit_checked, lss_sub, lss_sub))
                lss_number |= 1UL << bit_checked;
            if (bit_checked == 0x00U)
                break;
            bit_checked--;
        }
        if (!lss_fast_scan_send(dev, lss_number, bit_checked, lss_sub, (lss_sub == 3) ? 0 : (lss_sub+1))) {
            return false;
        }
        components[lss_sub] = mp_obj_new_int(lss_number);
    }

    return lss_configure_node(dev, id);
}
#endif

STATIC void can_rx_thread(void *arg1, void *arg2, void *arg3)
{
    machine_hard_can_obj_t *self = arg1;
    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);
    struct can_frame msg;

    while (1) {
        k_msgq_get(&can_msgq, &msg, K_FOREVER);
        atomic_inc(&received_messages);

#ifdef CONFIG_CANOPEN_LSS
        if (msg.id == 0x7e4) {
            if (msg.data[0] == 0x4f) {
                k_sem_give(&lss_sem_ident_slave);
            } else if (msg.data[0] == 0x11) {
                k_sem_give(&lss_sem_cfg_node_id);
            }
            continue;
        } else if (msg.id > 0x580 || msg.id < 0x600) {
            while (k_msgq_put(&sdo_msgq, &msg, K_NO_WAIT) != 0) {
                k_msgq_purge(&sdo_msgq);
            }
            continue;
        }
#endif

        mp_obj_t tuple[3];
        tuple[0] = mp_obj_new_int(msg.id);
        tuple[1] = mp_obj_new_int(msg.flags);
        tuple[2] = mp_obj_new_bytes(msg.data, can_dlc_to_bytes(msg.dlc));
        mp_obj_t obj = mp_obj_new_tuple(3, tuple);

        if (self->callback != mp_const_none) {
            mp_sched_schedule(self->callback, obj); // use mp_sched_schedule_node when not using it in REPL
        }
    }
}

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
    mp_obj_print_helper(print, self->callback, PRINT_REPR); // FIXME: don't print <function> but the callbacks' name
    mp_printf(print, ")");
}

STATIC mp_obj_t machine_hard_can_on_message(mp_obj_t self_in, mp_obj_t obj_in) {

    machine_hard_can_obj_t *self = self_in;

    if (rx_tid > 0) {
        k_thread_abort(rx_tid);
    }

    if (mp_obj_is_fun(obj_in)) {
        self->callback = obj_in;
    } else {
        self->callback = mp_const_none;
        if (obj_in != mp_const_none) {
            mp_raise_ValueError(MP_ERROR_TEXT("callback is not a function"));
        }
    }

    rx_tid = k_thread_create(&can_rx_thread_data, can_rx_thread_stack, K_THREAD_STACK_SIZEOF(can_rx_thread_stack), can_rx_thread, self, NULL, NULL, CAN_THREAD_PRIORITY, 0, K_NO_WAIT);

    if (!rx_tid) {
        mp_raise_ValueError(MP_ERROR_TEXT("create can rx thread failed"));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(can_obj_on_mesg_obj, machine_hard_can_on_message);

STATIC mp_obj_t sdo_download_expedited (size_t n_args, const mp_obj_t *args) // write
{
    machine_hard_can_obj_t *self = args[0];

    uint8_t nodeid = mp_obj_get_int(args[1]);    // 1<0xFF
    uint16_t idx   = mp_obj_get_int(args[2]);    // 0<0x10000
    uint8_t subidx = mp_obj_get_int(args[3]);    // 0<0x100
    uint8_t len    = mp_obj_get_int(args[4]);

    struct can_frame frame;
    memset(&frame, 0, sizeof(frame));

    if (mp_obj_is_float(args[5])) {
        float f = (float) mp_obj_get_float(args[5]);
        memcpy(&frame.data[4], &f, sizeof(float));
    } else if (mp_obj_is_small_int(args[5])) {
        uint32_t i = (uint32_t) mp_obj_get_int(args[5]);
        memcpy(&frame.data[4], &i, sizeof(uint32_t));
    } else if (mp_obj_is_type(args[5], &mp_type_bytes)) {
        GET_STR_DATA_LEN(args[5], s, l);
        if(l > len) {
            mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
        } else {
            memcpy(&frame.data[4], s, l);
        }
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("unsupported payload"));
    }

    frame.id = 0x600 + nodeid;
    frame.dlc = 8;
    frame.data[0] = ((4 - len) << 2) | 0x23;
    UNALIGNED_PUT(idx, (uint16_t *)&frame.data[1]);
    frame.data[3] = subidx;

    can_send(self->dev, &frame, K_FOREVER, can_tx_callback, NULL);

	struct can_frame reply;
	while (k_msgq_get(&sdo_msgq, &reply, K_MSEC(125)) == 0) {
		uint8_t i = reply.id & 0x7f;
		if (i == nodeid)
			return mp_obj_new_bool(true);
	}

	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(can_obj_sdo_exp_download_obj, 6, 6, sdo_download_expedited);

STATIC mp_obj_t sdo_upload_expedited (size_t n_args, const mp_obj_t *args) // read
{
    machine_hard_can_obj_t *self = args[0];

    uint8_t  nodeid = mp_obj_get_int(args[1]);    // 1<0xFF
    uint16_t idx    = mp_obj_get_int(args[2]);    // 0<0x10000
    uint8_t  subidx = mp_obj_get_int(args[3]);    // 0<0x100

    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0x600 + nodeid;
    frame.dlc = 8;
    frame.data[0] = 0x40;
    UNALIGNED_PUT(idx, (uint16_t *)&frame.data[1]);
    frame.data[3] = subidx;

    can_send(self->dev, &frame, K_FOREVER, can_tx_callback, NULL);

    struct can_frame reply;
    while (k_msgq_get(&sdo_msgq, &reply, K_MSEC(125)) == 0) {
        uint8_t i = reply.id & 0x7f;
        float f = *((float*)&reply.data[4]);
        //mp_printf(&mp_plat_print, "sdo: %02x %f\n", i, f);
        if (i == nodeid)
            return mp_obj_new_float(f);
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(can_obj_sdo_exp_upload_obj, 4, 4, sdo_upload_expedited);

// FIXME: de-init
STATIC mp_obj_t machine_hard_can_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *all_args) {
    enum {ARG_baudrate, ARG_loopback, ARG_on_message};

    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_baudrate, MP_ARG_INT, {.u_int = DEFAULT_CAN_BAUDRATE} },
        { MP_QSTR_loopback, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_on_message, MP_ARG_OBJ | MP_ARG_KW_ONLY,  {.u_obj = mp_const_none} },
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
    if (ret == -ENOSPC) {
        mp_raise_ValueError(MP_ERROR_TEXT("no filter available"));
    }

    can_set_mode(dev, cfg.loopback ? CAN_MODE_LOOPBACK : CAN_MODE_NORMAL);
    can_set_state_change_callback(dev, state_change_callback, NULL);
    can_start(dev);

#ifdef CONFIG_CANOPEN_LSS
    nmt_set_state(dev, nmt_reset_communcation, 0);
#endif

    return MP_OBJ_FROM_PTR(self);
}

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
    } else if (mp_obj_is_type(obj_in, &mp_type_bytes)) {
        GET_STR_DATA_LEN(obj_in, s, l);
        if(l > 8) {
            mp_raise_ValueError(MP_ERROR_TEXT("payload too large"));
        } else {
            memcpy(frame.data, s, l);
            frame.dlc = can_bytes_to_dlc(l);
        }
    } else {
        mp_raise_ValueError(MP_ERROR_TEXT("unsupported payload: needs float, int, or byte[]"));
    }

    ret = can_send(self->dev, &frame, K_FOREVER, can_tx_callback, NULL);

    if (ret < 0) {
        mp_raise_ValueError(can_error_to_str(-ret));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(can_obj_send_obj, machine_hard_can_send);

#ifdef CONFIG_CANOPEN_LSS
STATIC mp_obj_t lss_fastscan (size_t n_args, const mp_obj_t *args)
{
    machine_hard_can_obj_t *self = args[0];
    mp_obj_t components[5];
    int i;

    for (i = 1; i < n_args; i++) {
        if (!mp_obj_is_small_int(args[i]))
            mp_raise_ValueError(MP_ERROR_TEXT("parameter must be integer"));
        if (i > 1)
            components[i-2] = args[i];
        else
            components[4] = args[i];
    }
    for (; i < 5; i++) {
        components[i-2] = mp_const_none;
    }

    if (lss_fast_scan(self->dev, (void***)&components)) {
        return mp_obj_new_tuple(5, components);
    }

    return mp_obj_new_bool(false);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(can_obj_lss_fastscan_obj, 2, 5, lss_fastscan);
#endif

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
    { MP_ROM_QSTR(MP_QSTR_send),         MP_ROM_PTR(&can_obj_send_obj)    },
    { MP_ROM_QSTR(MP_QSTR_status),       MP_ROM_PTR(&can_obj_status_obj)  },
    { MP_ROM_QSTR(MP_QSTR_errors),       MP_ROM_PTR(&can_obj_errors_obj)  },
    { MP_ROM_QSTR(MP_QSTR_on_message),   MP_ROM_PTR(&can_obj_on_mesg_obj) },
#ifdef CONFIG_CANOPEN_LSS
    { MP_ROM_QSTR(MP_QSTR_lss_fastscan),     MP_ROM_PTR(&can_obj_lss_fastscan_obj) },
    { MP_ROM_QSTR(MP_QSTR_sdo_exp_download), MP_ROM_PTR(&can_obj_sdo_exp_download_obj) },
    { MP_ROM_QSTR(MP_QSTR_sdo_exp_upload),   MP_ROM_PTR(&can_obj_sdo_exp_upload_obj) },
#endif
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
