#include <stdio.h>
#include <stdint.h>

#include "py/builtin.h"
#include "py/nlr.h"
#include "py/runtime.h"
#include "py/mphal.h"

#include "mcp_can.h"

#include "nrf_gpio.h"
#include "nrfx_gpiote.h"

// mp_sched_schedule() ???

/* usage:
    can = CAN(SPI(0), Pin("P22", mode=Pin.OUT), 125000)

from machine import Pin
from can import Can

def callback(obj):
	print("obj:",obj)

c = Can(38)
c.on_message(callback)

//can = CAN(1, CAN.NORMAL, baudrate=500_000, brs_baudrate=1_000_000, sample_point=80)
//can.setfilter(0, CAN.RANGE, 0, (0xFFF0, 0xFFFF))
//can.send('a'*64, 0xFFFF, fdf=True, brs=True, extframe=True)
//can.recv(0)

*/

#if MICROPY_PY_CAN

#define NUM_OF_CANS 2

typedef struct _can_obj_t {
    mp_obj_base_t base;
    mp_int_t id;

    nrfx_gpiote_pin_t irq_pin;
    nrfx_gpiote_pin_t sck_spi_pin;
    nrfx_gpiote_pin_t miso_spi_pin;
    nrfx_gpiote_pin_t mosi_spi_pin;
    nrfx_gpiote_pin_t cs_spi_pin;

    mp_int_t msg_count;
} can_obj_t;

const mp_obj_type_t can_type;

static can_obj_t can_obj[] = { {{&can_type}, 0, 0, 0, 0, 0, 0, 0}, {{&can_type}, 1, 0, 0, 0, 0, 0, 0} };

void can_init0(void) {
    for (int i = 0; i < NUM_OF_CANS; i++) {
        MP_STATE_PORT(can_irq_handlers)[i] = mp_const_none;
    }
    // Initialize GPIOTE if not done yet.
    if (!nrfx_gpiote_is_init()) {
        nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
    }
}

STATIC void can_common_irq_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    for (int i = 0; i < NUM_OF_CANS; i++) {
        if (pin == can_obj[i].irq_pin) {
            mp_obj_t obj = (mp_obj_t)&can_obj[i];
            mp_obj_t callback = MP_STATE_PORT(can_irq_handlers)[i];
            mp_call_function_1(callback, obj);
        }
    }
}

/*
STATIC mp_obj_t can_evt(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_handler, ARG_trigger, ARG_wake};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_OBJ | MP_ARG_REQUIRED,  {.u_obj = mp_const_none} },
        { MP_QSTR_trigger, MP_ARG_INT,  {.u_int = NRF_GPIOTE_POLARITY_LOTOHI | NRF_GPIOTE_POLARITY_HITOLO} },
        { MP_QSTR_wake,    MP_ARG_BOOL, {.u_bool = false} },
    };
    //mp_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    //mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
	mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args); // function has no self arg because it is not an instance method

    nrfx_gpiote_pin_t pin = NRF_GPIO_PIN_MAP(1,6); // PCA10059 Button

    nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    if (args[ARG_trigger].u_int == NRF_GPIOTE_POLARITY_LOTOHI) {
        config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    } else if (args[ARG_trigger].u_int == NRF_GPIOTE_POLARITY_HITOLO) {
        config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    }
    config.pull = NRF_GPIO_PIN_PULLUP;

	MP_STATE_PORT(can_irq_handler) = mp_const_none;
	if (!nrfx_gpiote_is_init()) {
	    nrfx_gpiote_init(NRFX_GPIOTE_DEFAULT_CONFIG_IRQ_PRIORITY);
	}

    nrfx_err_t err_code = nrfx_gpiote_in_init(pin, &config, can_common_irq_handler);
    if (err_code == NRFX_ERROR_INVALID_STATE) {
        // Re-init if already configured.
        nrfx_gpiote_in_uninit(pin);
        nrfx_gpiote_in_init(pin, &config, can_common_irq_handler);
    }

    MP_STATE_PORT(can_irq_handler) = args[ARG_handler].u_obj;

    nrfx_gpiote_in_event_enable(pin, true);

    // return the irq object
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(can_evt_obj, 1, can_evt);
*/

void internal_can_init(void) {
  retry:
    if(mcp_can_begin(CAN_125KBPS) != CAN_OK)
    {
        nrf_delay_ms(1000);
        goto retry;
    }
}

void internal_can_send() {
	uint8_t buf[8] = {0x02, 0x01, 0x00, 0, 0, 0, 0, 0};
	uint32_t can_id;
	uint8_t len = 0;
	
	mcp_can_send_msg(0x7df, 0, 8, buf);
}

void internal_can_receive() {
	uint8_t buf[8];
	uint32_t can_id;
	uint8_t len;
	if(CAN_MSGAVAIL == mcp_can_check_receive())
	{
		mcp_can_read_msg(&can_id, &len, buf);
	}
}

void can_obj_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    can_obj_t *self = self_in;
    mp_printf(print, "CAN(irq=%lu, sck=%lu, miso=%lu, mosi=%lu, cs=%lu) [%lu messages]", self->irq_pin, self->sck_spi_pin, self->miso_spi_pin, self->mosi_spi_pin, self->cs_spi_pin, self->msg_count);
}

STATIC mp_obj_t can_obj_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    mp_arg_check_num(n_args, n_kw, 1, 1, false);
    mp_int_t pin = mp_obj_get_int(args[0]);

    if (!(1 <= id && id <= 2)) {
        mp_raise_ValueError(MP_ERROR_TEXT("CAN doesn't exist"));
    }

    id -= 1;
    can_obj[id].irq_pin = id;
    return (mp_obj_t)&can_obj[id];
}

STATIC mp_obj_t can_obj_send(mp_obj_t self_in) { // FIXME: send can frame as parameter
    can_obj_t *self = self_in;
    //printf("send\n");
    self->msg_count += 1;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(can_obj_send_obj, can_obj_send);

STATIC mp_obj_t can_obj_recv(mp_obj_t self_in) {
    can_obj_t *self = self_in;
    //printf("recv\n");
    //mp_obj_t mp_obj_new_bytes(const byte *data, size_t len);
    return MP_OBJ_NEW_SMALL_INT(self->msg_count); // FIXME: return can frame if available (only async)
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(can_obj_recv_obj, can_obj_recv);

STATIC mp_obj_t can_obj_mesg(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {ARG_handler, ARG_trigger, ARG_wake};
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_handler, MP_ARG_OBJ | MP_ARG_REQUIRED,  {.u_obj = mp_const_none} },
        { MP_QSTR_trigger, MP_ARG_INT,  {.u_int = NRF_GPIOTE_POLARITY_LOTOHI | NRF_GPIOTE_POLARITY_HITOLO} },
        { MP_QSTR_wake,    MP_ARG_BOOL, {.u_bool = false} },
    };
    can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    //self->irq_pin = NRF_GPIO_PIN_MAP(1,6); // PCA10059 Button

    nrfx_gpiote_in_config_t config = NRFX_GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    if (args[ARG_trigger].u_int == NRF_GPIOTE_POLARITY_LOTOHI) {
        config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
    } else if (args[ARG_trigger].u_int == NRF_GPIOTE_POLARITY_HITOLO) {
        config.sense = NRF_GPIOTE_POLARITY_HITOLO;
    }
    config.pull = NRF_GPIO_PIN_PULLUP;

    nrfx_err_t err_code = nrfx_gpiote_in_init(self->irq_pin, &config, can_common_irq_handler);
    if (err_code == NRFX_ERROR_INVALID_STATE) {
        // Re-init if already configured.
        nrfx_gpiote_in_uninit(self->irq_pin);
        nrfx_gpiote_in_init(self->irq_pin, &config, can_common_irq_handler);
    }

    MP_STATE_PORT(can_irq_handlers)[self->id] = args[ARG_handler].u_obj;

    nrfx_gpiote_in_event_enable(self->irq_pin, true);

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(can_obj_mesg_obj, 1, can_obj_mesg);

STATIC const mp_rom_map_elem_t can_locals_dict_table[] = {
    // instance methods
    //{ MP_ROM_QSTR(MP_QSTR_init),   MP_ROM_PTR(&can_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),       MP_ROM_PTR(&can_obj_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv),       MP_ROM_PTR(&can_obj_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_on_message), MP_ROM_PTR(&can_obj_mesg_obj) },
};

STATIC MP_DEFINE_CONST_DICT(can_locals_dict, can_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    can_type,
    MP_QSTR_Can,
    MP_TYPE_FLAG_NONE,
    make_new, can_obj_make_new,
    print, can_obj_print,
    locals_dict, &can_locals_dict
    );

STATIC const mp_rom_map_elem_t mp_module_can_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_can) },
    { MP_ROM_QSTR(MP_QSTR_Can), MP_ROM_PTR(&can_type) },
};
STATIC MP_DEFINE_CONST_DICT(mp_module_can_globals, mp_module_can_globals_table);

const mp_obj_module_t mp_module_can = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_can_globals,
};

MP_REGISTER_MODULE(MP_QSTR_can, mp_module_can);
MP_REGISTER_ROOT_POINTER(mp_obj_t can_irq_handlers[NUM_OF_CANS]);

#endif // MICROPY_PY_CAN