/*
 * Copyright (c) 2023 Marc Delling
 *
 * SPDX-License-Identifier: Proprietary
 */

#include "mcp_can.h"

K_SEM_DEFINE(lss_response_sem, 1, 1);
uint8_t lss_response;
extern struct device *can_dev;

/*
#define MAX_CANOPEN_DEVICES 7

typedef struct
{
    uint8_t id;
    uint32_t vendor;
    uint32_t product;
    uint32_t revision;
    uint32_t serial;
    union
    {
        uint8_t lss: 1;
        uint8_t lss_store: 1;
        uint8_t invalid: 1; 
    } flags;
} canopen_device_info;

K_MUTEX_DEFINE(devices_mutex);
canopen_device_info devices[MAX_CANOPEN_DEVICES] = { 0 };
*/

/* essential
struct can_frame {
	uint32_t id  : 29;
	uint8_t res0 : 3;
	uint8_t dlc;
	uint8_t flags;
	uint16_t timestamp;
	uint8_t data[8];
};
*/


static bool lss_identify_remote_slave (uint32_t vendor, uint32_t product, uint32_t revision_low, uint32_t revision_high, uint32_t serial_low, uint32_t serial_high)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0x7e5;
    frame.dlc = 8;

    frame.data[0] = 0x46;
    UNALIGNED_PUT(sys_cpu_to_be32(vendor), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x47;
    UNALIGNED_PUT(sys_cpu_to_be32(product), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x48;
    UNALIGNED_PUT(sys_cpu_to_be32(revision_low), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x49;
    UNALIGNED_PUT(sys_cpu_to_be32(revision_high), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x4a;
    UNALIGNED_PUT(sys_cpu_to_be32(serial_low), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x4b;
    UNALIGNED_PUT(sys_cpu_to_be32(serial_high), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    if ( (k_sem_take(&lss_response_sem, K_MSEC(25)) == 0) && (lss_response == 0x4f) )
    {
        return true;
    }

    printk("lss_configure_node failed: %d\n", lss_response);

    return false;
}

void lss_switch_mode_global (bool config)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0x7e5;
    frame.dlc = 8;
    frame.data[0] = 0x04;
    frame.data[1] = config ? 0x01 : 0x00;
	// frame.timestamp = k_uptime_get();

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);
}

bool lss_switch_mode_selective (uint32_t vendor, uint32_t product, uint32_t revision, uint32_t serial)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0x7e5;
    frame.dlc = 8;

    frame.data[0] = 0x40;
    UNALIGNED_PUT(sys_cpu_to_be32(vendor), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x41;
    UNALIGNED_PUT(sys_cpu_to_be32(product), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x42;
    UNALIGNED_PUT(sys_cpu_to_be32(revision), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    frame.data[0] = 0x43;
    UNALIGNED_PUT(sys_cpu_to_be32(serial), (uint32_t *)&frame.data[1]);
    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    if ( (k_sem_take(&lss_response_sem, K_MSEC(25)) == 0) && (lss_response == 0x44) )
    {
        return true;
    }

    printk("lss_configure_node failed: %d\n", lss_response);

    return false;
}

bool lss_configure_node (uint8_t canid)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0x7e5;
    frame.dlc = 8;
    frame.data[0] = 0x11;
    frame.data[1] = canid;

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    if ( (k_sem_take(&lss_response_sem, K_MSEC(25)) != 0) && (lss_response == 0x00) )
    {
        return true;
    }

    printk("lss_configure_node failed: %d\n", lss_response);

    return false;
}

enum nmt_state {
    operational = 0x01,
    stop = 0x02,
    pre_operational = 0x80, // heartbeat {700 + canid} + 7f means node in pre-operational
    reset_node = 0x81,
    reset_communcation = 0x82
};

void nmt_set_state (enum nmt_state state)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0;
    frame.dlc = 2;
    frame.data[0] = state;

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);
}

void nmt_reset_node (uint8_t nodeid)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0;
    frame.dlc = 2;
    frame.data[0] = reset_node;
    frame.data[1] = nodeid; // 00 = all, 01-7f = node

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);
}

void sdo_download_expedited (uint8_t nodeid, uint16_t index, uint8_t subindex, uint32_t payload) // write
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 600 + nodeid;
    frame.dlc = 8;
    frame.data[0] = 0x40;
    UNALIGNED_PUT(sys_cpu_to_be16(index), (uint16_t *)&frame.data[1]);
    frame.data[3] = subindex;
    UNALIGNED_PUT(sys_cpu_to_be32(payload), (uint32_t *)&frame.data[4]);

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);
}

void sdo_upload_expedited (uint8_t nodeid, uint16_t index, uint8_t subindex) // read
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 600 + nodeid;
    frame.dlc = 8;
    frame.data[0] = 0x40;
    UNALIGNED_PUT(sys_cpu_to_be16(index), (uint16_t *)&frame.data[1]);
    frame.data[3] = subindex;

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);
}

#define LSS_UNSTORE 0xff
void lss_store (uint8_t nodeid)
{
    struct can_frame frame;

    memset(&frame, 0, sizeof(frame));

    frame.id = 0x7e5;
    frame.dlc = 8;
    frame.data[0] = 0x17;
	frame.data[1] = nodeid;

    can_send(can_dev, &frame, K_MSEC(25), NULL, NULL);

    if ( (k_sem_take(&lss_response_sem, K_MSEC(25)) == 0) && (lss_response == 0x17) )
    {
        printk("lss_store: %d\n", nodeid); /* expect lss_response == 0x17 and additional byte 0x00 (success) or 0x02 (save err) */
    }

    nmt_reset_node(nodeid);
}

/*
bool inquire_tst (uint32_t vendor, uint32_t product, uint32_t rev_l, uint32_t rev_h, uint32_t ser_l, uint32_t ser_h)
{
    const uint32_t test_ser_1 = 23, test_ser_2 = 42, test_ser_3 = 1147483647;
    const uint32_t test_rev_1 = 12523, test_rev_2 = 548;

    if ( ( test_rev_1 <= rev_h && test_rev_1 >= rev_l ) ||
         ( test_rev_2 <= rev_h && test_rev_2 >= rev_l ) ||
         ( test_ser_1 <= ser_h && test_ser_1 >= ser_l ) ||
         ( test_ser_2 <= ser_h && test_ser_2 >= ser_l ) ||
         ( test_ser_3 <= ser_h && test_ser_3 >= ser_l ) )
    {
        return true;
    } else {
        return false;
    }
}

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

int main()
{
    printf("start.\n");
    lss_scan(0,0,548,548,0,0xffffffff);
    printf("end.\n");
    return 0;
}
*/


//sdo_upload_expedited(x, 0x6400, 0x01); // zirk-e
//sdo_upload_expedited(x, 0x6130, 0x01); // wire-port
//sdo_upload_expedited(x, 0x6130, 0x02); // wire-sens

//lss_scan(1171, 787200, 65537, 65537, 0, 0xffffffff); // zirk-e
//lss_scan(1171, 918272, 65542, 65542, 0, 0xffffffff); // wire-port
//lss_scan(1171, 794958, 0, 0xffffffff, 0, 0xffffffff); // wire-sens

void lss_scan (uint32_t vendor, uint32_t product, uint32_t revision_low, uint32_t revision_high, uint32_t serial_low, uint32_t serial_high)
{
    static int it = 0;
    bool result = lss_identify_remote_slave (vendor, product, revision_low, revision_high, serial_low, serial_high);

    printk("test ser_l:%u ser_h:%u result:%d it:%d\n", serial_low, serial_high, result, ++it);

    if (result)
    {
        uint32_t half;
        if (revision_low == revision_high)
        {
            if (serial_low == serial_high)
            {
                printk("found rev:%u ser:%u\n", revision_low, serial_low);
                return;
            }
            half = (serial_low + serial_high) >> 1;
            lss_scan (vendor, product, revision_low, revision_high, serial_low, half);
            lss_scan (vendor, product, revision_low, revision_high, half+1, serial_high);
        }
        else
        {
            half = (revision_low + revision_high) >> 1;
            lss_scan (vendor, product, revision_low, half, serial_low, serial_high);
            lss_scan (vendor, product, half+1, revision_high, serial_low, serial_high);
        }
    }
}