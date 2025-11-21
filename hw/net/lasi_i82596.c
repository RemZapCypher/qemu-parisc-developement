/*
 * QEMU LASI NIC i82596 emulation
 *
 * Copyright (c) 2019 Helge Deller <deller@gmx.de>
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 *
 * On PA-RISC, this is the Network part of LASI chip.
 * See:
 * https://parisc.wiki.kernel.org/images-parisc/7/79/Lasi_ers.pdf
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "system/system.h"
#include "net/eth.h"
#include "hw/net/lasi_82596.h"
#include "hw/net/i82596.h"
#include "trace.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"

#define PA_I82596_RESET         0       /* Offsets relative to LASI-LAN-Addr.*/
#define PA_CPU_PORT_L_ACCESS    4
#define PA_CHANNEL_ATTENTION    8
#define PA_GET_MACADDR          12

#define PORT_BYTEMASK           0x0f

static void lasi_82596_mem_write(void *opaque, hwaddr addr,
                            uint64_t val, unsigned size)
{
    SysBusI82596State *d = opaque;

    fprintf(stderr, "LASI 82596 WRITE: addr=0x%lx size=%u val=0x%lx\n", addr, size, val);
    trace_lasi_82596_mem_writew(addr, val);
    switch (addr) {
    case PA_I82596_RESET:
        i82596_h_reset(&d->state);
        break;
    case PA_CPU_PORT_L_ACCESS: {
        uint16_t wval = val & 0xFFFF;
        if (d->val_index == 0) {
            d->last_val = wval;
            d->val_index = 1;
        } else {
            uint32_t full_val = (wval << 16) | d->last_val;
            full_val &= ~PORT_BYTEMASK;
            uint8_t selector = d->last_val & PORT_BYTEMASK;

            i82596_ioport_writew(&d->state, selector, full_val);
            d->val_index = 0;
        }
        break;
    }
    case PA_CHANNEL_ATTENTION:
        i82596_ioport_writew(&d->state, PORT_CA, val);
        break;
    case PA_GET_MACADDR:
        /*
         * Provided for SeaBIOS only. Write MAC of Network card to addr @val.
         * Needed for the PDC_LAN_STATION_ID_READ PDC call.
         */
        address_space_write(&address_space_memory, val,
                            MEMTXATTRS_UNSPECIFIED, d->state.conf.macaddr.a,
                            ETH_ALEN);
        break;
    }
}

static uint64_t lasi_82596_mem_read(void *opaque, hwaddr addr,
                               unsigned size)
{
    SysBusI82596State *d = opaque;
    uint32_t val;

    if (addr == PA_GET_MACADDR) {
        val = 0xBEEFBABE;
    } else {
        val = i82596_ioport_readw(&d->state, addr);
    }
    fprintf(stderr, "LASI 82596 READ: addr=0x%lx size=%u val=0x%x\n", addr, size, val);
    trace_lasi_82596_mem_readw(addr, val);
    return val;
}

static const MemoryRegionOps lasi_82596_mem_ops = {
    .read = lasi_82596_mem_read,
    .write = lasi_82596_mem_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static NetClientInfo net_lasi_82596_info = {
    .type = NET_CLIENT_DRIVER_NIC,
    .size = sizeof(NICState),
    .can_receive = i82596_can_receive,
    .receive = i82596_receive,
    .receive_iov = i82596_receive_iov,
    .poll = i82596_poll,
    .link_status_changed = i82596_set_link_status,
};

static const VMStateDescription vmstate_lasi_82596 = {
    .name = "i82596",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT(state, SysBusI82596State, 0, vmstate_i82596,
                I82596State),
        VMSTATE_END_OF_LIST()
    }
};

static void lasi_82596_realize(DeviceState *dev, Error **errp)
{
    SysBusI82596State *d = SYSBUS_I82596(dev);
    I82596State *s = &d->state;

    fprintf(stderr, "LASI 82596: realize() called\n");
    
    memory_region_init_io(&s->mmio, OBJECT(d), &lasi_82596_mem_ops, d,
                "lasi_82596-mmio", 0x20);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    i82596_common_init(dev, s, &net_lasi_82596_info);
    
    fprintf(stderr, "LASI 82596: calling i82596_h_reset()\n");
    /* Initialize device state */
    i82596_h_reset(s);
    
    fprintf(stderr, "LASI 82596: realize() complete\n");
}

static void lasi_82596_reset(DeviceState *dev)
{
    SysBusI82596State *d = SYSBUS_I82596(dev);

    i82596_h_reset(&d->state);
}

static void lasi_82596_instance_init(Object *obj)
{
    SysBusI82596State *d = SYSBUS_I82596(obj);
    I82596State *s = &d->state;
    static const MACAddr HP_MAC = {
        .a = { 0x08, 0x00, 0x09, 0xef, 0x34, 0xf6 } };

    s->conf.macaddr = HP_MAC;

    device_add_bootindex_property(obj, &s->conf.bootindex,
                                  "bootindex", "/ethernet-phy@0",
                                  DEVICE(obj));
}

static const Property lasi_82596_properties[] = {
    DEFINE_NIC_PROPERTIES(SysBusI82596State, state.conf),
};

static void lasi_82596_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lasi_82596_realize;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->fw_name = "ethernet";
    device_class_set_legacy_reset(dc, lasi_82596_reset);
    dc->vmsd = &vmstate_lasi_82596;
    dc->user_creatable = false;
    device_class_set_props(dc, lasi_82596_properties);
}

static const TypeInfo lasi_82596_info = {
    .name          = TYPE_LASI_82596,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusI82596State),
    .class_init    = lasi_82596_class_init,
    .instance_init = lasi_82596_instance_init,
};

static void lasi_82596_register_types(void)
{
    type_register_static(&lasi_82596_info);
}

type_init(lasi_82596_register_types)
