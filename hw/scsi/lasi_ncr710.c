/*
 * LASI Wrapper for NCR710 SCSI I/O Processor
 *
 * Copyright (c) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 *
 * NCR710 SCSI I/O Processor implementation
 * Based on the NCR53C710 Technical Manual Version 3.2, December 2000
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "hw/scsi/lasi_ncr710.h"
#include "hw/scsi/ncr53c710.h"
#include "hw/sysbus.h"
#include "qemu/log.h"
#include "trace.h"
#include "system/blockdev.h"
#include "migration/vmstate.h"
#include "qapi/error.h"

#define HPHW_FIO    5           /* Fixed I/O module */
#define LASI_710_SVERSION    0x00082
#define SCNR                 0x53434E52
#define LASI_710_HVERSION       0x3D

/* Container helper to get LasiNCR710State from SCSIBus */
static inline LasiNCR710State *lasi_ncr710_from_scsi_bus(SCSIBus *bus)
{
    return container_of(bus, LasiNCR710State, bus);
}

/* SCSI callback functions - migrated from NCR710 core to LASI wrapper */
static void lasi_ncr710_request_cancelled(SCSIRequest *req)
{
    trace_lasi_ncr710_request_cancelled(req);

    req->hba_private = NULL;
    scsi_req_unref(req);
}

static void lasi_ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    /* Decode SCSI status for better debugging */
    const char *status_name = "UNKNOWN";
    switch (req->status) {
        case 0x00: status_name = "GOOD"; break;
        case 0x02: status_name = "CHECK_CONDITION"; break;
        case 0x04: status_name = "CONDITION_MET"; break;
        case 0x08: status_name = "BUSY"; break;
        case 0x10: status_name = "INTERMEDIATE"; break;
        case 0x14: status_name = "INTERMEDIATE_CONDITION_MET"; break;
        case 0x18: status_name = "RESERVATION_CONFLICT"; break;
        case 0x22: status_name = "COMMAND_TERMINATED"; break;
        case 0x28: status_name = "TASK_SET_FULL"; break;
        default: break;
    }

    trace_lasi_ncr710_command_complete(req->status, status_name, resid);
}

static void lasi_ncr710_transfer_data(SCSIRequest *req, uint32_t len)
{
    LasiNCR710State *lasi_s = lasi_ncr710_from_scsi_bus(req->bus);
    NCR710State *s = &lasi_s->ncr710;

    trace_lasi_ncr710_transfer_data(len);

    assert(req->hba_private);
    if (req->hba_private != s->current) {
        /* Handle queued request logic */
        return;
    }

    /* Handle the data transfer */
    s->command_complete = 1;
    if (s->current) {
        s->current->dma_len = len;
        /* Trigger DMA operation or script continuation */
        /* TODO: ncr710_do_dma(s, out); */
    }
}

static const struct SCSIBusInfo lasi_ncr710_scsi_info = {
    .tcq = true,
    .max_target = 8,
    .max_lun = 0,  /* LUN support buggy, eh? */

    .transfer_data = lasi_ncr710_transfer_data,
    .complete = lasi_ncr710_command_complete,
    .cancel = lasi_ncr710_request_cancelled,
};

static uint64_t lasi_ncr710_reg_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    LasiNCR710State *s = LASI_NCR710(opaque);
    uint64_t val = 0;

    trace_lasi_ncr710_reg_read(addr, 0, size);

    if (addr == 0x00) {  /* Device ID */
        val = (HPHW_FIO << 24) | LASI_710_SVERSION;
        trace_lasi_ncr710_reg_read_id(HPHW_FIO, LASI_710_SVERSION, val);
        return val;
    }

    if (addr == 0x08) {  /* HVersion */
        val = LASI_710_HVERSION;
        trace_lasi_ncr710_reg_read_hversion(val);
        return val;
    }

    if (addr == 0x0C) {  /* SCSI Identification */
        val = SCNR;
        trace_lasi_ncr710_reg_read_scsi_id(val);
        return val;
    }

    /* Forward to NCR710 core register read */
    if (addr >= 0x100) {
        val = ncr710_reg_read(&s->ncr710, addr - 0x100, size);
        trace_lasi_ncr710_reg_forward_read(addr, val);
    } else {
        val = 0;
        trace_lasi_ncr710_reg_read(addr, val, size);
    }
    return val;
}

static void lasi_ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    LasiNCR710State *s = LASI_NCR710(opaque);
    uint8_t val8 = val & 0xff;

    trace_lasi_ncr710_reg_write(addr, val8, size);
    if (addr <= 0x0F) {
        return;
    }
    /* Forward to NCR710 core register write */
    if (addr >= 0x100) {
        ncr710_reg_write(&s->ncr710, addr - 0x100, val, size);
        trace_lasi_ncr710_reg_forward_write(addr, val8);
    } else {
        trace_lasi_ncr710_reg_write(addr, val8, size);
    }
}

static const MemoryRegionOps lasi_ncr710_mmio_ops = {
    .read = lasi_ncr710_reg_read,
    .write = lasi_ncr710_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_lasi_ncr710 = {
    .name = "lasi-ncr710",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

static void lasi_ncr710_realize(DeviceState *dev, Error **errp)
{
    LasiNCR710State *s = LASI_NCR710(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    trace_lasi_ncr710_device_realize();

    memset(&s->ncr710, 0, sizeof(s->ncr710));

    /* Minimal set up for NCR710 default register values */
    s->ncr710.scntl0 = 0xc0;
    s->ncr710.scid = 0x80;
    s->ncr710.dstat = NCR710_DSTAT_DFE;
    s->ncr710.dien = 0x04;
    s->ncr710.ctest2 = NCR710_CTEST2_DACK;

    /* Initialize SCSI bus */
    scsi_bus_init(&s->bus, sizeof(s->bus), dev, &lasi_ncr710_scsi_info);

    /* Initialize memory region */
    memory_region_init_io(&s->mmio, OBJECT(dev), &lasi_ncr710_mmio_ops, s, "lasi-ncr710", 0x200);
    sysbus_init_mmio(sbd, &s->mmio);
    qdev_init_gpio_out(dev, &s->irq, 1);
}

void lasi_ncr710_handle_legacy_cmdline(DeviceState *lasi_dev)
{
    LasiNCR710State *s = LASI_NCR710(lasi_dev);
    SCSIBus *bus = &s->bus;
    int found_drives = 0;

    if (!bus) {
        return;
    }

    for (int unit = 0; unit <= 7; unit++) {
        DriveInfo *dinfo = drive_get(IF_SCSI, bus->busnr, unit);
        if (dinfo) {
            trace_lasi_ncr710_legacy_drive_found(bus->busnr, unit);
            found_drives++;
        }
    }

    trace_lasi_ncr710_handle_legacy_cmdline(bus->busnr, found_drives);

    scsi_bus_legacy_handle_cmdline(bus);
    BusChild *kid;
    QTAILQ_FOREACH(kid, &bus->qbus.children, sibling) {
        trace_lasi_ncr710_scsi_device_created(object_get_typename(OBJECT(kid->child)));
    }
}

DeviceState *lasi_ncr710_init(MemoryRegion *addr_space, hwaddr hpa, qemu_irq irq)
{
    DeviceState *dev;
    LasiNCR710State *s;

    dev = qdev_new(TYPE_LASI_NCR710);
    s = LASI_NCR710(dev);
    s->irq = irq;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    memory_region_add_subregion(addr_space, hpa,
                               sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));
    return dev;
}

static void lasi_ncr710_reset(DeviceState *dev)
{
    LasiNCR710State *s = LASI_NCR710(dev);

    trace_lasi_ncr710_device_reset();

    memset(&s->ncr710, 0, sizeof(s->ncr710));

    s->ncr710.scntl0 = 0xc0;
    s->ncr710.scid = 0x80;
    s->ncr710.dstat = NCR710_DSTAT_DFE;
    s->ncr710.dien = 0x04;
    s->ncr710.ctest2 = NCR710_CTEST2_DACK;
}

static void lasi_ncr710_instance_init(Object *obj)
{
    LasiNCR710State *s = LASI_NCR710(obj);

    s->hw_type = HPHW_FIO;
    s->sversion = LASI_710_SVERSION;
    s->hversion = LASI_710_HVERSION;

    memset(&s->ncr710, 0, sizeof(s->ncr710));
}

static void lasi_ncr710_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lasi_ncr710_realize;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->fw_name = "scsi";
    dc->desc = "HP-PARISC LASI NCR710 SCSI adapter";
    device_class_set_legacy_reset(dc, lasi_ncr710_reset);
    dc->vmsd = &vmstate_lasi_ncr710;
    dc->user_creatable = false;
}

static const TypeInfo lasi_ncr710_info = {
    .name          = TYPE_LASI_NCR710,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LasiNCR710State),
    .instance_init = lasi_ncr710_instance_init,
    .class_init    = lasi_ncr710_class_init,
};

static void lasi_ncr710_register_types(void)
{
    type_register_static(&lasi_ncr710_info);
}

type_init(lasi_ncr710_register_types)