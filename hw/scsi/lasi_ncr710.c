/*
 * NCR710 SCSI I/O Processor
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
#include "qapi/error.h"
#include "qemu/timer.h"
#include "hw/sysbus.h"
#include "hw/scsi/scsi.h"
#include "hw/scsi/ncr53c710.h"
#include "hw/scsi/lasi_ncr710.h"
#include "trace.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qemu/queue.h"
#include "exec/memop.h"

#define LASI_SCSI_RESET         0x000   /* SCSI Reset Register */
#define LASI_SCSI_NCR710_BASE   0x100   /* NCR53C710 registers start here */

/* PA-RISC hardware type constants */
#define HPHW_FIO    5           /* Fixed I/O module */

/* LASI NCR53C710 sversions */
#define LASI_710_HVERSION   0x3d
#define LASI_700_SVERSION   0x00071
#define LASI_710_SVERSION   0x00082

/* PA-RISC device identification register offsets */
#define PARISC_DEVICE_ID_OFF    0x00    /* HW type, HVERSION, SVERSION */
#define PARISC_DEVICE_CONFIG_OFF 0x04   /* Configuration data */

static void lasi_ncr710_mem_write(void *opaque, hwaddr addr,
                                 uint64_t val, unsigned size)
{
    LasiNCR710State *s = opaque;

    qemu_log("LASI NCR710: Write addr=0x%03x val=0x%08x size=%d\n", (int)addr, (int)val, size);

    /* Note: SCSI Reset (addr 0x000) is handled by NCR710 SCNTL0 register
     * No need for LASI-specific reset handling - let NCR710 handle it */

    /* Forward NCR710 SCSI register writes to the NCR710 device */
    if (s->ncr710_dev) {
        MemoryRegion *ncr710_mem = sysbus_mmio_get_region(SYS_BUS_DEVICE(s->ncr710_dev), 0);
        if (ncr710_mem && addr < memory_region_size(ncr710_mem)) {
            MemOp op = size_memop(size) | MO_BE;  /* Big-endian for PA-RISC */
            memory_region_dispatch_write(ncr710_mem, addr, val, op, MEMTXATTRS_UNSPECIFIED);
            qemu_log("LASI NCR710: Forwarded write to NCR710\n");
            return;
        }
    }

    /* Other writes to identification registers are ignored */
    qemu_log("LASI NCR710: Write to identification register ignored\n");
}

static uint64_t lasi_ncr710_mem_read(void *opaque, hwaddr addr,
                                    unsigned size)
{
    LasiNCR710State *s = LASI_NCR710(opaque);
    uint64_t val = 0;

    /* ALWAYS log all memory reads for debugging firmware discovery */
    qemu_log("LASI NCR710: DISCOVERY READ addr=0x%03x size=%d", (int)addr, size);

    /* Handle PA-RISC device identification registers - ONLY for non-conflicting offsets */
    if (addr == PARISC_DEVICE_ID_OFF) {
        /* Return hardware type and sversion in PA-RISC standard format */
        val = (s->hw_type << 24) | s->sversion;
        qemu_log(" -> Device ID: hw_type=HPHW_FIO(%d), sversion=0x%04x -> 0x%08x\n",
                 s->hw_type, s->sversion, (uint32_t)val);
        return val;
    }

    /* Also implement hversion register at offset 0x08 */
    if (addr == 0x08) {
        val = s->hversion;
        qemu_log(" -> HVersion = 0x%02x\n", (int)val);
        return val;
    }

    /* Implement status register at 0x0C indicating device is present and functional */
    if (addr == 0x0C) {
        val = 0x53434E52;  /* 'SCNR' - identify as SCSI NCR device */
        qemu_log(" -> SCSI Identification = 0x%08x\n", (int)val);
        return val;
    }

    if (s->ncr710_dev) {
        MemoryRegion *ncr710_mem = sysbus_mmio_get_region(SYS_BUS_DEVICE(s->ncr710_dev), 0);
        if (ncr710_mem && addr < memory_region_size(ncr710_mem)) {
            uint64_t read_val = 0;
            MemOp op = size_memop(size) | MO_BE;  /* Big-endian for PA-RISC */
            MemTxResult result = memory_region_dispatch_read(ncr710_mem, addr, &read_val, op, MEMTXATTRS_UNSPECIFIED);
            if (result == MEMTX_OK) {
                qemu_log(" -> NCR710 Register = 0x%x\n", (int)read_val);
                return read_val;
            }
        }
    }

    /* Other registers - return 0 for now */
    val = 0;
    qemu_log(" -> Default = 0x%x\n", (int)val);
    return val;
}

static const MemoryRegionOps lasi_ncr710_mem_ops = {
    .read = lasi_ncr710_mem_read,
    .write = lasi_ncr710_mem_write,
    .endianness = DEVICE_BIG_ENDIAN,  /* PA-RISC is big endian */
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
    SysBusDevice *sysbus_self = SYS_BUS_DEVICE(dev);

    qemu_log("LASI NCR710: Realizing device with endianness conversion support\n");

    s->hw_type = HPHW_FIO;
    s->sversion = LASI_710_SVERSION;
    s->hversion = 0x3d;
    qemu_log("LASI NCR710: Device ID - hw_type=%d (HPHW_FIO), sversion=0x%04x, hversion=0x%02x\n",
             s->hw_type, s->sversion, s->hversion);

    memory_region_init_io(&s->mmio, OBJECT(s), &lasi_ncr710_mem_ops, s,
                         "lasi-ncr710", LASI_SCSI_NCR710_BASE);
    sysbus_init_mmio(sysbus_self, &s->mmio); /* Not sure if this is correct, seems to work */
    sysbus_init_irq(sysbus_self, &s->irq);

    qemu_log("LASI NCR710: Device realized successfully\n");
}

DeviceState *lasi_ncr710_init(MemoryRegion *addr_space, hwaddr hpa, qemu_irq irq)
{
    DeviceState *dev;
    LasiNCR710State *s;
    DeviceState *ncr710_dev;
    SysBusDevice *ncr710_sysbus;

    dev = qdev_new(TYPE_LASI_NCR710);
    s = LASI_NCR710(dev);
    s->irq = irq;
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);
    ncr710_dev = qdev_new(TYPE_SYSBUS_NCR710_SCSI);
    ncr710_sysbus = SYS_BUS_DEVICE(ncr710_dev);
    qdev_realize_and_unref(ncr710_dev, NULL, &error_abort);
    s->ncr710_dev = ncr710_dev;
    memory_region_add_subregion(addr_space, hpa,
                               sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0));
    sysbus_mmio_map(ncr710_sysbus, 0, hpa + NCR710_REG_SIZE);
    sysbus_connect_irq(ncr710_sysbus, 0, irq);
    qemu_log("LASI NCR710: Initialization complete\n");
    return dev;
}

void lasi_ncr710_handle_legacy_cmdline(DeviceState *lasi_dev)
{
    LasiNCR710State *s = LASI_NCR710(lasi_dev);
    if (!s->ncr710_dev) {
        qemu_log("LASI NCR710: No NCR710 device found\n");
        return;
    }

    BusState *scsi_bus = qdev_get_child_bus(s->ncr710_dev, "scsi.0");
    if (scsi_bus) {
        scsi_bus_legacy_handle_cmdline(SCSI_BUS(scsi_bus));
        qemu_log("LASI NCR710: Legacy command line handled successfully\n");
    } else {
        qemu_log("LASI NCR710: Could not find SCSI bus on NCR710 device\n");
    }
}

static void lasi_ncr710_reset(DeviceState *dev)
{
    LasiNCR710State *s = LASI_NCR710(dev);
    qemu_log("LASI NCR710: Device reset called\n");
    if (s->ncr710_dev) {
        qemu_log("LASI NCR710: Resetting NCR710 device\n");
        device_cold_reset(s->ncr710_dev);
    }
    s->hw_type = HPHW_FIO;
    s->sversion = LASI_710_SVERSION;
    s->hversion = LASI_710_HVERSION;
    qemu_log("LASI NCR710: Device reset completed\n");
}

static void lasi_ncr710_instance_init(Object *obj)
{
    qemu_log("LASI NCR710: Instance init\n");
}

static void lasi_ncr710_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = lasi_ncr710_realize;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->fw_name = "scsi";
    device_class_set_legacy_reset(dc, lasi_ncr710_reset);
    dc->vmsd = &vmstate_lasi_ncr710;
    dc->user_creatable = false;
}

static const TypeInfo lasi_ncr710_info = {
    .name          = TYPE_LASI_NCR710,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(LasiNCR710State),
    .class_init    = lasi_ncr710_class_init,
    .instance_init = lasi_ncr710_instance_init,
};

static void lasi_ncr710_register_types(void)
{
    type_register_static(&lasi_ncr710_info);
}

type_init(lasi_ncr710_register_types)