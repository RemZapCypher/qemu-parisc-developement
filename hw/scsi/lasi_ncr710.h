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

#ifndef HW_LASI_NCR710_H
#define HW_LASI_NCR710_H

#include "hw/sysbus.h"
#include "qemu/osdep.h"
#include "exec/memattrs.h"
#include "hw/scsi/scsi.h"
#include "hw/scsi/ncr53c710.h"

#define TYPE_LASI_NCR710 "lasi-ncr710"
OBJECT_DECLARE_SIMPLE_TYPE(LasiNCR710State, LASI_NCR710)

/* LASI NCR53C710 state */
struct LasiNCR710State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;              /* Memory region for LASI SCSI interface */
    qemu_irq irq;                   /* IRQ line to LASI */

    DeviceState *ncr710_dev;

    /* PA-RISC device identification */
    uint32_t hw_type;
    uint32_t sversion;
    uint32_t hversion;
};

/* Create and initialize a LASI NCR710 device */
DeviceState *lasi_ncr710_init(MemoryRegion *addr_space, hwaddr hpa, qemu_irq irq);
void lasi_ncr710_handle_legacy_cmdline(DeviceState *lasi_dev);

#endif