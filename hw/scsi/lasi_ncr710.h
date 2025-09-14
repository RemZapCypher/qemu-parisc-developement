/*
 * LASI NCR53C710 SCSI Host Adapter
 *
 * Copyright (c) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 *
 * LASI wrapper for NCR53C710 SCSI I/O Processor
 * Based on the NCR53C710 Technical Manual Version 3.2, December 2000
 * and PA-RISC LASI chipset documentation
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

/* LASI-specific constants */
#define LASI_SCSI_RESET         0x000   /* SCSI Reset Register */
#define LASI_SCSI_NCR710_BASE   0x100   /* NCR53C710 registers start here */

/* PA-RISC device identification register offsets */
#define PARISC_DEVICE_ID_OFF    0x00    /* HW type, HVERSION, SVERSION */
#define PARISC_DEVICE_CONFIG_OFF 0x04   /* Configuration data */

/* LASI NCR53C710 state */
typedef struct LasiNCR710State {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    qemu_irq irq;
    uint32_t hw_type;        /* Hardware type (HPHW_*) */
    uint32_t sversion;       /* Software version */
    uint32_t hversion;       /* Hardware version */
    DeviceState *ncr710_dev; /* NCR710 device */
} LasiNCR710State;

/* Create and initialize a LASI NCR710 device */
DeviceState *lasi_ncr710_init(MemoryRegion *addr_space, hwaddr hpa, qemu_irq irq);
void lasi_ncr710_handle_legacy_cmdline(DeviceState *lasi_dev);

#endif