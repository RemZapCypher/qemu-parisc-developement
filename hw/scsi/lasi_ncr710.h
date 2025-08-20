/*
 * QEMU LASI NCR53C710 SCSI controller
 *
 * Copyright (c) 2025
 * This work is licensed under the GNU GPL license version 2 or later.
 */

#ifndef HW_LASI_NCR710_H
#define HW_LASI_NCR710_H

#include "hw/sysbus.h"
#include "qemu/osdep.h"
#include "exec/memattrs.h"
#include "hw/scsi/scsi.h"

#define TYPE_LASI_NCR710 "lasi-ncr710"
OBJECT_DECLARE_SIMPLE_TYPE(LasiNCR710State, LASI_NCR710)

/* LASI NCR53C710 state */
struct LasiNCR710State {
    SysBusDevice parent_obj;

    MemoryRegion mmio;              /* Memory region for LASI SCSI interface */
    qemu_irq irq;                   /* IRQ line to LASI */

    /* NCR710 child device - use pointer approach */
    DeviceState *ncr710_dev;

    /* PA-RISC device identification */
    uint32_t hw_type;
    uint32_t sversion;
    uint32_t hversion;
};

/* Create and initialize a LASI NCR710 device */
DeviceState *lasi_ncr710_init(MemoryRegion *addr_space, hwaddr hpa, qemu_irq irq);

/* Handle legacy SCSI command line for LASI NCR710 */
void lasi_ncr710_handle_legacy_cmdline(DeviceState *lasi_dev);

#endif
