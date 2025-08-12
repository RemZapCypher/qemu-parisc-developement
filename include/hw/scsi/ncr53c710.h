/*
 * NCR710 SCSI Controller
 *
 * Copyright (c) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 *
 * NCR710 SCSI I/O Processor
 * Based on the NCR710 Technical Manual Version 3.2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef HW_SCSI_NCR710_H
#define HW_SCSI_NCR710_H

#include "hw/scsi/scsi.h"
#include "hw/sysbus.h"
#include "hw/pci/pci_device.h"

/* Device identification */
#define TYPE_NCR710_SCSI "ncr710-scsi"
#define TYPE_SYSBUS_NCR710_SCSI "sysbus-ncr710-scsi"

/* SCSI phases */
typedef enum {
    SCSI_PHASE_DATA_OUT = 0,
    SCSI_PHASE_DATA_IN = 1,
    SCSI_PHASE_COMMAND = 2,
    SCSI_PHASE_STATUS = 3,
    SCSI_PHASE_MSG_OUT = 6,
    SCSI_PHASE_MSG_IN = 7,
} ScsiPhase;

/* SCRIPTS instruction formats */
typedef struct {
    uint32_t count:24;
    uint32_t opcode:8;
} ScriptsMoveInst;

typedef struct {
    uint32_t count:24;
    uint32_t phase:3;
    uint32_t opcode:5;
} ScriptsIOInst;

typedef struct {
    uint32_t addr_mode:1;
    uint32_t reserved:23;
    uint32_t phase:3;
    uint32_t opcode:5;
} ScriptsTransferInst;

/* Register access macros */
#define NCR710_REG_SIZE 0x100

/* Debugging and tracing */
#define NCR710_DPRINTF(fmt, ...) \
    do { \
        if (NCR710_DEBUG) { \
            fprintf(stderr, "ncr710: " fmt, ## __VA_ARGS__); \
        } \
    } while (0)

#ifndef NCR710_DEBUG
#define NCR710_DEBUG 0
#endif

/* Function prototypes for external interface */
DeviceState *ncr53c710_init(MemoryRegion *address_space, hwaddr addr, qemu_irq irq);
DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq);
void ncr710_handle_legacy_cmdline(DeviceState *ncr_dev);

/* Forward declarations for device types */
typedef struct NCR710State NCR710State;
typedef struct SysBusNCR710State SysBusNCR710State;

OBJECT_DECLARE_SIMPLE_TYPE(NCR710State, NCR710_SCSI)
OBJECT_DECLARE_SIMPLE_TYPE(SysBusNCR710State, SYSBUS_NCR710_SCSI)

#endif /* HW_SCSI_NCR710_H */
