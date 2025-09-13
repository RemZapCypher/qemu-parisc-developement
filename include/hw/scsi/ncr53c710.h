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
#include "qemu/timer.h"
#include "qemu/queue.h"

#define TYPE_NCR710_SCSI "ncr710-scsi"
#define TYPE_SYSBUS_NCR710_SCSI "sysbus-ncr710-scsi"

#define NCR710_REG_SIZE 0x100

#ifndef NCR710_DEBUG
#define NCR710_DEBUG 1
#endif

#define NCR710_DPRINTF(fmt, ...) \
    do { \
        if (NCR710_DEBUG) { \
            fprintf(stderr,"ncr710: " fmt, ## __VA_ARGS__); \
        } \
    } while (0)


typedef struct {
    uint8_t data[NCR710_DMA_FIFO_SIZE];
    uint8_t parity[NCR710_DMA_FIFO_SIZE];
    int head;       /* Write pointer */
    int tail;       /* Read pointer */
    int count;      /* Number of valid entries */
} NCR710_DMA_FIFO;

typedef struct {
    uint8_t data[NCR710_SCSI_FIFO_SIZE];
    uint8_t parity[NCR710_SCSI_FIFO_SIZE];
    int count;
} NCR710_SCSI_FIFO;

typedef NCR710_DMA_FIFO NCR710_DMA_FIFO;
typedef NCR710_SCSI_FIFO NCR710_SCSI_FIFO;

typedef struct NCR710Request {
    SCSIRequest *req;
    uint32_t tag;
    uint32_t dma_len;
    uint8_t *dma_buf;
    uint32_t pending;
    int out;
    QTAILQ_ENTRY(NCR710Request) next;
} NCR710Request;

/* SCRIPTS execution context */
typedef struct {
    bool running;
    uint32_t pc;
    uint32_t saved_pc;
    uint8_t phase;
    bool connected;
    bool initiator;
} NCR710_SCRIPTS_Context;

typedef struct NCR710State NCR710State;
typedef struct SysBusNCR710State SysBusNCR710State;

struct NCR710State {
    DeviceState parent_obj;

    MemoryRegion mmio;
    MemoryRegion ram;

    SCSIBus bus;

    /* SCSI Core Registers (0x00-0x0F) */
    uint8_t scntl0;
    uint8_t scntl1;
    uint8_t sdid;
    uint8_t sien;
    uint8_t scid;
    uint8_t sxfer;
    uint8_t sodl;
    uint8_t socl;
    uint8_t sfbr;
    uint8_t sidl;
    uint8_t sbdl;
    uint8_t sbcl;

    /* DMA Core and Status Registers */
    uint8_t dstat;
    uint8_t sstat0;
    uint8_t sstat1;
    uint8_t sstat2;

    /* DMA Registers */
    uint32_t dsa;
    uint8_t istat;
    uint32_t dbc;
    uint8_t dcmd;
    uint32_t dnad;
    uint32_t dsp;
    uint32_t dsps;
    uint32_t scratch;
    uint8_t dmode;
    uint8_t dien;
    uint8_t dwt;
    uint8_t dcntl;
    uint32_t adder;

    uint8_t ctest0, ctest1, ctest2, ctest3, ctest4, ctest5, ctest6, ctest7;
    uint8_t dfifo;
    uint8_t ctest8;
    uint8_t lcrc;
    uint32_t temp;
    uint32_t sbc;

    /* Internal FIFOs */
    NCR710_DMA_FIFO dma_fifo;
    NCR710_SCSI_FIFO scsi_fifo;
    NCR710_SCRIPTS_Context scripts;

    /* SCSI command management */
    QTAILQ_HEAD(, NCR710Request) queue;
    NCR710Request *current;
    uint32_t select_tag;
    int current_lun;
    int command_complete;
    int waiting;            /* 0=not waiting, 1=wait reselect, 2=DMA processing, 3=DMA in progress */
    int carry;              /* Carry flag for SCRIPTS arithmetic */
    int status;             /* Last command status */
    int msg_action;         /* Action after MSG IN phase */
    int msg_len;            /* Current message length */
    uint8_t msg[8];         /* Message buffer */

    /* Current SCSI operation state */
    SCSIRequest *current_req;
    uint8_t *current_dma_buf;
    uint32_t current_dma_len;
    uint8_t current_scsi_phase;
    bool selection_timeout_enabled;
    uint8_t script_active;

    /* Timers */
    QEMUTimer *selection_timer;
    QEMUTimer *watchdog_timer;

    /* IRQ line */
    qemu_irq irq;

    /* Host memory access */
    AddressSpace *as;

    /* Bus mode and endianness */
    bool big_endian;
    bool bus_mode_2;    /* true = 68040 mode, false = 68030 mode */

    /* Chip features */
    bool tolerant_enabled;
    bool differential_mode;
    bool cache_line_burst;
    uint8_t burst_length;   /* 1, 2, 4, 8 longwords */
    bool compatibility_mode;
};

struct SysBusNCR710State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t it_shift;
    NCR710State ncr710;
};

OBJECT_DECLARE_SIMPLE_TYPE(NCR710State, NCR710_SCSI)
OBJECT_DECLARE_SIMPLE_TYPE(SysBusNCR710State, SYSBUS_NCR710_SCSI)

DeviceState *ncr53c710_init(MemoryRegion *address_space, hwaddr addr, qemu_irq irq);
DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq);
bool ncr710_is_700_mode(NCR710State *s);


#endif /* HW_SCSI_NCR710_H */