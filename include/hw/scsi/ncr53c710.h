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

#define NCR710_DPRINTF(fmt, ...) \
    do { \
        if (NCR710_DEBUG) { \
            fprintf(stderr, "ncr710: " fmt, ## __VA_ARGS__); \
        } \
    } while (0)

#ifndef NCR710_DEBUG
#define NCR710_DEBUG 0
#endif

/* Constants for FIFO sizes */
#define NCR710_DMA_FIFO_SIZE    64
#define NCR710_SCSI_FIFO_SIZE   8

/* SCSI phases */
typedef enum {
    SCSI_PHASE_DATA_OUT = 0,
    SCSI_PHASE_DATA_IN  = 1,
    SCSI_PHASE_COMMAND  = 2,
    SCSI_PHASE_STATUS   = 3,
    SCSI_PHASE_MSG_OUT  = 6,
    SCSI_PHASE_MSG_IN   = 7,
} NCR710_SCSI_Phase;

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

/* DMA FIFO structure */
typedef struct {
    uint8_t data[NCR710_DMA_FIFO_SIZE];
    uint8_t parity[NCR710_DMA_FIFO_SIZE];
    int head;       /* Write pointer */
    int tail;       /* Read pointer */
    int count;      /* Number of valid entries */
} NCR710_DMA_FIFO;

/* SCSI FIFO structure */
typedef struct {
    uint8_t data[NCR710_SCSI_FIFO_SIZE];
    uint8_t parity[NCR710_SCSI_FIFO_SIZE];
    int count;      /* Number of valid bytes */
} NCR710_SCSI_FIFO;

/* Type aliases for compatibility */
typedef NCR710_DMA_FIFO NCR710_DMA_FIFO;
typedef NCR710_SCSI_FIFO NCR710_SCSI_FIFO;

/* Request structure for SCSI command tracking */
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
    uint32_t pc;        /* Current program counter (DSP) */
    uint32_t saved_pc;  /* Saved PC for calls (TEMP) */
    uint8_t phase;      /* Current SCSI phase */
    bool connected;     /* Connected to SCSI bus */
    bool initiator;     /* True if initiator, false if target */
} NCR710_SCRIPTS_Context;

/* Forward declarations for device types */
typedef struct NCR710State NCR710State;
typedef struct SysBusNCR710State SysBusNCR710State;

/* Device state structure definitions */
struct NCR710State {
    DeviceState parent_obj;

    /* Memory regions */
    MemoryRegion mmio;
    MemoryRegion ram;

    /* SCSI bus */
    SCSIBus bus;

    /* SCSI Core Registers (0x00-0x0F) */
    uint8_t scntl0;             /* 0x00 SCSI Control Zero */
    uint8_t scntl1;             /* 0x01 SCSI Control One */
    uint8_t sdid;               /* 0x02 SCSI Destination ID */
    uint8_t sien;               /* 0x03 SCSI Interrupt Enable */
    uint8_t scid;               /* 0x04 SCSI Chip ID */
    uint8_t sxfer;              /* 0x05 SCSI Transfer */
    uint8_t sodl;               /* 0x06 SCSI Output Data Latch */
    uint8_t socl;               /* 0x07 SCSI Output Control Latch */
    uint8_t sfbr;               /* 0x08 SCSI First Byte Received */
    uint8_t sidl;               /* 0x09 SCSI Input Data Latch */
    uint8_t sbdl;               /* 0x0A SCSI Bus Data Lines */
    uint8_t sbcl;               /* 0x0B SCSI Bus Control Lines */

    /* DMA Core and Status Registers */
    uint8_t dstat;              /* 0x0C DMA Status */
    uint8_t sstat0;             /* 0x0D SCSI Status 0 */
    uint8_t sstat1;             /* 0x0E SCSI Status 1 */
    uint8_t sstat2;             /* 0x0F SCSI Status 2 */

    /* DMA Registers */
    uint32_t dsa;               /* 0x10 Data Structure Address */
    uint8_t istat;              /* 0x21 Interrupt Status */
    uint32_t dbc;               /* 0x24 DMA Byte Counter */
    uint8_t dcmd;               /* 0x27 DMA Command */
    uint32_t dnad;              /* 0x28 DMA Next Address for Data */
    uint32_t dsp;               /* 0x2C DMA SCRIPTS Pointer */
    uint32_t dsps;              /* 0x30 DMA SCRIPTS Pointer Save */
    uint32_t scratch;           /* 0x34 Scratch Registers */
    uint8_t dmode;              /* 0x38 DMA Mode */
    uint8_t dien;               /* 0x39 DMA Interrupt Enable */
    uint8_t dwt;                /* 0x3A DMA Watchdog Timer */
    uint8_t dcntl;              /* 0x3B DMA Control */
    uint32_t adder;             /* 0x3C Adder Sum Output */

    /* Additional test registers */
    uint8_t ctest0, ctest1, ctest2, ctest3, ctest4, ctest5, ctest6, ctest7;
    uint8_t dfifo;              /* DMA FIFO */
    uint8_t ctest8;             /* Chip Test 8 */
    uint8_t lcrc;               /* Longitudinal Parity */
    uint32_t temp;              /* Temporary register */
    uint32_t sbc;               /* SCSI Byte Count - for compatibility */

    /* Internal FIFOs */
    NCR710_DMA_FIFO dma_fifo;
    NCR710_SCSI_FIFO scsi_fifo;

    /* SCRIPTS execution context */
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
    bool tolerant_enabled;  /* LSI Logic TolerANT Active Negation */
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

/* Function prototypes for external interface */
DeviceState *ncr53c710_init(MemoryRegion *address_space, hwaddr addr, qemu_irq irq);
DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq);
bool ncr710_is_700_mode(NCR710State *s);


#endif /* HW_SCSI_NCR710_H */