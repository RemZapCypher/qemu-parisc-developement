/*
 * NCR53C710 SCSI I/O Processor
 *
 * Copyright (c) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 *
 * NCR53C710 SCSI I/O Processor implementation
 * Based on the NCR53C710 Technical Manual Version 3.2, December 2000
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef HW_NCR53C710_H
#define HW_NCR53C710_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/scsi/scsi.h"
#include "qemu/fifo8.h"
#include "qom/object.h"
#include "exec/memory.h"
#include "hw/irq.h"
#include "qemu/timer.h"

#define TYPE_NCR710_SCSI "ncr710-scsi"
#define TYPE_SYSBUS_NCR710_SCSI "sysbus-ncr710-scsi"

#define SYSBUS_NCR710_SCSI(obj) \
    OBJECT_CHECK(SysBusNCR710State, (obj), TYPE_SYSBUS_NCR710_SCSI)

#define ENABLE_DEBUG 0
#if ENABLE_DEBUG
#define DBG(x)          x
#define NCR710_DPRINTF(fmt, ...) \
    fprintf(stderr, "QEMU: " fmt, ## __VA_ARGS__)
#define BADF(fmt, ...) \
    fprintf(stderr, "QEMU: error: " fmt, ## __VA_ARGS__)
#else
#define DBG(x)          do { } while (0)
#define NCR710_DPRINTF(fmt, ...) do { } while (0)
#define BADF(fmt, ...) do { } while (0)
#endif

/* NCR710 - Little Endian register Ordering */
#define NCR710_SCNTL0_REG       0x00    /* SCSI Control Zero */
#define NCR710_SCNTL1_REG       0x01    /* SCSI Control One */
#define NCR710_SDID_REG         0x02    /* SCSI Destination ID */
#define NCR710_SIEN_REG         0x03    /* SCSI Interrupt Enable */
#define NCR710_SCID_REG         0x04    /* SCSI Chip ID */
#define NCR710_SXFER_REG        0x05    /* SCSI Transfer */
#define NCR710_SODL_REG         0x06    /* SCSI Output Data Latch */
#define NCR710_SOCL_REG         0x07    /* SCSI Output Control Latch */
#define NCR710_SFBR_REG         0x08    /* SCSI First Byte Received */
#define NCR710_SIDL_REG         0x09    /* SCSI Input Data Latch */
#define NCR710_SBDL_REG         0x0A    /* SCSI Bus Data Lines */
#define NCR710_SBCL_REG         0x0B    /* SCSI Bus Control Lines */
#define NCR710_DSTAT_REG        0x0C    /* DMA Status */
#define NCR710_SSTAT0_REG       0x0D    /* SCSI Status Zero */
#define NCR710_SSTAT1_REG       0x0E    /* SCSI Status One */
#define NCR710_SSTAT2_REG       0x0F    /* SCSI Status Two */
#define NCR710_DSA_REG          0x10    /* Data Structure Address */
#define NCR710_CTEST0_REG       0x14    /* Chip Test Zero */
#define NCR710_CTEST1_REG       0x15    /* Chip Test One */
#define NCR710_CTEST2_REG       0x16    /* Chip Test Two */
#define NCR710_CTEST3_REG       0x17    /* Chip Test Three */
#define NCR710_CTEST4_REG       0x18    /* Chip Test Four */
#define NCR710_CTEST5_REG       0x19    /* Chip Test Five */
#define NCR710_CTEST6_REG       0x1A    /* Chip Test Six */
#define NCR710_CTEST7_REG       0x1B    /* Chip Test Seven */
#define NCR710_TEMP_REG         0x1C    /* Temporary Stack */
#define NCR710_DFIFO_REG        0x20    /* DMA FIFO */
#define NCR710_ISTAT_REG        0x21    /* Interrupt Status */
#define NCR710_CTEST8_REG       0x22    /* Chip Test Eight */
#define NCR710_LCRC_REG         0x23    /* Longitudinal Parity */
#define NCR710_DBC_REG          0x24    /* DMA Byte Counter (24-bit, LE) */
#define NCR710_DCMD_REG         0x27    /* DMA Command */
#define NCR710_DNAD_REG         0x28    /* DMA Next Data Address (32-bit, LE) */
#define NCR710_DSP_REG          0x2C    /* DMA SCRIPTS Pointer (32-bit, LE) */
#define NCR710_DSPS_REG         0x30    /* DMA SCRIPTS Pointer Save (32-bit, LE) */
#define NCR710_SCRATCH_REG      0x34    /* Scratch (32-bit, LE) */
#define NCR710_DMODE_REG        0x38    /* DMA Mode */
#define NCR710_DIEN_REG         0x39    /* DMA Interrupt Enable */
#define NCR710_DWT_REG          0x3A    /* DMA Watchdog Timer */
#define NCR710_DCNTL_REG        0x3B    /* DMA Control */
#define NCR710_ADDER_REG        0x3C    /* Adder Sum Output (32-bit, LE) */



#define AFTER_SELECTION 	0x100
#define BEFORE_CMD 		    0x200
#define AFTER_CMD 		    0x300
#define AFTER_STATUS 		0x400
#define AFTER_DATA_IN		0x500
#define AFTER_DATA_OUT		0x600
#define DURING_DATA_IN		0x700

#define NOT_MSG_OUT 		0x10
#define UNEXPECTED_PHASE 	0x20
#define NOT_MSG_IN 		    0x30
#define UNEXPECTED_MSG		0x40
#define MSG_IN			    0x50
#define SDTR_MSG_R		    0x60
#define REJECT_MSG_R		0x70
#define DISCONNECT		    0x80
#define MSG_OUT			    0x90
#define WDTR_MSG_R		    0xA0

#define GOOD_STATUS     0x1

#define NOT_MSG_OUT_AFTER_SELECTION         0x110
#define UNEXPECTED_PHASE_BEFORE_CMD         0x220
#define UNEXPECTED_PHASE_AFTER_CMD          0x320
#define NOT_MSG_IN_AFTER_STATUS             0x430
#define GOOD_STATUS_AFTER_STATUS            0x401
#define UNEXPECTED_PHASE_AFTER_DATA_IN      0x520
#define UNEXPECTED_PHASE_AFTER_DATA_OUT     0x620
#define UNEXPECTED_MSG_BEFORE_CMD           0x240
#define MSG_IN_BEFORE_CMD                   0x250
#define MSG_IN_AFTER_CMD                    0x350
#define SDTR_MSG_BEFORE_CMD                 0x260
#define REJECT_MSG_BEFORE_CMD               0x270
#define DISCONNECT_AFTER_CMD                0x380
#define SDTR_MSG_AFTER_CMD                  0x360
#define WDTR_MSG_AFTER_CMD                  0x3A0
#define MSG_IN_AFTER_STATUS                 0x440
#define DISCONNECT_AFTER_DATA               0x580
#define MSG_IN_AFTER_DATA_IN                0x550
#define MSG_IN_AFTER_DATA_OUT               0x650
#define MSG_OUT_AFTER_DATA_IN               0x590
#define DATA_IN_AFTER_DATA_IN               0x5a0
#define MSG_IN_DURING_DATA_IN               0x750
#define DISCONNECT_DURING_DATA              0x780

#define RESELECTED_DURING_SELECTION      0x1000
#define COMPLETED_SELECTION_AS_TARGET    0x1001
#define RESELECTION_IDENTIFIED           0x1003

#define FATAL                   0x2000
#define FATAL_UNEXPECTED_RESELECTION_MSG 0x2000
#define FATAL_SEND_MSG          0x2001
#define FATAL_NOT_MSG_IN_AFTER_SELECTION 0x2002
#define FATAL_ILLEGAL_MSG_LENGTH 0x2003

#define DEBUG_INTERRUPT     0x3000
#define DEBUG_INTERRUPT1    0x3001
#define DEBUG_INTERRUPT2    0x3002
#define DEBUG_INTERRUPT3    0x3003
#define DEBUG_INTERRUPT4    0x3004
#define DEBUG_INTERRUPT5    0x3005
#define DEBUG_INTERRUPT6    0x3006

#define COMMAND_COMPLETE_MSG    0x00
#define EXTENDED_MSG		    0x01
#define SDTR_MSG		        0x01
#define SAVE_DATA_PTRS_MSG	    0x02
#define RESTORE_DATA_PTRS_MSG	0x03
#define WDTR_MSG		        0x03
#define DISCONNECT_MSG		    0x04
#define REJECT_MSG		        0x07
#define PARITY_ERROR_MSG	    0x09
#define SIMPLE_TAG_MSG		    0x20
#define IDENTIFY_MSG		    0x80
#define IDENTIFY_MSG_MASK	    0x7F
#define TWO_BYTE_MSG		    0x20
#define TWO_BYTE_MSG_MASK	    0x0F



/* NCR710 register size */
#define NCR710_REG_SIZE         0x100

/* Alias names for backward compatibility */
#define NCR710_REVISION_2       0x02



/* Other constants */
#define NCR710_BUF_SIZE         4096
#define NCR710_HOST_ID          7
#define NCR710_MAX_MSGIN_LEN    8

#define NCR710_SCSI_FIFO_SIZE   8

/* Forward declarations */
typedef struct NCR710State NCR710State;
typedef struct NCR710Request NCR710Request;

/* SCSI FIFO structure - 8 transfers deep, 1 byte per transfer (9-bit wide with parity) */
typedef struct {
    uint8_t data[NCR710_SCSI_FIFO_SIZE];    /* SCSI FIFO buffer (8 bytes deep) */
    uint8_t parity[NCR710_SCSI_FIFO_SIZE];  /* Parity bits for each byte (9th bit) */
    int head;                                /* Head pointer for dequeue (0-7) */
    int count;                               /* Number of valid entries (0-8) */
} NCR710_SCSI_FIFO;

/* Request structure */
struct NCR710Request {
    SCSIRequest *req;
    uint32_t tag;
    uint32_t dma_len;
    uint32_t pending;
    uint8_t status;
    bool active;
    uint8_t *dma_buf;          /* DMA buffer pointer */
    bool out;                  /* Direction flag: true for output, false for input */
    uint32_t resume_offset;    /* SCRIPTS resume point after reselection */
    uint32_t saved_dnad;       /* Saved DMA address for immediate reselection */
};

/* NCR710 State structure */
struct NCR710State {
    SysBusDevice parent_obj;

    /* Memory and IRQ resources */
    MemoryRegion mmio;
    qemu_irq irq;

    /* SCSI bus */
    SCSIBus bus;
    AddressSpace *as;

    /* Registers */
    uint8_t scntl0;
    uint8_t scntl1;
    uint8_t sdid;
    uint8_t sien0;  /* Changed from sien */
    uint8_t scid;
    uint8_t sxfer;
    uint8_t sodl;
    uint8_t socl;
    uint8_t sfbr;
    uint8_t sidl;
    uint8_t sbdl;
    uint8_t sbcl;
    uint8_t dstat;
    uint8_t sstat0;
    uint8_t sstat1;
    uint8_t sstat2;
    uint32_t dsa;
    uint8_t ctest0;
    uint8_t ctest1;
    uint8_t ctest2;
    uint8_t ctest3;
    uint8_t ctest4;
    uint8_t ctest5;
    uint8_t ctest6;
    uint8_t ctest7;
    uint8_t ctest8;
    uint32_t temp;
    uint8_t dfifo;
    uint8_t istat;
    uint8_t lcrc;
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

    /* FIFO */
    NCR710_SCSI_FIFO scsi_fifo;

    /* Current SCSI command state */
    NCR710Request *current;    /* Changed from current_req */
    uint8_t status;
    uint8_t msg[NCR710_MAX_MSGIN_LEN];
    uint8_t msg_len;
    uint8_t msg_action;
    int carry;
    bool script_active;
    int waiting;
    int dma_pending;
    uint8_t command_complete;

    /* Script execution timer */
    QEMUTimer *script_timer;
    QEMUTimer *completion_irq_timer;
    QEMUTimer *reselection_retry_timer;  /* Timer for deferred reselection retry */

    /* FIX #17: Saved DSPS value for delayed interrupt */
    uint32_t saved_dsps;

    /* FIX #19: Track last DSPS to detect rapid 0x780->0x401 sequence */
    uint32_t last_dsps_generated;

    /* Additional required fields */
    uint32_t select_tag;       /* Select tag for SCSI device selection */
    uint8_t current_lun;       /* Current logical unit number */
    bool big_endian;           /* Endianness flag */
    int burst_length;          /* DMA burst length */
    bool tolerant_enabled;     /* Tolerant mode enabled flag */
    bool differential_mode;    /* Differential mode flag */
    bool cache_line_burst;     /* Cache line burst flag */
    uint8_t reselection_id;
    bool wait_reselect;
};

/* Define SysBusNCR710State */
typedef struct SysBusNCR710State {
    SysBusDevice parent_obj;
    MemoryRegion mmio;
    MemoryRegion iomem;
    qemu_irq irq;
    NCR710State ncr710;
} SysBusNCR710State;

/* Define register size */
#define NCR710_REG_SIZE         0x100

static inline NCR710State *ncr710_from_scsi_bus(SCSIBus *bus)
{
    return container_of(bus, NCR710State, bus);
}

static inline SysBusNCR710State *sysbus_from_ncr710(NCR710State *s)
{
    return container_of(s, SysBusNCR710State, ncr710);
}

/* Function prototypes */
DeviceState *ncr53c710_init(MemoryRegion *address_space, hwaddr addr, qemu_irq irq);
DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq);
void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);
uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size);
void ncr710_soft_reset(NCR710State *s);

/* NCR710 core SCSI callback functions */
void ncr710_request_cancelled(SCSIRequest *req);
void ncr710_command_complete(SCSIRequest *req, size_t resid);
void ncr710_transfer_data(SCSIRequest *req, uint32_t len);
void ncr710_execute_script(NCR710State *s);
void ncr710_set_phase(NCR710State *s, int phase);

/* NCR710 script timer callback */
void ncr710_script_timer_callback(void *opaque);
void ncr710_completion_irq_callback(void *opaque);
void ncr710_reselection_retry_callback(void *opaque);

#endif /* HW_NCR53C710_H */
