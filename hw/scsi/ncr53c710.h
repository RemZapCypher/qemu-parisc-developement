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

/* FIFO configuration */
/* NCR53C710 has 8-byte SCSI FIFO and 64-byte DMA FIFO */
#define NCR710_SCSI_FIFO_SIZE   8
#define NCR710_DMA_FIFO_SIZE    64
#define NCR710_FIFO_DEPTH       16  /* General FIFO depth for other operations */
#define NCR710_FIFO_FULL        0x01
#define NCR710_FIFO_EMPTY       0x02


#define DEBUG_NCR710
#define DEBUG_NCR710_REG

#ifdef DEBUG_NCR710
#define DPRINTF(fmt, ...) \
    qemu_log("ncr710: " fmt, ## __VA_ARGS__)
#define BADF(fmt, ...) \
    qemu_log("ncr710: error: " fmt, ## __VA_ARGS__)
#define NCR710_DPRINTF(fmt, ...) \
    qemu_log("ncr710: " fmt, ## __VA_ARGS__)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#define BADF(fmt, ...) \
    qemu_log("ncr710: error: " fmt, ## __VA_ARGS__)
#define NCR710_DPRINTF(fmt, ...) do {} while(0)
#endif

/* Define register IDs */
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

/* NCR710 register size */
#define NCR710_REG_SIZE         0x100

/* Other constants */
#define NCR710_BUF_SIZE         4096
#define NCR710_HOST_ID          7
#define NCR710_DEFAULT_HOST_ID  7
#define NCR710_MAX_MSGIN_LEN    8
#define NCR710_TAG_VALID        (1 << 16)

/* SCSI phases */
#define PHASE_DO   0  /* Data out phase */
#define PHASE_DI   1  /* Data in phase */
#define PHASE_CO   2  /* Command phase */
#define PHASE_SI   3  /* Status phase */
#define PHASE_MO   6  /* Message out phase */
#define PHASE_MI   7  /* Message in phase */
#define PHASE_MASK 7  /* Mask for phase bits */

/* Alias names for backward compatibility */
#define PHASE_CMD  PHASE_CO  /* Command phase alias */
#define PHASE_ST   PHASE_SI  /* Status phase alias */
#define NCR710_REVISION_2       0x02

#define TYPE_NCR53C710 "ncr53c710"
#define TYPE_SYSBUS_NCR710_SCSI "ncr53c710"
#define SYSBUS_NCR710_SCSI(obj) \
    OBJECT_CHECK(SysBusNCR710State, (obj), TYPE_SYSBUS_NCR710_SCSI)
OBJECT_DECLARE_SIMPLE_TYPE(NCR710State, NCR53C710)

/* DMA FIFO structure - 64 bytes with parity */
typedef struct {
    uint8_t data[NCR710_DMA_FIFO_SIZE];     /* DMA FIFO buffer (64 bytes) */
    uint8_t parity[NCR710_DMA_FIFO_SIZE];   /* Parity bits for each byte */
    int head;                                /* Write pointer */
    int tail;                                /* Read pointer */
    int count;                               /* Number of valid entries */
} NCR710_DMA_FIFO;

/* SCSI FIFO structure - 8 bytes with parity */
typedef struct {
    uint8_t data[NCR710_SCSI_FIFO_SIZE];    /* SCSI FIFO buffer (8 bytes) */
    uint8_t parity[NCR710_SCSI_FIFO_SIZE];  /* Parity bits for each byte */
    int count;                               /* Number of valid entries */
} NCR710_SCSI_FIFO;

/* Combined FIFO structure for NCR53C710 */
typedef struct {
    NCR710_DMA_FIFO dma;                    /* DMA FIFO */
    NCR710_SCSI_FIFO scsi;                  /* SCSI FIFO */
    uint8_t status;                         /* FIFO status flags */
} NCR710FIFO;

/* Forward declaration */
typedef struct NCR710Request NCR710Request;

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
    QTAILQ_ENTRY(NCR710Request) next;
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
    QTAILQ_HEAD(, NCR710Request) queue;

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
    uint8_t temp;
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
    NCR710_DMA_FIFO dma_fifo;   /* Changed from NCR710FIFO fifo */
    NCR710_SCSI_FIFO scsi_fifo;
    
    /* Current SCSI command state */
    NCR710Request *current;    /* Changed from current_req */
    uint8_t status;
    uint8_t msg[NCR710_MAX_MSGIN_LEN];
    uint8_t msg_len;
    uint8_t msg_action;
    int carry;                 /* Added */
    bool script_active;        /* Added */
    int waiting;               /* Added */
    bool mode_700;             /* Added - True for 700 mode, false for 710 mode */
    int dma_pending;           /* Added */
    uint8_t command_complete;  /* Added */
    
    /* Additional required fields */
    uint32_t select_tag;       /* Select tag for SCSI device selection */
    uint8_t current_lun;       /* Current logical unit number */
    bool big_endian;           /* Endianness flag */
    int burst_length;          /* DMA burst length */
    bool tolerant_enabled;     /* Tolerant mode enabled flag */
    bool differential_mode;    /* Differential mode flag */
    bool cache_line_burst;     /* Cache line burst flag */
    
    /* Timers */
    QEMUTimer *selection_timer; /* Selection timer */
    QEMUTimer *watchdog_timer;  /* Watchdog timer */
    
    /* Scripts state */
    struct {
        bool running;          /* Scripts running flag */
        uint32_t pc;           /* Scripts program counter */
    } scripts;
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

/* Function prototypes */
void ncr710_init(DeviceState *dev);
void ncr710_reset(DeviceState *dev);
uint64_t ncr710_mmio_read(void *opaque, hwaddr addr, unsigned size);
void ncr710_mmio_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);
void ncr710_irq_raise(NCR710State *s);
void ncr710_irq_lower(NCR710State *s);

/* Device creation functions */
DeviceState *ncr53c710_init(MemoryRegion *address_space, hwaddr addr, qemu_irq irq);
DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq);

/* Mode setting functions */
bool ncr710_is_700_mode(NCR710State *s);
void ncr710_set_700_mode(NCR710State *s, bool mode_700);

/* SCSI callbacks */
void ncr710_request_cancelled(SCSIRequest *req);
void ncr710_command_complete(SCSIRequest *req, size_t resid);
void ncr710_transfer_data(SCSIRequest *req, uint32_t len);

/* SCSI bus helper */
static inline NCR710State *ncr710_from_scsi_bus(SCSIBus *bus)
{
    return container_of(bus, NCR710State, bus);
}

/* SysBus helper */
static inline SysBusNCR710State *sysbus_from_ncr710(NCR710State *s)
{
    return container_of(s, SysBusNCR710State, ncr710);
}

/* FIFO operations */
int ncr710_fifo_push(NCR710State *s, uint8_t data, uint8_t parity);
int ncr710_fifo_pop(NCR710State *s, uint8_t *data, uint8_t *parity);
void ncr710_fifo_reset(NCR710State *s);
uint8_t ncr710_calculate_parity(uint8_t data, bool odd_parity);

/* Additional helper functions */
uint8_t ncr710_generate_scsi_parity(NCR710State *s, uint8_t data);
void ncr710_update_compatibility_mode(NCR710State *s, bool mode_700);
void ncr710_arbitrate_bus(NCR710State *s);
void ncr710_internal_scsi_bus_reset(NCR710State *s);
void ncr710_scripts_execute(NCR710State *s);
uint32_t ncr710_dma_fifo_drain_to_memory(NCR710State *s, uint32_t addr, int count);
const char *ncr710_reg_name(int offset);

#endif /* HW_NCR53C710_H */
