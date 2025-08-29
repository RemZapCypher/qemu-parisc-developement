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
#include "hw/sysbus.h"
#include "hw/scsi/scsi.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "system/dma.h"
#include "exec/address-spaces.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "trace.h"
#include "qapi/error.h"
#include "hw/scsi/ncr53c710.h"

/* Big Endian addressing - access size is 8-bit for all registers
 * NCR53C710 Register Offsets (from Table 4.1 Register Address Map)
 */
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
#define NCR710_DSA_REG          0x10    /* Data Structure Address (32-bit, LE) */
#define NCR710_CTEST0_REG       0x14    /* Chip Test Zero */
#define NCR710_CTEST1_REG       0x15    /* Chip Test One */
#define NCR710_CTEST2_REG       0x16    /* Chip Test Two */
#define NCR710_CTEST3_REG       0x17    /* Chip Test Three */
#define NCR710_CTEST4_REG       0x18    /* Chip Test Four */
#define NCR710_CTEST5_REG       0x19    /* Chip Test Five */
#define NCR710_CTEST6_REG       0x1A    /* Chip Test Six */
#define NCR710_CTEST7_REG       0x1B    /* Chip Test Seven */
#define NCR710_TEMP_REG         0x1C    /* Temporary Stack (32-bit, LE) */
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

/* SCNTL0 - SCSI Control Zero (Register 0x00) */
#define SCNTL0_ARB1             0x80    /* Arbitration Mode Bit 1 */
#define SCNTL0_ARB0             0x40    /* Arbitration Mode Bit 0 */
#define SCNTL0_ARB_MASK         0xC0    /* Arbitration Mode Mask */
#define SCNTL0_ARB_SIMPLE       0x00    /* Simple arbitration */
#define SCNTL0_ARB_FULL         0xC0    /* Full arbitration, selection/reselection */
#define SCNTL0_START            0x20    /* Start Sequence */
#define SCNTL0_WATN             0x10    /* Select with ATN/ on Start Sequence */
#define SCNTL0_EPC              0x08    /* Enable Parity Checking */
#define SCNTL0_EPG              0x04    /* Enable Parity Generation */
#define SCNTL0_AAP              0x02    /* Assert ATN/ on Parity Error */
#define SCNTL0_TRG              0x01    /* Target Mode */

/* SCNTL1 - SCSI Control One (Register 0x01) */
#define SCNTL1_EXC              0x80    /* Extra Clock Cycle */
#define SCNTL1_ADB              0x40    /* Assert Data Bus */
#define SCNTL1_ESR              0x20    /* Enable Selection/Reselection */
#define SCNTL1_CON              0x10    /* Connected */
#define SCNTL1_RST              0x08    /* Assert SCSI RST/ */
#define SCNTL1_AESP             0x04    /* Assert Even SCSI Parity */
#define SCNTL1_SND              0x02    /* Start SCSI Send */
#define SCNTL1_RCV              0x01    /* Start SCSI Receive */

/* SIEN - SCSI Interrupt Enable (Register 0x03) */
#define SIEN_M_A                0x80    /* Phase Mismatch/ATN Active */
#define SIEN_FCMP               0x40    /* Function Complete */
#define SIEN_STO                0x20    /* SCSI Bus Timeout */
#define SIEN_SEL                0x10    /* Selected/Reselected */
#define SIEN_SGE                0x08    /* SCSI Gross Error */
#define SIEN_UDC                0x04    /* Unexpected Disconnect */
#define SIEN_RST                0x02    /* SCSI RST/ Received */
#define SIEN_PAR                0x01    /* Parity Error */

/* SXFER - SCSI Transfer (Register 0x05) */
#define SXFER_DHP               0x80    /* Disable Halt on ATN/ or Parity Error (Target Mode) */
#define SXFER_TP2               0x40    /* Transfer Period bit 2 */
#define SXFER_TP1               0x20    /* Transfer Period bit 1 */
#define SXFER_TP0               0x10    /* Transfer Period bit 0 */
#define SXFER_TP_MASK           0x70    /* Transfer Period Mask */
#define SXFER_MO3               0x08    /* Max Offset bit 3 */
#define SXFER_MO2               0x04    /* Max Offset bit 2 */
#define SXFER_MO1               0x02    /* Max Offset bit 1 */
#define SXFER_MO0               0x01    /* Max Offset bit 0 */
#define SXFER_MO_MASK           0x0F    /* Max Offset Mask */

/* SOCL - SCSI Output Control Latch (Register 0x07) */
#define SOCL_REQ                0x80    /* Assert SCSI REQ/ */
#define SOCL_ACK                0x40    /* Assert SCSI ACK/ */
#define SOCL_BSY                0x20    /* Assert SCSI BSY/ */
#define SOCL_SEL                0x10    /* Assert SCSI SEL/ */
#define SOCL_ATN                0x08    /* Assert SCSI ATN/ */
#define SOCL_MSG                0x04    /* Assert SCSI MSG/ */
#define SOCL_CD                 0x02    /* Assert SCSI C/D/ */
#define SOCL_IO                 0x01    /* Assert SCSI I/O/ */

/* SBCL - SCSI Bus Control Lines (Register 0x0B) */
#define SBCL_REQ                0x80    /* SCSI REQ/ */
#define SBCL_ACK                0x40    /* SCSI ACK/ */
#define SBCL_BSY                0x20    /* SCSI BSY/ */
#define SBCL_SEL                0x10    /* SCSI SEL/ */
#define SBCL_ATN                0x08    /* SCSI ATN/ */
#define SBCL_MSG                0x04    /* SCSI MSG/ */
#define SBCL_CD                 0x02    /* SCSI C/D/ */
#define SBCL_IO                 0x01    /* SCSI I/O/ */

/* DSTAT - DMA Status (Register 0x0C) */
#define DSTAT_DFE               0x80    /* DMA FIFO Empty */
#define DSTAT_MDPE              0x40    /* Master Data Parity Error */
#define DSTAT_BF                0x20    /* Bus Fault */
#define DSTAT_ABRT              0x10    /* Aborted */
#define DSTAT_SSI               0x08    /* SCRIPTS Step Interrupt */
#define DSTAT_SIR               0x04    /* SCRIPTS Interrupt */
#define DSTAT_WTD               0x02    /* Watchdog Timer Expired */
#define DSTAT_IID               0x01    /* Illegal Instruction Detected */

/* SSTAT0 - SCSI Status Zero (Register 0x0D) */
#define SSTAT0_M_A              0x80    /* Phase Mismatch/ATN Active */
#define SSTAT0_FCMP             0x40    /* Function Complete */
#define SSTAT0_STO              0x20    /* SCSI Bus Timeout */
#define SSTAT0_SEL              0x10    /* Selected/Reselected */
#define SSTAT0_SGE              0x08    /* SCSI Gross Error */
#define SSTAT0_UDC              0x04    /* Unexpected Disconnect */
#define SSTAT0_RST              0x02    /* SCSI RST/ Received */
#define SSTAT0_PAR              0x01    /* Parity Error */

/* SSTAT1 - SCSI Status One (Register 0x0E) */
#define SSTAT1_ILF              0x80    /* Input Latch Full */
#define SSTAT1_ORF              0x40    /* Output Register Full */
#define SSTAT1_OLF              0x20    /* Output Latch Full */
#define SSTAT1_AIP              0x10    /* Arbitration In Progress */
#define SSTAT1_LOA              0x08    /* Lost Arbitration */
#define SSTAT1_WOA              0x04    /* Won Arbitration */
#define SSTAT1_RST              0x02    /* SCSI RST/ */
#define SSTAT1_SDP              0x01    /* SCSI Parity */

/* SSTAT2 - SCSI Status Two (Register 0x0F) */
#define SSTAT2_FF3              0x80    /* FIFO Flags bit 3 */
#define SSTAT2_FF2              0x40    /* FIFO Flags bit 2 */
#define SSTAT2_FF1              0x20    /* FIFO Flags bit 1 */
#define SSTAT2_FF0              0x10    /* FIFO Flags bit 0 */
#define SSTAT2_SPL1             0x08    /* SCSI Phase Latch bit 1 */
#define SSTAT2_SPL0             0x04    /* SCSI Phase Latch bit 0 */
#define SSTAT2_MSP              0x02    /* MSG/ Phase */
#define SSTAT2_SDP              0x01    /* SCSI Data Parity */

/* ISTAT - Interrupt Status (Register 0x21) */
#define ISTAT_ABRT              0x80    /* Abort Operation */
#define ISTAT_RST               0x40    /* Software Reset */
#define ISTAT_SIGP              0x20    /* Signal Process */
#define ISTAT_SEM               0x10    /* Semaphore */
#define ISTAT_CON               0x08    /* Connected */
#define ISTAT_INTF              0x04    /* Interrupt on Fly */
#define ISTAT_SIP               0x02    /* SCSI Interrupt Pending */
#define ISTAT_DIP               0x01    /* DMA Interrupt Pending */

/* DMODE - DMA Mode (Register 0x38) */
#define DMODE_BL1               0x80    /* Burst Length bit 1 */
#define DMODE_BL0               0x40    /* Burst Length bit 0 */
#define DMODE_BL_MASK           0xC0    /* Burst Length Mask */
#define DMODE_SIOM              0x20    /* Source I/O Memory Enable */
#define DMODE_DIOM              0x10    /* Destination I/O Memory Enable */
#define DMODE_ERL               0x08    /* Enable Read Line */
#define DMODE_ERMP              0x04    /* Enable Read Multiple */
#define DMODE_BOF               0x02    /* Burst Opcode Fetch */
#define DMODE_MAN               0x01    /* Manual Start Mode */

/* DIEN - DMA Interrupt Enable (Register 0x39) */
#define DIEN_MDPE               0x40    /* Master Data Parity Error */
#define DIEN_BF                 0x20    /* Bus Fault */
#define DIEN_ABRT               0x10    /* Aborted */
#define DIEN_SSI                0x08    /* SCRIPTS Step Interrupt */
#define DIEN_SIR                0x04    /* SCRIPTS Interrupt */
#define DIEN_WTD                0x02    /* Watchdog Timer Expired */
#define DIEN_IID                0x01    /* Illegal Instruction Detected */

/* DCNTL - DMA Control (Register 0x3B) */
#define DCNTL_CLSE              0x80    /* Cache Line Size Enable */
#define DCNTL_PFF               0x40    /* Prefetch Flush */
#define DCNTL_PFEN              0x20    /* Prefetch Enable */
#define DCNTL_SSM               0x10    /* Single Step Mode */
#define DCNTL_IRQM              0x08    /* IRQ Mode */
#define DCNTL_STD               0x04    /* Start DMA */
#define DCNTL_IRQD              0x02    /* IRQ Disable */
#define DCNTL_COM               0x01    /* 53C700 Compatibility */

/* SCRIPTS instruction types (bits 31:30 of instruction word) */
#define SCRIPTS_TYPE_MASK       0xC0000000
#define SCRIPTS_TYPE_BLOCK_MOVE 0x00000000  /* Block Move */
#define SCRIPTS_TYPE_IO         0x40000000  /* I/O instruction */
#define SCRIPTS_TYPE_READ_WRITE 0x80000000  /* Read/Write instruction */
#define SCRIPTS_TYPE_TRANSFER   0xC0000000  /* Transfer Control instruction */

/* Block Move instruction bits */
#define SCRIPTS_BM_INDIRECT     0x20000000  /* Indirect addressing */
#define SCRIPTS_BM_TABLE        0x10000000  /* Table indirect addressing */
#define SCRIPTS_BM_OPCODE       0x08000000  /* Opcode: 0=CHMOV, 1=MOVE */
#define SCRIPTS_BM_PHASE_MASK   0x07000000  /* SCSI phase mask */
#define SCRIPTS_BM_COUNT_MASK   0x00FFFFFF  /* Byte count mask */

/* Constants for FIFO sizes */
#define NCR710_DMA_FIFO_SIZE    64
#define NCR710_SCSI_FIFO_SIZE   8

/* I/O instruction opcodes (bits 29:27) */
#define SCRIPTS_IO_OPCODE_MASK  0x38000000
#define SCRIPTS_IO_RESELECT     0x00000000  /* Reselect */
#define SCRIPTS_IO_DISCONNECT   0x08000000  /* Disconnect */
#define SCRIPTS_IO_WAIT_SEL     0x10000000  /* Wait for Select */
#define SCRIPTS_IO_SET          0x18000000  /* Set */
#define SCRIPTS_IO_CLEAR        0x20000000  /* Clear */

/* Transfer Control instruction opcodes (bits 29:27) */
#define SCRIPTS_TC_OPCODE_MASK  0x38000000
#define SCRIPTS_TC_JUMP         0x00000000  /* Jump */
#define SCRIPTS_TC_CALL         0x08000000  /* Call */
#define SCRIPTS_TC_RETURN       0x10000000  /* Return */
#define SCRIPTS_TC_INT          0x18000000  /* Interrupt */

/* SCSI phases */
#define SCSI_PHASE_DATA_OUT     0x00
#define SCSI_PHASE_DATA_IN      0x01
#define SCSI_PHASE_COMMAND      0x02
#define SCSI_PHASE_STATUS       0x03
#define SCSI_PHASE_MSG_OUT      0x06
#define SCSI_PHASE_MSG_IN       0x07

/* FIFO sizes */
#define NCR710_DMA_FIFO_SIZE    64
#define NCR710_SCSI_FIFO_SIZE   8

/* Timing constants */
#define NCR710_SELECTION_TIMEOUT_MS   250
#define NCR710_BUS_SETTLE_DELAY       1200  /* 1.2 µs in ns */

/* Device constants - Based on Linux driver settings */
#define NCR710_VENDOR_ID        0x1000  /* LSI Logic */
#define NCR710_DEVICE_ID        0x0001  /* NCR53C710 */
#define NCR710_REVISION_2       0x20    /* Revision 2 */

/* Default clock frequency (from mvme16x driver) */
#define NCR710_DEFAULT_CLOCK_MHZ    50  /* 50MHz */

/* Driver-specific feature flags (from mvme16x_scsi.c) */
#define NCR710_CHIP710_FLAG     0x0001  /* chip710 = 1 */
#define NCR710_DMODE_FC2        0x02    /* dmode_extra = DMODE_FC2 */
#define NCR710_DCNTL_EA         0x20    /* dcntl_extra = EA_710 */
#define NCR710_CTEST7_TT1       0x02    /* ctest7_extra = CTEST7_TT1 */

/* Host adapter defaults (from Linux driver) */
#define NCR710_DEFAULT_HOST_ID  7       /* Default SCSI ID */

/* Extended CTEST7 bits (from driver) */
#define CTEST7_TT1              0x02    /* Transfer Type 1 */
#define CTEST7_DIFF             0x01    /* Differential SCSI */

/* Extended DCNTL bits (from driver) */
#define DCNTL_EA                0x20    /* Enable Ack */

/* Extended DMODE bits (from driver) */
#define DMODE_FC2               0x02    /* Flow Control 2 */

/* DMA FIFO structure - 64 bytes x 9 bits (4 byte lanes of 16 entries each) */
typedef struct {
    uint8_t data[NCR710_DMA_FIFO_SIZE];
    uint8_t parity[NCR710_DMA_FIFO_SIZE];
    int head;       /* Write pointer */
    int tail;       /* Read pointer */
    int count;      /* Number of valid entries */
} NCR710_DMA_FIFO;

/* SCSI FIFO structure - 8 bytes */
typedef struct {
    uint8_t data[NCR710_SCSI_FIFO_SIZE];
    uint8_t parity[NCR710_SCSI_FIFO_SIZE];
    int count;      /* Number of valid bytes */
} NCR710_SCSI_FIFO;

/* Type aliases for compatibility */
typedef NCR710_DMA_FIFO NCR710_DMA_FIFO;
typedef NCR710_SCSI_FIFO NCR710_SCSI_FIFO;

/* SCRIPTS execution context */
typedef struct {
    bool running;
    uint32_t pc;        /* Current program counter (DSP) */
    uint32_t saved_pc;  /* Saved PC for calls (TEMP) */
    uint8_t phase;      /* Current SCSI phase */
    bool connected;     /* Connected to SCSI bus */
    bool initiator;     /* True if initiator, false if target */
} NCR710_SCRIPTS_Context;

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

    /* DMA Core and Status Registers (0x10-0x13) */
    uint8_t dstat;              /* 0x14 DMA Status */
    uint8_t sstat0;             /* 0x15 SCSI Status 0 */
    uint8_t sstat1;             /* 0x16 SCSI Status 1 */
    uint8_t sstat2;             /* 0x17 SCSI Status 2 */

    /* DMA Registers (0x18-0x3F) */
    uint32_t dsa;               /* 0x10 Data Structure Address */
    uint8_t istat;              /* 0x14 Interrupt Status */
    uint32_t dbc;               /* 0x18 DMA Byte Counter */
    uint8_t dcmd;               /* 0x1B DMA Command */
    uint32_t dnad;              /* 0x1C DMA Next Address for Data */
    uint32_t dsp;               /* 0x20 DMA SCRIPTS Pointer */
    uint32_t dsps;              /* 0x24 DMA SCRIPTS Pointer Save */
    uint32_t scratch;           /* 0x28-0x2B Scratch Registers A-D */
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

    /* Internal FIFOs */
    NCR710_DMA_FIFO dma_fifo;
    NCR710_SCSI_FIFO scsi_fifo;

    /* SCRIPTS execution context */
    NCR710_SCRIPTS_Context scripts;

    /* Current SCSI operation state */
    SCSIRequest *current_req;
    uint8_t *current_dma_buf;
    uint32_t current_dma_len;
    uint8_t current_scsi_phase;
    bool selection_timeout_enabled;

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
};

/* SysBus device wrapper for NCR710 */
struct SysBusNCR710State {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;
    uint32_t it_shift;
    NCR710State ncr710;
};


/* Function prototypes for implemented functions only */
/* 1. Core device functions */
static void ncr710_update_irq(NCR710State *s);
static void ncr710_arbitrate_bus(NCR710State *s);
static void ncr710_internal_scsi_bus_reset(NCR710State *s);
static void ncr710_device_reset(DeviceState *dev);
static void ncr710_scripts_execute(NCR710State *s);
static void ncr710_dma_transfer(NCR710State *s);

/* 2. FIFO management functions */
static void ncr710_dma_fifo_init(NCR710_DMA_FIFO *fifo);
static bool ncr710_dma_fifo_empty(NCR710_DMA_FIFO *fifo);
static bool ncr710_dma_fifo_full(NCR710_DMA_FIFO *fifo);
static void ncr710_dma_fifo_push(NCR710_DMA_FIFO *fifo, uint8_t data, uint8_t parity);
static uint8_t ncr710_dma_fifo_pop(NCR710_DMA_FIFO *fifo, uint8_t *parity);
static void ncr710_dma_fifo_flush(NCR710_DMA_FIFO *fifo);

static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo);
static bool ncr710_scsi_fifo_empty(NCR710_SCSI_FIFO *fifo);
static bool ncr710_scsi_fifo_full(NCR710_SCSI_FIFO *fifo);
static void ncr710_scsi_fifo_push(NCR710_SCSI_FIFO *fifo, uint8_t data, uint8_t parity);
static uint8_t ncr710_scsi_fifo_pop(NCR710_SCSI_FIFO *fifo, uint8_t *parity);

/* 3. Memory access helpers */
static uint32_t ncr710_read_memory_32(NCR710State *s, uint32_t addr);
static uint32_t ncr710_calculate_parity(uint8_t data);

/* 4. Register access functions */
static uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size);
static void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);

/* 5. SCSI command initiation */
/* TODO: integrate with SCRIPTS processor */
/* static void ncr710_scsi_command_start(NCR710State *s, uint8_t target, uint8_t lun, uint8_t *cdb, uint32_t cdb_len); */

/* 6. SCSI bus callback functions */
static void ncr710_transfer_data(SCSIRequest *req, uint32_t len);
static void ncr710_command_complete(SCSIRequest *req, size_t resid);
static void ncr710_request_cancelled(SCSIRequest *req);

/* 7. Device initialization and class functions */
static void sysbus_ncr710_realize(DeviceState *dev, Error **errp);
static void sysbus_ncr710_init(Object *obj);
static void sysbus_ncr710_class_init(ObjectClass *oc, void *data);
static void ncr710_register_types(void);

/* Register access functions
 */
static uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR710State *s = (NCR710State *)opaque;
    uint64_t ret = 0;

    qemu_log("NCR710: READ addr=0x%02x size=%d\n", (int)addr, size);

    switch (addr) {
    case NCR710_SCNTL0_REG:
        ret = s->scntl0;
        break;
    case NCR710_SCNTL1_REG:
        ret = s->scntl1;
        break;
    case NCR710_SDID_REG:
        ret = s->sdid;
        break;
    case NCR710_SIEN_REG:
        ret = s->sien;
        break;
    case NCR710_SCID_REG:
        ret = s->scid;
        if ((ret & 0x7F) == 0) {
            ret = 0x80 | NCR710_DEFAULT_HOST_ID;
        } else {
            ret |= 0x80;
        }
        break;
    case NCR710_SXFER_REG:
        ret = s->sxfer;
        break;
    case NCR710_SODL_REG:
        ret = s->sodl;
        break;
    case NCR710_SOCL_REG:
        ret = s->socl;
        break;
    case NCR710_SFBR_REG:
        ret = s->sfbr;
        break;
    case NCR710_SIDL_REG:
        ret = s->sidl;
        s->sstat1 &= ~SSTAT1_ILF;
        break;
    case NCR710_SBDL_REG:
        ret = s->sbdl;
        break;
    case NCR710_SBCL_REG:
        ret = s->sbcl;
        break;
    case NCR710_DSTAT_REG:
        ret = s->dstat;
        if (s->dstat & (DSTAT_SSI | DSTAT_SIR | DSTAT_WTD | DSTAT_IID)) {
            s->dstat &= ~(DSTAT_SSI | DSTAT_SIR | DSTAT_WTD | DSTAT_IID);
            s->istat &= ~ISTAT_DIP;
            ncr710_update_irq(s);
        }
        break;
    case NCR710_SSTAT0_REG:
        ret = s->sstat0;
        if (s->sstat0 != 0) {
            s->sstat0 = 0;
            s->istat &= ~ISTAT_SIP;
            ncr710_update_irq(s);
        }
        break;
    case NCR710_SSTAT1_REG:
        ret = s->sstat1;
        break;
    case NCR710_SSTAT2_REG:
        ret = s->sstat2;
        break;

    case NCR710_DSA_REG:
        ret = s->dsa & 0xFF;
        break;
    case NCR710_DSA_REG + 1:
        ret = (s->dsa >> 8) & 0xFF;
        break;
    case NCR710_DSA_REG + 2:
        ret = (s->dsa >> 16) & 0xFF;
        break;
    case NCR710_DSA_REG + 3:
        ret = (s->dsa >> 24) & 0xFF;
        break;

    case NCR710_CTEST0_REG:
        ret = s->ctest0;
        break;
    case NCR710_CTEST1_REG:
        ret = s->ctest1;
        break;
    case NCR710_CTEST2_REG:
        ret = s->ctest2;
        if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
            s->ctest2 |= 0x04;
        } else {
            s->ctest2 &= ~0x04;
        }
        break;
    case NCR710_CTEST3_REG:
        ret = s->ctest3;
        if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
            uint8_t parity;
            ret = ncr710_scsi_fifo_pop(&s->scsi_fifo, &parity);
            if (parity) {
                s->ctest2 |= 0x10;
            } else {
                s->ctest2 &= ~0x10;
            }
        }
        break;
    case NCR710_CTEST4_REG:
        ret = s->ctest4;
        break;
    case NCR710_CTEST5_REG:
        ret = s->ctest5;
        break;
    case NCR710_CTEST6_REG:
        ret = s->ctest6;
        if (!ncr710_dma_fifo_empty(&s->dma_fifo)) {
            uint8_t parity;
            ret = ncr710_dma_fifo_pop(&s->dma_fifo, &parity);
            if (parity) {
                s->ctest2 |= 0x08;
            } else {
                s->ctest2 &= ~0x08;
            }
        }
        break;
    case NCR710_CTEST7_REG:
        ret = s->ctest7;
        break;

    case NCR710_TEMP_REG:
        ret = s->temp & 0xFF;
        break;
    case NCR710_TEMP_REG + 1:
        ret = (s->temp >> 8) & 0xFF;
        break;
    case NCR710_TEMP_REG + 2:
        ret = (s->temp >> 16) & 0xFF;
        break;
    case NCR710_TEMP_REG + 3:
        ret = (s->temp >> 24) & 0xFF;
        break;

    case NCR710_DFIFO_REG:
        ret = s->dfifo;
        s->dfifo = s->dma_fifo.count & 0x7F;
        if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
            s->dfifo |= 0x80; /* Set DFE bit */
        }
        break;
    case NCR710_ISTAT_REG:
        ret = s->istat;
        break;
    case NCR710_CTEST8_REG:
        ret = s->ctest8;
        if (ret == 0) {
            ret = 0x02; /* NCR710 revision 2 */
        }
        break;
    case NCR710_LCRC_REG:
        ret = s->lcrc;
        break;

    case NCR710_DBC_REG:
        ret = s->dbc & 0xFF;
        break;
    case NCR710_DBC_REG + 1:
        ret = (s->dbc >> 8) & 0xFF;
        break;
    case NCR710_DBC_REG + 2:
        ret = (s->dbc >> 16) & 0xFF;
        break;
    case NCR710_DCMD_REG:
        ret = s->dcmd;
        break;

    case NCR710_DNAD_REG:
        ret = s->dnad & 0xFF;
        break;
    case NCR710_DNAD_REG + 1:
        ret = (s->dnad >> 8) & 0xFF;
        break;
    case NCR710_DNAD_REG + 2:
        ret = (s->dnad >> 16) & 0xFF;
        break;
    case NCR710_DNAD_REG + 3:
        ret = (s->dnad >> 24) & 0xFF;
        break;

    case NCR710_DSP_REG:
        ret = s->dsp & 0xFF;
        break;
    case NCR710_DSP_REG + 1:
        ret = (s->dsp >> 8) & 0xFF;
        break;
    case NCR710_DSP_REG + 2:
        ret = (s->dsp >> 16) & 0xFF;
        break;
    case NCR710_DSP_REG + 3:
        ret = (s->dsp >> 24) & 0xFF;
        break;

    case NCR710_DSPS_REG:
        ret = s->dsps & 0xFF;
        break;
    case NCR710_DSPS_REG + 1:
        ret = (s->dsps >> 8) & 0xFF;
        break;
    case NCR710_DSPS_REG + 2:
        ret = (s->dsps >> 16) & 0xFF;
        break;
    case NCR710_DSPS_REG + 3:
        ret = (s->dsps >> 24) & 0xFF;
        break;

    case NCR710_SCRATCH_REG:
        ret = s->scratch & 0xFF;
        break;
    case NCR710_SCRATCH_REG + 1:
        ret = (s->scratch >> 8) & 0xFF;
        break;
    case NCR710_SCRATCH_REG + 2:
        ret = (s->scratch >> 16) & 0xFF;
        break;
    case NCR710_SCRATCH_REG + 3:
        ret = (s->scratch >> 24) & 0xFF;
        break;

    case NCR710_DMODE_REG:
        ret = s->dmode;
        break;
    case NCR710_DIEN_REG:
        ret = s->dien;
        break;
    case NCR710_DWT_REG:
        ret = s->dwt;
        break;
    case NCR710_DCNTL_REG:
        ret = s->dcntl;
        break;

    case NCR710_ADDER_REG:
        ret = s->adder & 0xFF;
        break;
    case NCR710_ADDER_REG + 1:
        ret = (s->adder >> 8) & 0xFF;
        break;
    case NCR710_ADDER_REG + 2:
        ret = (s->adder >> 16) & 0xFF;
        break;
    case NCR710_ADDER_REG + 3:
        ret = (s->adder >> 24) & 0xFF;
        break;

    default:
        trace_ncr710_reg_read_unhandled(addr, size);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "NCR710: invalid read at offset 0x%x\n", (int)addr);
        break;
    }

    trace_ncr710_reg_read(addr, ret);
    return ret;
}

static void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    NCR710State *s = (NCR710State *)opaque;
    uint8_t old_val;

    qemu_log("NCR710: WRITE addr=0x%02x val=0x%02x size=%d\n", (int)addr, (int)val, size);
    trace_ncr710_reg_write(addr, val);

    switch (addr) {
    case NCR710_SCNTL0_REG:
        s->scntl0 = val;
        if (val & SCNTL0_START) {
            ncr710_arbitrate_bus(s);
        }
        break;
    case NCR710_SCNTL1_REG:
        old_val = s->scntl1;
        s->scntl1 = val;
        if ((val & SCNTL1_RST) && !(old_val & SCNTL1_RST)) {
            ncr710_internal_scsi_bus_reset(s);
        }
        break;
    case NCR710_SDID_REG:
        s->sdid = val & 0x0F; /* Only lower 4 bits are valid */
        break;
    case NCR710_SIEN_REG:
        s->sien = val;
        ncr710_update_irq(s);
        break;
    case NCR710_SCID_REG:
        s->scid = val;
        break;
    case NCR710_SXFER_REG:
        s->sxfer = val;
        break;
    case NCR710_SODL_REG:
        s->sodl = val;
        s->sstat1 |= SSTAT1_OLF;
        break;
    case NCR710_SOCL_REG:
        s->socl = val;
        s->sbcl = val;
        break;
    case NCR710_SFBR_REG:
        s->sfbr = val;
        break;

    case NCR710_DSA_REG:
        s->dsa = (s->dsa & 0xFFFFFF00) | (val & 0xFF);
        break;
    case NCR710_DSA_REG + 1:
        s->dsa = (s->dsa & 0xFFFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_DSA_REG + 2:
        s->dsa = (s->dsa & 0xFF00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_DSA_REG + 3:
        s->dsa = (s->dsa & 0x00FFFFFF) | ((val & 0xFF) << 24);
        break;

    case NCR710_CTEST0_REG:
        s->ctest0 = val;
        if (val & 0x01) { /* EAN bit */
            s->tolerant_enabled = true;
        } else {
            s->tolerant_enabled = false;
        }
        break;
    case NCR710_CTEST1_REG:
        s->ctest1 = val;
        break;
    case NCR710_CTEST2_REG:
        s->ctest2 = val;
        break;
    case NCR710_CTEST3_REG:
        s->ctest3 = val;
        if (!ncr710_scsi_fifo_full(&s->scsi_fifo)) {
            uint8_t parity = ncr710_calculate_parity(val);
            ncr710_scsi_fifo_push(&s->scsi_fifo, val, parity);
        }
        break;
    case NCR710_CTEST4_REG:
        s->ctest4 = val;
        break;
    case NCR710_CTEST5_REG:
        s->ctest5 = val;
        break;
    case NCR710_CTEST6_REG:
        s->ctest6 = val;
        if (!ncr710_dma_fifo_full(&s->dma_fifo)) {
            uint8_t parity = (s->ctest7 & 0x08) ? 1 : 0; /* DFP bit */
            ncr710_dma_fifo_push(&s->dma_fifo, val, parity);
        }
        break;
    case NCR710_CTEST7_REG:
        s->ctest7 = val;
        if (val & 0x01) { /* DIFF bit */
            s->differential_mode = true;
        } else {
            s->differential_mode = false;
        }
        s->cache_line_burst = !(val & 0x80); /* CDIS bit inverted */
        break;

    case NCR710_TEMP_REG:
        s->temp = (s->temp & 0xFFFFFF00) | (val & 0xFF);
        break;
    case NCR710_TEMP_REG + 1:
        s->temp = (s->temp & 0xFFFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_TEMP_REG + 2:
        s->temp = (s->temp & 0xFF00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_TEMP_REG + 3:
        s->temp = (s->temp & 0x00FFFFFF) | ((val & 0xFF) << 24);
        break;

    case NCR710_ISTAT_REG:
        if (val & ISTAT_ABRT) {
            s->scripts.running = false;
            s->dstat |= DSTAT_ABRT;
            s->istat |= ISTAT_DIP;
            timer_del(s->selection_timer);
            timer_del(s->watchdog_timer);
            ncr710_update_irq(s);
        }
        if (val & ISTAT_RST) {
            uint8_t saved_ctest8 = s->ctest8;
            memset(s, 0, sizeof(NCR710State));
            s->ctest8 = saved_ctest8;  /* Preserve chip revision */
            s->dstat = DSTAT_DFE;
            ncr710_dma_fifo_init(&s->dma_fifo);
            ncr710_scsi_fifo_init(&s->scsi_fifo);
        }
        s->istat = (s->istat & 0x0F) | (val & 0xF0);
        break;

    case NCR710_CTEST8_REG:
        s->ctest8 = (s->ctest8 & 0xF0) | (val & 0x0F);
        if (val & 0x04) { /* FLF bit */
            ncr710_dma_fifo_flush(&s->dma_fifo);
        }
        break;
    case NCR710_LCRC_REG:
        s->lcrc = val;
        break;

    case NCR710_DBC_REG:
        s->dbc = (s->dbc & 0xFFFF00) | (val & 0xFF);
        break;
    case NCR710_DBC_REG + 1:
        s->dbc = (s->dbc & 0xFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_DBC_REG + 2:
        s->dbc = (s->dbc & 0x00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_DCMD_REG:
        s->dcmd = val;
        break;

    case NCR710_DNAD_REG:
        s->dnad = (s->dnad & 0xFFFFFF00) | (val & 0xFF);
        break;
    case NCR710_DNAD_REG + 1:
        s->dnad = (s->dnad & 0xFFFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_DNAD_REG + 2:
        s->dnad = (s->dnad & 0xFF00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_DNAD_REG + 3:
        s->dnad = (s->dnad & 0x00FFFFFF) | ((val & 0xFF) << 24);
        break;

    case NCR710_DSP_REG:
        s->dsp = (s->dsp & 0xFFFFFF00) | (val & 0xFF);
        break;
    case NCR710_DSP_REG + 1:
        s->dsp = (s->dsp & 0xFFFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_DSP_REG + 2:
        s->dsp = (s->dsp & 0xFF00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_DSP_REG + 3:
        s->dsp = (s->dsp & 0x00FFFFFF) | ((val & 0xFF) << 24);
        s->scripts.running = true;
        s->scripts.pc = s->dsp;
        ncr710_scripts_execute(s);
        break;

    case NCR710_DSPS_REG:
        s->dsps = (s->dsps & 0xFFFFFF00) | (val & 0xFF);
        break;
    case NCR710_DSPS_REG + 1:
        s->dsps = (s->dsps & 0xFFFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_DSPS_REG + 2:
        s->dsps = (s->dsps & 0xFF00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_DSPS_REG + 3:
        s->dsps = (s->dsps & 0x00FFFFFF) | ((val & 0xFF) << 24);
        break;

    case NCR710_SCRATCH_REG:
        s->scratch = (s->scratch & 0xFFFFFF00) | (val & 0xFF);
        break;
    case NCR710_SCRATCH_REG + 1:
        s->scratch = (s->scratch & 0xFFFF00FF) | ((val & 0xFF) << 8);
        break;
    case NCR710_SCRATCH_REG + 2:
        s->scratch = (s->scratch & 0xFF00FFFF) | ((val & 0xFF) << 16);
        break;
    case NCR710_SCRATCH_REG + 3:
        s->scratch = (s->scratch & 0x00FFFFFF) | ((val & 0xFF) << 24);
        break;

    case NCR710_DMODE_REG:
        s->dmode = val;
        switch (val & DMODE_BL_MASK) {
        case 0x00: s->burst_length = 1; break;
        case 0x40: s->burst_length = 2; break;
        case 0x80: s->burst_length = 4; break;
        case 0xC0: s->burst_length = 8; break;
        }
        break;
    case NCR710_DIEN_REG:
        s->dien = val;
        ncr710_update_irq(s);
        break;
    case NCR710_DWT_REG:
        s->dwt = val;
        break;
    case NCR710_DCNTL_REG:
        s->dcntl = val;
        if (val & DCNTL_STD) {
            ncr710_dma_transfer(s);
        }
        break;

    default:
        trace_ncr710_reg_write_unhandled(addr, val, size);
        qemu_log_mask(LOG_GUEST_ERROR,
                      "NCR710: invalid write at offset 0x%x\n", (int)addr);
        break;
    }
}

/* FIFO Implementation (DMA and SCSI)
 */
static void ncr710_dma_fifo_init(NCR710_DMA_FIFO *fifo)
{
    trace_ncr710_dma_fifo_init();
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
    memset(fifo->data, 0, sizeof(fifo->data));
    memset(fifo->parity, 0, sizeof(fifo->parity));
}

static bool ncr710_dma_fifo_empty(NCR710_DMA_FIFO *fifo)
{
    return fifo->count == 0;
}

static bool ncr710_dma_fifo_full(NCR710_DMA_FIFO *fifo)
{
    return fifo->count >= NCR710_DMA_FIFO_SIZE;
}

static void ncr710_dma_fifo_push(NCR710_DMA_FIFO *fifo, uint8_t data, uint8_t parity)
{
    if (!ncr710_dma_fifo_full(fifo)) {
        fifo->data[fifo->head] = data;
        fifo->parity[fifo->head] = parity;
        fifo->head = (fifo->head + 1) % NCR710_DMA_FIFO_SIZE;
        fifo->count++;
        trace_ncr710_dma_fifo_push(data, parity, fifo->count);
    } else {
        trace_ncr710_dma_fifo_overflow();
    }
}

static uint8_t ncr710_dma_fifo_pop(NCR710_DMA_FIFO *fifo, uint8_t *parity)
{
    uint8_t data = 0;
    uint8_t parity_val = 0;

    if (!ncr710_dma_fifo_empty(fifo)) {
        data = fifo->data[fifo->tail];
        if (parity) {
            *parity = fifo->parity[fifo->tail];
            parity_val = *parity;
        }
        fifo->tail = (fifo->tail + 1) % NCR710_DMA_FIFO_SIZE;
        fifo->count--;
        trace_ncr710_dma_fifo_pop(data, parity_val, fifo->count);
    } else {
        trace_ncr710_dma_fifo_underflow();
    }

    return data;
}

static void ncr710_dma_fifo_flush(NCR710_DMA_FIFO *fifo)
{
    int old_count = fifo->count;
    ncr710_dma_fifo_init(fifo);
    trace_ncr710_dma_fifo_flush(old_count);
}

static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo)
{
    trace_ncr710_scsi_fifo_init();
    fifo->count = 0;
    memset(fifo->data, 0, sizeof(fifo->data));
    memset(fifo->parity, 0, sizeof(fifo->parity));
}

static bool ncr710_scsi_fifo_empty(NCR710_SCSI_FIFO *fifo)
{
    return fifo->count == 0;
}

static bool ncr710_scsi_fifo_full(NCR710_SCSI_FIFO *fifo)
{
    return fifo->count >= NCR710_SCSI_FIFO_SIZE;
}

static void ncr710_scsi_fifo_push(NCR710_SCSI_FIFO *fifo, uint8_t data, uint8_t parity)
{
    if (!ncr710_scsi_fifo_full(fifo)) {
        fifo->data[fifo->count] = data;
        fifo->parity[fifo->count] = parity;
        fifo->count++;
        trace_ncr710_scsi_fifo_push(data, parity, fifo->count);
    } else {
        trace_ncr710_scsi_fifo_overflow();
    }
}

static uint8_t ncr710_scsi_fifo_pop(NCR710_SCSI_FIFO *fifo, uint8_t *parity)
{
    uint8_t data = 0;
    uint8_t parity_val = 0;

    if (!ncr710_scsi_fifo_empty(fifo)) {
        data = fifo->data[0];
        if (parity) {
            *parity = fifo->parity[0];
            parity_val = *parity;
        }

        /* Shift remaining data */
        for (int i = 0; i < fifo->count - 1; i++) {
            fifo->data[i] = fifo->data[i + 1];
            fifo->parity[i] = fifo->parity[i + 1];
        }
        fifo->count--;
        trace_ncr710_scsi_fifo_pop(data, parity_val, fifo->count);
    } else {
        trace_ncr710_scsi_fifo_underflow();
    }

    return data;
}


/* Memory access helpers */
static uint32_t ncr710_read_memory_32(NCR710State *s, uint32_t addr)
{
    uint32_t value = 0;

    /* Check if address space is initialized */
    if (!s->as) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "NCR710: address space not initialized, using fallback\n");
        s->as = &address_space_memory;
    }

    if (address_space_read(s->as, addr, MEMTXATTRS_UNSPECIFIED,
                          (uint8_t *)&value, 4) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "NCR710: failed to read memory at 0x%08x\n", addr);
        return 0;
    }

    value = le32_to_cpu(value);
    trace_ncr710_read_memory(addr, value);
    return value;
}


static uint32_t ncr710_calculate_parity(uint8_t data)
{
    uint32_t parity = 0;
    for (int i = 0; i < 8; i++) {
        if (data & (1 << i)) {
            parity++;
        }
    }

    return parity & 1;
}

static void ncr710_update_irq(NCR710State *s)
{
    bool irq_active = false;

    if (s->istat & (ISTAT_SIP | ISTAT_DIP)) {
        irq_active = true;
        trace_ncr710_irq_raise(s->sstat0, s->dstat);
    } else {
        trace_ncr710_irq_lower();
    }

    if (s->irq) {
        qemu_set_irq(s->irq, irq_active ? 1 : 0);
    }

    trace_ncr710_irq_update(irq_active ? 1 : 0);
}

/* We could have different storage devices right so all will be children?*/
static void ncr710_arbitrate_bus(NCR710State *s)
{
    /* Basic bus arbitration implementation */
    if (s->scntl0 & SCNTL0_START) {
        /* Start sequence initiated */
        trace_ncr710_arbitrate_start();
        s->scripts.connected = false;
        s->scripts.initiator = true;
        trace_ncr710_connected(false);

        /* Set arbitration in progress */
        s->sstat1 |= SSTAT1_AIP;

        /* For now, assume we always win arbitration */
        s->sstat1 |= SSTAT1_WOA;
        s->sstat1 &= ~SSTAT1_LOA;
        s->sstat1 &= ~SSTAT1_AIP;
        trace_ncr710_arbitrate_won(s->scid & 0x07);

        if (s->sdid != 0) {
            uint8_t target_id = 0;
            for (int i = 0; i < 8; i++) {
                if (s->sdid & (1 << i)) {
                    target_id = i;
                    break;
                }
            }

            trace_ncr710_selection_start(target_id);
            qemu_log("NCR710: Selecting SCSI target %d\n", target_id);
            qemu_log("NCR710: Using SCSI bus: %p\n", &s->bus);

            BusChild *kid;
            qemu_log("NCR710: Scanning all devices on SCSI bus:\n");
            qemu_log("NCR710: Bus children queue first: %p\n", QTAILQ_FIRST(&s->bus.qbus.children));

            int device_count = 0;
            RCU_READ_LOCK_GUARD();
            QTAILQ_FOREACH_RCU(kid, &s->bus.qbus.children, sibling) {
                device_count++;
                SCSIDevice *scsi_dev = SCSI_DEVICE(kid->child);
                qemu_log("NCR710: Found device #%d: channel=%d target=%d lun=%d type=%s realized=%s\n",
                        device_count, scsi_dev->channel, scsi_dev->id, scsi_dev->lun,
                        object_get_typename(OBJECT(scsi_dev)),
                        qdev_is_realized(&scsi_dev->qdev) ? "yes" : "no");
            }
            qemu_log("NCR710: Total devices found: %d\n", device_count);

            SCSIDevice *scsi_dev = scsi_device_find(&s->bus, 0, target_id, 0);
            if (scsi_dev) {
                qemu_log("NCR710: Found SCSI device at target %d\n", target_id);
                s->sstat0 |= SSTAT0_SEL;  /* Selected */
                s->scntl1 |= SCNTL1_CON;  /* Connected */
                s->scripts.connected = true;
                trace_ncr710_connected(true);
                trace_ncr710_selection_success(target_id);
            } else {
                qemu_log("NCR710: No SCSI device found at target %d\n", target_id);
                s->sstat0 |= SSTAT0_STO;  /* Selection timeout */
                trace_ncr710_selection_timeout(target_id);
            }
        }

        s->scntl0 &= ~SCNTL0_START;

        ncr710_update_irq(s);
    }
}

static void ncr710_internal_scsi_bus_reset(NCR710State *s)
{
    trace_ncr710_bus_reset();
    qemu_log("NCR710: Internal SCSI bus reset called\n");

    s->sstat0 = 0;
    s->sstat1 = 0;
    s->sstat2 = 0;
    s->dstat = DSTAT_DFE;  /* DMA FIFO Empty */
    s->scntl0 &= ~(SCNTL0_START | SCNTL0_WATN);
    s->scntl1 &= ~(SCNTL1_CON | SCNTL1_RST);
    s->socl = 0;
    s->sbcl = 0;
    s->scripts.connected = false;
    s->scripts.initiator = false;
    s->scripts.running = false;
    s->scripts.phase = SCSI_PHASE_DATA_OUT;
    trace_ncr710_connected(false);
    ncr710_dma_fifo_flush(&s->dma_fifo);
    ncr710_scsi_fifo_init(&s->scsi_fifo);
    s->istat &= ~(ISTAT_SIP | ISTAT_DIP | ISTAT_CON);
    s->sstat0 |= SSTAT0_RST;
    if (s->sien & SIEN_RST) {
        s->istat |= ISTAT_SIP;
    }
    ncr710_update_irq(s);
}

static void ncr710_device_reset(DeviceState *dev)
{
    SysBusNCR710State *sysbus_s = SYSBUS_NCR710_SCSI(dev);
    NCR710State *s = &sysbus_s->ncr710;

    trace_ncr710_device_reset();
    qemu_log("NCR710: Device reset function called\n");

    BusChild *kid;
    int device_count = 0;
    qemu_log("NCR710: BEFORE NCR710 device reset - devices on bus:\n");
    QTAILQ_FOREACH(kid, &s->bus.qbus.children, sibling) {
        device_count++;
        SCSIDevice *scsi_dev = SCSI_DEVICE(kid->child);
        qemu_log("NCR710: Device #%d: channel=%d target=%d lun=%d type=%s\n",
                device_count, scsi_dev->channel, scsi_dev->id, scsi_dev->lun,
                object_get_typename(OBJECT(scsi_dev)));
    }
    qemu_log("NCR710: Total devices BEFORE NCR710 reset: %d\n", device_count);

    ncr710_internal_scsi_bus_reset(s);

    device_count = 0;
    qemu_log("NCR710: AFTER NCR710 device reset - devices on bus:\n");
    QTAILQ_FOREACH(kid, &s->bus.qbus.children, sibling) {
        device_count++;
        SCSIDevice *scsi_dev = SCSI_DEVICE(kid->child);
        qemu_log("NCR710: Device #%d: channel=%d target=%d lun=%d type=%s\n",
                device_count, scsi_dev->channel, scsi_dev->id, scsi_dev->lun,
                object_get_typename(OBJECT(scsi_dev)));
    }
    qemu_log("NCR710: Total devices AFTER NCR710 reset: %d\n", device_count);
}

static void ncr710_scripts_execute(NCR710State *s)
{
    uint32_t instruction;
    uint32_t address;
    uint32_t opcode;
    uint32_t count;

    if (!s->scripts.running) {
        trace_ncr710_scripts_stop(s->dsp, "not running");
        return;
    }

    if (s->dsp == 0) {
        trace_ncr710_scripts_illegal_instruction(s->dsp, 0);
        s->dstat |= DSTAT_IID;  /* Illegal Instruction Detected */
        if (s->dien & DIEN_IID) {
            s->istat |= ISTAT_DIP;
        }
        s->scripts.running = false;
        ncr710_update_irq(s);
        return;
    }

    instruction = ncr710_read_memory_32(s, s->dsp);
    if (instruction == 0) {
        trace_ncr710_scripts_illegal_instruction(s->dsp, instruction);
        s->dstat |= DSTAT_IID;  /* Illegal Instruction Detected */
        if (s->dien & DIEN_IID) {
            s->istat |= ISTAT_DIP;
        }
        s->scripts.running = false;
        ncr710_update_irq(s);
        return;
    }

    address = ncr710_read_memory_32(s, s->dsp + 4);

    opcode = instruction & SCRIPTS_TYPE_MASK;
    count = instruction & SCRIPTS_BM_COUNT_MASK;

    trace_ncr710_scripts_execute(s->dsp, instruction);

    switch (opcode) {
    case SCRIPTS_TYPE_BLOCK_MOVE:
        if (instruction & SCRIPTS_BM_INDIRECT) {
            address = ncr710_read_memory_32(s, address);
        }

        s->dbc = count;
        s->dnad = address;
        s->dcmd = (instruction >> 24) & 0xFF;

        /* Advance DSP to next instruction */
        s->dsp += 8;

        /* Start DMA transfer */
        ncr710_dma_transfer(s);
        break;

    case SCRIPTS_TYPE_IO:
        /* I/O instruction (Select, Reselect, Wait, Set, Clear) */
        /* For now, just advance to next instruction */
        s->dsp += 8;
        trace_ncr710_scripts_io_instruction(instruction, "I/O operation");
        break;

    case SCRIPTS_TYPE_READ_WRITE:
        /* Read/Write instruction (not commonly used) */
        s->dsp += 8;
        trace_ncr710_scripts_instruction(s->dsp, instruction, "READ/WRITE");
        break;

    case SCRIPTS_TYPE_TRANSFER:
        /* Transfer Control instruction (Jump, Call, Return, Interrupt) */
        {
            uint32_t tc_opcode = instruction & SCRIPTS_TC_OPCODE_MASK;

            switch (tc_opcode) {
            case SCRIPTS_TC_JUMP:
                /* Unconditional jump */
                trace_ncr710_scripts_jump(s->dsp, address);
                s->dsp = address;
                break;

            case SCRIPTS_TC_CALL:
                /* Call subroutine - save return address */
                trace_ncr710_scripts_call(s->dsp, address);
                s->temp = s->dsp + 8;
                s->dsp = address;
                break;

            case SCRIPTS_TC_RETURN:
                /* Return from subroutine */
                trace_ncr710_scripts_return(s->temp);
                s->dsp = s->temp;
                break;

            case SCRIPTS_TC_INT:
                /* SCRIPTS interrupt */
                trace_ncr710_scripts_interrupt(address, address);
                s->dsps = address;  /* Save interrupt vector */
                s->dstat |= DSTAT_SIR;  /* SCRIPTS Interrupt */
                if (s->dien & DIEN_SIR) {
                    s->istat |= ISTAT_DIP;
                }
                s->scripts.running = false;
                ncr710_update_irq(s);
                break;

            default:
                /* Unknown transfer control instruction */
                trace_ncr710_scripts_illegal_instruction(s->dsp, instruction);
                s->dstat |= DSTAT_IID;
                if (s->dien & DIEN_IID) {
                    s->istat |= ISTAT_DIP;
                }
                s->scripts.running = false;
                ncr710_update_irq(s);
                break;
            }
        }
        break;

    default:
        /* Unknown instruction type */
        trace_ncr710_scripts_illegal_instruction(s->dsp, instruction);
        s->dstat |= DSTAT_IID;  /* Illegal Instruction Detected */
        if (s->dien & DIEN_IID) {
            s->istat |= ISTAT_DIP;
        }
        s->scripts.running = false;
        ncr710_update_irq(s);
        break;
    }

    /* Continue execution if still running */
    if (s->scripts.running) {
        /* In a real implementation, we might want to limit the number of
         * instructions executed per call to avoid blocking */
        ncr710_scripts_execute(s);
    }
}

static void ncr710_dma_transfer(NCR710State *s)
{
    /* Basic DMA transfer implementation */
    if (!s->dbc || !s->dnad) {
        /* No transfer to perform */
        s->dstat |= DSTAT_DFE;  /* DMA FIFO Empty */
        trace_ncr710_dma_abort("no data to transfer");
        return;
    }

    trace_ncr710_dma_start(s->dnad, s->dbc, s->scripts.phase);

    /* For now, just mark DMA as completed */
    uint32_t transferred = s->dbc;
    s->dbc = 0;  /* Clear byte counter */
    s->dstat |= DSTAT_DFE;  /* Set DMA FIFO Empty */

    trace_ncr710_dma_complete(transferred);

    /* Update interrupt status if enabled */
    ncr710_update_irq(s);
}

/* SCSI command initiation - TODO: integrate with SCRIPTS processor */
#if 0
static void ncr710_scsi_command_start(NCR710State *s, uint8_t target, uint8_t lun, uint8_t *cdb, uint32_t cdb_len)
{
    SCSIDevice *d;
    SCSIRequest *req;

    qemu_log("NCR710: Starting SCSI command - target=%d, lun=%d, opcode=0x%02x\n", 
             target, lun, cdb[0]);

    /* Find the SCSI device */
    d = scsi_device_find(&s->bus, 0, target, lun);
    if (!d) {
        qemu_log("NCR710: No device found at target %d, lun %d\n", target, lun);
        /* Set selection timeout */
        s->sstat0 |= SSTAT0_STO;  /* Selection Timeout */
        if (s->sien & SIEN_STO) {
            s->istat |= ISTAT_SIP;
        }
        ncr710_update_irq(s);
        return;
    }

    /* Create and start the SCSI request */
    req = scsi_req_new(d, 0, lun, cdb, cdb_len, s);
    if (!req) {
        qemu_log("NCR710: Failed to create SCSI request\n");
        return;
    }

    s->current_req = req;
    s->scripts.connected = true;
    
    /* Start the command */
    scsi_req_enqueue(req);
    qemu_log("NCR710: SCSI command enqueued successfully\n");
}
#endif

/* SCSI bus callback functions */
static void ncr710_transfer_data(SCSIRequest *req, uint32_t len)
{
    trace_ncr710_transfer_data_start(req, len);
    qemu_log("NCR710: Transfer data request - len=%d, direction=%s\n",
             len, req->cmd.mode == SCSI_XFER_FROM_DEV ? "READ" : "WRITE");

    /* For now, just complete the transfer immediately */
    /* In a full implementation, this would trigger DMA operations */
    if (req->cmd.mode == SCSI_XFER_FROM_DEV) {
        /* Read operation - provide data to guest */
        scsi_req_data(req, len);
    } else {
        /* Write operation - accept data from guest */
        scsi_req_data(req, len);
    }

    trace_ncr710_transfer_data_complete(req, len);
}

static void ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    SysBusNCR710State *sysbus_s = container_of(req->bus, SysBusNCR710State, ncr710.bus);
    NCR710State *s = &sysbus_s->ncr710;

    trace_ncr710_command_complete(req, req->status, resid);
    qemu_log("NCR710: Command complete - status=0x%02x, resid=%zu\n", req->status, resid);

    s->sstat0 |= SSTAT0_FCMP;  /* Function Complete */

    /* Signal interrupt if enabled */
    if (s->sien & SIEN_FCMP) {
        s->istat |= ISTAT_SIP;  /* SCSI Interrupt Pending */
        ncr710_update_irq(s);
    }

    scsi_req_unref(req);
}

static void ncr710_request_cancelled(SCSIRequest *req)
{
    SysBusNCR710State *sysbus_s = container_of(req->bus, SysBusNCR710State, ncr710.bus);
    NCR710State *s = &sysbus_s->ncr710;

    trace_ncr710_request_cancelled(req);
    qemu_log("NCR710: Request cancelled\n");

    /* Set abort status */
    s->dstat |= DSTAT_ABRT;  /* Aborted */

    /* Signal interrupt if enabled */
    if (s->dien & DIEN_ABRT) {
        s->istat |= ISTAT_DIP;  /* DMA Interrupt Pending */
        ncr710_update_irq(s);
    }

    scsi_req_unref(req);
}

static void ncr710_drained_begin(SCSIBus *bus)
{
    /* Stop submitting new requests during drain */
    trace_ncr710_drained_begin();
}

static void ncr710_drained_end(SCSIBus *bus)
{
    /* Resume normal operation after drain */
    trace_ncr710_drained_end();
}


/* QEMU Object Model Registration
 * SysBus NCR710 device
 */

/* Minimal device creation function for sysbus */
DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *sysbus;

    trace_ncr710_create_sysbus(addr);
    dev = qdev_new(TYPE_SYSBUS_NCR710_SCSI);
    sysbus = SYS_BUS_DEVICE(dev);

    qdev_realize_and_unref(dev, NULL, &error_abort);
    sysbus_mmio_map(sysbus, 0, addr);
    sysbus_connect_irq(sysbus, 0, irq);

    return dev;
}

/* Handle legacy command line SCSI drives */
void ncr710_handle_legacy_cmdline(DeviceState *ncr_dev)
{
    SysBusNCR710State *sysbus_s = SYSBUS_NCR710_SCSI(ncr_dev);
    NCR710State *s = &sysbus_s->ncr710;

    trace_ncr710_handle_legacy_cmdline();
    qemu_log("NCR710: Handling legacy command line directly\n");
    qemu_log("NCR710: Using bus: %p\n", &s->bus);

    scsi_bus_legacy_handle_cmdline(&s->bus);

    /* Debug: Check what devices are now on the bus */
    BusChild *kid;
    int device_count = 0;
    QTAILQ_FOREACH(kid, &s->bus.qbus.children, sibling) {
        device_count++;
        SCSIDevice *scsi_dev = SCSI_DEVICE(kid->child);
        qemu_log("NCR710: Legacy created device #%d: channel=%d target=%d lun=%d type=%s\n",
                device_count, scsi_dev->channel, scsi_dev->id, scsi_dev->lun,
                object_get_typename(OBJECT(scsi_dev)));
    }
    qemu_log("NCR710: Legacy handler created %d devices\n", device_count);
}

/* Main initialization function expected by HPPA machine */
DeviceState *ncr53c710_init(MemoryRegion *address_space, hwaddr addr, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *sysbus;
    SysBusNCR710State *s;

    trace_ncr710_device_init(addr);
    qemu_log("NCR710: Initializing device at 0x%08lx\n", addr);

    dev = qdev_new(TYPE_SYSBUS_NCR710_SCSI);
    sysbus = SYS_BUS_DEVICE(dev);

    qdev_realize_and_unref(dev, NULL, &error_abort);
    sysbus_mmio_map(sysbus, 0, addr);
    sysbus_connect_irq(sysbus, 0, irq);

    /* Ensure address space is initialized for legacy path */
    s = SYSBUS_NCR710_SCSI(dev);
    if (!s->ncr710.as) {
        s->ncr710.as = &address_space_memory;
        qemu_log("NCR710: Address space initialized in legacy init\n");
    }

    qemu_log("NCR710: Device mapped and IRQ connected\n");

    return dev;
}

static const struct SCSIBusInfo ncr710_scsi_info = {
    .tcq = false,     /* No tagged command queuing */
    .max_target = 8,  /* SCSI targets 0-7 */
    .max_lun = 8,     /* LUNs 0-7, buggy? */

    .transfer_data = ncr710_transfer_data,
    .complete = ncr710_command_complete,
    .cancel = ncr710_request_cancelled,
    .drained_begin = ncr710_drained_begin,
    .drained_end = ncr710_drained_end,
};

/* Memory region operations */
static const MemoryRegionOps ncr710_mmio_ops = {
    .read = ncr710_reg_read,
    .write = ncr710_reg_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .impl = {
        .min_access_size = 1,
        .max_access_size = 1,
    },
};

static void sysbus_ncr710_realize(DeviceState *dev, Error **errp)
{
    SysBusNCR710State *s = SYSBUS_NCR710_SCSI(dev);

    trace_ncr710_device_realize();
    qemu_log("NCR710: Realize function called\n");

    /* Initialize SCSI bus with default naming */
    scsi_bus_init(&s->ncr710.bus, sizeof(s->ncr710.bus), dev, &ncr710_scsi_info);

    /* Initialize address space for memory access */
    s->ncr710.as = &address_space_memory;

    /* Initialize FIFOs */
    ncr710_dma_fifo_init(&s->ncr710.dma_fifo);
    ncr710_scsi_fifo_init(&s->ncr710.scsi_fifo);

    s->ncr710.selection_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, NULL, NULL);
    s->ncr710.watchdog_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, NULL, NULL);

    /* Initialize default register values that Linux driver expects */
    s->ncr710.ctest8 = NCR710_REVISION_2;  /* NCR710 chip revision 2 */
    s->ncr710.scid = 0x80 | NCR710_DEFAULT_HOST_ID; /* Valid bit + SCSI ID 7 */
    s->ncr710.dstat = DSTAT_DFE;  /* DMA FIFO Empty initially */
    qemu_log("NCR710: Initialized DSTAT to 0x%02x\n", s->ncr710.dstat);

    /* Initialize critical control registers */
    s->ncr710.dcntl = DCNTL_EA;    /* Enable Ack bit for 710 */
    s->ncr710.dmode = DMODE_FC2;   /* Flow Control 2 for 710 */
    s->ncr710.ctest7 = CTEST7_TT1; /* Transfer Type 1 for 710 */

    /* Initialize SCRIPTS context */
    s->ncr710.scripts.running = false;
    s->ncr710.scripts.connected = false;
    s->ncr710.scripts.initiator = false;
    s->ncr710.scripts.phase = SCSI_PHASE_DATA_OUT;
    s->ncr710.scripts.pc = 0;
    s->ncr710.scripts.saved_pc = 0;

    /* Device configuration */
    s->ncr710.selection_timeout_enabled = true;
    s->ncr710.big_endian = true;
    s->ncr710.burst_length = 1;
    s->ncr710.tolerant_enabled = true;  /* Enable TolerANT active negation */

    /* Clear interrupt states */
    s->ncr710.istat = 0;
    s->ncr710.sstat0 = 0;
    s->ncr710.sstat1 = 0;
    s->ncr710.sstat2 = 0;

    /* Set up memory region */
    memory_region_init_io(&s->iomem, OBJECT(s), &ncr710_mmio_ops, &s->ncr710,
                         "ncr710", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->ncr710.irq);

    qemu_log("NCR710: Device realized with memory region and IRQ\n");
}

static void sysbus_ncr710_unrealize(DeviceState *dev)
{
    SysBusNCR710State *s = SYSBUS_NCR710_SCSI(dev);

    trace_ncr710_device_unrealize();
    qemu_log("NCR710: Device unrealize called\n");

    /* Clean up timers */
    if (s->ncr710.selection_timer) {
        timer_free(s->ncr710.selection_timer);
        s->ncr710.selection_timer = NULL;
    }
    if (s->ncr710.watchdog_timer) {
        timer_free(s->ncr710.watchdog_timer);
        s->ncr710.watchdog_timer = NULL;
    }

    qemu_log("NCR710: Device unrealize completed\n");
}

static void sysbus_ncr710_init(Object *obj)
{
    SysBusNCR710State *s = SYSBUS_NCR710_SCSI(obj);

    qemu_log("NCR710: Instance init called\n");

    /* Initialize the embedded NCR710 state directly */
    memset(&s->ncr710, 0, sizeof(NCR710State));

    /* Set default register values that identify this as an NCR53C710 */
    s->ncr710.ctest0 = 0x01;  /* Chip revision 1 */
    s->ncr710.scid = 0x80 | NCR710_DEFAULT_HOST_ID; /* Valid bit + SCSI ID 7 */
    s->ncr710.dstat = DSTAT_DFE;  /* DMA FIFO Empty */

    qemu_log("NCR710: State initialized\n");
}

static void sysbus_ncr710_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sysbus_ncr710_realize;
    dc->unrealize = sysbus_ncr710_unrealize;
    dc->desc = "NCR53C710 SCSI I/O Processor (SysBus)";
    device_class_set_legacy_reset(dc, ncr710_device_reset);
    dc->bus_type = NULL;  /* Explicitly set to NULL for sysbus devices */
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo sysbus_ncr710_info = {
    .name = TYPE_SYSBUS_NCR710_SCSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusNCR710State),
    .instance_init = sysbus_ncr710_init,
    .class_init = sysbus_ncr710_class_init,
};

/* Module initialization */
static void ncr710_register_types(void)
{
    type_register_static(&sysbus_ncr710_info);
}

type_init(ncr710_register_types)