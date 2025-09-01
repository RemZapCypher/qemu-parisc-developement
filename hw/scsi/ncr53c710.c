/*
 * LASI NCR710 SCSI I/O Processor
 *
 * Copyright (c) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 *
 * NCR710 SCSI I/O Processor implementation
 * Based on the NCR53C710 Technical Manual Version 3.2, December 2000
 *
 * Developed from the hackish implementation of NCR53C710 by Helge Deller
 * which was interm based on the hackish implementation by Toni Wilen for UAE
 * God, the linux kernel side is buggy for NCR710.
 * But workarounds work (probably, fingers crossed).
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
#include "qemu/bitops.h"
#include "qemu/rcu.h"
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

#define PHASE_DO          0
#define PHASE_DI          1
#define PHASE_CMD         2
#define PHASE_ST          3
#define PHASE_MO          6
#define PHASE_MI          7
#define PHASE_MASK        7

/* Maximum length of MSG IN data */
#define NCR710_MAX_MSGIN_LEN 8

/* Maximum SCSI CDB (Command Descriptor Block) size */
#define NCR710_MAX_CDB_SIZE  16

/* Flag set if this is a tagged command */
#define NCR710_TAG_VALID     (1 << 16)

/* SCSI message codes */
#define SCSI_MSG_COMMAND_COMPLETE    0x00
#define SCSI_MSG_EXTENDED_MESSAGE    0x01
#define SCSI_MSG_SAVE_DATA_POINTER   0x02
#define SCSI_MSG_RESTORE_POINTERS    0x03
#define SCSI_MSG_DISCONNECT          0x04
#define SCSI_MSG_INITIATOR_DET_ERROR 0x05
#define SCSI_MSG_ABORT               0x06
#define SCSI_MSG_MESSAGE_REJECT      0x07
#define SCSI_MSG_NOP                 0x08
#define SCSI_MSG_MESSAGE_PARITY_ERROR 0x09
#define SCSI_MSG_LINKED_CMD_COMPLETE 0x0A
#define SCSI_MSG_LINKED_FLG_CMD_COMPLETE 0x0B
#define SCSI_MSG_BUS_DEVICE_RESET    0x0C

/* Message action codes */
#define NCR710_MSG_ACTION_COMMAND    0
#define NCR710_MSG_ACTION_DISCONNECT 1
#define NCR710_MSG_ACTION_DOUT       2
#define NCR710_MSG_ACTION_DIN        3

#define NCR710_MAX_DEVS 7
#define NCR710_BUF_SIZE 4096

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



/* Function prototypes for implemented functions only */
/* 1. Core device functions */
static void ncr710_update_irq(NCR710State *s);
static void ncr710_arbitrate_bus(NCR710State *s);
static void ncr710_internal_scsi_bus_reset(NCR710State *s);
static void ncr710_device_reset(DeviceState *dev);
static void ncr710_scripts_execute(NCR710State *s);
static void ncr710_dma_transfer(NCR710State *s);

/* 2. Integrated functions from reference material */
static void ncr710_soft_reset(NCR710State *s);
static void ncr710_stop_script(NCR710State *s);
static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0);
static void ncr710_script_dma_interrupt(NCR710State *s, int stat);
static void ncr710_set_phase(NCR710State *s, int phase);
static void ncr710_bad_phase(NCR710State *s, int out, int new_phase);
static void ncr710_resume_script(NCR710State *s);
static void ncr710_disconnect(NCR710State *s);
static void ncr710_bad_selection(NCR710State *s, uint32_t id);
static void ncr710_do_dma(NCR710State *s, int out);
static void ncr710_add_msg_byte(NCR710State *s, uint8_t data);
static void ncr710_reselect(NCR710State *s, NCR710Request *p);
static NCR710Request *ncr710_find_by_tag(NCR710State *s, uint32_t tag);
static void ncr710_request_free(NCR710State *s, NCR710Request *p);
static int ncr710_queue_req(NCR710State *s, SCSIRequest *req, uint32_t len);
static void ncr710_do_command(NCR710State *s);
static void ncr710_do_status(NCR710State *s);
static void ncr710_do_msgin(NCR710State *s);
static void ncr710_do_msgout(NCR710State *s);
static uint8_t ncr710_get_msgbyte(NCR710State *s);
static void ncr710_skip_msgbytes(NCR710State *s, unsigned int n);
static void ncr710_memcpy(NCR710State *s, uint32_t dest, uint32_t src, int count);
static void ncr710_wait_reselect(NCR710State *s);
static void ncr710_handle_direct_scsi_command(NCR710State *s);

/* 3. FIFO management functions */
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

/* 4. Memory access helpers */
static uint32_t ncr710_read_memory_32(NCR710State *s, uint32_t addr);
static void ncr710_write_memory(NCR710State *s, uint32_t addr, const void *buf, int len);
static void ncr710_read_memory(NCR710State *s, uint32_t addr, void *buf, int len);
static uint32_t ncr710_calculate_parity(uint8_t data);
static int ncr710_idbitstonum(int id);
static inline int ncr710_irq_on_rsl(NCR710State *s);

/* 5. Register access functions */

// static uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size);
// static void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size);







static uint8_t ncr710_reg_readb(NCR710State *s, int offset);
static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val);

/* 6. SCSI bus callback functions */
static void ncr710_transfer_data(SCSIRequest *req, uint32_t len);
static void ncr710_command_complete(SCSIRequest *req, size_t resid);
static void ncr710_request_cancelled(SCSIRequest *req);

/* 7. Device initialization and class functions */
static void sysbus_ncr710_realize(DeviceState *dev, Error **errp);
static void sysbus_ncr710_init(Object *obj);
static void sysbus_ncr710_class_init(ObjectClass *oc, void *data);
static void ncr710_register_types(void);

// Diff TODO: investigate reselection interrupt handling difference
static inline int ncr710_irq_on_rsl(NCR710State *s)
{
    return 0; //return (s->sien0 & LSI_SIST0_RSL) && (s->scid & LSI_SCID_RRE);
}

bool ncr710_is_700_mode(NCR710State *s)
{
    return s->compatibility_mode;
}

static void ncr710_update_compatibility_mode(NCR710State *s)
{
    bool old_mode = s->compatibility_mode;
    s->compatibility_mode = (s->dcntl & DCNTL_COM) != 0;
    
    if (old_mode != s->compatibility_mode) {
        qemu_log("NCR710: Switching to %s compatibility mode\n", 
                 s->compatibility_mode ? "53C700" : "53C710");
    }
}

static void ncr710_soft_reset(NCR710State *s)
{
    trace_ncr710_device_reset();

    /* Clear script execution state */
    s->carry = 0;
    s->msg_action = NCR710_MSG_ACTION_COMMAND;
    s->msg_len = 0;
    s->waiting = 0;

    /* Reset DMA/SCRIPTS registers */
    s->dsa = 0;
    s->dnad = 0;
    s->dbc = 0;
    s->temp = 0;
    s->scratch = 0;
    s->dsp = 0;
    s->dsps = 0;
    s->dmode = 0;
    s->dcntl = 0;

    /* Preserve RST bit in ISTAT during soft reset */
    s->istat &= ISTAT_RST;
    s->dcmd = 0x40;
    s->dstat = DSTAT_DFE;  /* DMA FIFO empty after reset */
    s->dien = 0;
    s->sien = 0;

    /* Reset test registers */
    s->ctest2 = 0x01; /* DACK */
    s->ctest3 = 0;
    s->ctest4 = 0;
    s->ctest5 = 0;

    /* Reset SCSI control and status registers */
    s->scntl0 = 0xc0;  /* Default arbitration mode */
    s->scntl1 = 0;
    s->sstat0 = 0;
    s->sstat1 = 0;
    s->sstat2 = 0;
    s->scid = 0x80;    /* Default host adapter ID = 7 */
    s->sxfer = 0;
    s->socl = 0;
    s->sdid = 0;
    s->sidl = 0;
    s->sbdl = 0;
    s->sfbr = 0;
    s->sodl = 0;
    s->lcrc = 0;

    /* Clear command state */
    s->current_lun = 0;
    s->command_complete = 0;
    s->status = 0;
    s->select_tag = 0;

    /* Ensure no active requests */
    assert(QTAILQ_EMPTY(&s->queue));
    assert(!s->current);

    /* Reset SCRIPTS execution context */
    s->scripts.running = false;
    s->scripts.connected = false;
    s->scripts.initiator = false;
    s->scripts.phase = SCSI_PHASE_DATA_OUT;
    s->scripts.pc = 0;
    s->scripts.saved_pc = 0;
    s->script_active = 0;
    s->compatibility_mode = false;  /* Start in 710 mode by default */

    /* Reset FIFOs */
    ncr710_dma_fifo_init(&s->dma_fifo);
    ncr710_scsi_fifo_init(&s->scsi_fifo);
}

static void ncr710_stop_script(NCR710State *s)
{
    s->script_active = 0;
    s->scripts.running = false;
}

static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0)
{
    uint32_t mask0;

    NCR710_DPRINTF("SCSI Interrupt 0x%02x prev 0x%02x\n", stat0, s->sstat0);
    s->sstat0 |= stat0;

    /* Stop processor on fatal or unmasked interrupt. As a special hack
       we don't stop processing when raising STO. Instead continue
       execution and stop at the next insn that accesses the SCSI bus. */
    mask0 = s->sien | ~(SSTAT0_FCMP | SSTAT0_SEL);
    if (s->sstat0 & mask0) {
        ncr710_stop_script(s);
    }
    ncr710_update_irq(s);
}

static void ncr710_script_dma_interrupt(NCR710State *s, int stat)
{
    NCR710_DPRINTF("DMA Interrupt 0x%x prev 0x%x\n", stat, s->dstat);
    s->dstat |= stat;
    ncr710_update_irq(s);
    ncr710_stop_script(s);
}

static inline void ncr710_set_phase(NCR710State *s, int phase)
{
    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | phase;
    s->ctest0 &= ~1;
    if (phase == PHASE_DI) {
        s->ctest0 |= 1;
    }
    s->sbcl &= ~SBCL_REQ;
}

static void ncr710_bad_phase(NCR710State *s, int out, int new_phase)
{
    /* Trigger a phase mismatch */
    NCR710_DPRINTF("Phase mismatch interrupt\n");
    ncr710_script_scsi_interrupt(s, SSTAT0_M_A);
    ncr710_stop_script(s);
    ncr710_set_phase(s, new_phase);
    s->sbcl |= SBCL_REQ;
}

static void ncr710_resume_script(NCR710State *s)
{
    if (s->waiting != 2) {
        s->waiting = 0;
        ncr710_scripts_execute(s);
    } else {
        s->waiting = 0;
    }
}

static void ncr710_disconnect(NCR710State *s)
{
    s->scntl1 &= ~SCNTL1_CON;
    s->sstat2 &= ~PHASE_MASK;
    s->scripts.connected = false;
}

static void ncr710_bad_selection(NCR710State *s, uint32_t id)
{
    NCR710_DPRINTF("Selected absent target %d\n", id);
    ncr710_script_scsi_interrupt(s, SSTAT0_STO);
    ncr710_disconnect(s);
}

static int ncr710_idbitstonum(int id)
{
    int num = 0;
    while (id > 1) {
        num++;
        id >>= 1;
    }
    if (num > 7) {
        num = -1;
    }
    return num;
}

static void ncr710_write_memory(NCR710State *s, uint32_t addr, const void *buf, int len)
{
    if (!s->as) {
        s->as = &address_space_memory;
    }

    /* Log any memory writes, especially potential SCRIPTS uploads */
    if (len >= 4) {
        const uint32_t *words = (const uint32_t *)buf;
        qemu_log("NCR710: Memory write addr=0x%08x len=%d data=0x%08x",
                 addr, len, words[0]);
        printf("=== NCR710: Memory write addr=0x%08x len=%d data=0x%08x",
               addr, len, words[0]);
        if (len >= 8) {
            qemu_log(" 0x%08x", words[1]);
            printf(" 0x%08x", words[1]);
        }
        qemu_log("\n");
        printf(" ===\n");
    }

    if (address_space_write(s->as, addr, MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "NCR710: DMA write failed at 0x%08x\n", addr);
    }
}

static void ncr710_read_memory(NCR710State *s, uint32_t addr, void *buf, int len)
{
    if (!s->as) {
        s->as = &address_space_memory;
    }

    if (address_space_read(s->as, addr, MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "NCR710: DMA read failed at 0x%08x\n", addr);
        memset(buf, 0, len);
    }
}

/* Request management functions */

static NCR710Request *ncr710_find_by_tag(NCR710State *s, uint32_t tag)
{
    NCR710Request *p;

    QTAILQ_FOREACH(p, &s->queue, next) {
        if (p->tag == tag) {
            return p;
        }
    }
    return NULL;
}

static void ncr710_request_free(NCR710State *s, NCR710Request *p)
{
    if (p == s->current) {
        s->current = NULL;
    } else {
        QTAILQ_REMOVE(&s->queue, p, next);
    }
    g_free(p);
}

static int ncr710_queue_req(NCR710State *s, SCSIRequest *req, uint32_t len)
{
    NCR710Request *p = (NCR710Request*)req->hba_private;

    if (p->pending) {
        qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Multiple IO pending for request %p\n", p);
    }
    p->pending = len;

    /* Reselect if waiting for it, or if reselection triggers an IRQ
       and the bus is free. */
    if (s->waiting == 1 ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & SCNTL1_CON) &&
         !(s->istat & (ISTAT_SIP | ISTAT_DIP)))) {
        /* Reselect device */
        ncr710_reselect(s, p);
        return 0;
    } else {
        NCR710_DPRINTF("Queueing IO tag=0x%x\n", p->tag);
        p->pending = len;
        return 1;
    }
}

static void ncr710_add_msg_byte(NCR710State *s, uint8_t data)
{
    if (s->msg_len >= NCR710_MAX_MSGIN_LEN) {
        qemu_log_mask(LOG_GUEST_ERROR, "NCR710: MSG IN data too long\n");
    } else {
        NCR710_DPRINTF("MSG IN 0x%02x\n", data);
        s->msg[s->msg_len++] = data;
    }
}

static void ncr710_reselect(NCR710State *s, NCR710Request *p)
{
    int id;

    assert(s->current == NULL);
    QTAILQ_REMOVE(&s->queue, p, next);
    s->current = p;

    id = (p->tag >> 8) & 0xf;

    /* LSI53C700 Family Compatibility, see NCR53C710 4-73 */
    if (!(s->dcntl & DCNTL_COM)) {
        s->sfbr = 1 << (id & 0x7);
    }
    s->lcrc = 0;

    NCR710_DPRINTF("Reselected target %d\n", id);
    s->scntl1 |= SCNTL1_CON;
    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = p->out ? 2 : 3;
    s->current->dma_len = p->pending;

    ncr710_add_msg_byte(s, 0x80);
    if (s->current->tag & NCR710_TAG_VALID) {
        ncr710_add_msg_byte(s, 0x20);
        ncr710_add_msg_byte(s, p->tag & 0xff);
    }

    if (ncr710_irq_on_rsl(s)) {
        ncr710_script_scsi_interrupt(s, SSTAT0_SEL);
    }
}

static void ncr710_handle_direct_scsi_command(NCR710State *s)
{
    NCR710Request *req = s->current;
    SCSICommand *cmd;

    if (!req || !req->req) {
        qemu_log_mask(LOG_UNIMP, "NCR710: Direct SCSI handler called without valid request\n");
        return;
    }

    cmd = &req->req->cmd;

    qemu_log_mask(LOG_UNIMP, "NCR710: Direct SCSI command handler for command 0x%02x\n", cmd->buf[0]);

    switch (cmd->buf[0]) {
    case 0x12: /* INQUIRY */
        qemu_log_mask(LOG_UNIMP, "NCR710: Handling INQUIRY command directly\n");
        /* Set up basic INQUIRY response */
        s->istat |= 0x01; /* DIP - DMA interrupt pending */
        ncr710_update_irq(s);
        break;

    case 0x00: /* TEST_UNIT_READY */
        qemu_log_mask(LOG_UNIMP, "NCR710: Handling TEST_UNIT_READY command directly\n");
        /* Complete with good status */
        s->istat |= 0x01; /* DIP - DMA interrupt pending */
        ncr710_update_irq(s);
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "NCR710: Unsupported SCSI command 0x%02x in direct handler\n", cmd->buf[0]);
        /* Signal error */
        s->istat |= 0x01; /* DIP - DMA interrupt pending */
        ncr710_update_irq(s);
        break;
    }
}

static void ncr710_wait_reselect(NCR710State *s)
{
    NCR710Request *p;

    NCR710_DPRINTF("Wait Reselect\n");

    QTAILQ_FOREACH(p, &s->queue, next) {
        if (p->pending) {
            ncr710_reselect(s, p);
            break;
        }
    }
    if (s->current == NULL) {
        s->waiting = 1;
    }
}

/* SCSI operation functions from reference material */

static void ncr710_do_dma(NCR710State *s, int out)
{
    uint32_t count;
    dma_addr_t addr;
    SCSIDevice *dev;

    if (!s->current) {
        /* No current request - this can happen during SCRIPTS execution
         * before a command has been issued */
        NCR710_DPRINTF("DMA called with no current request\n");
        return;
    }

    if (!s->current->req) {
        /* No SCSI request associated with current request */
        NCR710_DPRINTF("DMA called with no SCSI request\n");
        return;
    }

    if (!s->current->dma_len) {
        /* Wait until data is available */
        NCR710_DPRINTF("DMA no data available\n");
        return;
    }

    dev = s->current->req->dev;
    if (!dev) {
        /* No device associated with request */
        NCR710_DPRINTF("DMA called with no device\n");
        return;
    }

    count = s->dbc;
    if (count > s->current->dma_len) {
        count = s->current->dma_len;
    }

    addr = s->dnad;

    NCR710_DPRINTF("DMA addr=0x%08lx len=%d\n", (unsigned long)addr, count);
    s->dnad += count;
    s->dbc -= count;

    if (s->current->dma_buf == NULL) {
        s->current->dma_buf = scsi_req_get_buf(s->current->req);
    }

    /* Set SFBR to first data byte */
    if (out) {
        ncr710_read_memory(s, addr, s->current->dma_buf, count);
    } else {
        ncr710_write_memory(s, addr, s->current->dma_buf, count);
    }

    s->current->dma_len -= count;
    if (s->current->dma_len == 0) {
        s->current->dma_buf = NULL;
        scsi_req_continue(s->current->req);
    } else {
        s->current->dma_buf += count;
        ncr710_resume_script(s);
    }
}

static void ncr710_do_command(NCR710State *s)
{
    SCSIDevice *dev;
    uint8_t buf[NCR710_MAX_CDB_SIZE];
    uint32_t id;
    int n;

    NCR710_DPRINTF("Send command len=%d\n", s->dbc);

    /* Limit command length to maximum CDB size */
    if (s->dbc > NCR710_MAX_CDB_SIZE) {
        s->dbc = NCR710_MAX_CDB_SIZE;
    }

    /* Ensure we have a valid command length */
    if (s->dbc == 0) {
        NCR710_DPRINTF("Zero-length command\n");
        ncr710_script_scsi_interrupt(s, SSTAT0_SGE);
        return;
    }

    /* Read the SCSI command from memory */
    ncr710_read_memory(s, s->dnad, buf, s->dbc);
    NCR710_DPRINTF("Send command len=%d %02x.%02x.%02x.%02x.%02x.%02x\n",
                   s->dbc, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    /* Store first byte in SFBR register as per NCR710 behavior */
    s->sfbr = buf[0];
    s->command_complete = 0;

    /* Extract target ID from select tag - upper 8 bits contain target mask */
    id = (s->select_tag >> 8) & 0xff;
    s->lcrc = id;  /* Store for parity calculation */

    /* Find the target device */
    dev = scsi_device_find(&s->bus, 0, ncr710_idbitstonum(id), s->current_lun);
    if (!dev) {
        NCR710_DPRINTF("Target %d not found\n", ncr710_idbitstonum(id));
        ncr710_bad_selection(s, id);
        return;
    }

    /* Ensure we don't have an active request already */
    if (s->current != NULL) {
        NCR710_DPRINTF("WARNING: Command started with active request\n");
        ncr710_request_free(s, s->current);
        s->current = NULL;
    }

    /* Create new request structure */
    s->current = g_new0(NCR710Request, 1);
    s->current->tag = s->select_tag;

    /* Create SCSI request and associate with our request structure */
    s->current->req = scsi_req_new(dev, s->current->tag, s->current_lun, buf, s->dbc, s->current);
    if (!s->current->req) {
        NCR710_DPRINTF("Failed to create SCSI request\n");
        g_free(s->current);
        s->current = NULL;
        ncr710_script_scsi_interrupt(s, SSTAT0_SGE);
        return;
    }

    /* Add to queue for proper management */
    QTAILQ_INSERT_TAIL(&s->queue, s->current, next);

    /* Enqueue the SCSI command */
    n = scsi_req_enqueue(s->current->req);

    if (n) {
        /* Command requires data transfer */
        if (n > 0) {
            /* Data In phase required */
            ncr710_set_phase(s, PHASE_DI);
            NCR710_DPRINTF("Command requires %d bytes of data in\n", n);
        } else {
            /* Data Out phase required */
            ncr710_set_phase(s, PHASE_DO);
            NCR710_DPRINTF("Command requires %d bytes of data out\n", -n);
        }

        /* Continue with the SCSI request */
        scsi_req_continue(s->current->req);

        if (!s->command_complete) {
            /* Command will complete asynchronously - prepare for disconnect */
            ncr710_add_msg_byte(s, SCSI_MSG_SAVE_DATA_POINTER);
            ncr710_add_msg_byte(s, SCSI_MSG_DISCONNECT);
            ncr710_set_phase(s, PHASE_MI);
            s->msg_action = NCR710_MSG_ACTION_DISCONNECT;
        }
    } else {
        /* No data transfer required - wait for status */
        NCR710_DPRINTF("Command requires no data transfer\n");
        ncr710_set_phase(s, PHASE_ST);
    }
}

static void ncr710_do_status(NCR710State *s)
{
    uint8_t status;

    NCR710_DPRINTF("Get status len=%d status=%d\n", s->dbc, s->status);

    /* Status phase should transfer exactly 1 byte */
    if (s->dbc != 1) {
        qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Bad Status move, expected 1 byte, got %d\n", s->dbc);
        s->dbc = 1;  /* Force to 1 byte */
    }

    status = s->status;
    s->sfbr = status;  /* Store in SFBR as per NCR710 behavior */

    /* Write status byte to memory */
    ncr710_write_memory(s, s->dnad, &status, 1);

    /* Transition to Message In phase for COMMAND COMPLETE */
    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = NCR710_MSG_ACTION_COMMAND;
    ncr710_add_msg_byte(s, SCSI_MSG_COMMAND_COMPLETE);

    NCR710_DPRINTF("Status byte 0x%02x sent, transitioning to MSG IN\n", status);
}

static void ncr710_do_msgin(NCR710State *s)
{
    int len;

    NCR710_DPRINTF("Message in len=%d/%d\n", s->dbc, s->msg_len);

    if (s->msg_len == 0) {
        NCR710_DPRINTF("No message data available\n");
        return;
    }

    s->sfbr = s->msg[0];  /* First message byte goes to SFBR */
    len = s->msg_len;

    if (len > s->dbc) {
        len = s->dbc;
    }

    /* Transfer message bytes to memory */
    ncr710_write_memory(s, s->dnad, s->msg, len);

    /* Linux drivers rely on the last byte being in the SIDL */
    s->sidl = s->msg[len - 1];
    s->msg_len -= len;

    if (s->msg_len) {
        /* More message bytes to send */
        memmove(s->msg, s->msg + len, s->msg_len);
    } else {
        /* Message transfer complete - determine next phase */
        switch (s->msg_action) {
        case NCR710_MSG_ACTION_COMMAND:
            ncr710_set_phase(s, PHASE_CMD);
            break;
        case NCR710_MSG_ACTION_DISCONNECT:
            ncr710_disconnect(s);
            break;
        case NCR710_MSG_ACTION_DOUT:
            ncr710_set_phase(s, PHASE_DO);
            break;
        case NCR710_MSG_ACTION_DIN:
            ncr710_set_phase(s, PHASE_DI);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Invalid message action %d\n", s->msg_action);
            ncr710_script_scsi_interrupt(s, SSTAT0_SGE);
            break;
        }
    }
}

static uint8_t ncr710_get_msgbyte(NCR710State *s)
{
    uint8_t data;
    ncr710_read_memory(s, s->dnad, &data, 1);
    s->dnad++;
    s->dbc--;
    return data;
}

static void ncr710_skip_msgbytes(NCR710State *s, unsigned int n)
{
    s->dnad += n;
    s->dbc -= n;
}

static void ncr710_do_msgout(NCR710State *s)
{
    uint8_t msg;
    int len;
    uint32_t current_tag;
    NCR710Request *current_req, *p, *p_next;

    if (s->current) {
        current_tag = s->current->tag;
        current_req = s->current;
    } else {
        current_tag = s->select_tag;
        current_req = ncr710_find_by_tag(s, current_tag);
    }

    NCR710_DPRINTF("MSG out len=%d\n", s->dbc);
    while (s->dbc) {
        msg = ncr710_get_msgbyte(s);
        s->sfbr = msg;

        switch (msg) {
        case SCSI_MSG_DISCONNECT:
            NCR710_DPRINTF("MSG: Disconnect\n");
            ncr710_disconnect(s);
            break;
        case SCSI_MSG_NOP:
            NCR710_DPRINTF("MSG: No Operation\n");
            ncr710_set_phase(s, PHASE_CMD);
            break;
        case SCSI_MSG_EXTENDED_MESSAGE:
            len = ncr710_get_msgbyte(s);
            msg = ncr710_get_msgbyte(s);
            (void)len; /* avoid warning */
            NCR710_DPRINTF("Extended message 0x%x (len %d)\n", msg, len);
            switch (msg) {
            case 1: /* SDTR */
                NCR710_DPRINTF("SDTR (ignored)\n");
                ncr710_skip_msgbytes(s, 2);
                break;
            case 3: /* WDTR */
                NCR710_DPRINTF("WDTR (ignored)\n");
                ncr710_skip_msgbytes(s, 1);
                break;
            default:
                goto bad;
            }
            break;
        case 0x20: /* SIMPLE queue tag */
            s->select_tag |= ncr710_get_msgbyte(s) | NCR710_TAG_VALID;
            NCR710_DPRINTF("SIMPLE queue tag=0x%x\n", s->select_tag & 0xff);
            break;
        case 0x21: /* HEAD of queue tag */
            qemu_log_mask(LOG_UNIMP, "NCR710: HEAD queue not implemented\n");
            s->select_tag |= ncr710_get_msgbyte(s) | NCR710_TAG_VALID;
            break;
        case 0x22: /* ORDERED queue tag */
            qemu_log_mask(LOG_UNIMP, "NCR710: ORDERED queue not implemented\n");
            s->select_tag |= ncr710_get_msgbyte(s) | NCR710_TAG_VALID;
            break;
        case 0x0d: /* ABORT TAG */
            /* The ABORT TAG message clears the current I/O process only */
            NCR710_DPRINTF("MSG: ABORT TAG tag=0x%x\n", current_tag);
            if (current_req) {
                scsi_req_cancel(current_req->req);
            }
            ncr710_disconnect(s);
            break;
        case SCSI_MSG_ABORT:
        case 0x0e: /* CLEAR QUEUE */
        case SCSI_MSG_BUS_DEVICE_RESET:
            /* Various abort/reset messages */
            if (msg == SCSI_MSG_ABORT) {
                NCR710_DPRINTF("MSG: ABORT tag=0x%x\n", current_tag);
            } else if (msg == 0x0e) {
                NCR710_DPRINTF("MSG: CLEAR QUEUE tag=0x%x\n", current_tag);
            } else if (msg == SCSI_MSG_BUS_DEVICE_RESET) {
                NCR710_DPRINTF("MSG: BUS DEVICE RESET tag=0x%x\n", current_tag);
            }

            /* clear the current I/O process */
            if (s->current) {
                scsi_req_cancel(s->current->req);
            }

            /* Clear all queued commands for the current device */
            QTAILQ_FOREACH_SAFE(p, &s->queue, next, p_next) {
                if ((p->tag & 0x0000ff00) == (current_tag & 0x0000ff00)) {
                    scsi_req_cancel(p->req);
                }
            }

            ncr710_disconnect(s);
            break;
        default:
            if ((msg & 0x80) == 0) {
                goto bad;
            }
            s->current_lun = msg & 7;
            NCR710_DPRINTF("Select LUN %d\n", s->current_lun);
            ncr710_set_phase(s, PHASE_CMD);
            break;
        }
    }
    return;

bad:
    qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Unimplemented message 0x%02x\n", msg);
    ncr710_set_phase(s, PHASE_MI);
    ncr710_add_msg_byte(s, 7); /* MESSAGE REJECT */
    s->msg_action = 0;
}

static void ncr710_memcpy(NCR710State *s, uint32_t dest, uint32_t src, int count)
{
    int n;
    uint8_t buf[NCR710_BUF_SIZE];

    NCR710_DPRINTF("memcpy dest 0x%08x src 0x%08x count %d\n", dest, src, count);
    while (count) {
        n = (count > NCR710_BUF_SIZE) ? NCR710_BUF_SIZE : count;
        ncr710_read_memory(s, src, buf, n);
        ncr710_write_memory(s, dest, buf, n);
        src += n;
        dest += n;
        count -= n;
    }
}

/* Register read/write helpers for SCRIPTS processor */
static uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR710State *s = opaque;
    uint8_t val = ncr710_reg_readb(s, addr & 0xff);
    trace_ncr710_reg_read(addr & 0xff, val);
    return val;
}

static void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    NCR710State *s = opaque;
    trace_ncr710_reg_write(addr & 0xff, val & 0xff);
    ncr710_reg_writeb(s, addr & 0xff, val & 0xff);
}

static uint8_t ncr710_reg_readb(NCR710State *s, int offset)
{
    uint8_t ret;
    bool is_700_mode = ncr710_is_700_mode(s);

#define CASE_GET_REG24(name, addr) \
    case addr: ret = s->name & 0xff; break; \
    case addr + 1: ret = (s->name >> 8) & 0xff; break; \
    case addr + 2: ret = (s->name >> 16) & 0xff; break;

#define CASE_GET_REG32(name, addr) \
    case addr: ret = s->name & 0xff; break; \
    case addr + 1: ret = (s->name >> 8) & 0xff; break; \
    case addr + 2: ret = (s->name >> 16) & 0xff; break; \
    case addr + 3: ret = (s->name >> 24) & 0xff; break;

    switch (offset) {
        case NCR710_SCNTL0_REG: /* SCNTL0 */
            ret = s->scntl0;
            break;
        case NCR710_SCNTL1_REG: /* SCNTL1 */
            ret = s->scntl1;
            if (is_700_mode) {
                /* TODO:
                 * In 700 mode: bits 1,0 are Start SCSI Send/Receive (removed in 710) */
                /* These don't exist in 710, so mask them out in 710 mode */
            }
            break;
        case NCR710_SDID_REG: /* SDID */
            ret = s->sdid;
            break;
        case NCR710_SIEN_REG: /* SIEN */
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
        case NCR710_SXFER_REG: /* SXFER */
            ret = s->sxfer;
            break;
        case NCR710_SODL_REG: /* SODL */
            ret = s->sodl;
            break;
        case NCR710_SOCL_REG: /* SOCL */
            ret = s->socl;
            break;
        case NCR710_SFBR_REG: /* SFBR */
            ret = s->sfbr;
            break;
        case NCR710_SIDL_REG: /* SIDL */
            ret = s->sidl;
            break;
        case NCR710_SBDL_REG: /* SBDL */
            ret = s->sbdl;
            break;
        case NCR710_SBCL_REG: /* SBCL */
            ret = 0;
            if (s->scntl1 & SCNTL1_CON) {
                ret = s->sstat2 & PHASE_MASK;
                ret |= s->sbcl;
                if (s->socl & SOCL_ATN)
                    ret |= SBCL_ATN;
            }
            break;
        case NCR710_DSTAT_REG: /* DSTAT: In 700 mode: bit 5 (Bus fault) doesn't exist */
            ret = s->dstat | DSTAT_DFE;
            if (is_700_mode) {
                ret &= ~0x20;  /* Mask out bus fault bit */
            }
            s->dstat = 0;
            ncr710_update_irq(s);
            break;
        case NCR710_SSTAT0_REG: /* SSTAT0 */
            ret = s->sstat0;
            if (s->sstat0 != 0) {
                s->sstat0 = 0;
                s->istat &= ~ISTAT_SIP;
                ncr710_update_irq(s);
            }
            break;
        case NCR710_SSTAT1_REG: /* SSTAT1 */
            ret = s->sstat1;
            break;
        case NCR710_SSTAT2_REG: /* SSTAT2 */
            ret = s->sstat2;
            break;
        CASE_GET_REG32(dsa, NCR710_DSA_REG)
            if (is_700_mode) {
                qemu_log_mask(LOG_GUEST_ERROR, "NCR710: DSA read in 700 compatibility mode\n");
                return 0;
            }
            break;
        case NCR710_CTEST0_REG: /* CTEST0 */
            ret = s->ctest0;
            break;
        case NCR710_CTEST1_REG: /* CTEST1 */
            ret = s->ctest1;
            break;
        case NCR710_CTEST2_REG: /* CTEST2 */
            ret = s->ctest2;
            if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
                s->ctest2 |= 0x04;
            } else {
                s->ctest2 &= ~0x04;
            }
            break;
        case NCR710_CTEST3_REG: /* CTEST3 */
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
        case NCR710_CTEST4_REG: /* CTEST4 */
            ret = s->ctest4;
            if (is_700_mode) {
                /* In 700 mode: bit 7 (Mux mode) doesn't exist */
                ret &= ~0x80;
            }
            break;
        case NCR710_CTEST5_REG: /* CTEST5 */
            ret = s->ctest5;
            break;
        case NCR710_CTEST6_REG: /* CTEST6 */
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
        case NCR710_CTEST7_REG: /* CTEST7 */
            if (is_700_mode) {
                /* In 700 mode: bits 7-4 are chip revision, bit 1 is DCI pin control */
                ret = (s->ctest7 & 0x0F) | (NCR710_REVISION_2 << 4);
                /* DCI pin control is at bit 1 in 700 mode */
                if (s->ctest7 & 0x02) {
                    ret |= 0x02;
                }
            } else {
                /* In 710 mode: new bits available */
                ret = s->ctest7;
            }
            break;
        CASE_GET_REG32(temp, NCR710_TEMP_REG)
        case NCR710_DFIFO_REG: /* DFIFO */
            if (is_700_mode) {
                /* In 700 mode: bit 7 is flush FIFO, bit 6 is clear FIFO */
                ret = s->dma_fifo.count & 0x3F; /* FIFO count (bits 5-0) */
                /* Map 710 CTEST8 bits back to 700 DFIFO layout */
                if (s->ctest8 & 0x08) ret |= 0x80;  /* Flush FIFO bit */
                if (s->ctest8 & 0x04) ret |= 0x40;  /* Clear FIFO bit */
            } else {
                /* In 710 mode: return current DFIFO value with DMA FIFO status */
                ret = s->dfifo;
                s->dfifo = s->dma_fifo.count & 0x7F;
                if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
                    s->dfifo |= 0x80; /* Set DFE bit */
                }
                ret = s->dfifo;
            }
            break;
        case NCR710_ISTAT_REG: /* ISTAT */
            ret = s->istat;
            if (is_700_mode) {
                if (s->dsp == 0 && s->dsps == 0) {
                    ret |= 0x04;  /* Set DSP/DSPS empty bit */
                }
                ret &= ~ISTAT_SIGP;  /* SIGP doesn't exist in 700 */
                ret &= ~ISTAT_RST;
            }
            break;
        case NCR710_CTEST8_REG: /* CTEST8 */
            if (is_700_mode) {
                qemu_log_mask(LOG_GUEST_ERROR, "NCR710: CTEST8 read in 700 compatibility mode\n");
                return 0;
            }
            ret = (s->ctest8 | (NCR710_REVISION_2 << 4)) & ~0x04;
            break;
        case NCR710_LCRC_REG: /* LCRC */
            if (is_700_mode) {
                qemu_log_mask(LOG_GUEST_ERROR, "NCR710: LCRC read in 700 compatibility mode\n");
                return 0;
            }
            ret = s->lcrc;
            break;
        CASE_GET_REG24(dbc, NCR710_DBC_REG)
        case NCR710_DCMD_REG: /* DCMD */
            ret = s->dcmd;
            break;
        CASE_GET_REG32(dnad, NCR710_DNAD_REG)
        CASE_GET_REG32(dsp, NCR710_DSP_REG)
        CASE_GET_REG32(dsps, NCR710_DSPS_REG)
        CASE_GET_REG32(scratch, NCR710_SCRATCH_REG)
            if (is_700_mode) {
                qemu_log_mask(LOG_GUEST_ERROR, "NCR710: SCRATCH read in 700 compatibility mode\n");
                return 0;
            }
            break;
        case NCR710_DMODE_REG: /* DMODE */
            ret = s->dmode;
            if (is_700_mode) {
                /* In 700 mode: different bit meanings */
                /* Bits 5-4: 16-bit DMA '286-mode bits (700 only) */
                /* Bit 3: I/O-memory mapped DMA bit (700 only) */
                /* Bit 1: Pipeline mode bit (700 only) */
                /* Clear 710-specific function code bits */
                ret &= ~0x38; /* Clear bits 5-3 which have different meanings in 710 */
            }
            break;
        case NCR710_DIEN_REG: /* DIEN */
            ret = s->dien;
            if (is_700_mode) {
                /* In 700 mode: bit 5 (Bus fault interrupt enable) doesn't exist */
                ret &= ~0x20;
            }
            break;
        case NCR710_DWT_REG: /* DWT */
            ret = s->dwt;
            break;
        case NCR710_DCNTL_REG: /* DCNTL */
            ret = s->dcntl;
            if (is_700_mode) {
                /* In 700 mode: mask out 710-specific bits */
                ret &= ~(DCNTL_EA | DCNTL_COM);  /* These don't exist in 700 */
                /* Software reset is bit 0 in 700 mode */
            }
            return ret;
        CASE_GET_REG32(adder, NCR710_ADDER_REG)
            if (is_700_mode) {
                qemu_log_mask(LOG_GUEST_ERROR, "NCR710: ADDER read in 700 compatibility mode\n");
                return 0;
            }
            break;
        default:
            trace_ncr710_reg_read_unhandled(offset, 1);
            qemu_log_mask(LOG_GUEST_ERROR,
                          "NCR710: invalid read at offset 0x%x\n", (int)offset);
            break;
    }

#undef CASE_GET_REG24
#undef CASE_GET_REG32
    return ret;
}

/* Unified register write function combining compatibility mode and FIFO handling */
static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val)
{
    bool is_700_mode = ncr710_is_700_mode(s);
    uint8_t old_val;

#define CASE_SET_REG24(name, addr) \
    case addr    : s->name &= 0xffffff00; s->name |= val;       break; \
    case addr + 1: s->name &= 0xffff00ff; s->name |= val << 8;  break; \
    case addr + 2: s->name &= 0xff00ffff; s->name |= val << 16; break;

#define CASE_SET_REG32(name, addr) \
    case addr    : s->name &= 0xffffff00; s->name |= val;       break; \
    case addr + 1: s->name &= 0xffff00ff; s->name |= val << 8;  break; \
    case addr + 2: s->name &= 0xff00ffff; s->name |= val << 16; break; \
    case addr + 3: s->name &= 0x00ffffff; s->name |= val << 24; break;

    NCR710_DPRINTF("Write reg %x = %02x\n", offset, val);
    
    switch (offset) {
    case NCR710_SCNTL0_REG: /* SCNTL0 */
        s->scntl0 = val;
        if (val & SCNTL0_START) {
            if (is_700_mode) {
                qemu_log_mask(LOG_UNIMP, "NCR710: Start sequence not implemented\n");
            } else {
                ncr710_arbitrate_bus(s);
            }
        }
        break;
        
    case NCR710_SCNTL1_REG: /* SCNTL1 */
        old_val = s->scntl1;
        s->scntl1 = val;
        
        if (val & SCNTL1_ADB) {
            qemu_log_mask(LOG_UNIMP, "NCR710: Immediate Arbitration not implemented\n");
        }
        
        if (val & SCNTL1_RST) {
            if (!(s->sstat0 & SSTAT0_RST)) {
                s->sstat0 |= SSTAT0_RST;
                ncr710_script_scsi_interrupt(s, SSTAT0_RST);
            }
            /* Enhanced reset handling for second implementation */
            if (!(old_val & SCNTL1_RST)) {
                ncr710_internal_scsi_bus_reset(s);
            }
        } else {
            s->sstat0 &= ~SSTAT0_RST;
        }
        break;
        
    case NCR710_SDID_REG: /* SDID */
        s->sdid = val & 0x0F; /* Only lower 4 bits are valid */
        break;
        
    case NCR710_SIEN_REG: /* SIEN */
        s->sien = val;
        ncr710_update_irq(s);
        break;
        
    case NCR710_SCID_REG: /* SCID */
        s->scid = val;
        break;
        
    case NCR710_SXFER_REG: /* SXFER */
        s->sxfer = val;
        break;
        
    case NCR710_SODL_REG: /* SODL */
        s->sodl = val;
        s->sstat1 |= SSTAT1_OLF; /* From second implementation */
        break;
        
    case NCR710_SOCL_REG: /* SOCL */
        s->socl = val;
        s->sbcl = val; /* From second implementation */
        break;
        
    case NCR710_SFBR_REG: /* SFBR */
        s->sfbr = val;
        break;
        
    case NCR710_SIDL_REG: /* SIDL */
        s->sidl = val;
        break;
        
    case NCR710_SBDL_REG: /* SBDL */
        s->sbdl = val;
        break;
        
    case NCR710_SBCL_REG: /* SBCL */
        ncr710_set_phase(s, val & PHASE_MASK);
        break;
        
    case NCR710_DSTAT_REG: 
    case NCR710_SSTAT0_REG: 
    case NCR710_SSTAT1_REG: 
    case NCR710_SSTAT2_REG:
        /* Linux writes to these readonly registers on startup */
        return;
        
    CASE_SET_REG32(dsa, NCR710_DSA_REG)
        if (is_700_mode) {
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: DSA write in 700 compatibility mode\n");
            return;
        }
        break;
        
    case NCR710_CTEST0_REG: /* CTEST0 */
        s->ctest0 = val;
        if (val & 0x01) { /* EAN bit - from second implementation */
            s->tolerant_enabled = true;
        } else {
            s->tolerant_enabled = false;
        }
        break;
        
    case NCR710_CTEST1_REG: /* CTEST1, read-only */
        s->ctest1 = val; /* From second implementation - some bits may be writable */
        break;
        
    case NCR710_CTEST2_REG: /* CTEST2, read-only */
        s->ctest2 = val; /* From second implementation */
        break;
        
    case NCR710_CTEST3_REG: /* CTEST3 */
        s->ctest3 = val;
        /* FIFO handling from second implementation */
        if (!ncr710_scsi_fifo_full(&s->scsi_fifo)) {
            uint8_t parity = ncr710_calculate_parity(val);
            ncr710_scsi_fifo_push(&s->scsi_fifo, val, parity);
        }
        break;
        
    case NCR710_CTEST4_REG: /* CTEST4 */
        s->ctest4 = val;
        break;
        
    case NCR710_CTEST5_REG: /* CTEST5 */
        s->ctest5 = val;
        break;
        
    case NCR710_CTEST6_REG: /* CTEST6 */
        s->ctest6 = val;
        /* FIFO handling from second implementation */
        if (!ncr710_dma_fifo_full(&s->dma_fifo)) {
            uint8_t parity = (s->ctest7 & 0x08) ? 1 : 0; /* DFP bit */
            ncr710_dma_fifo_push(&s->dma_fifo, val, parity);
        }
        break;
        
    case NCR710_CTEST7_REG: /* CTEST7 */
        s->ctest7 = val;
        /* Enhanced handling from second implementation */
        if (val & 0x01) { /* DIFF bit */
            s->differential_mode = true;
        } else {
            s->differential_mode = false;
        }
        s->cache_line_burst = !(val & 0x80); /* CDIS bit inverted */
        break;
        
    CASE_SET_REG32(temp, NCR710_TEMP_REG)
    
    case NCR710_DFIFO_REG: /* DFIFO, read-only */
        if (is_700_mode) {
            /* In 700 mode: some bits may be writable for FIFO control */
            /* Map to CTEST8 equivalents for compatibility */
            if (val & 0x80) s->ctest8 |= 0x08;  /* Flush FIFO */
            if (val & 0x40) s->ctest8 |= 0x04;  /* Clear FIFO */
        }
        break;
        
    case NCR710_ISTAT_REG: /* ISTAT */
        if (is_700_mode) {
            /* In 700 mode: no SIGP bit, no RST bit */
            s->istat = (s->istat & 0x0f) | (val & 0xDF); /* Mask out SIGP and RST */
        } else {
            /* Normal 710 mode with enhanced handling from second implementation */
            if (val & ISTAT_ABRT) {
                s->scripts.running = false;
                s->dstat |= DSTAT_ABRT;
                s->istat |= ISTAT_DIP;
                timer_del(s->selection_timer);
                timer_del(s->watchdog_timer);
                ncr710_script_dma_interrupt(s, DSTAT_ABRT);
                ncr710_update_irq(s);
            }
            if (s->waiting == 1 && (val & ISTAT_SIGP)) {
                NCR710_DPRINTF("Woken by SIGP\n");
                s->waiting = 0;
                s->dsp = s->dnad;
                ncr710_scripts_execute(s);
            }
            if (val & ISTAT_RST) {
                /* Enhanced reset from second implementation */
                uint8_t saved_ctest8 = s->ctest8;
                ncr710_soft_reset(s);
                s->ctest8 = saved_ctest8;  /* Preserve chip revision */
                s->dstat = DSTAT_DFE;
                ncr710_dma_fifo_init(&s->dma_fifo);
                ncr710_scsi_fifo_init(&s->scsi_fifo);
            }
            s->istat = (s->istat & 0x0f) | (val & 0xf0);
        }
        break;
        
    case NCR710_CTEST8_REG: /* CTEST8 */
        if (is_700_mode) {
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: CTEST8 write in 700 compatibility mode\n");
            return;
        }
        s->ctest8 = (s->ctest8 & 0xF0) | (val & 0x0F);
        if (val & 0x04) { /* FLF bit - from second implementation */
            ncr710_dma_fifo_flush(&s->dma_fifo);
        }
        break;
        
    case NCR710_LCRC_REG: /* LCRC */
        if (is_700_mode) {
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: LCRC write in 700 compatibility mode\n");
            return;
        }
        s->lcrc = val; /* From second implementation - preserve value */
        break;
        
    CASE_SET_REG24(dbc, NCR710_DBC_REG)
    
    case NCR710_DCMD_REG: /* DCMD */
        s->dcmd = val;
        break;
        
    CASE_SET_REG32(dnad, NCR710_DNAD_REG)
    
    case NCR710_DSP_REG: /* DSP[0:7] */
        s->dsp &= 0xffffff00;
        s->dsp |= val;
        if ((s->dmode & DMODE_MAN) == 0 && !is_700_mode) {
            s->waiting = 0;
            ncr710_scripts_execute(s);
        }
        break;
    case NCR710_DSP_REG + 1: /* DSP[8:15] */
        s->dsp &= 0xffff00ff;
        s->dsp |= val << 8;
        break;
    case NCR710_DSP_REG + 2: /* DSP[16:23] */
        s->dsp &= 0xff00ffff;
        s->dsp |= val << 16;
        break;
    case NCR710_DSP_REG + 3: /* DSP[24:31] */
        s->dsp &= 0x00ffffff;
        s->dsp |= val << 24;
        /* Enhanced handling from second implementation */
        printf("=== NCR710: DSP register write complete: DSP=0x%08x ===\n", s->dsp);
        fflush(stdout);
        if (!is_700_mode && (s->dmode & DMODE_MAN) == 0) {
            s->waiting = 0;
            s->scripts.running = true;
            s->scripts.pc = s->dsp;
            ncr710_scripts_execute(s);
        }
        break;
        
    CASE_SET_REG32(dsps, NCR710_DSPS_REG)
    
    CASE_SET_REG32(scratch, NCR710_SCRATCH_REG)
        if (is_700_mode) {
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: SCRATCH write in 700 compatibility mode\n");
            return;
        }
        break;
        
    case NCR710_DMODE_REG: /* DMODE */
        s->dmode = val;
        if (is_700_mode) {
            /* In 700 mode: different bit meanings */
            /* Bits 5-4: 16-bit DMA '286-mode bits (700 only) */
            /* Bit 3: I/O-memory mapped DMA bit (700 only) */
            /* Bit 1: Pipeline mode bit (700 only) */
            /* Clear 710-specific function code bits */
            s->dmode &= ~0x38; /* Clear bits 5-3 which have different meanings in 710 */
        } else {
            /* Enhanced handling from second implementation */
            switch (val & DMODE_BL_MASK) {
            case 0x00: s->burst_length = 1; break;
            case 0x40: s->burst_length = 2; break;
            case 0x80: s->burst_length = 4; break;
            case 0xC0: s->burst_length = 8; break;
            }
        }
        break;
        
    case NCR710_DIEN_REG: /* DIEN */
        s->dien = val;
        if (is_700_mode) {
            /* In 700 mode: bit 5 (Bus fault interrupt enable) doesn't exist */
            s->dien &= ~0x20;
        }
        ncr710_update_irq(s);
        break;
        
    case NCR710_DWT_REG: /* DWT */
        s->dwt = val;
        break;
        
    case NCR710_DCNTL_REG: /* DCNTL */
        if (is_700_mode) {
            /* In 700 mode: different bit layout */
            s->dcntl = val & ~(DCNTL_PFF | DCNTL_STD | DCNTL_EA);
            /* Handle software reset bit (bit 0 in 700 mode) */
            if (val & 0x01) {
                ncr710_soft_reset(s);
            }
            /* 16-bit SCSI scripts mode is bit 5 in 700 */
        } else {
            /* Normal 710 mode with enhanced handling */
            s->dcntl = val & ~(DCNTL_PFF | DCNTL_STD);
            if ((val & DCNTL_STD) && (s->dmode & DMODE_MAN) != 0) {
                ncr710_scripts_execute(s);
            }
            /* From second implementation */
            if (val & DCNTL_STD) {
                ncr710_dma_transfer(s);
            }
        }
        ncr710_update_compatibility_mode(s);
        break;
        
    CASE_SET_REG32(adder, NCR710_ADDER_REG)
        if (is_700_mode) {
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: ADDER write in 700 compatibility mode\n");
            return;
        }
        break;
        
    default:
        qemu_log_mask(LOG_UNIMP, "NCR710: write unknown register %02X\n", offset);
        break;
    }
    
#undef CASE_SET_REG24
#undef CASE_SET_REG32
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

    /* Add memory dump for SCRIPTS debugging - enabled for debugging */
    if (addr == s->dsp) {
        uint32_t dump[8];
        int i;
        printf("=== NCR710: Reading SCRIPTS at DSP=0x%08x ===\n", addr);
        for (i = 0; i < 8; i++) {
            uint32_t dump_addr = addr + (i * 4);
            if (address_space_read(s->as, dump_addr, MEMTXATTRS_UNSPECIFIED,
                                   (uint8_t*)&dump[i], 4) == MEMTX_OK) {
                dump[i] = le32_to_cpu(dump[i]);
                printf("  [%08x] = 0x%08x\n", dump_addr, dump[i]);
            } else {
                printf("  [%08x] = <read failed>\n", dump_addr);
                dump[i] = 0;
            }
        }
        printf("=== End SCRIPTS dump ===\n");
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
    int level;
    static int last_level;
    NCR710Request *p;

    /* It's unclear whether the DIP/SIP bits should be cleared when the
       Interrupt Status Registers are cleared or when istat0 is read.
       We currently do the former, which seems to work. */
    level = 0;
    if (s->dstat) {
        if (s->dstat & s->dien)
            level = 1;
        s->istat |= ISTAT_DIP;
    } else {
        s->istat &= ~ISTAT_DIP;
    }

    if (s->sstat0) {
        if ((s->sstat0 & s->sien))
            level = 1;
        s->istat |= ISTAT_SIP;
    } else {
        s->istat &= ~ISTAT_SIP;
    }

    if (level != last_level) {
        NCR710_DPRINTF("Update IRQ level %d dstat %02x sstat0 %02x\n",
                level, s->dstat, s->sstat0);
        last_level = level;
    }

    if (s->irq) {
        qemu_set_irq(s->irq, level);
    }

    if (!level && ncr710_irq_on_rsl(s) && !(s->scntl1 & SCNTL1_CON)) {
        NCR710_DPRINTF("Handled IRQs & disconnected, looking for pending processes\n");
        QTAILQ_FOREACH(p, &s->queue, next) {
            if (p->pending) {
                ncr710_reselect(s, p);
                break;
            }
        }
    }
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

            SCSIDevice *scsi_dev = scsi_device_find(&s->bus, 0, target_id, 0);
            if (scsi_dev) {
                qemu_log("NCR710: Found SCSI device at target %d\n", target_id);
                s->sstat0 |= SSTAT0_SEL;  /* Selected */
                s->scntl1 |= SCNTL1_CON;  /* Connected */
                s->scripts.connected = true;

                if (s->socl & SOCL_ATN) {
                    ncr710_set_phase(s, PHASE_MO);
                    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | PHASE_MO;
                } else {
                    ncr710_set_phase(s, PHASE_CMD);
                    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | PHASE_CMD;
                }

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
    NCR710Request *req, *next_req;

    trace_ncr710_bus_reset();
    qemu_log("NCR710: Internal SCSI bus reset called\n");

    QTAILQ_FOREACH_SAFE(req, &s->queue, next, next_req) {
        NCR710_DPRINTF("Cancelling queued request tag=0x%x\n", req->tag);
        scsi_req_cancel(req->req);
    }
    if (s->current) {
        NCR710_DPRINTF("Cancelling current request tag=0x%x\n", s->current->tag);
        scsi_req_cancel(s->current->req);
        s->current = NULL;
    }
    s->sstat0 = SSTAT0_RST;  /* Set RST bit to indicate reset occurred */
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
    s->script_active = 0;
    trace_ncr710_connected(false);
    ncr710_dma_fifo_flush(&s->dma_fifo);
    ncr710_scsi_fifo_init(&s->scsi_fifo);
    s->istat &= ~(ISTAT_SIP | ISTAT_DIP | ISTAT_CON);
    if (s->sien & SIEN_RST) {
        s->istat |= ISTAT_SIP;
    }
    bus_cold_reset(BUS(&s->bus));
    ncr710_update_irq(s);
    qemu_log("NCR710: SCSI bus reset completed\n");
}

static void ncr710_device_reset(DeviceState *dev)
{
    SysBusNCR710State *sysbus_s = SYSBUS_NCR710_SCSI(dev);
    NCR710State *s = &sysbus_s->ncr710;

    trace_ncr710_device_reset();
    qemu_log("NCR710: Device reset function called\n");

    ncr710_soft_reset(s);

    QTAILQ_INIT(&s->queue);

    s->scripts.running = false;
    s->scripts.connected = false;
    s->scripts.initiator = false;
    s->scripts.phase = SCSI_PHASE_DATA_OUT;
    s->scripts.pc = 0;
    s->scripts.saved_pc = 0;
    s->script_active = 0;

    if (s->irq) {
        qemu_set_irq(s->irq, 0);
    }
}

static void ncr710_scripts_execute(NCR710State *s)
{
    uint32_t insn;
    uint32_t addr;
    int opcode;
    int insn_processed = 0;

    printf("=== NCR710: SCRIPTS execution starting, DSP=0x%08x ===\n", s->dsp);
    fflush(stdout);
    s->script_active = 1;
    s->scripts.running = true;

again:
    insn_processed++;
    if (!s->scripts.running) {
        return;
    }

    insn = ncr710_read_memory_32(s, s->dsp);

    if (!insn) {
        qemu_log("NCR710: Empty opcode at DSP=0x%08x\n", s->dsp);
        printf("=== NCR710: Empty opcode at DSP=0x%08x ===\n", s->dsp);

        /* WORKAROUND: If we're getting too many empty opcodes, it means the SCRIPTS
         * aren't loaded properly. Let's try to handle basic SCSI operations directly. */
        if (insn_processed > 50) {
            printf("=== NCR710: Too many empty opcodes (%d), trying direct SCSI handling ===\n", insn_processed);

            /* Try to handle a basic INQUIRY command directly */
            if (s->select_tag && !s->current) {
                printf("=== NCR710: Attempting direct SCSI command handling ===\n");
                ncr710_handle_direct_scsi_command(s);
                return;
            }

            /* Stop SCRIPTS execution to prevent infinite loop */
            printf("=== NCR710: Stopping SCRIPTS execution due to too many empty opcodes ===\n");
            s->scripts.running = false;
            return;
        }

        s->dsp += 4;
        goto again;
    }

    /* Check for obviously invalid opcodes that indicate uninitialized memory */
    if (insn == 0xaaaaaaaa || insn == 0xbbbbbbbb || insn == 0xcccccccc ||
        insn == 0xdddddddd || insn == 0xeeeeeeee || insn == 0xffffffff) {
        printf("=== NCR710: Invalid opcode 0x%08x at DSP=0x%08x, stopping SCRIPTS ===\n", insn, s->dsp);
        qemu_log("NCR710: Invalid opcode 0x%08x at DSP=0x%08x, stopping SCRIPTS\n", insn, s->dsp);
        s->scripts.running = false;
        s->script_active = 0;

        /* Set an interrupt to indicate SCRIPTS stopped */
        s->dstat |= 0x04; /* IID - Illegal Instruction Detected */
        s->istat |= 0x01; /* DIP - DMA Interrupt Pending */
        ncr710_update_irq(s);
        return;
    }

    /* Additional bounds checking for the DSP address */
    if (s->dsp >= 0x20000000) {  /* Above reasonable physical memory */
        printf("=== NCR710: DSP address 0x%08x too high, stopping SCRIPTS ===\n", s->dsp);
        qemu_log("NCR710: DSP address 0x%08x too high, stopping SCRIPTS\n", s->dsp);
        s->scripts.running = false;
        s->script_active = 0;
        s->dstat |= 0x04; /* IID - Illegal Instruction Detected */
        s->istat |= 0x01; /* DIP - DMA Interrupt Pending */
        ncr710_update_irq(s);
        return;
    }
    addr = ncr710_read_memory_32(s, s->dsp + 4);
    qemu_log("NCR710: SCRIPTS dsp=%08x opcode %08x arg %08x\n", s->dsp, insn, addr);

    /* Only log every 100th instruction to reduce verbosity */
    if (insn_processed % 100 == 1) {
        printf("=== NCR710: SCRIPTS execute DSP=0x%08x opcode=0x%08x arg=0x%08x (insn %d) ===\n", s->dsp, insn, addr, insn_processed);
    }
    s->dsps = addr;
    s->dcmd = insn >> 24;
    s->dsp += 8;

    switch (insn >> 30) {
    case 0: /* Block move */
        if (s->sstat0 & SSTAT0_STO) {
            NCR710_DPRINTF("Delayed select timeout\n");
            ncr710_stop_script(s);
            break;
        }
        s->dbc = insn & 0xffffff;
        if (insn & (1 << 29)) {
            /* Indirect addressing */
            addr = ncr710_read_memory_32(s, addr);
        } else if (insn & (1 << 28)) {
            uint32_t buf[2];
            int32_t offset;
            /* Table indirect addressing */

            /* 32-bit Table indirect */
            offset = sextract32(addr, 0, 24);
            ncr710_read_memory(s, s->dsa + offset, buf, 8);
            /* byte count is stored in bits 0:23 only */
            s->dbc = le32_to_cpu(buf[0]) & 0xffffff;
            addr = le32_to_cpu(buf[1]);
        }
        if ((s->sstat2 & PHASE_MASK) != ((insn >> 24) & 7)) {
            NCR710_DPRINTF("Wrong phase got %d expected %d\n",
                    s->sstat2 & PHASE_MASK, (insn >> 24) & 7);
            printf("=== NCR710: Phase mismatch - current phase %d, expected phase %d ===\n",
                   s->sstat2 & PHASE_MASK, (insn >> 24) & 7);
            ncr710_script_scsi_interrupt(s, SSTAT0_M_A);
            s->sbcl |= SBCL_REQ;
            break;
        }
        s->dnad = addr;
        printf("=== NCR710: Processing phase %d, DBC=%d ===\n", s->sstat2 & 0x7, s->dbc);
        switch (s->sstat2 & 0x7) {
        case PHASE_DO:
            NCR710_DPRINTF("Data OUT phase, DBC=%d\n", s->dbc);
            s->waiting = 2;
            ncr710_do_dma(s, 1);
            if (s->waiting)
                s->waiting = 3;
            break;
        case PHASE_DI:
            NCR710_DPRINTF("Data IN phase, DBC=%d\n", s->dbc);
            s->waiting = 2;
            ncr710_do_dma(s, 0);
            if (s->waiting)
                s->waiting = 3;
            break;
        case PHASE_CMD:
            NCR710_DPRINTF("Command phase, DBC=%d\n", s->dbc);
            printf("=== NCR710: Command phase, DBC=%d ===\n", s->dbc);
            ncr710_do_command(s);
            break;
        case PHASE_ST:
            ncr710_do_status(s);
            break;
        case PHASE_MO:
            ncr710_do_msgout(s);
            break;
        case PHASE_MI:
            ncr710_do_msgin(s);
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Unimplemented phase %d\n", s->sstat2 & PHASE_MASK);
        }
        s->ctest5 = (s->ctest5 & 0xfc) | ((s->dbc >> 8) & 3);
        s->sbc = s->dbc;
        break;

    case 1: /* IO or Read/Write instruction */
        opcode = (insn >> 27) & 7;
        if (opcode < 5) {
            uint32_t id;

            if (insn & (1 << 25)) {
                id = ncr710_read_memory_32(s, s->dsa + sextract32(insn, 0, 24));
            } else {
                id = insn;
            }
            id = (id >> 16) & 0xff;
            if (insn & (1 << 26)) {
                addr = s->dsp + sextract32(addr, 0, 24);
            }
            s->dnad = addr;
            switch (opcode) {
            case 0: /* Select */
                s->sdid = id;
                if (s->scntl1 & SCNTL1_CON) {
                    NCR710_DPRINTF("Already reselected, jumping to alternative address\n");
                    s->dsp = s->dnad;
                    break;
                }
                s->sstat1 |= SSTAT1_WOA;
                if (!scsi_device_find(&s->bus, 0, ncr710_idbitstonum(id), 0)) {
                    ncr710_bad_selection(s, id);
                    break;
                }
                NCR710_DPRINTF("Selected target %d%s\n",
                        id, insn & (1 << 24) ? " ATN" : "");
                s->select_tag = id << 8;
                s->scntl1 |= SCNTL1_CON;
                if (insn & (1 << 24)) {
                    s->socl |= SOCL_ATN;
                    ncr710_set_phase(s, PHASE_MO);
                } else {
                    ncr710_set_phase(s, PHASE_CMD);
                }
                break;
            case 1: /* Disconnect */
                NCR710_DPRINTF("Wait Disconnect\n");
                s->scntl1 &= ~SCNTL1_CON;
                break;
            case 2: /* Wait Reselect */
                if (!ncr710_irq_on_rsl(s)) {
                    ncr710_wait_reselect(s);
                }
                break;
            case 3: /* Set */
                NCR710_DPRINTF("Set%s%s%s%s\n",
                        insn & (1 << 3) ? " ATN" : "",
                        insn & (1 << 6) ? " ACK" : "",
                        insn & (1 << 9) ? " TM" : "",
                        insn & (1 << 10) ? " CC" : "");
                if (insn & (1 << 3)) {
                    s->socl |= SOCL_ATN;
                    ncr710_set_phase(s, PHASE_MO);
                }
                if (insn & (1 << 9)) {
                    qemu_log_mask(LOG_UNIMP, "NCR710: Target mode not implemented\n");
                }
                if (insn & (1 << 10))
                    s->carry = 1;
                break;
            case 4: /* Clear */
                NCR710_DPRINTF("Clear%s%s%s%s\n",
                        insn & (1 << 3) ? " ATN" : "",
                        insn & (1 << 6) ? " ACK" : "",
                        insn & (1 << 9) ? " TM" : "",
                        insn & (1 << 10) ? " CC" : "");
                if (insn & (1 << 3)) {
                    s->socl &= ~SOCL_ATN;
                }
                if (insn & (1 << 10))
                    s->carry = 0;
                break;
            }
        } else {
            uint8_t op0;
            uint8_t op1;
            uint8_t data8;
            int reg;
            int xoperator;
            static const char *opcode_names[3] =
                {"Write", "Read", "Read-Modify-Write"};
            static const char *operator_names[8] =
                {"MOV", "SHL", "OR", "XOR", "AND", "SHR", "ADD", "ADC"};

            reg = ((insn >> 16) & 0x7f) | (insn & 0x80);
            data8 = (insn >> 8) & 0xff;
            opcode = (insn >> 27) & 7;
            xoperator = (insn >> 24) & 7;
            NCR710_DPRINTF("%s reg 0x%x %s data8=0x%02x sfbr=0x%02x%s\n",
                    opcode_names[opcode - 5], reg,
                    operator_names[xoperator], data8, s->sfbr,
                    (insn & (1 << 23)) ? " SFBR" : "");
            op0 = op1 = 0;
            switch (opcode) {
            case 5: /* From SFBR */
                op0 = s->sfbr;
                op1 = data8;
                break;
            case 6: /* To SFBR */
                if (xoperator)
                    op0 = ncr710_reg_readb(s, reg);
                op1 = data8;
                break;
            case 7: /* Read-modify-write */
                if (xoperator)
                    op0 = ncr710_reg_readb(s, reg);
                if (insn & (1 << 23)) {
                    op1 = s->sfbr;
                } else {
                    op1 = data8;
                }
                break;
            }

            switch (xoperator) {
            case 0: /* move */
                op0 = op1;
                break;
            case 1: /* Shift left */
                op1 = op0 >> 7;
                op0 = (op0 << 1) | s->carry;
                s->carry = op1;
                break;
            case 2: /* OR */
                op0 |= op1;
                break;
            case 3: /* XOR */
                op0 ^= op1;
                break;
            case 4: /* AND */
                op0 &= op1;
                break;
            case 5: /* SHR */
                op1 = op0 & 1;
                op0 = (op0 >> 1) | (s->carry << 7);
                s->carry = op1;
                break;
            case 6: /* ADD */
                op0 += op1;
                s->carry = op0 < op1;
                break;
            case 7: /* ADC */
                op0 += op1 + s->carry;
                if (s->carry)
                    s->carry = op0 <= op1;
                else
                    s->carry = op0 < op1;
                break;
            }

            switch (opcode) {
            case 5: /* From SFBR */
            case 7: /* Read-modify-write */
                ncr710_reg_writeb(s, reg, op0);
                break;
            case 6: /* To SFBR */
                s->sfbr = op0;
                break;
            }
        }
        break;

    case 2: /* Transfer Control */
        {
            int cond;
            int jmp;

            if ((insn & 0x002e0000) == 0) {
                NCR710_DPRINTF("NOP\n");
                break;
            }
            if (s->sstat0 & SSTAT0_STO) {
                NCR710_DPRINTF("Delayed select timeout\n");
                ncr710_stop_script(s);
                break;
            }
            cond = jmp = (insn & (1 << 19)) != 0;
            if (cond == jmp && (insn & (1 << 21))) {
                NCR710_DPRINTF("Compare carry %d\n", s->carry == jmp);
                cond = s->carry != 0;
            }
            if (cond == jmp && (insn & (1 << 17))) {
                NCR710_DPRINTF("Compare phase %d %c= %d\n",
                        (s->sstat2 & PHASE_MASK),
                        jmp ? '=' : '!',
                        ((insn >> 24) & 7));
                cond = (s->sstat2 & PHASE_MASK) == ((insn >> 24) & 7);
            }
            if (cond == jmp && (insn & (1 << 18))) {
                uint8_t mask;

                mask = (~insn >> 8) & 0xff;
                NCR710_DPRINTF("Compare data 0x%x & 0x%x %c= 0x%x\n",
                        s->sfbr, mask, jmp ? '=' : '!', insn & mask);
                cond = (s->sfbr & mask) == (insn & mask);
            }
            if (cond == jmp) {
                if (insn & (1 << 23)) {
                    /* Relative address */
                    addr = s->dsp + sextract32(addr, 0, 24);
                }
                switch ((insn >> 27) & 7) {
                case 0: /* Jump */
                    NCR710_DPRINTF("Jump to 0x%08x\n", addr);
                    s->dsp = addr;
                    break;
                case 1: /* Call */
                    NCR710_DPRINTF("Call 0x%08x\n", addr);
                    printf("=== NCR710: SCRIPTS Call to 0x%08x ===\n", addr);
                    /* Handle special case of calling address 0 (SGScriptStartAddress) */
                    if (addr == 0) {
                        NCR710_DPRINTF("Call to SGScriptStartAddress (0) - handling data transfer\n");
                        printf("=== NCR710: Call to SGScriptStartAddress (0) - returning immediately ===\n");
                        /* This is a scatter-gather data transfer call. For now, let's see if there's
                         * any active data transfer to handle. If the current phase suggests data transfer,
                         * handle it directly rather than calling scripts. */
                        if ((s->sstat2 & PHASE_MASK) == PHASE_DI) {
                            NCR710_DPRINTF("Data IN phase during SG call - returning immediately\n");
                            s->dsp = s->temp;  /* Return immediately */
                        } else if ((s->sstat2 & PHASE_MASK) == PHASE_DO) {
                            NCR710_DPRINTF("Data OUT phase during SG call - returning immediately\n");
                            s->dsp = s->temp;  /* Return immediately */
                        } else {
                            NCR710_DPRINTF("No data phase during SG call - returning immediately\n");
                            s->dsp = s->temp;  /* Return immediately */
                        }
                    } else {
                        s->temp = s->dsp;
                        s->dsp = addr;
                    }
                    break;
                case 2: /* Return */
                    NCR710_DPRINTF("Return to 0x%08x\n", s->temp);
                    s->dsp = s->temp;
                    break;
                case 3: /* Interrupt */
                    NCR710_DPRINTF("Interrupt 0x%08x\n", s->dsps);

                    /* HACK: Map unrecognized interrupt codes to A_GOOD_STATUS_AFTER_STATUS
                     * The kernel's SCRIPTS might be using different interrupt codes than
                     * what the 53c700 driver expects. For now, treat unexpected interrupts
                     * as successful command completion. */
                    if (s->dsps == 0x7348f84d) {
                        printf("=== NCR710: Mapping unrecognized interrupt 0x%08x to A_GOOD_STATUS_AFTER_STATUS (0x401) ===\n", s->dsps);
                        s->dsps = 0x401; /* A_GOOD_STATUS_AFTER_STATUS */
                    } else {
                        printf("=== NCR710: SCRIPTS Interrupt 0x%08x ===\n", s->dsps);
                    }

                    if ((insn & (1 << 20)) != 0) {
                        ncr710_update_irq(s);
                    } else {
                        ncr710_script_dma_interrupt(s, DSTAT_SIR);
                    }
                    break;
                default:
                    NCR710_DPRINTF("Illegal transfer control\n");
                    ncr710_script_dma_interrupt(s, DSTAT_IID);
                    break;
                }
            } else {
                NCR710_DPRINTF("Control condition failed\n");
            }
        }
        break;

    case 3:
        if ((insn & (1 << 29)) == 0) {
            /* Memory move */
            uint32_t dest;
            /* The docs imply the destination address is loaded into
               the TEMP register. However the Linux drivers rely on
               the value being preserved. */
            dest = ncr710_read_memory_32(s, s->dsp);
            s->dsp += 4;
            ncr710_memcpy(s, dest, addr, insn & 0xffffff);
        } else {
            uint8_t data[7];
            int reg;
            int n;
            int i;

            if (insn & (1 << 28)) {
                addr = s->dsa + sextract32(addr, 0, 24);
            }
            n = (insn & 7);
            reg = (insn >> 16) & 0xff;
            if (insn & (1 << 24)) {
                ncr710_read_memory(s, addr, data, n);
                NCR710_DPRINTF("Load reg 0x%x size %d addr 0x%08x = %08x\n", reg, n,
                        addr, *(int *)data);
                for (i = 0; i < n; i++) {
                    ncr710_reg_writeb(s, reg + i, data[i]);
                }
            } else {
                NCR710_DPRINTF("Store reg 0x%x size %d addr 0x%08x\n", reg, n, addr);
                for (i = 0; i < n; i++) {
                    data[i] = ncr710_reg_readb(s, reg + i);
                }
                ncr710_write_memory(s, addr, data, n);
            }
        }
    }

    if (insn_processed > 10000 && !s->waiting) {
        /* Some windows drivers make the device spin waiting for a memory
           location to change. If we have been executed a lot of code then
           assume this is the case and force an unexpected device disconnect.
           This is apparently sufficient to beat the drivers into submission. */
        if (!(s->sien & SIEN_UDC))
            fprintf(stderr, "NCR710: inf. loop with UDC masked\n");
        ncr710_script_scsi_interrupt(s, SSTAT0_UDC);
        ncr710_disconnect(s);
    } else if (s->script_active && !s->waiting) {
        if (s->dcntl & DCNTL_SSM) {
            ncr710_script_dma_interrupt(s, DSTAT_SSI);
        } else {
            goto again;
        }
    }
    NCR710_DPRINTF("SCRIPTS execution stopped\n");
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

static void ncr710_request_cancelled(SCSIRequest *req)
{
    SysBusNCR710State *sysbus_s = container_of(req->bus, SysBusNCR710State, ncr710.bus);
    NCR710State *s = &sysbus_s->ncr710;
    NCR710Request *p = (NCR710Request*)req->hba_private;

    req->hba_private = NULL;
    ncr710_request_free(s, p);
    scsi_req_unref(req);
}

static void ncr710_transfer_data(SCSIRequest *req, uint32_t len)
{
    SysBusNCR710State *sysbus_s = container_of(req->bus, SysBusNCR710State, ncr710.bus);
    NCR710State *s = &sysbus_s->ncr710;
    int out;

    assert(req->hba_private);
    if (s->waiting == 1 || req->hba_private != s->current ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & SCNTL1_CON))) {
        if (ncr710_queue_req(s, req, len)) {
            return;
        }
    }

    out = (s->sstat2 & PHASE_MASK) == PHASE_DO;

    /* host adapter (re)connected */
    NCR710_DPRINTF("Data ready tag=0x%x len=%d\n", req->tag, len);
    s->current->dma_len = len;
    s->command_complete = 1;
    if (s->waiting) {
        if (s->waiting == 1 || s->dbc == 0) {
            ncr710_resume_script(s);
        } else {
            ncr710_do_dma(s, out);
        }
    }
}

static void ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    SysBusNCR710State *sysbus_s = container_of(req->bus, SysBusNCR710State, ncr710.bus);
    NCR710State *s = &sysbus_s->ncr710;
    int out;

    out = (s->sstat2 & PHASE_MASK) == PHASE_DO;
    NCR710_DPRINTF("Command complete status=%d\n", (int)req->status);
    s->lcrc = 0;
    s->status = req->status;
    s->command_complete = 2;
    if (s->waiting && s->dbc != 0) {
        ncr710_bad_phase(s, out, PHASE_ST);
    } else {
        ncr710_set_phase(s, PHASE_ST);
    }

    if (req->hba_private == s->current) {
        req->hba_private = NULL;
        ncr710_request_free(s, s->current);
        scsi_req_unref(req);
    }
    ncr710_resume_script(s);
}



/*  
* QEMU Object Model Registration SysBus NCR710 device 
*/

/* TODO: FIX Draining causing segmentation fault */
// static void ncr710_drained_begin(SCSIBus *bus)
// {
//     SysBusNCR710State *sysbus_s = container_of(bus, SysBusNCR710State, ncr710.bus);
//     NCR710State *s = &sysbus_s->ncr710;

//     if (s->draining) {
//         return;
//     }

//     trace_ncr710_drained_begin();
//     s->draining = true;
//     s->drain_state.was_running = s->scripts.running;
//     s->drain_state.saved_dsp = s->dsp;
//     s->drain_state.saved_waiting = s->waiting;

//     s->scripts.running = false;
//     s->script_active = 0;
//     s->waiting = 0;

//     if (s->selection_timer) {
//         timer_del(s->selection_timer);
//     }
//     if (s->watchdog_timer) {
//         timer_del(s->watchdog_timer);
//     }
//     ncr710_dma_fifo_flush(&s->dma_fifo);

//     qemu_log("NCR710: Drain begin completed\n");
// }

// /* TODO: FIX Draining causing segmentation fault */
// static void ncr710_drained_end(SCSIBus *bus)
// {
//     SysBusNCR710State *sysbus_s = container_of(bus, SysBusNCR710State, ncr710.bus);
//     NCR710State *s = &sysbus_s->ncr710;
//     NCR710Request *req;

//     if (!s || !s->draining) {
//         return;
//     }

//     trace_ncr710_drained_end();
//     s->draining = false;
//     s->waiting = s->drain_state.saved_waiting;
    
//     if (s->drain_state.was_running && s->drain_state.saved_dsp != 0) {
//         s->dsp = s->drain_state.saved_dsp;
//         s->scripts.running = true;
//         ncr710_scripts_execute(s);
//     }
    
//     QTAILQ_FOREACH(req, &s->queue, next) {
//         if (req->pending) {
//             ncr710_reselect(s, req);
//             break;
//         }
//     }

//     qemu_log("NCR710: Drain end completed\n");
// }

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
    .max_lun = 8,     /* LUNs 0-7, lun is buggy in kernel? */

    .transfer_data = ncr710_transfer_data,
    .complete = ncr710_command_complete,
    .cancel = ncr710_request_cancelled,
};

static const MemoryRegionOps ncr710_mmio_ops = {
    .read = ncr710_reg_read,
    .write = ncr710_reg_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
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

    QTAILQ_INIT(&s->ncr710.queue);
    scsi_bus_init(&s->ncr710.bus, sizeof(s->ncr710.bus), dev, &ncr710_scsi_info);
    s->ncr710.as = &address_space_memory;
    ncr710_dma_fifo_init(&s->ncr710.dma_fifo);
    ncr710_scsi_fifo_init(&s->ncr710.scsi_fifo);
    s->ncr710.selection_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, NULL, NULL);
    s->ncr710.watchdog_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, NULL, NULL);
    s->ncr710.ctest8 = NCR710_REVISION_2;  /* NCR710 chip revision 2 */
    s->ncr710.scid = 0x80 | NCR710_DEFAULT_HOST_ID; /* Valid bit + SCSI ID 7 */
    s->ncr710.dstat = DSTAT_DFE;  /* DMA FIFO Empty initially */
    qemu_log("NCR710: Initialized DSTAT to 0x%02x\n", s->ncr710.dstat);
    s->ncr710.dcntl = DCNTL_EA;    /* Enable Ack bit for 710 */
    s->ncr710.dmode = DMODE_FC2;   /* Flow Control 2 for 710 */
    s->ncr710.ctest7 = CTEST7_TT1; /* Transfer Type 1 for 710 */
    s->ncr710.scripts.running = false;
    s->ncr710.scripts.connected = false;
    s->ncr710.scripts.initiator = false;
    s->ncr710.scripts.phase = SCSI_PHASE_DATA_OUT;
    s->ncr710.scripts.pc = 0;
    s->ncr710.scripts.saved_pc = 0;
    s->ncr710.script_active = 0;
    s->ncr710.current = NULL;
    s->ncr710.select_tag = 0;
    s->ncr710.current_lun = 0;
    s->ncr710.command_complete = 0;
    s->ncr710.waiting = 0;
    s->ncr710.carry = 0;
    s->ncr710.status = 0;
    s->ncr710.msg_action = 0;
    s->ncr710.msg_len = 0;
    memset(s->ncr710.msg, 0, sizeof(s->ncr710.msg));
    s->ncr710.selection_timeout_enabled = true;
    s->ncr710.big_endian = true;
    s->ncr710.burst_length = 1;
    s->ncr710.tolerant_enabled = true;
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
    memset(&s->ncr710, 0, sizeof(NCR710State));

    s->ncr710.ctest0 = 0x01;  /* Chip revision 1 */
    s->ncr710.scid = 0x80 | NCR710_DEFAULT_HOST_ID; /* Valid bit + SCSI ID 7 */
    s->ncr710.dstat = DSTAT_DFE;
    qemu_log("NCR710: State initialized\n");
}

static void sysbus_ncr710_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sysbus_ncr710_realize;
    dc->unrealize = sysbus_ncr710_unrealize;
    dc->desc = "NCR53C710 SCSI I/O Processor (SysBus)";
    device_class_set_legacy_reset(dc, ncr710_device_reset);
    dc->bus_type = NULL;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
}

static const TypeInfo sysbus_ncr710_info = {
    .name = TYPE_SYSBUS_NCR710_SCSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusNCR710State),
    .instance_init = sysbus_ncr710_init,
    .class_init = sysbus_ncr710_class_init,
};

static void ncr710_register_types(void)
{
    type_register_static(&sysbus_ncr710_info);
}

type_init(ncr710_register_types)