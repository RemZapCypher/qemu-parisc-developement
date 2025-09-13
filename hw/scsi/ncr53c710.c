/*
 * LASI NCR710 SCSI I/O Processor
 *
 * Copyright (c) 2025 Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 *
 * NCR710 SCSI I/O Processor implementation
 * Based on the NCR53C710 Technical Manual Version 3.2, December 2000
 *
 * Developed from the hackish implementation of NCR53C710 by Helge Deller
 * which was interim based on the hackish implementation by Toni Wilen for UAE
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

#define NCR710_MAX_DEVS 7

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

/* SCNTL0 - SCSI Control Zero (Register 0x00) */
#define SCNTL0_ARB1             0x80    /* Arbitration Mode Bit 1 */
#define SCNTL0_ARB0             0x40    /* Arbitration Mode Bit 0 */
#define SCNTL0_ARB_MASK         0xC0    /* Arbitration Mode Mask */
#define SCNTL0_ARB_SIMPLE       0x00    /* Simple arbitration */
#define SCNTL0_ARB_FULL         0xC0    /* Full arbitration, selection/reselection */
#define NCR_SCNTL0_START            0x20    /* Start Sequence */
#define NCR_SCNTL0_WATN             0x10    /* Select with ATN/ on Start Sequence */
#define NCR_SCNTL0_EPC              0x08    /* Enable Parity Checking */
#define NCR_SCNTL0_EPG              0x04    /* Enable Parity Generation */
#define NCR_SCNTL0_AAP              0x02    /* Assert ATN/ on Parity Error */
#define NCR_SCNTL0_TRG              0x01    /* Target Mode */

/* SCNTL1 - SCSI Control One (Register 0x01) */
#define NCR_SCNTL1_ADB              0x40    /* Assert Data Bus */
#define NCR_SCNTL1_CON              0x10    /* Connected */
#define NCR_SCNTL1_RST              0x08    /* Assert SCSI RST/ */
#define NCR_SCNTL1_AESP             0x04    /* Assert Even SCSI Parity */
#define SCNTL1_EXC              0x80    /* Extra Clock Cycle */
#define SCNTL1_ESR              0x20    /* Enable Selection/Reselection */
#define SCNTL1_SND              0x02    /* Start SCSI Send */
#define SCNTL1_RCV              0x01    /* Start SCSI Receive */

/* SIEN - SCSI Interrupt Enable (Register 0x03) */
#define SIEN_M_A                0x80    /* Phase Mismatch/ATN Active */
#define SIEN_FCMP               0x40    /* Function Complete */
#define SIEN_DIS                0x04    /* Disconnect (alias for UDC) */
#define SIEN_SGE                0x08    /* SCSI Gross Error */
#define NCR_SIEN_STO                0x20    /* SCSI Bus Timeout */
#define NCR_SIEN_SEL                0x10    /* Selected/Reselected */
#define NCR_SIEN_RSL                0x10    /* Reselected (alias for SEL) */
#define NCR_SIEN_UDC                0x04    /* Unexpected Disconnect */
#define NCR_SIEN_RST                0x02    /* SCSI RST/ Received */
#define NCR_SIEN_PAR                0x01    /* Parity Error */

/* SOCL - SCSI Output Control Latch (Register 0x07) */
#define NCR_SOCL_ATN                0x08    /* Assert SCSI ATN/ */
#define SOCL_REQ                0x80    /* Assert SCSI REQ/ */
#define SOCL_ACK                0x40    /* Assert SCSI ACK/ */
#define SOCL_BSY                0x20    /* Assert SCSI BSY/ */
#define SOCL_SEL                0x10    /* Assert SCSI SEL/ */
#define SOCL_MSG                0x04    /* Assert SCSI MSG/ */
#define SOCL_CD                 0x02    /* Assert SCSI C/D/ */
#define SOCL_IO                 0x01    /* Assert SCSI I/O/ */

/* SBCL - SCSI Bus Control Lines (Register 0x0B) */
#define SBCL_ACK                0x40    /* SCSI ACK/ */
#define SBCL_BSY                0x20    /* SCSI BSY/ */
#define SBCL_SEL                0x10    /* SCSI SEL/ */
#define NCR_SBCL_REQ                0x80    /* SCSI REQ/ */
#define NCR_SBCL_ATN                0x08    /* SCSI ATN/ */
#define NCR_SBCL_MSG                0x04    /* SCSI MSG/ */
#define NCR_SBCL_CD                 0x02    /* SCSI C/D/ */
#define NCR_SBCL_IO                 0x01    /* SCSI I/O/ */

/* DSTAT - DMA Status (Register 0x0C) */
#define DSTAT_BF                0x20    /* Bus Fault */
#define DSTAT_MDPE              0x40    /* Master Data Parity Error */
#define DSTAT_WTD               0x02    /* Watchdog Timer Expired */
#define NCR_DSTAT_DFE               0x80    /* DMA FIFO Empty */
#define NCR_DSTAT_IID               0x01    /* Illegal Instruction Detected */
#define NCR_DSTAT_ABRT              0x10    /* Aborted */
#define NCR_DSTAT_SSI               0x08    /* SCRIPTS Step Interrupt */
#define NCR_DSTAT_SIR               0x04    /* SCRIPTS Interrupt */

/* SSTAT0 - SCSI Status Zero (Register 0x0D) */
#define NCR_SSTAT0_M_A              0x80    /* Phase Mismatch/ATN Active */
#define NCR_SSTAT0_FCMP             0x40    /* Function Complete */
#define NCR_SSTAT0_STO              0x20    /* SCSI Bus Timeout */
#define NCR_SSTAT0_SEL              0x10    /* Selected/Reselected */
#define NCR_SSTAT0_RSL              0x10    /* Reselected (alias for SEL) */
#define NCR_SSTAT0_SGE              0x08    /* SCSI Gross Error */
#define NCR_SSTAT0_UDC              0x04    /* Unexpected Disconnect */
#define NCR_SSTAT0_RST              0x02    /* SCSI RST/ Received */
#define NCR_SSTAT0_PAR              0x01    /* Parity Error */

/* SSTAT1 - SCSI Status One (Register 0x0E) */
#define SSTAT1_RST              0x02    /* SCSI RST/ */
#define NCR_SSTAT1_ILF              0x80    /* Input Latch Full */
#define NCR_SSTAT1_ORF              0x40    /* Output Register Full */
#define NCR_SSTAT1_OLF              0x20    /* Output Latch Full */
#define NCR_SSTAT1_AIP              0x10    /* Arbitration In Progress */
#define NCR_SSTAT1_LOA              0x08    /* Lost Arbitration */
#define NCR_SSTAT1_WOA              0x04    /* Won Arbitration */
#define NCR_SSTAT1_SDP              0x01    /* SCSI Parity */

/* SSTAT2 - SCSI Status Two (Register 0x0F) */
#define NCR_SSTAT2_FF3              0x80    /* FIFO Flags bit 3 */
#define NCR_SSTAT2_SDP              0x01    /* SCSI Data Parity */
#define SSTAT2_FF2              0x40    /* FIFO Flags bit 2 */
#define SSTAT2_FF1              0x20    /* FIFO Flags bit 1 */
#define SSTAT2_FF0              0x10    /* FIFO Flags bit 0 */
#define SSTAT2_SPL1             0x08    /* SCSI Phase Latch bit 1 */
#define SSTAT2_SPL0             0x04    /* SCSI Phase Latch bit 0 */
#define SSTAT2_MSP              0x02    /* MSG/ Phase */

/* ISTAT - Interrupt Status (Register 0x21) */
#define NCR_ISTAT_ABRT              0x80    /* Abort Operation */
#define NCR_ISTAT_RST               0x40    /* Software Reset */
#define NCR_ISTAT_SIGP              0x20    /* Signal Process */
#define NCR_ISTAT_CON               0x08    /* Connected */
#define NCR_ISTAT_SIP               0x02    /* SCSI Interrupt Pending */
#define NCR_ISTAT_DIP               0x01    /* DMA Interrupt Pending */
#define ISTAT_INTF              0x04    /* Interrupt on Fly */
#define ISTAT_SEM               0x10    /* Semaphore */

/* DMODE - DMA Mode (Register 0x38) */
#define NCR_DMODE_BL_MASK           0xC0    /* Burst Length Mask */
#define NCR_DMODE_MAN               0x01    /* Manual Start Mode */
#define DMODE_BL1               0x80    /* Burst Length bit 1 */
#define DMODE_BL0               0x40    /* Burst Length bit 0 */
#define DMODE_SIOM              0x20    /* Source I/O Memory Enable */
#define DMODE_DIOM              0x10    /* Destination I/O Memory Enable */
#define DMODE_ERL               0x08    /* Enable Read Line */
#define DMODE_ERMP              0x04    /* Enable Read Multiple */
#define DMODE_BOF               0x02    /* Burst Opcode Fetch */

/* DIEN - DMA Interrupt Enable (Register 0x39) */
#define DIEN_MDPE               0x40    /* Master Data Parity Error */
#define DIEN_BF                 0x20    /* Bus Fault */
#define DIEN_ABRT               0x10    /* Aborted */
#define DIEN_SSI                0x08    /* SCRIPTS Step Interrupt */
#define DIEN_SIR                0x04    /* SCRIPTS Interrupt */
#define DIEN_WTD                0x02    /* Watchdog Timer Expired */
#define DIEN_IID                0x01    /* Illegal Instruction Detected */

/* DCNTL - DMA Control (Register 0x3B) */
#define NCR_DCNTL_COM               0x01    /* 53C700 Compatibility */
#define NCR_DCNTL_PFF               0x40    /* Prefetch Flush */
#define NCR_DCNTL_SSM               0x10    /* Single Step Mode */
#define NCR_DCNTL_STD               0x04    /* Start DMA */
#define DCNTL_CLSE              0x80    /* Cache Line Size Enable */
#define DCNTL_PFEN              0x20    /* Prefetch Enable */
#define DCNTL_IRQM              0x08    /* IRQ Mode */
#define DCNTL_IRQD              0x02    /* IRQ Disable */


/* Maximum length of MSG IN data */
#define NCR710_MAX_MSGIN_LEN    8
#define NCR710_MAX_CDB_SIZE     16

#define NCR710_SCSI_FIFO_SIZE   8
#define NCR710_DMA_FIFO_SIZE    64

#define NCR710_BUF_SIZE 4096

/* SCSI message codes */
#define SCSI_MSG_COMMAND_COMPLETE           0x00
#define SCSI_MSG_EXTENDED_MESSAGE           0x01
#define SCSI_MSG_SAVE_DATA_POINTER          0x02
#define SCSI_MSG_RESTORE_POINTERS           0x03
#define SCSI_MSG_DISCONNECT                 0x04
#define SCSI_MSG_INITIATOR_DET_ERROR        0x05
#define SCSI_MSG_ABORT                      0x06
#define SCSI_MSG_MESSAGE_REJECT             0x07
#define SCSI_MSG_NOP                        0x08
#define SCSI_MSG_MESSAGE_PARITY_ERROR       0x09
#define SCSI_MSG_LINKED_CMD_COMPLETE        0x0A
#define SCSI_MSG_LINKED_FLG_CMD_COMPLETE    0x0B
#define SCSI_MSG_BUS_DEVICE_RESET           0x0C

/* This might not be necessary lets see */
enum {
    NCR_NOWAIT, /* SCRIPTS are running or stopped */
    NCR_WAIT_RESELECT, /* Wait Reselect instruction has been issued */
    NCR_DMA_SCRIPTS, /* processing DMA from ncr710_execute_script */
    NCR_DMA_IN_PROGRESS, /* DMA operation is in progress */
    NCR_WAIT_SCRIPTS, /* SCRIPTS stopped because of instruction count limit */
};

enum {
    NCR710_MSG_ACTION_COMMAND    = 0,
    NCR710_MSG_ACTION_DISCONNECT = 1,
    NCR710_MSG_ACTION_DOUT       = 2,
    NCR710_MSG_ACTION_DIN        = 3,
};

#define PHASE_DO          0
#define PHASE_DI          1
#define PHASE_CMD         2
#define PHASE_ST          3
#define PHASE_MO          6
#define PHASE_MI          7
#define PHASE_MASK        7

#define NCR710_REVISION_2       0x20
#define NCR710_DEFAULT_HOST_ID  7

#define CTEST7_TT1              0x02    /* Transfer Type 1 */
#define CTEST7_DIFF             0x01    /* Differential SCSI */
#define DCNTL_EA                0x20    /* Enable Ack */
#define DMODE_FC2               0x02    /* Flow Control 2 */

#define NCR710_TAG_VALID     (1 << 16)

// static void ncr710_dma_transfer(NCR710State *s)
// {
//     /* Basic DMA transfer implementation */
//     if (!s->dbc || !s->dnad) {
//         /* No transfer to perform */
//         s->dstat |= NCR_DSTAT_DFE;  /* DMA FIFO Empty */
//         trace_ncr710_dma_abort("no data to transfer");
//         return;
//     }

//     trace_ncr710_dma_start(s->dnad, s->dbc, s->scripts.phase);

//     /* For now, just mark DMA as completed */
//     uint32_t transferred = s->dbc;
//     s->dbc = 0;  /* Clear byte counter */
//     s->dstat |= NCR_DSTAT_DFE;  /* Set DMA FIFO Empty */

//     trace_ncr710_dma_complete(transferred);

//     /* Update interrupt status if enabled */
//     ncr710_update_irq(s);
// }

// static void ncr710_scsi_command_start(NCR710State *s, uint8_t target, uint8_t lun, uint8_t *cdb, uint32_t cdb_len)
// {
//     SCSIDevice *d;
//     SCSIRequest *req;

//     NCR710_DPRINTF("NCR710: Starting SCSI command - target=%d, lun=%d, opcode=0x%02x\n",
//              target, lun, cdb[0]);

//     /* Find the SCSI device */
//     d = scsi_device_find(&s->bus, 0, target, lun);
//     if (!d) {
//         NCR710_DPRINTF("NCR710: No device found at target %d, lun %d\n", target, lun);
//         /* Set selection timeout */
//         s->sstat0 |= NCR_SSTAT0_STO;  /* Selection Timeout */
//         if (s->sien & NCR_SIEN_STO) {
//             s->istat |= NCR_ISTAT_SIP;
//         }
//         ncr710_update_irq(s);
//         return;
//     }

//     /* Create and start the SCSI request */
//     req = scsi_req_new(d, 0, lun, cdb, cdb_len, s);
//     if (!req) {
//         NCR710_DPRINTF("NCR710: Failed to create SCSI request\n");
//         return;
//     }

//     s->current_req = req;
//     s->scripts.connected = true;

//     /* Start the command */
//     scsi_req_enqueue(req);
//     NCR710_DPRINTF("NCR710: SCSI command enqueued successfully\n");
// }

// static uint32_t ncr710_scsi_fifo_fill_from_memory(NCR710State *s, uint32_t addr, uint32_t max_count)
// {
//     uint32_t bytes_transferred = 0;
//     uint32_t fifo_space = NCR710_SCSI_FIFO_SIZE - s->scsi_fifo.count;
//     uint32_t transfer_count = (max_count < fifo_space) ? max_count : fifo_space;

//     /* Fill FIFO with data from memory */
//     for (uint32_t i = 0; i < transfer_count; i++) {
//         uint8_t data;
//         ncr710_read_memory(s, addr + i, &data, 1);

//         uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
//                         ncr710_generate_scsi_parity(s, data) : 0;
//         ncr710_scsi_fifo_push(&s->scsi_fifo, data, parity);
//         bytes_transferred++;
//     }

//     return bytes_transferred;
// }

// static uint32_t ncr710_scsi_fifo_drain_to_buffer(NCR710State *s, uint8_t *buffer, uint32_t max_count)
// {
//     uint32_t bytes_transferred = 0;
//     uint32_t available = s->scsi_fifo.count;
//     uint32_t transfer_count = (max_count < available) ? max_count : available;

//     /* Drain FIFO to buffer */
//     for (uint32_t i = 0; i < transfer_count; i++) {
//         uint8_t fifo_parity;
//         buffer[i] = ncr710_scsi_fifo_pop(&s->scsi_fifo, &fifo_parity);
//         bytes_transferred++;

//         /* Check parity if enabled */
//         if ((s->scntl0 & NCR_SCNTL0_EPC) && !ncr710_check_scsi_parity(s, buffer[i], fifo_parity)) {
//             ncr710_handle_parity_error(s);
//         }

//         /* Store first byte in SFBR */
//         if (i == 0) {
//             s->sfbr = buffer[i];
//         }
//     }

//     return bytes_transferred;
// }

// static uint32_t ncr710_scsi_fifo_fill_from_buffer(NCR710State *s, const uint8_t *buffer, uint32_t max_count)
// {
//     uint32_t bytes_transferred = 0;
//     uint32_t fifo_space = NCR710_SCSI_FIFO_SIZE - s->scsi_fifo.count;
//     uint32_t transfer_count = (max_count < fifo_space) ? max_count : fifo_space;

//     /* Fill FIFO from buffer */
//     for (uint32_t i = 0; i < transfer_count; i++) {
//         uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
//                         ncr710_generate_scsi_parity(s, buffer[i]) : 0;
//         ncr710_scsi_fifo_push(&s->scsi_fifo, buffer[i], parity);
//         bytes_transferred++;

//         /* Store first byte in SFBR */
//         if (i == 0) {
//             s->sfbr = buffer[i];
//         }
//     }

//     return bytes_transferred;
// }

// static uint32_t ncr710_scsi_fifo_drain_to_memory(NCR710State *s, uint32_t addr, uint32_t max_count)
// {
//     uint32_t bytes_transferred = 0;
//     uint32_t available = s->scsi_fifo.count;
//     uint32_t transfer_count = (max_count < available) ? max_count : available;

//     /* Drain FIFO to memory */
//     for (uint32_t i = 0; i < transfer_count; i++) {
//         uint8_t data, fifo_parity;
//         data = ncr710_scsi_fifo_pop(&s->scsi_fifo, &fifo_parity);
//         ncr710_write_memory(s, addr + i, &data, 1);
//         bytes_transferred++;

//         /* Check parity if enabled */
//         if ((s->scntl0 & NCR_SCNTL0_EPC) && !ncr710_check_scsi_parity(s, data, fifo_parity)) {
//             ncr710_handle_parity_error(s);
//         }

//         /* Store first/last bytes in registers */
//         if (i == 0) {
//             s->sfbr = data;
//         }
//         if (i == transfer_count - 1) {
//             s->sidl = data;
//         }
//     }

//     return bytes_transferred;
// }

/* DEBUG: Helper for register name mapping */
static const char *ncr710_reg_name(int offset)
{
    switch (offset) {
    case NCR710_SCNTL0_REG:  return "SCNTL0";
    case NCR710_SCNTL1_REG:  return "SCNTL1";
    case NCR710_SDID_REG:    return "SDID";
    case NCR710_SIEN_REG:    return "SIEN";
    case NCR710_SCID_REG:    return "SCID";
    case NCR710_SXFER_REG:   return "SXFER";
    case NCR710_SODL_REG:    return "SODL";
    case NCR710_SOCL_REG:    return "SOCL";
    case NCR710_SFBR_REG:    return "SFBR";
    case NCR710_SIDL_REG:    return "SIDL";
    case NCR710_SBDL_REG:    return "SBDL";
    case NCR710_SBCL_REG:    return "SBCL";
    case NCR710_DSTAT_REG:   return "DSTAT";
    case NCR710_SSTAT0_REG:  return "SSTAT0";
    case NCR710_SSTAT1_REG:  return "SSTAT1";
    case NCR710_SSTAT2_REG:  return "SSTAT2";
    case NCR710_DSA_REG:     return "DSA";
    case NCR710_DSA_REG+1:   return "DSA+1";
    case NCR710_DSA_REG+2:   return "DSA+2";
    case NCR710_DSA_REG+3:   return "DSA+3";
    case NCR710_CTEST0_REG:  return "CTEST0";
    case NCR710_CTEST1_REG:  return "CTEST1";
    case NCR710_CTEST2_REG:  return "CTEST2";
    case NCR710_CTEST3_REG:  return "CTEST3";
    case NCR710_CTEST4_REG:  return "CTEST4";
    case NCR710_CTEST5_REG:  return "CTEST5";
    case NCR710_CTEST6_REG:  return "CTEST6";
    case NCR710_CTEST7_REG:  return "CTEST7";
    case NCR710_TEMP_REG:    return "TEMP";
    case NCR710_TEMP_REG+1:  return "TEMP+1";
    case NCR710_TEMP_REG+2:  return "TEMP+2";
    case NCR710_TEMP_REG+3:  return "TEMP+3";
    case NCR710_DFIFO_REG:   return "DFIFO";
    case NCR710_ISTAT_REG:   return "ISTAT";
    case NCR710_CTEST8_REG:  return "CTEST8";
    case NCR710_LCRC_REG:    return "LCRC";
    case NCR710_DBC_REG:     return "DBC";
    case NCR710_DBC_REG+1:   return "DBC+1";
    case NCR710_DBC_REG+2:   return "DBC+2";
    case NCR710_DCMD_REG:    return "DCMD";
    case NCR710_DNAD_REG:    return "DNAD";
    case NCR710_DNAD_REG+1:  return "DNAD+1";
    case NCR710_DNAD_REG+2:  return "DNAD+2";
    case NCR710_DNAD_REG+3:  return "DNAD+3";
    case NCR710_DSP_REG:     return "DSP";
    case NCR710_DSP_REG+1:   return "DSP+1";
    case NCR710_DSP_REG+2:   return "DSP+2";
    case NCR710_DSP_REG+3:   return "DSP+3";
    case NCR710_DSPS_REG:    return "DSPS";
    case NCR710_DSPS_REG+1:  return "DSPS+1";
    case NCR710_DSPS_REG+2:  return "DSPS+2";
    case NCR710_DSPS_REG+3:  return "DSPS+3";
    case NCR710_SCRATCH_REG: return "SCRATCH";
    case NCR710_SCRATCH_REG+1: return "SCRATCH+1";
    case NCR710_SCRATCH_REG+2: return "SCRATCH+2";
    case NCR710_SCRATCH_REG+3: return "SCRATCH+3";
    case NCR710_DMODE_REG:   return "DMODE";
    case NCR710_DIEN_REG:    return "DIEN";
    case NCR710_DWT_REG:     return "DWT";
    case NCR710_DCNTL_REG:   return "DCNTL";
    case NCR710_ADDER_REG:   return "ADDER";
    case NCR710_ADDER_REG+1: return "ADDER+1";
    case NCR710_ADDER_REG+2: return "ADDER+2";
    case NCR710_ADDER_REG+3: return "ADDER+3";
    default:                 return "UNKNOWN";
    }
}

static void ncr710_resume_script(NCR710State *s)
{
    if (s->waiting != 2) {
        s->waiting = 0;
        ncr710_scripts_execute(s);
    } else {
        s->waiting = NCR_NOWAIT;
    }
}

static void ncr710_disconnect(NCR710State *s)
{
    s->scntl1 &= ~NCR_SCNTL1_CON;
    s->sstat2 &= ~PHASE_MASK;
    s->scripts.connected = false;
}

static void ncr710_bad_selection(NCR710State *s, uint32_t id)
{
    NCR710_DPRINTF("Selected absent target %d\n", id);
    ncr710_script_scsi_interrupt(s, NCR_SSTAT0_STO);
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

/* Perform reselection to continue a command.  */
static void ncr710_reselect(NCR710State *s, NCR710Request *p)
{
    int id;

    assert(s->current == NULL);
    QTAILQ_REMOVE(&s->queue, p, next);
    s->current = p;

    id = (p->tag >> 8) & 0xf;

    /* LSI53C700 Family Compatibility, see NCR53C710 4-73 */
    if (!(s->dcntl & NCR_DCNTL_COM)) {
        s->sfbr = 1 << (id & 0x7);
    }
    s->lcrc = 0;

    NCR710_DPRINTF("Reselected target %d\n", id);
    s->scntl1 |= NCR_SCNTL1_CON;
    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = p->out ? 2 : 3;
    s->current->dma_len = p->pending;

    ncr710_add_msg_byte(s, 0x80);
    if (s->current->tag & NCR710_TAG_VALID) {
        ncr710_add_msg_byte(s, 0x20);
        ncr710_add_msg_byte(s, p->tag & 0xff);
    }

    if (ncr710_irq_on_rsl(s)) {
        ncr710_script_scsi_interrupt(s, NCR_SSTAT0_SEL);
    }
}


static void ncr710_wait_reselect(NCR710State *s)
{
    NCR710Request *p;

    NCR710_DPRINTF("Wait Reselect\n");
    trace_ncr710_scripts_wait_reselect();

    /* NCR710: Enhanced reselection for kernel driver compatibility */
    QTAILQ_FOREACH(p, &s->queue, next) {
        if (p->pending) {
            /* Found a pending request - perform reselection */
            ncr710_reselect(s, p);

            /* NCR710: Kernel drivers expect immediate reselection interrupt */
            if (s->sien & NCR_SIEN_RSL) {
                s->sstat0 |= NCR_SSTAT0_RSL; /* Reselected */
                s->istat |= NCR_ISTAT_SIP;   /* SCSI Interrupt Pending */
                ncr710_update_irq(s);
            }

            /* SCRIPTS should continue from reselection point */
            s->scripts.running = true;
            return;
        }
    }

    /* No pending reselection - enter wait state */
    if (s->current == NULL) {
        s->waiting = 1;
        /* NCR710: Halt SCRIPTS execution until reselection occurs */
        s->scripts.running = false;
        s->script_active = 0;

        /* Kernel drivers may poll for reselection - set up for it */
        s->sstat2 |= NCR_SSTAT2_FF3; /* FIFO flags for reselection */
    }
}

static void ncr710_do_dma(NCR710State *s, int out)
{
    uint32_t count, addr;
    SCSIDevice *dev;

    if (!s->current) {
        NCR710_DPRINTF("DMA called with no current request - may be device discovery\n");

        out = (s->sstat2 & PHASE_MASK) != PHASE_DI;
        // For device discovery or SCRIPTS testing, perform a dummy transfer
        if (s->dbc > 0) {
            count = s->dbc;
            addr = s->dnad;

            if (out) {
                uint8_t dummy_buf[256];
                uint32_t chunk = (count > sizeof(dummy_buf)) ? sizeof(dummy_buf) : count;
                ncr710_read_memory(s, addr, dummy_buf, chunk);

                // Use the proper DMA FIFO functions
                for (uint32_t i = 0; i < chunk && !ncr710_dma_fifo_full(&s->dma_fifo); i++) {
                    uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
                                   ncr710_generate_scsi_parity(s, dummy_buf[i]) : 0;
                    ncr710_dma_fifo_push(&s->dma_fifo, dummy_buf[i], parity);
                }

                count = chunk;
            } else {
                // Write zeros to memory from DMA FIFO or generate zeros
                uint8_t zero_buf[256] = {0};
                uint32_t chunk = (count > sizeof(zero_buf)) ? sizeof(zero_buf) : count;

                // If we have DMA FIFO data, use it, otherwise use zeros
                for (uint32_t i = 0; i < chunk; i++) {
                    if (!ncr710_dma_fifo_empty(&s->dma_fifo)) {
                        uint8_t parity;
                        zero_buf[i] = ncr710_dma_fifo_pop(&s->dma_fifo, &parity);
                    }
                }

                ncr710_write_memory(s, addr, zero_buf, chunk);
                count = chunk;
            }

            // Update DMA state
            s->dnad += count;
            s->dbc -= count;

            NCR710_DPRINTF("Dummy DMA transfer: %d bytes, %s\n",
                          count, out ? "OUT" : "IN");
        }

        // Continue SCRIPTS execution
        ncr710_resume_script(s);
        return;
    }

    // Normal DMA path with existing request
    if (!s->current->req) {
        NCR710_DPRINTF("DMA called with no SCSI request\n");
        return;
    }

    if (!s->current->dma_len) {
        NCR710_DPRINTF("DMA no data available\n");
        return;
    }

    dev = s->current->req->dev;
    if (!dev) {
        NCR710_DPRINTF("DMA called with no device\n");
        return;
    }

    count = s->dbc;
    if (count > s->current->dma_len) {
        count = s->current->dma_len;
    }

    addr = s->dnad;
    NCR710_DPRINTF("DMA addr=0x%08x len=%d out=%d\n", addr, count, out);

    if (s->current->dma_buf == NULL) {
        s->current->dma_buf = scsi_req_get_buf(s->current->req);
    }

    if (out) {
        // Data OUT: Memory -> DMA FIFO -> Device
        count = ncr710_dma_fifo_fill_from_buffer(s, s->current->dma_buf, count);
        // Then drain from memory to DMA FIFO
        uint8_t temp_buf[256];
        uint32_t chunk = (count > sizeof(temp_buf)) ? sizeof(temp_buf) : count;
        ncr710_read_memory(s, addr, temp_buf, chunk);
        ncr710_dma_fifo_drain_to_buffer(s, s->current->dma_buf, chunk);
        count = chunk;
    } else {
        // Data IN: Device -> DMA FIFO -> Memory
        count = ncr710_dma_fifo_fill_from_buffer(s, s->current->dma_buf, count);
        count = ncr710_dma_fifo_drain_to_memory(s, addr, count);
    }

    // Update DMA state
    s->dnad += count;
    s->dbc -= count;
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

    NCR710_DPRINTF("NCR710_DEBUG: ncr710_do_command() called - len=%d, select_tag=0x%08x\n", s->dbc, s->select_tag);
    NCR710_DPRINTF("Send command len=%d\n", s->dbc);

    if (s->dbc > NCR710_MAX_CDB_SIZE) {
        s->dbc = NCR710_MAX_CDB_SIZE;
    }

    if (s->dbc == 0) {
        NCR710_DPRINTF("Zero-length command\n");
        ncr710_script_scsi_interrupt(s, NCR_SSTAT0_SGE);
        return;
    }

    // Read command through SCSI FIFO for proper emulation
    ncr710_read_memory(s, s->dnad, buf, s->dbc);

    // Update SCSI FIFO with command bytes
    for (int i = 0; i < s->dbc && !ncr710_scsi_fifo_full(&s->scsi_fifo); i++) {
        uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
                        ncr710_generate_scsi_parity(s, buf[i]) : 0;
        ncr710_scsi_fifo_push(&s->scsi_fifo, buf[i], parity);
    }

    NCR710_DPRINTF("Send command len=%d %02x.%02x.%02x.%02x.%02x.%02x\n",
                   s->dbc, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    // Store first byte in SFBR register
    s->sfbr = buf[0];
    s->command_complete = 0;

    // Extract target ID
    id = (s->select_tag >> 8) & 0xff;
    s->lcrc = id;

    NCR710_DPRINTF("NCR710_DEBUG: Extracted target ID - raw_id=0x%02x, target_num=%d, current_lun=%d\n",
           id, ncr710_idbitstonum(id), s->current_lun);

    // Find the target device
    dev = scsi_device_find(&s->bus, 0, ncr710_idbitstonum(id), s->current_lun);
    if (!dev) {
        NCR710_DPRINTF("NCR710_DEBUG: Target %d NOT FOUND - calling ncr710_bad_selection()\n", ncr710_idbitstonum(id));
        NCR710_DPRINTF("Target %d not found\n", ncr710_idbitstonum(id));
        ncr710_bad_selection(s, id);
        return;
    }

    NCR710_DPRINTF("NCR710_DEBUG: Target %d FOUND - device=%p, proceeding with command\n", ncr710_idbitstonum(id), dev);

    // Create new request structure
    if (s->current != NULL) {
        ncr710_request_free(s, s->current);
        s->current = NULL;
    }

    s->current = g_new0(NCR710Request, 1);
    s->current->tag = s->select_tag;

    // Create SCSI request
    s->current->req = scsi_req_new(dev, s->current->tag, s->current_lun, buf, s->dbc, s->current);
    if (!s->current->req) {
        NCR710_DPRINTF("NCR710_DEBUG: FAILED to create SCSI request\n");
        NCR710_DPRINTF("Failed to create SCSI request\n");
        g_free(s->current);
        s->current = NULL;
        ncr710_script_scsi_interrupt(s, NCR_SSTAT0_SGE);
        return;
    }

    NCR710_DPRINTF("NCR710_DEBUG: Created SCSI request successfully - req=%p, opcode=0x%02x\n",
           s->current->req, buf[0]);

    QTAILQ_INSERT_TAIL(&s->queue, s->current, next);
    n = scsi_req_enqueue(s->current->req);

    NCR710_DPRINTF("NCR710_DEBUG: scsi_req_enqueue() returned n=%d\n", n);

    if (n) {
        if (n > 0) {
            NCR710_DPRINTF("NCR710_DEBUG: Setting PHASE_DI (data in)\n");
            ncr710_set_phase(s, PHASE_DI);
        } else {
            NCR710_DPRINTF("NCR710_DEBUG: Setting PHASE_DO (data out)\n");
            ncr710_set_phase(s, PHASE_DO);
        }
        NCR710_DPRINTF("NCR710_DEBUG: Calling scsi_req_continue()\n");
        scsi_req_continue(s->current->req);

        if (!s->command_complete) {
            NCR710_DPRINTF("NCR710_DEBUG: Command not complete - adding disconnect messages\n");
            ncr710_add_msg_byte(s, SCSI_MSG_SAVE_DATA_POINTER);
            ncr710_add_msg_byte(s, SCSI_MSG_DISCONNECT);
            ncr710_set_phase(s, PHASE_MI);
            s->msg_action = NCR710_MSG_ACTION_DISCONNECT;
        } else {
            NCR710_DPRINTF("NCR710_DEBUG: Command already complete\n");
        }
    } else {
        NCR710_DPRINTF("NCR710_DEBUG: No data transfer needed - setting PHASE_ST\n");
        ncr710_set_phase(s, PHASE_ST);
    }
}


static void ncr710_do_status(NCR710State *s)
{

    uint8_t status = s->status;
    for (int i = 0; i < s->dbc; i++) {
        ncr710_write_memory(s, s->dnad + i, &status, 1);
    }
    s->dnad += s->dbc;
    s->dbc = 0;

    status = s->status;

    // Store in SCSI FIFO for proper emulation
    if (!ncr710_scsi_fifo_full(&s->scsi_fifo)) {
        uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
                        ncr710_generate_scsi_parity(s, status) : 0;
        ncr710_scsi_fifo_push(&s->scsi_fifo, status, parity);
    }

    // Transfer from SCSI FIFO to memory
    if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
        uint8_t parity;
        uint8_t fifo_byte = ncr710_scsi_fifo_pop(&s->scsi_fifo, &parity);
        ncr710_write_memory(s, s->dnad, &fifo_byte, 1);
    }

    // Transition to Message In phase
    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = NCR710_MSG_ACTION_COMMAND;
    ncr710_add_msg_byte(s, NCR_MSG_ACTION_COMMAND);

    NCR710_DPRINTF("Status byte 0x%02x sent\n", status);
}

static void ncr710_do_msgin(NCR710State *s)
{
    int len;

    NCR710_DPRINTF("Message in len=%d/%d\n", s->dbc, s->msg_len);

    if (s->msg_len == 0) {
        NCR710_DPRINTF("No message data available\n");
        return;
    }

    len = s->msg_len;
    if (len > s->dbc) {
        len = s->dbc;
    }

    // Transfer message through SCSI FIFO
    for (int i = 0; i < len && !ncr710_scsi_fifo_full(&s->scsi_fifo); i++) {
        uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
                        ncr710_generate_scsi_parity(s, s->msg[i]) : 0;
        ncr710_scsi_fifo_push(&s->scsi_fifo, s->msg[i], parity);
    }

    // Transfer from SCSI FIFO to memory
    uint8_t temp_buf[256];
    int transferred = 0;
    while (!ncr710_scsi_fifo_empty(&s->scsi_fifo) && transferred < len) {
        uint8_t parity;
        temp_buf[transferred] = ncr710_scsi_fifo_pop(&s->scsi_fifo, &parity);
        transferred++;
    }

    if (transferred > 0) {
        ncr710_write_memory(s, s->dnad, temp_buf, transferred);
    }

    // Update message state
    s->msg_len -= len;
    if (s->msg_len) {
        memmove(s->msg, s->msg + len, s->msg_len);
    } else {
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
            NCR710_DPRINTF("NCR710: Invalid message action %d\n", s->msg_action);
            ncr710_script_scsi_interrupt(s, NCR_SSTAT0_SGE);
            break;
        }
    }
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
        // Read through SCSI FIFO for proper emulation
        uint8_t temp_byte;
        ncr710_read_memory(s, s->dnad, &temp_byte, 1);

        // Put in SCSI FIFO
        if (!ncr710_scsi_fifo_full(&s->scsi_fifo)) {
            uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
                            ncr710_generate_scsi_parity(s, temp_byte) : 0;
            ncr710_scsi_fifo_push(&s->scsi_fifo, temp_byte, parity);
        }

        // Get from SCSI FIFO
        if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
            uint8_t parity;
            msg = ncr710_scsi_fifo_pop(&s->scsi_fifo, &parity);
        } else {
            msg = temp_byte;
        }

        s->dnad += 1;
        s->dbc -= 1;

        // Process message as before (rest of the function stays the same)
        switch (msg) {
        case 0x00: /* COMMAND_COMPLETE (unexpected in msgout, but handle) */
            NCR710_DPRINTF("MSG: Command Complete\n");
            break;
        case SCSI_MSG_DISCONNECT:
            NCR710_DPRINTF("MSG: Disconnect\n");
            ncr710_disconnect(s);
            break;
        case SCSI_MSG_NOP:
            NCR710_DPRINTF("MSG: No Operation\n");
            ncr710_set_phase(s, PHASE_CMD);
            break;
        case NCR_MSG_ACTION_DISCONNECT:
            // Read extended message length
            ncr710_read_memory(s, s->dnad, &len, 1);
            s->dnad += 1;
            s->dbc -= 1;

            // Read extended message type
            ncr710_read_memory(s, s->dnad, &msg, 1);
            s->dnad += 1;
            s->dbc -= 1;

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
            {
                uint8_t tag;
                ncr710_read_memory(s, s->dnad, &tag, 1);
                s->dnad += 1;
                s->dbc -= 1;
                s->select_tag |= tag | NCR710_TAG_VALID;
                NCR710_DPRINTF("SIMPLE queue tag=0x%x\n", s->select_tag & 0xff);
            }
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
            NCR710_DPRINTF("MSG: ABORT TAG tag=0x%x\n", current_tag);
            if (current_req) {
                scsi_req_cancel(current_req->req);
            }
            ncr710_disconnect(s);
            break;
        case SCSI_MSG_ABORT:
        case 0x0e: /* CLEAR QUEUE */
        case SCSI_MSG_BUS_DEVICE_RESET:
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
    NCR710_DPRINTF("NCR710: Unimplemented message 0x%02x\n", msg);
    ncr710_set_phase(s, PHASE_MI);
    ncr710_add_msg_byte(s, 7); /* MESSAGE REJECT */
    s->msg_action = 0;
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

/* Calculate parity according to NCR710 spec:
 * - Default is odd parity
 * - NCR_SCNTL1_AESP controls even/odd parity assertion
 */
static uint8_t ncr710_generate_scsi_parity(NCR710State *s, uint8_t data)
{
    uint8_t parity = ncr710_calculate_parity(data);

    if (s->scntl1 & NCR_SCNTL1_AESP) {
        parity = !parity;  /* Even parity */
    }

    return parity;
}

static bool ncr710_check_scsi_parity(NCR710State *s, uint8_t data, uint8_t received_parity)
{
    if (!(s->scntl0 & NCR_SCNTL0_EPC)) {
        return true; /* No parity checking - assume OK */
    }

    uint8_t expected_parity = ncr710_generate_scsi_parity(s, data);
    return (expected_parity == received_parity);
}

static void ncr710_handle_parity_error(NCR710State *s)
{
    uint8_t calculated_parity = ncr710_calculate_parity(s->sfbr);
    uint8_t expected_parity = (s->scntl1 & NCR_SCNTL1_AESP) ? 0 : 1; /* Even/Odd parity */
    trace_ncr710_parity_error(s->sfbr, expected_parity, calculated_parity);

    s->sstat0 |= NCR_SSTAT0_PAR;

    if (s->scntl0 & NCR_SCNTL0_AAP) {
        s->socl |= NCR_SOCL_ATN;
        trace_ncr710_atn_asserted_parity_error();
    }

    if (s->sien & NCR_SIEN_PAR) {
        ncr710_script_scsi_interrupt(s, NCR_SSTAT0_PAR);
    }
}

/* Arbitration just always wins: This is here just for the formalities */
static void ncr710_arbitrate_bus(NCR710State *s)
{
    /* Basic bus arbitration implementation */
    if (s->scntl0 & NCR_SCNTL0_START) {
        /* Start sequence initiated */
        trace_ncr710_arbitrate_start();
        s->scripts.connected = false;
        s->scripts.initiator = true;
        trace_ncr710_connected(false);

        /* Set arbitration in progress */
        s->sstat1 |= NCR_SSTAT1_AIP;

        /* For now, assume we always win arbitration */
        s->sstat1 |= NCR_SSTAT1_WOA;
        s->sstat1 &= ~NCR_SSTAT1_LOA;
        s->sstat1 &= ~NCR_SSTAT1_AIP;
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
            NCR710_DPRINTF("NCR710: Selecting SCSI target %d\n", target_id);

            SCSIDevice *scsi_dev = scsi_device_find(&s->bus, 0, target_id, 0);
            if (scsi_dev) {
                NCR710_DPRINTF("NCR710: Found SCSI device at target %d\n", target_id);
                s->sstat0 |= NCR_SSTAT0_SEL;  /* Selected */
                s->scntl1 |= NCR_SCNTL1_CON;  /* Connected */
                s->scripts.connected = true;

                if (s->socl & NCR_SOCL_ATN) {
                    ncr710_set_phase(s, PHASE_MO);
                    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | PHASE_MO;
                } else {
                    ncr710_set_phase(s, PHASE_CMD);
                    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | PHASE_CMD;
                }

                trace_ncr710_connected(true);
                trace_ncr710_selection_success(target_id);
            } else {
                NCR710_DPRINTF("NCR710: No SCSI device found at target %d\n", target_id);
                s->sstat0 |= NCR_SSTAT0_STO;  /* Selection timeout */
                trace_ncr710_selection_timeout(target_id);
            }
        }

        s->scntl0 &= ~NCR_SCNTL0_START;
        ncr710_update_irq(s);
    }
}

static void ncr710_script_dma_interrupt(NCR710State *s, int stat)
{
    NCR710_DPRINTF("DMA Interrupt 0x%x prev 0x%x\n", stat, s->dstat);
    s->dstat |= stat;
    s->istat |= NCR_ISTAT_DIP; /* DMA Interrupt Pending */
    if (stat & NCR_DSTAT_SIR) {
        trace_ncr710_scripts_interrupt(s->dsps, s->dsp);
    }
    ncr710_update_irq(s);
    ncr710_stop_script(s);
}

static inline uint32_t read_dword(NCR710State *s, uint32_t addr)
{
    uint32_t buf;
    ncr710_read_memory(s, addr, &buf, 4);
    return le32_to_cpu(buf);
}

static void ncr710_read_memory(NCR710State *s, uint32_t addr, void *buf, int len)
{
    if (!s->as) {
        s->as = &address_space_memory;
    }
    if (address_space_read(s->as, addr, MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Memory read error at 0x%x\n", addr);
        memset(buf, 0, len);  // Fill with zeros on error
    }
}

static void ncr710_write_memory(NCR710State *s, uint32_t addr, const void *buf, int len)
{
    if (!s->as) {
        s->as = &address_space_memory;
    }

    if (address_space_write(s->as, addr, MEMTXATTRS_UNSPECIFIED, buf, len) != MEMTX_OK) {
    } else {
        if (len == 4) {
            NCR710_DPRINTF("NCR710: DMA write SUCCESS at 0x%08x value=0x%08x len=%d\n",
                     addr, *(const uint32_t*)buf, len);
        }
    }
}

static void ncr710_scripts_execute(NCR710State *s)
{
    uint32_t insn;
    uint32_t addr;
    int opcode;
    int insn_processed = 0;

    s->script_active = 1;
again:
    insn_processed++;
    insn = read_dword(s, s->dsp);
    if (!insn) {
        /* If we receive an empty opcode increment the DSP by 4 bytes
           instead of 8 and execute the next opcode at that location */
        s->dsp += 4;
        goto again;
    }
    addr = read_dword(s, s->dsp + 4);
    printf("NCR710_DEBUG: Script instruction - DSP=0x%x, opcode=0x%x, arg=0x%x, type=%d\n",
           s->dsp, insn, addr, insn >> 30);
    NCR710_DPRINTF("SCRIPTS dsp=%08x opcode %08x arg %08x\n", s->dsp, insn, addr);
    s->dsps = addr;
    s->dcmd = insn >> 24;
    s->dsp += 8;

    switch (insn >> 30) {
    case 0: /* Block move */
    NCR710_DPRINTF("=== BLOCK MOVE ANALYSIS ===\n");
    NCR710_DPRINTF("Raw instruction: 0x%08x\n", insn);
    NCR710_DPRINTF("Instruction type: %s\n", (insn & (1 << 27)) ? "MOVE" : "CHMOV");
    NCR710_DPRINTF("Phase: %d (%s)\n", (insn >> 24) & 7,
                   ((insn >> 24) & 7) == 4 ? "STATUS" : "OTHER");
    NCR710_DPRINTF("Byte count: 0x%06x (%d bytes)\n",
                   insn & 0xffffff, insn & 0xffffff);
    NCR710_DPRINTF("Indirect: %s\n", (insn & (1 << 29)) ? "YES" : "NO");
    NCR710_DPRINTF("Table indirect: %s\n", (insn & (1 << 28)) ? "YES" : "NO");
    NCR710_DPRINTF("==============================\n");

        if (s->sstat0 & NCR_SSTAT0_STO) {
            NCR710_DPRINTF("Delayed select timeout\n");
            ncr710_stop_script(s);
            break;
        }

        s->dbc = insn & 0xffffff;

        if (insn & (1 << 29)) {
            /* Indirect addressing */
            addr = read_dword(s, addr);
        } else if (insn & (1 << 28)) {
            uint32_t buf[2];
            int32_t offset;
            /* Table indirect addressing.  */

            /* 32-bit Table indirect */
            offset = sextract32(addr, 0, 24);
			ncr710_read_memory(s, s->dsa + offset, buf, 8);
            /* byte count is stored in bits 0:23 only */
            s->dbc = cpu_to_le32(buf[0]) & 0xffffff;
            addr = cpu_to_le32(buf[1]);
        }
        s->sstat2 = (s->sstat2 & ~PHASE_MASK) | ((insn >> 24) & 7);

        if ((s->sstat2 & PHASE_MASK) != ((insn >> 24) & 7)) {
            NCR710_DPRINTF("Wrong phase got %d expected %d\n",
                    s->sstat2 & PHASE_MASK, (insn >> 24) & 7);
            ncr710_script_scsi_interrupt(s, NCR_SSTAT0_M_A);
            s->sbcl |= NCR_SBCL_REQ;
            break;
        }

        s->dnad = addr;
        switch (s->sstat2 & PHASE_MASK) {
        case PHASE_DO:
            s->waiting = 2;
            ncr710_do_dma(s, 1);
            if (s->waiting)
                s->waiting = 3;
            break;
        case PHASE_DI:
            s->waiting = 2;
            ncr710_do_dma(s, 0);
            if (s->waiting)
                s->waiting = 3;
            break;
        case PHASE_CMD:
            ncr710_do_command(s);
            break;
        case PHASE_ST:
            ncr710_do_status(s);
            break;
        case 4:
            ncr710_do_status(s);
            break;
        case PHASE_MO:
            ncr710_do_msgout(s);
            break;
        case PHASE_MI:
            ncr710_do_msgin(s);
            break;
        default:
            NCR710_DPRINTF("Unimplemented phase %d\n", s->sstat2 & PHASE_MASK);
            ncr710_script_scsi_interrupt(s, NCR_SSTAT0_SGE);
            break;
        }
        s->ctest5 = (s->ctest5 & 0xfc) | ((s->dbc >> 8) & 3);
        s->sbc = s->dbc;
        break;

    case 1: /* IO or Read/Write instruction.  */
        opcode = (insn >> 27) & 7;
        if (opcode < 5) {
            uint32_t id;

            if (insn & (1 << 25)) {
                id = read_dword(s, s->dsa + sextract32(insn, 0, 24));
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
                if (s->scntl1 & NCR_SCNTL1_CON) {
                    NCR710_DPRINTF("Already reselected, jumping to alternative address\n");
                    s->dsp = s->dnad;
                    break;
                }
                s->sstat1 |= NCR_SSTAT1_WOA;
				if (!scsi_device_find(&s->bus, 0, ncr710_idbitstonum(id), 0)) {
                    ncr710_bad_selection(s, id);
                    break;
                }
                NCR710_DPRINTF("Selected target %d%s\n",
                        id, insn & (1 << 24) ? " ATN" : "");
                /* ??? Linux drivers compain when this is set.  Maybe
                   it only applies in low-level mode (unimplemented).
                ncr710_script_scsi_interrupt(s, NCR_SSTAT0_FCMP); */
                s->select_tag = id << 8;
                s->scntl1 |= NCR_SCNTL1_CON;
                if (insn & (1 << 24)) {
                    s->socl |= NCR_SOCL_ATN;
					ncr710_set_phase(s, PHASE_MO);
				} else {
					ncr710_set_phase(s, PHASE_CMD);
				}
                break;
            case 1: /* Disconnect */
                NCR710_DPRINTF("Wait Disconnect\n");
                s->scntl1 &= ~NCR_SCNTL1_CON;
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
                    s->socl |= NCR_SOCL_ATN;
                    ncr710_set_phase(s, PHASE_MO);
                }
                if (insn & (1 << 9)) {
                    NCR710_DPRINTF("Target mode not implemented\n");
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
                    s->socl &= ~NCR_SOCL_ATN;
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
#ifdef NCR710_DEBUG
            static const char *opcode_names[3] =
                {"Write", "Read", "Read-Modify-Write"};
            static const char *operator_names[8] =
                {"MOV", "SHL", "OR", "XOR", "AND", "SHR", "ADD", "ADC"};
#endif

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

    case 2: /* Transfer Control.  */
        {
            int cond;
            int jmp;

            if ((insn & 0x002e0000) == 0) {
                NCR710_DPRINTF("NOP\n");
                break;
            }
            if (s->sstat0 & NCR_SSTAT0_STO) {
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
                    /* Relative address.  */
                    addr = s->dsp + sextract32(addr, 0, 24);
                }
                switch ((insn >> 27) & 7) {
                case 0: /* Jump */
                    NCR710_DPRINTF("Jump to 0x%08x\n", addr);
                    s->dsp = addr;
                    break;
                case 1: /* Call */
                    NCR710_DPRINTF("Call 0x%08x\n", addr);
                    s->temp = s->dsp;
                    s->dsp = addr;
                    break;
                case 2: /* Return */
                    NCR710_DPRINTF("Return to 0x%08x\n", s->temp);
                    s->dsp = s->temp;
                    break;
                case 3: /* Interrupt */
                    NCR710_DPRINTF("Interrupt 0x%08x\n", s->dsps);
                    if ((insn & (1 << 20)) != 0) {
                        ncr710_update_irq(s);
                    } else {
                        ncr710_script_dma_interrupt(s, NCR_DSTAT_SIR);
                    }
                    break;
                default:
                    NCR710_DPRINTF("Illegal transfer control\n");
                    ncr710_script_dma_interrupt(s, NCR_DSTAT_IID);
                    break;
                }
            } else {
                NCR710_DPRINTF("Control condition failed\n");
            }
        }
        break;

    case 3:
        if ((insn & (1 << 29)) == 0) {
            /* Memory move.  */
            uint32_t dest;
            /* ??? The docs imply the destination address is loaded into
               the TEMP register.  However the Linux drivers rely on
               the value being presrved.  */
            dest = read_dword(s, s->dsp);
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
        if (!(s->sien & NCR_SIEN_UDC)) {
            qemu_log_mask(LOG_GUEST_ERROR, "NCR710: Script execution timeout\n");
        }
        ncr710_script_scsi_interrupt(s, NCR_SSTAT0_UDC);
        ncr710_disconnect(s);
    } else if (s->script_active && !s->waiting) {
        if (s->dcntl & NCR_DCNTL_SSM) {
            ncr710_script_dma_interrupt(s, NCR_DSTAT_SSI);
        } else {
            goto again;
        }
    }
    NCR710_DPRINTF("SCRIPTS execution stopped\n");
}

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
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR_SCNTL1_CON))) {
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


/* ==========================
*      THINGS I'M SURE OF
* ========================== */

static inline int ncr710_irq_on_rsl(NCR710State *s)
{
   return (s->sien & NCR_SIEN_SEL) && (s->scid & 0x80); // 0x80 is the enable bit in SCID
}

static int ncr710_queue_req(NCR710State *s, SCSIRequest *req, uint32_t len)
{
    NCR710Request *p = (NCR710Request*)req->hba_private;

    if (p->pending) {
        NCR710_DPRINTF("NCR710: Multiple IO pending for request %p\n", p);
    }
    p->pending = len;
    /* Reselect if waiting for it, or if reselection triggers an IRQ
       and the bus is free.
       Since no interrupt stacking is implemented in the emulation, it
       is also required that there are no pending interrupts waiting
       for service from the device driver. */
    if (s->waiting == 1 ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR_SCNTL1_CON) &&
         !(s->istat & (NCR_ISTAT_SIP | NCR_ISTAT_DIP)))) {
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
        NCR710_DPRINTF("NCR710: MSG IN data too long\n");
    } else {
        NCR710_DPRINTF("MSG IN 0x%02x\n", data);
        s->msg[s->msg_len++] = data;
    }
}

static void ncr710_stop_script(NCR710State *s)
{
    s->script_active = 0;
    s->scripts.running = false;
}


static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0)
{
    uint32_t mask0;

    NCR710_DPRINTF("NCR710_DEBUG: ncr710_script_scsi_interrupt() called - stat0=0x%02x, prev_sstat0=0x%02x\n",
           stat0, s->sstat0);
    NCR710_DPRINTF("SCSI Interrupt 0x%02x prev 0x%02x\n", stat0, s->sstat0);

    s->sstat0 |= stat0;
    /* Stop processor on fatal or unmasked interrupt.  As a special hack
       we don't stop processing when raising STO.  Instead continue
       execution and stop at the next insn that accesses the SCSI bus.  */
    mask0 = s->sien | ~(NCR_SSTAT0_FCMP | NCR_SSTAT0_SEL);

    NCR710_DPRINTF("NCR710_DEBUG: sstat0=0x%02x, mask0=0x%02x, masked=0x%02x\n",
           s->sstat0, mask0, s->sstat0 & mask0);

    if (s->sstat0 & mask0) {
        NCR710_DPRINTF("NCR710_DEBUG: Stopping script due to interrupt\n");
        ncr710_stop_script(s);
    }

    NCR710_DPRINTF("NCR710_DEBUG: Calling ncr710_update_irq()\n");
    ncr710_update_irq(s);
}

static inline void ncr710_set_phase(NCR710State *s, int phase)
{
    int old_phase = s->sstat2 & PHASE_MASK;
    NCR710_DPRINTF("NCR710_DEBUG: Phase change: %d -> %d\n", old_phase, phase);

    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | phase;
    s->scripts.phase = phase;

    // Set appropriate control lines based on phase
    switch (phase) {
    case PHASE_DO:  // Data Out
        s->sbcl &= ~(NCR_SBCL_MSG | NCR_SBCL_CD | NCR_SBCL_IO);
        break;
    case PHASE_DI:  // Data In
        s->sbcl &= ~(NCR_SBCL_MSG | NCR_SBCL_CD);
        s->sbcl |= NCR_SBCL_IO;
        break;
    case PHASE_CMD: // Command
        s->sbcl &= ~(NCR_SBCL_MSG | NCR_SBCL_IO);
        s->sbcl |= NCR_SBCL_CD;
        break;
    case PHASE_ST:  // Status
        s->sbcl &= ~NCR_SBCL_MSG;
        s->sbcl |= (NCR_SBCL_CD | NCR_SBCL_IO);
        break;
    case PHASE_MO:  // Message Out
        s->sbcl &= ~NCR_SBCL_IO;
        s->sbcl |= (NCR_SBCL_MSG | NCR_SBCL_CD);
        break;
    case PHASE_MI:  // Message In
        s->sbcl |= (NCR_SBCL_MSG | NCR_SBCL_CD | NCR_SBCL_IO);
        break;
    }

    s->ctest0 &= ~1;
    if (phase == PHASE_DI) {
        s->ctest0 |= 1;
    }
    s->sbcl &= ~NCR_SBCL_REQ;
}

static void ncr710_bad_phase(NCR710State *s, int out, int new_phase)
{
    /* Trigger a phase mismatch */
    NCR710_DPRINTF("Phase mismatch interrupt\n");
    ncr710_script_scsi_interrupt(s, NCR_SSTAT0_M_A);
    ncr710_stop_script(s);
    ncr710_set_phase(s, new_phase);
    s->sbcl |= NCR_SBCL_REQ;
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
        s->istat |= NCR_ISTAT_DIP;
    } else {
        s->istat &= ~NCR_ISTAT_DIP;
    }

    if (s->sstat0) {
        if ((s->sstat0 & s->sien))
            level = 1;
        s->istat |= NCR_ISTAT_SIP;
    } else {
        s->istat &= ~NCR_ISTAT_SIP;
    }

    if (level != last_level) {
        NCR710_DPRINTF("Update IRQ level %d dstat %02x sstat0 %02x\n",
                level, s->dstat, s->sstat0);
        last_level = level;
    }

    if (s->irq) {
        NCR710_DPRINTF("IRQ FROM ncr710_update_irq func");
        qemu_set_irq(s->irq, level);
    }

    if (!level && ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR_SCNTL1_CON)) {
        NCR710_DPRINTF("Handled IRQs & disconnected, looking for pending processes\n");
        QTAILQ_FOREACH(p, &s->queue, next) {
            if (p->pending) {
                ncr710_reselect(s, p);
                break;
            }
        }
    }
}

static void ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    SysBusNCR710State *sysbus_s = container_of(req->bus, SysBusNCR710State, ncr710.bus);
    NCR710State *s = &sysbus_s->ncr710;
    int out;

    NCR710_DPRINTF("NCR710_DEBUG: ncr710_command_complete() called - status=%d, resid=%zu\n",
           (int)req->status, resid);

    out = (s->sstat2 & PHASE_MASK) == PHASE_DO;
    NCR710_DPRINTF("Command complete status=%d\n", (int)req->status);
    s->lcrc = 0;
    s->status = req->status;
    s->command_complete = 2;

    NCR710_DPRINTF("NCR710_DEBUG: Set command_complete=2, status=%d\n", s->status);

    if (s->waiting && s->dbc != 0) {
        NCR710_DPRINTF("NCR710_DEBUG: Bad phase - calling ncr710_bad_phase()\n");
        ncr710_bad_phase(s, out, PHASE_ST);
    } else {
        NCR710_DPRINTF("NCR710_DEBUG: Setting PHASE_ST for status phase\n");
        ncr710_set_phase(s, PHASE_ST);
    }

    if (req->hba_private == s->current) {
        NCR710_DPRINTF("NCR710_DEBUG: Cleaning up current request\n");
        req->hba_private = NULL;
        ncr710_request_free(s, s->current);
        scsi_req_unref(req);
    }

    NCR710_DPRINTF("NCR710_DEBUG: Calling ncr710_resume_script()\n");
    ncr710_resume_script(s);
}

bool ncr710_is_700_mode(NCR710State *s)
{
    return s->compatibility_mode;
}

static void ncr710_update_compatibility_mode(NCR710State *s)
{
    bool old_mode = s->compatibility_mode;
    s->compatibility_mode = (s->dcntl & NCR_DCNTL_COM) != 0;

    NCR710_DPRINTF("DCNTL=0x%02x, COM bit=%d, compatibility_mode=%s\n",
                   s->dcntl, (s->dcntl & NCR_DCNTL_COM) ? 1 : 0,
                   s->compatibility_mode ? "53C700" : "53C710");

    if (old_mode != s->compatibility_mode) {
        NCR710_DPRINTF("NCR710: Switching to %s compatibility mode\n",
                 s->compatibility_mode ? "53C700" : "53C710");
    }
}

static void ncr710_internal_scsi_bus_reset(NCR710State *s)
{
    NCR710Request *req, *next_req;

    trace_ncr710_bus_reset();
    NCR710_DPRINTF("NCR710: Internal SCSI bus reset called\n");

    QTAILQ_FOREACH_SAFE(req, &s->queue, next, next_req) {
        NCR710_DPRINTF("Cancelling queued request tag=0x%x\n", req->tag);
        scsi_req_cancel(req->req);
    }
    if (s->current) {
        NCR710_DPRINTF("Cancelling current request tag=0x%x\n", s->current->tag);
        scsi_req_cancel(s->current->req);
        s->current = NULL;
    }
    s->sstat0 = NCR_SSTAT0_RST;  /* Set RST bit to indicate reset occurred */
    s->sstat1 = 0;
    s->sstat2 = PHASE_DO;
    s->dstat = NCR_DSTAT_DFE;  /* DMA FIFO Empty */
    s->scntl0 &= ~(NCR_SCNTL0_START | NCR_SCNTL0_WATN);
    s->scntl1 &= ~(NCR_SCNTL1_CON | NCR_SCNTL1_RST);
    s->socl = 0;
    s->sbcl = 0;
    s->scripts.connected = false;
    s->scripts.initiator = false;
    s->scripts.running = false;
    s->scripts.phase = PHASE_DO;
    s->script_active = 0;

    trace_ncr710_connected(false);

    ncr710_dma_fifo_flush(&s->dma_fifo);
    ncr710_scsi_fifo_init(&s->scsi_fifo);

    s->istat &= ~(NCR_ISTAT_SIP | NCR_ISTAT_DIP | NCR_ISTAT_CON);
    if (s->sien & NCR_SIEN_RST) {
        s->istat |= NCR_ISTAT_SIP;
    }
    bus_cold_reset(BUS(&s->bus));
    ncr710_update_irq(s);
    NCR710_DPRINTF("NCR710: SCSI bus reset completed\n");
}

static void ncr710_soft_reset(NCR710State *s)
{
    trace_ncr710_device_reset();

    ncr710_dma_fifo_init(&s->dma_fifo);
    ncr710_scsi_fifo_init(&s->scsi_fifo);

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
    s->istat &= NCR_ISTAT_RST;
    s->dcmd = 0x40;
    s->dstat = NCR_DSTAT_DFE;
    s->dien = 0;
    s->sien = 0;

    /* Reset test registers */
    s->ctest2 = 0x01;
    s->ctest3 = 0;
    s->ctest4 = 0;
    s->ctest5 = 0;

    /* Reset SCSI control and status registers */
    s->scntl0 = 0xc0;
    s->scntl1 = 0;
    s->sstat0 = 0;
    s->sstat1 = 0;

    s->sstat2 = PHASE_DO;

    s->scid = 0x80;
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
    s->scripts.phase = PHASE_DO;
    s->scripts.pc = 0;
    s->scripts.saved_pc = 0;
    s->script_active = 0;
    s->compatibility_mode = false;
}

static void ncr710_device_reset(DeviceState *dev)
{
    SysBusNCR710State *sysbus_s = SYSBUS_NCR710_SCSI(dev);
    NCR710State *s = &sysbus_s->ncr710;
    NCR710_DPRINTF("NCR710: Device reset function called\n");
    trace_ncr710_device_reset();

    ncr710_soft_reset(s);

    QTAILQ_INIT(&s->queue);

    s->scripts.running = false;
    s->scripts.connected = false;
    s->scripts.initiator = false;
    s->scripts.phase = PHASE_DO;
    s->scripts.pc = 0;
    s->scripts.saved_pc = 0;
    s->script_active = 0;
    qemu_set_irq(s->irq, 0);
}

/* FIFO Implementation (DMA and SCSI, YES NCR710 HAS BOTH) */
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
    NCR710_DPRINTF("Flushing DMA FIFO, old count=%d\n", fifo->count);
    int old_count = fifo->count;
    ncr710_dma_fifo_init(fifo);
    trace_ncr710_dma_fifo_flush(old_count);
}

static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo)
{
    NCR710_DPRINTF("Initializing SCSI FIFO\n");
    trace_ncr710_scsi_fifo_init();
    fifo->count = 0;
    memset(fifo->data, 0, sizeof(fifo->data));
    memset(fifo->parity, 0, sizeof(fifo->parity));
}

static bool ncr710_scsi_fifo_empty(NCR710_SCSI_FIFO *fifo)
{
    NCR710_DPRINTF("Checking if SCSI FIFO is empty: count=%d\n", fifo->count);
    return fifo->count == 0;
}

static bool ncr710_scsi_fifo_full(NCR710_SCSI_FIFO *fifo)
{
    NCR710_DPRINTF("Checking if SCSI FIFO is full: count=%d\n", fifo->count);
    return fifo->count >= NCR710_SCSI_FIFO_SIZE;
}

static void ncr710_scsi_fifo_push(NCR710_SCSI_FIFO *fifo, uint8_t data, uint8_t parity)
{

    NCR710_DPRINTF("Pushing data to SCSI FIFO: data=0x%02x parity=0x%02x\n", data, parity);
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

/*
 * FIFO IMPLEMENTATION WITH PROPER BUFFERING AND FLOW CONTROL
 *
 * Our implementation provides hardware-accurate FIFO behavior with:
 *
 * 1. SEPARATE PHASES: Push and pop operations are separated to allow
 *    proper buffering and flow control
 *
 * 2. FIFO CAPACITY LIMITS: Operations respect FIFO size limitations,
 *    enabling proper burst transfer optimization
 *
 * 3. FLOW CONTROL: Data can accumulate in FIFOs, allowing for
 *    interrupt-driven transfers and proper timing
 *
 * 4. PARITY HANDLING: Full parity generation and checking throughout
 *    the FIFO data path
 *
 * 5. REGISTER ACCURACY: FIFO status properly reflected in SSTAT/DSTAT
 *    registers for software compatibility
 *
 * DMA FIFO Flow:
 *   OUT: Memory -> DMA FIFO -> Device Buffer
 *   IN:  Device Buffer -> DMA FIFO -> Memory
 *
 * SCSI FIFO Flow:
 *   OUT: Memory -> SCSI FIFO -> Command/Message Processing
 *   IN:  Status/Message -> SCSI FIFO -> Memory
 */

/* Drain DMA FIFO to device buffer - used for DMA OUT operations */
static uint32_t ncr710_dma_fifo_drain_to_buffer(NCR710State *s, uint8_t *buffer, uint32_t max_count)
{
    uint32_t bytes_transferred = 0;
    uint32_t available = s->dma_fifo.count;
    uint32_t transfer_count = (max_count < available) ? max_count : available;

    /* Drain FIFO to device buffer */
    for (uint32_t i = 0; i < transfer_count; i++) {
        uint8_t parity;
        buffer[i] = ncr710_dma_fifo_pop(&s->dma_fifo, &parity);
        bytes_transferred++;

        /* Store first byte in SFBR */
        if (i == 0) {
            s->sfbr = buffer[i];
        }
    }

    /* Update DMA FIFO Empty flag */
    if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
        s->dstat |= NCR_DSTAT_DFE;
    } else {
        s->dstat &= ~NCR_DSTAT_DFE;
    }

    return bytes_transferred;
}

/* Fill DMA FIFO from device buffer - used for DMA IN operations */
static uint32_t ncr710_dma_fifo_fill_from_buffer(NCR710State *s, const uint8_t *buffer, uint32_t max_count)
{
    uint32_t bytes_transferred = 0;
    uint32_t fifo_space = NCR710_DMA_FIFO_SIZE - s->dma_fifo.count;
    uint32_t transfer_count = (max_count < fifo_space) ? max_count : fifo_space;

    /* Fill FIFO from device buffer */
    for (uint32_t i = 0; i < transfer_count; i++) {
        uint8_t parity = (s->scntl0 & NCR_SCNTL0_EPG) ?
                        ncr710_generate_scsi_parity(s, buffer[i]) : 0;
        ncr710_dma_fifo_push(&s->dma_fifo, buffer[i], parity);
        bytes_transferred++;

        /* Store first byte in SFBR */
        if (i == 0) {
            s->sfbr = buffer[i];
        }
    }

    /* Update DMA FIFO Empty flag */
    if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
        s->dstat |= NCR_DSTAT_DFE;
    } else {
        s->dstat &= ~NCR_DSTAT_DFE;
    }

    return bytes_transferred;
}

/* Drain DMA FIFO to memory - used for DMA IN operations */
static uint32_t ncr710_dma_fifo_drain_to_memory(NCR710State *s, uint32_t addr, uint32_t max_count)
{
    uint32_t bytes_transferred = 0;
    uint32_t available = s->dma_fifo.count;
    uint32_t transfer_count = (max_count < available) ? max_count : available;

    /* Drain FIFO to memory */
    uint8_t temp_buffer[NCR710_DMA_FIFO_SIZE];
    for (uint32_t i = 0; i < transfer_count; i++) {
        uint8_t parity;
        temp_buffer[i] = ncr710_dma_fifo_pop(&s->dma_fifo, &parity);
        bytes_transferred++;
    }

    if (bytes_transferred > 0) {
        ncr710_write_memory(s, addr, temp_buffer, bytes_transferred);
    }

    /* Update DMA FIFO Empty flag */
    if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
        s->dstat |= NCR_DSTAT_DFE;
    } else {
        s->dstat &= ~NCR_DSTAT_DFE;
    }

    return bytes_transferred;
}

/* Read and write wrapper */
static uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR710State *s = opaque;
    uint8_t offset = addr & 0xff;
    uint8_t val = ncr710_reg_readb(s, offset);
    trace_ncr710_reg_read(offset, ncr710_reg_name(offset), val);
    return val;
}

static void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    NCR710State *s = opaque;
    uint8_t offset = addr & 0xff;
    uint8_t val8 = val & 0xff;
    trace_ncr710_reg_write(offset, ncr710_reg_name(offset), val8);
    ncr710_reg_writeb(s, offset, val8);
}

static uint8_t ncr710_reg_readb(NCR710State *s, int offset)
{
    uint8_t ret;
    bool is_700_mode = ncr710_is_700_mode(s);

#define CASE_GET_REG24(name, addr) \
    case addr: ret = (s->name >> 16) & 0xff; break; \
    case addr + 1: ret = (s->name >> 8) & 0xff; break; \
    case addr + 2: ret = s->name & 0xff; break;

#define CASE_GET_REG32(name, addr) \
    case addr: ret = (s->name >> 24) & 0xff; break; \
    case addr + 1: ret = (s->name >> 16) & 0xff; break; \
    case addr + 2: ret = (s->name >> 8) & 0xff; break; \
    case addr + 3: ret = s->name & 0xff; break;

    switch (offset) {
        case NCR710_SCNTL0_REG: /* SCNTL0 */
            ret = s->scntl0;
            break;
        case NCR710_SCNTL1_REG: /* SCNTL1 */
            ret = s->scntl1; /* Fix implement changes for 700 mode */
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
            if (s->scntl1 & NCR_SCNTL1_CON) {
                ret = s->sstat2 & PHASE_MASK;
                ret |= s->sbcl;
                if (s->socl & NCR_SOCL_ATN)
                    ret |= NCR_SBCL_ATN;
            }
            break;
        case NCR710_DSTAT_REG: /* DSTAT */
            ret = s->dstat;

            /* Update DMA FIFO Empty flag */
            if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
                ret |= NCR_DSTAT_DFE;
            } else {
                ret &= ~NCR_DSTAT_DFE;
            }

            if (is_700_mode) {
                ret &= ~0x20;  /* Mask out bus fault bit in 700 mode */
            }

            s->dstat = 0;
            ncr710_update_irq(s);
            break;
        case NCR710_SSTAT0_REG: /* SSTAT0 */
            ret = s->sstat0;
            if (s->sstat0 != 0) {
                s->sstat0 = 0;
                s->istat &= ~NCR_ISTAT_SIP;
                ncr710_update_irq(s);
            }
            break;
        case NCR710_SSTAT1_REG: /* SSTAT1 */
            ret = s->sstat1;

            /* Update Input Latch Full (ILF) based on SCSI FIFO */
            if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
                ret |= NCR_SSTAT1_ILF;
            } else {
                ret &= ~NCR_SSTAT1_ILF;
            }

            /* Update Output Register Full (ORF) based on DMA FIFO */
            if (!ncr710_dma_fifo_empty(&s->dma_fifo)) {
                ret |= NCR_SSTAT1_ORF;
            } else {
                ret &= ~NCR_SSTAT1_ORF;
            }

            /* Update SCSI Parity bit (SDP) with live parity signal */
            if (s->scntl0 & NCR_SCNTL0_EPG) {
                if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
                    uint8_t parity = s->scsi_fifo.parity[0];
                    if (parity) {
                        ret |= NCR_SSTAT1_SDP;
                    } else {
                        ret &= ~NCR_SSTAT1_SDP;
                    }
                }
            }
            break;
        case NCR710_SSTAT2_REG: /* SSTAT2 */
            ret = s->sstat2;
            ret &= ~0xF0; /* Clear FF bits */
            if (s->scsi_fifo.count <= 8) {
                ret |= (s->scsi_fifo.count & 0x0F) << 4;
            } else {
                ret |= 0x80; /* Set to maximum (8 bytes) */
            }

            if (s->sidl != 0) {
                uint8_t expected_parity = ncr710_generate_scsi_parity(s, s->sidl);
                if (expected_parity) {
                    ret |= NCR_SSTAT2_SDP;
                } else {
                    ret &= ~NCR_SSTAT2_SDP;
                }
            } else {
                ret &= ~NCR_SSTAT2_SDP;
            }
            break;
        CASE_GET_REG32(dsa, NCR710_DSA_REG)
            if (is_700_mode) {
                NCR710_DPRINTF("NCR710: DSA read in 700 compatibility mode\n");
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
                    // s->dfifo |= 0x80; /* Set DFE bit */
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
                ret &= ~NCR_ISTAT_SIGP;  /* SIGP doesn't exist in 700 */
                ret &= ~NCR_ISTAT_RST;
            }
            break;
        case NCR710_CTEST8_REG: /* CTEST8 */
            if (is_700_mode) {
                NCR710_DPRINTF("NCR710: CTEST8 read in 700 compatibility mode\n");
                return 0;
            }
            ret = s->ctest8;
            ret = (ret & 0x4F) | (NCR710_REVISION_2 << 4);
            break;
        case NCR710_LCRC_REG: /* LCRC */
            if (is_700_mode) {
                NCR710_DPRINTF("NCR710: LCRC read in 700 compatibility mode\n");
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
                NCR710_DPRINTF("NCR710: SCRATCH read in 700 compatibility mode\n");
                return 0;
            }
            break;
        case NCR710_DMODE_REG: /* DMODE */
            ret = s->dmode;
            NCR710_DPRINTF("NCR710: DMODE read: 0x%02x\n", ret);
            if (is_700_mode) {
                /* In 700 mode: different bit meanings
                 * Bits 5-4: 16-bit DMA '286-mode bits (700 only)
                 * Bit 3: I/O-memory mapped DMA bit (700 only)
                 * Bit 1: Pipeline mode bit (700 only)
                 * Clear 710-specific function code bits
                */
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
                ret &= ~(DCNTL_EA | NCR_DCNTL_COM);  /* These don't exist in 700 */
                /* Software reset is bit 0 in 700 mode */
            }
            return ret;
        CASE_GET_REG32(adder, NCR710_ADDER_REG)
            if (is_700_mode) {
                NCR710_DPRINTF("NCR710: ADDER read in 700 compatibility mode\n");
                return 0;
            }
            break;
        default:
            trace_ncr710_reg_read_unhandled(offset, 1);
            NCR710_DPRINTF("NCR710: invalid read at offset 0x%x\n", (int)offset);
            break;
    }

#undef CASE_GET_REG24
#undef CASE_GET_REG32
    return ret;
}

static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val)
{
    uint8_t old_val;
    bool is_700_mode = ncr710_is_700_mode(s);

#define CASE_SET_REG24(name, addr) \
    case addr    : s->name = (s->name & 0x0000ffff) | (val << 16); break; \
    case addr + 1: s->name = (s->name & 0x00ff00ff) | (val << 8);  break; \
    case addr + 2: s->name = (s->name & 0x00ffff00) | val;       break;

#define CASE_SET_REG32(name, addr) \
    case addr    : s->name = (s->name & 0x00ffffff) | (val << 24); break; \
    case addr + 1: s->name = (s->name & 0xff00ffff) | (val << 16); break; \
    case addr + 2: s->name = (s->name & 0xffff00ff) | (val << 8);  break; \
    case addr + 3: s->name = (s->name & 0xffffff00) | val;       break;

    /* Multi-method debug output to ensure visibility */
    NCR710_DPRINTF("NCR710: Write %s[0x%02x] = 0x%02x\n", ncr710_reg_name(offset), offset, val);
    trace_ncr710_reg_write(offset, ncr710_reg_name(offset), val);

    switch (offset) {
    case NCR710_SCNTL0_REG: /* SCNTL0 */
        old_val = s->scntl0;
        s->scntl0 = val;
        NCR710_DPRINTF("NCR710: SCNTL0: 0x%02x->0x%02x parity=%s%s\n", old_val, val,
                 (val & NCR_SCNTL0_EPC) ? "chk" : "", (val & NCR_SCNTL0_EPG) ? "gen" : "");
        NCR710_DPRINTF("NCR710: SCNTL0: 0x%02x->0x%02x parity=%s%s\n", old_val, val,
                (val & NCR_SCNTL0_EPC) ? "chk" : "", (val & NCR_SCNTL0_EPG) ? "gen" : "");

        /* Handle parity control bits according to NCR710 manual */
        if ((val & NCR_SCNTL0_EPC) != (old_val & NCR_SCNTL0_EPC)) {
            /* Enable Parity Checking bit changed */
            trace_ncr710_parity_checking_changed((val & NCR_SCNTL0_EPC) != 0);
        }

        if ((val & NCR_SCNTL0_EPG) != (old_val & NCR_SCNTL0_EPG)) {
            /* Enable Parity Generation bit changed */
            trace_ncr710_parity_generation_changed((val & NCR_SCNTL0_EPG) != 0);
        }

        if ((val & NCR_SCNTL0_AAP) != (old_val & NCR_SCNTL0_AAP)) {
            /* Assert ATN/ on Parity Error bit changed */
            trace_ncr710_atn_on_parity_changed((val & NCR_SCNTL0_AAP) != 0);
        }

        if (val & NCR_SCNTL0_START) {
            NCR710_DPRINTF("NCR710: SCNTL0: START bit set, mode=%s\n", is_700_mode ? "700" : "710");
            NCR710_DPRINTF("NCR710: SCNTL0: START bit set, mode=%s\n", is_700_mode ? "700" : "710");
            if (is_700_mode) {
                qemu_log_mask(LOG_UNIMP, "NCR710: Start sequence not implemented\n");
            } else {
                ncr710_arbitrate_bus(s); /* Might delete later */
            }
        }
        break;

    case NCR710_SCNTL1_REG: /* SCNTL1 */
        old_val = s->scntl1;
        s->scntl1 = val;
        NCR710_DPRINTF("NCR710: SCNTL1: 0x%02x->0x%02x%s%s\n", old_val, val,
                 (val & NCR_SCNTL1_RST) ? " RST" : "",
                 (val & NCR_SCNTL1_ADB) ? " ADB" : "");
        NCR710_DPRINTF("NCR710: SCNTL1: 0x%02x->0x%02x%s%s\n", old_val, val,
                (val & NCR_SCNTL1_RST) ? " RST" : "",
                (val & NCR_SCNTL1_ADB) ? " ADB" : "");

        /* Handle Assert Even SCSI Parity (AESP) bit changes */
        if ((val & NCR_SCNTL1_AESP) != (old_val & NCR_SCNTL1_AESP)) {
            trace_ncr710_parity_sense_changed((val & NCR_SCNTL1_AESP) != 0 ? "even" : "odd");
        }

        if (val & NCR_SCNTL1_ADB) {
            qemu_log_mask(LOG_UNIMP, "NCR710: Immediate Arbitration not implemented\n");
        }

        if (val & NCR_SCNTL1_RST) {
            if (!(s->sstat0 & NCR_SSTAT0_RST)) {
                s->sstat0 |= NCR_SSTAT0_RST;
                ncr710_script_scsi_interrupt(s, NCR_SSTAT0_RST);
            }
            /* Enhanced reset handling for second implementation */
            if (!(old_val & NCR_SCNTL1_RST)) {
                NCR710_DPRINTF("NCR710: SCNTL1: SCSI bus reset initiated\n");
                NCR710_DPRINTF("NCR710: SCNTL1: SCSI bus reset initiated\n");
                ncr710_internal_scsi_bus_reset(s);
            }
        } else {
            s->sstat0 &= ~NCR_SSTAT0_RST;
        }
        break;

    case NCR710_SDID_REG: /* SDID */
        s->sdid = val & 0x0F; /* Only lower 4 bits are valid */
        NCR710_DPRINTF("NCR710: SDID: set target ID=%d\n", s->sdid);
        NCR710_DPRINTF("NCR710: SDID: set target ID=%d\n", s->sdid);
        break;

    case NCR710_SIEN_REG: /* SIEN */
        s->sien = val;
        NCR710_DPRINTF("SIEN: interrupt mask=0x%02x\n", val);
        ncr710_update_irq(s);
        break;

    case NCR710_SCID_REG: /* SCID */
        s->scid = val;
        NCR710_DPRINTF("SCID: host ID=%d enable=%s\n", val & 0x7F, (val & 0x80) ? "on" : "off");
        break;

    case NCR710_SXFER_REG: /* SXFER */
        s->sxfer = val;
        NCR710_DPRINTF("SXFER: sync period=%d offset=%d\n", (val >> 4) & 0x0F, val & 0x0F);
        break;

    case NCR710_SODL_REG: /* SODL */
        s->sodl = val;
        s->sstat1 |= NCR_SSTAT1_OLF; /* From second implementation */
        NCR710_DPRINTF("SODL: 0x%02x (output latch full)\n", val);
        break;

    case NCR710_SOCL_REG: /* SOCL */
        s->socl = val;
        NCR710_DPRINTF("SOCL: 0x%02x (phase control)\n", val);
        break;

    case NCR710_SFBR_REG: /* SFBR */
        s->sfbr = val;
        break;

    case NCR710_SIDL_REG: /* SIDL */
    case NCR710_SBDL_REG: /* SBDL */
        break;

    case NCR710_SBCL_REG: /* SBCL */
        s->sbcl = val;
        ncr710_set_phase(s, val & PHASE_MASK);
        break;

    case NCR710_DSTAT_REG:
    case NCR710_SSTAT0_REG:
    case NCR710_SSTAT1_REG:
    case NCR710_SSTAT2_REG:
        /* Linux writes to these readonly registers on startup */
        return;

    CASE_SET_REG32(dsa, NCR710_DSA_REG)
        /* Allow DSA writes even in 700 compatibility mode for now */
        NCR710_DPRINTF("DSA: 0x%08x (compatibility mode: %s)\n",
                       s->dsa, is_700_mode ? "53C700" : "53C710");
        break;

    case NCR710_CTEST0_REG: /* CTEST0 */
        s->ctest0 = val;
        NCR710_DPRINTF("CTEST0: 0x%02x tolerant=%s\n", val, (val & 0x01) ? "on" : "off");
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
        NCR710_DPRINTF("CTEST3: SCSI FIFO write 0x%02x\n", val);
        /* SCSI FIFO write - use proper parity generation */
        if (!ncr710_scsi_fifo_full(&s->scsi_fifo)) {
            uint8_t parity = 0;
            if (s->scntl0 & NCR_SCNTL0_EPG) {
                parity = ncr710_generate_scsi_parity(s, val);
            }
            ncr710_scsi_fifo_push(&s->scsi_fifo, val, parity);

            /* Update CTEST2 bit 4 (SCSI FIFO Parity) according to manual */
            if (parity) {
                s->ctest2 |= 0x10;
            } else {
                s->ctest2 &= ~0x10;
            }
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
        NCR710_DPRINTF("CTEST6: DMA FIFO write 0x%02x\n", val);
        /* DMA FIFO write - use proper parity from CTEST1 bit 3 (DFP) */
        if (!ncr710_dma_fifo_full(&s->dma_fifo)) {
            uint8_t parity = (s->ctest1 & 0x08) ? 1 : 0; /* DFP bit from CTEST1 */
            ncr710_dma_fifo_push(&s->dma_fifo, val, parity);

            /* Update CTEST2 bit 3 (DMA FIFO Parity) according to manual */
            if (parity) {
                s->ctest2 |= 0x08;
            } else {
                s->ctest2 &= ~0x08;
            }
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
        NCR710_DPRINTF("NCR710: ISTAT write 0x%02x mode=%s\n", val, is_700_mode ? "700" : "710");
        if (is_700_mode) {
            /* In 700 mode: no SIGP bit, no RST bit */
            s->istat = (s->istat & 0x0f) | (val & 0xDF); /* Mask out SIGP and RST */
        } else {
            if (val & NCR_ISTAT_ABRT) {
                NCR710_DPRINTF("NCR710: ISTAT ABRT bit set - aborting SCRIPTS\n");
                s->scripts.running = false;
                s->dstat |= NCR_DSTAT_ABRT;
                s->istat |= NCR_ISTAT_DIP;
                timer_del(s->selection_timer);
                timer_del(s->watchdog_timer);
                ncr710_script_dma_interrupt(s, NCR_DSTAT_ABRT);
                ncr710_update_irq(s);
            }
            if (s->waiting == 1 && (val & NCR_ISTAT_SIGP)) {
                NCR710_DPRINTF("NCR710: SIGP received - waking up script execution\n");
                s->waiting = 0;
                s->dsp = s->dnad;
                NCR710_DPRINTF("NCR710: Starting script execution from DSP=0x%x\n", s->dsp);
                ncr710_scripts_execute(s);
            }
            if (val & NCR_ISTAT_RST) {
                NCR710_DPRINTF("NCR710: ISTAT RST bit set - soft reset\n");
                /* Enhanced reset from second implementation */
                uint8_t saved_ctest8 = s->ctest8;
                ncr710_soft_reset(s);
                s->ctest8 = saved_ctest8;  /* Preserve chip revision */
                s->dstat = NCR_DSTAT_DFE;
                ncr710_dma_fifo_init(&s->dma_fifo);
                ncr710_scsi_fifo_init(&s->scsi_fifo);
            }
            s->istat = (s->istat & 0x0f) | (val & 0xf0);
        }
        break;

    case NCR710_CTEST8_REG: /* CTEST8 */
        if (is_700_mode) {
            NCR710_DPRINTF("CTEST8: write blocked in 700 mode\n");
            return;
        }
        uint8_t control_bits = val & 0x4F;  /* Bits 6,3,2,1,0 are writable */
        uint8_t revision_bits = NCR710_REVISION_2;
        s->ctest8 = revision_bits | control_bits;
        NCR710_DPRINTF("CTEST8: 0x%02x -> 0x%02x %s%s%s\n", val, s->ctest8,
                       (val & 0x40) ? "snoop " : "",
                       (val & 0x08) ? "flush_dma " : "",
                       (val & 0x04) ? "clear_fifos " : "");
        if (val & 0x08) {
            NCR710_DPRINTF("CTEST8: Flushing DMA FIFO\n");
            if (s->dma_fifo.count > 0) {
                uint32_t flushed = ncr710_dma_fifo_drain_to_memory(s, s->dnad, s->dma_fifo.count);
                s->dnad += flushed;
                NCR710_DPRINTF("CTEST8: Flushed %d bytes from DMA FIFO to 0x%08x\n", flushed, s->dnad - flushed);
            }
            s->dstat |= NCR_DSTAT_DFE;  /* Set DMA FIFO Empty */
        }
        if (val & 0x04) {
            NCR710_DPRINTF("CTEST8: Clearing all FIFOs\n");
            ncr710_dma_fifo_init(&s->dma_fifo);
            ncr710_scsi_fifo_init(&s->scsi_fifo);
            s->dstat |= NCR_DSTAT_DFE;  /* Set DMA FIFO Empty */
        }
        break;
    case NCR710_LCRC_REG: /* LCRC */
        if (is_700_mode) {
            NCR710_DPRINTF("LCRC: write blocked in 700 mode\n");
            return;
        }
        s->lcrc = val;
        NCR710_DPRINTF("LCRC: 0x%02x\n", val);
        break;

    CASE_SET_REG24(dbc, NCR710_DBC_REG)

    case NCR710_DCMD_REG: /* DCMD */
        s->dcmd = val;
        break;

    CASE_SET_REG32(dnad, NCR710_DNAD_REG)
    case 0x2c: /* DSP[24:31] - MSB , writing the DSP in big-endian format */
        s->dsp &= 0x00ffffff;
        s->dsp |= val << 24;
        NCR710_DPRINTF("NCR710: DSP write byte 0: 0x%02x, DSP now=0x%08x\n", val, s->dsp);
        break;
    case 0x2d: /* DSP[16:23] */
        s->dsp &= 0xff00ffff;
        s->dsp |= val << 16;
        NCR710_DPRINTF("NCR710: DSP write byte 1: 0x%02x, DSP now=0x%08x\n", val, s->dsp);
        break;
    case 0x2e: /* DSP[8:15] */
        s->dsp &= 0xffff00ff;
        s->dsp |= val << 8;
        NCR710_DPRINTF("NCR710: DSP write byte 2: 0x%02x, DSP now=0x%08x\n", val, s->dsp);
        break;
    case 0x2f: /* DSP[0:7] - LSB */
        s->dsp &= 0xffffff00;
        s->dsp |= val;
        NCR710_DPRINTF("NCR710: DSP write byte 3: 0x%02x, DSP FINAL=0x%08x\n", val, s->dsp);
        if (s->dsp != 0) {
            s->waiting = 0;
            s->scripts.running = true;
            s->scripts.pc = s->dsp;
            s->script_active = 1;
            ncr710_scripts_execute(s);
        }
        break;
    CASE_SET_REG32(dsps, NCR710_DSPS_REG)
    CASE_SET_REG32(scratch, NCR710_SCRATCH_REG)
        if (is_700_mode) {
            NCR710_DPRINTF("SCRATCH: write blocked in 700 mode\n");
            return;
        }
        NCR710_DPRINTF("SCRATCH: 0x%08x\n", s->scratch);
        break;

    case NCR710_DMODE_REG: /* DMODE */
        s->dmode = val;
        NCR710_DPRINTF("NCR710: DMODE read: 0x%02x\n", val);
        if (is_700_mode) {
            /* In 700 mode: different bit meanings
             * Bits 5-4: 16-bit DMA '286-mode bits (700 only)
             * Bit 3: I/O-memory mapped DMA bit (700 only)
             * Bit 1: Pipeline mode bit (700 only)
             * Clear 710-specific function code bits
             */
            /* Clear bits 5-3 which have different meanings in 710 */
            s->dmode &= ~0x38;
            NCR710_DPRINTF("DMODE: 0x%02x (700 mode)\n", val);
        } else {
            switch (val & NCR_DMODE_BL_MASK) {
            case 0x00: s->burst_length = 1; break;
            case 0x40: s->burst_length = 2; break;
            case 0x80: s->burst_length = 4; break;
            case 0xC0: s->burst_length = 8; break;
            }
            NCR710_DPRINTF("DMODE: 0x%02x burst=%d\n", val, s->burst_length);
        }
        break;

    case NCR710_DIEN_REG: /* DIEN */
        s->dien = val;
        if (is_700_mode) {
            /* In 700 mode: bit 5 (Bus fault interrupt enable) doesn't exist */
            s->dien &= ~0x20;
        }
        NCR710_DPRINTF("DIEN: interrupt enable=0x%02x\n", val);
        ncr710_update_irq(s);
        break;

    case NCR710_DWT_REG: /* DWT */
        s->dwt = val;
        NCR710_DPRINTF("DWT: watchdog timeout=0x%02x\n", val);
        break;

    case NCR710_DCNTL_REG: /* DCNTL */
        if (is_700_mode) {
            s->dcntl = val & ~(NCR_DCNTL_PFF | NCR_DCNTL_STD | DCNTL_EA);
            if (val & 0x01) {
                ncr710_soft_reset(s);
            }
        } else {
            s->dcntl = val & ~(NCR_DCNTL_PFF);
            if (val & NCR_DCNTL_STD) {
                NCR710_DPRINTF("NCR_DCNTL_STD triggered - manually starting SCRIPTS at DSP=0x%08x\n", s->dsp);
                trace_ncr710_NCR_DCNTL_STD_triggered(s->dsp);
                s->waiting = 0;
                s->scripts.running = true;
                s->scripts.pc = s->dsp;
                ncr710_scripts_execute(s);
                s->dcntl &= ~NCR_DCNTL_STD;
            }
        }
        ncr710_update_compatibility_mode(s);
        NCR710_DPRINTF("DCNTL: 0x%02x mode=%s manual=%s\n", val, is_700_mode ? "700" : "710",
                 (s->dmode & NCR_DMODE_MAN) ? "yes" : "no");
        break;

    CASE_SET_REG32(adder, NCR710_ADDER_REG)
        if (is_700_mode) {
            NCR710_DPRINTF("ADDER: write blocked in 700 mode\n");
            return;
        }
        NCR710_DPRINTF("ADDER: 0x%08x\n", s->adder);
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "NCR710: write unknown register %02X\n", offset);
        break;
    }

#undef CASE_SET_REG24
#undef CASE_SET_REG32
}

/*
* QEMU Object Model Registration SysBus NCR710 device
*/

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

    dev = qdev_new(TYPE_SYSBUS_NCR710_SCSI);
    sysbus = SYS_BUS_DEVICE(dev);

    qdev_realize_and_unref(dev, NULL, &error_abort);
    sysbus_mmio_map(sysbus, 0, addr);
    sysbus_connect_irq(sysbus, 0, irq);

    s = SYSBUS_NCR710_SCSI(dev);
    if (!s->ncr710.as) {
        s->ncr710.as = &address_space_memory;
        NCR710_DPRINTF("NCR710: !s->ncr710.as doing it now \n");
    }

    NCR710_DPRINTF("NCR710: Device mapped and IRQ connected\n");
    return dev;
}

static const struct SCSIBusInfo ncr710_scsi_info = {
    .tcq = true,
    .max_target = 8,
    .max_lun = 0,  /* LUN support is buggy */

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
    NCR710_DPRINTF("NCR710: Realize function called\n");
    QTAILQ_INIT(&s->ncr710.queue);
    scsi_bus_init(&s->ncr710.bus, sizeof(s->ncr710.bus), dev, &ncr710_scsi_info);
    s->ncr710.as = &address_space_memory;

    ncr710_dma_fifo_init(&s->ncr710.dma_fifo);
    ncr710_scsi_fifo_init(&s->ncr710.scsi_fifo);

    s->ncr710.ctest8 = NCR710_REVISION_2;
    s->ncr710.scid = 0x80 | NCR710_DEFAULT_HOST_ID;
    s->ncr710.big_endian = false;

    memset(s->ncr710.msg, 0, sizeof(s->ncr710.msg));

    memory_region_init_io(&s->iomem, OBJECT(s), &ncr710_mmio_ops, &s->ncr710,
    "ncr710", 0x100);
    sysbus_init_mmio(SYS_BUS_DEVICE(s), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(s), &s->ncr710.irq);

    NCR710_DPRINTF("NCR710: Device realized with memory region and IRQ\n");
}

static void sysbus_ncr710_init(Object *obj)
{
    SysBusNCR710State *s = SYSBUS_NCR710_SCSI(obj);
    NCR710_DPRINTF("NCR710: Instance init called\n");
    memset(&s->ncr710, 0, sizeof(NCR710State));
    s->ncr710.ctest0 = 0x01;
    s->ncr710.scid = 0x80 | NCR710_DEFAULT_HOST_ID;
    s->ncr710.dstat = NCR_DSTAT_DFE;
}

static void sysbus_ncr710_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sysbus_ncr710_realize;
    device_class_set_legacy_reset(dc, ncr710_device_reset);
    dc->bus_type = NULL;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "NCR53C710 SCSI I/O Processor (SysBus)";
}

static const TypeInfo sysbus_ncr710_info = {
    .name = TYPE_SYSBUS_NCR710_SCSI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(SysBusNCR710State),
    .instance_init = sysbus_ncr710_init,
    .class_init = sysbus_ncr710_class_init,
};

/* Type registration */
static void ncr710_register_types(void)
{
    type_register_static(&sysbus_ncr710_info);
}

type_init(ncr710_register_types)