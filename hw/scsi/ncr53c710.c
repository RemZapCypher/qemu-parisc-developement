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
 *
 * INDEX:
 * 1. Register Definitions
 * 2. Register name functions
 * 3. Parity functions
 * 4. SCSI FIFO Structures
 * 5. Scripts Misc functions
 * 6. DMA functions
 * 7. Scripts functions
 * 8. Read and Write functions
 * 9. QEMU Device model functions
 *
 * [CURRENT ISSUE]
 * 1. The NCR710 shows "Host state after interrupt: BUSY" but after this it should goto Host State: FREE
 * 2. Their is issue with double interrupt occuring one at the correct time and then another one after a delay so need to fix this issue now.
 *
 *
 *
 * Please Ignore below for personal notes during devel--
 * CHECK THIS::
 * [x] add breakpoint in DO_COMMAND
 * [x] LSI_SSTAT0_STO
 * [x] DO_DMA
 * [x] DO_STATUS
 * [ ] DO_MSG_IN
 * [ ] DO_MSG_OUT
 * [ ] EXECUTE_SCRIPT
 *
 * [CURRENT ISSUE] ISTAT should raise an interrupt when scripts complete but doesnt.
 * [NEXT TODO] QUEUE is their so not much issue involve the NCR710 script fixes and then
 *
 *
 * Note
 * The enqueue and dequeue functions need to be properly setup as the scripts is completing one phase and then is suppose to move on to the next but now is stuck inbetween.
 */

#include "qemu/osdep.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "hw/irq.h"
#include "hw/sysbus.h"
#include "hw/scsi/scsi.h"
#include "hw/scsi/ncr53c710.h"
#include "migration/vmstate.h"
#include "system/dma.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "trace.h"
#include "qom/object.h"

#define NCR710_MAX_DEVS 7

/* SCNTL0 (0x00) - SCSI Control Register 0 */
#define NCR710_SCNTL0_TRG    0x01  /* Bit 0: Target Mode */
#define NCR710_SCNTL0_AAP    0x02  /* Bit 1: Assert ATN on Parity Error */
#define NCR710_SCNTL0_EPG    0x04  /* Bit 2: Enable Parity Generation */
#define NCR710_SCNTL0_EPC    0x08  /* Bit 3: Enable Parity Checking */
#define NCR710_SCNTL0_WATN   0x10  /* Bit 4: Select with ATN */
#define NCR710_SCNTL0_START  0x20  /* Bit 5: Start Sequence */
#define NCR710_SCNTL0_ARB0   0x40  /* Bit 6: Arbitration Mode bit 0 */
#define NCR710_SCNTL0_ARB1   0x80  /* Bit 7: Arbitration Mode bit 1 */

/* SCNTL1 (0x01) - SCSI Control Register 1 */
#define NCR710_SCNTL1_RES0   0x01  /* Bit 0: Reserved */
#define NCR710_SCNTL1_RES1   0x02  /* Bit 1: Reserved */
#define NCR710_SCNTL1_AESP   0x04  /* Bit 2: Assert Even SCSI Parity (Force Bad Parity) */
#define NCR710_SCNTL1_RST    0x08  /* Bit 3: Assert SCSI RST Signal */
#define NCR710_SCNTL1_CON    0x10  /* Bit 4: Connected */
#define NCR710_SCNTL1_ESR    0x20  /* Bit 5: Enable Selection/Reselection */
#define NCR710_SCNTL1_ADB    0x40  /* Bit 6: Assert SCSI Data Bus */
#define NCR710_SCNTL1_EXC    0x80  /* Bit 7: Extra Clock Cycle of Data Setup */

/* ISTAT (0x21) - Interrupt Status Register */
#define NCR710_ISTAT_DIP    0x01  /* DMA Interrupt Pending */
#define NCR710_ISTAT_SIP    0x02  /* SCSI Interrupt Pending */
#define NCR710_ISTAT_CON    0x08  /* Connected */
#define NCR710_ISTAT_SIGP   0x20  /* Signal Process */
#define NCR710_ISTAT_RST    0x40  /* Software Reset (write-only) */
#define NCR710_ISTAT_ABRT   0x80  /* Abort Operation */

/* SSTAT0 (0x0D) - SCSI Status Register 0 */
#define NCR710_SSTAT0_PAR    0x01  /* Parity Error */
#define NCR710_SSTAT0_RST    0x02  /* SCSI RST Signal Received */
#define NCR710_SSTAT0_UDC    0x04  /* Unexpected Disconnect */
#define NCR710_SSTAT0_SGE    0x08  /* SCSI Gross Error */
#define NCR710_SSTAT0_SEL    0x10  /* Selected */
#define NCR710_SSTAT0_STO    0x20  /* Selection Timeout */
#define NCR710_SSTAT0_FCMP   0x40  /* Function Complete */
#define NCR710_SSTAT0_MA     0x80  /* Phase Mismatch/ATN */

/* SSTAT1 (0x0E) - SCSI Status Register 1 */
#define NCR710_SSTAT1_ORF    0x02  /* SCSI Output Register Full */
#define NCR710_SSTAT1_ILF    0x04  /* SCSI Input Latch Full */

/* SSTAT2 (0x0F) - SCSI Status Register 2 */
#define NCR710_SSTAT2_FF0    0x01  /* SCSI FIFO Flags bit 0 */
#define NCR710_SSTAT2_FF1    0x02  /* SCSI FIFO Flags bit 1 */
#define NCR710_SSTAT2_FF2    0x04  /* SCSI FIFO Flags bit 2 */
#define NCR710_SSTAT2_FF3    0x08  /* SCSI FIFO Flags bit 3 */

/* SOCL (0x07) / SBCL (0x0B) - SCSI Output/Bus Control Lines */
#define NCR710_SOCL_IO       0x01  /* I/O Signal */
#define NCR710_SOCL_CD       0x02  /* C/D Signal */
#define NCR710_SOCL_MSG      0x04  /* MSG Signal */
#define NCR710_SOCL_ATN      0x08  /* ATN Signal */
#define NCR710_SOCL_SEL      0x10  /* SEL Signal */
#define NCR710_SOCL_BSY      0x20  /* BSY Signal */
#define NCR710_SOCL_ACK      0x40  /* ACK Signal */
#define NCR710_SOCL_REQ      0x80  /* REQ Signal */

/* SBCL bits same as SOCL */
#define NCR710_SBCL_IO       NCR710_SOCL_IO
#define NCR710_SBCL_CD       NCR710_SOCL_CD
#define NCR710_SBCL_MSG      NCR710_SOCL_MSG
#define NCR710_SBCL_ATN      NCR710_SOCL_ATN
#define NCR710_SBCL_SEL      NCR710_SOCL_SEL
#define NCR710_SBCL_BSY      NCR710_SOCL_BSY
#define NCR710_SBCL_ACK      NCR710_SOCL_ACK
#define NCR710_SBCL_REQ      NCR710_SOCL_REQ

/* DSTAT (0x0C) - DMA Status Register */
#define NCR710_DSTAT_IID     0x01  /* Illegal Instruction Detected */
#define NCR710_DSTAT_SIR     0x04  /* SCRIPTS Interrupt Instruction Received */
#define NCR710_DSTAT_SSI     0x08  /* SCRIPTS Step Interrupt */
#define NCR710_DSTAT_ABRT    0x10  /* Aborted */
#define NCR710_DSTAT_BF      0x20  /* Bus Fault */
#define NCR710_DSTAT_MDPE    0x40  /* Master Data Parity Error */
#define NCR710_DSTAT_DFE     0x80  /* DMA FIFO Empty */

/* DCNTL (0x3B) - DMA Control Register */
#define NCR710_DCNTL_COM     0x01  /* Compatibility Mode */
#define NCR710_DCNTL_IRQD    0x02  /* IRQ Disable */
#define NCR710_DCNTL_STD     0x04  /* Start DMA Operation */
#define NCR710_DCNTL_IRQM    0x08  /* IRQ Mode */
#define NCR710_DCNTL_SSM     0x10  /* Single Step Mode */
#define NCR710_DCNTL_PFEN    0x20  /* Pre-fetch Enable */
#define NCR710_DCNTL_PFF     0x40  /* Pre-fetch Flush */
/* NOTE: Bit 7 is reserved in NCR710 (EA/CLSE are LSI-specific) */

/* DMODE (0x38) - DMA Mode Register */
#define NCR710_DMODE_MAN     0x01  /* Manual Start Mode */
#define NCR710_DMODE_BOF     0x02  /* Burst Op Code Fetch Enable */
#define NCR710_DMODE_ERMP    0x04  /* Enable Read Multiple */
#define NCR710_DMODE_ERL     0x08  /* Enable Read Line */
#define NCR710_DMODE_DIOM    0x10  /* Destination I/O Memory Enable */
#define NCR710_DMODE_SIOM    0x20  /* Source I/O Memory Enable */
#define NCR710_DMODE_BL_MASK 0xC0  /* Burst Length Mask */
#define NCR710_DMODE_BL_1    0x00  /* Burst Length: 1 transfer */
#define NCR710_DMODE_BL_2    0x40  /* Burst Length: 2 transfers */
#define NCR710_DMODE_BL_4    0x80  /* Burst Length: 4 transfers */
#define NCR710_DMODE_BL_8    0xC0  /* Burst Length: 8 transfers */

/* CTEST2 (0x16) - Chip Test Register 2 */
#define NCR710_CTEST2_DACK   0x01  /* Data Acknowledge */
#define NCR710_CTEST2_DREQ   0x02  /* Data Request */
#define NCR710_CTEST2_TEOP   0x04  /* SCSI True End of Process */
#define NCR710_CTEST2_PCICIE 0x08  /* PCI Interrupt Cause Enable */
#define NCR710_CTEST2_CM     0x10  /* Configured as Memory */
#define NCR710_CTEST2_CIO    0x20  /* Configured as I/O */
#define NCR710_CTEST2_SIGP   0x40  /* Signal Process */
#define NCR710_CTEST2_DDIR   0x80  /* Data Transfer Direction */

/* CTEST5 (0x19) - Chip Test Register 5 */
#define NCR710_CTEST5_BL2    0x04  /* Burst Length bit 2 */
#define NCR710_CTEST5_DDIR   0x08  /* DMA Direction */
#define NCR710_CTEST5_MASR   0x10  /* Master Control for Set or Reset Pulses */
#define NCR710_CTEST5_DFSN   0x20  /* DMA FIFO Size (Read-only) */
#define NCR710_CTEST5_BBCK   0x40  /* Back-to-Back Clock */
#define NCR710_CTEST5_ADCK   0x80  /* Increment DNAD Register */

/* SCID (0x04) - SCSI Chip ID Register */
#define NCR710_SCID_RRE      0x60  /* Enable Response to Reselection (bits 6-5) */
#define NCR710_SCID_ID_MASK  0x07  /* Chip ID mask (bits 2-0) */

#define NCR710_HOST_ID       7

/* NCR53C710 has 8-byte SCSI FIFO */
#define SCRIPT_STACK_SIZE 8
#define NCR710_FIFO_DEPTH       16  /* General FIFO depth for other operations */
#define NCR710_FIFO_FULL        0x01
#define NCR710_FIFO_EMPTY       0x02
#define NCR710_MAX_MSGIN_LEN 8
#define NCR710_BUF_SIZE         4096

/* Standard SCSI Message Byte Constants */
#define SCSI_MSG_ABORT                  0x06
#define SCSI_MSG_BUS_DEVICE_RESET       0x0c
#define SCSI_MSG_COMMAND_COMPLETE       0x00
#define SCSI_MSG_DISCONNECT             0x04
#define SCSI_MSG_EXTENDED_MESSAGE       0x01
#define SCSI_MSG_IDENTIFY               0x80
#define SCSI_MSG_IGNORE_WIDE_RESIDUE    0x23
#define SCSI_MSG_MESSAGE_PARITY_ERROR   0x09
#define SCSI_MSG_MESSAGE_REJECT         0x07
#define SCSI_MSG_NO_OPERATION           0x08
#define SCSI_MSG_RELEASE_RECOVERY       0x10
#define SCSI_MSG_RESTORE_POINTERS       0x03
#define SCSI_MSG_SAVE_DATA_POINTER      0x02
#define SCSI_MSG_SYNCHRONOUS_DATA_TRANSFER  0x01
#define SCSI_MSG_WIDE_DATA_TRANSFER     0x03

/* Script interrupt codes - matching Linux 53c700 driver expectations */
#define A_GOOD_STATUS_AFTER_STATUS          0x401
#define A_DISCONNECT_AFTER_CMD              0x380
#define A_DISCONNECT_AFTER_DATA             0x580
#define A_DISCONNECT_DURING_DATA            0x780
#define A_RESELECTION_IDENTIFIED            0x1003
#define A_UNEXPECTED_PHASE                  0x20
#define A_FATAL                             0x2000
#define A_DEBUG_INTERRUPT                   0x3000

/* SCSI Script execution states */
#define SCRIPT_STATE_IDLE                  0
#define SCRIPT_STATE_SELECTING             1
#define SCRIPT_STATE_COMMAND               2
#define SCRIPT_STATE_DATA                  3
#define SCRIPT_STATE_STATUS                4
#define SCRIPT_STATE_MESSAGE               5
#define SCRIPT_STATE_DISCONNECTED          6

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



/* SCSI phases */
#define PHASE_DO   0  /* Data out phase */
#define PHASE_DI   1  /* Data in phase */
#define PHASE_CO   2  /* Command phase */
#define PHASE_SI   3  /* Status phase */
#define PHASE_ST   3  /* Status phase (alias) */
#define PHASE_MO   6  /* Message out phase */
#define PHASE_MI   7  /* Message in phase */
#define PHASE_MASK 7  /* Mask for phase bits */


#define NCR710_TAG_VALID     (1 << 16)

static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo);
const char *ncr710_reg_name(int offset);
static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0);
static void ncr710_update_irq(NCR710State *s);
static void ncr710_script_dma_interrupt(NCR710State *s, int stat);
static void ncr710_request_free(NCR710State *s, NCR710Request *p);
static inline void ncr710_dma_read(NCR710State *s, uint32_t addr, void *buf, uint32_t len);
static inline void ncr710_dma_write(NCR710State *s, uint32_t addr, const void *buf, uint32_t len);

static inline int ncr710_irq_on_rsl(NCR710State *s)
{
    return (s->sien0 & NCR710_SSTAT0_SEL) != 0;
}

static void ncr710_clear_pending_irq(NCR710State *s)
{
    if (s->current) {
        DPRINTF("ncr710_soft_reset: Cleaning up leftover s->current=%p\n", s->current);
        if (s->current->req) {
            s->current->req->hba_private = NULL;
        }
        ncr710_request_free(s, s->current);
        s->current = NULL;
    }
}

void ncr710_soft_reset(NCR710State *s)
{
    DPRINTF("Reset\n");
    s->carry = 0;
    s->msg_action = 0;
    s->msg_len = 0;
    s->waiting = 0;
    s->wait_reselect = false;
    s->reselection_id = 0;
    s->dsa = 0;
    s->dnad = 0;
    s->dbc = 0;
    s->temp = 0;
	s->scratch = 0;
    s->istat &= 0x40;
    s->dcmd = 0x40;
    s->dstat = NCR710_DSTAT_DFE;
    s->dien = 0x04;
    s->sien0 = 0;
    s->ctest2 = NCR710_CTEST2_DACK;
    s->ctest3 = 0;
    s->ctest4 = 0;
    s->ctest5 = 0;
    s->dsp = 0;
    s->dsps = 0;
    s->dmode = 0;
    s->dcntl = 0;
    s->scntl0 = 0xc0;
    s->scntl1 = 0;
    s->sstat0 = 0;
    s->sstat1 = 0;
	s->sstat2 = 0;
    s->scid = 0x80;
    s->sxfer = 0;
    s->socl = 0;
    s->sdid = 0;
    s->sbcl = 0;
    s->sidl = 0;
    s->sfbr = 0;
    qemu_set_irq(s->irq, 0);
    ncr710_clear_pending_irq(s);
    ncr710_scsi_fifo_init(&s->scsi_fifo);
}

const char *ncr710_reg_name(int offset)
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

/* Parity functionality*/
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

static uint8_t ncr710_generate_scsi_parity(NCR710State *s, uint8_t data)
{
    uint8_t parity = ncr710_calculate_parity(data);

    if (s->scntl1 & NCR710_SCNTL1_AESP) {
        parity = !parity;  /* Even parity */
    }

    return parity;
}

static bool ncr710_check_scsi_parity(NCR710State *s, uint8_t data, uint8_t parity)
{
    /* Only check if parity checking is enabled */
    if (!(s->scntl0 & NCR710_SCNTL0_EPC)) {
        return true;
    }

    uint8_t expected_parity = ncr710_generate_scsi_parity(s, data);
    return parity == expected_parity;
}

static void ncr710_handle_parity_error(NCR710State *s)
{
    s->sstat0 |= NCR710_SSTAT0_PAR;  /* Set parity error bit */

    /* If parity error ATN is enabled, assert ATN */
    if (s->scntl0 & NCR710_SCNTL0_AAP) {
        s->socl |= NCR710_SOCL_ATN;
    }

    /* Generate interrupt if enabled */
    ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_PAR);
}

/* TODO:: Message byte handling functions - Implement NOW!
static uint8_t ncr710_get_msgbyte(NCR710State *s)
{
    uint8_t data;
    ncr710_dma_read(s, s->dnad, &data, 1);
    s->dnad++;
    s->dbc--;
    return data;
}

static void ncr710_skip_msgbytes(NCR710State *s, unsigned int n)
{
    s->dnad += n;
    s->dbc  -= n;
}
*/

/*
 * NCR710 SCSI FIFO IMPLEMENTATION
 *
 * NCR710 SCSI FIFO Specifications:
 * - Width: 9 bits (8 data bits + 1 parity bit per byte lane)
 * - Data Width: 1 byte (8 bits) per transfer
 * - Depth: 8 transfers deep
 * - Total Capacity: 8-byte FIFO
 *
 * Implementation Features:
 * 1. 8-DEEP FIFO: Exactly matches NCR710 hardware specification
 * 2. PARITY SUPPORT: 9th bit stores parity for each byte
 * 3. FIFO SEMANTICS: First-in-first-out with head pointer and count
 * 4. HARDWARE ACCURACY: Matches actual NCR710 FIFO behavior
 *
 * SCSI FIFO Data Flow:
 * - Enqueue: Add byte at tail position (head + count)
 * - Dequeue: Remove byte from head position
 * - Status: Empty when count=0, Full when count=8
 */

/* SCSI FIFO Operations:
 * - ncr710_scsi_fifo_init() - Initialize 8-deep FIFO
 * - ncr710_scsi_fifo_enqueue() - Add byte to FIFO tail
 * - ncr710_scsi_fifo_dequeue() - Remove byte from FIFO head
 * - ncr710_scsi_fifo_empty/full() - Check FIFO status
 */

/* Initialize a SCSI FIFO - 8 transfers deep */
static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo)
{
    memset(fifo->data, 0, NCR710_SCSI_FIFO_SIZE);
    memset(fifo->parity, 0, NCR710_SCSI_FIFO_SIZE);
    fifo->head = 0;
    fifo->count = 0;
}

/* Check if SCSI FIFO is empty */
static inline bool ncr710_scsi_fifo_empty(NCR710_SCSI_FIFO *fifo)
{
    return fifo->count == 0;
}

/* Check if SCSI FIFO is full */
static inline bool ncr710_scsi_fifo_full(NCR710_SCSI_FIFO *fifo)
{
    return fifo->count == NCR710_SCSI_FIFO_SIZE;
}

/* Enqueue a byte with its parity bit into the SCSI FIFO (add at tail) */
static inline int ncr710_scsi_fifo_enqueue(NCR710_SCSI_FIFO *fifo, uint8_t data, uint8_t parity)
{
    if (ncr710_scsi_fifo_full(fifo)) {
        return -1; /* FIFO full - 8 transfers deep */
    }

    /* Add data at the tail position (head + count) */
    int tail_pos = (fifo->head + fifo->count) % NCR710_SCSI_FIFO_SIZE;
    fifo->data[tail_pos] = data;
    fifo->parity[tail_pos] = parity;
    fifo->count++;

    return 0;
}

/* Dequeue a byte and its parity bit from the SCSI FIFO (remove from head) */
static inline uint8_t ncr710_scsi_fifo_dequeue(NCR710_SCSI_FIFO *fifo, uint8_t *parity)
{
    uint8_t data;

    if (ncr710_scsi_fifo_empty(fifo)) {
        *parity = 0;
        return 0; /* FIFO empty */
    }

    /* Take data from the head position */
    data = fifo->data[fifo->head];
    *parity = fifo->parity[fifo->head];
    fifo->head = (fifo->head + 1) % NCR710_SCSI_FIFO_SIZE;
    fifo->count--;

    return data;
}

static uint8_t ncr710_reg_readb(NCR710State *s, int offset);
static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val);
/* Function prototypes - queue management functions removed */

static inline uint32_t ncr710_read_dword(NCR710State *s, uint32_t addr)
{
    uint32_t buf;
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                      (uint8_t *)&buf, 4);
    /* The NCR710 datasheet saying "operates internally in LE mode"
    * refers to its internal register organization,
    * not how it reads SCRIPTS from host memory.
    * This was intiallhy confusing.
    */
    buf = be32_to_cpu(buf);
    DPRINTF("Read dword %08x from %08x\n", buf, addr);
    return buf;
}

static inline void ncr710_dma_read(NCR710State *s, uint32_t addr, void *buf, uint32_t len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
        buf, len);
    DPRINTF("Read %d bytes from %08x: ", len, addr);
    for (int i = 0; i < len && i < 16; i++) {
        printf("%02x ", ((uint8_t*)buf)[i]);
    }
    printf("\n");
}

static inline void ncr710_dma_write(NCR710State *s, uint32_t addr, const void *buf, uint32_t len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
    DPRINTF("Wrote %d bytes to %08x\n", len, addr);
}

static void ncr710_stop_script(NCR710State *s)
{
    s->script_active = 0;
    s->scntl1 &= ~NCR710_SCNTL1_CON;
    s->istat &= ~NCR710_ISTAT_CON;
}

static void ncr710_update_irq(NCR710State *s)
{
    DPRINTF("NCR710_UPDATE_IRQ: dstat=0x%02x, sstat0=0x%02x, istat=0x%02x\n",
            s->dstat, s->sstat0, s->istat);
    int level;
    static int last_level;

    /* It's unclear whether the DIP/SIP bits should be cleared when the
       Interrupt Status Registers are cleared or when istat0 is read.
       We currently do the former, which seems to work.  */
    level = 0;
    if (s->dstat) {
        if (s->dstat & s->dien) {
            level = 1;
            DPRINTF("NCR710_UPDATE_IRQ: DMA interrupt enabled, level=1 (dstat=0x%02x, dien=0x%02x)\n",
                    s->dstat, s->dien);
        } else {
            DPRINTF("NCR710_UPDATE_IRQ: DMA interrupt disabled (dstat=0x%02x, dien=0x%02x)\n",
                    s->dstat, s->dien);
        }
        s->istat |= NCR710_ISTAT_DIP;
    } else {
        s->istat &= ~NCR710_ISTAT_DIP;
    }

    if (s->sstat0) {
        if ((s->sstat0 & s->sien0)) {
            level = 1;
            DPRINTF("NCR710_UPDATE_IRQ: SCSI interrupt enabled, level=1 (sstat0=0x%02x, sien0=0x%02x)\n",
                    s->sstat0, s->sien0);
        } else {
            DPRINTF("NCR710_UPDATE_IRQ: SCSI interrupt disabled (sstat0=0x%02x, sien0=0x%02x)\n",
                    s->sstat0, s->sien0);
        }
        s->istat |= NCR710_ISTAT_SIP;
    } else {
        s->istat &= ~NCR710_ISTAT_SIP;
    }

    if (level != last_level) {
        DPRINTF("Update IRQ level %d dstat %02x sist %02x%02x (istat now=0x%02x)\n",
                level, s->dstat, s->sstat0, s->sstat1, s->istat);
        last_level = level;
    } else {
        DPRINTF("NCR710_UPDATE_IRQ: IRQ level unchanged (%d), istat=0x%02x\n", level, s->istat);
    }
    qemu_set_irq(s->irq, level);

    if (!level && ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR710_SCNTL1_CON)) {
        DPRINTF("Handled IRQs & disconnected, kernel will handle next command\n");
    }
}

/* Stop SCRIPTS execution and raise a SCSI interrupt.  */
static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0)
{
    uint32_t mask0;

    DPRINTF("SCSI Interrupt 0x%02x prev 0x%02x\n",
            (unsigned)stat0, (unsigned)s->sstat0);
    s->sstat0 |= stat0;
    mask0 = stat0 & s->sien0;
    if (mask0) {
        ncr710_stop_script(s);
    }
    ncr710_update_irq(s);
}

void ncr710_completion_irq_callback(void *opaque)
{
    NCR710State *s = (NCR710State *)opaque;
    uint32_t current_dsps = s->dsps;
    s->dsps = s->saved_dsps;
    DPRINTF("FIX #17: Delayed GOOD_STATUS interrupt firing now (saved_dsps=0x%08x, current_dsps=0x%08x)\n",
            s->saved_dsps, current_dsps);

    if (s->dstat & NCR710_DSTAT_DFE) {
        DPRINTF("FIX #17: Clearing DFE before setting SIR (was 0x%02x)\n", s->dstat);
        s->dstat &= ~NCR710_DSTAT_DFE;
    }
    s->dstat |= NCR710_DSTAT_SIR;
    ncr710_update_irq(s);
    ncr710_stop_script(s);
}

/* Stop SCRIPTS execution and raise a DMA interrupt.  */
static void ncr710_script_dma_interrupt(NCR710State *s, int stat)
{
    DPRINTF("DMA Interrupt 0x%x prev 0x%x\n", stat, s->dstat);
    if (stat == NCR710_DSTAT_SIR && (s->dstat & NCR710_DSTAT_DFE)) {
        DPRINTF("FIX #14: Clearing DFE bit before setting SIR (DSTAT was 0x%02x)\n", s->dstat);
        s->dstat &= ~NCR710_DSTAT_DFE;
    }

    s->dstat |= stat;
    ncr710_update_irq(s);
    ncr710_stop_script(s);
}

inline void ncr710_set_phase(NCR710State *s, int phase)
{
    const char *phase_names[] = {"DO", "DI", "CO", "SI", "4", "5", "MO", "MI"};
    int old_phase = s->sstat2 & PHASE_MASK;
    if (old_phase != phase) {
        DPRINTF("Phase change: %s -> %s (sstat2: 0x%02x -> 0x%02x, script_active=%d)\n",
                phase_names[old_phase],
                phase_names[phase & PHASE_MASK],
                s->sstat2, (s->sstat2 & ~PHASE_MASK) | phase,
                s->script_active);
    }
    s->sstat2 = (s->sstat2 & ~PHASE_MASK) | phase;
	s->ctest0 &= ~1;
	if (phase == PHASE_DI)
		s->ctest0 |= 1;
	s->sbcl &= ~NCR710_SBCL_REQ;
}

/* Resume SCRIPTS execution after a DMA operation.  */
static void ncr710_resume_script(NCR710State *s)
{
    DPRINTF("ncr710_resume_script: waiting=%d, script_active=%d\n", s->waiting, s->script_active);
    s->waiting = 0;
    DPRINTF("ncr710_resume_script: Resuming SCRIPTS\n");
    ncr710_execute_script(s);
}

static void ncr710_disconnect(NCR710State *s)
{
    if (s->waiting == 0) {
        s->scntl1 &= ~NCR710_SCNTL1_CON;
        s->istat &= ~NCR710_ISTAT_CON;  /* Clear CON bit in ISTAT */
    } else {
        DPRINTF("Disconnect: Keeping CON bit set for reselection (waiting=%d)\n", s->waiting);
    }
    s->sstat2 &= ~PHASE_MASK;
}

static void ncr710_bad_selection(NCR710State *s, uint32_t id)
{
    DPRINTF("SELECT failed: Target %d not responding\n", id);

    /* Clear ALL DMA interrupt state since selection timeout should be
     * handled as pure SCSI interrupt, not DMA interrupt */
    s->dstat = 0;  /* Clear all DMA interrupt flags */
    s->dsps = 0;   /* Clear script interrupt data */

    /* Generate SCSI interrupt for selection timeout */
    ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_STO);
    ncr710_disconnect(s);
}

/* Clear selection timeout after Linux has processed it */
static void ncr710_clear_selection_timeout(NCR710State *s)
{
    if (s->sstat0 & NCR710_SSTAT0_STO) {
        DPRINTF("Clearing selection timeout after Linux processing\n");
        s->sstat0 &= ~NCR710_SSTAT0_STO;
        if (s->sstat0 == 0) {
            s->istat &= ~NCR710_ISTAT_SIP;
        }
        ncr710_update_irq(s);
    }
}

/* Initiate a SCSI layer data transfer using FIFOs.  */
static void ncr710_do_dma(NCR710State *s, int out)
{
    DPRINTF("NCR710_DO_DMA START\n");
    uint32_t count;
    uint32_t addr;
    SCSIDevice *dev;

    assert(s->current);
    if (!s->current->dma_len) {
        /* Wait until data is available.  */
        DPRINTF("DMA no data available\n");
        return;
    }

    dev = s->current->req->dev;
    assert(dev);

    count = s->dbc;
    if (count > s->current->dma_len)
        count = s->current->dma_len;

    addr = s->dnad;

    DPRINTF("DMA addr=%d && len=%d\n", addr, count);
    s->dnad += count;
    s->dbc -= count;
     if (s->current->dma_buf == NULL) {
		 s->current->dma_buf = scsi_req_get_buf(s->current->req);
    }
    /* ??? Set SFBR to first data byte.  */
    if (out) {
		ncr710_dma_read(s, addr, s->current->dma_buf, count);
    } else {
		ncr710_dma_write(s, addr, s->current->dma_buf, count);
    }
    s->current->dma_len -= count;
    if (s->current->dma_len == 0) {
        s->current->dma_buf = NULL;
        s->current->pending = 0;  /* Clear pending flag when transfer completes */
        DPRINTF("DMA transfer complete, calling scsi_req_continue (waiting=%d)\n", s->waiting);
		scsi_req_continue(s->current->req);
        DPRINTF("scsi_req_continue returned (waiting=%d, command_complete=%d)\n",
                s->waiting, s->command_complete);
    } else {
        s->current->dma_buf += count;
        ncr710_resume_script(s);
    }
}

static void ncr710_add_msg_byte(NCR710State *s, uint8_t data)
{
    DPRINTF("NCR710_ADD_MSG_BYTE START\n");
    if (s->msg_len >= NCR710_MAX_MSGIN_LEN) {
        BADF("MSG IN data too long\n");
    } else {
        DPRINTF("MSG IN 0x%02x\n", data);
        s->msg[s->msg_len++] = data;
    }
    DPRINTF("NCR710_ADD_MSG_BYTE END\n");
}

static void ncr710_request_free(NCR710State *s, NCR710Request *p)
{
    if (p == s->current) {
        s->current = NULL;
    }
    g_free(p);
}

void ncr710_request_cancelled(SCSIRequest *req)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    NCR710Request *p = (NCR710Request*)req->hba_private;
    DPRINTF("NCR710_REQUEST_CANCELLED START\n");
    req->hba_private = NULL;
    ncr710_request_free(s, p);
	scsi_req_unref(req);
}

/* Record that data is available for a queued command.  Returns zero if
   the device was reselected, nonzero if the IO is deferred.  */
static int ncr710_queue_req(NCR710State *s, SCSIRequest *req, uint32_t len)
{
    NCR710Request *p = (NCR710Request*)req->hba_private;

    DPRINTF("NCR710_QUEUE_REQ START \n");
    if (p->pending) {
        BADF("Multiple IO pending for request %p\n", p);
    }
    p->pending = len;
    if ((s->waiting == 1 && !(s->istat & (NCR710_ISTAT_SIP | NCR710_ISTAT_DIP))) ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR710_SCNTL1_CON) &&
         !(s->istat & (NCR710_ISTAT_SIP | NCR710_ISTAT_DIP)))) {
        DPRINTF("ncr710_queue_req: Processing immediately (waiting=%d, istat=0x%02x)\n",
                s->waiting, s->istat);
        s->current = p;
        return 0;
    } else {
        DPRINTF("Queueing IO tag=0x%x (waiting=%d, istat=0x%02x)\n",
                p->tag, s->waiting, s->istat);
        p->pending = len;
        s->current = p;
        return 1;
    }
    DPRINTF("NCR710_QUEUE_REQ END \n");
}

 /* Callback to indicate that the SCSI layer has completed a command.  */
void ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    NCR710Request *p = (NCR710Request*)req->hba_private;

    DPRINTF("Command complete status=%d\n", (int)req->status);

    s->lcrc = 0;
    s->status = req->status;
    s->command_complete = 2;

    if (p) {
        p->pending = 0;
    }

    DPRINTF("command_complete: s->waiting=%d, s->dbc=%d\n", s->waiting, s->dbc);
    ncr710_set_phase(s, PHASE_ST);

    /* WORKAROUND: Don't free s->current immediately - let driver process completion first
     * The Linux 53c700 driver needs req->hba_private (SCp->host_scribble) to remain
     * valid when processing the completion interrupt. We'll free everything when the
     * next command starts.
     */
    if (req->hba_private == s->current) {
        scsi_req_unref(req);
    }
    DPRINTF("command_complete: waiting=%d, dstat=0x%02x, istat=0x%02x\n",
            s->waiting, s->dstat, s->istat);
    if (s->waiting == 1) {
        DPRINTF("FIX #24: Command completed while paused at WAIT DISCONNECT - resuming SCRIPTS\n");
        DPRINTF("FIX #24: DSP=0x%08x will re-execute WAIT DISCONNECT with command_complete=%d\n",
                s->dsp, s->command_complete);
        ncr710_resume_script(s);

        DPRINTF("FIX #24: SCRIPTS resumed, WAIT DISCONNECT will skip disconnect path\n");
    } else if (s->waiting == 2) {
        DPRINTF("command_complete: Resuming SCRIPTS (waiting=2 - DMA wait)\n");
        ncr710_resume_script(s);
    }
    DPRINTF("NCR710_COMMAND_COMPLETE END\n");
}

 /* Callback to indicate that the SCSI layer has completed a transfer.  */
void ncr710_transfer_data(SCSIRequest *req, uint32_t len)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);

    DPRINTF("NCR710_TRANSFER_DATA START (wait_reselect=%d, waiting=%d)\n",
            s->wait_reselect, s->waiting);
    assert(req->hba_private);
    if (s->waiting == 2) {
        DPRINTF("===== DATA ARRIVED FOR PENDING DMA =====\n");
        DPRINTF("Device has data ready while SCRIPTS waiting for DMA (len=%d)\n", len);
        DPRINTF("Resuming DMA transfer directly - NO reselection interrupt needed\n");

        /* Update current request with data length */
        NCR710Request *p = (NCR710Request *)req->hba_private;
        if (p) {
            p->dma_len = len;
        }
        DPRINTF("Current DSP=0x%08x, backing up by 8 bytes to re-execute DO_DMA\n", s->dsp);
        s->dsp -= 8;  /* Back up to the DO_DMA instruction (8 bytes: opcode + address) */
        DPRINTF("New DSP=0x%08x (DO_DMA instruction will be re-executed)\n", s->dsp);

        s->waiting = 0;
        DPRINTF("Cleared waiting state (was 2), now resuming SCRIPTS\n");

        ncr710_execute_script(s);
        DPRINTF("NCR710_TRANSFER_DATA END (DMA resumed)\n");
        return;
    }

    if (s->wait_reselect) {
        DPRINTF("=====  RESELECTION DETECTED DURING WAIT RESELECT =====\n");
        DPRINTF("Device %d is reselecting with data ready (len=%d)\n", req->dev->id, len);
        DPRINTF("  req->hba_private=%p, s->current=%p\n", req->hba_private, s->current);

        s->current = (NCR710Request *)req->hba_private;
        s->current->dma_len = len;

        s->waiting = 1;  /* Mark as reselection in progress */
    }

    if (req->hba_private != s->current ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR710_SCNTL1_CON))|| s->waiting == 1) {
        if (ncr710_queue_req(s, req, len) != 0) {
            DPRINTF("NCR710_TRANSFER_DATA: Request queued\n");
            return;
        }
        DPRINTF("NCR710_TRANSFER_DATA: Processing immediately (waiting was set)\n");
    }

    /* host adapter (re)connected */
    DPRINTF("Data ready tag=0x%x len=%d\n", req->tag, len);
    s->current->dma_len = len;
    s->command_complete = 1;
    if (!s->current) {
        DPRINTF("Transfer data called with no current request\n");
        return;
    }
    if (s->waiting) {
        DPRINTF("===== RESELECTION EVENT =====\n");
        DPRINTF("Device reselecting after disconnect\n");
        DPRINTF("Reselection state: tag=0x%x, dbc=%d, dma_len=%d, dnad=0x%08x, dsp=0x%08x\n",
                req->tag, s->dbc, s->current->dma_len, s->dnad, s->dsp);
        DPRINTF("Reselection: resume_offset=0x%08x\n", s->current->resume_offset);
        s->scntl1 |= NCR710_SCNTL1_CON;
        s->istat |= NCR710_ISTAT_CON;
        DPRINTF("Reselection: Set CON bit (SCNTL1=0x%02x, ISTAT=0x%02x) so SBCL reads correctly\n",
                s->scntl1, s->istat);
        s->sbcl = NCR710_SBCL_IO | NCR710_SBCL_CD | NCR710_SBCL_MSG |
                  NCR710_SBCL_BSY | NCR710_SBCL_SEL | NCR710_SBCL_REQ;
        DPRINTF("Reselection: Set SBCL=0x%02x (I/O=1, MSG=1, CD=1 for MESSAGE IN phase)\n", s->sbcl);
        uint8_t host_id = (s->scid & 0x07);  /* Extract host ID from SCID register (bits 2-0) */

        /* Use Linux-compatible encoding: target 0 = 0x00, target 1 = 0x01, etc. */
        if (req->dev->id == 0 && host_id == 0) {
            /* Special case: both target and host are ID 0 */
            s->sfbr = 0x00;  /* Linux expects 0x00 for target 0 */
        } else {
            /* For non-zero IDs, use standard bit positions */
            s->sfbr = (req->dev->id == 0 ? 0 : (1 << req->dev->id)) |
                      (host_id == 0 ? 0 : (1 << host_id));
        }
        DPRINTF("Reselection: Captured target ID %d + host ID %d in SFBR (0x%02x) [Linux-compatible encoding]\n",
                req->dev->id, host_id, s->sfbr);

        /* Set phase to MESSAGE IN for GetReselectionData SCRIPTS */
        ncr710_set_phase(s, PHASE_MI);

        /* Prepare reselection message for GetReselectionData to read:
         * - Byte 0: IDENTIFY message (0x80 | LUN)
         * - Bytes 1-2: Tag message if tagged (0x20 = SIMPLE_TAG, then tag number)
         *
         * The GetReselectionData SCRIPTS will read 1 byte (for untagged) or
         * 3 bytes (for tagged) using MOVE instructions in MESSAGE IN phase.
         */
        if (s->current) {
            uint8_t identify_msg = 0x80 | (req->lun & 0x07);  /* IDENTIFY + LUN */
            DPRINTF("Reselection: Preparing IDENTIFY message 0x%02x\n", identify_msg);
            ncr710_add_msg_byte(s, identify_msg);

            /* If this is a tagged command, add tag bytes */
            if (s->current->tag) {
                DPRINTF("Reselection: Adding SIMPLE_TAG 0x20 and tag 0x%02x\n",
                        s->current->tag & 0xff);
                ncr710_add_msg_byte(s, 0x20);  /* SIMPLE_TAG_MSG */
                ncr710_add_msg_byte(s, s->current->tag & 0xff);
            }
        }

        /* Keep DSP at current position (resume_offset) - driver will read it and
         * then jump to GetReselectionData or GetReselectionWithTag */
        DPRINTF("Reselection: DSP remains at 0x%08x (resume_offset)\n", s->dsp);

        s->sstat0 |= NCR710_SSTAT0_SEL;  /* Set SELECTED bit */
        s->istat |= NCR710_ISTAT_SIP;    /* Set SCSI interrupt pending */
        s->dsps = RESELECTED_DURING_SELECTION;  /* Set DSPS to 0x1000 */
        ncr710_update_irq(s);

        DPRINTF("Reselection: Generated RESELECTED_DURING_SELECTION interrupt (DSPS=0x%04x)\n",
                s->dsps);
        DPRINTF("Reselection: Driver will execute GetReselectionData SCRIPTS\n");
        DPRINTF("===== END RESELECTION EVENT =====\n");

        /* Don't continue script execution here - let driver handle reselection interrupt
         * and manually jump to GetReselectionData SCRIPTS */
        s->waiting = 0;  /* Clear waiting flag - reselection is now driver's responsibility */
        return;
    }
    DPRINTF("NCR710_TRANSFER_DATA END\n");
}

static int idbitstonum(int id)
{
	int num = 0;
	while (id > 1) {
		num++;
		id >>= 1;
	}
	if (num > 7)
		num = -1;
	return num;
}

static void ncr710_do_command(NCR710State *s)
{
    SCSIDevice *dev;
    uint8_t buf[16];
    uint32_t id;
    int n;
    int bytes_read;
    DPRINTF("DBC=%d, DNAD=%08x\n", s->dbc, s->dnad);
    if (s->dbc > 16)
        s->dbc = 16;

    /* Read command data directly from memory
     * NOTE: SCSI commands can be up to 16 bytes (e.g., READ_CAPACITY_10 is 10 bytes)
     * but the NCR710 SCSI FIFO is only 8 bytes deep. For command phase, we bypass
     * the FIFO and read directly from memory since commands don't need FIFO buffering
     * I mean this would be accrurate to hardware but "Things we do for convinience and
     * optimization".
     */
    bytes_read = MIN(s->dbc, 16);
    ncr710_dma_read(s, s->dnad, buf, bytes_read);

    DPRINTF("Read %d command bytes directly from memory\n", bytes_read);
    s->dnad += bytes_read;
    s->dbc -= bytes_read;
    s->sfbr = buf[0];  /* Update SFBR with first byte */

    /* Decode and log SCSI command for debugging */
    const char *cmd_name = "UNKNOWN";
    switch (buf[0]) {
        case 0x00: cmd_name = "TEST_UNIT_READY"; break;
        case 0x03: cmd_name = "REQUEST_SENSE"; break;
        case 0x08: cmd_name = "READ_6"; break;
        case 0x0A: cmd_name = "WRITE_6"; break;
        case 0x12: cmd_name = "INQUIRY"; break;
        case 0x15: cmd_name = "MODE_SELECT_6"; break;
        case 0x16: cmd_name = "RESERVE_6"; break;
        case 0x17: cmd_name = "RELEASE_6"; break;
        case 0x1A: cmd_name = "MODE_SENSE_6"; break;
        case 0x1B: cmd_name = "START_STOP_UNIT"; break;
        case 0x1D: cmd_name = "SEND_DIAGNOSTIC"; break;
        case 0x1E: cmd_name = "PREVENT_ALLOW_MEDIUM_REMOVAL"; break;
        case 0x24: cmd_name = "SET_WINDOW (or vendor-specific)"; break;
        case 0x25: cmd_name = "READ_CAPACITY_10"; break;
        case 0x28: cmd_name = "READ_10"; break;
        case 0x2A: cmd_name = "WRITE_10"; break;
        case 0x43: cmd_name = "READ_TOC"; break;
        case 0x5A: cmd_name = "MODE_SENSE_10"; break;
        default: cmd_name = "VENDOR_SPECIFIC_OR_UNKNOWN"; break;
    }

    DPRINTF("Send command len=%d %s(0x%02x) %02x.%02x.%02x.%02x.%02x.%02x\n",
            bytes_read, cmd_name, buf[0], buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);

    if (buf[0] == 0x43) {  // READ_TOC
        DPRINTF("*** READ_TOC COMMAND DETECTED ***\n");
        DPRINTF("*** This command may not be properly supported by the target device ***\n");
        DPRINTF("*** If timeout occurs, it's a SCSI device issue, not NCR710 ***\n");

        // Log full CDB for debugging
        DPRINTF("READ_TOC CDB: ");
        for (int i = 0; i < bytes_read && i < 10; i++) {
            printf("%02x ", buf[i]);
        }
        printf("\n");

        // Log target device info
        id = (s->select_tag >> 8) & 0xff;
        DPRINTF("Target ID: %d, LUN: %d\n", idbitstonum(id), s->current_lun);
        printf("*** NCR710: READ_TOC to target %d - if this hangs, it's a device compatibility issue ***\n",
               idbitstonum(id));
        fflush(stdout);
    }
    /* Special handling for problematic commands */
    s->command_complete = 0;

    id = (s->select_tag >> 8) & 0xff;
    s->lcrc = id;

    DPRINTF("DEVICE LOOKUP: Searching bus=%p for channel=0 target=%d lun=%d\n",
            &s->bus, idbitstonum(id), s->current_lun);
    dev = scsi_device_find(&s->bus, 0, idbitstonum(id), s->current_lun);
    DPRINTF("DEVICE LOOKUP RESULT: device=%p (target %d)\n", dev, idbitstonum(id));

    if (!dev) {
        ncr710_bad_selection(s, id);
        return;
    }

    if (s->current) {
        DPRINTF("NCR710_DO_COMMAND: Freeing leftover s->current=%p before new command\n", s->current);
        ncr710_request_free(s, s->current);
        s->current = NULL;
    }

    s->current = g_new0(NCR710Request, 1);
    DPRINTF("NCR710_DO_COMMAND: Created s->current=%p\n", s->current);
    s->current->tag = s->select_tag;
    s->current->resume_offset = 0;  /* Initialize resume offset */

    s->current->req = scsi_req_new(dev, s->current->tag, s->current_lun,
                                  buf, bytes_read, s->current);
    DPRINTF("NCR710_DO_COMMAND: s->current=%p, req=%p\n", s->current, s->current->req);
    n = scsi_req_enqueue(s->current->req);
    if (n) {
        if (n > 0) {
            ncr710_set_phase(s, PHASE_DI);
            DPRINTF("Command expects data in, set phase to DI\n");
        } else if (n < 0) {
            ncr710_set_phase(s, PHASE_DO);
            DPRINTF("Command expects data out, set phase to DO\n");
        }
        scsi_req_continue(s->current->req);
    }
    if (!s->command_complete) {
        if (n) {
            DPRINTF("Command requires data transfer, waiting for transfer_data callback\n");
        } else {
            ncr710_set_phase(s, PHASE_SI);
            DPRINTF("No immediate data transfer, set phase to SI (status)\n");
        }
    }
    DPRINTF("NCR710_DO_COMMAND END\n");
}

static void ncr710_do_status(NCR710State *s)
{
    uint8_t status = s->status;
    uint8_t parity = 0;
    printf("NCR710_DO_STATUS: status=0x%02x\n", status);

    if (s->dbc != 1)
        BADF("Bad Status move\n");
    s->dbc = 1;
    s->sfbr = status;

    /* Generate parity if enabled and enqueue status byte */
    if (s->scntl0 & NCR710_SCNTL0_EPG) {
        parity = ncr710_generate_scsi_parity(s, status);
    }
    ncr710_scsi_fifo_enqueue(&s->scsi_fifo, status, parity);

    /* Dequeue status byte and write to memory */
    status = ncr710_scsi_fifo_dequeue(&s->scsi_fifo, &parity);
    if (s->scntl0 & NCR710_SCNTL0_EPC) {
        if (!ncr710_check_scsi_parity(s, status, parity)) {
            ncr710_handle_parity_error(s);
        }
    }
    ncr710_dma_write(s, s->dnad, &status, 1);

    /* Advance address/count */
    s->dnad += 1;
    s->dbc  -= 1;

    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = 1;
    ncr710_add_msg_byte(s, 0); /* COMMAND COMPLETE */
}

static void ncr710_do_msgin(NCR710State *s)
{
    int len;
    len = s->msg_len;
    if (len > s->dbc) {
        len = s->dbc;
    }
    s->sfbr = s->msg[0];

    for (int i = 0; i < len; i++) {
        uint8_t parity = 0;
        if (s->scntl0 & NCR710_SCNTL0_EPG) {
            parity = ncr710_generate_scsi_parity(s, s->msg[i]);
        }
        ncr710_scsi_fifo_enqueue(&s->scsi_fifo, s->msg[i], parity);
    }

    /* Dequeue message bytes and write to memory */
    uint8_t buf[NCR710_MAX_MSGIN_LEN];
    for (int i = 0; i < len; i++) {
        uint8_t parity;
        buf[i] = ncr710_scsi_fifo_dequeue(&s->scsi_fifo, &parity);
        if (s->scntl0 & NCR710_SCNTL0_EPC) {
            if (!ncr710_check_scsi_parity(s, buf[i], parity)) {
                ncr710_handle_parity_error(s);
            }
        }
    }
    ncr710_dma_write(s, s->dnad, buf, len);

    s->dnad += len;
    s->dbc  -= len;
    s->sidl = s->msg[len - 1];
    s->msg_len -= len;
    if (s->msg_len) {
        memmove(s->msg, s->msg + len, s->msg_len);
        return;
    }
    switch (s->msg_action) {
    case 0:
        ncr710_set_phase(s, PHASE_CO);
        break;
    case 1:
        ncr710_disconnect(s);
        break;
    case 2:
        ncr710_set_phase(s, PHASE_DO);
        break;
    case 3:
        ncr710_set_phase(s, PHASE_DI);
        break;
    default:
        abort();
    }
}

static void ncr710_do_msgout(NCR710State *s)
{
    uint32_t current_tag;
    NCR710Request *current_req;

    if (s->current) {
        current_tag = s->current->tag;
        current_req = s->current;
    } else {
        current_tag = s->select_tag;
        current_req = NULL;  /* No queue to search - kernel manages commands */
    }

    DPRINTF("MSG out len=%d\n", s->dbc);

    /* Process message bytes in FIFO-sized chunks */
    while (s->dbc > 0) {
        /* Read data from memory and process through FIFO */
        int to_move = MIN((int)s->dbc, NCR710_SCSI_FIFO_SIZE);
        uint8_t temp_buf[NCR710_SCSI_FIFO_SIZE];

        /* Read from memory first */
        ncr710_dma_read(s, s->dnad, temp_buf, to_move);

        /* Enqueue data with parity generation */
        int filled = 0;
        for (int j = 0; j < to_move && !ncr710_scsi_fifo_full(&s->scsi_fifo); j++) {
            uint8_t parity = 0;
            if (s->scntl0 & NCR710_SCNTL0_EPG) {
                parity = ncr710_generate_scsi_parity(s, temp_buf[j]);
            }
            if (ncr710_scsi_fifo_enqueue(&s->scsi_fifo, temp_buf[j], parity) == 0) {
                filled++;
            } else {
                break;
            }
        }

        if (filled <= 0) {
            break;
        }

        /* Dequeue into a local buffer for parsing */
        uint8_t buf[NCR710_SCSI_FIFO_SIZE];
        int bytes = 0;
        for (int j = 0; j < filled && !ncr710_scsi_fifo_empty(&s->scsi_fifo); j++) {
            uint8_t parity;
            buf[bytes] = ncr710_scsi_fifo_dequeue(&s->scsi_fifo, &parity);
            if (s->scntl0 & NCR710_SCNTL0_EPC) {
                if (!ncr710_check_scsi_parity(s, buf[bytes], parity)) {
                    ncr710_handle_parity_error(s);
                }
            }
            bytes++;
        }

        /* Update DMA address/count as we consumed host memory */
        s->dnad += bytes;
        s->dbc  -= bytes;

        /* Parse message stream */
        int i = 0;
        while (i < bytes) {
            uint8_t msg = buf[i++];
            s->sfbr = msg;

            switch (msg) {
            case SCSI_MSG_COMMAND_COMPLETE: /* 0x00 - NOP / padding byte / Command Complete */
                DPRINTF("MSG: Command Complete/NOP/padding byte (0x%02x)\n", msg);
                /* Just ignore padding bytes, continue processing */
                break;

            case SCSI_MSG_DISCONNECT: /* 0x04 - Disconnect */
                DPRINTF("MSG: Disconnect (0x%02x)\n", msg);
                ncr710_disconnect(s);
                break;

            case SCSI_MSG_MESSAGE_REJECT: /* 0x07 - Message Reject */
                DPRINTF("MSG: Message Reject (0x%02x)\n", msg);
                /* Target is rejecting our last message */
                ncr710_set_phase(s, PHASE_CO);
                break;

            case SCSI_MSG_NO_OPERATION: /* 0x08 - NOP */
                DPRINTF("MSG: No Operation (0x%02x)\n", msg);
                ncr710_set_phase(s, PHASE_CO);
                break;

            case SCSI_MSG_SAVE_DATA_POINTER: /* 0x02 - Save Data Pointer */
                DPRINTF("MSG: Save Data Pointer (0x%02x)\n", msg);
                /* Save current data pointer for later restore */
                break;

            case SCSI_MSG_RESTORE_POINTERS: /* 0x03 - Restore Pointers */
                DPRINTF("MSG: Restore Pointers (0x%02x)\n", msg);
                /* Restore previously saved data pointer */
                break;

            case SCSI_MSG_EXTENDED_MESSAGE: { /* 0x01 - Extended message */
                DPRINTF("MSG: Extended Message (0x%02x)\n", msg);
                if (i >= bytes) {
                    /* Not enough data; let next chunk continue parsing */
                    i--; /* rewind one to reparse later */
                    goto out_chunk;
                }
                int ext_len = buf[i++];

                if (i >= bytes) {
                    i -= 2; /* rewind msg + ext_len for next chunk */
                    goto out_chunk;
                }
                uint8_t ext_code = buf[i++];

                DPRINTF("Extended message 0x%x (len %d)\n", ext_code, ext_len);

                switch (ext_code) {
                case 1: /* SDTR (ignore body) */
                    DPRINTF("SDTR (ignored)\n");
                    /* Body has 2 bytes, may span chunks: skip what we have */
                    {
                        int skip = MIN(2, bytes - i);
                        i += skip;
                        /* If not all skipped this chunk, rest will arrive in next loop */
                    }
                    break;
                case 3: /* WDTR (ignore body) */
                    DPRINTF("WDTR (ignored)\n");
                    if (i < bytes) {
                        i++; /* skip one param byte if present this chunk */
                    }
                    break;
                default:
                    /* Unknown extended message: reject */
                    goto bad;
                }
                break;
            }

            case 0x20: /* SIMPLE queue */
                if (i < bytes) {
                    s->select_tag |= buf[i++] | NCR710_TAG_VALID;
                    DPRINTF("SIMPLE queue tag=0x%x\n", s->select_tag & 0xff);
                } else {
                    /* Tag byte not in this chunk; rewind and reparse next loop */
                    i--; /* put back msg */
                    goto out_chunk;
                }
                break;

            case 0x21: /* HEAD of queue (not implemented) */
                BADF("HEAD queue not implemented\n");
                if (i < bytes) {
                    s->select_tag |= buf[i++] | NCR710_TAG_VALID;
                } else {
                    i--;
                    goto out_chunk;
                }
                break;

            case 0x22: /* ORDERED queue (not implemented) */
                BADF("ORDERED queue not implemented\n");
                if (i < bytes) {
                    s->select_tag |= buf[i++] | NCR710_TAG_VALID;
                } else {
                    i--;
                    goto out_chunk;
                }
                break;

            case 0x0d: /* ABORT TAG */
                DPRINTF("MSG: ABORT TAG tag=0x%x\n", current_tag);
                if (current_req) {
                    scsi_req_cancel(current_req->req);
                }
                ncr710_disconnect(s);
                break;

            case SCSI_MSG_ABORT: /* 0x06 - ABORT */
            case 0x0e: /* CLEAR QUEUE */
            case SCSI_MSG_BUS_DEVICE_RESET: /* 0x0c - BUS DEVICE RESET */
                if (msg == SCSI_MSG_ABORT) DPRINTF("MSG: ABORT (0x%02x) tag=0x%x\n", msg, current_tag);
                if (msg == 0x0e) DPRINTF("MSG: CLEAR QUEUE (0x%02x) tag=0x%x\n", msg, current_tag);
                if (msg == SCSI_MSG_BUS_DEVICE_RESET) DPRINTF("MSG: BUS DEVICE RESET (0x%02x) tag=0x%x\n", msg, current_tag);

                if (s->current) {
                    scsi_req_cancel(s->current->req);
                }
                /* Kernel manages queue - device only handles current command */
                ncr710_disconnect(s);
                break;

            default:
                if (msg & SCSI_MSG_IDENTIFY) {
                    bool disconnect_allowed = (msg & 0x40) != 0;
                    uint8_t lun = msg & 0x07;
                    s->current_lun = lun;
                    DPRINTF("MSG: IDENTIFY (0x%02x) - LUN=%d, disconnect=%s\n",
                           msg, lun, disconnect_allowed ? "allowed" : "not allowed");
                    ncr710_set_phase(s, PHASE_CO);
                    break;
                }

                /* Unknown message - reject it */
                DPRINTF("MSG: Unknown message 0x%02x - rejecting\n", msg);
                goto bad;
            }
        }

    out_chunk:
        /* If we broke early due to incomplete extended message parameters,
           the outer while will refill and continue parsing */
        continue;
    }

    return;

bad:
    BADF("Unimplemented/Invalid message 0x%02x\n", s->sfbr);
    /* Send MESSAGE REJECT back to target */
    ncr710_set_phase(s, PHASE_MI);
    ncr710_add_msg_byte(s, 7); /* MESSAGE REJECT */
    s->msg_action = 0;
}

static void ncr710_memcpy(NCR710State *s, uint32_t dest, uint32_t src, int count)
{
    DPRINTF("SCRIPTS MEMCPY: dest=0x%08x src=0x%08x count=%d (DSP=0x%08x)\n",
            dest, src, count, s->dsp);

    /* Direct memory to memory transfer using temporary buffer like reference implementation */
    #define NCR710_BUF_SIZE 4096
    uint8_t buf[NCR710_BUF_SIZE];

    while (count) {
        int chunk = MIN(count, NCR710_BUF_SIZE);

        /* Read from source */
        ncr710_dma_read(s, src, buf, chunk);

        /* Write to destination */
        ncr710_dma_write(s, dest, buf, chunk);

        DPRINTF("MEMCPY chunk: src=0x%08x dest=0x%08x chunk=%d\n",
                src, dest, chunk);

        src += chunk;
        dest += chunk;
        count -= chunk;
    }
}

static void ncr710_wait_reselect(NCR710State *s)
{
    DPRINTF("WAIT RESELECT instruction: Entering reselection wait state\n");

    s->wait_reselect = true;
    s->waiting = 1;
    s->script_active = false;  /* Pause SCRIPTS execution */

    s->scntl1 &= ~NCR710_SCNTL1_CON;
    s->istat &= ~NCR710_ISTAT_CON;

    DPRINTF("WAIT RESELECT: SCRIPTS paused, waiting for device to reselect\n");
    DPRINTF("WAIT RESELECT: DSP=0x%08x (will resume here after reselection)\n", s->dsp);
}

/* Timer callback to continue script execution */
void ncr710_script_timer_callback(void *opaque)
{
    NCR710State *s = opaque;

    DPRINTF("Script timer callback, waiting=%d, script_active=%d\n",
            s->waiting, s->script_active);

    if (s->script_active) {
        ncr710_execute_script(s);
    }
}

void ncr710_reselection_retry_callback(void *opaque)
{
    NCR710State *s = opaque;

    DPRINTF("Reselection retry callback: Processing queued request\n");

    if (!s->current || s->current->pending == 0) {
        DPRINTF("Reselection retry: No pending request (current=%p, pending=%d)\n",
                s->current, s->current ? s->current->pending : 0);
        return;
    }

    if (s->waiting != 1) {
        DPRINTF("Reselection retry: Not in waiting state (waiting=%d)\n", s->waiting);
        return;
    }

    if (s->istat & (NCR710_ISTAT_SIP | NCR710_ISTAT_DIP)) {
        DPRINTF("Reselection retry: Interrupts still pending (istat=0x%02x), rescheduling\n", s->istat);
        timer_mod(s->reselection_retry_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
        return;
    }

    DPRINTF("Reselection retry: Triggering reselection for tag=0x%x (pending=%d)\n",
            s->current->tag, s->current->pending);

    NCR710Request *p = s->current;
    uint32_t len = p->pending;
    p->pending = 0;  /* Clear pending flag */

    SCSIRequest *req = p->req;
    s->command_complete = 0;
    p->dma_len = len;

    s->scntl1 |= NCR710_SCNTL1_CON;
    s->istat |= NCR710_ISTAT_CON;
    DPRINTF("Reselection retry: Set CON bit (SCNTL1=0x%02x, ISTAT=0x%02x)\n", s->scntl1, s->istat);

    s->sbcl = NCR710_SBCL_IO | NCR710_SBCL_CD | NCR710_SBCL_MSG |
              NCR710_SBCL_BSY | NCR710_SBCL_SEL | NCR710_SBCL_REQ;
    DPRINTF("Reselection retry: Set SBCL=0x%02x\n", s->sbcl);

    uint8_t host_id = (s->scid & 0x07);
    if (req->dev->id == 0 && host_id == 0) {
        s->sfbr = 0x00;
    } else {
        s->sfbr = (req->dev->id == 0 ? 0 : (1 << req->dev->id)) |
                  (host_id == 0 ? 0 : (1 << host_id));
    }
    DPRINTF("Reselection retry: Captured target ID %d in SFBR (0x%02x)\n", req->dev->id, s->sfbr);

    ncr710_set_phase(s, PHASE_MI);

    uint8_t identify_msg = 0x80 | (req->lun & 0x07);
    ncr710_add_msg_byte(s, identify_msg);

    if (p->tag) {
        ncr710_add_msg_byte(s, 0x20);  /* SIMPLE_TAG_MSG */
        ncr710_add_msg_byte(s, p->tag & 0xff);
    }

    s->dsp = p->resume_offset - 8;
    DPRINTF("Reselection retry: Set DSP=0x%08x (resume_offset=0x%08x - 8)\n", s->dsp, p->resume_offset);

    s->dsps = RESELECTED_DURING_SELECTION;  /* Set DSPS to 0x1000 */
    s->sstat0 |= NCR710_SSTAT0_SEL;         /* Set SELECTED bit */
    s->istat |= NCR710_ISTAT_SIP;           /* Set SCSI interrupt pending */
    ncr710_update_irq(s);
    DPRINTF("Reselection retry: Generated RESELECTED_DURING_SELECTION interrupt (DSPS=0x%04x)\n", s->dsps);

    s->waiting = 0;
}

/* Enhanced script execution with proper autonomous flow */
void ncr710_execute_script(NCR710State *s)
{
    uint32_t insn;
    uint32_t addr;
    int opcode;
    int insn_processed = 0;
    // const int MAX_INSTRUCTIONS = 1000; /* Prevent infinite loops */

    DPRINTF("SCRIPT_EXEC START: DSP=0x%08x, active=%d, waiting=%d, istat=0x%02x\n",
            s->dsp, s->script_active, s->waiting, s->istat);

    s->script_active = 1;

again:
    insn_processed++;

    insn = ncr710_read_dword(s, s->dsp);
    if (!insn) {
        /* If we receive an empty opcode increment the DSP by 4 bytes
           instead of 8 and execute the next opcode at that location */
        s->dsp += 4;
        goto again;
    }
    addr = ncr710_read_dword(s, s->dsp + 4);
    DPRINTF("SCRIPTS dsp=%08x opcode %08x arg %08x\n", s->dsp, insn, addr);
    s->dsps = addr;
    s->dcmd = insn >> 24;
    s->dsp += 8;
    switch (insn >> 30) {
    case 0: /* Block move.  */
        if (s->sstat0 & NCR710_SSTAT0_STO) {
            DPRINTF("Delayed select timeout\n");
            ncr710_stop_script(s);
            ncr710_update_irq(s);
            break;
        }
        s->dbc = insn & 0xffffff;
        if (insn & (1 << 29)) {
            /* Indirect addressing.  */
            addr = ncr710_read_dword(s, addr);
        } else if (insn & (1 << 28)) {
            uint32_t buf[2];
            int32_t offset;
            /* Table indirect addressing.  */

            /* 32-bit Table indirect */
            offset = sextract32(addr, 0, 24);
			ncr710_dma_read(s, s->dsa + offset, buf, 8);
            /* byte count is stored in bits 0:23 only */
            s->dbc = cpu_to_le32(buf[0]) & 0xffffff;
            addr = cpu_to_le32(buf[1]);

        }
        /* Check phase match for block move instructions */
        if ((s->sstat2 & PHASE_MASK) != ((insn >> 24) & 7)) {
            uint8_t current_phase = s->sstat2 & PHASE_MASK;
            uint8_t expected_phase = (insn >> 24) & 7;
            const char *phase_names[] = {"DO", "DI", "CO", "SI", "4", "5", "MO", "MI"};

            DPRINTF("Phase mismatch: got %s(%d) expected %s(%d)\n",
                    phase_names[current_phase], current_phase,
                    phase_names[expected_phase], expected_phase);
            DPRINTF("NCR53C710: Phase mismatch - generating SCSI interrupt (MA bit)\n");
            ncr710_set_phase(s, current_phase);
            s->sbcl |= NCR710_SBCL_REQ;
            ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_MA);
            ncr710_stop_script(s);
            break;
        }

        s->dnad = addr;
        switch (s->sstat2 & 0x7) {
        case PHASE_DO:
            s->waiting = 2;
            ncr710_do_dma(s, 1);
            break;
        case PHASE_DI:
            s->waiting = 2;
            ncr710_do_dma(s, 0);
            if (s->waiting != 0) {
                /* Async - stop and wait */
                break;
            }
            /* Sync - continue execution */
            DPRINTF("DO_DMA: Synchronous completion, continuing SCRIPTS\n");
            break;
        case PHASE_CO:
            ncr710_do_command(s);
            break;
        case PHASE_SI:
            ncr710_do_status(s);
            break;
        case PHASE_MO:
            ncr710_do_msgout(s);
            break;
        case PHASE_MI:
            ncr710_do_msgin(s);
            break;
        default:
            BADF("Unimplemented phase %d\n", s->sstat2 & PHASE_MASK);
        }
        s->ctest5 = (s->ctest5 & 0xfc) | ((s->dbc >> 8) & 3);
        s->sbcl = s->dbc; /* TODO: Investigate */
        break;

    case 1: /* IO or Read/Write instruction.  */
        opcode = (insn >> 27) & 7;
        if (opcode < 5) {
            uint32_t id;

            if (insn & (1 << 25)) {
                id = ncr710_read_dword(s, s->dsa + sextract32(insn, 0, 24));
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
                if (s->scntl1 & NCR710_SCNTL1_CON) {
                    if (insn & (1 << 24)) {
                        DPRINTF("Already reselected, continuing with GetReselectionData (not jumping to alt address)\n");
                    } else {
                        DPRINTF("Already reselected, jumping to alternative address\n");
                        s->dsp = s->dnad;
                        break;
                    }
                } else if (!scsi_device_find(&s->bus, 0, idbitstonum(id), 0)) {
                    DPRINTF("SELECT: Target %d (bitmask 0x%02x) not found\n", idbitstonum(id), id);
                    ncr710_bad_selection(s, id);
                    break;
                } else {
                    DPRINTF("SELECT: Target %d (bitmask 0x%02x)%s\n", idbitstonum(id), id, insn & (1 << 24) ? " with ATN" : "");

                    /* ??? Linux drivers compain when this is set.  Maybe
                       it only applies in low-level mode (unimplemented).
                    ncr710_script_scsi_interrupt(s, NCR710_SIST0_CMP, 0); */
                    s->select_tag = id << 8;
                    s->scntl1 |= NCR710_SCNTL1_CON;

                    if (insn & (1 << 24)) {
                        s->socl |= NCR710_SOCL_ATN;
                        ncr710_set_phase(s, PHASE_MO);
                        DPRINTF("SELECT: Set phase to MSG_OUT for target %d\n", idbitstonum(id));
                    } else {
                        ncr710_set_phase(s, PHASE_CO);
                        DPRINTF("SELECT: Set phase to COMMAND for target %d\n", idbitstonum(id));
                    }
                }
                break;
            case 1: /* Disconnect */
                DPRINTF("Wait Disconnect\n");

                if (s->command_complete != 0) {
                    DPRINTF("FIX #24: Command complete (=%d) before disconnect - skipping disconnect path\n",
                            s->command_complete);
                    s->scntl1 &= ~NCR710_SCNTL1_CON;
                    s->istat &= ~NCR710_ISTAT_CON;
                    if (s->waiting == 1) {
                        DPRINTF("FIX #24: Clearing waiting=1 since command complete\n");
                        s->waiting = 0;
                    }
                    DPRINTF("FIX #24: Continuing to Status phase (no disconnect interrupt)\n");
                } else {
                    DPRINTF("FIX #24: Command not complete - pausing at WAIT DISCONNECT\n");
                    if (s->current) {
                        s->current->resume_offset = s->dsp;
                        DPRINTF("FIX #24: Saved resume_offset=0x%08x for tag=0x%x\n",
                                s->current->resume_offset, s->current->tag);
                    }

                    s->waiting = 1;
                    ncr710_stop_script(s);
                    DPRINTF("FIX #24: SCRIPTS paused at WAIT DISCONNECT (waiting=1)\n");
                }
                break;
            case 2: /* Wait Reselect */
                if (!ncr710_irq_on_rsl(s)) {
                    ncr710_wait_reselect(s);
                }
                break;
            case 3: /* Set */
                DPRINTF("Set%s%s%s%s\n",
                        insn & (1 << 3) ? " ATN" : "",
                        insn & (1 << 6) ? " ACK" : "",
                        insn & (1 << 9) ? " TM" : "",
                        insn & (1 << 10) ? " CC" : "");
                if (insn & (1 << 3)) {
                    s->socl |= NCR710_SOCL_ATN;
                    ncr710_set_phase(s, PHASE_MO);
                }
                if (insn & (1 << 9)) {
                    DPRINTF("Target Mode bit set (acknowledged but not fully implemented)\n");
                }
                if (insn & (1 << 10))
                    s->carry = 1;
                break;
            case 4: /* Clear */
                DPRINTF("Clear%s%s%s%s\n",
                        insn & (1 << 3) ? " ATN" : "",
                        insn & (1 << 6) ? " ACK" : "",
                        insn & (1 << 9) ? " TM" : "",
                        insn & (1 << 10) ? " CC" : "");
                if (insn & (1 << 3)) {
                    s->socl &= ~NCR710_SOCL_ATN;
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
#ifdef DEBUG_NCR710
            static const char *opcode_names[3] =
                {"Write", "Read", "Read-Modify-Write"};
            static const char *operator_names[8] =
                {"MOV", "SHL", "OR", "XOR", "AND", "SHR", "ADD", "ADC"};
#endif

            reg = ((insn >> 16) & 0x7f) | (insn & 0x80);
            data8 = (insn >> 8) & 0xff;
            opcode = (insn >> 27) & 7;
            xoperator = (insn >> 24) & 7;
            DPRINTF("%s reg 0x%x %s data8=0x%02x sfbr=0x%02x%s\n",
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

            if ((insn & 0x002e0000) != 0) {
                DPRINTF("TRANSFER CONTROL: insn=0x%08x sfbr=0x%02x addr=0x%08x\n",
                        insn, s->sfbr, addr);
            }
            if (s->sstat0 & NCR710_SSTAT0_STO) {
                DPRINTF("Delayed select timeout\n");
                ncr710_stop_script(s);
                break;
            }
            cond = jmp = (insn & (1 << 19)) != 0;
            if (cond == jmp && (insn & (1 << 21))) {
                DPRINTF("Compare carry %d\n", s->carry == jmp);
                cond = s->carry != 0;
            }
            if (cond == jmp && (insn & (1 << 17))) {
                DPRINTF("Compare phase %d %c= %d\n",
                        (s->sstat2 & PHASE_MASK),
                        jmp ? '=' : '!',
                        ((insn >> 24) & 7));
                cond = (s->sstat2 & PHASE_MASK) == ((insn >> 24) & 7);
            }
            if (cond == jmp && (insn & (1 << 18))) {
                uint8_t mask;

                mask = (~insn >> 8) & 0xff;
                DPRINTF("Compare data 0x%x & 0x%x %c= 0x%x\n",
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
                    DPRINTF("Jump to 0x%08x\n", addr);
                    s->dsp = addr;
                    break;
                case 1: /* Call */
                    DPRINTF("Call 0x%08x\n", addr);
                    DPRINTF("Saving return address: 0x%08x\n", s->dsp);
                    s->temp = s->dsp;
                    s->dsp = addr;
                    break;
                case 2: /* Return */
                    DPRINTF("Return to 0x%08x\n", s->temp);
                    if (s->temp == 0) {
                        DPRINTF("ERROR: Returning to address 0! Stopping script.\n");
                        ncr710_script_dma_interrupt(s, NCR710_DSTAT_IID);
                        break;
                    }
                    s->dsp = s->temp;
                    break;
                case 3: /* Interrupt */
                    DPRINTF("Interrupt 0x%08x\n", s->dsps);
                    /* Decode common interrupt codes using new constants */
                    switch (s->dsps) {
                    case MSG_IN_BEFORE_CMD: /* 0x250 */
                        DPRINTF("SCRIPTS: Message In received before Command phase (normal)\n");
                        break;
                    case UNEXPECTED_PHASE_BEFORE_CMD: /* 0x220 */
                        DPRINTF("SCRIPTS: Unexpected phase before Command\n");
                        break;
                    case GOOD_STATUS_AFTER_STATUS: /* 0x401 */
                        DPRINTF("SCRIPTS: Good status after Status phase (success)\n");
                        break;
                    case DISCONNECT_AFTER_DATA: /* 0x580 */
                        DPRINTF("SCRIPTS: Disconnect after Data phase\n");
                        break;
                    case REJECT_MSG_BEFORE_CMD: /* 0x270 */
                        DPRINTF("SCRIPTS: Message Reject received before Command\n");
                        break;
                    case DISCONNECT_AFTER_CMD: /* 0x380 */
                        DPRINTF("SCRIPTS: Disconnect after Command phase\n");
                        break;
                    case MSG_IN_AFTER_CMD: /* 0x350 */
                        DPRINTF("SCRIPTS: Message In after Command phase\n");
                        break;
                    case UNEXPECTED_PHASE_AFTER_CMD: /* 0x320 */
                        DPRINTF("SCRIPTS: Unexpected phase after Command\n");
                        break;
                    case NOT_MSG_OUT_AFTER_SELECTION: /* 0x110 */
                        DPRINTF("SCRIPTS: Expected MSG_OUT after selection, got different phase\n");
                        break;
                    case MSG_IN_AFTER_STATUS: /* 0x440 */
                        DPRINTF("SCRIPTS: Message In after Status phase\n");
                        break;
                    case DISCONNECT_DURING_DATA: /* 0x780 */
                        DPRINTF("SCRIPTS: Disconnect during Data phase\n");
                        break;
                    case RESELECTION_IDENTIFIED: /* 0x1003 */
                        DPRINTF("SCRIPTS: Reselection identified\n");
                        break;
                    case FATAL: /* 0x2000 */
                        DPRINTF("SCRIPTS: Fatal error\n");
                        break;
                    case DEBUG_INTERRUPT: /* 0x3000 */
                        DPRINTF("SCRIPTS: Debug interrupt\n");
                        break;
                    default:
                        DPRINTF("SCRIPTS: Unknown interrupt code 0x%x\n", s->dsps);
                        break;
                    }
                    if ((insn & (1 << 20)) != 0) {
                        ncr710_update_irq(s);
                    } else {
                        if (s->dsps == GOOD_STATUS_AFTER_STATUS) {
                            DPRINTF("Script completion: Processing GOOD_STATUS_AFTER_STATUS\n");
                            DPRINTF("Script completion: Command state preserved for driver processing\n");
                        }

                        if (s->dsps == GOOD_STATUS_AFTER_STATUS) {
                            DPRINTF("FIX #17: Generating immediate GOOD_STATUS interrupt (DSPS=0x%08x)\n", s->dsps);
                            ncr710_script_dma_interrupt(s, NCR710_DSTAT_SIR);
                            s->command_complete = 0;
                        } else {
                            ncr710_script_dma_interrupt(s, NCR710_DSTAT_SIR);
                        }
                    }
                    break;
                default:
                    DPRINTF("Illegal transfer control\n");
                    ncr710_script_dma_interrupt(s, NCR710_DSTAT_IID);
                    break;
                }
            } else {
                DPRINTF("Control condition failed\n");
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
            dest = ncr710_read_dword(s, s->dsp);
            s->dsp += 4;
            ncr710_memcpy(s, dest, addr, insn & 0xffffff);
        } else {
            uint8_t data[8];  /* Max 8 bytes for DSA/DSP registers */
            int reg;
            int n;
            int i;
            uint32_t original_addr = addr;
            bool dsa_relative = (insn & (1 << 28)) != 0;
            bool is_load = (insn & (1 << 24)) != 0;

            if (dsa_relative) {
                addr = s->dsa + sextract32(addr, 0, 24);
                DPRINTF("%s: DSA-relative addressing: DSA=0x%08x + offset=0x%06x = 0x%08x\n",
                        is_load ? "LOAD" : "STORE", s->dsa, sextract32(original_addr, 0, 24), addr);
            }

            n = (insn & 7);
            if (n == 0) n = 8;  /* 0 means 8 bytes */

            reg = (insn >> 16) & 0xff;

            if (is_load) {
                ncr710_dma_read(s, addr, data, n);
                DPRINTF("LOAD reg 0x%02x size %d from addr 0x%08x = ", reg, n, addr);
                for (i = 0; i < n; i++) {
                    DPRINTF("%02x ", data[i]);
                }
                DPRINTF("\n");

                for (i = 0; i < n; i++) {
                    ncr710_reg_writeb(s, reg + i, data[i]);
                }

                if (reg == NCR710_DSA_REG && n == 4) {
                    uint32_t new_dsa = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24);
                    DPRINTF("LOAD DSA: Set DSA register to 0x%08x\n", new_dsa);
                }
            } else {
                DPRINTF("STORE reg 0x%02x size %d to addr 0x%08x: ", reg, n, addr);
                for (i = 0; i < n; i++) {
                    data[i] = ncr710_reg_readb(s, reg + i);
                    DPRINTF("%02x ", data[i]);
                }
                DPRINTF("\n");
                ncr710_dma_write(s, addr, data, n);
            }
        }
    }

    /* Continue script execution if still active and not waiting */
    if (s->script_active && s->waiting == 0) {
        if (s->dcntl & NCR710_DCNTL_SSM) {
            ncr710_script_dma_interrupt(s, NCR710_DSTAT_SSI);
            return;
        } else {
            goto again;
        }
    } else if (s->waiting == 1) {
        DPRINTF("SCRIPTS waiting for DMA/transfer_data, keeping script active\n");
        /* Keep script active - it will be resumed when transfer_data is called */
        return;
    } else if (s->waiting == 2 || s->waiting == 3) {
        if (s->command_complete == 2) {
            DPRINTF("SCRIPTS waiting=2 but command completed, continuing\n");
            s->waiting = 0;
            goto again;
        }
        DPRINTF("SCRIPTS in waiting state %d, will be resumed later\n", s->waiting);
        return;
    }    DPRINTF("SCRIPT_EXEC END: stopped (active=%d, waiting=%d, DSP=0x%08x, istat=0x%02x)\n",
            s->script_active, s->waiting, s->dsp, s->istat);
}

static uint8_t ncr710_reg_readb(NCR710State *s, int offset)
{
    uint8_t ret = 0;

#define CASE_GET_REG24(name, addr) \
    case addr: \
        printf("READ REG24 %s[0x%02x] = 0x%02x\n", #name, addr, s->name & 0xff); \
        ret = s->name & 0xff; break; \
    case addr + 1: \
        printf("READ REG24 %s[0x%02x] = 0x%02x\n", #name, addr + 1, (s->name >> 8) & 0xff); \
        ret = (s->name >> 8) & 0xff; break; \
    case addr + 2: \
        printf("READ REG24 %s[0x%02x] = 0x%02x\n", #name, addr + 2, (s->name >> 16) & 0xff); \
        ret = (s->name >> 16) & 0xff; break;

#define CASE_GET_REG32(name, addr) \
    case addr: \
        printf("READ REG32 %s[0x%02x] = 0x%02x\n", #name, addr, s->name & 0xff); \
        ret = s->name & 0xff; break; \
    case addr + 1: \
        printf("READ REG32 %s[0x%02x] = 0x%02x\n", #name, addr + 1, (s->name >> 8) & 0xff); \
        ret = (s->name >> 8) & 0xff; break; \
    case addr + 2: \
        printf("READ REG32 %s[0x%02x] = 0x%02x\n", #name, addr + 2, (s->name >> 16) & 0xff); \
        ret = (s->name >> 16) & 0xff; break; \
    case addr + 3: \
        printf("READ REG32 %s[0x%02x] = 0x%02x\n", #name, addr + 3, (s->name >> 24) & 0xff); \
        ret = (s->name >> 24) & 0xff; break;

    printf("DEBUG: NCR710 register read - offset=0x%02x\n", offset);
    switch (offset) {
        case NCR710_SCNTL0_REG: /* SCNTL0 */
            ret = s->scntl0;
            break;
        case NCR710_SCNTL1_REG: /* SCNTL1 */
            ret = s->scntl1;
            break;
        case NCR710_SDID_REG: /* SDID */
            ret = s->sdid;
            break;
        case NCR710_SIEN_REG: /* SIEN */
            ret = s->sien0;
            break;
        case NCR710_SCID_REG:
            ret = s->scid;
            if ((ret & 0x7F) == 0) {
                ret = 0x80 | NCR710_HOST_ID;
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
            DPRINTF("SFBR read: 0x%02x (waiting=%d, script_active=%d)\n",
                    ret, s->waiting, s->script_active);
            break;
        case NCR710_SIDL_REG: /* SIDL */
            ret = s->sidl;
            break;
        case NCR710_SBDL_REG: /* SBDL */
            ret = s->sbdl;
            break;
        case NCR710_SBCL_REG: /* SBCL */
            ret = 0;
            if (s->scntl1 & NCR710_SCNTL1_CON) {
                ret = s->sstat2 & PHASE_MASK;
                ret |= s->sbcl;
                if (s->socl & NCR710_SOCL_ATN)
                    ret |= NCR710_SBCL_ATN;
            }
            break;
        case NCR710_DSTAT_REG: /* DSTAT */
            ret = s->dstat;

            DPRINTF("DSTAT read: 0x%02x (script_active=%d, waiting=%d, dsps=0x%08x)\n",
                    ret, s->script_active, s->waiting, s->dsps);

            /* Not freeing s->current here:: driver needs it for completion processing.
             * It will be freed when the next command starts.
             */
            if (s->dstat & NCR710_DSTAT_SIR) {
                DPRINTF("DSTAT read: Script interrupt cleared after read (was 0x%02x)\n", s->dstat);
            }
            s->dstat = 0;  /* Clear all DMA interrupt status bits */
            s->dstat |= NCR710_DSTAT_DFE;  /* Set DFE back after clearing (always "ready") */
            s->istat &= ~NCR710_ISTAT_DIP;
            ncr710_update_irq(s);

            if (s->waiting == 1 && s->current && s->current->pending > 0) {
                DPRINTF("DSTAT cleared with queued reselection: Scheduling deferred retry for tag=0x%x (pending=%d)\n",
                        s->current->tag, s->current->pending);
                /* Schedule timer to fire immediately (next event loop iteration) */
                timer_mod(s->reselection_retry_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
            }

            if (!s->script_active && s->current && s->current->pending > 0 && s->command_complete == 2) {
                DPRINTF("DSTAT cleared: Resuming deferred command completion for tag=0x%x (pending=%d)\n",
                        s->current->tag, s->current->pending);
                s->current->pending = 0;
                s->waiting = 0;  /* Make sure waiting is cleared */
                ncr710_resume_script(s);
            }

            if (s->waiting && s->current && s->current->pending > 0 && s->command_complete == 2) {
                DPRINTF("DSTAT cleared: Resuming deferred command completion for tag=0x%x\n",
                        s->current->tag);
                s->current->pending = 0;
                ncr710_resume_script(s);
            }

            return ret;
        case NCR710_SSTAT0_REG: /* SSTAT0 */
            ret = s->sstat0;
            /* For selection timeouts, we don't clear SSTAT0 immediately on read.
             * Linux needs to read the value and then use it for processing.
             * We'll clear it later when the interrupt is fully processed. */
            if (s->sstat0 != 0 && !(s->sstat0 & NCR710_SSTAT0_STO)) {
                /* Clear non-selection-timeout SCSI interrupts immediately */
                s->sstat0 = 0;
                s->istat &= ~NCR710_ISTAT_SIP;
                ncr710_update_irq(s);
                if (s->sbcl != 0) {
                    DPRINTF("SSTAT0 read: Clearing SBCL (was 0x%02x) after interrupt acknowledged\n", s->sbcl);
                    s->sbcl = 0;
                }
            }
            /* Note: Selection timeout (STO) will be cleared when Linux writes to
             * a register or when the next command starts */
            break;
        case NCR710_SSTAT1_REG: /* SSTAT1 */
            ret = s->sstat0;
            printf("DEBUG: SSTAT1 read (53C700 SSTAT0 alias at 0x0E), returning 0x%02x\n", ret);
            break;
        case NCR710_SSTAT2_REG: /* SSTAT2 */
            ret = s->dstat;
            printf("DEBUG: SSTAT2 read (53C700 DSTAT alias at 0x0F), returning 0x%02x (dsps=0x%08x)\n", ret, s->dsps);

            if (s->dstat & NCR710_DSTAT_SIR) {
                printf("DEBUG: SSTAT2/DSTAT alias: Script interrupt cleared after read (was 0x%02x)\n", s->dstat);
            }
            s->dstat = 0;
            s->istat &= ~NCR710_ISTAT_DIP;
            ncr710_update_irq(s);
            break;
        CASE_GET_REG32(dsa, NCR710_DSA_REG)
            break;
        case NCR710_CTEST0_REG: /* CTEST0 */
            ret = s->ctest0;
            break;
        case NCR710_CTEST1_REG: /* CTEST1 */
            ret = s->ctest1;
            break;
        case NCR710_CTEST2_REG: /* CTEST2 */
            ret = s->ctest2;
            /* DMA FIFO is always empty in direct transfer mode */
            s->ctest2 |= 0x04;
            break;
        case NCR710_CTEST3_REG: /* CTEST3 */
            ret = s->ctest3;
            if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
                uint8_t parity;
                ret = ncr710_scsi_fifo_dequeue(&s->scsi_fifo, &parity);
                if (parity) {
                    s->ctest2 |= 0x10;
                } else {
                    s->ctest2 &= ~0x10;
                }
            }
            break;
        case NCR710_CTEST4_REG: /* CTEST4 */
            ret = s->ctest4;
            break;
        case NCR710_CTEST5_REG: /* CTEST5 */
            ret = s->ctest5;
            break;
        case NCR710_CTEST6_REG: /* CTEST6 */
            ret = s->ctest6;
            /* No DMA FIFO to pop with direct transfers */
            break;
        case NCR710_CTEST7_REG: /* CTEST7 */
            ret = s->ctest7;
            break;
        CASE_GET_REG32(temp, NCR710_TEMP_REG)
        case NCR710_DFIFO_REG: /* DFIFO */
            /* With direct transfers, DMA FIFO is always empty */
            ret = s->dfifo;
            s->dfifo = 0;  /* DMA FIFO count is always 0 */
            ret = s->dfifo;
            break;
        case NCR710_ISTAT_REG: /* ISTAT */
            ret = s->istat;
            DPRINTF("ISTAT read: 0x%02x (CON=%d, DIP=%d, SIP=%d) dstat=0x%02x dsps=0x%08x\n",
                    ret, !!(ret & NCR710_ISTAT_CON), !!(ret & NCR710_ISTAT_DIP), !!(ret & NCR710_ISTAT_SIP),
                    s->dstat, s->dsps);
            break;
        case NCR710_CTEST8_REG: /* CTEST8 */
            ret = s->istat;
            printf("DEBUG: CTEST8/ISTAT read at offset 0x22, returning ISTAT=0x%02x (dstat=0x%02x)\n", ret, s->dstat);
            break;
        case NCR710_LCRC_REG: /* LCRC */
            ret = s->lcrc;
            break;
        CASE_GET_REG24(dbc, NCR710_DBC_REG)
        case NCR710_DCMD_REG: /* DCMD */
            ret = s->dcmd;
            break;
        CASE_GET_REG32(dnad, NCR710_DNAD_REG)
        case NCR710_DSP_REG:
            printf("READ REG32 dsp[0x%02x] = 0x%02x\n", NCR710_DSP_REG, s->dsp & 0xff);
            ret = s->dsp & 0xff;
            break;
        case NCR710_DSP_REG + 1:
            printf("READ REG32 dsp[0x%02x] = 0x%02x\n", NCR710_DSP_REG + 1, (s->dsp >> 8) & 0xff);
            ret = (s->dsp >> 8) & 0xff;
            break;
        case NCR710_DSP_REG + 2:
            printf("READ REG32 dsp[0x%02x] = 0x%02x\n", NCR710_DSP_REG + 2, (s->dsp >> 16) & 0xff);
            ret = (s->dsp >> 16) & 0xff;
            break;
        case NCR710_DSP_REG + 3:
            printf("READ REG32 dsp[0x%02x] = 0x%02x\n", NCR710_DSP_REG + 3, (s->dsp >> 24) & 0xff);
            ret = (s->dsp >> 24) & 0xff;
            if (s->dsps == GOOD_STATUS_AFTER_STATUS && (s->dstat & NCR710_DSTAT_SIR)) {
                DPRINTF("FIX #16: Driver read DSP with DSPS=0x401 and SIR pending - clearing SIR to prevent duplicate interrupt\n");
                s->dstat &= ~NCR710_DSTAT_SIR;
                s->istat &= ~NCR710_ISTAT_DIP;
                ncr710_update_irq(s);
            }
            break;
        case NCR710_DSPS_REG:
            printf("READ REG32 dsps[0x%02x] = 0x%02x\n", NCR710_DSPS_REG, s->dsps & 0xff);
            ret = s->dsps & 0xff;
            break;
        case NCR710_DSPS_REG + 1:
            printf("READ REG32 dsps[0x%02x] = 0x%02x\n", NCR710_DSPS_REG + 1, (s->dsps >> 8) & 0xff);
            ret = (s->dsps >> 8) & 0xff;
            break;
        case NCR710_DSPS_REG + 2:
            printf("READ REG32 dsps[0x%02x] = 0x%02x\n", NCR710_DSPS_REG + 2, (s->dsps >> 16) & 0xff);
            ret = (s->dsps >> 16) & 0xff;
            break;
        case NCR710_DSPS_REG + 3:
            printf("READ REG32 dsps[0x%02x] = 0x%02x\n", NCR710_DSPS_REG + 3, (s->dsps >> 24) & 0xff);
            ret = (s->dsps >> 24) & 0xff;
            if (!(s->dstat & NCR710_DSTAT_SIR) && s->dsps != 0) {
                DPRINTF("FIX #15: Clearing stale DSPS (was 0x%08x) after complete read with SIR acknowledged\n", s->dsps);
                s->dsps = 0;
            }
            break;
        CASE_GET_REG32(scratch, NCR710_SCRATCH_REG)
            break;
        case NCR710_DMODE_REG: /* DMODE */
            ret = s->dmode;
            NCR710_DPRINTF("NCR710: DMODE read: 0x%02x\n", ret);
            break;
        case NCR710_DIEN_REG: /* DIEN */
            ret = s->dien;
            break;
        case NCR710_DWT_REG: /* DWT */
            ret = s->dwt;
            break;
        case NCR710_DCNTL_REG: /* DCNTL */
            ret = s->dcntl;
            return ret;
        CASE_GET_REG32(adder, NCR710_ADDER_REG)
            break;
        default:
            NCR710_DPRINTF("NCR710: invalid read at offset 0x%x\n", (int)offset);
            ret = 0;
            break;
    }

#undef CASE_GET_REG24
#undef CASE_GET_REG32
    return ret;
}

static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val)
{
    uint8_t old_val;

#define CASE_SET_REG24(name, addr) \
    case addr    : \
        printf("WRITE REG24 %s[0x%02x] = 0x%02x\n", #name, addr, val); \
        s->name &= 0xffffff00; s->name |= val;       break; \
    case addr + 1: \
        printf("WRITE REG24 %s[0x%02x] = 0x%02x\n", #name, addr + 1, val); \
        s->name &= 0xffff00ff; s->name |= val << 8;  break; \
    case addr + 2: \
        printf("WRITE REG24 %s[0x%02x] = 0x%02x\n", #name, addr + 2, val); \
        s->name &= 0xff00ffff; s->name |= val << 16; break;

#define CASE_SET_REG32(name, addr) \
    case addr    : \
        printf("WRITE REG32 %s[0x%02x] = 0x%02x\n", #name, addr, val); \
        s->name &= 0xffffff00; s->name |= val;       break; \
    case addr + 1: \
        printf("WRITE REG32 %s[0x%02x] = 0x%02x\n", #name, addr + 1, val); \
        s->name &= 0xffff00ff; s->name |= val << 8;  break; \
    case addr + 2: \
        printf("WRITE REG32 %s[0x%02x] = 0x%02x\n", #name, addr + 2, val); \
        s->name &= 0xff00ffff; s->name |= val << 16; break; \
    case addr + 3: \
        printf("WRITE REG32 %s[0x%02x] = 0x%02x\n", #name, addr + 3, val); \
        s->name &= 0x00ffffff; s->name |= val << 24; break;

    /* Multi-method debug output to ensure visibility */
    NCR710_DPRINTF("NCR710: Write %s[0x%02x] = 0x%02x\n", ncr710_reg_name(offset), offset, val);
    trace_ncr710_reg_write(ncr710_reg_name(offset), offset, val);

    switch (offset) {
    case NCR710_SCNTL0_REG: /* SCNTL0 */
        old_val = s->scntl0;
        s->scntl0 = val;
        break;

    case NCR710_SCNTL1_REG: /* SCNTL1 */
        old_val = s->scntl1;
        s->scntl1 = val;
        NCR710_DPRINTF("NCR710: SCNTL1: 0x%02x->0x%02x%s%s\n", old_val, val,
                 (val & NCR710_SCNTL1_RST) ? " RST" : "",
                 (val & NCR710_SCNTL1_ADB) ? " ADB" : "");
        NCR710_DPRINTF("NCR710: SCNTL1: 0x%02x->0x%02x%s%s\n", old_val, val,
                (val & NCR710_SCNTL1_RST) ? " RST" : "",
                (val & NCR710_SCNTL1_ADB) ? " ADB" : "");

        /* Handle Assert Even SCSI Parity (AESP) bit changes */
        if ((val & NCR710_SCNTL1_AESP) != (old_val & NCR710_SCNTL1_AESP)) {
            /* trace_ncr710_parity_sense_changed((val & NCR710_SCNTL1_AESP) != 0 ? "even" : "odd"); */
        }

        if (val & NCR710_SCNTL1_RST) {
            if (!(s->sstat0 & NCR710_SSTAT0_RST)) {
                s->sstat0 |= NCR710_SSTAT0_RST;
                ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_RST);
            }
            /* Enhanced reset handling for second implementation */
            if (!(old_val & NCR710_SCNTL1_RST)) {
                NCR710_DPRINTF("NCR710: SCNTL1: SCSI bus reset initiated\n");
                ncr710_soft_reset(s);
            }
        } else {
            s->sstat0 &= ~NCR710_SSTAT0_RST;
        }
        break;

    case NCR710_SDID_REG: /* SDID */
        s->sdid = val & 0x0F; /* Only lower 4 bits are valid */
        NCR710_DPRINTF("NCR710: SDID: set target ID=%d\n", s->sdid);
        break;

    case NCR710_SIEN_REG: /* SIEN */
        s->sien0 = val;
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
        s->sstat1 |= NCR710_SSTAT1_ORF; /* SCSI Output Register Full */
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
        break;

    case NCR710_CTEST0_REG: /* CTEST0 */
        s->ctest0 = val;
        NCR710_DPRINTF("CTEST0: 0x%02x tolerant=%s\n", val, (val & 0x01) ? "on" : "off");
        if (val & 0x01) {
            s->tolerant_enabled = true;
        } else {
            s->tolerant_enabled = false;
        }
        break;

    case NCR710_CTEST1_REG: /* CTEST1, read-only */
        s->ctest1 = val;
        break;

    case NCR710_CTEST2_REG: /* CTEST2, read-only */
        s->ctest2 = val;
        break;

    case NCR710_CTEST3_REG: /* CTEST3 */
        s->ctest3 = val;
        NCR710_DPRINTF("CTEST3: Control register write 0x%02x\n", val);
        break;

    case NCR710_CTEST4_REG: /* CTEST4 */
        s->ctest4 = val;
        break;

    case NCR710_CTEST5_REG: /* CTEST5 */
        s->ctest5 = val;
        break;

    case NCR710_CTEST6_REG: /* CTEST6 */
        s->ctest6 = val;
        NCR710_DPRINTF("CTEST6: Value 0x%02x (no DMA FIFO with direct transfers)\n", val);
        /* No DMA FIFO to push to with direct transfers */
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
        break;

    case NCR710_ISTAT_REG: /* ISTAT */
        old_val = s->istat;

        if ((old_val & NCR710_ISTAT_DIP) && !(val & NCR710_ISTAT_DIP)) {
            DPRINTF("ISTAT write: Linux clearing DMA interrupt - resetting state (DSPS was 0x%08x)\n", s->dsps);
            s->dstat = 0;  /* Clear all DMA status bits */
            s->dsps = 0;   /* Clear script interrupt data after Linux processes it */
            DPRINTF("FIX #15: Cleared DSPS to prevent double-processing\n");
        }

        if ((old_val & NCR710_ISTAT_SIP) && !(val & NCR710_ISTAT_SIP)) {
            DPRINTF("ISTAT write: Linux clearing SCSI interrupt - resetting state\n");
            s->sstat0 = 0;  /* Clear all SCSI status bits */
        }

        s->istat = (val & ~(NCR710_ISTAT_DIP | NCR710_ISTAT_SIP)) |
                  (s->istat & (NCR710_ISTAT_DIP | NCR710_ISTAT_SIP));
        ncr710_update_irq(s);

        if (val & NCR710_ISTAT_ABRT) {
            ncr710_script_dma_interrupt(s, NCR710_DSTAT_ABRT);
        }
        break;

    case NCR710_CTEST8_REG: /* CTEST8 */

        if (val & 0x08) {
            /* No DMA FIFO to flush with direct transfers */
            NCR710_DPRINTF("CTEST8: FIFO flush command (no-op with direct transfers)\n");
            s->dstat |= NCR710_DSTAT_DFE;  /* Set DMA FIFO Empty */
        }
        if (val & 0x04) {
            NCR710_DPRINTF("CTEST8: Clearing SCSI FIFO\n");
            ncr710_scsi_fifo_init(&s->scsi_fifo);
            s->dstat |= NCR710_DSTAT_DFE;  /* Set DMA FIFO Empty */
        }
        break;
    case NCR710_LCRC_REG: /* LCRC */
        s->lcrc = val;
        NCR710_DPRINTF("LCRC: 0x%02x\n", val);
        break;

    CASE_SET_REG24(dbc, NCR710_DBC_REG)

    case NCR710_DCMD_REG: /* DCMD */
        s->dcmd = val;
        break;

    CASE_SET_REG32(dnad, NCR710_DNAD_REG)
    case 0x2c: /* DSP[0:7] */
        {
            uint32_t old_dsp = s->dsp;
            s->dsp &= 0xffffff00;
            s->dsp |= val;
            NCR710_DPRINTF("Write DSP -> 0: 0x%02x, DSP now=0x%08x\n", val, s->dsp);
            if ((old_dsp & 0xff) != val && s->current && s->current->resume_offset != 0) {
                DPRINTF("*** DSP BYTE 0 CHANGED: 0x%08x -> 0x%08x (resume_offset=0x%08x) ***\n",
                        old_dsp, s->dsp, s->current->resume_offset);
            }
        }
        break;
    case 0x2d: /* DSP[8:15] */
        {
            uint32_t old_dsp = s->dsp;
            s->dsp &= 0xffff00ff;
            s->dsp |= val << 8;
            NCR710_DPRINTF("Write DSP -> 1: 0x%02x, DSP now=0x%08x\n", val, s->dsp);
            if (((old_dsp >> 8) & 0xff) != val && s->current && s->current->resume_offset != 0) {
                DPRINTF("*** DSP BYTE 1 CHANGED: 0x%08x -> 0x%08x (resume_offset=0x%08x) ***\n",
                        old_dsp, s->dsp, s->current->resume_offset);
            }
        }
        break;
    case 0x2e: /* DSP[16:23] */
        {
            uint32_t old_dsp = s->dsp;
            s->dsp &= 0xff00ffff;
            s->dsp |= val << 16;
            NCR710_DPRINTF("Write DSP -> 2: 0x%02x, DSP now=0x%08x\n", val, s->dsp);
            if (((old_dsp >> 16) & 0xff) != val && s->current && s->current->resume_offset != 0) {
                DPRINTF("*** DSP BYTE 2 CHANGED: 0x%08x -> 0x%08x (resume_offset=0x%08x) ***\n",
                        old_dsp, s->dsp, s->current->resume_offset);
            }
        }
        break;
    case 0x2f: /* DSP[24:31] */
        {
            uint32_t old_dsp = s->dsp;
            s->dsp &= 0x00ffffff;
            s->dsp |= val << 24;
            NCR710_DPRINTF("NCR710: DSP write byte 3: 0x%02x, DSP FINAL=0x%08x\n", val, s->dsp);
            NCR710_DPRINTF("DSP WRITE prev_state(active=%d, waiting=%d, istat=0x%02x)\n",
                    s->script_active, s->waiting, s->istat);

            if (s->current && s->current->resume_offset != 0) {
                if (s->dsp == s->current->resume_offset) {
                    DPRINTF("*** DRIVER PATCHED DSP TO RESUME_OFFSET! ***\n");
                    DPRINTF("Old DSP:       0x%08x\n", old_dsp);
                    DPRINTF("New DSP:       0x%08x\n", s->dsp);
                    DPRINTF("Resume Offset: 0x%08x <- Matching now\n", s->current->resume_offset);
                    DPRINTF("Tag:           0x%08x\n", s->current->tag);
                    DPRINTF("SCRIPTS will now resume from disconnect point\n");
                } else {
                    DPRINTF("*** DSP PATCHED: 0x%08x -> 0x%08x (resume_offset=0x%08x, Damn, NOT MATCH!) ***\n",
                            old_dsp, s->dsp, s->current->resume_offset);
                }
            }

            s->waiting = 0;
            s->script_active = 1;
            s->istat |= NCR710_ISTAT_CON;
            ncr710_clear_selection_timeout(s);
            NCR710_DPRINTF("Starting script execution DMODE=0x%02x & DSP=0x%08x\n", s->dmode, s->dsp);
            ncr710_execute_script(s);
        }
        break;
    CASE_SET_REG32(dsps, NCR710_DSPS_REG)
    CASE_SET_REG32(scratch, NCR710_SCRATCH_REG)
        NCR710_DPRINTF("SCRATCH: 0x%08x\n", s->scratch);
        break;

    case NCR710_DMODE_REG: /* DMODE */
        s->dmode = val;
        NCR710_DPRINTF("NCR710: DMODE write: 0x%02x\n", val);
        switch (val & NCR710_DMODE_BL_MASK) {
        case 0x00: s->burst_length = 1; break;
        case 0x40: s->burst_length = 2; break;
        case 0x80: s->burst_length = 4; break;
        case 0xC0: s->burst_length = 8; break;
        }
        NCR710_DPRINTF("DMODE: 0x%02x burst=%d manual_mode=%s\n", val, s->burst_length,
                      (val & NCR710_DMODE_MAN) ? "ON" : "OFF");
        break;

    case NCR710_DIEN_REG: /* DIEN */
        s->dien = val;
        NCR710_DPRINTF("DIEN: interrupt enable=0x%02x\n", val);
        ncr710_update_irq(s);
        break;

    case NCR710_DWT_REG: /* DWT */
        s->dwt = val;
        NCR710_DPRINTF("DWT: watchdog timeout=0x%02x\n", val);
        break;

    case NCR710_DCNTL_REG: /* DCNTL */
        s->dcntl = val & ~(NCR710_DCNTL_PFF);
        if (val & NCR710_DCNTL_STD) {
            NCR710_DPRINTF("NCR710_DCNTL_STD triggered - manually starting SCRIPTS \
                 at DSP=0x%08x\n", s->dsp);
            s->waiting = 0;
            ncr710_execute_script(s);
            s->dcntl &= ~NCR710_DCNTL_STD;
        }
        break;

    CASE_SET_REG32(adder, NCR710_ADDER_REG)
        NCR710_DPRINTF("ADDER: 0x%08x\n", s->adder);
        break;

    default:
        NCR710_DPRINTF("NCR710: write unknown register %02X\n", offset);
        break;
    }

#undef CASE_SET_REG24
#undef CASE_SET_REG32
}

/* Memory region operations for NCR710 registers */
uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR710State *s = opaque;
    uint8_t offset = addr & 0xff;
    uint8_t val = ncr710_reg_readb(s, offset);
    trace_ncr710_reg_read(ncr710_reg_name(offset), offset, val);
    return val;
}

void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{
    NCR710State *s = opaque;
    uint8_t offset = addr & 0xff;
    uint8_t val8 = val & 0xff;
    trace_ncr710_reg_write(ncr710_reg_name(offset), offset, val8);
    ncr710_reg_writeb(s, offset, val8);
}

/* Device reset */
static void ncr710_device_reset(DeviceState *dev)
{
    SysBusNCR710State *sysbus_dev = SYSBUS_NCR710_SCSI(dev);
    NCR710State *s = &sysbus_dev->ncr710;

    ncr710_soft_reset(s);
}

static const struct SCSIBusInfo ncr710_scsi_info = {
    .tcq = true,
    .max_target = 8,
    .max_lun = 8,  /* LUN support buggy, eh? */

    .transfer_data = ncr710_transfer_data,
    .complete = ncr710_command_complete,
    .cancel = ncr710_request_cancelled,
};

static const MemoryRegionOps ncr710_mmio_ops = {
    .read = ncr710_reg_read,
    .write = ncr710_reg_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
};

static const VMStateDescription vmstate_ncr710_scsi_fifo = {
    .name = "ncr710_scsi_fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8_ARRAY(data, NCR710_SCSI_FIFO, NCR710_SCSI_FIFO_SIZE),
        VMSTATE_UINT8_ARRAY(parity, NCR710_SCSI_FIFO, NCR710_SCSI_FIFO_SIZE),
        VMSTATE_INT32(count, NCR710_SCSI_FIFO),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_ncr710 = {
    .name = "ncr710",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(scntl0, NCR710State),
        VMSTATE_UINT8(scntl1, NCR710State),
        VMSTATE_UINT8(sdid, NCR710State),
        VMSTATE_UINT8(sien0, NCR710State),
        VMSTATE_UINT8(scid, NCR710State),
        VMSTATE_UINT8(sxfer, NCR710State),
        VMSTATE_UINT8(sodl, NCR710State),
        VMSTATE_UINT8(socl, NCR710State),
        VMSTATE_UINT8(sfbr, NCR710State),
        VMSTATE_UINT8(sidl, NCR710State),
        VMSTATE_UINT8(sbdl, NCR710State),
        VMSTATE_UINT8(sbcl, NCR710State),
        VMSTATE_UINT8(dstat, NCR710State),
        VMSTATE_UINT8(sstat0, NCR710State),
        VMSTATE_UINT8(sstat1, NCR710State),
        VMSTATE_UINT8(sstat2, NCR710State),
        VMSTATE_UINT8(ctest0, NCR710State),
        VMSTATE_UINT8(ctest1, NCR710State),
        VMSTATE_UINT8(ctest2, NCR710State),
        VMSTATE_UINT8(ctest3, NCR710State),
        VMSTATE_UINT8(ctest4, NCR710State),
        VMSTATE_UINT8(ctest5, NCR710State),
        VMSTATE_UINT8(ctest6, NCR710State),
        VMSTATE_UINT8(ctest7, NCR710State),
        VMSTATE_UINT8(ctest8, NCR710State),
        VMSTATE_UINT32(temp, NCR710State),
        VMSTATE_UINT8(dfifo, NCR710State),
        VMSTATE_UINT8(istat, NCR710State),
        VMSTATE_UINT8(lcrc, NCR710State),
        VMSTATE_UINT8(dcmd, NCR710State),
        VMSTATE_UINT8(dmode, NCR710State),
        VMSTATE_UINT8(dien, NCR710State),
        VMSTATE_UINT8(dwt, NCR710State),
        VMSTATE_UINT8(dcntl, NCR710State),
        VMSTATE_UINT32(dsa, NCR710State),
        VMSTATE_UINT32(dbc, NCR710State),
        VMSTATE_UINT32(dnad, NCR710State),
        VMSTATE_UINT32(dsp, NCR710State),
        VMSTATE_UINT32(dsps, NCR710State),
        VMSTATE_UINT32(scratch, NCR710State),
        VMSTATE_UINT32(adder, NCR710State),
        VMSTATE_STRUCT(scsi_fifo, NCR710State, 1,
            vmstate_ncr710_scsi_fifo, NCR710_SCSI_FIFO),
        VMSTATE_UINT8(status, NCR710State),
        VMSTATE_UINT8_ARRAY(msg, NCR710State,
            NCR710_MAX_MSGIN_LEN),
        VMSTATE_UINT8(msg_len, NCR710State),
        VMSTATE_UINT8(msg_action, NCR710State),
        VMSTATE_INT32(carry, NCR710State),
        VMSTATE_BOOL(script_active, NCR710State),
        VMSTATE_INT32(waiting, NCR710State),
        VMSTATE_UINT8(command_complete, NCR710State),
        VMSTATE_UINT32(select_tag, NCR710State),
        VMSTATE_UINT8(current_lun, NCR710State),
        VMSTATE_BOOL(big_endian, NCR710State),
        VMSTATE_INT32(burst_length, NCR710State),
        VMSTATE_BOOL(tolerant_enabled, NCR710State),
        VMSTATE_BOOL(differential_mode, NCR710State),
        VMSTATE_BOOL(cache_line_burst, NCR710State),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_sysbus_ncr710 = {
    .name = "sysbus_ncr710",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(ncr710, SysBusNCR710State, 1, vmstate_ncr710, NCR710State),
        VMSTATE_END_OF_LIST()
    }
};

DeviceState *ncr710_device_create_sysbus(hwaddr addr, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *sysbus;

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

    /* trace_ncr710_device_init(addr); */

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

static void sysbus_ncr710_realize(DeviceState *dev, Error **errp)
{
    SysBusNCR710State *s = SYSBUS_NCR710_SCSI(dev);

    trace_ncr710_device_realize();
    NCR710_DPRINTF("NCR710: Realize function called\n");
    scsi_bus_init(&s->ncr710.bus, sizeof(s->ncr710.bus), dev, &ncr710_scsi_info);
    s->ncr710.as = &address_space_memory;

    ncr710_scsi_fifo_init(&s->ncr710.scsi_fifo);
    s->ncr710.dcntl &= ~NCR710_DCNTL_COM;
    s->ncr710.scid = 0x80 | NCR710_HOST_ID;

    /* Initialize script timer */
    s->ncr710.script_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         ncr710_script_timer_callback,
                                         &s->ncr710);

    s->ncr710.completion_irq_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                                   ncr710_completion_irq_callback,
                                                   &s->ncr710);

    fprintf(stderr, "QEMU: ***** SYSBUS_NCR710_REALIZE: About to initialize reselection timer *****\n");
    fprintf(stderr, "QEMU: ***** SysBusNCR710State *s = %p, &s->ncr710 = %p *****\n", s, &s->ncr710);
    s->ncr710.reselection_retry_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                                     ncr710_reselection_retry_callback,
                                                     &s->ncr710);

    fprintf(stderr, "QEMU: ***** Initialized s->ncr710.reselection_retry_timer = %p *****\n", s->ncr710.reselection_retry_timer);
    fprintf(stderr, "QEMU: ***** Verify: Reading back s->ncr710.reselection_retry_timer = %p *****\n", s->ncr710.reselection_retry_timer);

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
    s->ncr710.scid = 0x80 | NCR710_HOST_ID;
    s->ncr710.dstat = NCR710_DSTAT_DFE;
}

static void sysbus_ncr710_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = sysbus_ncr710_realize;
    device_class_set_legacy_reset(dc, ncr710_device_reset);
    dc->bus_type = NULL;
    set_bit(DEVICE_CATEGORY_STORAGE, dc->categories);
    dc->desc = "NCR53C710 SCSI I/O Processor (SysBus)";
    dc->vmsd = &vmstate_sysbus_ncr710;
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
