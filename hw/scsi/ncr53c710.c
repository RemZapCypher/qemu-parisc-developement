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

#define NCR710_SCNTL0_TRG    0x01
#define NCR710_SCNTL0_AAP    0x02
#define NCR710_SCNTL0_EPG    0x08
#define NCR710_SCNTL0_EPC    0x08
#define NCR710_SCNTL0_WATN   0x10
#define NCR710_SCNTL0_START  0x20

#define NCR710_SCNTL1_RCV    0x01
#define NCR710_SCNTL1_SND   0x02
#define NCR710_SCNTL1_AESP   0x04
#define NCR710_SCNTL1_RST    0x08
#define NCR710_SCNTL1_CON    0x10
#define NCR710_SCNTL1_ESR    0x20
#define NCR710_SCNTL1_ADB    0x40
#define NCR710_SCNTL1_EXC    0x80

#define NCR710_SCNTL2_WSR    0x01
#define NCR710_SCNTL2_VUE0   0x02
#define NCR710_SCNTL2_VUE1   0x04
#define NCR710_SCNTL2_WSS    0x08
#define NCR710_SCNTL2_SLPHBEN 0x10
#define NCR710_SCNTL2_SLPMD  0x20
#define NCR710_SCNTL2_CHM    0x40
#define NCR710_SCNTL2_SDU    0x80

#define NCR710_ISTAT_DIP    0x01
#define NCR710_ISTAT_SIP    0x02
#define NCR710_ISTAT_CON    0x08
#define NCR710_ISTAT_SIGP   0x20
#define NCR710_ISTAT_RST    0x40
#define NCR710_ISTAT_ABRT   0x80

#define NCR710_SSTAT1_WOA    0x04

#define NCR710_SSTAT0_PAR    0x01
#define NCR710_SSTAT0_RST    0x02
#define NCR710_SSTAT0_UDC    0x04
#define NCR710_SSTAT0_SGE    0x08
#define NCR710_SSTAT0_SEL    0x10
#define NCR710_SSTAT0_STO    0x20
#define NCR710_SSTAT0_FCMP   0x40
#define NCR710_SSTAT0_MA     0x80

#define NCR710_SOCL_IO       0x01
#define NCR710_SOCL_CD       0x02
#define NCR710_SOCL_MSG      0x04
#define NCR710_SOCL_ATN      0x08
#define NCR710_SOCL_SEL      0x10
#define NCR710_SOCL_BSY      0x20
#define NCR710_SOCL_ACK      0x40
#define NCR710_SOCL_REQ      0x80

#define NCR710_DSTAT_IID     0x01
#define NCR710_DSTAT_SIR     0x04
#define NCR710_DSTAT_SSI     0x08
#define NCR710_DSTAT_ABRT    0x10
#define NCR710_DSTAT_BF      0x20
#define NCR710_DSTAT_MDPE    0x40
#define NCR710_DSTAT_DFE     0x80

#define NCR710_DCNTL_COM     0x01
#define NCR710_DCNTL_IRQD    0x02
#define NCR710_DCNTL_STD     0x04
#define NCR710_DCNTL_IRQM    0x08
#define NCR710_DCNTL_SSM     0x10
#define NCR710_DCNTL_PFEN    0x20
#define NCR710_DCNTL_PFF     0x40
#define NCR710_DCNTL_EA      0x80
/* DMODE register bits */
#define NCR710_DMODE_MAN     0x01
#define NCR710_DMODE_BOF     0x02
#define NCR710_DMODE_ERMP    0x04
#define NCR710_DMODE_ERL     0x08
#define NCR710_DMODE_DIOM    0x10
#define NCR710_DMODE_SIOM    0x20
#define NCR710_DMODE_BL_MASK 0xc0
#define NCR710_DMODE_BL_1    0x00  /* 1 transfer */
#define NCR710_DMODE_BL_2    0x40  /* 2 transfers */
#define NCR710_DMODE_BL_4    0x80  /* 4 transfers */
#define NCR710_DMODE_BL_8    0xc0  /* 8 transfers */

/* SSTAT1 register bits */
#define NCR710_SSTAT1_SDP    0x01
#define NCR710_SSTAT1_RST    0x02
#define NCR710_SSTAT1_WTN    0x04
#define NCR710_SSTAT1_SDO    0x08
#define NCR710_SSTAT1_ORF    0x10  /* SCSI Output Register Full */
#define NCR710_SSTAT1_OLF    0x10  /* Alias for ORF */
#define NCR710_SSTAT1_ILF    0x20  /* SCSI Input Latch Full */
#define NCR710_SSTAT1_MSG    0x40
#define NCR710_SSTAT1_BSY    0x80

/* SSTAT2 register bits */
#define NCR710_SSTAT2_FF0    0x01
#define NCR710_SSTAT2_FF1    0x02
#define NCR710_SSTAT2_FF2    0x04
#define NCR710_SSTAT2_FF3    0x08
#define NCR710_SSTAT2_SDP    0x10
#define NCR710_SSTAT2_LDSC   0x20
#define NCR710_SSTAT2_STO    0x40
#define NCR710_SSTAT2_STCNT  0x80

/* SBCL register bits */
#define NCR710_SBCL_IO       0x01
#define NCR710_SBCL_CD       0x02
#define NCR710_SBCL_MSG      0x04
#define NCR710_SBCL_ATN      0x08
#define NCR710_SBCL_SEL      0x10
#define NCR710_SBCL_BSY      0x20
#define NCR710_SBCL_ACK      0x40
#define NCR710_SBCL_REQ      0x80
#define NCR710_DCNTL_CLSE    0x80

#define NCR710_DMODE_MAN     0x01
#define NCR710_DMODE_UO      0x02
#define NCR710_DMODE_FAM     0x04
#define NCR710_DMODE_PD      0x08

#define NCR710_CTEST2_DACK   0x01
#define NCR710_CTEST2_DREQ   0x02
#define NCR710_CTEST2_TEOP   0x04
#define NCR710_CTEST2_PCICIE 0x08
#define NCR710_CTEST2_CM     0x10
#define NCR710_CTEST2_CIO    0x20
#define NCR710_CTEST2_SIGP   0x40
#define NCR710_CTEST2_DDIR   0x80

#define NCR710_CTEST5_BL2    0x04
#define NCR710_CTEST5_DDIR   0x08
#define NCR710_CTEST5_MASR   0x10
#define NCR710_CTEST5_DFSN   0x20
#define NCR710_CTEST5_BBCK   0x40
#define NCR710_CTEST5_ADCK   0x80

#define NCR710_CCNTL0_DILS   0x01
#define NCR710_CCNTL0_DISFC  0x10
#define NCR710_CCNTL0_ENNDJ  0x20
#define NCR710_CCNTL0_PMJCTL 0x40
#define NCR710_CCNTL0_ENPMJ  0x80

#define NCR710_CCNTL1_EN64DBMV  0x01
#define NCR710_CCNTL1_EN64TIBMV 0x02
#define NCR710_CCNTL1_64TIMOD   0x04
#define NCR710_CCNTL1_DDAC      0x08
#define NCR710_CCNTL1_ZMOD      0x80

#define NCR710_SBCL_IO  0x01
#define NCR710_SBCL_CD  0x02
#define NCR710_SBCL_MSG 0x04
#define NCR710_SBCL_ATN 0x08
#define NCR710_SBCL_SEL 0x10
#define NCR710_SBCL_BSY 0x20
#define NCR710_SBCL_ACK 0x40
#define NCR710_SBCL_REQ 0x80

#define NCR710_SCID_RRE      0x60
#define NCR710_BUF_SIZE      4096
#define NCR710_HOST_ID    7

/* NCR53C710 has 8-byte SCSI FIFO */
#define SCRIPT_STACK_SIZE 8
#define NCR710_FIFO_DEPTH       16  /* General FIFO depth for other operations */
#define NCR710_FIFO_FULL        0x01
#define NCR710_FIFO_EMPTY       0x02
#define NCR710_MAX_MSGIN_LEN 8

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

static inline int ncr710_irq_on_rsl(NCR710State *s)
{
	return 0;
}

/* Forward declaration */
static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo);
const char *ncr710_reg_name(int offset);
static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0);
static void ncr710_update_irq(NCR710State *s);
static void ncr710_script_dma_interrupt(NCR710State *s, int stat);
static inline void ncr710_dma_read(NCR710State *s, uint32_t addr, void *buf, uint32_t len);
static inline void ncr710_dma_write(NCR710State *s, uint32_t addr, const void *buf, uint32_t len);
void ncr710_request_cancelled(SCSIRequest *req);
void ncr710_command_complete(SCSIRequest *req, size_t resid);
void ncr710_transfer_data(SCSIRequest *req, uint32_t len);



static void ncr710_soft_reset(NCR710State *s)
{
    /* Note: Resetting the device doesnt actuall reset the device */
    DPRINTF("Reset\n");
    s->carry = 0;
    s->msg_action = 0;
    s->msg_len = 0;
    s->waiting = 0;
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
    assert(QTAILQ_EMPTY(&s->queue));
    assert(!s->current);
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

/*
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
 * SCSI FIFO Flow:
 *   OUT: Memory -> SCSI FIFO -> Command/Message Processing
 *   IN:  Status/Message -> SCSI FIFO -> Memory
 *
 * DMA operations are performed directly between memory and device buffers,
 * similar to the reference NCR710 implementation.
 */

/* Starting with SCSI FIFO:
 * - scsi_fifo_init() to initialize the FIFO
 * - scsi_fifo_push() to push data into the FIFO
 * - scsi_fifo_pop() to pop data from the FIFO
 * - scsi_fifo_empty() and scsi_fifo_full() to check FIFO status
 * - scsi_fifo_flush() to clear the FIFO
*/

/* Initialize a SCSI FIFO */
static void ncr710_scsi_fifo_init(NCR710_SCSI_FIFO *fifo)
{
    memset(fifo->data, 0, NCR710_SCSI_FIFO_SIZE);
    memset(fifo->parity, 0, NCR710_SCSI_FIFO_SIZE);
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

/* Push a byte with its parity bit into the SCSI FIFO */
static inline int ncr710_scsi_fifo_push(NCR710_SCSI_FIFO *fifo, uint8_t data, uint8_t parity)
{
    if (ncr710_scsi_fifo_full(fifo)) {
        return -1; /* FIFO full */
    }

    /* Add data at the end (FIFO behavior - Last In goes to the back) */
    fifo->data[fifo->count] = data;
    fifo->parity[fifo->count] = parity;
    fifo->count++;

    return 0;
}

/* Pop a byte and its parity bit from the SCSI FIFO */
static inline uint8_t ncr710_scsi_fifo_pop(NCR710_SCSI_FIFO *fifo, uint8_t *parity)
{
    uint8_t data;

    if (ncr710_scsi_fifo_empty(fifo)) {
        *parity = 0;
        return 0; /* FIFO empty */
    }

    /* Take data from the beginning (FIFO behavior - First In, First Out) */
    data = fifo->data[0];
    *parity = fifo->parity[0];

    /* Shift all remaining data forward */
    fifo->count--;
    for (int i = 0; i < fifo->count; i++) {
        fifo->data[i] = fifo->data[i+1];
        fifo->parity[i] = fifo->parity[i+1];
    }

    return data;
}

/* Fill SCSI FIFO from memory */
static int ncr710_scsi_fifo_fill_from_memory(NCR710State *s, uint32_t addr, int len)
{
    int count = 0;
    uint8_t buf[NCR710_SCSI_FIFO_SIZE];

    /* Read data from memory first */
    int to_read = MIN(len, NCR710_SCSI_FIFO_SIZE);
    ncr710_dma_read(s, addr, buf, to_read);

    /* Then push into FIFO */
    while (count < to_read && !ncr710_scsi_fifo_full(&s->scsi_fifo)) {
        uint8_t data = buf[count];
        uint8_t parity = 0;

        if (s->scntl0 & NCR710_SCNTL0_EPG) {
            parity = ncr710_generate_scsi_parity(s, data);
        }

        if (ncr710_scsi_fifo_push(&s->scsi_fifo, data, parity) < 0) {
            break;
        }

        count++;
    }

    return count;
}

/* Transfer data from SCSI FIFO to buffer */
static int ncr710_scsi_fifo_drain_to_buffer(NCR710State *s, uint8_t *buf, int max_len)
{
    int count = 0;
    uint8_t parity, data;

    DPRINTF("FIFO drain: max_len=%d, fifo_count=%d\n", max_len, s->scsi_fifo.count);
    while (count < max_len && !ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
        data = ncr710_scsi_fifo_pop(&s->scsi_fifo, &parity);
        DPRINTF("FIFO drain byte[%d]: 0x%02x\n", count, data);

        /* Check parity if enabled */
        if (s->scntl0 & NCR710_SCNTL0_EPC) {
            if (!ncr710_check_scsi_parity(s, data, parity)) {
                ncr710_handle_parity_error(s);
            }
        }

        buf[count++] = data;
    }
    DPRINTF("FIFO drain complete: drained %d bytes\n", count);
    if (count > 0) {
        DPRINTF("Buffer after drain: ");
        for (int i = 0; i < count && i < 16; i++) {
            printf("%02x ", buf[i]);
        }
        printf("\n");
    }

    return count;
}

/* Fill SCSI FIFO from buffer */
static int ncr710_scsi_fifo_fill_from_buffer(NCR710State *s, const uint8_t *buf, int len)
{
    int count = 0;

    while (count < len && !ncr710_scsi_fifo_full(&s->scsi_fifo)) {
        uint8_t data = buf[count];
        uint8_t parity = 0;

        if (s->scntl0 & NCR710_SCNTL0_EPG) {
            parity = ncr710_generate_scsi_parity(s, data);
        }

        if (ncr710_scsi_fifo_push(&s->scsi_fifo, data, parity) < 0) {
            break;
        }

        count++;
    }

    return count;
}

/* Transfer data from SCSI FIFO to memory */
static int ncr710_scsi_fifo_drain_to_memory(NCR710State *s, uint32_t addr, int max_len)
{
    int count = 0;
    uint8_t parity, data;
    uint8_t buf[NCR710_SCSI_FIFO_SIZE];

    /* First collect data from FIFO */
    while (count < max_len && count < NCR710_SCSI_FIFO_SIZE &&
           !ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
        data = ncr710_scsi_fifo_pop(&s->scsi_fifo, &parity);

        /* Check parity if enabled */
        if (s->scntl0 & NCR710_SCNTL0_EPC) {
            if (!ncr710_check_scsi_parity(s, data, parity)) {
                ncr710_handle_parity_error(s);
            }
        }

        buf[count++] = data;
    }

    if (count > 0) {
        /* Then write to memory */
        ncr710_dma_write(s, addr, buf, count);
    }

    return count;
}

static uint8_t ncr710_reg_readb(NCR710State *s, int offset);
static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val);
static void ncr710_execute_script(NCR710State *s);
static void ncr710_reselect(NCR710State *s, NCR710Request *p);

static inline uint32_t ncr710_read_dword(NCR710State *s, uint32_t addr)
{
    uint32_t buf;
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                      (uint8_t *)&buf, 4);
    DPRINTF("Read dword %08x from %08x\n", be32_to_cpu(buf), addr);
    /* be32_to_cpu() works in practical but the datasheet,
     * says " NCR710 operates interntally in LE mode"
     * pls take a look into this.
     */
    return be32_to_cpu(buf);
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
    DPRINTF("SCRIPT_STOP: Stopping script execution (prev: active=%d, DSP=0x%08x, dsps=0x%08x)\n",
            s->script_active, s->dsp, s->dsps);
    s->script_active = 0;
    s->scripts.running = false;
    s->istat &= ~NCR710_ISTAT_CON;
    DPRINTF("SCRIPTS execution stopped at PC=0x%08x (istat now=0x%02x)\n", s->scripts.pc, s->istat);
}

static void ncr710_update_irq(NCR710State *s)
{
    DPRINTF("NCR710_UPDATE_IRQ: dstat=0x%02x, sstat0=0x%02x, istat=0x%02x\n",
            s->dstat, s->sstat0, s->istat);
    SysBusNCR710State *ncr710_dev = sysbus_from_ncr710(s);
    int level;
    static int last_level;
    NCR710Request *p;

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
    qemu_set_irq(ncr710_dev->irq, level);

    if (level == 1) {
        printf("*** NCR710 FIRING INTERRUPT *** IRQ Line Called with level=1\n");
        printf("*** DSTAT=0x%02x DIEN=0x%02x ISTAT=0x%02x ***\n", s->dstat, s->dien, s->istat);
        fflush(stdout);
    }

    if (!level && ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR710_SCNTL1_CON)) {
        DPRINTF("Handled IRQs & disconnected, looking for pending "
                "processes\n");
        QTAILQ_FOREACH(p, &s->queue, next) {
            if (p->pending) {
                ncr710_reselect(s, p);
                break;
            }
        }
    }
}

/* Stop SCRIPTS execution and raise a SCSI interrupt.  */
static void ncr710_script_scsi_interrupt(NCR710State *s, int stat0)
{
    uint32_t mask0;

    DPRINTF("SCSI Interrupt 0x%02x prev 0x%02x\n",
            (unsigned)stat0, (unsigned)s->sstat0);
    s->sstat0 |= stat0;
    /* Stop processor on fatal or unmasked interrupt.  As a special hack
       we don't stop processing when raising STO.  Instead continue
       execution and stop at the next insn that accesses the SCSI bus.  */
    mask0 = s->sien0 | ~(NCR710_SSTAT0_FCMP | NCR710_SSTAT0_SEL);
    if (s->sstat0 & mask0) {
        ncr710_stop_script(s);
    }
    ncr710_update_irq(s);
}

/* Stop SCRIPTS execution and raise a DMA interrupt.  */
static void ncr710_script_dma_interrupt(NCR710State *s, int stat)
{
    DPRINTF("DMA Interrupt 0x%x prev 0x%x\n", stat, s->dstat);
    s->dstat |= stat;

    /* For phase mismatch and other recoverable errors, don't stop the script
     * completely - just pause execution so the driver can handle it */
    if (stat == NCR710_DSTAT_SIR) {
        switch (s->dsps) {
            case 0x00000401: /* Good status after status */
                DPRINTF("COMPLETION INTERRUPT - command successful (dsps=0x%08x, active=%d->0)\n",
                        s->dsps, s->script_active);
                s->script_active = 0; /* Pause but don't reset script state? */
                s->istat |= NCR710_ISTAT_DIP;
                s->istat &= ~NCR710_ISTAT_CON;  // Clear "Connected" bit
                ncr710_stop_script(s);
                ncr710_update_irq(s);
                return;

        case 0x00000520: /* Phase mismatch - data phase */
        case 0x00000020: /* Unexpected phase */
                DPRINTF("Phase mismatch interrupt - pausing script for driver handling (dsps=0x%08x, active=%d->0)\n",
                        s->dsps, s->script_active);
                s->script_active = 0; /* Pause but don't reset script state */
                s->istat |= NCR710_ISTAT_DIP;
                ncr710_update_irq(s);
                return;
        default:
            DPRINTF("Other script interrupt: dsps=0x%08x\n", s->dsps);
            break;
        }
    }

    ncr710_update_irq(s);
    ncr710_stop_script(s);
}

static void ncr710_script_interrupt_with_dsps(NCR710State *s, uint32_t dsps_value)
{
    const char *interrupt_name = "UNKNOWN";
    switch (dsps_value) {
        case GOOD_STATUS_AFTER_STATUS: interrupt_name = "GOOD_STATUS_AFTER_STATUS"; break;
        case DISCONNECT_AFTER_CMD: interrupt_name = "DISCONNECT_AFTER_CMD"; break;
        case DISCONNECT_AFTER_DATA: interrupt_name = "DISCONNECT_AFTER_DATA"; break;
        case MSG_IN_BEFORE_CMD: interrupt_name = "MSG_IN_BEFORE_CMD"; break;
        case UNEXPECTED_PHASE_BEFORE_CMD: interrupt_name = "UNEXPECTED_PHASE_BEFORE_CMD"; break;
        case NOT_MSG_OUT_AFTER_SELECTION: interrupt_name = "NOT_MSG_OUT_AFTER_SELECTION"; break;
        case RESELECTION_IDENTIFIED: interrupt_name = "RESELECTION_IDENTIFIED"; break;
        case FATAL: interrupt_name = "FATAL"; break;
        case DEBUG_INTERRUPT: interrupt_name = "DEBUG_INTERRUPT"; break;
        default: interrupt_name = "CUSTOM_OR_UNKNOWN"; break;
    }

    DPRINTF("Script interrupt: DSPS=0x%08x (%s)\n", dsps_value, interrupt_name);

    s->dsps = dsps_value;

    ncr710_script_dma_interrupt(s, NCR710_DSTAT_SIR);
}

static inline void ncr710_set_phase(NCR710State *s, int phase)
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

static void ncr710_bad_phase(NCR710State *s, int out, int new_phase)
{
    /* Trigger a phase mismatch.  */
    DPRINTF("Phase mismatch interrupt\n");
    ncr710_set_phase(s, new_phase);
	s->sbcl |= NCR710_SBCL_REQ;
    ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_MA);
    ncr710_stop_script(s);
}

/* Resume SCRIPTS execution after a DMA operation.  */
static void ncr710_resume_script(NCR710State *s)
{
    s->waiting = 0;
    if (s->script_active) {
        ncr710_execute_script(s);
    }
}

static void ncr710_disconnect(NCR710State *s)
{
    s->scntl1 &= ~NCR710_SCNTL1_CON;
    s->sstat2 &= ~PHASE_MASK;
}

static void ncr710_bad_selection(NCR710State *s, uint32_t id)
{
    DPRINTF("SELECT failed: Target %d not responding\n", id);

    /* Clear selection attempt */
    s->scntl1 &= ~NCR710_SCNTL1_CON;
    s->socl &= ~NCR710_SOCL_ATN;

    /* Generate appropriate interrupt based on context */
    ncr710_script_interrupt_with_dsps(s, NOT_MSG_OUT_AFTER_SELECTION);
    ncr710_disconnect(s);
}

/* Initiate a SCSI layer data transfer using FIFOs.  */
static void ncr710_do_dma(NCR710State *s, int out)
{
    DPRINTF("NCR710_DO_DMA START\n");
    uint32_t count;
    uint32_t addr;
    SCSIDevice *dev;
    int transferred = 0;

    if (!s->current || s->current->dma_len == 0) {
        /* Wait until data is available. */
        DPRINTF("DMA no data available, current=%p\n", s->current);

        if (!s->current) {
            /* No SCSI request active - this might be a script error */
            DPRINTF("DMA called without active SCSI request - stopping script\n");
            s->waiting = 0;
            s->script_active = 0;
            /* Generate script interrupt to notify driver */
            ncr710_script_interrupt_with_dsps(s, A_DISCONNECT_AFTER_CMD);
            return;
        }

        DPRINTF("DMA debug: current=%p, phase=0x%02x, dma_len=%d\n",
                s->current, s->sstat2 & PHASE_MASK,
                s->current ? s->current->dma_len : 0);
        s->waiting = 1;
        return;
    }
    // assert(s->current->dma_len);

    dev = s->current->req->dev;
    assert(dev);

    count = s->dbc;
    if (count > s->current->dma_len)
        count = s->current->dma_len;

    addr = s->dnad;

    DPRINTF("DMA addr=%d len=%d out=%d\n", addr, count, out);

    if (s->current->dma_buf == NULL) {
        s->current->dma_buf = scsi_req_get_buf(s->current->req);
    }

    if (out) {
        /* Memory to device (Write operation) - Direct transfer */
        ncr710_dma_read(s, addr, s->current->dma_buf, count);
        transferred = count;

        DPRINTF("DMA Write: direct transfer=%d bytes\n", transferred);

    } else {
        /* Device to memory (Read operation) - Direct transfer */
        ncr710_dma_write(s, addr, s->current->dma_buf, count);
        transferred = count;

        DPRINTF("DMA Read: direct transfer=%d bytes\n", transferred);
    }

    /* Update DMA pointers and counters */
    s->dnad += transferred;
    s->dbc -= transferred;
    s->current->dma_len -= transferred;

    if (s->current->dma_len == 0) {
        s->current->dma_buf = NULL;
        s->waiting = 0;  /* Clear waiting state */
        s->script_active = 1; /* Resume script execution */
        scsi_req_continue(s->current->req);
        /* Schedule script continuation */
        ncr710_resume_script(s);
    } else {
        s->current->dma_buf += transferred;
        /* Continue DMA transfer */
        ncr710_resume_script(s);
    }

    /* Update DSTAT register - DMA FIFO is always empty after direct transfer */
    s->dstat |= NCR710_DSTAT_DFE;  /* DMA FIFO Empty */
    DPRINTF("NCR710_DO_DMA END\n");
}


/* Add a command to the queue.  */
static void ncr710_queue_command(NCR710State *s)
{
    DPRINTF("NCR710_QUEUE_COMMAND START\n");
    NCR710Request *p = s->current;

    DPRINTF("Queueing tag=0x%x\n", p->tag);
    assert(s->current != NULL);
    assert(s->current->dma_len == 0);
    QTAILQ_INSERT_TAIL(&s->queue, s->current, next);
    s->current = NULL;

    p->pending = 0;
    p->out = (s->sstat2 & PHASE_MASK) == PHASE_DO;
    DPRINTF("NCR710_QUEUE_COMMAND END\n");
}

/* Queue a byte for a MSG IN phase.  */
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

/* Perform reselection to continue a command.  */
static void ncr710_reselect(NCR710State *s, NCR710Request *p)
{
    int id;

    DPRINTF("NCR710_RESELECT START\n");
    assert(s->current == NULL);
    QTAILQ_REMOVE(&s->queue, p, next);
    s->current = p;

    id = (p->tag >> 8) & 0xf;
    /* LSI53C700 Family Compatibility, see NCR53C710 4-73 */
    if (!(s->dcntl & NCR710_DCNTL_COM)) {
        s->sfbr = 1 << (id & 0x7);
    }
	s->lcrc = 0;
    DPRINTF("Reselected target %d\n", id);
    s->scntl1 |= NCR710_SCNTL1_CON;
    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = p->out ? 2 : 3;
    s->current->dma_len = p->pending;
    ncr710_add_msg_byte(s, 0x80);
    if (s->current->tag & NCR710_TAG_VALID) {
        ncr710_add_msg_byte(s, 0x20);
        ncr710_add_msg_byte(s, p->tag & 0xff);
    }

    if (ncr710_irq_on_rsl(s)) {
        ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_SEL);
    }
    DPRINTF("NCR710_RESELECT END\n");
}

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

void ncr710_request_cancelled(SCSIRequest *req)
{
    DPRINTF("NCR710_REQUEST_CANCELLED START\n");
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    NCR710Request *p = (NCR710Request*)req->hba_private;

    req->hba_private = NULL;
    ncr710_request_free(s, p);
    DPRINTF("NCR710_REQUEST_CANCELLED END\n");
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
    /* Reselect if waiting for it, or if reselection triggers an IRQ
       and the bus is free.
       Since no interrupt stacking is implemented in the emulation, it
       is also required that there are no pending interrupts waiting
       for service from the device driver. */
    if (s->waiting == 1 ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR710_SCNTL1_CON) &&
         !(s->istat & (NCR710_ISTAT_SIP | NCR710_ISTAT_DIP)))) {
        /* Reselect device.  */
        ncr710_reselect(s, p);
        return 0;
    } else {
        DPRINTF("Queueing IO tag=0x%x\n", p->tag);
        p->pending = len;
        return 1;
    }
    DPRINTF("NCR710_QUEUE_REQ END \n");
}

 /* Callback to indicate that the SCSI layer has completed a command.  */
void ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);

    DPRINTF("NCR710_COMMAND_COMPLETE START\n");

    /* Decode SCSI status for better debugging */
    const char *status_name = "UNKNOWN";
    switch (req->status) {
        case 0x00: status_name = "GOOD"; break;
        case 0x02: status_name = "CHECK_CONDITION"; break;
        case 0x04: status_name = "CONDITION_MET"; break;
        case 0x08: status_name = "BUSY"; break;
        case 0x10: status_name = "INTERMEDIATE"; break;
        case 0x14: status_name = "INTERMEDIATE_CONDITION_MET"; break;
        case 0x18: status_name = "RESERVATION_CONFLICT"; break;
        case 0x22: status_name = "COMMAND_TERMINATED"; break;
        case 0x28: status_name = "TASK_SET_FULL"; break;
        default: status_name = "VENDOR_SPECIFIC"; break;
    }

    DPRINTF("Command complete status=%d (%s)\n", (int)req->status, status_name);

    int out = (s->sstat2 & PHASE_MASK) == PHASE_DO;
    if (req->status == 0x02) {  /* CHECK CONDITION */
        DPRINTF("TODO:IMPLEMENT");
    }
	s->lcrc = 0;
    s->status = req->status;
    s->command_complete = 2;

    if (s->waiting && s->dbc != 0) {
        /* Raise phase mismatch for short transfers.  */
        ncr710_bad_phase(s, out, PHASE_ST);
    } else {
        ncr710_set_phase(s, PHASE_ST);
    }

    DPRINTF("Command completed with status=%d - letting SCRIPTS continue\n", req->status);

    if (req->hba_private == s->current) {
        req->hba_private = NULL;
        ncr710_request_free(s, s->current);
		scsi_req_unref(req);
    }
    DPRINTF("NCR710_COMMAND_COMPLETE END\n");
}

 /* Callback to indicate that the SCSI layer has completed a transfer.  */
void ncr710_transfer_data(SCSIRequest *req, uint32_t len)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    int out;

    DPRINTF("NCR710_TRANSFER_DATA START\n");
    assert(req->hba_private);
    if (req->hba_private != s->current ||
        (ncr710_irq_on_rsl(s) && !(s->scntl1 & NCR710_SCNTL1_CON))) {
        if (ncr710_queue_req(s, req, len)) {
            return;
        }
    }

    out = (s->sstat2 & PHASE_MASK) == PHASE_DO;

    /* host adapter (re)connected */
    DPRINTF("Data ready tag=0x%x len=%d\n", req->tag, len);
    s->current->dma_len = len;
    s->command_complete = 1;
    if (!s->current) {
        DPRINTF("Transfer data called with no current request\n");
        return;
    }
    if (s->waiting) {
        if (s->waiting == 1 || s->waiting == 3 || s->dbc == 0) {
            ncr710_resume_script(s);
        } else {
            ncr710_do_dma(s, out);
        }
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
    DPRINTF("NCR710_DO_COMMAND START\n");
    DPRINTF("DBC=%d, DNAD=%08x\n", s->dbc, s->dnad);
    if (s->dbc > 16)
        s->dbc = 16;
    DPRINTF("SCSI FIFO before fill: count=%d\n", s->scsi_fifo.count);
    ncr710_scsi_fifo_fill_from_memory(s, s->dnad, s->dbc);
    DPRINTF("SCSI FIFO after fill: count=%d\n", s->scsi_fifo.count);
    bytes_read = ncr710_scsi_fifo_drain_to_buffer(s, buf, s->dbc);
    DPRINTF("Drained %d bytes from SCSI FIFO\n", bytes_read);
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
    dev = scsi_device_find(&s->bus, 0, idbitstonum(id), s->current_lun);
    if (!dev) {
        ncr710_bad_selection(s, id);
        return;
    }

    assert(s->current == NULL);
    s->current = g_new0(NCR710Request, 1);
    s->current->tag = s->select_tag;
    s->current->req = scsi_req_new(dev, s->current->tag, s->current_lun,
                                  buf, bytes_read, s->current);
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
            /* Command did not complete immediately so disconnect.  */
            ncr710_add_msg_byte(s, 2); /* SAVE DATA POINTER */
            ncr710_add_msg_byte(s, 4); /* DISCONNECT */
            /* wait data */
            ncr710_set_phase(s, PHASE_MI);
            s->msg_action = 1;
            ncr710_queue_command(s);
        } else {
            /* wait command complete - use status phase */
            ncr710_set_phase(s, PHASE_SI);
            DPRINTF("No immediate data transfer, set phase to SI (status)\n");
        }
    }
    DPRINTF("NCR710_DO_COMMAND END\n");
}

static void ncr710_do_status(NCR710State *s)
{
    uint8_t status = s->status;
    printf("NCR710_DO_STATUS: status=0x%02x\n", status);

    if (s->dbc != 1)
        BADF("Bad Status move\n");
    s->dbc = 1;
    s->sfbr = status;
    ncr710_scsi_fifo_fill_from_buffer(s, &status, 1);
    ncr710_scsi_fifo_drain_to_memory(s, s->dnad, 1);

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
    if (len <= 0) {
        return;
    }
    s->sfbr = s->msg[0];

    /* Parse incoming message for common cases */
    switch (s->msg[0]) {
    case 0x00: /* Command Complete */
        DPRINTF("MSG IN: Command Complete\n");
        s->command_complete = 1;
        break;
    case 0x04: /* Disconnect */
        DPRINTF("MSG IN: Disconnect\n");
        s->msg_action = 1; /* Will disconnect after transfer */
        break;
    case 0x07: /* Message Reject */
        DPRINTF("MSG IN: Message Reject\n");
        /* Target rejected our message - continue anyway */
        break;
    default:
        DPRINTF("MSG IN: 0x%02x\n", s->msg[0]);
        break;
    }

    ncr710_scsi_fifo_fill_from_buffer(s, s->msg, len);
    ncr710_scsi_fifo_drain_to_memory(s, s->dnad, len);
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
    NCR710Request *current_req, *p, *p_next;

    if (s->current) {
        current_tag = s->current->tag;
        current_req = s->current;
    } else {
        current_tag = s->select_tag;
        current_req = ncr710_find_by_tag(s, current_tag);
    }

    DPRINTF("MSG out len=%d\n", s->dbc);

    /* Process message bytes in FIFO-sized chunks */
    while (s->dbc > 0) {
        /* Fill SCSI FIFO from memory (host->target) */
        int to_move = MIN((int)s->dbc, NCR710_SCSI_FIFO_SIZE);
        int filled  = ncr710_scsi_fifo_fill_from_memory(s, s->dnad, to_move);

        if (filled <= 0) {
            break;
        }

        /* Drain into a local buffer for parsing */
        uint8_t buf[NCR710_SCSI_FIFO_SIZE];
        int bytes = ncr710_scsi_fifo_drain_to_buffer(s, buf, filled);

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
                QTAILQ_FOREACH_SAFE(p, &s->queue, next, p_next) {
                    if ((p->tag & 0x0000ff00) == (current_tag & 0x0000ff00)) {
                        scsi_req_cancel(p->req);
                    }
                }
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
    NCR710Request *p;

    DPRINTF("Wait Reselect\n");

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

/* Check if we should continue script execution or wait for external events */
static int ncr710_should_continue_script(NCR710State *s)
{
    if (!s->script_active) {
        return 0;
    }

    if (s->waiting == 1) {
        DPRINTF("Script waiting for transfer_data callback\n");
        return 0;
    }
    if (s->waiting == 2 || s->waiting == 3) {
        DPRINTF("Script waiting for SCSI phase, continuing\n");
        s->waiting = 0;  /* Allow script to continue */
        return 1;
    }
    return (s->waiting == 0);
}

/* Timer callback to continue script execution */
static void ncr710_script_timer_callback(void *opaque)
{
    NCR710State *s = opaque;

    DPRINTF("Script timer callback, waiting=%d, script_active=%d\n",
            s->waiting, s->script_active);

    if (s->script_active && ncr710_should_continue_script(s)) {
        ncr710_execute_script(s);
    }
}

/* Enhanced script execution with proper autonomous flow */
static void ncr710_execute_script(NCR710State *s)
{
    uint32_t insn;
    uint32_t addr;
    int opcode;
    int insn_processed = 0;
    // const int MAX_INSTRUCTIONS = 1000; /* Prevent infinite loops */

    DPRINTF("SCRIPT_EXEC START: DSP=0x%08x, active=%d, waiting=%d, istat=0x%02x\n",
            s->dsp, s->script_active, s->waiting, s->istat);

    s->script_active = 1;
    s->scripts.running = true;

again:
    insn_processed++;
    s->scripts.pc = s->dsp;  /* Update scripts program counter */

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

        /* Check if we have a connection before doing DMA */
        if (!(s->scntl1 & NCR710_SCNTL1_CON)) {
            DPRINTF("Block move without SCSI connection - need SELECT first\n");
            ncr710_script_interrupt_with_dsps(s, A_DISCONNECT_AFTER_CMD);
            break;
        }

        s->dnad = addr;
        switch (s->sstat2 & 0x7) {
        case PHASE_DO:
            s->waiting = 2;
            ncr710_do_dma(s, 1);
            /* Check if we can continue immediately */
            if (s->waiting == 0) {
                ncr710_resume_script(s);
                return;
            }
            break;
        case PHASE_DI:
            s->waiting = 2;
            ncr710_do_dma(s, 0);
            /* Check if we can continue immediately */
            if (s->waiting == 0) {
                ncr710_resume_script(s);
                return;
            }
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
                    DPRINTF("Already reselected, jumping to alternative address\n");
                    s->dsp = s->dnad;
                    break;
                }
                s->sstat1 |= NCR710_SSTAT1_WOA;
                if (!scsi_device_find(&s->bus, 0, idbitstonum(id), 0)) {
                    DPRINTF("SELECT: Target %d not found\n", id);
                    ncr710_bad_selection(s, id);
                    break;
                }
                DPRINTF("SELECT: Target %d%s\n", id, insn & (1 << 24) ? " with ATN" : "");

                /* ??? Linux drivers compain when this is set.  Maybe
                   it only applies in low-level mode (unimplemented).
                ncr710_script_scsi_interrupt(s, NCR710_SIST0_CMP, 0); */
                s->select_tag = id << 8;
                s->scntl1 |= NCR710_SCNTL1_CON;

                if (insn & (1 << 24)) {
                    /* SELECT with ATN - go to MESSAGE OUT phase */
                    s->socl |= NCR710_SOCL_ATN;
                    ncr710_set_phase(s, PHASE_MO);
                    DPRINTF("SELECT: Set phase to MSG_OUT for target %d\n", id);
                } else {
                    /* SELECT without ATN - go directly to COMMAND phase */
                    ncr710_set_phase(s, PHASE_CO);
                    DPRINTF("SELECT: Set phase to COMMAND for target %d\n", id);
                }
                break;
            case 1: /* Disconnect */
                DPRINTF("Wait Disconnect\n");
                s->scntl1 &= ~NCR710_SCNTL1_CON;
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
                    BADF("Target mode not implemented\n");
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
                        ncr710_script_dma_interrupt(s, NCR710_DSTAT_SIR);
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
				ncr710_dma_read(s, addr, data, n);
                DPRINTF("Load reg 0x%x size %d addr 0x%08x = %08x\n", reg, n,
                        addr, *(int *)data);
                for (i = 0; i < n; i++) {
                    ncr710_reg_writeb(s, reg + i, data[i]);
                }
            } else {
                DPRINTF("Store reg 0x%x size %d addr 0x%08x\n", reg, n, addr);
                for (i = 0; i < n; i++) {
                    data[i] = ncr710_reg_readb(s, reg + i);
                }
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
        DPRINTF("SCRIPTS in waiting state %d, will be resumed later\n", s->waiting);
        return;
    }

    DPRINTF("SCRIPT_EXEC END: stopped (active=%d, waiting=%d, DSP=0x%08x, istat=0x%02x)\n",
            s->script_active, s->waiting, s->dsp, s->istat);
}

static uint8_t ncr710_reg_readb(NCR710State *s, int offset)
{
    uint8_t ret = 0;

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
            ret |= NCR710_DSTAT_DFE;

            bool had_script_interrupt = (s->dstat & NCR710_DSTAT_SIR) != 0;
            bool was_completion = (s->dsps == 0x00000401);  // Good status completion

            s->dstat = 0;  /* Clear DSTAT on read */
            s->istat &= ~NCR710_ISTAT_DIP;
            ncr710_update_irq(s);

            if (was_completion) {
                DPRINTF("*** COMPLETION INTERRUPT HANDLED - SCRIPT STAYS PAUSED ***\n");
                DPRINTF("*** DRIVER SHOULD PROCESS COMPLETION AND START NEXT COMMAND ***\n");
            }

            /* DON'T automatically resume scripts after completion interrupts */
            if (had_script_interrupt && s->script_active == 0 && !was_completion) {
                DPRINTF("DSTAT read cleared script interrupt, resuming execution\n");
                s->script_active = 1;
                ncr710_execute_script(s);
            }
            break;
        case NCR710_SSTAT0_REG: /* SSTAT0 */
            ret = s->sstat0;
            if (s->sstat0 != 0) {
                s->sstat0 = 0;
                s->istat &= ~NCR710_ISTAT_SIP;
                ncr710_update_irq(s);
            }
            break;
        case NCR710_SSTAT1_REG: /* SSTAT1 */
            ret = s->sstat1;
            if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
                ret |= NCR710_SSTAT1_ILF;
            } else {
                ret &= ~NCR710_SSTAT1_ILF;
            }
            /* DMA FIFO is always empty with direct transfers */
            ret &= ~NCR710_SSTAT1_ORF;
            if (s->scntl0 & NCR710_SCNTL0_EPG) {
                if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
                    uint8_t parity = s->scsi_fifo.parity[0];
                    if (parity) {
                        ret |= NCR710_SSTAT1_SDP;
                    } else {
                        ret &= ~NCR710_SSTAT1_SDP;
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
                    ret |= NCR710_SSTAT2_SDP;
                } else {
                    ret &= ~NCR710_SSTAT2_SDP;
                }
            } else {
                ret &= ~NCR710_SSTAT2_SDP;
            }
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
            DPRINTF("ISTAT read: 0x%02x (CON=%d, DIP=%d, SIP=%d)\n",
                    ret, !!(ret & NCR710_ISTAT_CON), !!(ret & NCR710_ISTAT_DIP), !!(ret & NCR710_ISTAT_SIP));
            break;
        case NCR710_CTEST8_REG: /* CTEST8 */
            ret = s->ctest8;
            ret = (ret & 0x4F) | (NCR710_REVISION_2 << 4);
            break;
        case NCR710_LCRC_REG: /* LCRC */
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
        s->sstat1 |= NCR710_SSTAT1_OLF; /* From second implementation */
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
        DPRINTF("ISTAT WRITE: val=0x%02x (prev=0x%02x, ABRT=%d, SIGP=%d, script_active=%d, waiting=%d)\n",
                val, s->istat, !!(val & NCR710_ISTAT_ABRT), !!(val & NCR710_ISTAT_SIGP),
                s->script_active, s->waiting);
        if (val & NCR710_ISTAT_ABRT) {
            NCR710_DPRINTF("NCR710: ISTAT ABRT bit set - aborting SCRIPTS\n");
            s->scripts.running = false;
            s->dstat |= NCR710_DSTAT_ABRT;
            s->istat |= NCR710_ISTAT_DIP;
            ncr710_script_dma_interrupt(s, NCR710_DSTAT_ABRT);
            ncr710_update_irq(s);
        }
        if (s->waiting == 1 && (val & NCR710_ISTAT_SIGP)) {
            NCR710_DPRINTF("NCR710: SIGP received - waking up script execution\n");
            s->waiting = 0;
            s->dsp = s->dnad;
            NCR710_DPRINTF("NCR710: Starting script execution from DSP=0x%x\n", s->dsp);
            ncr710_execute_script(s);
        }
        if (val & NCR710_ISTAT_RST) {
            NCR710_DPRINTF("NCR710: ISTAT RST bit set - soft reset\n");
            /* Enhanced reset from second implementation */
            uint8_t saved_ctest8 = s->ctest8;
            ncr710_soft_reset(s);
            s->ctest8 = saved_ctest8;  /* Preserve chip revision */
            s->dstat = NCR710_DSTAT_DFE;
            ncr710_scsi_fifo_init(&s->scsi_fifo);
        }
        s->istat = (s->istat & 0x0f) | (val & 0xf0);
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
    case 0x2c: /* DSP[24:31] */
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
        DPRINTF("DSP WRITE FINAL: prev_state(active=%d, waiting=%d, istat=0x%02x)\n",
                s->script_active, s->waiting, s->istat);
        s->waiting = 0;
        s->scripts.running = true;
        s->scripts.pc = s->dsp;
        s->script_active = 1;
        s->istat |= NCR710_ISTAT_CON;
        NCR710_DPRINTF("NCR710: Starting script execution (DMODE=0x%02x)\n", s->dmode);
        DPRINTF("DSP WRITE: About to execute script at DSP=0x%08x (CON bit set)\n", s->dsp);
        ncr710_execute_script(s);
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
            NCR710_DPRINTF("NCR710_DCNTL_STD triggered - manually starting SCRIPTS at DSP=0x%08x\n", s->dsp);
            s->waiting = 0;
            s->scripts.running = true;
            s->scripts.pc = s->dsp;
            ncr710_execute_script(s);
            s->dcntl &= ~NCR710_DCNTL_STD;
        }
        break;

    CASE_SET_REG32(adder, NCR710_ADDER_REG)
        NCR710_DPRINTF("ADDER: 0x%08x\n", s->adder);
        break;

    default:
        qemu_log_mask(LOG_UNIMP, "NCR710: write unknown register %02X\n", offset);
        break;
    }

#undef CASE_SET_REG24
#undef CASE_SET_REG32
}

/* Memory region operations for NCR710 registers */
static uint64_t ncr710_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    NCR710State *s = opaque;
    uint8_t offset = addr & 0xff;
    uint8_t val = ncr710_reg_readb(s, offset);
    trace_ncr710_reg_read(ncr710_reg_name(offset), offset, val);
    return val;
}

static void ncr710_reg_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
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
    .max_lun = 0,  /* LUN support buggy, eh? */

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
        VMSTATE_BOOL(scripts.running, NCR710State),
        VMSTATE_UINT32(scripts.pc, NCR710State),

        /* Note: Queue and current request are not migrated (complex pointers/lists).
           If needed, we will add custom pre/post load/save handlers later. */
        VMSTATE_END_OF_LIST()
    }
};

/* VMState for SysBusNCR710State */
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

    /* trace_ncr710_create_sysbus(addr); */
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
    QTAILQ_INIT(&s->ncr710.queue);
    scsi_bus_init(&s->ncr710.bus, sizeof(s->ncr710.bus), dev, &ncr710_scsi_info);
    s->ncr710.as = &address_space_memory;

    ncr710_scsi_fifo_init(&s->ncr710.scsi_fifo);
    s->ncr710.dcntl &= ~NCR710_DCNTL_COM;
    s->ncr710.scid = 0x80 | NCR710_HOST_ID;
    s->ncr710.big_endian = false;

    /* Initialize script timer */
    s->ncr710.script_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                         ncr710_script_timer_callback,
                                         &s->ncr710);

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
