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

/* Enable Response to Reselection */
#define NCR710_SCID_RRE      0x60
#define NCR710_BUF_SIZE      4096
#define SCRIPT_STACK_SIZE 8
#define NCR710_HOST_ID    7

/* Maximum length of MSG IN data.  */
#define NCR710_MAX_MSGIN_LEN 8

/* Flag set if this is a tagged command.  */
#define NCR710_TAG_VALID     (1 << 16)

static inline int ncr710_irq_on_rsl(NCR710State *s)
{
	return 0;
}

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
    s->dien = 0;
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

static uint8_t ncr710_reg_readb(NCR710State *s, int offset);
static void ncr710_reg_writeb(NCR710State *s, int offset, uint8_t val);
static void ncr710_execute_script(NCR710State *s);
static void ncr710_reselect(NCR710State *s, NCR710Request *p);

static inline uint32_t ncr710_read_dword(NCR710State *s, uint32_t addr)
{
    uint32_t buf;

    /* For now, use simple memory access - this may need DMA mapping later */
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                      (uint8_t *)&buf, 4);
    return le32_to_cpu(buf);
}

static inline void ncr710_dma_read(NCR710State *s, uint32_t addr, void *buf, uint32_t len)
{
    address_space_read(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                      buf, len);
}

static inline void ncr710_dma_write(NCR710State *s, uint32_t addr, const void *buf, uint32_t len)
{
    address_space_write(&address_space_memory, addr, MEMTXATTRS_UNSPECIFIED,
                       buf, len);
}

static void ncr710_stop_script(NCR710State *s)
{
    s->script_active = 0;
}

static void ncr710_update_irq(NCR710State *s)
{
    SysBusNCR710State *ncr710_dev = sysbus_from_ncr710(s);
    int level;
    static int last_level;
    NCR710Request *p;

    /* It's unclear whether the DIP/SIP bits should be cleared when the
       Interrupt Status Registers are cleared or when istat0 is read.
       We currently do the formwer, which seems to work.  */
    level = 0;
    if (s->dstat) {
        if (s->dstat & s->dien)
            level = 1;
        s->istat |= NCR710_ISTAT_DIP;
    } else {
        s->istat &= ~NCR710_ISTAT_DIP;
    }

    if (s->sstat0) {
        if ((s->sstat0 & s->sien0))
            level = 1;
        s->istat |= NCR710_ISTAT_SIP;
    } else {
        s->istat &= ~NCR710_ISTAT_SIP;
    }

    if (level != last_level) {
        DPRINTF("Update IRQ level %d dstat %02x sist %02x%02x\n",
                level, s->dstat, s->sstat0, s->sstat1);
        last_level = level;
    }
    qemu_set_irq(ncr710_dev->irq, level);

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
    ncr710_update_irq(s);
    ncr710_stop_script(s);
}

static inline void ncr710_set_phase(NCR710State *s, int phase)
{
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
    ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_MA);
    ncr710_stop_script(s);
    ncr710_set_phase(s, new_phase);
	s->sbcl |= NCR710_SBCL_REQ;
}


/* Resume SCRIPTS execution after a DMA operation.  */
static void ncr710_resume_script(NCR710State *s)
{
    if (s->waiting != 2) {
        s->waiting = 0;
        ncr710_execute_script(s);
    } else {
        s->waiting = 0;
    }
}

static void ncr710_disconnect(NCR710State *s)
{
    s->scntl1 &= ~NCR710_SCNTL1_CON;
    s->sstat2 &= ~PHASE_MASK;
}

static void ncr710_bad_selection(NCR710State *s, uint32_t id)
{
    DPRINTF("Selected absent target %d\n", id);
    ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_STO);
    ncr710_disconnect(s);
}

/* Initiate a SCSI layer data transfer.  */
static void ncr710_do_dma(NCR710State *s, int out)
{
    uint32_t count;
    dma_addr_t addr;
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

    DPRINTF("DMA addr=0x" DMA_ADDR_FMT " len=%d\n", addr, count);
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
		scsi_req_continue(s->current->req);
    } else {
        s->current->dma_buf += count;
        ncr710_resume_script(s);
    }
}


/* Add a command to the queue.  */
static void ncr710_queue_command(NCR710State *s)
{
    NCR710Request *p = s->current;

    DPRINTF("Queueing tag=0x%x\n", p->tag);
    assert(s->current != NULL);
    assert(s->current->dma_len == 0);
    QTAILQ_INSERT_TAIL(&s->queue, s->current, next);
    s->current = NULL;

    p->pending = 0;
    p->out = (s->sstat2 & PHASE_MASK) == PHASE_DO;
}

/* Queue a byte for a MSG IN phase.  */
static void ncr710_add_msg_byte(NCR710State *s, uint8_t data)
{
    if (s->msg_len >= NCR710_MAX_MSGIN_LEN) {
        BADF("MSG IN data too long\n");
    } else {
        DPRINTF("MSG IN 0x%02x\n", data);
        s->msg[s->msg_len++] = data;
    }
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
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    NCR710Request *p = (NCR710Request*)req->hba_private;

    req->hba_private = NULL;
    ncr710_request_free(s, p);
	scsi_req_unref(req);
}

/* Record that data is available for a queued command.  Returns zero if
   the device was reselected, nonzero if the IO is deferred.  */
static int ncr710_queue_req(NCR710State *s, SCSIRequest *req, uint32_t len)
{
    NCR710Request *p = (NCR710Request*)req->hba_private;

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
}

 /* Callback to indicate that the SCSI layer has completed a command.  */
void ncr710_command_complete(SCSIRequest *req, size_t resid)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    int out;

    out = (s->sstat2 & PHASE_MASK) == PHASE_DO;
    DPRINTF("Command complete status=%d\n", (int)req->status);
	s->lcrc = 0;
    s->status = req->status;
    s->command_complete = 2;
    if (s->waiting && s->dbc != 0) {
        /* Raise phase mismatch for short transfers.  */
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

 /* Callback to indicate that the SCSI layer has completed a transfer.  */
void ncr710_transfer_data(SCSIRequest *req, uint32_t len)
{
    NCR710State *s = ncr710_from_scsi_bus(req->bus);
    int out;

    assert(req->hba_private);
    if (s->waiting == 1 || req->hba_private != s->current ||
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
    if (s->waiting) {
        if (s->waiting == 1 || s->dbc == 0) {
            ncr710_resume_script(s);
        } else {
            ncr710_do_dma(s, out);
        }
    }
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

    DPRINTF("Send command len=%d\n", s->dbc);
    if (s->dbc > 16)
        s->dbc = 16;
	ncr710_dma_read(s, s->dnad, buf, s->dbc);
    DPRINTF("Send command len=%d %02x.%02x.%02x.%02x.%02x.%02x\n", s->dbc, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
    s->sfbr = buf[0];
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
	s->current->req = scsi_req_new(dev, s->current->tag, s->current_lun, buf, s->dbc, s->current);

	n = scsi_req_enqueue(s->current->req);
    if (n) {
        if (n > 0) {
            ncr710_set_phase(s, PHASE_DI);
        } else if (n < 0) {
            ncr710_set_phase(s, PHASE_DO);
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
            /* wait command complete */
            ncr710_set_phase(s, PHASE_DI);
        }
    }
}

static void ncr710_do_status(NCR710State *s)
{
    uint8_t status;
    DPRINTF("Get status len=%d status=%d\n", s->dbc, s->status);
    if (s->dbc != 1)
        BADF("Bad Status move\n");
    s->dbc = 1;
    status = s->status;
    s->sfbr = status;
	ncr710_dma_write(s, s->dnad, &status, 1);
    ncr710_set_phase(s, PHASE_MI);
    s->msg_action = 1;
    ncr710_add_msg_byte(s, 0); /* COMMAND COMPLETE */
}

static void ncr710_do_msgin(NCR710State *s)
{
    int len;
    DPRINTF("Message in len=%d/%d\n", s->dbc, s->msg_len);
    s->sfbr = s->msg[0];
    len = s->msg_len;
    if (len > s->dbc)
        len = s->dbc;
	ncr710_dma_write(s, s->dnad, s->msg, len);
    /* Linux drivers rely on the last byte being in the SIDL.  */
    s->sidl = s->msg[len - 1];
    s->msg_len -= len;
    if (s->msg_len) {
        memmove(s->msg, s->msg + len, s->msg_len);
    } else {
        /* ??? Check if ATN (not yet implemented) is asserted and maybe
           switch to PHASE_MO.  */
        switch (s->msg_action) {
        case 0:
            ncr710_set_phase(s, PHASE_CMD);
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
}

/* Read the next byte during a MSGOUT phase.  */
static uint8_t ncr710_get_msgbyte(NCR710State *s)
{
    uint8_t data;
	ncr710_dma_read(s, s->dnad, &data, 1);
    s->dnad++;
    s->dbc--;
    return data;
}

/* Skip the next n bytes during a MSGOUT phase. */
static void ncr710_skip_msgbytes(NCR710State *s, unsigned int n)
{
    s->dnad += n;
    s->dbc  -= n;
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

    DPRINTF("MSG out len=%d\n", s->dbc);
    while (s->dbc) {
        msg = ncr710_get_msgbyte(s);
        s->sfbr = msg;

        switch (msg) {
        case 0x04:
            DPRINTF("MSG: Disconnect\n");
            ncr710_disconnect(s);
            break;
        case 0x08:
            DPRINTF("MSG: No Operation\n");
            ncr710_set_phase(s, PHASE_CMD);
            break;
        case 0x01:
            len = ncr710_get_msgbyte(s);
            msg = ncr710_get_msgbyte(s);
            (void)len; /* avoid a warning about unused variable*/
            DPRINTF("Extended message 0x%x (len %d)\n", msg, len);
            switch (msg) {
            case 1:
                DPRINTF("SDTR (ignored)\n");
                ncr710_skip_msgbytes(s, 2);
                break;
            case 3:
                DPRINTF("WDTR (ignored)\n");
                ncr710_skip_msgbytes(s, 1);
                break;
            default:
                goto bad;
            }
            break;
        case 0x20: /* SIMPLE queue */
            s->select_tag |= ncr710_get_msgbyte(s) | NCR710_TAG_VALID;
            DPRINTF("SIMPLE queue tag=0x%x\n", s->select_tag & 0xff);
            break;
        case 0x21: /* HEAD of queue */
            BADF("HEAD queue not implemented\n");
            s->select_tag |= ncr710_get_msgbyte(s) | NCR710_TAG_VALID;
            break;
        case 0x22: /* ORDERED queue */
            BADF("ORDERED queue not implemented\n");
            s->select_tag |= ncr710_get_msgbyte(s) | NCR710_TAG_VALID;
            break;
        case 0x0d:
            /* The ABORT TAG message clears the current I/O process only. */
            DPRINTF("MSG: ABORT TAG tag=0x%x\n", current_tag);
            if (current_req) {
				scsi_req_cancel(current_req->req);
            }
            ncr710_disconnect(s);
            break;
        case 0x06:
        case 0x0e:
        case 0x0c:
            /* The ABORT message clears all I/O processes for the selecting
               initiator on the specified logical unit of the target. */
            if (msg == 0x06) {
                DPRINTF("MSG: ABORT tag=0x%x\n", current_tag);
            }
            /* The CLEAR QUEUE message clears all I/O processes for all
               initiators on the specified logical unit of the target. */
            if (msg == 0x0e) {
                DPRINTF("MSG: CLEAR QUEUE tag=0x%x\n", current_tag);
            }
            /* The BUS DEVICE RESET message clears all I/O processes for all
               initiators on all logical units of the target. */
            if (msg == 0x0c) {
                DPRINTF("MSG: BUS DEVICE RESET tag=0x%x\n", current_tag);
            }

            /* clear the current I/O process */
            if (s->current) {
				scsi_req_cancel(s->current->req);
            }

            /* As the current implemented devices scsi_disk and scsi_generic
               only support one LUN, we don't need to keep track of LUNs.
               Clearing I/O processes for other initiators could be possible
               for scsi_generic by sending a SG_SCSI_RESET to the /dev/sgX
               device, but this is currently not implemented (and seems not
               to be really necessary). So let's simply clear all queued
               commands for the current device: */
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
            DPRINTF("Select LUN %d\n", s->current_lun);
            ncr710_set_phase(s, PHASE_CMD);
            break;
        }
    }
    return;
bad:
    BADF("Unimplemented message 0x%02x\n", msg);
    ncr710_set_phase(s, PHASE_MI);
    ncr710_add_msg_byte(s, 7); /* MESSAGE REJECT */
    s->msg_action = 0;
}

static void ncr710_memcpy(NCR710State *s, uint32_t dest, uint32_t src, int count)
{
    int n;
    uint8_t buf[NCR710_BUF_SIZE];

    DPRINTF("memcpy dest 0x%08x src 0x%08x count %d\n", dest, src, count);
    while (count) {
        n = (count > NCR710_BUF_SIZE) ? NCR710_BUF_SIZE : count;
		ncr710_dma_read(s, src, buf, n);
		ncr710_dma_write(s, dest, buf, n);
        src += n;
        dest += n;
        count -= n;
    }
}

/* NCR53C710 FIFO Helper Functions */

/* Initialize a DMA FIFO */
static void ncr710_dma_fifo_init(NCR710_DMA_FIFO *fifo)
{
    memset(fifo->data, 0, NCR710_DMA_FIFO_SIZE);
    memset(fifo->parity, 0, NCR710_DMA_FIFO_SIZE);
    fifo->head = 0;
    fifo->tail = 0;
    fifo->count = 0;
}

/* Check if DMA FIFO is empty */
static inline bool ncr710_dma_fifo_empty(NCR710_DMA_FIFO *fifo)
{
    return fifo->count == 0;
}

/* Check if DMA FIFO is full */
static inline bool ncr710_dma_fifo_full(NCR710_DMA_FIFO *fifo)
{
    return fifo->count == NCR710_DMA_FIFO_SIZE;
}

/* Push a byte with its parity bit into the DMA FIFO */
static inline int ncr710_dma_fifo_push(NCR710_DMA_FIFO *fifo, uint8_t data, uint8_t parity)
{
    if (ncr710_dma_fifo_full(fifo)) {
        return -1; /* FIFO full */
    }
    
    fifo->data[fifo->head] = data;
    fifo->parity[fifo->head] = parity;
    fifo->head = (fifo->head + 1) % NCR710_DMA_FIFO_SIZE;
    fifo->count++;
    return 0;
}

/* Pop a byte and its parity bit from the DMA FIFO */
static inline uint8_t ncr710_dma_fifo_pop(NCR710_DMA_FIFO *fifo, uint8_t *parity)
{
    uint8_t data;
    
    if (ncr710_dma_fifo_empty(fifo)) {
        *parity = 0;
        return 0; /* FIFO empty */
    }
    
    data = fifo->data[fifo->tail];
    *parity = fifo->parity[fifo->tail];
    fifo->tail = (fifo->tail + 1) % NCR710_DMA_FIFO_SIZE;
    fifo->count--;
    
    return data;
}

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
    
    /* Shift FIFO to make room at position 0 */
    for (int i = fifo->count; i > 0; i--) {
        fifo->data[i] = fifo->data[i-1];
        fifo->parity[i] = fifo->parity[i-1];
    }
    
    fifo->data[0] = data;
    fifo->parity[0] = parity;
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
    
    /* Take data from the first position */
    data = fifo->data[0];
    *parity = fifo->parity[0];
    
    /* Shift FIFO down */
    fifo->count--;
    for (int i = 0; i < fifo->count; i++) {
        fifo->data[i] = fifo->data[i+1];
        fifo->parity[i] = fifo->parity[i+1];
    }
    
    return data;
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

static void ncr710_execute_script(NCR710State *s)
{
    uint32_t insn;
    uint32_t addr;
    int opcode;
    int insn_processed = 0;

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
        if ((s->sstat2 & PHASE_MASK) != ((insn >> 24) & 7)) {
            DPRINTF("Wrong phase got %d expected %d\n",
                    s->sstat2 & PHASE_MASK, (insn >> 24) & 7);
            ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_MA);
			s->sbcl |= NCR710_SBCL_REQ;
            break;
        }
        s->dnad = addr;
        switch (s->sstat2 & 0x7) {
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
                    ncr710_bad_selection(s, id);
                    break;
                }
                DPRINTF("Selected target %d%s\n",
                        id, insn & (1 << 24) ? " ATN" : "");
                /* ??? Linux drivers compain when this is set.  Maybe
                   it only applies in low-level mode (unimplemented).
                ncr710_script_scsi_interrupt(s, NCR710_SIST0_CMP, 0); */
                s->select_tag = id << 8;
                s->scntl1 |= NCR710_SCNTL1_CON;
                if (insn & (1 << 24)) {
                    s->socl |= NCR710_SOCL_ATN;
					ncr710_set_phase(s, PHASE_MO);
				} else {
					ncr710_set_phase(s, PHASE_CMD);
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
#ifdef DEBUG_LSI
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

            if ((insn & 0x002e0000) == 0) {
                DPRINTF("NOP\n");
                break;
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
                    s->temp = s->dsp;
                    s->dsp = addr;
                    break;
                case 2: /* Return */
                    DPRINTF("Return to 0x%08x\n", s->temp);
                    s->dsp = s->temp;
                    break;
                case 3: /* Interrupt */
                    DPRINTF("Interrupt 0x%08x\n", s->dsps);
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
    if (insn_processed > 10000 && !s->waiting) {
        /* Some windows drivers make the device spin waiting for a memory
           location to change.  If we have been executed a lot of code then
           assume this is the case and force an unexpected device disconnect.
           This is apparently sufficient to beat the drivers into submission.
         */
        if (!(s->sien0 & NCR710_SSTAT0_UDC))
            fprintf(stderr, "inf. loop with UDC masked\n");
        ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_UDC);
        ncr710_disconnect(s);
    } else if (s->script_active && !s->waiting) {
        if (s->dcntl & NCR710_DCNTL_SSM) {
            ncr710_script_dma_interrupt(s, NCR710_DSTAT_SSI);
        } else {
            goto again;
        }
    }
    DPRINTF("SCRIPTS execution stopped\n");
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
            ret = s->sien0;
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
            if (s->scntl1 & NCR710_SCNTL1_CON) {
                ret = s->sstat2 & PHASE_MASK;
                ret |= s->sbcl;
                if (s->socl & NCR710_SOCL_ATN)
                    ret |= NCR710_SBCL_ATN;
            }
            break;
        case NCR710_DSTAT_REG: /* DSTAT */
            ret = s->dstat;

            /* Update DMA FIFO Empty flag */
            if (ncr710_dma_fifo_empty(&s->dma_fifo)) {
                ret |= NCR710_DSTAT_DFE;
            } else {
                ret &= ~NCR710_DSTAT_DFE;
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
                s->istat &= ~NCR710_ISTAT_SIP;
                ncr710_update_irq(s);
            }
            break;
        case NCR710_SSTAT1_REG: /* SSTAT1 */
            ret = s->sstat1;

            /* Update Input Latch Full (ILF) based on SCSI FIFO */
            if (!ncr710_scsi_fifo_empty(&s->scsi_fifo)) {
                ret |= NCR710_SSTAT1_ILF;
            } else {
                ret &= ~NCR710_SSTAT1_ILF;
            }

            /* Update Output Register Full (ORF) based on DMA FIFO */
            if (!ncr710_dma_fifo_empty(&s->dma_fifo)) {
                ret |= NCR710_SSTAT1_ORF;
            } else {
                ret &= ~NCR710_SSTAT1_ORF;
            }

            /* Update SCSI Parity bit (SDP) with live parity signal */
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
                // uint8_t expected_parity = ncr710_generate_scsi_parity(s, s->sidl);
                uint8_t expected_parity = 0; // Parity checking disabled for now
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
                ret &= ~NCR710_ISTAT_SIGP;  /* SIGP doesn't exist in 700 */
                ret &= ~NCR710_ISTAT_RST;
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
                ret &= ~(NCR710_DCNTL_EA | NCR710_DCNTL_COM);  /* These don't exist in 700 */
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
                 (val & NCR710_SCNTL0_EPC) ? "chk" : "", (val & NCR710_SCNTL0_EPG) ? "gen" : "");
        NCR710_DPRINTF("NCR710: SCNTL0: 0x%02x->0x%02x parity=%s%s\n", old_val, val,
                (val & NCR710_SCNTL0_EPC) ? "chk" : "", (val & NCR710_SCNTL0_EPG) ? "gen" : "");

        /* Handle parity control bits according to NCR710 manual */
        if ((val & NCR710_SCNTL0_EPC) != (old_val & NCR710_SCNTL0_EPC)) {
            /* Enable Parity Checking bit changed */
            /* trace_ncr710_parity_checking_changed((val & NCR710_SCNTL0_EPC) != 0); */
        }

        if ((val & NCR710_SCNTL0_EPG) != (old_val & NCR710_SCNTL0_EPG)) {
            /* Enable Parity Generation bit changed */
            /* trace_ncr710_parity_generation_changed((val & NCR710_SCNTL0_EPG) != 0); */
        }

        if ((val & NCR710_SCNTL0_AAP) != (old_val & NCR710_SCNTL0_AAP)) {
            /* Assert ATN/ on Parity Error bit changed */
            /* trace_ncr710_atn_on_parity_changed((val & NCR710_SCNTL0_AAP) != 0); */
        }

        if (val & NCR710_SCNTL0_START) {
            NCR710_DPRINTF("NCR710: SCNTL0: START bit set, mode=%s\n", is_700_mode ? "700" : "710");
            NCR710_DPRINTF("NCR710: SCNTL0: START bit set, mode=%s\n", is_700_mode ? "700" : "710");
            if (is_700_mode) {
                qemu_log_mask(LOG_UNIMP, "NCR710: Start sequence not implemented\n");
            }
        }
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

        if (val & NCR710_SCNTL1_ADB) {
            qemu_log_mask(LOG_UNIMP, "NCR710: Immediate Arbitration not implemented\n");
        }

        if (val & NCR710_SCNTL1_RST) {
            if (!(s->sstat0 & NCR710_SSTAT0_RST)) {
                s->sstat0 |= NCR710_SSTAT0_RST;
                ncr710_script_scsi_interrupt(s, NCR710_SSTAT0_RST);
            }
            /* Enhanced reset handling for second implementation */
            if (!(old_val & NCR710_SCNTL1_RST)) {
                NCR710_DPRINTF("NCR710: SCNTL1: SCSI bus reset initiated\n");
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
            if (s->scntl0 & NCR710_SCNTL0_EPG) {
                // parity = ncr710_generate_scsi_parity(s, val);
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
            if (val & NCR710_ISTAT_ABRT) {
                NCR710_DPRINTF("NCR710: ISTAT ABRT bit set - aborting SCRIPTS\n");
                s->scripts.running = false;
                s->dstat |= NCR710_DSTAT_ABRT;
                s->istat |= NCR710_ISTAT_DIP;
                timer_del(s->selection_timer);
                timer_del(s->watchdog_timer);
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
                // uint32_t flushed = ncr710_dma_fifo_drain_to_memory(s, s->dnad, s->dma_fifo.count);
                // s->dnad += flushed;
                s->dnad += 0;
                // NCR710_DPRINTF("CTEST8: Flushed %d bytes from DMA FIFO to 0x%08x\n", flushed, s->dnad - flushed);
            }
            s->dstat |= NCR710_DSTAT_DFE;  /* Set DMA FIFO Empty */
        }
        if (val & 0x04) {
            NCR710_DPRINTF("CTEST8: Clearing all FIFOs\n");
            ncr710_dma_fifo_init(&s->dma_fifo);
            ncr710_scsi_fifo_init(&s->scsi_fifo);
            s->dstat |= NCR710_DSTAT_DFE;  /* Set DMA FIFO Empty */
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
            ncr710_execute_script(s);
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
            switch (val & NCR710_DMODE_BL_MASK) {
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
            s->dcntl = val & ~(NCR710_DCNTL_PFF | NCR710_DCNTL_STD | NCR710_DCNTL_EA);
            if (val & 0x01) {
                ncr710_soft_reset(s);
            }
        } else {
            s->dcntl = val & ~(NCR710_DCNTL_PFF);
            if (val & NCR710_DCNTL_STD) {
                NCR710_DPRINTF("NCR710_DCNTL_STD triggered - manually starting SCRIPTS at DSP=0x%08x\n", s->dsp);
                /* trace_ncr710_NCR_DCNTL_STD_triggered(s->dsp); */
                s->waiting = 0;
                s->scripts.running = true;
                s->scripts.pc = s->dsp;
                ncr710_execute_script(s);
                s->dcntl &= ~NCR710_DCNTL_STD;
            }
        }
        ncr710_update_compatibility_mode(s, (val & NCR710_DCNTL_COM) != 0);
        NCR710_DPRINTF("DCNTL: 0x%02x mode=%s manual=%s\n", val, is_700_mode ? "700" : "710",
                 (s->dmode & NCR710_DMODE_MAN) ? "yes" : "no");
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

/* SCSI Host Bus Adapter interface */
static const struct SCSIBusInfo ncr710_scsi_info = {
    .tcq = false,
    .max_target = 7,
    .max_lun = 1,

    .transfer_data = ncr710_transfer_data,
    .complete = ncr710_command_complete,
    .cancel = ncr710_request_cancelled,
};

/* Memory region operations for NCR710 registers */
uint64_t ncr710_mmio_read(void *opaque, hwaddr addr, unsigned size)
{
    SysBusNCR710State *dev = opaque;
    NCR710State *s = &dev->ncr710;
    uint8_t val;

    addr &= 0xff;
    val = ncr710_reg_readb(s, addr);

    return val;
}

void ncr710_mmio_write(void *opaque, hwaddr addr, uint32_t val, unsigned size)
{
    SysBusNCR710State *dev = opaque;
    NCR710State *s = &dev->ncr710;

    addr &= 0xff;
    ncr710_reg_writeb(s, addr, val);
}

/* Device reset */
static void ncr710_device_reset(DeviceState *dev)
{
    SysBusNCR710State *sysbus_dev = SYSBUS_NCR710_SCSI(dev);
    NCR710State *s = &sysbus_dev->ncr710;

    ncr710_soft_reset(s);
}

/* Device realization */
static void ncr710_realize(DeviceState *dev, Error **errp)
{
    SysBusNCR710State *sysbus_dev = SYSBUS_NCR710_SCSI(dev);
    NCR710State *s = &sysbus_dev->ncr710;
    scsi_bus_init(&s->bus, sizeof(s->bus), dev, &ncr710_scsi_info);
    QTAILQ_INIT(&s->queue);
    memory_region_init_io(&sysbus_dev->mmio, OBJECT(dev), &ncr710_mmio_ops,
                         sysbus_dev, "ncr710-mmio", NCR710_REG_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &sysbus_dev->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &sysbus_dev->irq);
    s->mode_700 = false;  /* Default to 710 mode */
    ncr710_soft_reset(s);
}

static const VMStateDescription vmstate_ncr710 = {
    .name = "ncr710",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_END_OF_LIST()
    }
};

/* Helper functions */
bool ncr710_is_700_mode(NCR710State *s)
{
    return s->mode_700;
}

void ncr710_update_compatibility_mode(NCR710State *s, bool mode_700)
{
    s->mode_700 = mode_700;
}

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

static const struct SCSIBusInfo ncr710_scsi_info = {
    .tcq = true,
    .max_target = 8,
    .max_lun = 0,  /* LUN support is buggy */

    .transfer_data = ncr710_transfer_data,
    .complete = ncr710_command_complete,
    .cancel = ncr710_request_cancelled,
};

static const MemoryRegionOps ncr710_mmio_ops = {
    .read = ncr710_mmio_read,
    .write = ncr710_mmio_write,
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