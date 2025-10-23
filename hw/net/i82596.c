/*
 * QEMU Intel i82596 (Apricot) emulation
 *
 * Copyright (c) 2019 Helge Deller <deller@gmx.de>
 *
 * Additional functionality added by:
 * Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 * During GSOC 2025.
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 * This software was written to be compatible with the specification:
 * https://parisc.docs.kernel.org/en/latest/_downloads/96672be0650d9fc046bbcea40b92482f/82596CA.pdf
 *
 * INDEX:
 * 1.  Reset
 * 2.  Address Translation
 * 3.  Transmit functions
 * 4.  Receive Helper functions
 * 5.  Receive functions
 * 6.  Misc Functionality Functions
 * 6.1 Individual Address
 * 6.2 Multicast Address List
 * 6.3 Link Status
 * 6.4 CSMA/CD functions
 * 6.5 Unified CRC Calculation
 * 6.6 Unified Statistics Update
 * 7.  Bus Throttling Timer
 * 8.  Dump functions
 * 9.  Configure
 * 10. Command Loop
 * 11. Examine SCB
 * 12. Channel attention (CA)
 * 13. LASI interface
 * 14. Polling functions
 * 15. QOM and interface functions
 *
 * TODO:
 * [ ] Implement proper Rx and Tx functions. Use the kernel side debug to get as much info as possible, then proceed.
 * [ ] Add support for the dump functions.
 * [ ] Adding monitor mode config
 * [ ] Unified Error Recording
 * [ ] Unified CRC Functionality
 * [ ] Unified Error Recording
 *
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "net/net.h"
#include "net/eth.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "exec/address-spaces.h"
#include "qemu/module.h"
#include "trace.h"
#include "i82596.h"
#include <zlib.h> /* for crc32 */

#define ENABLE_DEBUG    1
#if defined(ENABLE_DEBUG)
#define DBG(x)          x
#else
#define DBG(x)          do { } while (0)
#endif
#define USE_TIMER       1

#define MAX_MC_CNT      64
#define I596_NULL       ((uint32_t)0xffffffff)
#define BITS(n, m)      (((0xffffffffU << (31 - n)) >> (31 - n + m)) << m)

/* ISCP "busy" flag (first byte) */
#define ISCP_BUSY              0x01

#define NANOSECONDS_PER_MICROSECOND 1000

#define SCB_STATUS_CX   0x8000  /* CU finished command with I bit */
#define SCB_STATUS_FR   0x4000  /* RU finished receiving a frame */
#define SCB_STATUS_CNA  0x2000  /* CU left active state */
#define SCB_STATUS_RNR  0x1000  /* RU left active state */
#define SCB_ACK_MASK    0xF000  /* All interrupt acknowledge bits */

/* 82596 Operational Modes */
#define I82586_MODE                 0x00
#define I82596_MODE_SEGMENTED       0x01
#define I82596_MODE_LINEAR          0x02

/* Monitor Options */
#define MONITOR_NORMAL      0x00 /* Monitor non-filtered frames */
#define MONITOR_FILTERED    0x01 /* Monitor only filtered frames */
#define MONITOR_ALL         0x02 /* Monitor all frames */
#define MONITOR_DISABLED    0x03 /* Default: Monitor mode disabled */

/* Operation mode flags from SYSBUS byte */
#define SYSBUS_LOCK_EN         0x08
#define SYSBUS_INT_ACTIVE_LOW  0x10
#define SYSBUS_BIG_ENDIAN_32   0x80  /* Enhanced Big Endian (C-step) */
#define SYSBUS_THROTTLE_MASK   0x60  /* Bus Throttle Timer trigger modes */

/* SCB commands - Command Unit (CU) */
#define SCB_CUC_NOP            0x00
#define SCB_CUC_START          0x01
#define SCB_CUC_RESUME         0x02
#define SCB_CUC_SUSPEND        0x03
#define SCB_CUC_ABORT          0x04
#define SCB_CUC_LOAD_THROTTLE  0x05
#define SCB_CUC_LOAD_START     0x06

/* SCB commands - Receive Unit (RU) */
#define SCB_RUC_NOP            0x00
#define SCB_RUC_START          0x01
#define SCB_RUC_RESUME         0x02
#define SCB_RUC_SUSPEND        0x03
#define SCB_RUC_ABORT          0x04

/* SCB statuses - Command Unit (CU) */
#define CU_IDLE         0
#define CU_SUSPENDED    1
#define CU_ACTIVE       2

/* SCB statuses - Receive Unit (RU) */
#define RX_IDLE         0x00
#define RX_SUSPENDED    0x01
#define RX_NO_RESOURCES 0x02
#define RX_READY        0x04
#define RX_NO_RESO_RBD  0x0A
#define RX_NO_MORE_RBD  0x0C

#define CMD_FLEX        0x0008  /* Enable flexible memory model */
#define CMD_MASK        0x0007  /* Mask for command bits */

#define CMD_EOL         0x8000  /* The last command of the list, stop, wrong use */
#define CMD_SUSP        0x4000  /* Suspend after doing cmd. */
#define CMD_INTR        0x2000  /* Interrupt after doing cmd. */

enum commands {
        CmdNOp = 0, CmdSASetup = 1, CmdConfigure = 2, CmdMulticastList = 3,
        CmdTx = 4, CmdTDR = 5, CmdDump = 6, CmdDiagnose = 7
};

#define DUMP_BUF_SZ     304     /* Linear and 32bit segmented mode */

#define STAT_C          0x8000  /* Set to 0 after execution */
#define STAT_B          0x4000  /* Command being executed */
#define STAT_OK         0x2000  /* Command executed ok */
#define STAT_A          0x1000  /* Command aborted */

#define I596_EOF        0x8000
#define SIZE_MASK       0x3fff

#define CSMA_SLOT_TIME         51     /* Slot time in microseconds (for 10Mbps) */
#define CSMA_MAX_RETRIES       16     /* Maximum number of retransmission attempts */
#define CSMA_BACKOFF_LIMIT     10     /* Maximum backoff factor (2^10 = 1024 slots) */

/* Global Flags fetched from config bytes */
#define I596_PREFETCH       (s->config[0] & 0x80)           /* Enable prefetch of data structures */
#define SAVE_BAD_FRAMES     (s->config[2] & 0x80)           /* Store frames with errors in memory */
#define I596_NO_SRC_ADD_IN  (s->config[3] & 0x08)           /* If 1, do not insert MAC in Tx Packet */
#define I596_LOOPBACK       (s->config[3] >> 6)             /* Determine the Loopback mode */
#define I596_PROMISC        (s->config[8] & 0x01)           /* Receive all packets regardless of MAC */
#define I596_BC_DISABLE     (s->config[8] & 0x02)           /* Disable reception of broadcast packets */
#define I596_NOCRC_INS      (s->config[8] & 0x08)           /* Do not append CRC to Tx frame */
#define I596_CRC16_32       (s->config[8] & 0x10)           /* Use CRC-32 if set, otherwise CRC-16 */
#define I596_PADDING        (s->config[8] & 0x80)           /* Add padding to short frames */
#define I596_MIN_FRAME_LEN  (s->config[10])                 /* Minimum Ethernet frame length */
#define I596_CRCINM         (s->config[11] & 0x04)          /* Rx CRC appended in memory */
#define I596_MONITOR_MODE   ((s->config[11] >> 6) & 0x03)   /* Determine monitor mode */
#define I596_MC_ALL         (s->config[11] & 0x20)          /* Receive all multicast packets */
#define I596_FULL_DUPLEX    (s->config[12] & 0x40)          /* Full-duplex mode if set, half if clear */
#define I596_MULTIIA        (s->config[13] & 0x40)          /* Enable multiple individual addresses */

/* RX ERRORS */
#define RX_COLLISIONS         0x0001  /* Collision detected during frame reception */
#define RX_LENGTH_ERRORS      0x0080  /* Frame length error during reception */
#define RX_OVER_ERRORS        0x0100  /* Receiver overflow error */
#define RX_FIFO_ERRORS        0x0400  /* FIFO buffer overflow during reception */
#define RX_FRAME_ERRORS       0x0800  /* Frame alignment error */
#define RX_CRC_ERRORS         0x1000  /* CRC checksum error in received frame */
#define RX_LENGTH_ERRORS_ALT  0x2000  /* Duplicate definition - frame length error flag */
#define RFD_STATUS_TRUNC      0x0020  /* Received frame truncated due to buffer size */
#define RFD_STATUS_NOBUFS     0x0200  /* No buffer resources available for frame reception */

/* TX ERRORS */
#define TX_COLLISIONS       0x0020  /* TX collision detection flag */
#define TX_HEARTBEAT_ERRORS 0x0040  /* Heartbeat signal lost during transmission */
#define TX_CARRIER_ERRORS   0x0400  /* Carrier sense signal lost during transmission */
#define TX_COLLISIONS_ALT   0x0800  /* Frame experienced collisions during transmission */
#define TX_ABORTED_ERRORS   0x1000  /* Transmission aborted due to excessive collisions */

static void i82596_bus_throttle_timer(void *opaque);
static void i82596_flush_queue_timer(void *opaque);
static void i82596_update_scb_irq(I82596State *s, bool trigger);
static void i82596_update_cu_status(I82596State *s, uint16_t cmd_status, bool generate_interrupt);
static void update_scb_status(I82596State *s);
static void examine_scb(I82596State *s);
static bool i82596_check_medium_status(I82596State *s);
static int i82596_csma_backoff(I82596State *s, int retry_count);
static uint16_t i82596_calculate_crc16(const uint8_t *data, size_t len);
static size_t i82596_append_crc(I82596State *s, uint8_t *buffer, size_t len);
static bool i82596_verify_crc(I82596State *s, const uint8_t *data, size_t len);
static void __attribute__((unused)) i82596_update_rx_statistics(I82596State *s, bool pkt_completed, bool crc_ok, size_t frame_size);
static void i82596_update_statistics(I82596State *s, bool is_tx, uint16_t error_flags,
                                     uint16_t collision_count);

static uint8_t get_byte(uint32_t addr)
{
    return ldub_phys(&address_space_memory, addr);
}

static void set_byte(uint32_t addr, uint8_t c)
{
    return stb_phys(&address_space_memory, addr, c);
}

static uint16_t get_uint16(uint32_t addr)
{
    return lduw_be_phys(&address_space_memory, addr);
}

static void set_uint16(uint32_t addr, uint16_t w)
{
    return stw_be_phys(&address_space_memory, addr, w);
}

static uint32_t get_uint32(uint32_t addr)
{
    uint32_t lo = lduw_be_phys(&address_space_memory, addr);
    uint32_t hi = lduw_be_phys(&address_space_memory, addr + 2);
    return (hi << 16) | lo;
}

static void set_uint32(uint32_t addr, uint32_t val)
{
    set_uint16(addr, (uint16_t) val);
    set_uint16(addr + 2, val >> 16);
}

/* Centralized error detection and update mechanism */
static void i82596_record_error(I82596State *s, uint16_t error_type, bool is_tx)
{
    if (is_tx) {
        /* Handle TX-specific errors */
        if (error_type & TX_ABORTED_ERRORS) {
            s->tx_aborted_errors++;
            set_uint32(s->scb + 28, s->tx_aborted_errors);
            DBG(printf("TX Error: Excessive collisions (aborted), count=%d\n",
                       s->tx_aborted_errors));
        }

        if (error_type & TX_CARRIER_ERRORS) {
            DBG(printf("TX Error: Carrier sense lost\n"));
        }

        if (error_type & TX_HEARTBEAT_ERRORS) {
            DBG(printf("TX Error: Heartbeat check failure\n"));
        }
    } else {
        if (error_type & RX_CRC_ERRORS) {
            s->crc_err++;
            set_uint32(s->scb + 16, s->crc_err);
            DBG(printf("RX Error: CRC error, count=%d\n", s->crc_err));
        }

        if (error_type & (RX_LENGTH_ERRORS | RX_LENGTH_ERRORS_ALT | RX_FRAME_ERRORS)) {
            s->align_err++;
            set_uint32(s->scb + 18, s->align_err);
            DBG(printf("RX Error: Alignment/Length error, count=%d\n", s->align_err));
        }

        if (error_type & RFD_STATUS_NOBUFS) {
            s->resource_err++;
            set_uint32(s->scb + 20, s->resource_err);
            DBG(printf("RX Error: No buffer resources, count=%d\n", s->resource_err));
        }

        if (error_type & (RX_OVER_ERRORS | RX_FIFO_ERRORS)) {
            s->over_err++;
            set_uint32(s->scb + 22, s->over_err);
            DBG(printf("RX Error: Overrun/FIFO error, count=%d\n", s->over_err));
        }

        if (error_type & RFD_STATUS_TRUNC) {
            s->short_fr_error++;
            set_uint32(s->scb + 26, s->short_fr_error);
            DBG(printf("RX Error: Frame truncated, count=%d\n", s->short_fr_error));
        }
    }
}

/*Design the Rx functions
 * For the Rx functions
 * struct i82596_rx_frame_descriptor
 * struct i82596_rx_buffer_descriptor
 * rx_buffer[PKT_BUF_SZ]
 *
 *
 *
 */

struct i82596_tx_descriptor {
    uint16_t status;          /* +0h: Status word (C, B, OK, A) + collision count */
    uint16_t command;         /* +2h: Command word (EL, S, I, NC, SF, CMD) */
    uint32_t link;            /* +4h: Link to next command block */
    uint32_t tbd_addr;        /* +8h: TBD address (0xFFFFFFFF = no TBDs) */
    uint16_t tcb_count;       /* +Ch: Byte count in TCB (bits 13:0) + EOF (bit 15) */
    uint16_t dest_addr01;     /* +Eh: Destination address bytes 0-1 */
    uint16_t dest_addr23;     /* +10h: Destination address bytes 2-3 */
    uint16_t dest_addr45;     /* +12h: Destination address bytes 4-5 */
};

struct i82596_tx_buffer_desc {
    uint16_t size;            /* +0h: Buffer size (bits 13:0) + EOF (bit 15) */
    uint16_t reserved;        /* +2h: Reserved/padding */
    uint32_t link;            /* +4h: Link to next TBD */
    uint32_t buffer;          /* +8h: Buffer address (32-bit in linear mode) */
};

struct i82596_rx_descriptor {
    uint16_t status;          /* +0h: Status word (C, B, OK, error bits) */
    uint16_t command;         /* +2h: Command word (EL, S, SF) */
    uint32_t link;            /* +4h: Link to next RFD */
    uint32_t rbd_addr;        /* +8h: RBD address (0xFFFFFFFF = Simplified mode) */
    uint16_t actual_count;    /* +Ch: Actual count (bits 13:0) + EOF + F bits */
    uint16_t size;            /* +Eh: RFD data area size (bits 13:0) */
    uint16_t dest_addr01;     /* +10h: Destination address bytes 0-1 */
    uint16_t dest_addr23;     /* +12h: Destination address bytes 2-3 */
    uint16_t dest_addr45;     /* +14h: Destination address bytes 4-5 */
    uint16_t src_addr01;      /* +16h: Source address bytes 0-1 */
    uint16_t src_addr23;      /* +18h: Source address bytes 2-3 */
    uint16_t src_addr45;      /* +1Ah: Source address bytes 4-5 */
    uint16_t length_field;    /* +1Ch: Length field from frame */
};

struct i82596_rx_buffer_desc {
    uint16_t count;           /* +0h: Actual count (bits 13:0) + EOF + F bits */
    uint16_t reserved1;       /* +2h: Reserved */
    uint32_t link;            /* +4h: Link to next RBD */
    uint32_t buffer;          /* +8h: Buffer address (32-bit) */
    uint16_t size;            /* +Ch: Buffer size (bits 13:0) + EL + P bits */
    uint16_t reserved2;       /* +Eh: Reserved */
};


/* Design the Tx functions
 * from what we know, the initial tfd + 0 passes the values to the kernel
 * so we need a function that determines the error
 * and passes the required values to the kernel side
 *
 * For the Tx functions
 * struct i82596_tx_frame_descriptor
 * struct i82596_tx_buffer_descriptor
 * tx_buffer[PKT_BUF_SZ]
 *
 * i82596_tx_max_collisions(s, tx_status): We can pass it the tx frame and it will update the statistics
 * and pass it back
 *
 * i82596_tx_status_tracker(s, tx_status): This will take the status bits of the tx frame
 * and then pass the required values to it and then update the kernel side.
 *
 * i82596_desc_read(I82596State *s, hwaddr p, struct i82596_descriptor *desc): This will load the frame from the memory
 * Making it easier for us to parse the values.
 * i82596_desc_write(I82596State *s, hwaddr p, struct i82596_descriptor *desc): This will write the frame back to the memory
 *
 * We also need similar debug functions to tulip for the Tx and the Rx functions
 * i82596_dump_tx_descriptor: This will dump the contents of the Tx descriptor
 * i82596_dump_rx_descriptor: This will dump the contents of the Rx descriptor
 */

/* Forward declarations: Helps reorganise the code for better readability */


static void i82596_tx_desc_read(I82596State *s, hwaddr addr,
                                struct i82596_tx_descriptor *desc)
{
    desc->status = get_uint16(addr + 0);
    desc->command = get_uint16(addr + 2);
    desc->link = get_uint32(addr + 4);
    desc->tbd_addr = get_uint32(addr + 8);
    desc->tcb_count = get_uint16(addr + 12);
    desc->dest_addr01 = get_uint16(addr + 14);
    desc->dest_addr23 = get_uint16(addr + 16);
    desc->dest_addr45 = get_uint16(addr + 18);
}

static void i82596_tx_desc_write(I82596State *s, hwaddr addr,
                                 struct i82596_tx_descriptor *desc)
{
    set_uint16(addr + 0, desc->status);
    set_uint16(addr + 2, desc->command);
    set_uint32(addr + 4, desc->link);
    set_uint32(addr + 8, desc->tbd_addr);
    set_uint16(addr + 12, desc->tcb_count);
    set_uint16(addr + 14, desc->dest_addr01);
    set_uint16(addr + 16, desc->dest_addr23);
    set_uint16(addr + 18, desc->dest_addr45);
}

static void i82596_tbd_read(I82596State *s, hwaddr addr,
                            struct i82596_tx_buffer_desc *tbd)
{
    tbd->size = get_uint16(addr + 0);
    tbd->reserved = get_uint16(addr + 2);
    tbd->link = get_uint32(addr + 4);
    tbd->buffer = get_uint32(addr + 8);
}

static void i82596_rx_desc_read(I82596State *s, hwaddr addr,
                                struct i82596_rx_descriptor *desc)
{
    desc->status = get_uint16(addr + 0x0);
    desc->command = get_uint16(addr + 0x2);
    desc->link = get_uint32(addr + 0x4);
    desc->rbd_addr = get_uint32(addr + 0x8);
    desc->actual_count = get_uint16(addr + 0xC);
    desc->size = get_uint16(addr + 0xE);
    desc->dest_addr01 = get_uint16(addr + 0x10);
    desc->dest_addr23 = get_uint16(addr + 0x12);
    desc->dest_addr45 = get_uint16(addr + 0x14);
    desc->src_addr01 = get_uint16(addr + 0x16);
    desc->src_addr23 = get_uint16(addr + 0x18);
    desc->src_addr45 = get_uint16(addr + 0x1A);
    desc->length_field = get_uint16(addr + 0x1C);
}

static void i82596_rx_desc_write(I82596State *s, hwaddr addr,
                                 struct i82596_rx_descriptor *desc,
                                 bool simplified)
{
    /* Write basic RFD fields (always present in both modes) */
    set_uint16(addr + 0x0, desc->status);
    set_uint16(addr + 0x2, desc->command);
    set_uint32(addr + 0x4, desc->link);
    set_uint32(addr + 0x8, desc->rbd_addr);
    set_uint16(addr + 0xC, desc->actual_count);
    set_uint16(addr + 0xE, desc->size);

    if (simplified) {
        set_uint16(addr + 0x10, desc->dest_addr01);
        set_uint16(addr + 0x12, desc->dest_addr23);
        set_uint16(addr + 0x14, desc->dest_addr45);
        set_uint16(addr + 0x16, desc->src_addr01);
        set_uint16(addr + 0x18, desc->src_addr23);
        set_uint16(addr + 0x1A, desc->src_addr45);
        set_uint16(addr + 0x1C, desc->length_field);
    }
}

static void i82596_rbd_read(I82596State *s, hwaddr addr,
                            struct i82596_rx_buffer_desc *rbd)
{
    rbd->count = get_uint16(addr + 0x0);
    rbd->reserved1 = get_uint16(addr + 0x2);
    rbd->link = get_uint32(addr + 0x4);
    rbd->buffer = get_uint32(addr + 0x8);
    rbd->size = get_uint16(addr + 0xC);
    rbd->reserved2 = get_uint16(addr + 0xE);
}

static void i82596_rbd_write(I82596State *s, hwaddr addr,
                             struct i82596_rx_buffer_desc *rbd)
{
    set_uint16(addr + 0x0, rbd->count);
    set_uint16(addr + 0x2, rbd->reserved1);
    set_uint32(addr + 0x4, rbd->link);
    set_uint32(addr + 0x8, rbd->buffer);
    set_uint16(addr + 0xC, rbd->size);
    set_uint16(addr + 0xE, rbd->reserved2);
}

static void i82596_dump_tx_descriptor(I82596State *s, hwaddr addr,
                                      struct i82596_tx_descriptor *desc)
{
    DBG(printf("TFD @0x%08lx: status=0x%04x cmd=0x%04x link=0x%08x "
               "tbd=0x%08x count=%d\n",
               (unsigned long)addr, desc->status, desc->command,
               desc->link, desc->tbd_addr, desc->tcb_count));
}

// static void i82596_dump_rx_descriptor(I82596State *s, hwaddr addr,
//                                       struct i82596_rx_descriptor *desc)
// {
//     uint8_t dest[6], src[6];
//
//     // Extract destination address
//     dest[0] = desc->dest_addr01 & 0xFF;
//     dest[1] = (desc->dest_addr01 >> 8) & 0xFF;
//     dest[2] = desc->dest_addr23 & 0xFF;
//     //dest[3] = (dest_addr23 >> 8) & 0xFF;
//     dest[4] = desc->dest_addr45 & 0xFF;
//     dest[5] = (desc->dest_addr45 >> 8) & 0xFF;
//
//     // Extract source address
//     src[0] = desc->src_addr01 & 0xFF;
//     src[1] = (desc->src_addr01 >> 8) & 0xFF;
//     src[2] = desc->src_addr23 & 0xFF;
//     src[3] = (desc->src_addr23 >> 8) & 0xFF;
//     src[4] = desc->src_addr45 & 0xFF;
//     src[5] = (desc->src_addr45 >> 8) & 0xFF;
//
//     DBG(printf("RFD @0x%08lx:\n", (unsigned long)addr));
//     DBG(printf("  Status=0x%04x Cmd=0x%04x Link=0x%08x RBD=0x%08x\n",
//                desc->status, desc->command, desc->link, desc->rbd_addr));
//     DBG(printf("  Count=%d Size=%d EOF=%d F=%d\n",
//                desc->actual_count & 0x3FFF,
//                desc->size & 0x3FFF,
//                !!(desc->actual_count & I596_EOF),
//                !!(desc->actual_count & 0x4000)));
//     DBG(printf("  Dest=" MAC_FMT " Src=" MAC_FMT " Len=0x%04x\n",
//                MAC_ARG(dest), MAC_ARG(src), desc->length_field));
// }

static void i82596_dump_rbd(I82596State *s, hwaddr addr,
                            struct i82596_rx_buffer_desc *rbd)
{
    DBG(printf("  RBD @0x%08lx: count=%d EOF=%d F=%d buf=0x%08x size=%d EL=%d P=%d\n",
               (unsigned long)addr,
               rbd->count & 0x3FFF,
               !!(rbd->count & I596_EOF),
               !!(rbd->count & 0x4000),
               rbd->buffer,
               rbd->size & 0x3FFF,
               !!(rbd->size & CMD_EOL),
               !!(rbd->size & 0x4000)));
}


/* Packet Header Debugger */
struct qemu_ether_header {
    uint8_t ether_dhost[6];
    uint8_t ether_shost[6];
    uint16_t ether_type;
};

#define PRINT_PKTHDR(txt, BUF) do {                  \
    struct qemu_ether_header *hdr = (void *)(BUF); \
    printf(txt ": packet dhost=" MAC_FMT ", shost=" MAC_FMT ", type=0x%04x\n",\
           MAC_ARG(hdr->ether_dhost), MAC_ARG(hdr->ether_shost),        \
           be16_to_cpu(hdr->ether_type));       \
} while (0)

static void i82596_cleanup(I82596State *s)
{
    if (s->throttle_timer) {
        timer_del(s->throttle_timer);
    }
    if (s->flush_queue_timer) {
        timer_del(s->flush_queue_timer);
    }
}

static void i82596_s_reset(I82596State *s)
{
    trace_i82596_s_reset(s);
    i82596_cleanup(s);
    memset(s->config, 0, sizeof(s->config)); /* Clearing config bits */
    s->scp = 0x00FFFFF4;
    s->scb = 0;
    s->scb_base = 0;
    s->scb_status = 0;
    s->cu_status = CU_IDLE;
    s->rx_status = RX_IDLE;
    s->cmd_p = I596_NULL;
    s->lnkst = 0x8000;
    s->ca = s->ca_active = 0;
    s->send_irq = 0;

    /* Statistical Counters */
    s->crc_err = 0;
    s->align_err = 0;
    s->resource_err = 0;
    s->over_err = 0;
    s->rcvdt_err = 0;
    s->short_fr_error = 0;
    s->total_frames = 0;
    s->total_good_frames = 0;
    s->collision_events = 0;
    s->total_collisions = 0;
    s->tx_good_frames = 0;
    s->tx_collisions = 0;
    s->tx_aborted_errors = 0;
    s->last_tx_len = 0;

    s->last_good_rfa = 0;
    s->queue_head = 0;
    s->queue_tail = 0;
    s->queue_count = 0;

    s->t_on = 0xFFFF;
    s->t_off = 0;
    s->throttle_state = true;
    timer_del(s->throttle_timer);
    s->throttle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   i82596_bus_throttle_timer, s);
    if (!I596_FULL_DUPLEX && s->t_on != 0xFFFF) {
        timer_mod(s->throttle_timer,
                 qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                 s->t_on * NANOSECONDS_PER_MICROSECOND);
    }

    qemu_set_irq(s->irq, 1); /* Here, we confirm to the driver done reset, HPUX needs this */
}

void i82596_h_reset(void *opaque)
{
    I82596State *s = opaque;

    i82596_s_reset(s);
}

/*
 * Mode Transition of address functionality.
 * Note: As of now the 82596 is tested only for Linear Mode as it is most
 * widely used by Linux and HPUX systems. This function is here for futureproofing
 * our 82596 device model.
 * According to the documentation the translation of addresses based on mode are
 * done for the following cases: ISCP, SCB, CBP, RFD, TFD,
 * RBD, TBD, Rx Buffers, Tx Buffers
 * Please refer to the documentation for more details.
 */
static inline uint32_t i82596_translate_address(I82596State *s, uint32_t addr, bool is_data_buffer)
{
    if (addr == I596_NULL || addr == 0) {
        return addr;
    }
    switch (s->mode) {
    case I82586_MODE:
        if (is_data_buffer) {
            return addr & 0x00FFFFFF;
        } else {
            if (s->scb_base) {
                return (s->scb_base & 0x00FFFFFF) + (addr & 0xFFFF);
            } else {
                return addr & 0x00FFFFFF;
            }
        }
        break;
    case I82596_MODE_SEGMENTED:
        if (is_data_buffer) {
            return addr;
        } else {
            if (s->scb_base) {
                return s->scb_base + (addr & 0xFFFF);
            } else {
                return addr;
            }
        }
        break;
    case I82596_MODE_LINEAR:
    default:
        return addr;
    }
}


static int i82596_tx_copy_buffers(I82596State *s, hwaddr tfd_addr,
                                  struct i82596_tx_descriptor *desc)
{
    bool simplified_mode = !(desc->command & CMD_FLEX);
    uint32_t total_len = 0;
    uint32_t tbd_addr;
    struct i82596_tx_buffer_desc tbd;

    s->tx_frame_len = 0;

    if (simplified_mode) {
        /* Simplified mode: data follows TFD header */
        uint16_t frame_len = desc->tcb_count & SIZE_MASK;

        DBG(printf("TX: Simplified mode, reading %d bytes from TFD\n", frame_len));

        if (frame_len == 0 || frame_len > sizeof(s->tx_buffer)) {
            DBG(printf("TX: Invalid frame length %d\n", frame_len));
            return -1;
        }

        /* Data starts at offset 16 in TFD */
        address_space_read(&address_space_memory, tfd_addr + 16,
                          MEMTXATTRS_UNSPECIFIED, s->tx_buffer, frame_len);
        total_len = frame_len;

    } else {
        /* Flexible mode: follow TBD chain */
        tbd_addr = desc->tbd_addr;

        while (tbd_addr != I596_NULL && tbd_addr != 0) {
            uint16_t buf_size;
            uint32_t buf_addr;

            /* Translate and validate TBD address */
            tbd_addr = i82596_translate_address(s, tbd_addr, false);
            if (tbd_addr == 0 || tbd_addr == I596_NULL) {
                DBG(printf("TX: Invalid TBD address\n"));
                return -1;
            }

            /* Read TBD */
            i82596_tbd_read(s, tbd_addr, &tbd);

            buf_size = tbd.size & SIZE_MASK;
            buf_addr = i82596_translate_address(s, tbd.buffer, true);

            /* Dump TBD details */
            DBG(printf("  TBD @0x%08x: size=0x%04x(mask=0x%04x) len=%d buf_addr=0x%08x EOF=%d link=0x%08x\n",
                       tbd_addr, tbd.size, SIZE_MASK, buf_size, buf_addr,
                       !!(tbd.size & I596_EOF), tbd.link));

            /* Check buffer space */
            if (total_len + buf_size > sizeof(s->tx_buffer)) {
                DBG(printf("TX: Buffer overflow, max=%zu current=%d adding=%d\n",
                           sizeof(s->tx_buffer), total_len, buf_size));
                return -1;
            }

            /* Copy buffer data */
            if (buf_size > 0 && buf_addr != 0 && buf_addr != I596_NULL) {
                address_space_read(&address_space_memory, buf_addr,
                                  MEMTXATTRS_UNSPECIFIED,
                                  s->tx_buffer + total_len, buf_size);
                total_len += buf_size;
            }

            /* Check for end of frame */
            if (tbd.size & I596_EOF) {
                break;
            }

            /* Move to next TBD */
            tbd_addr = tbd.link;
        }
    }

    s->tx_frame_len = total_len;
    return total_len;
}

static int i82596_tx_process_frame(I82596State *s, bool insert_crc)
{
    uint32_t total_len = s->tx_frame_len;

    if (total_len == 0) {
        return 0;
    }

    /* Insert source MAC address if required */
    if (I596_NO_SRC_ADD_IN == 0 && total_len >= ETH_ALEN * 2) {
        memcpy(&s->tx_buffer[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
        DBG(printf("TX: Inserted source MAC address\n"));
    }

    /* Add padding if frame is too short */
    if (I596_PADDING && total_len < I596_MIN_FRAME_LEN) {
        size_t pad_len = I596_MIN_FRAME_LEN - total_len;
        memset(s->tx_buffer + total_len, 0, pad_len);
        total_len = I596_MIN_FRAME_LEN;
        DBG(printf("TX: Added %zu bytes of padding\n", pad_len));
    }

    /* Calculate and append CRC if required */
    if (insert_crc) {
        total_len = i82596_append_crc(s, s->tx_buffer, total_len);
        DBG(printf("TX: CRC appended, total length now %d\n", (int)total_len));
    }

    s->tx_frame_len = total_len;
    return total_len;
}

static void i82596_tx_update_status(I82596State *s, hwaddr tfd_addr,
                                    struct i82596_tx_descriptor *desc,
                                    uint16_t tx_status, uint16_t collision_count,
                                    bool success)
{
    /* Build status word */
    desc->status = STAT_C;  /* Command complete */

    if (success) {
        desc->status |= STAT_OK;
    } else {
        desc->status |= STAT_A;  /* Abort */
    }

    /* Add collision count in lower byte (bits 0-3) */
    if (collision_count > 0) {
        desc->status |= (collision_count & 0x0F);
    }

    /* Write descriptor back to memory */
    i82596_tx_desc_write(s, tfd_addr, desc);

    DBG(printf("TX: Updated TFD status=0x%04x tx_status=0x%04x collisions=%d\n",
               desc->status, tx_status, collision_count));
}

static int i82596_tx_csma_cd(I82596State *s, uint16_t *tx_status)
{
    int retry_count = 0;
    bool medium_available;

    /* Skip CSMA/CD in full-duplex or loopback mode */
    if (I596_FULL_DUPLEX || I596_LOOPBACK) {
        return 0;
    }

    /* Try to access medium with exponential backoff */
    while (retry_count < CSMA_MAX_RETRIES) {
        medium_available = i82596_check_medium_status(s);

        if (medium_available) {
            break;
        }

        /* Medium busy - perform backoff */
        int backoff_time = i82596_csma_backoff(s, retry_count);

        DBG(printf("CSMA/CD: Collision detected, backoff=%d µs (retry %d/%d)\n",
                   backoff_time, retry_count + 1, CSMA_MAX_RETRIES));

        retry_count++;
        s->total_collisions++;
    }

    /* Check for excessive collisions */
    if (retry_count >= CSMA_MAX_RETRIES) {
        DBG(printf("CSMA/CD: Excessive collisions, aborting transmission\n"));
        *tx_status |= TX_ABORTED_ERRORS;
        return -1;
    }

    /* Set collision flag if any retries occurred */
    if (retry_count > 0) {
        *tx_status |= TX_COLLISIONS;
        s->collision_events++;
    }

    return retry_count;
}

static void i82596_xmit(I82596State *s, uint32_t addr)
{
    struct i82596_tx_descriptor tfd;
    hwaddr tfd_addr = addr;
    uint16_t tx_status = 0;
    int collision_count = 0;
    int frame_len;
    bool success = true;
    bool insert_crc;

    DBG(printf("====== TX START: TFD @0x%08x ======\n", addr));

    i82596_tx_desc_read(s, tfd_addr, &tfd);
    i82596_dump_tx_descriptor(s, tfd_addr, &tfd); /* For debug*/
    s->current_tx_desc = tfd_addr;
    insert_crc = (I596_NOCRC_INS == 0) && ((tfd.command & 0x10) == 0) && !I596_LOOPBACK;
    collision_count = i82596_tx_csma_cd(s, &tx_status);
    if (collision_count < 0) {
        success = false;
        goto tx_complete;
    }
    frame_len = i82596_tx_copy_buffers(s, tfd_addr, &tfd);
    if (frame_len < 0) {
        DBG(printf("TX: Failed to copy buffers\n"));
        tx_status |= TX_ABORTED_ERRORS;
        success = false;
        goto tx_complete;
    }
    DBG(printf("TX: Copied %d bytes from descriptors\n", frame_len));
    frame_len = i82596_tx_process_frame(s, insert_crc);
    if (frame_len <= 0) {
        DBG(printf("TX: Frame processing failed\n"));
        tx_status |= TX_ABORTED_ERRORS;
        success = false;
        goto tx_complete;
    }
    s->last_tx_len = frame_len;
    DBG(PRINT_PKTHDR("TX Send", s->tx_buffer));
    DBG(printf("TX: Transmitting %d bytes\n", frame_len));

    if (I596_LOOPBACK) {
        i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, frame_len);
    } else {
        if (s->nic) {
            qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, frame_len);
        }
    }

tx_complete:
    i82596_tx_update_status(s, tfd_addr, &tfd, tx_status, collision_count, success);
    i82596_update_statistics(s, true, tx_status, collision_count);
    if (tfd.command & CMD_INTR) {
        i82596_update_cu_status(s, tfd.status, true);
    }
    DBG(printf("====== TX END: status=0x%04x success=%d collisions=%d ======\n",
               tfd.status, success, collision_count));
}

bool i82596_can_receive(NetClientState *nc)
{
    I82596State *s = qemu_get_nic_opaque(nc);

    /* Note: The Tx taking presidence over Rx in half-duplex mode
     * is handled in the i82596_check_medium_status() function.
     * Half duplex: Cannot Rx while in Tx
     * Full duplex: we can Rx during transmission
     */
    if (!s->throttle_state && !I596_FULL_DUPLEX) {
        DBG(printf("CAN_RX: FALSE - throttle off in half duplex\n"));
        return false;
    }

    if (s->rx_status == RX_SUSPENDED) {
        DBG(printf("CAN_RX: FALSE - RX suspended\n"));
        return false;
    }

    if (!s->lnkst) {
        DBG(printf("CAN_RX: FALSE - Link down\n"));
        return false;
    }

    if (timer_pending(s->flush_queue_timer)) {
        bool can_rx = s->rx_status == RX_READY;
        return can_rx;
    }

    DBG(printf("CAN_RX: TRUE - all conditions passed\n"));
    return true;
}

static int i82596_validate_receive_state(I82596State *s, size_t *sz)
{
    if (*sz < 14 || *sz > PKT_BUF_SZ - 4) {
        trace_i82596_receive_analysis(">>> Packet size invalid");
        return -1;
    }
    if (s->rx_status == RX_SUSPENDED) {
        trace_i82596_receive_analysis(">>> Receiving is suspended");
        return -1;
    }

    if (s->rx_status != RX_READY) {
        trace_i82596_receive_analysis(">>> RU not ready");
        return -1;
    }

    if (!s->lnkst) {
        trace_i82596_receive_analysis(">>> Link is down");
        return -1;
    }

    return 1;
}

static bool i82596_check_packet_filter(I82596State *s, const uint8_t *buf, uint16_t *is_broadcast)
{
    static const uint8_t broadcast_macaddr[6] = {
        0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

    /* Handle packet based on MAC address type */
    if (I596_PROMISC || I596_LOOPBACK) {
        trace_i82596_receive_analysis(">>> packet received in promiscuous mode");
        return true;
    } else {
        if (!memcmp(buf, broadcast_macaddr, 6)) {
            /* broadcast address */
            if (I596_BC_DISABLE) {
                trace_i82596_receive_analysis(">>> broadcast packet rejected");
                return false;
            }
            trace_i82596_receive_analysis(">>> broadcast packet received");
            *is_broadcast = 1;
            return true;
        } else if (buf[0] & 0x01) {
            /* multicast */
            if (!I596_MC_ALL) {
                trace_i82596_receive_analysis(">>> multicast packet rejected");
                return false;
            }

            int mcast_idx = (net_crc32(buf, ETH_ALEN) & BITS(7, 2)) >> 2;
            assert(mcast_idx < 8 * sizeof(s->mult));

            if (!(s->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7)))) {
                trace_i82596_receive_analysis(">>> multicast address mismatch");
                return false;
            }

            trace_i82596_receive_analysis(">>> multicast packet received");
            *is_broadcast = 1;
            return true;
        } else if (!memcmp(s->conf.macaddr.a, buf, 6)) {
            /* match */
            trace_i82596_receive_analysis(">>> physical address matching packet received");
            return true;
        } else {
            trace_i82596_receive_analysis(">>> unknown packet");
            return false;
        }
    }
}

static bool i82596_monitor(I82596State *s, const uint8_t *buf, size_t sz, bool packet_passes_filter)
{
    if (I596_MONITOR_MODE == MONITOR_DISABLED) {
        return true;
    }
    DBG(printf("MONITOR: Processing packet in monitor mode %d\n", I596_MONITOR_MODE));
    if (sz < I596_MIN_FRAME_LEN) {
        s->short_fr_error++;
        DBG(printf("MONITOR: Short frame detected (%zu bytes)\n", sz));
    }
    if ((sz % 2) != 0) {
        s->align_err++;
        DBG(printf("MONITOR: Alignment error detected\n"));
    }

    switch (I596_MONITOR_MODE) {
        case MONITOR_NORMAL: /* Mode 0 - Dont Monitor just add to total frames */
            if (packet_passes_filter) {
                s->total_good_frames++;
                DBG(printf("MONITOR: Normal mode - receiving filtered frame\n"));
                return true;
            } else {
                DBG(printf("MONITOR: Normal mode - monitoring rejected frame\n"));
                return false;
            }
            break;
        case MONITOR_FILTERED: /* Mode 01 - Monitor only filtered packets */
            s->total_frames++;
            if (packet_passes_filter) {
                s->total_good_frames++;
                DBG(printf("MONITOR: Filtered mode - frame passed filtering\n"));
            }
            return false;
        case MONITOR_ALL: /* Mode 02 - Monitor all packets */
            s->total_frames++;
            if (packet_passes_filter) {
                s->total_good_frames++;
            }
            DBG(printf("MONITOR: All mode - monitoring all frames\n"));
            return false;

        default:
            return true;
    }
}

static void i82596_update_rx_state(I82596State *s, int new_state)
{
    s->rx_status = new_state;

    switch (new_state) {
    case RX_NO_RESOURCES:
        s->scb_status |= SCB_STATUS_RNR;
        break;
    case RX_SUSPENDED:
        s->scb_status |= SCB_STATUS_RNR;
        timer_del(s->flush_queue_timer);
        break;
    default:
        break;
    }
}

static void __attribute__((unused)) i82596_rx_store_frame_header(I82596State *s,
                                         struct i82596_rx_descriptor *rfd,
                                         const uint8_t *buf, size_t size)
{
    /* Store destination address (6 bytes) - network byte order */
    rfd->dest_addr01 = (buf[0] << 8) | buf[1];
    rfd->dest_addr23 = (buf[2] << 8) | buf[3];
    rfd->dest_addr45 = (buf[4] << 8) | buf[5];

    /* Store source address (6 bytes) - network byte order */
    if (size >= 12) {
        rfd->src_addr01 = (buf[6] << 8) | buf[7];
        rfd->src_addr23 = (buf[8] << 8) | buf[9];
        rfd->src_addr45 = (buf[10] << 8) | buf[11];
    }

    /* Store length/type field (2 bytes) - network byte order */
    if (size >= 14) {
        rfd->length_field = (buf[12] << 8) | buf[13];
    }

    DBG(printf("RX: Stored frame header in RFD: "
               "dest=%02x:%02x:%02x:%02x:%02x:%02x "
               "src=%02x:%02x:%02x:%02x:%02x:%02x "
               "len=0x%04x\n",
               buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
               buf[6], buf[7], buf[8], buf[9], buf[10], buf[11],
               rfd->length_field));
}

static size_t __attribute__((unused)) i82596_rx_copy_to_rfd(I82596State *s, hwaddr rfd_addr,
                                    const uint8_t *buf, size_t size,
                                    size_t rfd_size)
{
    size_t to_copy = MIN(size, rfd_size);
    size_t data_offset = 0x1E;  /* Data area starts after frame header fields */

    if (to_copy > 0) {
        address_space_write(&address_space_memory, rfd_addr + data_offset,
                           MEMTXATTRS_UNSPECIFIED, buf, to_copy);
        DBG(printf("RX: Copied %zu bytes to RFD data area @ 0x%08lx\n",
                   to_copy, (unsigned long)(rfd_addr + data_offset)));
    }

    return to_copy;
}

static size_t __attribute__((unused)) i82596_rx_copy_to_rbds(I82596State *s, hwaddr rbd_addr,
                                     const uint8_t *buf, size_t size,
                                     bool *out_of_resources)
{
    size_t bytes_copied = 0;
    hwaddr current_rbd = rbd_addr;
    *out_of_resources = false;

    DBG(printf("RX: Processing RBD chain starting at 0x%08lx, %zu bytes to copy\n",
               (unsigned long)rbd_addr, size));

    while (bytes_copied < size && current_rbd != I596_NULL && current_rbd != 0) {
        struct i82596_rx_buffer_desc rbd;

        /* Read RBD */
        i82596_rbd_read(s, current_rbd, &rbd);

        /* Check if RBD is prefetched (cannot be modified) */
        if (rbd.size & 0x4000) {  /* P bit set */
            DBG(printf("RX: RBD is prefetched, skipping\n"));
            break;
        }

        /* Get buffer size and address */
        uint16_t buf_size = rbd.size & 0x3FFF;
        hwaddr buf_addr = i82596_translate_address(s, rbd.buffer, true);

        i82596_dump_rbd(s, current_rbd, &rbd);

        /* Check for valid buffer */
        if (buf_addr == 0 || buf_addr == I596_NULL) {
            DBG(printf("RX: Invalid RBD buffer address\n"));
            *out_of_resources = true;
            break;
        }

        /* Calculate how much to copy */
        size_t remaining = size - bytes_copied;
        size_t to_copy = MIN(remaining, buf_size);

        /* Copy data to buffer */
        if (to_copy > 0) {
            address_space_write(&address_space_memory, buf_addr,
                               MEMTXATTRS_UNSPECIFIED,
                               buf + bytes_copied, to_copy);
            bytes_copied += to_copy;

            DBG(printf("RX: Copied %zu bytes to RBD buffer @ 0x%08lx\n",
                       to_copy, (unsigned long)buf_addr));
        }

        rbd.count = to_copy | 0x4000;  /* Set F (filled) bit */
        if (bytes_copied >= size) {
            rbd.count |= I596_EOF;  /* Set EOF bit */
            DBG(printf("RX: Marked RBD with EOF\n"));
        }

        /* Write RBD back */
        i82596_rbd_write(s, current_rbd, &rbd);

        /* Check if this is end of list */
        if (rbd.size & CMD_EOL) {  /* EL bit */
            DBG(printf("RX: Reached end of RBD list\n"));
            if (bytes_copied < size) {
                *out_of_resources = true;
            }
            break;
        }

        /* Move to next RBD */
        current_rbd = i82596_translate_address(s, rbd.link, false);
    }

    DBG(printf("RX: RBD chain copy complete, %zu bytes copied\n", bytes_copied));
    return bytes_copied;
}

static void __attribute__((unused)) i82596_rx_update_rfd_status(I82596State *s, hwaddr rfd_addr,
                                        struct i82596_rx_descriptor *rfd,
                                        uint16_t rx_status, size_t actual_count,
                                        bool eof)
{
    /* Set completion and busy flags */
    rfd->status = rx_status | STAT_C | STAT_OK;  /* Complete and OK */
    rfd->status &= ~STAT_B;  /* Clear busy */

    /* Set actual count with EOF and F flags */
    rfd->actual_count = (actual_count & 0x3FFF) | 0x4000;  /* Set F bit */
    if (eof) {
        rfd->actual_count |= I596_EOF;
    }

    /* Write RFD back to memory (Flexible mode - no frame header fields) */
    i82596_rx_desc_write(s, rfd_addr, rfd, false);

    DBG(printf("RX: Updated RFD status=0x%04x count=%zu EOF=%d\n",
               rfd->status, actual_count, eof));
}

ssize_t i82596_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    I82596State *s = qemu_get_nic_opaque(nc);
    struct i82596_rx_descriptor rfd;
    uint32_t rfa_pointer, rfd_addr, rbd_addr;
    uint16_t rx_status = 0;
    uint16_t is_broadcast = 0;
    bool packet_completed = true;
    bool simplified_mode = false;
    size_t frame_size = size;
    size_t payload_size = 0;  /* Size of payload after header */
    size_t bytes_copied = 0;
    size_t crc_size = I596_CRC16_32 ? 4 : 2;
    const uint8_t *packet_data = buf;
    bool crc_valid = true;
    bool out_of_resources = false;

    DBG(printf("\n========== i82596_receive() START ==========\n"));
    DBG(printf("RX: Packet size=%zu bytes\n", size));
    DBG(PRINT_PKTHDR("[RX] Frame header", buf));

    /* === STEP 1: Validate RU State and Link Status === */
    DBG(printf("RX: [STEP 1] Validating RU state (rx_status=%d)\n", s->rx_status));

    if (!I596_FULL_DUPLEX && !s->throttle_state) {
        DBG(printf("RX: REJECTED - Throttle off in half-duplex mode\n"));
        DBG(printf("RX: Half-duplex requires throttle_state=true for reception\n"));
        return size; /* Pretend we received it to avoid network stack issues */
    }

    if (i82596_validate_receive_state(s, &size) < 0) {
        DBG(printf("RX: ERROR - Invalid RU state (suspended or not ready)\n"));
        return -1;
    }
    DBG(printf("RX: State validation passed\n"));

    /* === STEP 2: Apply Packet Filtering === */
    DBG(printf("RX: [STEP 2] Applying packet filter\n"));
    bool passes_filter = i82596_check_packet_filter(s, buf, &is_broadcast);

    if (is_broadcast) {
        DBG(printf("RX: Broadcast frame detected\n"));
    }

    /* === STEP 3: Handle Monitor Mode === */
    DBG(printf("RX: [STEP 3] Checking monitor mode\n"));
    if (!i82596_monitor(s, buf, size, passes_filter)) {
        DBG(printf("RX: Frame handled by monitor mode, no further processing\n"));
        return size;
    }

    /* Only continue normal processing if packet passes filter */
    if (!passes_filter) {
        DBG(printf("RX: REJECTED - Frame did not pass address filter\n"));
        return size;
    }
    DBG(printf("RX: Frame passed filter, proceeding with reception\n"));

    /* === STEP 4: CRC Verification === */
    DBG(printf("RX: [STEP 4] CRC handling (Loopback=%d, CRC16/32=%d)\n",
               I596_LOOPBACK, I596_CRC16_32));

    if (I596_LOOPBACK && size > crc_size) {
        /* Only verify CRC in loopback mode where we know CRC is present */
        crc_valid = i82596_verify_crc(s, buf, size);
        if (!crc_valid) {
            DBG(printf("RX: WARNING - CRC verification FAILED in loopback\n"));
            rx_status |= RX_CRC_ERRORS;
            i82596_record_error(s, RX_CRC_ERRORS, false);
            s->crc_err++;

            if (!SAVE_BAD_FRAMES) {
                DBG(printf("RX: Discarding frame (SAVE_BAD_FRAMES=false)\n"));
                return size;  /* Discard frame with bad CRC */
            }
            DBG(printf("RX: Saving bad frame (SAVE_BAD_FRAMES=true)\n"));
        } else {
            DBG(printf("RX: CRC verification passed\n"));
        }
        frame_size = size - crc_size;  /* Remove existing CRC */
        DBG(printf("RX: Removed CRC, frame_size=%zu\n", frame_size));
    } else {
        /* For normal network reception, assume CRC is good (handled by physical layer) */
        crc_valid = true;
        frame_size = size;  /* No CRC to remove */
        DBG(printf("RX: Normal reception, assuming hardware CRC validation\n"));
    }

    /* === STEP 5: Get RFD from SCB RFA Pointer === */
    DBG(printf("RX: [STEP 5] Reading RFD from SCB RFA pointer\n"));

    rfd_addr = get_uint32(s->scb + 8);  /* Read current RFA from SCB */
    DBG(printf("RX: Read RFA from SCB = 0x%08lx\n", (unsigned long)rfd_addr));

    if (rfd_addr == 0 || rfd_addr == I596_NULL) {
        DBG(printf("RX: ERROR - Invalid cached RFD address\n"));
        i82596_update_rx_state(s, RX_NO_RESOURCES);
        s->resource_err++;
        set_uint16(s->scb, get_uint16(s->scb) | SCB_STATUS_RNR);
        i82596_update_scb_irq(s, true);
        return -1;
    }

    /* In Flexible mode, verify this RFD has an RBD attached */
    int rfd_attempts = 0;
    bool found_rfd_with_rbd = false;
    rfa_pointer = s->last_good_rfa;  /* Use saved logical address */

    while (rfd_attempts < 64 && rfd_addr != 0 && rfd_addr != I596_NULL) {
        DBG(printf("RX: Checking RFD #%d at 0x%08lx\n", rfd_attempts, (unsigned long)rfd_addr));

        if (rfd_addr == 0 || rfd_addr == I596_NULL) {
            break;
        }

        i82596_rx_desc_read(s, rfd_addr, &rfd);

        /* Check if RFD is busy */
        if (rfd.status & STAT_B) {
            DBG(printf("RX: RFD is BUSY, trying next\n"));
            rfa_pointer = rfd.link;
            rfd_attempts++;
            continue;
        }

        bool is_flexible = (rfd.command & CMD_FLEX);
        if (is_flexible) {
            hwaddr check_rbd = i82596_translate_address(s, rfd.rbd_addr, false);
            if (check_rbd == I596_NULL || check_rbd == 0 || check_rbd == 0xFFFFFFFF) {
                DBG(printf("RX: RFD has no RBD (0x%08x), trying next\n", rfd.rbd_addr));
                rfa_pointer = rfd.link;
                rfd_addr = i82596_translate_address(s, rfd.link, false);
                rfd_attempts++;
                continue;
            }
        }

        /* Found a usable RFD! */
        DBG(printf("RX: Found usable RFD at 0x%08lx (attempt %d)\n",
                   (unsigned long)rfd_addr, rfd_attempts));
        found_rfd_with_rbd = true;

        /* Update cached pointer to this usable RFD */
        s->current_rx_desc = rfd_addr;

        if (rfd_attempts == 0) {
            /* This is the first RFD we checked and it has RBD - this is our "good" address */
            s->last_good_rfa = rfa_pointer;
            DBG(printf("RX: Saved last_good_rfa = 0x%08x\n", s->last_good_rfa));
        } else if (rfd_attempts > 0 && s->last_good_rfa == 0) {
            /* We had to skip some RFDs to find this one - save it */
            s->last_good_rfa = rfa_pointer;
            DBG(printf("RX: Saved last_good_rfa = 0x%08x (after skipping %d RFDs)\n",
                       s->last_good_rfa, rfd_attempts));
        }

        break;
    }

    if (!found_rfd_with_rbd) {
        if (s->last_good_rfa != 0) {
            DBG(printf("RX: No RBD at current RFA - trying last_good_rfa 0x%08x\n", s->last_good_rfa));
            rfd_addr = s->last_good_rfa;
            i82596_rx_desc_read(s, rfd_addr, &rfd);
            hwaddr check_rbd = i82596_translate_address(s, rfd.rbd_addr, false);
            if (check_rbd != 0 && check_rbd != I596_NULL && check_rbd != 0xFFFFFFFF) {
                DBG(printf("RX: SUCCESS - Found RBD at last_good_rfa 0x%08x, using it!\n", s->last_good_rfa));
                found_rfd_with_rbd = true;
            } else {
                DBG(printf("RX: last_good_rfa also has no RBD - RBD truly unavailable\n"));
            }
        }

        if (!found_rfd_with_rbd) {
            DBG(printf("RX: No RBD available - setting RX_NO_RESOURCES and generating RNR interrupt\n"));
            i82596_update_rx_state(s, RX_NO_RESOURCES);
            s->resource_err++;
            set_uint16(s->scb, get_uint16(s->scb) | SCB_STATUS_RNR);
            i82596_update_scb_irq(s, true);
            DBG(printf("RX: Packet dropped due to no RBD resources - driver will rebuild\n"));
            return -1;
        }
    }

    DBG(printf("RX: Using RFD at address = 0x%08lx\n", (unsigned long)rfd_addr));

    if (rfd_attempts > 0) {
        set_uint32(s->scb + 8, rfa_pointer);
        DBG(printf("RX: Updated RFA pointer in SCB after skipping %d RFDs\n", rfd_attempts));
    }

    s->current_rx_desc = rfd_addr;

    /* === STEP 6: Read RFD Structure from Memory (refresh after search) === */
    DBG(printf("RX: [STEP 6] Re-reading RFD structure for processing\n"));
    i82596_rx_desc_read(s, rfd_addr, &rfd);

    /* Dump RFD contents before modification */
    DBG(printf("RX: RFD contents BEFORE processing:\n"));
    // i82596_dump_rx_descriptor(s, rfd_addr, &rfd);  // Disabled - function commented out

    /* === STEP 7: Determine Simplified vs Flexible Mode === */
    DBG(printf("RX: [STEP 7] Determining memory structure mode\n"));
    simplified_mode = !(rfd.command & CMD_FLEX);
    DBG(printf("RX: Mode = %s (SF bit = %d)\n",
               simplified_mode ? "SIMPLIFIED" : "FLEXIBLE",
               !!(rfd.command & CMD_FLEX)));
    DBG(printf("RX: Command word = 0x%04x (EL=%d S=%d SF=%d)\n",
               rfd.command,
               !!(rfd.command & CMD_EOL),
               !!(rfd.command & CMD_SUSP),
               !!(rfd.command & CMD_FLEX)));

    /* Set busy bit to indicate we're processing this RFD */
    rfd.status |= STAT_B;
    DBG(printf("RX: Set RFD BUSY bit\n"));

    /* === STEP 8: Validate Frame Size === */
    DBG(printf("RX: [STEP 8] Validating frame size\n"));

    /* Check minimum frame size */
    if (frame_size < 14) {
        DBG(printf("RX: ERROR - Frame too short (%zu bytes), minimum is 14\n", frame_size));
        rx_status |= RX_LENGTH_ERRORS;
        i82596_record_error(s, RX_LENGTH_ERRORS, false);
        s->short_fr_error++;
        packet_completed = false;
        goto rx_complete;
    }

    payload_size = frame_size;  /* Store complete frame including header */
    DBG(printf("RX: Frame size valid, will store complete %zu-byte frame\n", payload_size));

    /* === STEP 9: Process Frame Payload === */
    DBG(printf("RX: [STEP 9] Copying frame payload\n"));

    if (simplified_mode) {
        /* === SIMPLIFIED MODE: All payload goes into RFD data area === */
        uint16_t rfd_size = rfd.size & 0x3FFF;  /* Get RFD data area size */

        DBG(printf("RX: SIMPLIFIED MODE processing\n"));
        DBG(printf("RX: RFD data area size = %d bytes\n", rfd_size));
        DBG(printf("RX: Payload to store = %zu bytes\n", payload_size));

        /* Validate RFD size */
        if (rfd_size % 2 != 0) {
            DBG(printf("RX: ERROR - RFD size is not even (%d), rejecting\n", rfd_size));
            rx_status |= RX_LENGTH_ERRORS;
            i82596_record_error(s, RX_LENGTH_ERRORS, false);
            s->align_err++;
            packet_completed = false;
            goto rx_complete;
        }

        /* Check if payload fits in RFD */
        if (payload_size > rfd_size) {
            DBG(printf("RX: WARNING - Payload too large for RFD (%zu > %d)\n",
                       payload_size, rfd_size));
            rx_status |= RFD_STATUS_TRUNC;
            payload_size = rfd_size;
            packet_completed = !SAVE_BAD_FRAMES ? false : true;
            DBG(printf("RX: Truncating to %zu bytes (SAVE_BAD_FRAMES=%d)\n",
                       payload_size, SAVE_BAD_FRAMES));
        }

        /* Copy COMPLETE frame to RFD data area (starts at offset 0x1E after header fields) */
        if (payload_size > 0) {
            address_space_write(&address_space_memory, rfd_addr + 0x1E,
                               MEMTXATTRS_UNSPECIFIED, packet_data, payload_size);
            bytes_copied = payload_size;
            DBG(printf("RX: Copied complete %zu-byte frame to RFD data area @ 0x%08lx\n",
                       payload_size, (unsigned long)(rfd_addr + 0x1E)));
        }

        i82596_rx_store_frame_header(s, &rfd, packet_data, frame_size);
        DBG(printf("RX: Extracted and stored frame header in RFD fields (Simplified mode)\n"));

    } else {
        /* === FLEXIBLE MODE: Frame distributed between RFD and RBD chain === */
        uint16_t rfd_size = rfd.size & 0x3FFF;  /* Get RFD data area size */
        size_t rfd_frame_size = 0;

        DBG(printf("RX: FLEXIBLE MODE processing\n"));
        DBG(printf("RX: RFD data area size = %d bytes\n", rfd_size));
        DBG(printf("RX: Total frame to store = %zu bytes (including header)\n", payload_size));
        DBG(printf("RX: Note: In Flexible mode, RFD is only 16 bytes (no frame header fields)\n"));

        /* Copy initial frame data to RFD if SIZE > 0 */
        if (rfd_size > 0 && payload_size > 0) {
            rfd_frame_size = MIN(payload_size, rfd_size);
            address_space_write(&address_space_memory, rfd_addr + 0x10,
                              MEMTXATTRS_UNSPECIFIED, packet_data, rfd_frame_size);
            bytes_copied = rfd_frame_size;
            DBG(printf("RX: Copied %zu bytes to RFD data area @ 0x%08lx\n",
                       rfd_frame_size, (unsigned long)(rfd_addr + 0x10)));
        }

        /* === STEP 10: Process RBD Chain for Remaining Frame Data === */
        if (bytes_copied < payload_size) {
            size_t remaining = payload_size - bytes_copied;

            DBG(printf("RX: [STEP 10] Processing RBD chain for remaining %zu bytes\n", remaining));

            /* Get RBD pointer from RFD */
            rbd_addr = i82596_translate_address(s, rfd.rbd_addr, false);
            DBG(printf("RX: RBD address from RFD = 0x%08lx\n", (unsigned long)rbd_addr));

            if (rbd_addr == I596_NULL || rbd_addr == 0 || rbd_addr == 0xFFFFFFFF) {
                /* CRITICAL: No RBD available in Flexible mode - cannot receive frame */
                DBG(printf("RX: CRITICAL - No RBD available (0xFFFFFFFF) in Flexible mode\n"));
                DBG(printf("RX: Cannot receive frame - setting RU to NO_RESOURCES\n"));

                /* Record the error */
                i82596_record_error(s, RFD_STATUS_NOBUFS, false);
                s->resource_err++;

                /* Set RU state to NO_RESOURCES (stops further reception) */
                i82596_update_rx_state(s, RX_NO_RESOURCES);

                /* Set RNR interrupt to notify driver */
                set_uint16(s->scb, get_uint16(s->scb) | SCB_STATUS_RNR);
                i82596_update_scb_irq(s, true);  /* Generate interrupt */

                /* Do NOT mark RFD as complete - leave it for driver to retry */
                /* Do NOT advance RFA pointer - stay on this RFD */
                DBG(printf("RX: RNR interrupt generated, RU stopped, frame discarded\n"));

                return -1;  /* Frame not received - RU needs resources */
            } else {
                /* Use helper function to copy remaining frame data to RBD chain */
                size_t rbd_bytes = i82596_rx_copy_to_rbds(s, rbd_addr,
                                                          packet_data + bytes_copied,
                                                          remaining,
                                                          &out_of_resources);
                bytes_copied += rbd_bytes;

                DBG(printf("RX: Copied %zu bytes through RBD chain\n", rbd_bytes));

                if (out_of_resources) {
                    /* Ran out of RBDs during frame reception */
                    DBG(printf("RX: CRITICAL - Ran out of RBDs mid-frame (NO_RESOURCES)\n"));

                    /* Record the error */
                    i82596_record_error(s, RFD_STATUS_NOBUFS, false);
                    s->resource_err++;

                    /* Set RU state to NO_RESOURCES */
                    i82596_update_rx_state(s, RX_NO_RESOURCES);

                    /* Set RNR interrupt */
                    set_uint16(s->scb, get_uint16(s->scb) | SCB_STATUS_RNR);
                    i82596_update_scb_irq(s, true);  /* Generate interrupt */

                    DBG(printf("RX: RNR interrupt generated, RU stopped\n"));

                    /* Mark frame as incomplete */
                    packet_completed = false;
                }

                if (bytes_copied < payload_size) {
                    DBG(printf("RX: WARNING - Only copied %zu of %zu frame bytes\n",
                               bytes_copied, payload_size));
                    rx_status |= RFD_STATUS_TRUNC;
                    if (!SAVE_BAD_FRAMES) {
                        packet_completed = false;
                    }
                }
            }
        }
        DBG(printf("RX: Frame header not stored separately (Flexible mode)\n"));
    }

rx_complete:

    /* === STEP 11: Optional CRC Storage === */
    DBG(printf("RX: [STEP 11] CRC storage (CRCINM=%d)\n", I596_CRCINM));

    if (I596_CRCINM && packet_completed) {
        uint8_t crc_data[4];
        size_t crc_len = crc_size;

        if (I596_CRC16_32) {
            uint32_t crc = crc32(~0, packet_data, frame_size);
            crc = cpu_to_be32(crc);
            memcpy(crc_data, &crc, 4);
            DBG(printf("RX: Calculated CRC-32 = 0x%08x\n", crc));
        } else {
            uint16_t crc = i82596_calculate_crc16(packet_data, frame_size);
            crc = cpu_to_be16(crc);
            memcpy(crc_data, &crc, 2);
            DBG(printf("RX: Calculated CRC-16 = 0x%04x\n", crc));
        }

        if (simplified_mode) {
            address_space_write(&address_space_memory, rfd_addr + 0x1E + bytes_copied,
                               MEMTXATTRS_UNSPECIFIED, crc_data, crc_len);
            DBG(printf("RX: Stored CRC in RFD @ 0x%08lx\n",
                       (unsigned long)(rfd_addr + 0x1E + bytes_copied)));
        } else {
            DBG(printf("RX: CRC storage in RBD chain not yet implemented\n"));
        }
    }

    /* === STEP 12: Update Statistics === */
    DBG(printf("RX: [STEP 12] Updating statistics\n"));
    /* === STEP 13: Set Final RFD Status === */
    DBG(printf("RX: [STEP 13] Setting final RFD status\n"));

    if (packet_completed && crc_valid) {
        rx_status |= STAT_C | STAT_OK;  /* Complete and OK */
        if (is_broadcast) {
            rx_status |= 0x0001;  /* Broadcast bit */
        }
        DBG(printf("RX: Frame received successfully\n"));
    } else if (packet_completed) {
        /* Frame completed but with errors */
        rx_status |= STAT_C;  /* Complete but not OK */
        if (!crc_valid) {
            rx_status |= RX_CRC_ERRORS;
        }
        DBG(printf("RX: Frame received with errors (crc_valid=%d)\n", crc_valid));
    } else {
        DBG(printf("RX: Frame NOT completed (packet_completed=false)\n"));
        rx_status |= STAT_B;
    }

    /* Clear busy bit and set final status */
    rfd.status = rx_status & ~STAT_B;  /* Clear busy */
    rfd.actual_count = (bytes_copied & 0x3FFF) | 0x4000;  /* Set F bit */
    if (packet_completed) {
        rfd.actual_count |= I596_EOF;  /* Set EOF bit */
    }

    i82596_rx_desc_write(s, rfd_addr, &rfd, simplified_mode);

    DBG(printf("RX: RFD updated - status=0x%04x actual_count=%zu EOF=%d F=%d\n",
               rfd.status, bytes_copied,
               !!(rfd.actual_count & I596_EOF),
               !!(rfd.actual_count & 0x4000)));

    DBG(printf("RX: RFD contents AFTER processing:\n"));

    /* === STEP 14: Advance RFA Pointer === */
    DBG(printf("RX: [STEP 14] Advancing RFA pointer\n"));

    if (packet_completed && crc_valid) {
        hwaddr next_rfd_addr = i82596_translate_address(s, rfd.link, false);
        if (next_rfd_addr != 0 && next_rfd_addr != I596_NULL && next_rfd_addr != 0xFFFFFFFF) {
            struct i82596_rx_descriptor next_rfd;
            i82596_rx_desc_read(s, next_rfd_addr, &next_rfd);

            hwaddr current_rbd_addr = i82596_translate_address(s, rfd.rbd_addr, false);
            if (current_rbd_addr != 0 && current_rbd_addr != I596_NULL && current_rbd_addr != 0xFFFFFFFF) {
                struct i82596_rx_buffer_desc current_rbd;
                i82596_rbd_read(s, current_rbd_addr, &current_rbd);

                uint32_t next_rbd_link = current_rbd.link;

                next_rfd.rbd_addr = next_rbd_link;
                DBG(printf("RX: Advanced RBD chain: current RBD=0x%08x, next RBD=0x%08x for next RFD at 0x%08lx\n",
                           rfd.rbd_addr, next_rbd_link, (unsigned long)next_rfd_addr));

                /* Write updated next RFD back to memory */
                i82596_rx_desc_write(s, next_rfd_addr, &next_rfd, simplified_mode);

                /* Clear RBD pointer from current RFD (mark as used) */
                rfd.rbd_addr = I596_NULL;
                i82596_rx_desc_write(s, rfd_addr, &rfd, simplified_mode);

                /* Update last_good_rfa to point to next RFD where next RBD now is */
                s->last_good_rfa = next_rfd_addr;
                DBG(printf("RX: Updated last_good_rfa to 0x%08x (next RFD with next RBD)\n", s->last_good_rfa));
            }

            set_uint32(s->scb + 8, next_rfd_addr);
            DBG(printf("RX: *** ADVANCED SCB RFA pointer from 0x%08lx to 0x%08lx ***\n",
                       (unsigned long)rfd_addr, (unsigned long)next_rfd_addr));
        } else {
            DBG(printf("RX: Reached end of RFD list (next=0x%08lx)\n", (unsigned long)next_rfd_addr));
        }
    }

    /* === STEP 15: Handle RFD Command Flags === */
    DBG(printf("RX: [STEP 15] Processing command flags\n"));

    if (rfd.command & CMD_SUSP) {
        DBG(printf("RX: SUSPEND bit set - suspending RU after frame\n"));
        i82596_update_rx_state(s, RX_SUSPENDED);
    }

    if (rfd.command & CMD_EOL) {
        DBG(printf("RX: END-OF-LIST bit set - no more RFDs available\n"));
        i82596_update_rx_state(s, RX_NO_RESOURCES);
    }

    /* === STEP 16: Generate Interrupt === */
    DBG(printf("RX: [STEP 16] Interrupt handling\n"));

    if (packet_completed && crc_valid) {
        s->scb_status |= SCB_STATUS_FR;  /* Frame Received */
        DBG(printf("RX: Set SCB_STATUS_FR (Frame Received)\n"));

        i82596_update_scb_irq(s, true);
        DBG(printf("RX: Generated FR interrupt\n"));
    } else {
        DBG(printf("RX: No interrupt (frame not completed successfully)\n"));
    }

    /* === STEP 17: Update SCB Statistics Counters === */
    DBG(printf("RX: [STEP 17] Updating SCB error counters\n"));
    set_uint32(s->scb + 16, s->crc_err);        /* CRC error counter */
    set_uint32(s->scb + 18, s->align_err);      /* Alignment error counter */
    set_uint32(s->scb + 20, s->resource_err);   /* Resource error counter */
    set_uint32(s->scb + 22, s->over_err);       /* Overrun error counter */

    DBG(printf("RX: Error counters - CRC:%d Align:%d Resource:%d Overrun:%d\n",
               s->crc_err, s->align_err, s->resource_err, s->over_err));

    DBG(printf("========== i82596_receive() END ==========\n"));
    DBG(printf("RX: Final status=0x%04x payload_copied=%zu frame_size=%zu\n",
               rx_status, bytes_copied, frame_size));
    DBG(printf("RX: Result: %s\n",
               packet_completed && crc_valid ? "SUCCESS" : "FAILED/INCOMPLETE"));
    if (packet_completed && crc_valid && s->last_good_rfa != 0) {
        uint32_t current_rfa = get_uint32(s->scb + 8);
        if (current_rfa != s->last_good_rfa) {
            set_uint32(s->scb + 8, s->last_good_rfa);
            DBG(printf("RX: Post-reception RFA reset from 0x%08x to 0x%08x (proactive fix)\n",
                       current_rfa, s->last_good_rfa));
        }
    }

    DBG(printf("\n"));

    timer_mod(s->flush_queue_timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 5000000); /* 5ms */

    return size;
}

ssize_t i82596_receive_iov(NetClientState *nc, const struct iovec *iov, int iovcnt)
{
    size_t sz = 0;
    uint8_t *buf;
    int i;
    for (i = 0; i < iovcnt; i++) {
        sz += iov[i].iov_len;
    }
    if (sz == 0) {
        return -1;
    }
    buf = g_malloc(sz);
    if (!buf) {
        return -1;
    }
    size_t offset = 0;
    for (i = 0; i < iovcnt; i++) {
        if (iov[i].iov_base == NULL) {
            g_free(buf);
            return -1;
        }
        memcpy(buf + offset, iov[i].iov_base, iov[i].iov_len);
        offset += iov[i].iov_len;
    }
    PRINT_PKTHDR("Receive IOV:", buf);
    i82596_receive(nc, buf, sz);
    g_free(buf);
    return sz;
}

static void set_individual_address(I82596State *s, uint32_t addr)
{
    NetClientState *nc;
    uint8_t *m;

    nc = qemu_get_queue(s->nic);
    m = s->conf.macaddr.a;
    address_space_read(&address_space_memory, addr + 8,
                       MEMTXATTRS_UNSPECIFIED, m, ETH_ALEN);
    qemu_format_nic_info_str(nc, m);
    trace_i82596_new_mac(nc->info_str);
}

static void set_multicast_list(I82596State *s, uint32_t addr)
{
    uint16_t mc_count, i;

    memset(&s->mult[0], 0, sizeof(s->mult));
    mc_count = get_uint16(addr + 8) / ETH_ALEN;
    addr += 10;
    if (mc_count > MAX_MC_CNT) {
        mc_count = MAX_MC_CNT;
    }
    for (i = 0; i < mc_count; i++) {
        uint8_t multicast_addr[ETH_ALEN];
        address_space_read(&address_space_memory, addr + i * ETH_ALEN,
                           MEMTXATTRS_UNSPECIFIED, multicast_addr, ETH_ALEN);
        unsigned mcast_idx = (net_crc32(multicast_addr, ETH_ALEN) &
                              BITS(7, 2)) >> 2;
        assert(mcast_idx < 8 * sizeof(s->mult));
        s->mult[mcast_idx >> 3] |= (1 << (mcast_idx & 7));
    }
    trace_i82596_set_multicast(mc_count);
}

void i82596_set_link_status(NetClientState *nc)
{
    I82596State *s = qemu_get_nic_opaque(nc);
    bool was_up = s->lnkst != 0;

    s->lnkst = nc->link_down ? 0 : 0x8000;
    bool is_up = s->lnkst != 0;

    if (!was_up && is_up && s->rx_status == RX_READY) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static bool i82596_check_medium_status(I82596State *s)
{
    if (I596_FULL_DUPLEX) {
        return true;
    }

    if (!s->throttle_state) {
        DBG(printf("CSMA/CD: Medium busy (throttle off)\n"));
        return false;
    }

    if (!I596_LOOPBACK && (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) % 100 < 5)) {
        s->collision_events++;
        DBG(printf("CSMA/CD: Simulated collision detected\n"));
        return false;
    }

    return true;
}

static int i82596_csma_backoff(I82596State *s, int retry_count)
{
    int backoff_factor, slot_count, backoff_time;

    backoff_factor = MIN(retry_count, CSMA_BACKOFF_LIMIT);
    slot_count = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) % (1 << backoff_factor);
    backoff_time = slot_count * CSMA_SLOT_TIME;

    DBG(printf("CSMA/CD: Backing off for %d microseconds (retry %d)\n",
               backoff_time, retry_count));

    return backoff_time;
}

static uint16_t i82596_calculate_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    size_t i, j;

    for (i = 0; i < len; i++) {
        crc ^= data[i] << 8;
        for (j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static size_t i82596_append_crc(I82596State *s, uint8_t *buffer, size_t len)
{
    if (len + 4 > PKT_BUF_SZ) {
        DBG(printf("CRC: Buffer too small to append CRC\n"));
        return len;
    }

    if (I596_CRC16_32) {
        uint32_t crc = crc32(~0, buffer, len);
        crc = cpu_to_be32(crc);
        memcpy(&buffer[len], &crc, sizeof(crc));
        DBG(printf("CRC: Appended CRC-32: 0x%08x\n", be32_to_cpu(crc)));
        return len + sizeof(crc);
    } else {
        uint16_t crc = i82596_calculate_crc16(buffer, len);
        crc = cpu_to_be16(crc);
        memcpy(&buffer[len], &crc, sizeof(crc));
        return len + sizeof(crc);
    }
}

static bool i82596_verify_crc(I82596State *s, const uint8_t *data, size_t len)
{
    if (I596_CRC16_32) {
        /* Verify CRC-32 */
        if (len < 4) {
            return false;
        }
        uint32_t received_crc = be32_to_cpu(*(const uint32_t *)(data + len - 4));
        uint32_t calculated_crc = crc32(~0, data, len - 4);
        bool valid = (received_crc == calculated_crc);
        DBG(printf("CRC: CRC-32 check: received=0x%08x calculated=0x%08x %s\n",
                   received_crc, calculated_crc, valid ? "OK" : "FAIL"));
        return valid;
    } else {
        /* Verify CRC-16 */
        if (len < 2) {
            DBG(printf("CRC: Frame too short for CRC-16 verification\n"));
            return false;
        }
        uint16_t received_crc = be16_to_cpu(*(const uint16_t *)(data + len - 2));
        uint16_t calculated_crc = i82596_calculate_crc16(data, len - 2);
        bool valid = (received_crc == calculated_crc);
        DBG(printf("CRC: CRC-16 check: received=0x%04x calculated=0x%04x %s\n",
                   received_crc, calculated_crc, valid ? "OK" : "FAIL"));
        return valid;
    }
}

static inline size_t i82596_get_crc_size(I82596State *s)
{
    return I596_CRC16_32 ? 4 : 2;
}




// TODO MAKE A UNIFIED FUNCTION FOR UPDATING THE STATISTICS
/* Update TX status and counters in SCB */
/* Update RX frame statistics */
static void __attribute__((unused)) i82596_update_rx_statistics(I82596State *s, bool pkt_completed, bool crc_ok, size_t frame_size)
{
    /* Update frame counters */
    s->total_frames++;

    if (pkt_completed && crc_ok) {
        s->total_good_frames++;
    }

    /* Check for short frames */
    if (frame_size < I596_MIN_FRAME_LEN) {
        s->short_fr_error++;
    }

    /* Update SCB statistics immediately */
    set_uint32(s->scb + 40, s->total_frames);      /* Total frames received */
    set_uint32(s->scb + 44, s->total_good_frames); /* Good frames received */
    set_uint32(s->scb + 26, s->short_fr_error);    /* Short frame errors */

    DBG(printf("RX stats updated: total=%d, good=%d, short_errors=%d\n",
               s->total_frames, s->total_good_frames, s->short_fr_error));
}

static void i82596_update_statistics(I82596State *s, bool is_tx, uint16_t error_flags,
                                     uint16_t collision_count)
{
    if (is_tx) {
        /* TX Statistics Update */

        /* Update collision counters */
        if (collision_count > 0) {
            s->tx_collisions += collision_count;
            s->collision_events++;
            s->total_collisions += collision_count;
            set_uint32(s->scb + 32, s->tx_collisions);
            DBG(printf("TX Stats: Collisions=%d (this frame), total=%d, events=%d\n",
                       collision_count, s->tx_collisions, s->collision_events));
        }

        /* Record errors if present */
        if (error_flags) {
            i82596_record_error(s, error_flags, true);
        }

        /* Count successful transmissions (no fatal errors) */
        if (!(error_flags & (TX_ABORTED_ERRORS | TX_CARRIER_ERRORS))) {
            s->tx_good_frames++;
            set_uint32(s->scb + 36, s->tx_good_frames);
        }

        DBG(printf("TX Stats: good_frames=%d, aborted=%d\n",
                   s->tx_good_frames, s->tx_aborted_errors));

    } else {
        /* RX Statistics Update */

        s->total_frames++;
        set_uint32(s->scb + 40, s->total_frames);

        /* Record errors if present */
        if (error_flags) {
            i82596_record_error(s, error_flags, false);
        } else {
            /* No errors - count as good frame */
            s->total_good_frames++;
            set_uint32(s->scb + 44, s->total_good_frames);
        }

        DBG(printf("RX Stats: total=%d, good=%d, errors=0x%04x\n",
                   s->total_frames, s->total_good_frames, error_flags));
    }
}

/* Bus Throttle Functionality */
static void i82596_bus_throttle_timer(void *opaque)
{
    I82596State *s = opaque;

    DBG(printf("Bus throttle timer fired, current state: %s\n",
               s->throttle_state ? "ON" : "OFF"));

    if (s->throttle_state) {
        /* Currently ON, switch to OFF */
        s->throttle_state = false;
        DBG(printf("Switching bus to OFF state\n"));

        /* Set timer for t_off duration if non-zero */
        if (s->t_off > 0) {
            timer_mod(s->throttle_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->t_off * NANOSECONDS_PER_MICROSECOND);
            DBG(printf("Scheduled OFF period for %d microseconds\n", s->t_off));
        } else {
            /* Zero OFF time means immediately go back to ON */
            s->throttle_state = true;
            DBG(printf("Zero OFF time, immediately switching back to ON\n"));
        }
    } else {
        /* Currently OFF, switch to ON */
        s->throttle_state = true;
        DBG(printf("Switching bus to ON state\n"));

        /* Set timer for t_on duration if non-zero and not infinite */
        if (s->t_on > 0 && s->t_on != 0xFFFF) {
            timer_mod(s->throttle_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->t_on * NANOSECONDS_PER_MICROSECOND);
            DBG(printf("Scheduled ON period for %d microseconds\n", s->t_on));
        } else {
            DBG(printf("Infinite ON time or zero, not scheduling next transition\n"));
        }
    }
}

static void i82596_load_throttle_timers(I82596State *s, bool start_now)
{
    uint16_t previous_t_on = s->t_on;
    uint16_t previous_t_off = s->t_off;

    /* Read T-ON and T-OFF values from SCB */
    s->t_on = get_uint16(s->scb + 36);   /* Offset 16 in SCB for T-ON */
    s->t_off = get_uint16(s->scb + 38);  /* Offset 18 in SCB for T-OFF */

    DBG(printf("Load throttle: T-ON=%d, T-OFF=%d, start=%d\n",
              s->t_on, s->t_off, start_now));

    /* Check if values changed */
    bool values_changed = (s->t_on != previous_t_on || s->t_off != previous_t_off);

    /* Start the timer if requested or if values changed significantly */
    if (start_now || (values_changed && s->throttle_timer)) {
        /* Cancel any pending timer */
        timer_del(s->throttle_timer);

        /* Start with the bus ON */
        s->throttle_state = true;

        /* Schedule the T-ON timer if not infinite */
        if (s->t_on > 0 && s->t_on != 0xFFFF && !I596_FULL_DUPLEX) {
            DBG(printf("Starting throttle timer with T-ON=%d microseconds\n", s->t_on));
            timer_mod(s->throttle_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->t_on * NANOSECONDS_PER_MICROSECOND);
        } else {
            DBG(printf("Not starting throttle timer: infinite T-ON or full duplex\n"));
        }
    }
}

static void i82596_init_dump_area(I82596State *s, uint8_t *buffer)
{
    memset(buffer, 0, DUMP_BUF_SZ);

    printf("This is the dump area function for i82596 QEMU side \n"
            "If you are seeing this message, please contact:\n"
            "Soumyajyotii Sarkar <soumyajyotisarkar23@gmail.com>\n"
            "With the process in which you encountered this issue:\n"
            "This still needs developement so, I will be more than delighted to help you out!\n");

    auto void write_uint16(int offset, uint16_t value) {
        buffer[offset] = value >> 8;
        buffer[offset + 1] = value & 0xFF;
    }

    auto void write_uint32(int offset, uint32_t value) {
        write_uint16(offset, value >> 16);
        write_uint16(offset + 2, value & 0xFFFF);
    }

    /* ----------------- Configuration Bytes ------------------ */
    /* Configure bytes at offset 0x00 - actual config values */
    write_uint16(0x00, (s->config[5] << 8) | s->config[4]);
    write_uint16(0x02, (s->config[3] << 8) | s->config[2]);

    /* Configure bytes at offset 0x04 */
    write_uint16(0x04, (s->config[9] << 8) | s->config[8]);
    write_uint16(0x06, (s->config[7] << 8) | s->config[6]);

    /* Configure bytes at offset 0x08 */
    write_uint16(0x08, (s->config[13] << 8) | s->config[12]);
    write_uint16(0x0A, (s->config[11] << 8) | s->config[10]);

    /* --------------- Individual Address (MAC) --------------- */
    /* Individual address (MAC) at offset 0x0C - first 2 bytes */
    buffer[0x0C] = s->conf.macaddr.a[0];
    buffer[0x0D] = s->conf.macaddr.a[1];

    /* Individual address continued at offset 0x10 - remaining 4 bytes */
    buffer[0x10] = s->conf.macaddr.a[2];
    buffer[0x11] = s->conf.macaddr.a[3];
    buffer[0x12] = s->conf.macaddr.a[4];
    buffer[0x13] = s->conf.macaddr.a[5];

    /* --------------- CRC and Status Values ----------------- */
    /* TX CRC bytes and status at offset 0x14 */
    if (s->last_tx_len > 0) {
        uint32_t tx_crc = crc32(~0, s->tx_buffer, s->last_tx_len);
        write_uint16(0x14, tx_crc & 0xFFFF);
        write_uint16(0x16, tx_crc >> 16);
    }

    /* -------------- Hash Table Values --------------------- */
    /* Hash registers at offset 0x24-0x2C - copy multicast hash table */
    memcpy(&buffer[0x24], s->mult, sizeof(s->mult));

    /* -------------- Status and Counters ------------------ */
    /* CU and RU status at offset 0xB0 */
    buffer[0xB0] = s->cu_status;
    buffer[0xB1] = s->rx_status;

    /* Statistical counters - use tracking variables */
    write_uint32(0xB4, s->crc_err);
    write_uint32(0xB8, s->align_err);
    write_uint32(0xBC, s->resource_err);
    write_uint32(0xC0, s->over_err);

    /* -------------- Monitor Mode Counters ---------------- */
    /* Add monitor mode counters at offsets 0xC4-0xCC */
    write_uint32(0xC4, s->short_fr_error);
    write_uint32(0xC8, s->total_frames);
    write_uint32(0xCC, s->total_good_frames);

    /* ----------------- Flag Array -------------------------- */
    /* Flag array at offset 0xD0 - real device state */
    buffer[0xD0] = I596_PROMISC ? 1 : 0;          /* Promiscuous mode */
    buffer[0xD1] = I596_BC_DISABLE ? 1 : 0;       /* Broadcast disabled */
    buffer[0xD2] = I596_FULL_DUPLEX ? 1 : 0;      /* Full duplex mode */
    buffer[0xD3] = I596_LOOPBACK;                 /* Loopback setting */

    /* Count active multicast addresses */
    uint8_t mc_count = 0;
    for (int i = 0; i < sizeof(s->mult); i++) {
        /* Count bits set in each byte of the multicast mask */
        uint8_t byte = s->mult[i];
        while (byte) {
            if (byte & 0x01) {
                mc_count++;
            }
            byte >>= 1;
        }
    }
    buffer[0xD4] = mc_count;                      /* Multicast address count */
    buffer[0xD5] = I596_NOCRC_INS ? 1 : 0;        /* No CRC insertion */
    buffer[0xD6] = I596_CRC16_32 ? 1 : 0;         /* CRC16 or CRC32 */

    /* ------------- Network and Bus Status ----------------- */
    /* Link status */
    write_uint16(0xD8, s->lnkst);

    /* Monitor mode configuration byte */
    buffer[0xDA] = I596_MONITOR_MODE;

    /* Store collision events counter in monitor mode */
    write_uint32(0xDC, s->collision_events);

    /* ------------- Throttle Timers ----------------------- */
    /* Throttle timers at offset 0x110 */
    write_uint16(0x110, s->t_on);
    write_uint16(0x112, s->t_off);

    /* DIU control register at offset 0x114 - bus state */
    write_uint16(0x114, s->throttle_state ? 0x0001 : 0x0000);

    /* BIU control register at offset 0x120 - system bus mode */
    write_uint16(0x120, s->sysbus);

    /* SCB status word at offset 0x128 */
    write_uint16(0x128, s->scb_status);

    /* Signature indicating dump is complete */
    write_uint32(0, 0xFFFF0000);
}

static void i82596_port_dump(I82596State *s, uint32_t dump_addr)
{
    uint8_t dump_buffer[DUMP_BUF_SZ];

    DBG(printf("i82596: PORT Dump command to address 0x%08x\n", dump_addr));
    i82596_init_dump_area(s, dump_buffer);

    address_space_write(&address_space_memory, dump_addr,
                      MEMTXATTRS_UNSPECIFIED, dump_buffer, sizeof(dump_buffer));

    set_uint32(dump_addr, 0xFFFF0000);
    s->scb_status |= SCB_STATUS_CX;
    s->send_irq = 1;
    DBG(printf("i82596: PORT Dump command completed\n"));
}

static void i82596_command_dump(I82596State *s, uint32_t cmd_addr)
{
    uint32_t dump_addr;
    uint8_t dump_buffer[DUMP_BUF_SZ];
    uint16_t cmd = get_uint16(cmd_addr + 2);
    uint16_t status;

    dump_addr = get_uint32(cmd_addr + 8);

    i82596_init_dump_area(s, dump_buffer);
    address_space_write(&address_space_memory, dump_addr,
                      MEMTXATTRS_UNSPECIFIED, dump_buffer, sizeof(dump_buffer));
    status = STAT_C | STAT_OK;
    set_uint16(cmd_addr, status);
    if (cmd & CMD_INTR) {
        s->scb_status |= SCB_STATUS_CX;
        s->send_irq = 1;
    }
    if (cmd & CMD_SUSP) {
        s->cu_status = CU_SUSPENDED;
        s->scb_status |= SCB_STATUS_CNA;
    }
}

static void i82596_configure(I82596State *s, uint32_t addr)
{
    uint8_t byte_cnt;
    byte_cnt = get_byte(addr + 8) & 0x0f;
    byte_cnt = MAX(byte_cnt, 4);
    byte_cnt = MIN(byte_cnt, sizeof(s->config));
    s->config[2] &= 0x82; /* mask valid bits */
    s->config[2] |= 0x40;
    s->config[7]  &= 0xf7; /* clear zero bit */

    address_space_read(&address_space_memory, addr + 8,
                       MEMTXATTRS_UNSPECIFIED, s->config, byte_cnt);

    if (byte_cnt > 12) {
        bool previous_duplex = I596_FULL_DUPLEX;
        if (previous_duplex != I596_FULL_DUPLEX) {
            DBG(printf("DUPLEX: Mode changed to %s duplex\n",
                       I596_FULL_DUPLEX ? "FULL" : "HALF"));
        }
        s->config[12] &= 0x40; /* Preserve only duplex bit */


        if (byte_cnt > 11) {
            uint8_t monitor_mode = I596_MONITOR_MODE;
            s->config[11] &= ~0xC0; /* Clear bits 6-7 */
            s->config[11] |= (monitor_mode << 6); /* Set monitor mode */

            DBG(printf("MONITOR: Mode set to %d (%s)\n", monitor_mode,
                       monitor_mode == MONITOR_NORMAL ? "NORMAL" :
                       monitor_mode == MONITOR_FILTERED ? "FILTERED" :
                       monitor_mode == MONITOR_ALL ? "ALL" : "DISABLED"));
        }
    }

    if (s->rx_status == RX_READY) {
        timer_mod(s->flush_queue_timer,
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + 100000000); /* 100ms in nanoseconds */
    }

    s->scb_status |= SCB_STATUS_CNA;
    s->config[13] |= 0x3f; /* set ones in byte 13, this is reserved right? TODO CHECK DOCS */
    qemu_set_irq(s->irq, 1);
}

static void i82596_update_scb_irq(I82596State *s, bool trigger)
{
    update_scb_status(s);

    if (trigger) {
        s->send_irq = 1;
        qemu_set_irq(s->irq, 1);
    }
}

static void i82596_update_cu_status(I82596State *s, uint16_t cmd_status, bool generate_interrupt)
{
    /* Update CU state based on command completion */
    if (cmd_status & STAT_C) {
        /* Command completed */
        if (cmd_status & STAT_OK) {
            /* Command completed successfully */
            if (s->cu_status == CU_ACTIVE) {
                /* CU remains active if more commands in queue */
                if (s->cmd_p == I596_NULL) {
                    s->cu_status = CU_IDLE;
                    s->scb_status |= SCB_STATUS_CNA;
                }
            }
        } else {
            /* Command failed or aborted */
            s->cu_status = CU_IDLE;
            s->scb_status |= SCB_STATUS_CNA;
        }

        /* Generate completion interrupt if requested */
        if (generate_interrupt) {
            s->scb_status |= SCB_STATUS_CX;
            i82596_update_scb_irq(s, true);
        }
    }

    update_scb_status(s);
}

/**
 * Update SCB Status
 * Synchronizes device state with SCB status word and statistics counters.
 * This function is called frequently to keep the kernel driver updated.
 */
static void update_scb_status(I82596State *s)
{
    /* Update status word with CU state, RU state, and link status */
    s->scb_status = (s->scb_status & 0xf000)
        | (s->cu_status << 8) | (s->rx_status << 4) | (s->lnkst >> 8);
    set_uint16(s->scb, s->scb_status);

    /* Update TX-related counters */
    set_uint32(s->scb + 28, s->tx_aborted_errors);
    set_uint32(s->scb + 32, s->tx_collisions);
    set_uint32(s->scb + 36, s->tx_good_frames);

    /* Update RX-related counters */
    set_uint32(s->scb + 16, s->crc_err);
    set_uint32(s->scb + 18, s->align_err);
    set_uint32(s->scb + 20, s->resource_err);
    set_uint32(s->scb + 22, s->over_err);
    set_uint32(s->scb + 24, s->rcvdt_err);
    set_uint32(s->scb + 26, s->short_fr_error);
}

static void command_loop(I82596State *s)
{
    DBG(printf("CMD_LOOP: Start - CU=%d cmd_p=0x%08x\n", s->cu_status, s->cmd_p));

    /* Process commands while CU is active and valid command pointer exists */
    while (s->cu_status == CU_ACTIVE && s->cmd_p != I596_NULL && s->cmd_p != 0) {
        uint16_t status = get_uint16(s->cmd_p);

        /* Skip already processed commands */
        if (status & (STAT_C | STAT_B)) {
            uint32_t next = get_uint32(s->cmd_p + 4);
            if (next == 0 || next == s->cmd_p) {
                s->cmd_p = I596_NULL;
                s->cu_status = CU_IDLE;
                s->scb_status |= SCB_STATUS_CNA;
                break;
            }
            s->cmd_p = i82596_translate_address(s, next, false);
            continue;
        }

        /* Mark command as busy */
        set_uint16(s->cmd_p, STAT_B);

        /* Read command and link */
        uint16_t cmd = get_uint16(s->cmd_p + 2);
        uint32_t next_addr = get_uint32(s->cmd_p + 4);

        DBG(printf("CMD_LOOP: Exec cmd=0x%04x type=%d at 0x%08x\n",
                   cmd, cmd & CMD_MASK, s->cmd_p));

        /* Translate next address */
        next_addr = (next_addr == 0) ? I596_NULL :
                    i82596_translate_address(s, next_addr, false);

        /* Execute command based on type */
        switch (cmd & CMD_MASK) {
        case CmdNOp:
            break;
        case CmdSASetup:
            set_individual_address(s, s->cmd_p);
            break;
        case CmdConfigure:
            i82596_configure(s, s->cmd_p);
            break;
        case CmdTDR:
            set_uint32(s->cmd_p + 8, s->lnkst);
            break;
        case CmdTx:
            i82596_xmit(s, s->cmd_p);
            goto skip_status_update;
        case CmdMulticastList:
            set_multicast_list(s, s->cmd_p);
            break;
        case CmdDump:
            i82596_command_dump(s, s->cmd_p);
            break;
        case CmdDiagnose:
            break;
        default:
            DBG(printf("CMD_LOOP: Unknown command %d\n", cmd & CMD_MASK));
            break;
        }

        /* Set command complete status */
        status = get_uint16(s->cmd_p);
        if (!(status & STAT_C)) {
            set_uint16(s->cmd_p, STAT_C | STAT_OK);
        }

skip_status_update:
        /* Handle command flags */
        if (cmd & CMD_INTR) {
            s->scb_status |= SCB_STATUS_CX;
            s->send_irq = 1;
        }

        bool stop = false;

        if (cmd & CMD_SUSP) {
            s->cu_status = CU_SUSPENDED;
            s->scb_status |= SCB_STATUS_CNA;
            stop = true;
            DBG(printf("CMD_LOOP: Suspend\n"));
        }

        if (cmd & CMD_EOL) {
            s->cmd_p = I596_NULL;
            s->cu_status = CU_IDLE;
            s->scb_status |= SCB_STATUS_CNA;
            stop = true;
            DBG(printf("CMD_LOOP: End of list\n"));
        } else if (!stop) {
            /* Advance to next command */
            if (next_addr == 0 || next_addr == I596_NULL || next_addr == s->cmd_p) {
                s->cmd_p = I596_NULL;
                s->cu_status = CU_IDLE;
                s->scb_status |= SCB_STATUS_CNA;
                stop = true;
                DBG(printf("CMD_LOOP: Invalid next\n"));
            } else {
                s->cmd_p = next_addr;
            }
        }

        update_scb_status(s);

        if (stop || s->cu_status != CU_ACTIVE) {
            break;
        }
    }

    /* Final status update */
    update_scb_status(s);

    /* Flush RX queue if ready */
    if (s->rx_status == RX_READY && s->nic) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }

    DBG(printf("CMD_LOOP: End - CU=%d cmd_p=0x%08x\n", s->cu_status, s->cmd_p));
}

static void examine_scb(I82596State *s)
{
    uint16_t command = get_uint16(s->scb + 2);
    uint8_t cuc = (command >> 8) & 0x7;
    uint8_t ruc = (command >> 4) & 0x7;

    DBG(printf("SCB: command=0x%04x CUC=%d RUC=%d\n", command, cuc, ruc));

    /* Clear command word and acknowledge interrupts */
    set_uint16(s->scb + 2, 0);
    s->scb_status &= ~(command & SCB_ACK_MASK);

    /* Process Command Unit (CU) commands */
    switch (cuc) {
    case SCB_CUC_NOP:
        break;

    case SCB_CUC_START: {
        uint32_t cmd_ptr = get_uint32(s->scb + 4);
        s->cmd_p = i82596_translate_address(s, cmd_ptr, false);
        s->cu_status = CU_ACTIVE;
        DBG(printf("SCB: CU Start at 0x%08x\n", s->cmd_p));
        break;
    }

    case SCB_CUC_RESUME:
        if (s->cu_status == CU_SUSPENDED) {
            s->cu_status = CU_ACTIVE;
            DBG(printf("SCB: CU Resume\n"));
        }
        break;

    case SCB_CUC_SUSPEND:
        s->cu_status = CU_SUSPENDED;
        s->scb_status |= SCB_STATUS_CNA;
        DBG(printf("SCB: CU Suspend\n"));
        break;

    case SCB_CUC_ABORT:
        s->cu_status = CU_IDLE;
        s->scb_status |= SCB_STATUS_CNA;
        DBG(printf("SCB: CU Abort\n"));
        break;

    case SCB_CUC_LOAD_THROTTLE: {
        bool external_trigger = (s->sysbus & I82586_MODE);
        i82596_load_throttle_timers(s, !external_trigger);
        DBG(printf("SCB: Load throttle timers (external=%d)\n", external_trigger));
        break;
    }

    case SCB_CUC_LOAD_START:
        i82596_load_throttle_timers(s, true);
        DBG(printf("SCB: Load and start throttle timers\n"));
        break;
    }

    /* Process Receive Unit (RU) commands */
    switch (ruc) {
    case SCB_RUC_NOP:
        break;

    case SCB_RUC_START: {
        uint32_t rfd_log = get_uint32(s->scb + 8);
        hwaddr rfd = i82596_translate_address(s, rfd_log, false);

        DBG(printf("SCB: RU Start - RFA=0x%08x\n", rfd_log));

        if (rfd == 0 || rfd == I596_NULL) {
            s->rx_status = RX_NO_RESOURCES;
            s->scb_status |= SCB_STATUS_RNR;
            DBG(printf("SCB: RU Start failed - invalid RFA\n"));
            break;
        }

        /* Find first usable RFD with valid RBD */
        struct i82596_rx_descriptor test_rfd;
        hwaddr test_rfd_addr = rfd;
        uint32_t test_rfd_log = rfd_log;
        hwaddr first_usable_rfd = 0;
        uint32_t first_usable_rfd_log = 0;
        bool found = false;

        for (int i = 0; i < 10 && test_rfd_addr != 0 && test_rfd_addr != I596_NULL; i++) {
            i82596_rx_desc_read(s, test_rfd_addr, &test_rfd);

            if (test_rfd.command & CMD_FLEX) {
                hwaddr rbd = i82596_translate_address(s, test_rfd.rbd_addr, false);
                if (rbd != I596_NULL && rbd != 0) {
                    first_usable_rfd = test_rfd_addr;
                    first_usable_rfd_log = test_rfd_log;
                    found = true;
                    DBG(printf("SCB: Found usable RFD at 0x%08x\n", test_rfd_log));
                    break;
                }
            }

            test_rfd_log = test_rfd.link;
            test_rfd_addr = i82596_translate_address(s, test_rfd.link, false);
        }

        if (found) {
            s->current_rx_desc = first_usable_rfd;
            s->last_good_rfa = first_usable_rfd_log;
            s->rx_status = RX_READY;

            /* Update RFA pointer if needed */
            if (first_usable_rfd != rfd) {
                set_uint32(s->scb + 8, first_usable_rfd_log);
            }

            qemu_flush_queued_packets(qemu_get_queue(s->nic));
            DBG(printf("SCB: RU Ready - RFD=0x%08x\n", first_usable_rfd_log));
        } else {
            s->rx_status = RX_NO_RESOURCES;
            s->scb_status |= SCB_STATUS_RNR;
            DBG(printf("SCB: RU Start failed - no usable RFD\n"));
        }
        break;
    }

    case SCB_RUC_RESUME:
        if (s->rx_status == RX_SUSPENDED) {
            s->rx_status = RX_READY;
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
            DBG(printf("SCB: RU Resume\n"));
        }
        break;

    case SCB_RUC_SUSPEND:
        s->rx_status = RX_SUSPENDED;
        s->scb_status |= SCB_STATUS_RNR;
        DBG(printf("SCB: RU Suspend\n"));
        break;

    case SCB_RUC_ABORT:
        s->rx_status = RX_IDLE;
        s->scb_status |= SCB_STATUS_RNR;
        DBG(printf("SCB: RU Abort\n"));
        break;
    }

    /* Handle reset command */
    if (command & 0x80) {
        DBG(printf("SCB: Reset command\n"));
        i82596_s_reset(s);
        return;
    }
    /* Execute command loop if CU is active */
    if (s->cu_status == CU_ACTIVE) {
        if (s->cmd_p == I596_NULL) {
            s->cmd_p = get_uint32(s->scb + 4);
        }
        update_scb_status(s);
        command_loop(s);
    } else {
        update_scb_status(s);
    }
}

static void signal_ca(I82596State *s)
{
    if (s->scp) {
        /* CA after reset -> initialize with new SCP */
        s->sysbus = get_byte(s->scp + 3);
        s->mode = (s->sysbus >> 1) & 0x03;  /* Extract mode bits (m0, m1) */
        s->iscp = get_uint32(s->scp + 8);

        s->scb = get_uint32(s->iscp + 4);

        s->scb_base = (s->mode == I82596_MODE_LINEAR) ? 0 : get_uint32(s->iscp + 8);
        s->scb = i82596_translate_address(s, s->scb, false);
        DBG(printf("CA: Initialization - SCB=0x%08x, mode=%d, base=0x%08x\n",
                   s->scb, s->mode, s->scb_base));

        /* Complete initialization sequence:
         * - Clear BUSY flag in ISCP
         * - Set CX and CNA in SCB status
         * - Clear SCB command word
         * - Signal interrupt
         */
        set_byte(s->iscp + 1, 0);
        s->scb_status |= SCB_STATUS_CX | SCB_STATUS_CNA;
        update_scb_status(s);
        set_uint16(s->scb + 2, 0);
        s->scp = 0;
        qemu_set_irq(s->irq, 1);
        return;
    }

    if (s->ca_active) {
        s->ca++;
        return;
    }
    s->ca_active = 1;
    s->ca++;

    while (s->ca > 0) {
        s->ca--;
        examine_scb(s);
    }

    s->ca_active = 0;

    if (s->send_irq) {
        s->send_irq = 0;
        qemu_set_irq(s->irq, 1);
    }
}

static void i82596_self_test(I82596State *s, uint32_t val)
{
    /* The documentation for the self test is a bit unclear,
     * we are currently doing this and it seems to work.
     */
    set_uint32(val, 0xFFC00000);
    set_uint32(val + 4, 0);

    s->scb_status &= ~SCB_STATUS_CNA;
    s->scb_status |= SCB_STATUS_CNA;

    qemu_set_irq(s->irq, 1);
    update_scb_status(s);
}

/* LASI specific interfaces */
static uint32_t bit_align_16(uint32_t val)
{
    return val & ~0x0f;
}

uint32_t i82596_ioport_readw(void *opaque, uint32_t addr)
{
    return -1;
}

void i82596_ioport_writew(void *opaque, uint32_t addr, uint32_t val)
{
    I82596State *s = opaque;
    DBG(printf("i82596_ioport_writew addr=0x%08x val=0x%04x\n", addr, val));
    switch (addr) {
    case PORT_RESET: /* Reset */
        i82596_s_reset(s);
        break;
    case PORT_SELFTEST:
        val = bit_align_16(val);
        i82596_self_test(s, val);
        break;
    case PORT_ALTSCP:
        s->scp = bit_align_16(val);
        break;
    case PORT_ALTDUMP:
        printf("Dumping statistics to memory at %08x\n", val);
        i82596_port_dump(s, bit_align_16(val));
        break;
    case PORT_CA:
        signal_ca(s);
        break;
    }
}

void i82596_poll(NetClientState *nc, bool enable)
{
    I82596State *s = qemu_get_nic_opaque(nc);

    if (!enable) {
        return;
    }

    if (s->send_irq) {
        qemu_set_irq(s->irq, 1);
    }

    if (s->rx_status == RX_NO_RESOURCES) {
        if (s->cmd_p != I596_NULL) {
            s->rx_status = RX_READY;
            update_scb_status(s);
        }
    }

    if (s->cu_status == CU_ACTIVE && s->cmd_p != I596_NULL) {
        examine_scb(s);
    }
    qemu_set_irq(s->irq, 0);
}

const VMStateDescription vmstate_i82596 = {
    .name = "i82596",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(mode, I82596State),
        VMSTATE_UINT16(t_on, I82596State),
        VMSTATE_UINT16(t_off, I82596State),
        VMSTATE_BOOL(throttle_state, I82596State),
        VMSTATE_UINT32(iscp, I82596State),
        VMSTATE_UINT8(sysbus, I82596State),
        VMSTATE_UINT32(scb, I82596State),
        VMSTATE_UINT32(scb_base, I82596State),
        VMSTATE_UINT16(scb_status, I82596State),
        VMSTATE_UINT8(cu_status, I82596State),
        VMSTATE_UINT8(rx_status, I82596State),
        VMSTATE_UINT16(lnkst, I82596State),
        VMSTATE_UINT32(cmd_p, I82596State),
        VMSTATE_INT32(ca, I82596State),
        VMSTATE_INT32(ca_active, I82596State),
        VMSTATE_INT32(send_irq, I82596State),
        VMSTATE_BUFFER(mult, I82596State),
        VMSTATE_BUFFER(config, I82596State),
        VMSTATE_BUFFER(tx_buffer, I82596State),
        VMSTATE_END_OF_LIST()
    }
};

static void i82596_flush_queue_timer(void *opaque)
{
    I82596State *s = opaque;
    DBG(printf("RX: Timer callback fired! rx_status=%d\n", s->rx_status));
    if (s->rx_status == RX_READY) {
        DBG(printf("RX: Calling qemu_flush_queued_packets to retry queued packets\n"));
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

void i82596_common_init(DeviceState *dev, I82596State *s, NetClientInfo *info)
{
    if (s->conf.macaddr.a[0] == 0) {
        qemu_macaddr_default_if_unset(&s->conf.macaddr);
    }
    s->nic = qemu_new_nic(info, &s->conf, object_get_typename(OBJECT(dev)),
                dev->id, &dev->mem_reentrancy_guard, s);
    qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);

    if (USE_TIMER) {
        /* Added throttle & flush timers */
        s->flush_queue_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    i82596_flush_queue_timer, s);
        s->throttle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    i82596_bus_throttle_timer, s);
    }

    s->lnkst = 0x8000; /* initial link state: up */
}
