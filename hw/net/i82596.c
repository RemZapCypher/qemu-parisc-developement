/*
 * QEMU Intel i82596 (Apricot) emulation
 *
 * Copyright (c) 2019 Helge Deller <deller@gmx.de>
 * Additional functionality added by:
 * Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 * This software was written to be compatible with the specification:
 * https://parisc.docs.kernel.org/en/latest/_downloads/96672be0650d9fc046bbcea40b92482f/82596CA.pdf
 *
 * INDEX:
 * 1.  Reset
 * 2.  Misc Functionality Functions
 * 2.1 Address Translation
 * 2.2 Individual Address
 * 2.3 Multicast Address List
 * 2.4 Link Status
 * 2.5 CSMA/CD functions
 * 2.6 Unified CRC Calculation
 * 2.7 Unified Statistics Update
 * 3.  Transmit functions
 * 4.  Bus Throttling Timer
 * 5.  Dump functions
 * 6.  Configure
 * 7.  Command Loop
 * 8.  Examine SCB
 * 9.  Channel attention (CA)
 * 10. LASI interface
 * 11. Receive Helper functions
 * 12. Receive functions
 * 13. Polling functions
 * 14. QOM and interface functions
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

static int rx_copybreak = 100;

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
    s->lnkst = 0x8000; /* initial link state: up */
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
    qemu_set_irq(s->irq, 1); /* Why not 0 ???*/
    
    s->t_on = 0xFFFF;
    s->t_off = 0;
    s->throttle_state = true;

    if (s->throttle_timer) {
        timer_del(s->throttle_timer);
        s->throttle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       i82596_bus_throttle_timer, s);
        /* Only start timer in half-duplex mode with non-infinite T-ON */
        if (!I596_FULL_DUPLEX && s->t_on != 0xFFFF) {
            timer_mod(s->throttle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     s->t_on * NANOSECONDS_PER_MICROSECOND);
        }
    }
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
        DBG(printf("Add multicast entry " MAC_FMT "\n",
                    MAC_ARG(multicast_addr)));
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


/* Note: Attempt at implementing a unified CRC line ie both the TX and RX functions would make use of the same
 * CRC Subsystem to determine the CRC for the packets this is because of the similarities
 * in the CRC calculation for both TX and RX.
 */
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
    if (I596_CRC16_32) {
        /* Use CRC-32 */
        uint32_t crc = crc32(~0, buffer, len);
        crc = cpu_to_be32(crc);
        memcpy(&buffer[len], &crc, sizeof(crc));
        return len + sizeof(crc);
    } else {
        /* Use CRC-16 */
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
        if (len < 4) return false;
        uint32_t received_crc = be32_to_cpu(*(uint32_t *)(data + len - 4));
        uint32_t calculated_crc = crc32(~0, data, len - 4);
        return received_crc == calculated_crc;
    } else {
        /* Verify CRC-16 */
        if (len < 2) return false;
        uint16_t received_crc = be16_to_cpu(*(uint16_t *)(data + len - 2));
        uint16_t calculated_crc = i82596_calculate_crc16(data, len - 2);
        return received_crc == calculated_crc;
    }
}




// TODO MAKE A UNIFIED FUNCTION FOR UPDATING THE STATISTICS
/* Update TX status and counters in SCB */
static void i82596_update_tx_counters(I82596State *s, uint16_t tx_status)
{
    if (tx_status & TX_COLLISIONS) {
        s->tx_collisions++;
        s->collision_events++;
    }

    if (tx_status & TX_ABORTED_ERRORS) {
        s->tx_aborted_errors++;
        set_uint32(s->scb + 28, s->tx_aborted_errors);  /* Update SCB aborted errors counter */
    }

    if (!(tx_status & (TX_ABORTED_ERRORS | TX_CARRIER_ERRORS))) {
        s->tx_good_frames++;
    }

    /* Update collision counter in SCB */
    set_uint32(s->scb + 32, s->tx_collisions);

    /* Update good frames counter in SCB */
    set_uint32(s->scb + 36, s->tx_good_frames);

    DBG(printf("TX counters updated: collisions=%d, aborted=%d, good=%d\n",
               s->tx_collisions, s->tx_aborted_errors, s->tx_good_frames));
}

/* Update RX frame statistics */
static void i82596_update_rx_statistics(I82596State *s, bool pkt_completed, bool crc_ok, size_t frame_size)
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

static void i82596_xmit(I82596State *s, uint32_t addr)
{
    uint32_t tdb_p; /* Transmit Buffer Descriptor */
    uint16_t cmd, tx_status = 0;
    int insert_crc;
    int retry_count = 0;
    bool medium_available = false;
    bool transmission_success = true;
    uint32_t total_len = 0;
    uint8_t *tx_buffer_ptr = s->tx_buffer;
    size_t buffer_pos = 0;
    bool simplified_mode = false;

    /* Read command and TBD pointer */
    tdb_p = get_uint32(addr + 8);
    cmd = get_uint16(addr + 2);
    insert_crc = (I596_NOCRC_INS == 0) && ((cmd & 0x10) == 0) && !I596_LOOPBACK;

    /* Check if this is simplified or flexible mode */
    simplified_mode = !(cmd & CMD_FLEX);

    DBG(printf("TX: Starting transmission, TBD=0x%08x, cmd=0x%04x, mode=%s\n",
               tdb_p, cmd, simplified_mode ? "simplified" : "flexible"));

    /* CSMA/CD: Only perform medium polling in half-duplex mode */
    if (!I596_FULL_DUPLEX && !I596_LOOPBACK) {
        /* Try transmitting with CSMA/CD backoff and retry */
        while (retry_count < CSMA_MAX_RETRIES) {
            medium_available = i82596_check_medium_status(s);

            if (medium_available) {
                break;
            }

            /* Medium busy or collision, perform backoff */
            int backoff_time = i82596_csma_backoff(s, retry_count);

            /* Wait for the backoff period (simulated) */
            if (backoff_time > 0) {
                DBG(printf("CSMA/CD: Waiting for %d microseconds\n", backoff_time));
                /* In real hardware this would be a delay, in emulation we just record it */
            }

            retry_count++;
            s->total_collisions++;
        }

        /* Check if we exceeded max retries */
        if (retry_count >= CSMA_MAX_RETRIES) {
            DBG(printf("CSMA/CD: Excessive collisions, transmission aborted\n"));
            tx_status |= TX_ABORTED_ERRORS;
            transmission_success = false;
        }

        /* Record collision count in status */
        if (retry_count > 0) {
            tx_status |= TX_COLLISIONS;
            s->collision_events++;
            DBG(printf("CSMA/CD: Transmission proceeded after %d retries\n", retry_count));
        }
    }

    /* Process TBD chain - handle both simplified and flexible modes */
    if (simplified_mode) {
        /* Simplified mode: Data is in the command block itself */
        uint16_t frame_len = get_uint16(addr + 12);  /* Frame length from command */
        uint32_t data_addr = addr + 16;  /* Data starts after command header */

        DBG(printf("TX: Simplified mode, frame_len=%d\n", frame_len));

        if (frame_len > 0 && frame_len <= PKT_BUF_SZ - 8) {
            address_space_read(&address_space_memory, data_addr,
                              MEMTXATTRS_UNSPECIFIED, tx_buffer_ptr, frame_len);
            total_len = frame_len;
        } else {
            DBG(printf("TX: Invalid frame length in simplified mode: %d\n", frame_len));
            tx_status |= TX_ABORTED_ERRORS;
            transmission_success = false;
        }
    } else {
        /* Flexible mode: Process TBD chain */
        while (tdb_p != I596_NULL && transmission_success) {
            uint16_t tbd_size, tbd_len;
            uint32_t tbd_addr;

            /* Translate TBD address */
            tdb_p = i82596_translate_address(s, tdb_p, false);
            if (tdb_p == 0 || tdb_p == I596_NULL) {
                DBG(printf("TX: Invalid TBD address\n"));
                tx_status |= TX_ABORTED_ERRORS;
                transmission_success = false;
                break;
            }

            /* Read TBD fields */
            tbd_size = get_uint16(tdb_p);
            tbd_len = tbd_size & SIZE_MASK;
            tbd_addr = get_uint32(tdb_p + 8);
            tbd_addr = i82596_translate_address(s, tbd_addr, true);

            DBG(printf("TX: Processing TBD 0x%08x, size=0x%04x, len=%d, addr=0x%08x\n",
                       tdb_p, tbd_size, tbd_len, tbd_addr));

            /* Check buffer space */
            if (buffer_pos + tbd_len > PKT_BUF_SZ - 8) {
                DBG(printf("TX: Buffer overflow, truncating frame\n"));
                tx_status |= TX_ABORTED_ERRORS;
                transmission_success = false;
                break;
            }

            /* Copy data from TBD buffer */
            if (tbd_len > 0 && tbd_addr != 0 && tbd_addr != I596_NULL) {
                address_space_read(&address_space_memory, tbd_addr,
                                  MEMTXATTRS_UNSPECIFIED,
                                  tx_buffer_ptr + buffer_pos, tbd_len);
                buffer_pos += tbd_len;
                total_len += tbd_len;
            }

            /* Check for end of frame */
            if (tbd_size & I596_EOF) {
                DBG(printf("TX: End of frame marker found\n"));
                break;
            }

            /* Get next TBD */
            tdb_p = get_uint32(tdb_p + 4);
        }
    }

    /* Process the frame if transmission should proceed */
    if (transmission_success && total_len > 0) {
        /* Insert source MAC address if required */
        if (I596_NO_SRC_ADD_IN == 0 && total_len >= ETH_ALEN * 2) {
            memcpy(&tx_buffer_ptr[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
        }

        /* Add padding if frame is too short */
        if (I596_PADDING && total_len < I596_MIN_FRAME_LEN) {
            size_t pad_len = I596_MIN_FRAME_LEN - total_len;
            memset(tx_buffer_ptr + total_len, 0, pad_len);
            total_len = I596_MIN_FRAME_LEN;
            DBG(printf("TX: Added %zu bytes of padding\n", pad_len));
        }

        /* Calculate and append CRC if required */
        if (insert_crc) {
            total_len = i82596_append_crc(s, tx_buffer_ptr, total_len);
            DBG(printf("TX: CRC appended, total length now %d\n", (int)total_len));
        }

        /* Store transmission length for statistics */
        s->last_tx_len = total_len;

        DBG(PRINT_PKTHDR("TX Send", tx_buffer_ptr));
        DBG(printf("TX: Sending %d bytes (crc_inserted=%d)\n", (int)total_len, insert_crc));

        /* Transmit the frame */
        switch (I596_LOOPBACK) {
        case 0:     /* No loopback, send packet */
            if (s->nic) {
                qemu_send_packet_raw(qemu_get_queue(s->nic), tx_buffer_ptr, total_len);
            }
            break;
        default:    /* Loopback mode */
            i82596_receive(qemu_get_queue(s->nic), tx_buffer_ptr, total_len);
            break;
        }

        /* Update successful transmission counters */
        if (!(tx_status & TX_ABORTED_ERRORS)) {
            s->tx_good_frames++;
        }
    }

    /* Update command status */
    uint16_t cmd_status = STAT_C | STAT_OK;  /* Command complete and OK */
    if (!transmission_success) {
        cmd_status &= ~STAT_OK;  /* Clear OK bit on failure */
        cmd_status |= STAT_A;    /* Set abort bit */
    }

    /* Write status back to command block */
    set_uint16(addr, cmd_status);
    if (tx_status) {
        set_uint16(addr + 14, tx_status);  /* Write detailed TX status */
    }

    /* Update counters and CU status */
    i82596_update_tx_counters(s, tx_status);

    /* Generate interrupt if requested */
    bool generate_interrupt = (cmd & CMD_INTR) != 0;
    i82596_update_cu_status(s, cmd_status, generate_interrupt);

    DBG(printf("TX: Transmission complete, status=0x%04x, tx_status=0x%04x\n",
               cmd_status, tx_status));
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
        DBG(printf("DUPLEX: Current mode is %s\n", I596_FULL_DUPLEX ? "FULL" : "HALF"));
        //TODO ADD MONITOR MODE BITS HERE

    }

    if (s->rx_status == RX_READY) {
        timer_mod(s->flush_queue_timer,
            qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }

    s->scb_status |= SCB_STATUS_CNA;
    s->config[13] |= 0x3f; /* set ones in byte 13, this is reserved right? TODO CHECK DOCS */
    qemu_set_irq(s->irq, 1);
}

static void update_scb_status(I82596State *s)
{
    s->scb_status = (s->scb_status & 0xf000)
        | (s->cu_status << 8) | (s->rx_status << 4) | (s->lnkst >> 8);
    set_uint16(s->scb, s->scb_status);

    /* Note:
     * When we are updating the SCB we update this values keeps
     * the values in check in the kernel api and also updates frequently
    */
   
   /* Update TX-related counters */
   set_uint32(s->scb + 28, s->tx_aborted_errors);  /* TX aborted errors */
   set_uint32(s->scb + 32, s->tx_collisions);      /* Total collisions */
   set_uint32(s->scb + 36, s->tx_good_frames);     /* Successfully transmitted frames */
   
   /* Update RX-related counters */
   set_uint32(s->scb + 16, s->crc_err);        /* CRC error counter */
   set_uint32(s->scb + 18, s->align_err);      /* Alignment error counter */
   set_uint32(s->scb + 20, s->resource_err);   /* Resource error counter */
   set_uint32(s->scb + 22, s->over_err);       /* Overrun error counter */
   set_uint32(s->scb + 24, s->rcvdt_err);      /* Receive data timeout counter */
   set_uint32(s->scb + 26, s->short_fr_error); /* Short frame error counter */
    //TODO ADD THE REST OF THE TX COUNTERS
}

static void command_loop(I82596State *s)
{
    uint16_t cmd, status;
    uint32_t next_cmd_addr;

    DBG(printf("STARTING COMMAND LOOP cmd_p=%08x\n", s->cmd_p));

    while (s->cmd_p != I596_NULL && s->cmd_p != 0 && s->cu_status == CU_ACTIVE) {
        /* Check status != BUSY in progress or completed */
        status = get_uint16(s->cmd_p);
        if (status & (STAT_C | STAT_B)) {
            /* Command already busy or complete, move to next command */
            next_cmd_addr = get_uint32(s->cmd_p + 4);
            if (next_cmd_addr == 0 || next_cmd_addr == s->cmd_p) {
                s->cmd_p = I596_NULL;
                s->cu_status = CU_IDLE;
                s->scb_status |= SCB_STATUS_CNA;
                break;
            }
            s->cmd_p = next_cmd_addr;
            continue;
        }
        /* Set status to BUSY */
        status = STAT_B;
        set_uint16(s->cmd_p, status);
        /* Get command word */
        cmd = get_uint16(s->cmd_p + 2);
        DBG(printf("Running command %04x at %08x\n", cmd, s->cmd_p));
        next_cmd_addr = get_uint32(s->cmd_p + 4);
        if (next_cmd_addr == 0) {
            next_cmd_addr = I596_NULL;
        } else {
            next_cmd_addr = i82596_translate_address(s, next_cmd_addr, false);
        }
        /* Execute command based on type */
        switch (cmd & CMD_MASK) {
        case CmdNOp:
            /* No operation */
            break;
        case CmdSASetup:
            set_individual_address(s, s->cmd_p);
            break;
        case CmdConfigure:
            i82596_configure(s, s->cmd_p);
            break;
        case CmdTDR:
            /* get signal LINK */
            set_uint32(s->cmd_p + 8, s->lnkst);
            break;
        case CmdTx:
            i82596_xmit(s, s->cmd_p);
            break;
        case CmdMulticastList:
            set_multicast_list(s, s->cmd_p);
            break;
        case CmdDump:
            printf("Dumped statistics to memory at %08x\n", s->cmd_p + 8);
            i82596_command_dump(s, s->cmd_p);
            break;
        case CmdDiagnose:
            printf("Command Diagnose not implemented\n");
            break;
        }

        //TODO CHECK THIS SECTION AND POLISH IT
        bool end_processing = false;
        status = STAT_C | STAT_OK;
        set_uint16(s->cmd_p, status);

        /* Interrupt after doing cmd? */
        if (cmd & CMD_INTR) {
            s->scb_status |= SCB_STATUS_CX;
            s->send_irq = 1;
        } else {
            s->scb_status &= ~SCB_STATUS_CX;
        }

        /* Suspend after doing cmd? */
        if (cmd & CMD_SUSP) {
            s->cu_status = CU_SUSPENDED;
            s->scb_status |= SCB_STATUS_CNA;
            end_processing = true;
        }

        /* End of list? */
        if (cmd & CMD_EOL) {
            s->cmd_p = I596_NULL;
            s->cu_status = CU_IDLE;
            s->scb_status |= SCB_STATUS_CNA;
            end_processing = true;
        } else {
            /* Move to next command */
            if (next_cmd_addr == s->cmd_p) {
                s->cmd_p = I596_NULL;
                s->cu_status = CU_IDLE;
                s->scb_status |= SCB_STATUS_CNA;
                end_processing = true;
            } else {
                s->cmd_p = next_cmd_addr;
            }
        }
        update_scb_status(s);
        if (end_processing || s->cu_status != CU_ACTIVE) {
            break;
        }
    }

    update_scb_status(s);

    if (s->rx_status == RX_READY) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
    }
}

static void schedule_packet_processing(I82596State *s, uint32_t rfd_p)
{
    if (s->rx_status == RX_READY) {
        timer_mod(s->flush_queue_timer,
                 qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
    }
}

static void examine_scb(I82596State *s)
{
    uint16_t command, cuc, ruc;

    command = get_uint16(s->scb + 2);   /* Get the SCB command word */
    cuc = (command >> 8) & 0x7;         /* Command Unit Command */
    ruc = (command >> 4) & 0x7;         /* Receive Unit Command */
    DBG(printf("MAIN COMMAND %04x  cuc %02x ruc %02x\n", command, cuc, ruc));

    set_uint16(s->scb + 2, 0);
    s->scb_status &= ~(command & SCB_ACK_MASK);
    switch (cuc) {
    case SCB_CUC_NOP:
        /* No operation */
        break;
    case SCB_CUC_START:
        /* Start Command Unit */
        s->cu_status = CU_ACTIVE;
        uint32_t cmd_ptr = get_uint32(s->scb + 4);
        s->cmd_p = i82596_translate_address(s, cmd_ptr, false);
        break;
    case SCB_CUC_RESUME:
        if (s->cu_status != CU_ACTIVE) {
            s->cu_status = CU_ACTIVE;
        }
        break;
    case SCB_CUC_SUSPEND:
        s->cu_status = CU_SUSPENDED;
        s->scb_status |= SCB_STATUS_CNA;
        break;
    case SCB_CUC_ABORT:
        s->cu_status = CU_IDLE;
        s->scb_status |= SCB_STATUS_CNA;
        break;
    case SCB_CUC_LOAD_THROTTLE:
            bool external_trigger = (s->sysbus & I82586_MODE);
            i82596_load_throttle_timers(s, !external_trigger);
        break;
    case SCB_CUC_LOAD_START:
            i82596_load_throttle_timers(s, true);
        break;
    }


    switch (ruc) {
    case SCB_RUC_NOP:
        /* No operation */
        break;
    case SCB_RUC_START:
        s->rx_status = RX_READY;
        uint32_t rfd = get_uint32(s->scb + 8);
        rfd = i82596_translate_address(s, rfd, false);
        if (rfd == 0 || rfd == I596_NULL) {
            s->rx_status = RX_NO_RESOURCES;
            s->scb_status |= SCB_STATUS_RNR;
        } else {
            schedule_packet_processing(s, rfd);
        }
        break;
    case SCB_RUC_RESUME:
        /* Resume Receive Unit */
        if (s->rx_status == RX_SUSPENDED) {
            s->rx_status = RX_READY;
            timer_mod(s->flush_queue_timer,
                     qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
        }
        break;
    case SCB_RUC_SUSPEND:
        s->rx_status = RX_SUSPENDED;
        s->scb_status |= SCB_STATUS_RNR;
        timer_del(s->flush_queue_timer);
        break;
    case SCB_RUC_ABORT:
        s->rx_status = RX_IDLE;
        s->scb_status |= SCB_STATUS_RNR;
        timer_del(s->flush_queue_timer);
        break;
    }

    //TODO AM I DOING THIS AGAIN???
    set_uint32(s->scb + 12, s->crc_err);
    set_uint32(s->scb + 16, s->align_err);
    set_uint32(s->scb + 20, s->resource_err);
    set_uint32(s->scb + 24, s->over_err);
    set_uint32(s->scb + 28, s->rcvdt_err);
    set_uint32(s->scb + 32, s->short_fr_error);

    if (command & 0x80) {
        i82596_s_reset(s);
    } else {
        /* Execute commands only if CU is active */
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
}

static void signal_ca(I82596State *s)
{
    /* trace_i82596_channel_attention(s); */
    s->iscp = 0;
    if (s->scp) {
        /* CA after reset -> do init with new scp. */
        s->sysbus = get_byte(s->scp + 3); /* big endian */
        s->mode = (s->sysbus >> 1) & 0x03; /* m0 & m1 */
        s->iscp = get_uint32(s->scp + 8);

        /* Get SCB address */
        s->scb = get_uint32(s->iscp + 4);

        /* In segmented modes, we need to get the base address as well */
        if (!(s->mode == I82596_MODE_LINEAR)){
            s->scb_base = get_uint32(s->iscp + 8); /* Get SCB base */
        } else {
            s->scb_base = 0;
        }

        s->scb = i82596_translate_address(s, s->scb, false);
        DBG(printf("Translated SCB address: 0x%08x\n", s->scb));

        /* Clear BUSY flag in ISCP, set CX and CNR to equal 1 in the SCB, clears the SCB command word,
         * sends an interrupt to the CPU, and awaits another Channel Attention signal. */
        set_byte(s->iscp + 1, 0);
        s->scb_status |= SCB_STATUS_CX | SCB_STATUS_CNA;
        update_scb_status(s);
        set_uint16(s->scb + 2, 0);
        s->scp = 0;
        qemu_set_irq(s->irq, 1);
        return;
    }

    s->ca++;    /* count ca() */
    if (!s->ca_active) {
        s->ca_active = 1;
        while (s->ca)   {
            examine_scb(s);
            s->ca--;
        }
        s->ca_active = 0;
    }

    if (s->send_irq) {
        s->send_irq = 0;
        qemu_set_irq(s->irq, 1);
    }
}

static uint32_t bit_align_16(uint32_t val)
{
    return val & ~0x0f;
}

uint32_t i82596_ioport_readw(void *opaque, uint32_t addr)
{
    return -1;
}


static void i82596_self_test(I82596State *s, uint32_t val)
{
    DBG(printf("Performing Self test\n"));
    set_uint32(val, 0xFFC00000);
    set_uint32(val + 4, 0);

    s->scb_status &= ~SCB_STATUS_CNA;
    s->scb_status |= SCB_STATUS_CNA;

    qemu_set_irq(s->irq, 1);
    update_scb_status(s);
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

static void i82596_record_error(I82596State *s, uint16_t error_type)
{
    /* Update local counters first */
    switch (error_type) {
    case RX_CRC_ERRORS:
        s->crc_err++;
        break;
    case RX_LENGTH_ERRORS:
    case RX_LENGTH_ERRORS_ALT:
        s->align_err++;
        break;
    case RFD_STATUS_NOBUFS:
        s->resource_err++;
        break;
    case RX_OVER_ERRORS:
    case RX_FIFO_ERRORS:
        s->over_err++;
        break;
    case RFD_STATUS_TRUNC:
        s->short_fr_error++;
        break;
    default:
        DBG(printf("Unknown error type: 0x%04x\n", error_type));
        return;
    }

    /* Update SCB counters immediately */
    set_uint32(s->scb + 16, s->crc_err);        /* CRC error counter */
    set_uint32(s->scb + 18, s->align_err);      /* Alignment error counter */
    set_uint32(s->scb + 20, s->resource_err);   /* Resource error counter */
    set_uint32(s->scb + 22, s->over_err);       /* Overrun error counter */

    DBG(printf("Error recorded: type=0x%04x, crc=%d, align=%d, resource=%d, over=%d\n",
               error_type, s->crc_err, s->align_err, s->resource_err, s->over_err));
}

bool i82596_can_receive(NetClientState *nc)
{
    I82596State *s = qemu_get_nic_opaque(nc);

    /* In full duplex, we can receive during transmission */
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

ssize_t i82596_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    I82596State *s = qemu_get_nic_opaque(nc);
    uint32_t rfa_pointer, rfd_addr, next_rfd, rbd_addr;
    uint16_t status = 0, command;
    uint16_t is_broadcast = 0;
    bool packet_completed = true;
    bool simplified_mode = false;
    size_t target_size = size;
    size_t bytes_copied = 0;
    size_t crc_size = I596_CRC16_32 ? 4 : 2;
    uint8_t *packet_data = (uint8_t *)buf;
    bool crc_valid = true;

    DBG(printf("====== i82596_receive() START ======\n"));
    DBG(PRINT_PKTHDR("[RX] packet", buf));

    /* Validate receive state and link status */
    if (!I596_FULL_DUPLEX && !s->throttle_state) {
        DBG(printf("RX: Rejected - throttle off in half duplex\n"));
        return size; /* Pretend we received it */
    }

    if (!i82596_validate_receive_state(s, &size)) {
        DBG(printf("ERROR: Invalid RX state, rejecting packet\n"));
        return -1;
    }

    /* Apply packet filtering */
    bool passes_filter = i82596_check_packet_filter(s, buf, &is_broadcast);

    /* Handle monitor mode */
    if (!i82596_monitor(s, buf, size, passes_filter)) {
        DBG(printf("Packet handled by monitor mode, not processing further\n"));
        return size;
    }

    /* Only continue normal processing if packet passes filter */
    if (!passes_filter) {
        DBG(printf("Packet rejected by filter\n"));
        return size;
    }

    /* Handle CRC verification and processing */
    if (I596_LOOPBACK && size > crc_size) {
        /* Only verify CRC in loopback mode where we know CRC is present */
        crc_valid = i82596_verify_crc(s, buf, size);
        if (!crc_valid) {
            DBG(printf("RX: CRC verification failed in loopback\n"));
            status |= RX_CRC_ERRORS;
            i82596_record_error(s, RX_CRC_ERRORS);
            s->crc_err++;

            if (!SAVE_BAD_FRAMES) {
                return size;  /* Discard frame with bad CRC */
            }
        }
        target_size = size - crc_size;  /* Remove existing CRC */
    } else {
        /* For normal network reception, assume CRC is good (handled by physical layer) */
        crc_valid = true;
        target_size = size;  /* No CRC to remove */
        DBG(printf("RX: Normal network reception, assuming good CRC\n"));
    }

    /* Get RFD pointer and validate */
    rfa_pointer = get_uint32(s->scb + 8);
    rfd_addr = rfa_pointer;
    rfd_addr = i82596_translate_address(s, rfd_addr, false);

    if (rfd_addr == 0 || rfd_addr == I596_NULL) {
        DBG(printf("No RFD available, setting NO_RESOURCES state\n"));
        i82596_update_rx_state(s, RX_NO_RESOURCES);
        s->resource_err++;
        return -1;
    }

    /* Read RFD command and determine mode */
    command = get_uint16(rfd_addr + 2);
    simplified_mode = !(command & CMD_FLEX);

    DBG(printf("RX: Processing in %s mode, RFD=0x%08x, cmd=0x%04x\n",
               simplified_mode ? "simplified" : "flexible", rfd_addr, command));

    /* Check if RFD is busy */
    uint16_t rfd_status = get_uint16(rfd_addr);
    if (rfd_status & STAT_B) {
        DBG(printf("RX: RFD is busy, cannot receive\n"));
        return -1;
    }

    /* Process frame based on mode */
    if (simplified_mode) {
        /* Simplified mode: All data goes into RFD */
        uint16_t rfd_size = get_uint16(rfd_addr + 14);  /* Available space in RFD */
        uint16_t data_offset = 16;  /* Data area offset in RFD */

        DBG(printf("RX: Simplified mode, RFD size=%d, target_size=%zu\n", rfd_size, target_size));

        /* Validate RFD size */
        if (rfd_size % 2 != 0) {
            DBG(printf("RFD size is not even, rejecting packet\n"));
            status |= RX_LENGTH_ERRORS;
            i82596_record_error(s, RX_LENGTH_ERRORS);
            s->align_err++;
            return -1;
        }

        /* Check if frame fits in RFD */
        if (target_size > rfd_size) {
            DBG(printf("Frame too large for RFD in simplified mode\n"));
            status |= RFD_STATUS_TRUNC;
            target_size = rfd_size;
            packet_completed = !SAVE_BAD_FRAMES ? false : true;
        }

        /* Copy frame data to RFD */
        address_space_write(&address_space_memory, rfd_addr + data_offset,
                           MEMTXATTRS_UNSPECIFIED, packet_data, target_size);
        bytes_copied = target_size;

        /* Update RFD actual count */
        set_uint16(rfd_addr + 12, target_size);

    } else {
        /* Flexible mode: Data distributed between RFD and RBDs */
        uint16_t rfd_size = get_uint16(rfd_addr + 14);
        uint16_t rfd_data_size = MIN(target_size, MIN(rx_copybreak, rfd_size));
        uint16_t data_offset = 16;  /* Data area offset in RFD */

        DBG(printf("RX: Flexible mode, RFD size=%d, will copy %d to RFD\n",
                   rfd_size, rfd_data_size));

        /* Copy initial data to RFD if there's space */
        if (rfd_data_size > 0) {
            address_space_write(&address_space_memory, rfd_addr + data_offset,
                              MEMTXATTRS_UNSPECIFIED, packet_data, rfd_data_size);
            bytes_copied = rfd_data_size;
            set_uint16(rfd_addr + 12, rfd_data_size);  /* Update actual count */
        }

        /* Process remaining data through RBD chain if needed */
        if (bytes_copied < target_size) {
            rbd_addr = get_uint32(rfd_addr + 8);  /* Get RBD pointer */
            rbd_addr = i82596_translate_address(s, rbd_addr, false);

            if (rbd_addr == I596_NULL || rbd_addr == 0) {
                DBG(printf("No RBD available for remaining data\n"));
                status |= RFD_STATUS_NOBUFS;
                i82596_record_error(s, RFD_STATUS_NOBUFS);
                s->resource_err++;
                i82596_update_rx_state(s, RX_NO_RESOURCES);
                packet_completed = !SAVE_BAD_FRAMES ? false : true;
            } else {
                /* Process RBD chain */
                while (bytes_copied < target_size && rbd_addr != I596_NULL) {
                    uint16_t rbd_status, rbd_size;
                    uint32_t rbd_buf_addr, next_rbd;

                    /* Read RBD fields */
                    rbd_status = get_uint16(rbd_addr);
                    rbd_buf_addr = get_uint32(rbd_addr + 8);
                    rbd_buf_addr = i82596_translate_address(s, rbd_buf_addr, true);
                    rbd_size = get_uint16(rbd_addr + 12) & SIZE_MASK;

                    DBG(printf("RX: Processing RBD 0x%08x, buf=0x%08x, size=%d\n",
                              rbd_addr, rbd_buf_addr, rbd_size));

                    /* Calculate how much to copy to this RBD */
                    size_t remaining = target_size - bytes_copied;
                    size_t to_copy = MIN(remaining, rbd_size);

                    /* Copy data to RBD buffer */
                    if (to_copy > 0 && rbd_buf_addr != 0 && rbd_buf_addr != I596_NULL) {
                        address_space_write(&address_space_memory, rbd_buf_addr,
                                          MEMTXATTRS_UNSPECIFIED,
                                          packet_data + bytes_copied, to_copy);
                        bytes_copied += to_copy;
                    }

                    /* Update RBD status */
                    uint16_t rbd_actual_count = to_copy;
                    if (bytes_copied >= target_size) {
                        rbd_actual_count |= I596_EOF;  /* Mark end of frame */
                    }
                    set_uint16(rbd_addr, rbd_actual_count);

                    /* Move to next RBD if we need more space */
                    if (bytes_copied < target_size) {
                        if (rbd_status & CMD_EOL) {  /* End of RBD list */
                            DBG(printf("RX: End of RBD list reached\n"));
                            status |= RFD_STATUS_NOBUFS;
                            i82596_record_error(s, RFD_STATUS_NOBUFS);
                            s->resource_err++;
                            packet_completed = !SAVE_BAD_FRAMES ? false : true;
                            break;
                        }

                        next_rbd = get_uint32(rbd_addr + 4);
                        next_rbd = i82596_translate_address(s, next_rbd, false);

                        if (next_rbd == 0 || next_rbd == I596_NULL || next_rbd == rbd_addr) {
                            DBG(printf("RX: Invalid next RBD 0x%08x\n", next_rbd));
                            status |= RFD_STATUS_NOBUFS;
                            packet_completed = !SAVE_BAD_FRAMES ? false : true;
                            break;
                        }
                        rbd_addr = next_rbd;
                    } else {
                        /* Frame completely received, advance RBD chain properly */
                        next_rfd = get_uint32(rfd_addr + 4);
                        next_rfd = i82596_translate_address(s, next_rfd, false);

                        if (next_rfd != I596_NULL && next_rfd != 0) {
                            /* Get the next RBD in the chain */
                            next_rbd = get_uint32(rbd_addr + 4);
                            uint16_t current_rbd_status = get_uint16(rbd_addr);

                            if (!(current_rbd_status & CMD_EOL) && next_rbd != I596_NULL && next_rbd != 0) {
                                /* Advance to next RBD in chain */
                                set_uint32(next_rfd + 8, next_rbd);
                                DBG(printf("RX: Updated next RFD 0x%08x to use RBD 0x%08x\n",
                                          next_rfd, next_rbd));
                            } else {
                                /* End of RBD chain - maintain current RBD for reuse */
                                /* Reset the RBD status to make it available again */
                                set_uint16(rbd_addr, 0);  /* Clear RBD status */
                                set_uint32(next_rfd + 8, rbd_addr);
                                DBG(printf("RX: End of RBD chain, reusing RBD 0x%08x for next RFD 0x%08x\n",
                                          rbd_addr, next_rfd));
                            }
                        }
                        break;
                    }
                }
            }
        }
    }

    /* Handle CRC storage if required */
    if (I596_CRCINM && packet_completed) {
        uint8_t crc_data[4];
        size_t crc_len = crc_size;

        if (I596_CRC16_32) {
            uint32_t crc = crc32(~0, packet_data, target_size);
            crc = cpu_to_be32(crc);
            memcpy(crc_data, &crc, 4);
        } else {
            uint16_t crc = i82596_calculate_crc16(packet_data, target_size);
            crc = cpu_to_be16(crc);
            memcpy(crc_data, &crc, 2);
        }

        /* Store CRC in appropriate location */
        if (simplified_mode) {
            uint16_t data_offset = 16;
            address_space_write(&address_space_memory, rfd_addr + data_offset + target_size,
                               MEMTXATTRS_UNSPECIFIED, crc_data, crc_len);
        } else {
            /* Store CRC in the last used RBD if possible */
            if (rbd_addr != I596_NULL) {
                uint32_t rbd_buf_addr = get_uint32(rbd_addr + 8);
                rbd_buf_addr = i82596_translate_address(s, rbd_buf_addr, true);
                if (rbd_buf_addr != 0 && rbd_buf_addr != I596_NULL) {
                    size_t last_copied = (bytes_copied - (target_size - bytes_copied));
                    address_space_write(&address_space_memory, rbd_buf_addr + last_copied,
                                       MEMTXATTRS_UNSPECIFIED, crc_data, crc_len);
                }
            }
        }

        DBG(printf("RX: CRC-%d stored: 0x%08x\n", I596_CRC16_32 ? 32 : 16,
                   I596_CRC16_32 ? *(uint32_t*)crc_data : *(uint16_t*)crc_data));
    }

    /* Update frame statistics and set final RFD status */
    i82596_update_rx_statistics(s, packet_completed, crc_valid, size);

    /* Set final RFD status */
    if (packet_completed && crc_valid) {
        status |= STAT_C | STAT_OK;  /* Complete and OK */
        if (is_broadcast) {
            status |= 0x0001;  /* Broadcast bit */
        }
    } else {
        status |= STAT_C;  /* Complete but not OK */
        if (!crc_valid) {
            status |= RX_CRC_ERRORS;
        }
    }

    set_uint16(rfd_addr, status);

    /* Advance RFA pointer to next RFD if frauint16_t rx_buffer_len;me was completed successfully */
    if (packet_completed) {
        next_rfd = get_uint32(rfd_addr + 4);  /* Get next RFD pointer */
        if (next_rfd != I596_NULL && next_rfd != 0) {
            next_rfd = i82596_translate_address(s, next_rfd, false);
            if (next_rfd != 0 && next_rfd != I596_NULL) {
                set_uint32(s->scb + 8, next_rfd);  /* Update RFA pointer in SCB */
                DBG(printf("RX: Advanced RFA pointer to 0x%08x\n", next_rfd));
            }
        }
    }

    /* Handle command flags */
    if (command & CMD_SUSP) {
        i82596_update_rx_state(s, RX_SUSPENDED);
        DBG(printf("RX: Suspending after packet (S bit set)\n"));
    }

    if (command & CMD_EOL) {
        i82596_update_rx_state(s, RX_NO_RESOURCES);
        DBG(printf("RX: End of RFD list reached (EL bit set)\n"));
    }

    /* Generate interrupt if packet was received successfully */
    if (packet_completed && crc_valid) {
        s->scb_status |= SCB_STATUS_FR;
        if (command & CMD_INTR) {
            i82596_update_scb_irq(s, true);
        }
    }

    /* Update SCB counters */
    set_uint32(s->scb + 16, s->crc_err);        /* CRC error counter */
    set_uint32(s->scb + 18, s->align_err);      /* Alignment error counter */
    set_uint32(s->scb + 20, s->resource_err);   /* Resource error counter */
    set_uint32(s->scb + 22, s->over_err);       /* Overrun error counter */

    DBG(printf("====== i82596_receive() END: status=0x%04x, copied=%zu ======\n",
               status, bytes_copied));
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
        /* In a real device, we might try to reclaim resources here
           For now we just check if we should reset the RU state */
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
        /* Device mode and configuration */
        VMSTATE_UINT8(mode, I82596State),
        VMSTATE_UINT16(t_on, I82596State),
        VMSTATE_UINT16(t_off, I82596State),
        VMSTATE_BOOL(throttle_state, I82596State),

        /* System addressing fields */
        VMSTATE_UINT32(iscp, I82596State),
        VMSTATE_UINT8(sysbus, I82596State),
        VMSTATE_UINT32(scb, I82596State),
        VMSTATE_UINT32(scb_base, I82596State),
        VMSTATE_UINT16(scb_status, I82596State),
        VMSTATE_UINT8(cu_status, I82596State),
        VMSTATE_UINT8(rx_status, I82596State),
        VMSTATE_UINT16(lnkst, I82596State),

        /* Command state */
        VMSTATE_UINT32(cmd_p, I82596State),
        VMSTATE_INT32(ca, I82596State),
        VMSTATE_INT32(ca_active, I82596State),
        VMSTATE_INT32(send_irq, I82596State),

        /* Configuration data */
        VMSTATE_BUFFER(mult, I82596State),
        VMSTATE_BUFFER(config, I82596State),

        /* Transmit buffer */
        VMSTATE_BUFFER(tx_buffer, I82596State),
        VMSTATE_END_OF_LIST()
    }
};

static void i82596_flush_queue_timer(void *opaque)
{
    I82596State *s = opaque;
    if (s->rx_status == RX_READY) {
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
        s->flush_queue_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    i82596_flush_queue_timer, s);
        s->throttle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                    i82596_bus_throttle_timer, s);
    }

    s->lnkst = 0x8000; /* initial link state: up */
}
