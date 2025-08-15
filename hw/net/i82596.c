/*
 * QEMU Intel i82596 (Apricot) emulation
 *
 * Copyright (c) 2019 Helge Deller <deller@gmx.de>
 * Additional improvement by Soumyajyotii Ssarkar <soumyajyotisarkar23@gmail.com>
 * This work is licensed under the GNU GPL license version 2 or later.
 *
 * This software was written to be compatible with the specification:
 * https://parisc.docs.kernel.org/en/latest/_downloads/96672be0650d9fc046bbcea40b92482f/82596CA.pdf
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

#define I82596_SPEED_MBPS       10
#define TX_TIMEOUT	            (HZ/20)
#define I82596_BYTES_PER_SEC    (I82596_SPEED_MBPS * 1000000 / 8)

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

#define RX_COLLISIONS         0x0001  /* Collision detected during frame reception */
#define RX_LENGTH_ERRORS      0x0080  /* Frame length error during reception */
#define RX_OVER_ERRORS        0x0100  /* Receiver overflow error */
#define RX_FIFO_ERRORS        0x0200  /* FIFO buffer overflow during reception */
#define RX_FRAME_ERRORS       0x0400  /* Frame alignment error */
#define RX_CRC_ERRORS         0x0800  /* CRC checksum error in received frame */
#define RX_LENGTH_ERRORS_ALT  0x1000  /* Duplicate definition - frame length error flag */
#define RFD_STATUS_TRUNC      0x0020  /* Received frame truncated due to buffer size */
#define RFD_STATUS_NOBUFS     0x0200  /* No buffer resources available for frame reception */

#define TX_COLLISIONS       0x0020  /* TX collision detection flag */
#define TX_HEARTBEAT_ERRORS 0x0040  /* Heartbeat signal lost during transmission */
#define TX_CARRIER_ERRORS   0x0400  /* Carrier sense signal lost during transmission */
#define TX_COLLISIONS_ALT   0x0800  /* Frame experienced collisions during transmission */
#define TX_ABORTED_ERRORS   0x1000  /* Transmission aborted due to excessive collisions */

/* Physmem access functions */
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


/*
 * Mode Transition of address function
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

static void update_scb_status(I82596State *s)
{
    s->scb_status = (s->scb_status & 0xf000)
        | (s->cu_status << 8) | (s->rx_status << 4);
    set_uint16(s->scb, s->scb_status);


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


static void i82596_xmit(I82596State *s, uint32_t addr)
{
    uint32_t tdb_p; /* Transmit Buffer Descriptor */
    uint16_t cmd;
    int insert_crc;
    int retry_count = 0;
    bool medium_available = false;
    uint16_t tx_status = 0;

    tdb_p = get_uint32(addr + 8);
    cmd = get_uint16(addr + 2);
    insert_crc = (I596_NOCRC_INS == 0) && ((cmd & 0x10) == 0) && !I596_LOOPBACK;

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
            /* Record excessive collision error */
            tx_status |= TX_ABORTED_ERRORS;
            /* Still proceed with simulated transmission but mark it as failed */
        }
        
        /* Record collision count in status */
        if (retry_count > 0) {
            tx_status |= TX_COLLISIONS;
            s->collision_events++;
            DBG(printf("CSMA/CD: Transmission proceeded after %d retries\n", retry_count));
        }
    }

    while (tdb_p != I596_NULL) {
        uint16_t size, len;
        uint32_t tba;

        size = get_uint16(tdb_p);
        len = size & SIZE_MASK;
        tba = get_uint32(tdb_p + 8);
        trace_i82596_transmit(len, tba);

        if (s->nic && len) {
            uint16_t new_len;
            new_len = len + 4;
            assert(new_len <= sizeof(s->tx_buffer));
            address_space_read(&address_space_memory, tba,
                MEMTXATTRS_UNSPECIFIED, s->tx_buffer, len);

            if (I596_NO_SRC_ADD_IN == 0) {
                memcpy(&s->tx_buffer[ETH_ALEN], s->conf.macaddr.a, ETH_ALEN);
            }

            DBG(printf("i82596_transmit: insert_crc = %d  insert SRC = %d\n",
                        insert_crc, I596_NO_SRC_ADD_IN == 0));
            if (insert_crc) {
                uint32_t crc = crc32(~0, s->tx_buffer, len);
                crc = cpu_to_be32(crc);
                memcpy(&s->tx_buffer[len], &crc, sizeof(crc));
                len += sizeof(crc);
            }

            DBG(PRINT_PKTHDR("Send", &s->tx_buffer));
            DBG(printf("Sending %d bytes (crc_inserted=%d)\n", len, insert_crc));
            
            /* Store transmission length for statistics */
            s->last_tx_len = len;
            
            switch (I596_LOOPBACK) {
            case 0:     /* no loopback, send packet */
                /* Update TX status with CSMA/CD status */
                if (tx_status) {
                    set_uint16(tdb_p + 2, tx_status);  /* Write collision status */
                }
                qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, len);
                break;
            default:
                i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, len);
                break;
            }
        }

        /* was this the last package? */
        if (size & I596_EOF) {
            qemu_flush_queued_packets(qemu_get_queue(s->nic));
            break;
        }

        /* get next buffer pointer */
        tdb_p = get_uint32(tdb_p + 4);
    }
}

/* Bus Throttle Functionality */
static void i82596_bus_throttle_timer(void *opaque)
{
    I82596State *s = opaque;

    if (s->cu_status != CU_ACTIVE) {
        timer_del(s->throttle_timer);
        return;
    }

    if (s->throttle_state) {
        /* Currently ON, switch to OFF */
        DBG(printf("THROTTLE: Switching from ON to OFF (duplex=%s)\n",
                   I596_FULL_DUPLEX ? "full" : "half"));
        s->throttle_state = false;

        if (s->t_off > 0) {
            /* Add jitter for half duplex to simulate CSMA/CD randomness */
            int delay = s->t_off;
            if (!I596_FULL_DUPLEX && s->t_off > 10) {
                int jitter = s->t_off / 5;
                int actual_jitter = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) % jitter;
                delay += actual_jitter;
                DBG(printf("THROTTLE: Half-duplex jitter added: %d microseconds\n",
                           actual_jitter));
            }

            timer_mod(s->throttle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     delay * 1000);
            DBG(printf("THROTTLE: OFF for %d microseconds\n", delay));
        } else {
            s->throttle_state = true;
            DBG(printf("THROTTLE: No OFF time specified, staying ON\n"));
        }
    } else {
        DBG(printf("THROTTLE: Switching from OFF to ON (duplex=%s)\n",
                   I596_FULL_DUPLEX ? "full" : "half"));
        s->throttle_state = true;

        if (s->t_on > 0 && s->t_on != 0xFFFF) {
            timer_mod(s->throttle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     s->t_on * 1000);
            DBG(printf("THROTTLE: ON for %d microseconds\n", s->t_on));
        } else {
            DBG(printf("THROTTLE: Staying ON indefinitely (t_on=%d)\n", s->t_on));
        }
    }
}

static void i82596_load_throttle_timers(I82596State *s, bool start_now)
{
    uint32_t t_on_addr, t_off_addr;

    /* Get previous values for comparison */
    uint16_t prev_t_on = s->t_on;
    uint16_t prev_t_off = s->t_off;

    /* TODO: Change offset based on the Linear or Segmented mode */
    t_on_addr = s->scb + 0x1E;
    t_off_addr = s->scb + 0x20;

    /* Read T-ON and T-OFF values */
    s->t_on = get_uint16(t_on_addr);
    s->t_off = get_uint16(t_off_addr);

    if (!I596_FULL_DUPLEX) {
        /* If t_on is zero or too low, use a reasonable default for half-duplex */
        if (s->t_on < 500) {
            /* ~1200μs corresponds to standard 1500 byte packet at 10Mbps */
            s->t_on = 1200;
            /* Write back to SCB memory */
            set_uint16(t_on_addr, s->t_on);
        }

        /* Ensure t_off is at least 20% of t_on for collision detection */
        if (s->t_off < (s->t_on / 10)) {
            s->t_off = s->t_on / 5;  /* 20% off time */
            set_uint16(t_off_addr, s->t_off);
        }
    } else {
        /* For full-duplex mode, we can have shorter off time */
        if (s->t_on < 100) {
            s->t_on = 1000;  /* Still need some throttling for 10Mbps */
            set_uint16(t_on_addr, s->t_on);
        }
    }

    if (prev_t_on != s->t_on || prev_t_off != s->t_off) {
        DBG(printf("THROTTLE PARAMS: Changed from ON=%d,OFF=%d to ON=%d,OFF=%d\n",
                   prev_t_on, prev_t_off, s->t_on, s->t_off));
    }

    DBG(printf("THROTTLE LOAD: T-ON=%d, T-OFF=%d, start=%d, duplex=%s\n",
               s->t_on, s->t_off, start_now, I596_FULL_DUPLEX ? "full" : "half"));

    if (start_now) {
        if (!s->throttle_timer) {
            s->throttle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                           i82596_bus_throttle_timer, s);
            DBG(printf("THROTTLE: Created new timer\n"));
        } else {
            timer_del(s->throttle_timer);
            DBG(printf("THROTTLE: Deleted existing timer\n"));
        }

        /* Start with the bus ON */
        s->throttle_state = true;
        DBG(printf("THROTTLE: Starting with state ON\n"));

        /* Schedule the T-ON timer if not infinite */
        if (s->t_on > 0 && s->t_on != 0xFFFF) {
            timer_mod(s->throttle_timer,
                      qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                      s->t_on * 1000);
            DBG(printf("THROTTLE: Scheduled ON timer for %d microseconds\n", s->t_on));
        } else {
            DBG(printf("THROTTLE: No timer scheduled (t_on=%d)\n", s->t_on));
        }
    }
}

static void i82596_s_reset(I82596State *s)
{
    trace_i82596_s_reset(s);
    s->scp = 0x00FFFFF4; /* default SCP pointer */
    s->scb_status = 0;
    s->cu_status = CU_IDLE;
    s->rx_status = RX_IDLE;
    s->cmd_p = I596_NULL;
    s->lnkst = 0x8000; /* initial link state: up */
    s->ca = s->ca_active = 0;
    s->send_irq = 0;

    /* Clear the config array */
    memset(s->config, 0, sizeof(s->config));

    /* Bus configurations */
    s->t_on = 0xFFFF; /* T-ON initally */
    s->t_off = 0;     /* No idle phase */
    s->throttle_state = true;

    if (s->throttle_timer) {
        timer_del(s->throttle_timer);
        s->throttle_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                       i82596_bus_throttle_timer, s);
        if (!I596_FULL_DUPLEX && s->t_on != 0xFFFF) {
            timer_mod(s->throttle_timer,
                     qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                     s->t_on * 1000);
        }
    }
    if (s->flush_queue_timer) {
        timer_del(s->flush_queue_timer);
    }
    qemu_set_irq(s->irq, 1);
}

void i82596_h_reset(void *opaque)
{
    I82596State *s = opaque;

    i82596_s_reset(s);
}

static void i82596_configure(I82596State *s, uint32_t addr)
{
    uint8_t byte_cnt;
    byte_cnt = get_byte(addr + 8) & 0x0f;

    byte_cnt = MAX(byte_cnt, 4);
    byte_cnt = MIN(byte_cnt, sizeof(s->config));
    /* copy byte_cnt max. */
    address_space_read(&address_space_memory, addr + 8,
                       MEMTXATTRS_UNSPECIFIED, s->config, byte_cnt);
    /* config byte according to page 35ff */
    s->config[2] &= 0x82; /* mask valid bits */
    s->config[2] |= 0x40;
    s->config[7]  &= 0xf7; /* clear zero bit */

    /* Configure throttling parameters to enforce 10Mbps speed limit */
    if (byte_cnt > 12) {
        bool previous_duplex = I596_FULL_DUPLEX;
        s->config[12] &= 0x40; /* Preserve only full duplex bit */

        if (previous_duplex != I596_FULL_DUPLEX) {
            DBG(printf("DUPLEX: Mode changed to %s duplex\n",
                       I596_FULL_DUPLEX ? "FULL" : "HALF"));
        }
        DBG(printf("DUPLEX: Current mode is %s\n", I596_FULL_DUPLEX ? "FULL" : "HALF"));
    }

    /* Configure throttling parameters to enforce 10Mbps speed limit */
    bool duplex_changed = false;
    static bool last_duplex_state = false;

    if (byte_cnt > 12) {
        duplex_changed = ((I596_FULL_DUPLEX) != last_duplex_state);
        if (duplex_changed) {
            DBG(printf("DUPLEX: State transition from %s to %s\n",
                       last_duplex_state ? "FULL" : "HALF",
                       I596_FULL_DUPLEX ? "FULL" : "HALF"));
        }
        last_duplex_state = I596_FULL_DUPLEX;
    }

    /* Only update throttle parameters if they haven't been set or duplex mode changed */
    if (s->t_on == 0xFFFF || duplex_changed) {
        /* Calculate throttling parameters for 10Mbps */
        uint32_t bytes_per_us = I82596_BYTES_PER_SEC / 1000000;
        uint16_t packet_time_us;

        /* Standard 1500 byte packet at 10Mbps takes ~1.2ms */
        packet_time_us = 1500 / bytes_per_us; /* ~1200μs at 10Mbps */

        if (I596_FULL_DUPLEX) {
            /* Full duplex: less throttling needed */
            s->t_on = packet_time_us * 5;
            s->t_off = packet_time_us / 20; /* Very short off time */
            DBG(printf("THROTTLE CONFIG: Full duplex - ON=%d, OFF=%d microseconds\n",
                      s->t_on, s->t_off));
        } else {
            /* Half duplex: more throttling to simulate collisions and CSMA/CD */
            s->t_on = packet_time_us;
            s->t_off = packet_time_us / 5; /* 20% off time */
            DBG(printf("THROTTLE CONFIG: Half duplex - ON=%d, OFF=%d microseconds\n",
                      s->t_on, s->t_off));
        }

        /* Start throttling with new parameters */
        i82596_load_throttle_timers(s, true);
    }

    if (s->rx_status == RX_READY) {
        timer_mod(s->flush_queue_timer,
            qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 100);
    }
    s->config[13] |= 0x3f; /* set ones in byte 13 */
    s->scb_status |= SCB_STATUS_CNA;
    qemu_set_irq(s->irq, 1);
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

static void i82596_flush_queue_timer(void *opaque)
{
    I82596State *s = opaque;
    if (s->rx_status == RX_READY) {
        qemu_flush_queued_packets(qemu_get_queue(s->nic));
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

    /* Write the updated stats to SCB */
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
    set_uint32(val, 0xFFC00000);  /* Success signature */
    set_uint32(val + 4, 0);

    s->scb_status &= ~STAT_OK;
    s->scb_status |= STAT_OK;

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
    uint32_t counter_addr;
    uint16_t count;

    /* Map error types to counter addresses */
    switch (error_type) {
    case RFD_STATUS_NOBUFS:
        counter_addr = s->scb + 20;  /* No buffer resources counter */
        break;
        case RFD_STATUS_TRUNC:
        counter_addr = s->scb + 22;  /* Truncated frames counter */
        break;
        default:
        return;
    }

    /* Increment the appropriate counter */
    count = get_uint16(counter_addr);
    set_uint16(counter_addr, count + 1);
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

static void i82596_update_scb_irq(I82596State *s, bool send_irq)
{
    update_scb_status(s);
    if (send_irq) {
        qemu_set_irq(s->irq, 1);
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
    uint32_t rfd_addr_head, rfd_addr, next_rfd, rbd_addr;
    uint16_t status = 0, command, sf_bit;
    uint16_t is_broadcast = 0;
    bool packet_completed = true;
    size_t target_size = size;
    size_t bytes_copied, rfd_write_area;
    uint32_t crc;
    uint8_t *crc_ptr = NULL;

    DBG(printf("====== i82596_receive() START ======\n"));
    DBG(PRINT_PKTHDR("[RX] packet", buf));

    /* VAlidation */
    if (!I596_FULL_DUPLEX && !s->throttle_state) {
        return size; /* Pretend we received it */
    }

    if (!i82596_validate_receive_state(s, &size)) {
        DBG(printf("ERROR: Invalid RX state, rejecting packet\n"));
        return -1;
    }

    bool passes_filter = i82596_check_packet_filter(s, buf, &is_broadcast);
    
    if (!i82596_monitor(s, buf, size, passes_filter)) {
        DBG(printf("Packet handled by monitor mode, not processing further\n"));
        return size; 
    }
    
    /* Only continue normal processing if monitor mode allows it */
    if (!passes_filter) {
        DBG(printf("Packet rejected by filter\n"));
        return size;
    }

    /* Get the head */
    rfd_addr_head = get_uint32(s->scb + 8);
    rfd_addr = rfd_addr_head;
    rfd_addr = i82596_translate_address(s, rfd_addr_head, false);

    if (rfd_addr == 0 || rfd_addr == I596_NULL) { /*Check if NULL */
        DBG(printf("No RFD available, setting NO_RESOURCES state\n"));
        i82596_update_rx_state(s, RX_NO_RESOURCES);
        return -1;
    }

    command = get_uint16(rfd_addr + 2);
    sf_bit = !(command & CMD_FLEX);  /* SF bit at bit 3 */


    do{
        next_rfd = get_uint32(rfd_addr + 4);
        /* Check if RFD is busy */
        if (status & STAT_B) {
            return -1;
        }

        uint16_t rfd_size = get_uint16(rfd_addr + 14) & SIZE_MASK;  /* Size field */
        uint16_t data_offset = 30;  /* Yes, Data starts after header fields */

        /*
         * Write starts from actual count?
         * Seems to work.
         */
        rfd_write_area= rfd_addr + 12;
        if (sf_bit || I596_LOOPBACK) {
            uint16_t data_size = MAX(target_size, rfd_size);

            address_space_write(&address_space_memory, rfd_write_area,
                               MEMTXATTRS_UNSPECIFIED, buf, data_size);
            target_size -= data_size;

            /* Check if frame was truncated */
            if ((data_size < size - data_offset) && !I596_LOOPBACK) {
                DBG(printf("Frame truncated in simplified mode\n"));
                status |= RFD_STATUS_TRUNC;
                i82596_record_error(s, RFD_STATUS_TRUNC);

                if (!SAVE_BAD_FRAMES) {
                    packet_completed = false;
                }
            }
        } else {
            /* FLEXIBLE MODE: Data split between RFD (till rx_copybreak) and then RBDs */
            bytes_copied = 0;
            bool out_of_buffers = false;

            /* Copy initial data to RFD if size > 0 */
            uint16_t rfd_data_size = MIN(rx_copybreak, rfd_size);
            if (rfd_data_size > 0) {
                address_space_write(&address_space_memory, rfd_addr + 30,
                                  MEMTXATTRS_UNSPECIFIED, buf, rfd_data_size);
                bytes_copied += rfd_data_size;
                target_size -= bytes_copied;
                /* Update RFD actual count */
                set_uint16(rfd_addr + 12, rfd_data_size);
            } else {
                bytes_copied = 0;
            }

            /* Process RBDs only if we haven't copied the entire frame yet */
            if (bytes_copied < target_size) {
                rbd_addr = get_uint32(rfd_addr + 8);
                rbd_addr = i82596_translate_address(s, rbd_addr, false);

                if (rbd_addr == I596_NULL || rbd_addr == 0) {
                    DBG(printf("No RBD for flexible mode\n"));
                    status |= RFD_STATUS_NOBUFS;
                    i82596_record_error(s, RFD_STATUS_NOBUFS);
                    i82596_update_rx_state(s, RX_NO_RESOURCES);
                    if (!SAVE_BAD_FRAMES) {
                        packet_completed = false;
                    }
                } else {

                    /* Process data through RBD chain */
                    while (bytes_copied < target_size && rbd_addr != I596_NULL) {
                        uint16_t rbd_status, buf_size;
                        uint32_t buf_addr, next_rbd;

                        /* Read RBD fields */
                        rbd_status = get_uint16(rbd_addr);
                        buf_addr = get_uint32(rbd_addr + 8);  /* Buffer address */
                        buf_addr = i82596_translate_address(s, buf_addr, true);
                        buf_size = get_uint16(rbd_addr + 12) & SIZE_MASK; /* Size field */

                        DBG(printf("RBD at %08x, buf=%08x, size=%d\n",
                                  rbd_addr, buf_addr, buf_size));

                        /* Copy data to buffer */
                        uint16_t to_copy = target_size;
                        address_space_write(&address_space_memory, buf_addr,
                                          MEMTXATTRS_UNSPECIFIED, buf + bytes_copied, to_copy);
                        bytes_copied += to_copy;
                        target_size -= to_copy;

                        uint16_t actual_count = bytes_copied;
                        if (bytes_copied >= target_size) {
                            actual_count |= I596_EOF;  /* EOF flag */
                        }
                        set_uint16(rbd_addr, actual_count | CMD_SUSP);  /* F bit */

                        /* Move to next RBD if needed */
                        if (bytes_copied < target_size) {
                            /* Check for end of RBD list */
                            if (rbd_status & CMD_EOL) {  /* EL bit */
                                out_of_buffers = true;
                                break;
                            }

                            next_rbd = get_uint32(rbd_addr + 4);
                            next_rbd = i82596_translate_address(s, next_rbd, true);

                            /* Check for NULL & Cyclic reference*/
                            if (next_rbd == 0 || next_rbd == I596_NULL || next_rbd == rbd_addr) {
                                out_of_buffers = true;
                                status |= RFD_STATUS_NOBUFS;
                                printf(" out of buffers, next_rbd=%08x\n", next_rbd);
                                break;
                            }
                            rbd_addr = next_rbd;
                        }
                    }

                    /* Handle resource exhaustion */
                    if (out_of_buffers && bytes_copied < target_size) {
                        DBG(printf("Out of buffers, frame truncated\n"));
                        status |= RFD_STATUS_NOBUFS;
                        i82596_record_error(s, RFD_STATUS_NOBUFS);
                        i82596_update_rx_state(s, RX_NO_RESOURCES);
                        if (!SAVE_BAD_FRAMES) {
                            packet_completed = false;
                        }
                        rfd_addr= rfd_addr_head;  /* Reset to head RFD */
                    }

                    next_rfd = get_uint32(rfd_addr + 4);
                    next_rfd = i82596_translate_address(s, next_rfd, false);
                    if (next_rfd != I596_NULL && rbd_addr != I596_NULL) {
                        rfd_addr = next_rfd;
                        printf("Moving to next RFD: %08x\n", rfd_addr);
                    }
                }
            }
        }
    } while (target_size > 0);

    if (I596_CRCINM && I596_LOOPBACK && !sf_bit) {
        crc = crc32(~0, buf, 4);
        crc = cpu_to_be32(crc);
        crc_ptr = (uint8_t *) &crc;
        size += 4;
        printf("CRC: %08x\n", crc);
        if (sf_bit){
            address_space_write(&address_space_memory, rfd_addr + 30 + size - 4,
                               MEMTXATTRS_UNSPECIFIED, crc_ptr, 4);
        } else {
            if (rbd_addr != I596_NULL) {
                uint32_t rbd_crc_addr = get_uint32(rbd_addr + 8) + bytes_copied;
                address_space_write(&address_space_memory, rbd_crc_addr,
                                   MEMTXATTRS_UNSPECIFIED, crc_ptr, 4);
            } else {
                DBG(printf("[ERROR]: No RBD available for CRC write\n"));
            }
        }
    }

    /* Update RFD status after reception */
    if (packet_completed) {
        status |= STAT_C | STAT_OK | STAT_B | is_broadcast;  /* Set complete and OK bits */
    } else {
        /* Failed reception - update bits accordingly */
        status &= ~STAT_B;  /* Messed up somewhere busy bit */
        status |= STAT_C;   /* Set complete bit */
        status &= ~STAT_OK; /* Not OK bit */
    }
    set_uint16(rfd_addr, status);

    if (command & CMD_SUSP) {
        i82596_update_rx_state(s, RX_SUSPENDED);
        DBG(printf("RX: End of RFD list reached (EL bit set)\n"));
    }

    if (command & CMD_EOL) {
        i82596_update_rx_state(s, RX_NO_RESOURCES);
        DBG(printf("RX: RX suspended (S bit set)\n"));
    }

    /* Generate interrupt if packet was completed successfully */
    if (packet_completed) {
        s->scb_status |= SCB_STATUS_FR;
        i82596_update_scb_irq(s, true);
    }

    DBG(printf("====== i82596_receive() END ======\n"));
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
    }

    s->lnkst = 0x8000; /* initial link state: up */
}
