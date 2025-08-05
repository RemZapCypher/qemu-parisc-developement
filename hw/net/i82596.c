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

#define RFD_STATUS_TRUNC  0x0020  /* Frame truncated */
#define RFD_STATUS_NOBUFS 0x0200  /* Out of buffer space */

#define CMD_FLEX        0x0008  /* Enable flexible memory model */
#define CMD_MASK        0x0007  /* Mask for command bits */

#define CMD_EOL         0x8000  /* The last command of the list, stop, wrong use */
#define CMD_SUSP        0x4000  /* Suspend after doing cmd. */
#define CMD_INTR        0x2000  /* Interrupt after doing cmd. */

enum commands {
        CmdNOp = 0, CmdSASetup = 1, CmdConfigure = 2, CmdMulticastList = 3,
        CmdTx = 4, CmdTDR = 5, CmdDump = 6, CmdDiagnose = 7
};

#define STAT_C          0x8000  /* Set to 0 after execution */
#define STAT_B          0x4000  /* Command being executed */
#define STAT_OK         0x2000  /* Command executed ok */
#define STAT_A          0x1000  /* Command aborted */

/* RFD status error bits */
#define RFD_ERR_COLLISION      0x0001  /* Collision detected */
#define RFD_ERR_IA_MATCH       0x0002  /* Did not match individual address (multicast) */
#define RFD_ERR_NO_EOP         0x0040  /* No EOP flag (bit stuffing only) */
#define RFD_ERR_SHORT_FRAME    0x0080  /* Frame is too short */
#define RFD_ERR_DMA_OVER       0x0100  /* DMA overrun */
#define RFD_ERR_NO_RESOURCES   0x0200  /* Out of buffer space (no RBDs) */
#define RFD_ERR_ALIGN          0x0400  /* Alignment error */
#define RFD_ERR_CRC            0x0800  /* CRC error */
#define RFD_ERR_LENGTH         0x1000  /* Length error */
#define RFD_ERR_TRUNCATED      0x0020  /* Frame truncated, in simple mode */

/* Buffer descriptor bits */
#define BD_EOF                 0x8000  /* End of frame */
#define BD_F                   0x4000  /* Buffer used */
#define BD_COUNT_MASK          0x3FFF  /* Actual count mask */

/* Global Flags fetched from config bytes */
#define I596_PREFETCH       (s->config[0] & 0x80)    /* Enable prefetch of data structures */
#define SAVE_BAD_FRAMES     (s->config[2] & 0x80)    /* Store frames with errors in memory */
#define I596_NO_SRC_ADD_IN  (s->config[3] & 0x08)    /* If 1, do not insert MAC in Tx Packet */
#define I596_LOOPBACK       (s->config[3] >> 6)      /* Determine the Loopback mode */
#define I596_PROMISC        (s->config[8] & 0x01)    /* Receive all packets regardless of MAC */
#define I596_BC_DISABLE     (s->config[8] & 0x02)    /* Disable reception of broadcast packets */
#define I596_NOCRC_INS      (s->config[8] & 0x08)    /* Do not append CRC to Tx frame */
#define I596_CRC16_32       (s->config[8] & 0x10)    /* Use CRC-32 if set, otherwise CRC-16 */
#define I596_PADDING        (s->config[8] & 0x80)    /* Add padding to short frames */
#define I596_MIN_FRAME_LEN  (s->config[10])          /* Minimum Ethernet frame length */
#define I596_CRCINM         (s->config[11] & 0x04)   /* Rx CRC appended in memory */
#define I596_MC_ALL         (s->config[11] & 0x20)   /* Receive all multicast packets */
#define I596_FULL_DUPLEX    (s->config[12] & 0x40)   /* Full-duplex mode if set, half if clear */
#define I596_MULTIIA        (s->config[13] & 0x40)   /* Enable multiple individual addresses */

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

static size_t i82596_calculate_crc(I82596State *s, const uint8_t *buf, size_t size,
                                  uint8_t *crc_buf)
{
    if (I596_CRC16_32) {
        uint32_t crc = net_crc32(buf, size);
        crc = cpu_to_be32(crc);
        memcpy(crc_buf, &crc, sizeof(crc));

        DBG(printf("CRC-32 calculated: %08x\n", be32_to_cpu(crc)));
        return 4;
    } else {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < size; i++) {
            crc = (crc >> 8) | (crc << 8);
            crc ^= buf[i];
            crc ^= (crc & 0xff) >> 4;
            crc ^= (crc << 8) << 4;
            crc ^= ((crc & 0xff) << 4) << 1;
        }

        crc = cpu_to_be16(crc);
        memcpy(crc_buf, &crc, sizeof(crc));
        DBG(printf("CRC-16 calculated: %04x\n", be16_to_cpu(crc)));
        return 2;
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

static void i82596_xmit(I82596State *s, uint32_t addr)
{
    uint32_t tbd_p; /* Transmit Buffer Descriptor */
    uint16_t cmd;
    uint16_t tcb_bytes = 0;
    uint16_t tx_data_len = 0;
    bool insert_crc;

    if (!s->throttle_state && !I596_FULL_DUPLEX) {
        /* In half duplex mode, defer transmission until throttle is on */
        DBG(printf("TX COLLISION: Half duplex collision detected, deferring transmission\n"));
        timer_mod(s->flush_queue_timer,
                 qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) + 10);
        return;
    }
    cmd = get_uint16(addr + 2);
    assert(cmd & CMD_FLEX);    /* check flexible mode */

    /* Get TBD pointer */
    tbd_p = get_uint32(addr + 8);
    tbd_p = i82596_translate_address(s, tbd_p, false);
    /* Get TCB byte count (immediate data in TCB) */
    tcb_bytes = get_uint16(addr + 12);

    /* Copy immediate data from TCB if present */
    if (tcb_bytes > 0) {
        assert(tcb_bytes <= sizeof(s->tx_buffer));
        address_space_read(&address_space_memory, addr + 16,
                           MEMTXATTRS_UNSPECIFIED, s->tx_buffer, tcb_bytes);
    }

    /* Process TBD chain if present */
    if (tbd_p != I596_NULL) {
        while (tbd_p != I596_NULL && tx_data_len < sizeof(s->tx_buffer)) {
            uint16_t size;
            uint32_t tba;
            uint16_t buf_len;

            size = get_uint16(tbd_p);
            buf_len = size & BD_COUNT_MASK;
            tba = get_uint32(tbd_p + 8);
            tba = i82596_translate_address(s, tba, true);  /* true for data buffer */

            trace_i82596_transmit(buf_len, tba);

            if (buf_len > 0 && (tx_data_len + buf_len) <= sizeof(s->tx_buffer)) {
                address_space_read(&address_space_memory, tba,
                                   MEMTXATTRS_UNSPECIFIED,
                                   &s->tx_buffer[tx_data_len], buf_len);
                tx_data_len += buf_len;
            }

            /* Check if this is the last TBD */
            if (size & BD_EOF) {
                break;
            }

            /* Get next TBD pointer */
            tbd_p = get_uint32(tbd_p + 4);
            tbd_p = i82596_translate_address(s, tbd_p, false);
        }
    }

    /* Check if we should insert CRC */
    insert_crc = (I596_NOCRC_INS == 0) && ((cmd & 0x10) == 0) && !I596_LOOPBACK;

    if (s->nic && tx_data_len > 0) {
        DBG(printf("i82596_transmit: insert_crc = %d, len = %d\n", insert_crc, tx_data_len));

        if (insert_crc && (tx_data_len + 4) <= sizeof(s->tx_buffer)) {
            /* Use our CRC calculation function */
            uint8_t crc_buffer[4];
            size_t crc_size = i82596_calculate_crc(s, s->tx_buffer, tx_data_len, crc_buffer);

            /* Copy CRC to the end of the buffer */
            memcpy(&s->tx_buffer[tx_data_len], crc_buffer, crc_size);
            tx_data_len += crc_size;

            DBG(printf("Added %zu-byte CRC to transmitted frame\n", crc_size));
        }

        /* Validate minimum frame size */
        if (tx_data_len < I596_MIN_FRAME_LEN) { /* Minimum Ethernet frame header */
            DBG(printf("Frame too short (%d bytes), padding needed\n", tx_data_len));

            if (I596_PADDING) {
                int padding_needed = I596_MIN_FRAME_LEN - tx_data_len;
                if (padding_needed > 0 && (tx_data_len + padding_needed) <= sizeof(s->tx_buffer)) {
                    memset(&s->tx_buffer[tx_data_len], 0, padding_needed);
                    tx_data_len += padding_needed;
                    DBG(printf("Added %d bytes of padding\n", padding_needed));
                } else {
                    DBG(printf("WARNING: Cannot add %d bytes of padding - would overflow buffer\n",
                              padding_needed));
                }
            }
        }
    }

    DBG(PRINT_PKTHDR("Send", s->tx_buffer));
    DBG(printf("Sending %d bytes (crc_inserted=%d)\n", tx_data_len, insert_crc));

    printf("LOOPBACKING: %d",I596_LOOPBACK);
    switch (I596_LOOPBACK) {
    case 0:     /* no loopback, send packet */
        qemu_send_packet_raw(qemu_get_queue(s->nic), s->tx_buffer, tx_data_len);
        break;
    case 1:     /* external loopback enabled */
    case 2:
    case 3:
        i82596_receive(qemu_get_queue(s->nic), s->tx_buffer, tx_data_len);
        break;
    default:    /* all other loopback modes: ignore! */
        break;
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

static void schedule_packet_processing(I82596State *s, uint32_t rfd_addr)
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

    /* Clear the SCB command word */
    set_uint16(s->scb + 2, 0);

    /* Handle interrupt acknowledgment */
    s->scb_status &= ~(command & SCB_ACK_MASK);

    /* Process Command Unit Command */
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
        /* Resume Command Unit */
        if (s->cu_status != CU_ACTIVE) {
            s->cu_status = CU_ACTIVE;
        }
        break;

    case SCB_CUC_SUSPEND:
        /* Suspend Command Unit */
        s->cu_status = CU_SUSPENDED;
        s->scb_status |= SCB_STATUS_CNA;
        break;

    case SCB_CUC_ABORT:
        /* Abort Command Unit */
        s->cu_status = CU_IDLE;
        s->scb_status |= SCB_STATUS_CNA;
        break;

    case SCB_CUC_LOAD_THROTTLE:
            /* Load Throttle Timers, unless in 82596 mode */
            bool external_trigger = (s->sysbus & I82586_MODE);
            i82596_load_throttle_timers(s, !external_trigger);
        break;

    case SCB_CUC_LOAD_START:
            i82596_load_throttle_timers(s, true);
        break;
    }

    /* Process Receive Unit Command */
    switch (ruc) {
    case SCB_RUC_NOP:
        /* No operation */
        break;
    case SCB_RUC_START:
        s->rx_status = RX_READY;
        uint32_t rfd_addr = get_uint32(s->scb + 8);
        rfd_addr = i82596_translate_address(s, rfd, false);
        if (rfd_addr == 0 || rfd_addr == I596_NULL) {
            s->rx_status = RX_NO_RESOURCES;
            s->scb_status |= SCB_STATUS_RNR;
        } else {
            schedule_packet_processing(s, rfd_addr);
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
        /* Suspend Receive Unit */
        s->rx_status = RX_SUSPENDED;
        s->scb_status |= SCB_STATUS_RNR;
        /* Stop the flush timer when suspended */
        timer_del(s->flush_queue_timer);
        break;

    case SCB_RUC_ABORT:
        /* Abort Receive Unit */
        s->rx_status = RX_IDLE;
        s->scb_status |= SCB_STATUS_RNR;
        /* Stop the flush timer when aborted */
        timer_del(s->flush_queue_timer);
        break;
    }

    /* Check for software reset */
    if (command & 0x80) {
        i82596_s_reset(s);
    } else {
        /* Execute commands if CU is active and not already processing commands */
        if (s->cu_status == CU_ACTIVE) {
            if (s->cmd_p == I596_NULL) {
                s->cmd_p = get_uint32(s->scb + 4);
            }
            /* Update SCB status */
            update_scb_status(s);

            /* Process any pending commands */
            command_loop(s);
        } else {
            /* Just update SCB status */
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

    /* Clear bit 13 in the SCB status */
    s->scb_status &= ~SCB_STATUS_CNA;
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
        printf("Alternate dump command not implemented\n");
        break;
    case PORT_CA:
        signal_ca(s);
        break;
    }
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
        DBG(printf("CAN_RX: %s - flush timer pending, rx_status=%d\n",
                  can_rx ? "TRUE" : "FALSE", s->rx_status));
        return can_rx;
    }

    DBG(printf("CAN_RX: TRUE - all conditions passed\n"));
    return true;
}

static int i82596_validate_receive_state(I82596State *s, size_t *sz, size_t *bufsz, size_t *len)
{
    if (*sz < 14 || *sz > PKT_BUF_SZ - 4) {
        trace_i82596_receive_analysis(">>> Packet size invalid");
        return -1;
    }

    if (timer_pending(s->flush_queue_timer)) {
        trace_i82596_receive_analysis(">>> Flush timer pending, cannot receive");
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

static ssize_t i82596_receive_iov(NetClientState *nc, const struct iovec *iov, int iovcnt)
{
    I82596State *s = qemu_get_nic_opaque(nc);
    uint32_t rfd_addr, rbd_addr, next_rfd_addr;
    uint16_t rfd_status, rfd_cmd;
    uint8_t *filter_buf = iov->iov_base;
    size_t size = iov_size(iov, iovcnt);
    size_t iov_ofs = 0;
    size_t total_size = size;
    size_t bytes_copied = 0;
    uint16_t is_broadcast = 0;
    uint16_t actual_count;
    bool el_bit = false;
    bool packet_accepted = false;
    bool simplified_mode = false;
    bool crc_valid = true;
    uint8_t crc_buffer[4];
    size_t crc_size;
    size_t data_size;

    DBG(printf("RX: ============= PACKET RECEPTION START =============\n"));
    DBG(printf("RX: Device state - cu_status=%d, rx_status=%d, size %zu bytes, loopback=%d\n",
               s->cu_status, s->rx_status, size, I596_LOOPBACK));
    PRINT_PKTHDR("RX", filter_buf);

    /* Validate device state and packet size */
    if (!i82596_validate_receive_state(s, &size, NULL, NULL)) {
        DBG(printf("RX: Packet validation failed, rejecting\n"));
        return -1;
    }

    /* Apply packet filtering */
    if (!i82596_check_packet_filter(s, filter_buf, &is_broadcast)) {
        DBG(printf("RX: Packet filtered out by MAC filter\n"));
        return -1;
    }

    /* In loopback mode, special handling for CRC */
    if (I596_LOOPBACK || I596_NOCRC_INS) {
        /* In loopback mode, data_size = total_size (no CRC) and always valid */
        data_size = total_size;
        crc_valid = true;
        trace_i82596_receive_analysis("Loopback mode enabled - skipping CRC validation");
    } else {
        /* Normal CRC validation for non-loopback packets */
        crc_size = I596_CRC16_32 ? 4 : 2;

        DBG(printf("RX: Normal mode: validating CRC-%s\n", I596_CRC16_32 ? "32" : "16"));

        /* Determine actual data size excluding CRC */
        if (total_size >= crc_size) {
            data_size = total_size - crc_size;

            DBG(printf("RX: Data size=%zu, CRC size=%zu, total=%zu\n",
                       data_size, crc_size, total_size));

            /* Calculate expected CRC */
            i82596_calculate_crc(s, filter_buf, data_size, crc_buffer);
            DBG(printf("RX: CRC validation %s\n", crc_valid ? "PASSED" : "FAILED"));
        } else {
            data_size = total_size;
            crc_valid = false; /* Too small to have a valid CRC */
            DBG(printf("RX: Packet too small (%zu) for CRC validation, minimum size=%zu\n",
                       total_size, crc_size));
        }
    }

    /* Determine if we should reject frame with invalid CRC */
    if ((!crc_valid && !SAVE_BAD_FRAMES) || !SAVE_BAD_FRAMES) {
        DBG(printf("RX: Frame with invalid CRC rejected (crc_valid=%d, SAVE_BAD_FRAMES=%d)\n",
                  crc_valid, SAVE_BAD_FRAMES));
        DBG(printf("RX: ============= PACKET RECEPTION END (CRC REJECT) =============\n"));
        return total_size; /* Consume the frame but don't process it */
    }

    /* Get the address of the current RFD */
    rfd_addr = get_uint32(s->scb + 8);
    rfd_addr = i82596_translate_address(s, rfd_addr, false);
    DBG(printf("RX: Got RFD address: 0x%08x (from SCB+8)\n", rfd_addr));

    if (rfd_addr == 0 || rfd_addr == I596_NULL) {
        s->rx_status = RX_NO_RESOURCES;
        s->scb_status |= SCB_STATUS_RNR;
        DBG(printf("RX: No valid RFD address available\n"));
        trace_i82596_receive_analysis("No RFDs available");
        qemu_set_irq(s->irq, 1);
        DBG(printf("RX: ============= PACKET RECEPTION END (NO RFD) =============\n"));
        return -1;
    }

    /* Process RFD chain */
    DBG(printf("RX: Starting to process RFD chain\n"));
    while (rfd_addr != 0 && rfd_addr != I596_NULL && !packet_accepted) {
        /* Read the RFD status and command words */
        rfd_status = get_uint16(rfd_addr);
        rfd_cmd = get_uint16(rfd_addr + 2);
        DBG(printf("RX: RFD at 0x%08x - status=0x%04x, cmd=0x%04x\n",
                   rfd_addr, rfd_status, rfd_cmd));

        /* Check if using simplified mode (SF bit in command word) - 3rd bit */
        simplified_mode = (rfd_cmd & 0x0008) != 0;  /* 0x0008 is the SF bit */
        DBG(printf("RX: RFD command word: 0x%04x, SF bit (0x0008): %d\n",
                   rfd_cmd, simplified_mode ? 1 : 0));
        DBG(printf("RX: CMD_FLEX value is 0x%04x (for reference)\n", CMD_FLEX));

        /* Check if RFD is already complete (C bit set) */
        if (rfd_status & STAT_C) {
            DBG(printf("RX: RFD already marked complete, trying next in chain\n"));
            /* This RFD is already in use, try next one in the chain */
            next_rfd_addr = get_uint32(rfd_addr + 4);
            next_rfd_addr = i82596_translate_address(s, next_rfd_addr, false);

            DBG(printf("RX: Next RFD address: 0x%08x\n", next_rfd_addr));
            if (next_rfd_addr == rfd_addr || next_rfd_addr == 0 || next_rfd_addr == I596_NULL) {
                /* No more RFDs available */
                s->rx_status = RX_NO_RESOURCES;
                s->scb_status |= SCB_STATUS_RNR;
                DBG(printf("RX: No more valid RFDs in chain\n"));
                trace_i82596_receive_analysis("RFD chain exhausted (all marked complete)");
                qemu_set_irq(s->irq, 1);
                DBG(printf("RX: ============= PACKET RECEPTION END (ALL RFDs COMPLETE) =============\n"));
                return -1;
            }

            rfd_addr = next_rfd_addr;
            continue;
        }

        /* Mark this RFD as busy */
        rfd_status |= STAT_B;
        set_uint16(rfd_addr, rfd_status);
        DBG(printf("RX: Marked RFD as busy, new status=0x%04x\n", rfd_status));

        /* Determine size of data to store based on I596_CRCINM config */
        size_t store_size = I596_CRCINM ? total_size : data_size;
        DBG(printf("RX: CRCINM=%d, store_size=%zu (data_size=%zu, total_size=%zu)\n",
                   I596_CRCINM, store_size, data_size, total_size));

        /* Get the RBD link field */
        rbd_addr = get_uint32(rfd_addr + 8);
        rbd_addr = i82596_translate_address(s, rbd_addr, false);
        DBG(printf("RX: RBD address from RFD: 0x%08x\n", rbd_addr));

        /* Check if we're in simplified mode - either by SF bit or by NULL RBD pointer */
        if (simplified_mode || rbd_addr == 0 || rbd_addr == I596_NULL) {
            DBG(printf("RX: Using SIMPLIFIED mode (SF=%d, RBD=0x%08x)\n",
                       simplified_mode, rbd_addr));
            DBG(printf("RX: Simplified mode triggered by %s\n",
                      simplified_mode ? "SF bit" : "NULL RBD pointer"));
            /*
             * Simplified mode - copy directly to buffer in RFD
             * In this case, we copy the data directly to RFD + 16 (after the header)
             */
            uint32_t buf_addr = rfd_addr + 16;
            size_t copy_size = store_size;
            bytes_copied = 0;

            /* Don't copy more than what can fit */
            if (copy_size > s->rx_buffer_size) {
                copy_size = s->rx_buffer_size;
                rfd_status |= RFD_STATUS_TRUNC;
                DBG(printf("RX: Simplified mode - Truncating to %zu bytes (buffer size=%zu)\n",
                          copy_size, s->rx_buffer_size));
            }

            /* Copy the data */
            DBG(printf("RX: Simplified mode - Copying %zu bytes to 0x%08x\n", copy_size, buf_addr));
            do {
                size_t iov_copy = MIN(copy_size - bytes_copied, iov->iov_len - iov_ofs);
                DBG(printf("RX: Copy iteration - bytes to copy: %zu, already copied: %zu\n",
                          iov_copy, bytes_copied));
                address_space_write(&address_space_memory, buf_addr + bytes_copied,
                                   MEMTXATTRS_UNSPECIFIED,
                                   iov->iov_base + iov_ofs, iov_copy);
                bytes_copied += iov_copy;
                iov_ofs += iov_copy;
                if (iov_ofs == iov->iov_len) {
                    iov++;
                    iov_ofs = 0;
                    DBG(printf("RX: Moving to next iov segment\n"));
                }
            } while (bytes_copied < copy_size);

            /* Set actual count received in the RFD length field */
            set_uint16(rfd_addr + 12, bytes_copied);
            DBG(printf("RX: Simplified mode - Set RFD length to %zu bytes\n", bytes_copied));

            packet_accepted = true;
            DBG(printf("RX: Packet accepted in simplified mode, %zu bytes\n", bytes_copied));
        } else {
            /* Normal mode - process RBD chain */
            DBG(printf("RX: Using NORMAL mode (chained RBDs)\n"));
            bytes_copied = 0;
            bool first_buffer = true;
            uint32_t first_buffer_addr = 0;
            uint16_t rbd_size, rbd_status;

            /* Reset iov pointer and offset for the RBD chain */
            const struct iovec *current_iov = iov;
            size_t current_iov_ofs = 0;

            while (rbd_addr != 0 && rbd_addr != I596_NULL && bytes_copied < store_size) {
                /* Read RBD size, status and buffer address */
                rbd_size = get_uint16(rbd_addr + 2) & 0x3FFF; /* Size mask */
                rbd_status = get_uint16(rbd_addr);
                uint32_t buf_addr = get_uint32(rbd_addr + 8);
                buf_addr = i82596_translate_address(s, buf_addr, true);

                DBG(printf("RX: Processing RBD at 0x%08x, status=0x%04x, size=%u, buf=0x%08x\n",
                           rbd_addr, rbd_status, rbd_size, buf_addr));
                DBG(printf("RX: RBD status bits - EOF=%d, F=%d\n",
                          (rbd_status & BD_EOF) ? 1 : 0,
                          (rbd_status & BD_F) ? 1 : 0));

                if (buf_addr == 0 || buf_addr == I596_NULL) {
                    /* Invalid buffer address */
                    DBG(printf("RX: Invalid buffer address in RBD\n"));
                    break;
                }

                /* Remember first buffer address for reference */
                if (first_buffer) {
                    first_buffer = false;
                    first_buffer_addr = buf_addr;
                    DBG(printf("RX: First buffer at 0x%08x\n", first_buffer_addr));
                }

                /* Calculate how much to copy to this buffer */
                uint16_t copy_size = MIN(rbd_size, store_size - bytes_copied);
                DBG(printf("RX: Will copy %u bytes to buffer (out of %zu remaining)\n",
                           copy_size, store_size - bytes_copied));

                /* Copy the data */
                size_t buffer_bytes = 0;
                do {
                    size_t iov_copy = MIN(copy_size - buffer_bytes,
                                        current_iov->iov_len - current_iov_ofs);

                    DBG(printf("RX: Copying %zu bytes to RBD buffer (buffer offset=%zu)\n",
                              iov_copy, buffer_bytes));
                    address_space_write(&address_space_memory, buf_addr + buffer_bytes,
                                      MEMTXATTRS_UNSPECIFIED,
                                      current_iov->iov_base + current_iov_ofs, iov_copy);

                    buffer_bytes += iov_copy;
                    current_iov_ofs += iov_copy;

                    if (current_iov_ofs == current_iov->iov_len) {
                        current_iov++;
                        current_iov_ofs = 0;
                        DBG(printf("RX: Moving to next iov segment in RBD processing\n"));
                    }
                } while (buffer_bytes < copy_size);

                bytes_copied += buffer_bytes;
                DBG(printf("RX: Copied %zu bytes to buffer, total so far: %zu\n",
                           buffer_bytes, bytes_copied));

                /* Set actual count in RBD */
                actual_count = buffer_bytes;

                /* Set EOF bit if this is the last buffer for the frame */
                if (bytes_copied >= store_size) {
                    rbd_status |= BD_EOF; /* EOF bit */
                    DBG(printf("RX: Setting EOF bit in RBD, copied all data\n"));
                }

                /* Update RBD status and count */
                DBG(printf("RX: Updating RBD status=0x%04x, count=%u\n",
                          rbd_status, actual_count));
                set_uint16(rbd_addr, rbd_status);
                set_uint16(rbd_addr + 2, actual_count | (rbd_status & 0xC000));
                DBG(printf("RX: Updated RBD status=0x%04x, count word=0x%04x\n",
                          rbd_status, actual_count | (rbd_status & 0xC000)));

                /* Check for EL bit in RBD status */
                el_bit = (rbd_status & 0x8000) != 0; /* EL bit */

                if (el_bit) {
                    DBG(printf("RX: Found EL bit in RBD (0x8000), end of buffer list\n"));
                    /* End of buffer list */
                    if (bytes_copied < store_size) {
                        /* Not enough space for the entire packet */
                        rfd_status |= RFD_STATUS_TRUNC;
                        DBG(printf("RX: Packet truncated at %zu bytes (needed %zu)\n",
                                  bytes_copied, store_size));
                    }
                    break;
                }

                /* Move to next RBD */
                uint32_t next_rbd = get_uint32(rbd_addr + 4);
                next_rbd = i82596_translate_address(s, next_rbd, false);
                DBG(printf("RX: Moving to next RBD at 0x%08x\n", next_rbd));

                if (next_rbd == rbd_addr || next_rbd == 0 || next_rbd == I596_NULL) {
                    /* No more RBDs available */
                    DBG(printf("RX: No more valid RBDs available\n"));
                    if (bytes_copied < store_size) {
                        rfd_status |= RFD_STATUS_TRUNC;
                        DBG(printf("RX: Packet truncated at %zu bytes (needed %zu)\n",
                                  bytes_copied, store_size));
                    }
                    break;
                }

                rbd_addr = next_rbd;
            }

            /* Set actual count received in the RFD length field */
            set_uint16(rfd_addr + 12, bytes_copied);
            DBG(printf("RX: Set RFD length to %zu bytes\n", bytes_copied));

            packet_accepted = true;
            DBG(printf("RX: Packet accepted in normal mode, %zu bytes\n", bytes_copied));
        }

        /* If we've successfully processed this packet, mark RFD complete */
        if (packet_accepted) {
            /* Mark RFD as complete */
            rfd_status &= ~STAT_B;  /* Clear busy bit */
            rfd_status |= STAT_C;   /* Set complete bit */
            DBG(printf("RX: Marking RFD as complete, clearing busy bit\n"));

            /* Set OK bit only if CRC is valid or we accept bad frames */
            if (crc_valid || SAVE_BAD_FRAMES) {
                rfd_status |= STAT_OK;
                DBG(printf("RX: Setting OK bit in RFD status (crc_valid=%d, SAVE_BAD_FRAMES=%d)\n",
                          crc_valid, SAVE_BAD_FRAMES));
            } else {
                DBG(printf("RX: Not setting OK bit due to CRC failure\n"));
            }

            /* Check for suspend bit in the command word */
            if (rfd_cmd & CMD_SUSP) {
                s->rx_status = RX_SUSPENDED;
                DBG(printf("RX: Suspend bit set, suspending receive unit\n"));
            }

            /* Get next RFD address for SCB update */
            next_rfd_addr = get_uint32(rfd_addr + 4);
            next_rfd_addr = i82596_translate_address(s, next_rfd_addr, false);
            DBG(printf("RX: Next RFD in chain: 0x%08x\n", next_rfd_addr));

            /* Check EL bit in the RFD command word */
            if (rfd_cmd & CMD_EOL) {
                /* This is the last RFD in the chain */
                s->rx_status = RX_NO_MORE_RBD;
                s->scb_status |= SCB_STATUS_RNR;
                DBG(printf("RX: EOL bit set in RFD, this is the last RFD\n"));
            }

            /* Update RFD status */
            set_uint16(rfd_addr, rfd_status);
            DBG(printf("RX: Final RFD status: 0x%04x\n", rfd_status));

            /* Update SCB to point to the next RFD */
            if (!(rfd_cmd & CMD_SUSP) && next_rfd_addr != 0 && next_rfd_addr != I596_NULL) {
                set_uint32(s->scb + 8, next_rfd_addr);
                DBG(printf("RX: Updated SCB to point to next RFD: 0x%08x\n", next_rfd_addr));
            }

            /* Set FR bit in the SCB status register to indicate frame received */
            s->scb_status |= SCB_STATUS_FR;
            update_scb_status(s);
            DBG(printf("RX: Set FR bit in SCB status, updated status=0x%04x\n", s->scb_status));

            /* Generate interrupt if needed */
            if (rfd_cmd & CMD_INTR) {
                DBG(printf("RX: Generating interrupt as requested by RFD\n"));
                qemu_set_irq(s->irq, 1);
            }

            DBG(printf("RX: ============= PACKET RECEPTION END (SUCCESS) =============\n"));
            return total_size;
        }
    }

    /* If we get here, we couldn't find a usable RFD */
    s->rx_status = RX_NO_RESOURCES;
    s->scb_status |= SCB_STATUS_RNR;
    DBG(printf("RX: Failed to process packet - no usable RFD found\n"));
    qemu_set_irq(s->irq, 1);
    DBG(printf("RX: ============= PACKET RECEPTION END (NO USABLE RFD) =============\n"));

    return -1;
}

ssize_t i82596_receive(NetClientState *nc, const uint8_t *buf, size_t size)
{
    const struct iovec iov = {
        .iov_base = (uint8_t *)buf,
        .iov_len = size
    };

    return i82596_receive_iov(nc, &iov, 1);
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
