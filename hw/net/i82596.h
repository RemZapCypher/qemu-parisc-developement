#ifndef HW_I82596_H
#define HW_I82596_H

#define I82596_IOPORT_SIZE       0x20

#include "exec/memory.h"
#include "exec/address-spaces.h"

#define RX_RING_SIZE    16
#define PKT_BUF_SZ      1536

#define PORT_RESET              0x00    /* reset 82596 */
#define PORT_SELFTEST           0x01    /* selftest */
#define PORT_ALTSCP             0x02    /* alternate SCB address */
#define PORT_ALTDUMP            0x03    /* Alternate DUMP address */
#define PORT_CA                 0x10    /* QEMU-internal CA signal */

typedef struct I82596State_st I82596State;

struct I82596State_st {
    MemoryRegion mmio;
    MemoryRegion *as;
    qemu_irq irq;
    NICState *nic;
    NICConf conf;
    QEMUTimer *flush_queue_timer;
    uint8_t mode;                   /* Determine the mode of 82596 */

    QEMUTimer *throttle_timer;
    uint16_t t_on;
    uint16_t t_off;
    bool throttle_state;

    hwaddr scp;                     /* pointer to SCP */
    uint32_t iscp;                  /* pointer to ISCP */
    uint8_t sysbus;
    uint32_t scb;                   /* SCB */
    uint32_t scb_base;
    uint16_t scb_status;
    uint8_t cu_status, rx_status;
    uint16_t lnkst;
    uint32_t last_tx_len;
    uint32_t collision_events;
    uint32_t total_collisions;

    /* TX retry mechanism fields */
    uint32_t tx_retry_addr;        /* Address of command being retried */
    int tx_retry_count;            /* Current retry count for TX */
    uint32_t tx_good_frames;       /* Successfully transmitted frames */
    uint32_t tx_collisions;        /* Total collision count */
    uint32_t tx_aborted_errors;    /* Aborted TX due to excessive collisions */
    
    uint32_t cmd_p;                 /* addr of current command */
    int ca;
    int ca_active;
    int send_irq;
    
    /* Hash register (multicast mask array, multiple individual addresses). */
    uint8_t mult[8];
    uint8_t config[14];             /* config bytes from CONFIGURE command */

    /* Statistical Counters */
    uint32_t crc_err;
    uint32_t align_err;
    uint32_t resource_err;
    uint32_t over_err;
    uint32_t rcvdt_err;
    uint32_t short_fr_error;
    uint32_t total_frames;
    uint32_t total_good_frames;

    uint8_t tx_buffer[PKT_BUF_SZ];
    uint8_t rx_buffer[PKT_BUF_SZ];
    uint16_t tx_frame_len;
    uint16_t rx_frame_len;
    
    hwaddr current_tx_desc;
    hwaddr current_rx_desc;
};

/* i82596 Transmit Frame Descriptor (TFD) */
struct i82596_tx_descriptor {
    uint16_t status;          /* Status word */
    uint16_t command;         /* Command word */
    uint32_t link;            /* Link to next TFD */
    uint32_t tbd_addr;        /* TBD address (or data in simplified mode) */
    uint16_t tcb_count;       /* Actual count (simplified mode) */
    uint16_t pad;             /* Padding */
    /* Flexible mode uses separate TBD chain */
    /* Simplified mode has data here */
};

/* i82596 Transmit Buffer Descriptor (TBD) */
struct i82596_tx_buffer_desc {
    uint16_t size;            /* Buffer size and EOF flag */
    uint16_t pad;             /* Padding */
    uint32_t link;            /* Link to next TBD */
    uint32_t buffer;          /* Buffer address */
};

/* i82596 Receive Frame Descriptor (RFD) */
struct i82596_rx_descriptor {
    uint16_t status;          /* Status word */
    uint16_t command;         /* Command word */
    uint32_t link;            /* Link to next RFD */
    uint32_t rbd_addr;        /* RBD address */
    uint16_t actual_count;    /* Actual count */
    uint16_t size;            /* Buffer size */
    /* Data area follows in simplified mode */
};

/* i82596 Receive Buffer Descriptor (RBD) */
struct i82596_rx_buffer_desc {
    uint16_t count;           /* Count and EOF/F flags */
    uint16_t pad;             /* Padding */
    uint32_t link;            /* Link to next RBD */
    uint32_t buffer;          /* Buffer address */
    uint16_t size;            /* Buffer size */
    uint16_t pad2;            /* Padding */
};

void i82596_h_reset(void *opaque);
void i82596_ioport_writew(void *opaque, uint32_t addr, uint32_t val);
uint32_t i82596_ioport_readw(void *opaque, uint32_t addr);
ssize_t i82596_receive(NetClientState *nc, const uint8_t *buf, size_t size_);
ssize_t i82596_receive_iov(NetClientState *nc, const struct iovec *iov, int iovcnt);
bool i82596_can_receive(NetClientState *nc);
void i82596_set_link_status(NetClientState *nc);
void i82596_poll(NetClientState *nc, bool enable);
void i82596_common_init(DeviceState *dev, I82596State *s, NetClientInfo *info);
extern const VMStateDescription vmstate_i82596;
#endif
