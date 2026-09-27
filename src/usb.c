/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Universal Serial Bus emulation (UHCI registers and root
 *          hub, dummy OHCI).
 *
 * Authors: Miran Grca, <mgrca8@gmail.com>
 *          skiretic, <skiretic@proton.me>
 *
 *          Copyright 2020 Miran Grca.
 *          Copyright 2026 skiretic.
 */
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/device.h>
#include <86box/io.h>
#include <86box/mem.h>
#include <86box/timer.h>
#include <86box/dma.h>
#include <86box/usb.h>
#include <86box/usb_dev.h>
#include <86box/usb_log.h>
#include "cpu.h"
#include <86box/plat_unused.h>

/* UHCI, written from the Intel UHCI Design Guide 1.1 and the PIIX3/PIIX4
   datasheets. */
#define UHCI_USBCMD    0x00
#define UHCI_USBSTS    0x02
#define UHCI_USBINTR   0x04
#define UHCI_FRNUM     0x06
#define UHCI_FLBASEADD 0x08
#define UHCI_SOFMOD    0x0c
#define UHCI_PORTSC1   0x10
#define UHCI_PORTSC2   0x12

#define UHCI_CMD_RS      0x0001
#define UHCI_CMD_HCRESET 0x0002
#define UHCI_CMD_GRESET  0x0004
#define UHCI_CMD_EGSM    0x0008
#define UHCI_CMD_FGR     0x0010
#define UHCI_CMD_SWDBG   0x0020
#define UHCI_CMD_CF      0x0040
#define UHCI_CMD_MAXP    0x0080

#define UHCI_STS_USBINT   0x0001
#define UHCI_STS_ERRINT   0x0002
#define UHCI_STS_RD       0x0004
#define UHCI_STS_HSE      0x0008
#define UHCI_STS_HCPE     0x0010
#define UHCI_STS_HCHALTED 0x0020

#define UHCI_INTR_TOCRC  0x0001
#define UHCI_INTR_RESUME 0x0002
#define UHCI_INTR_IOC    0x0004
#define UHCI_INTR_SP     0x0008

#define UHCI_PORT_CCS  0x0001
#define UHCI_PORT_CSC  0x0002
#define UHCI_PORT_PE   0x0004
#define UHCI_PORT_PEDC 0x0008
#define UHCI_PORT_DP   0x0010
#define UHCI_PORT_DM   0x0020
#define UHCI_PORT_RD   0x0040
#define UHCI_PORT_ONE  0x0080
#define UHCI_PORT_LSDA 0x0100
#define UHCI_PORT_PR   0x0200
#define UHCI_PORT_SUSP 0x1000

#define UHCI_PORTS 2

/* Frame list / QH / TD link pointer bits (s3.1-3.3). */
#define UHCI_LP_T  0x00000001
#define UHCI_LP_Q  0x00000002
#define UHCI_LP_VF 0x00000004

/* TD control and status, dword 1 (s3.2.2). */
#define UHCI_TD_ACTLEN   0x000007ff
#define UHCI_TD_BITSTUFF 0x00020000
#define UHCI_TD_CRCTO    0x00040000
#define UHCI_TD_NAK      0x00080000
#define UHCI_TD_BABBLE   0x00100000
#define UHCI_TD_DBE      0x00200000
#define UHCI_TD_STALLED  0x00400000
#define UHCI_TD_ACTIVE   0x00800000
#define UHCI_TD_IOC      0x01000000
#define UHCI_TD_IOS      0x02000000
#define UHCI_TD_LS       0x04000000
#define UHCI_TD_CERR     0x18000000
#define UHCI_TD_SPD      0x20000000
#define UHCI_TD_STATUS   0x00fe0000

typedef struct uhci_port_t {
    uint16_t   sc;  /* CCS, CSC, PE, PEDC, RD, LSDA, PR, SUSP */
    usb_dev_t *usb; /* the device on the wire, if any */
} uhci_port_t;

struct uhci_t {
    uint16_t    cmd;
    uint16_t    sts;
    uint16_t    intr;
    uint16_t    frnum;
    uint32_t    flbaseadd;
    uint8_t     sofmod;
    uint8_t     int_ioc; /* sources behind USBSTS.USBINT, each with its own enable */
    uint8_t     int_spd;
    uint8_t     irq_level;
    uint8_t     frame_frozen;
    uint64_t    frame_left_us;
    /* Completions of the frame in progress; they reach USBSTS at its end (s4.1). */
    uint8_t     eof_ioc;
    uint8_t     eof_spd;
    uint8_t     eof_err;
    int         bus;     /* pcap bus number */
    uhci_port_t port[UHCI_PORTS];
    pc_timer_t  frame_timer;
    pc_timer_t  detect_timer;
    void      (*set_irq)(void *priv, int level);
    void       *irq_priv;
};

#ifdef ENABLE_USB_LOG
static const char *
uhci_reg_name(uint16_t off)
{
    switch (off & 0x1e) {
        case UHCI_USBCMD:
            return "USBCMD";
        case UHCI_USBSTS:
            return "USBSTS";
        case UHCI_USBINTR:
            return "USBINTR";
        case UHCI_FRNUM:
            return "FRNUM";
        case UHCI_FLBASEADD:
        case UHCI_FLBASEADD + 2:
            return "FLBASEADD";
        case UHCI_SOFMOD:
            return "SOFMOD";
        case UHCI_PORTSC1:
            return "PORTSC1";
        case UHCI_PORTSC2:
            return "PORTSC2";
        default:
            return "reserved";
    }
}
#endif

static void
uhci_update_irq(uhci_t *dev)
{
    int level = 0;

    /* HCPE and HSE cannot be masked (UHCI 1.1 s2.1.3, s4.2.2). */
    if (dev->sts & (UHCI_STS_HCPE | UHCI_STS_HSE))
        level = 1;
    if ((dev->sts & UHCI_STS_RD) && (dev->intr & UHCI_INTR_RESUME))
        level = 1;
    if ((dev->sts & UHCI_STS_ERRINT) && (dev->intr & UHCI_INTR_TOCRC))
        level = 1;
    if ((dev->sts & UHCI_STS_USBINT) && ((dev->int_ioc && (dev->intr & UHCI_INTR_IOC)) || (dev->int_spd && (dev->intr & UHCI_INTR_SP))))
        level = 1;

    if (level == dev->irq_level)
        return;

    dev->irq_level = level;
    usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI line %s (USBSTS %04x USBINTR %04x)%s",
              level ? "assert" : "deassert", dev->sts, dev->intr, dev->set_irq ? "" : ", no route");
    if (dev->set_irq != NULL)
        dev->set_irq(dev->irq_priv, level);
}

static void
uhci_set_halted(uhci_t *dev, int halted)
{
    uint16_t old = dev->sts;

    if (halted)
        dev->sts |= UHCI_STS_HCHALTED;
    else
        dev->sts &= ~UHCI_STS_HCHALTED;

    if (dev->sts != old)
        usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x -> %04x (HCHalted %s)", old, dev->sts, halted ? "set" : "cleared");
}

static double
uhci_frame_us(const uhci_t *dev)
{
    /* SOF cycle = 11936 + SOFMOD[6:0] clocks of 12 MHz (UHCI 1.1 s2.1.6). */
    return (11936.0 + (double) (dev->sofmod & 0x7f)) / 12.0;
}

static int
uhci_running(const uhci_t *dev)
{
    return (dev->cmd & UHCI_CMD_RS) && !(dev->cmd & (UHCI_CMD_EGSM | UHCI_CMD_GRESET));
}

/* Completions of the frame reach USBSTS at its end (s4.1.2, s4.1.3). */
static void
uhci_latch_frame_status(uhci_t *dev)
{
    uint16_t old = dev->sts;

    if (dev->eof_ioc) {
        dev->sts |= UHCI_STS_USBINT;
        dev->int_ioc = 1;
    }
    if (dev->eof_spd) {
        dev->sts |= UHCI_STS_USBINT;
        dev->int_spd = 1;
    }
    if (dev->eof_err)
        dev->sts |= UHCI_STS_ERRINT;

    dev->eof_ioc = dev->eof_spd = dev->eof_err = 0;

    if (dev->sts != old) {
        usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x -> %04x (end of frame%s%s)", old, dev->sts,
                  dev->int_ioc ? ", IOC" : "", dev->int_spd ? ", SPD" : "");
        uhci_update_irq(dev);
    }
}

static void uhci_run_frame(uhci_t *dev);

/* Called after every USBCMD change: starts or stops the frame clock. */
static void
uhci_update_frame_clock(uhci_t *dev, int was_running)
{
    int running = uhci_running(dev);

    if (running == was_running)
        return;

    if (running) {
        if (dev->frame_frozen && (dev->cmd & UHCI_CMD_SWDBG))
            timer_on_auto(&dev->frame_timer, (double) dev->frame_left_us);
        else
            timer_on_auto(&dev->frame_timer, uhci_frame_us(dev));
        dev->frame_frozen = 0;
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI frame clock started, FLBASEADD %08x", dev->flbaseadd);
        uhci_run_frame(dev);
    } else {
        /* Outside debug mode a stop resets the SOF counter; in debug mode it freezes (Table 4). */
        dev->frame_frozen  = !!(dev->cmd & UHCI_CMD_SWDBG);
        dev->frame_left_us = timer_get_remaining_us(&dev->frame_timer);
        timer_stop(&dev->frame_timer);
        uhci_latch_frame_status(dev);
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI frame clock stopped%s", dev->frame_frozen ? " (frozen, SWDBG)" : "");
    }
}

/* The controller clears Run/Stop itself: process error or debug single step (Table 7). */
static void
uhci_self_stop(uhci_t *dev, uint16_t sts_bit)
{
    int was_running = uhci_running(dev);

    dev->cmd &= ~UHCI_CMD_RS;
    if (sts_bit != 0) {
        dev->sts |= sts_bit;
        usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x (%s), Run/Stop cleared", dev->sts,
                  (sts_bit == UHCI_STS_HCPE) ? "process error" : "host system error");
    }
    uhci_set_halted(dev, 1);
    uhci_update_frame_clock(dev, was_running);
    uhci_update_irq(dev);
}

/* Bus time of one transaction in ns (USB 1.1 s5.11.3), Host_Delay 0 and
   Hub_LS_Setup at its minimum of four full-speed bit times. */
static double
uhci_bus_ns(int low_speed, int iso, int in, int bytes)
{
    double bits = floor(3.167 + (1.1667 * 8.0 * (double) bytes));

    if (low_speed)
        return (in ? 64060.0 : 64107.0) + (2.0 * 333.0) + ((in ? 676.67 : 667.0) * bits);
    if (iso)
        return (in ? 7268.0 : 6265.0) + (83.54 * bits);

    return 9107.0 + (83.54 * bits);
}

/* One descriptor fetch, about one PCI burst read; it also bounds a loop
   of empty queue heads to the frame. */
#define UHCI_FETCH_NS 250.0

static usb_dev_t *
uhci_find_device(uhci_t *dev, int addr, int low_speed)
{
    for (int i = 0; i < UHCI_PORTS; i++) {
        const uhci_port_t *port = &dev->port[i];

        /* Only an enabled, unsuspended port carries traffic, and a low-speed
           token only reaches a low-speed device. */
        if ((port->usb == NULL) || !(port->sc & UHCI_PORT_PE) || (port->sc & (UHCI_PORT_SUSP | UHCI_PORT_PR)))
            continue;
        if ((port->usb->addr == addr) && (port->usb->low_speed == low_speed))
            return port->usb;
    }

    return NULL;
}

enum {
    TD_INACTIVE = 0,
    TD_ADVANCE,   /* completed: the queue moves on */
    TD_SHORT,     /* short packet with SPD: inactive, queue not advanced */
    TD_RETRY,     /* left active: NAK or a counted error */
    TD_FAILED,    /* inactive with an error: queue not advanced */
    TD_END_FRAME, /* babble ends the frame */
    TD_NO_TIME,   /* not started: past the point where it could finish */
    TD_HALT       /* process error: Run/Stop cleared */
};

#ifdef ENABLE_USB_LOG
static const char *const uhci_ret_names[] = { "ACK", "NAK", "STALL", "timeout", "babble", "toggle mismatch" };
static const uint8_t     usbmon_xfer[4]   = { 2, 0, 3, 1 };
#endif

static int
uhci_exec_td(uhci_t *dev, uint32_t addr, int in_queue, double *left, uint32_t *link)
{
    uint32_t     td[4];
    uint32_t     ctrl;
    uint8_t      buf[1280];
    usb_packet_t p;
    usb_dev_t   *usb;
    int          pid;
    int          len;
    int          ls;
    int          iso;
    int          cerr;
    int          ret;
    int          result;
    double       cost;
    double       maxp_ns;

    dma_bm_read(addr, (uint8_t *) td, 16, 4);
    *link = td[0];
    ctrl  = td[1];

    /* IOC fires at the end of the frame for any TD found inactive (s3.2.2 bit 24). */
    if (!(ctrl & UHCI_TD_ACTIVE)) {
        if (ctrl & UHCI_TD_IOC)
            dev->eof_ioc = 1;
        usb_trace(USB_LOG_SCHED, dev->frnum, "TD %08x inactive, ctrl %08x", addr, ctrl);
        return TD_INACTIVE;
    }

    pid = td[2] & 0xff;
    len = (td[2] >> 21) & 0x7ff;

    /* Illegal PID or MaxLen 500h-7FEh: consistency check failure (s3.2.3, s4.2.2). */
    if (((pid != USB_PID_IN) && (pid != USB_PID_OUT) && (pid != USB_PID_SETUP)) ||
        ((len >= 0x500) && (len != 0x7ff))) {
        usb_trace(USB_LOG_SCHED, dev->frnum, "TD %08x token %08x: process error", addr, td[2]);
        uhci_self_stop(dev, UHCI_STS_HCPE);
        return TD_HALT;
    }

    len = (len + 1) & 0x7ff;
    ls  = !!(ctrl & UHCI_TD_LS);
    iso = !!(ctrl & UHCI_TD_IOS);

    /* Nothing starts that cannot finish before EOF, nor inside the window
       kept for a MAXP-sized packet (s1.3.1 cases A and B). */
    cost    = uhci_bus_ns(ls, iso, pid == USB_PID_IN, len);
    maxp_ns = uhci_bus_ns(0, 0, 1, (dev->cmd & UHCI_CMD_MAXP) ? 64 : 32);
    if (*left < ((cost > maxp_ns) ? cost : maxp_ns))
        return TD_NO_TIME;

    p.pid    = pid;
    p.ep     = (td[2] >> 15) & 0x0f;
    p.toggle = (td[2] >> 19) & 1;
    p.data   = buf;
    p.len    = len;
    p.actual = 0;

    if ((pid != USB_PID_IN) && (len > 0))
        dma_bm_read(td[3], buf, len, 1);

    usb = uhci_find_device(dev, (td[2] >> 8) & 0x7f, ls);
    ret = (usb != NULL) ? usb_dev_packet(usb, &p) : USB_RET_NO_RESPONSE;

    /* An isochronous OUT has no handshake (USB 1.1 s8.5.4): once token and
       data are sent the transaction is over, whoever took the data. */
    if (iso && (pid == USB_PID_OUT))
        ret = USB_RET_ACK;

    /* OUT data is on the bus whatever the handshake; IN data only if sent. */
    *left -= uhci_bus_ns(ls, iso, pid == USB_PID_IN, (pid == USB_PID_IN) ? p.actual : len);

    /* Each execution rewrites the status field (s3.2.2 bits 23:16). */
    ctrl &= ~(UHCI_TD_STATUS & ~UHCI_TD_ACTIVE);
    result = TD_RETRY;

    switch (ret) {
        case USB_RET_ACK:
            if (pid == USB_PID_IN) {
                if (p.actual > 0)
                    dma_bm_write(td[3], buf, p.actual, 1);
                len = p.actual;
            }
            /* ActLen is n-1 encoded: 7FFh is zero bytes. */
            ctrl   = (ctrl & ~(UHCI_TD_ACTIVE | UHCI_TD_ACTLEN)) | ((len - 1) & UHCI_TD_ACTLEN);
            result = TD_ADVANCE;
            if ((pid == USB_PID_IN) && in_queue && (ctrl & UHCI_TD_SPD) && (p.actual < p.len)) {
                dev->eof_spd = 1;
                result       = TD_SHORT;
            }
            break;

        case USB_RET_TOGGLE_MISMATCH:
            /* ACKed and discarded (USB 1.1 s8.6.4): the TD waits for the right packet. */
            break;

        case USB_RET_NAK:
            ctrl |= UHCI_TD_NAK;
            break;

        case USB_RET_STALL:
            ctrl   = (ctrl & ~UHCI_TD_ACTIVE) | UHCI_TD_STALLED;
            result = TD_FAILED;
            dev->eof_err = 1;
            break;

        case USB_RET_BABBLE:
            if (p.actual > 0)
                dma_bm_write(td[3], buf, p.actual, 1);
            ctrl   = (ctrl & ~(UHCI_TD_ACTIVE | UHCI_TD_ACTLEN)) | UHCI_TD_BABBLE | UHCI_TD_STALLED |
                     ((p.actual - 1) & UHCI_TD_ACTLEN);
            result = TD_END_FRAME;
            dev->eof_err = 1;
            break;

        default:
            /* No handshake: CRC/timeout, counted down by C_ERR; 0 retries forever (s3.2.2). */
            ctrl |= UHCI_TD_CRCTO;
            cerr = (ctrl & UHCI_TD_CERR) >> 27;
            if (cerr != 0) {
                cerr--;
                ctrl = (ctrl & ~UHCI_TD_CERR) | (cerr << 27);
                if (cerr == 0) {
                    ctrl   = (ctrl & ~UHCI_TD_ACTIVE) | UHCI_TD_STALLED;
                    result = TD_FAILED;
                    dev->eof_err = 1;
                }
            }
            break;
    }

    /* Isochronous TDs go inactive after one execution whatever happened (s3.2.2 bit 25). */
    if (iso && (ctrl & UHCI_TD_ACTIVE)) {
        ctrl &= ~UHCI_TD_ACTIVE;
        result = TD_FAILED;
    }

    if (!(ctrl & UHCI_TD_ACTIVE) && (ctrl & UHCI_TD_IOC))
        dev->eof_ioc = 1;

    dma_bm_write(addr + 4, (uint8_t *) &ctrl, 4, 4);

    usb_trace(USB_LOG_SCHED, dev->frnum, "TD %08x %s dev %i ep %i D%i len %i%s -> %s, ctrl %08x",
              addr, (pid == USB_PID_IN) ? "IN" : ((pid == USB_PID_OUT) ? "OUT" : "SETUP"),
              (td[2] >> 8) & 0x7f, p.ep, p.toggle, p.len, ls ? " LS" : "",
              uhci_ret_names[ret], ctrl);

#ifdef ENABLE_USB_LOG
    if ((usb != NULL) && (ret != USB_RET_NAK)) {
        int xfer = usbmon_xfer[((pid == USB_PID_IN) ? usb->ep_in[p.ep].type : usb->ep_out[p.ep].type) & 3];

        if (pid == USB_PID_SETUP)
            usb_log_pcap(addr, 'S', xfer, buf[0] >> 7, p.ep, dev->bus, usb->addr, -115, buf, NULL, 0);
        else if (pid == USB_PID_OUT)
            usb_log_pcap(addr, 'S', xfer, 0, p.ep, dev->bus, usb->addr, -115, NULL, buf, len);
        else
            usb_log_pcap(addr, 'C', xfer, 1, p.ep, dev->bus, usb->addr,
                         (ret == USB_RET_STALL) ? -32 : 0, NULL, buf, (ret == USB_RET_ACK) ? p.actual : 0);
    }
#endif

    return result;
}

/* One frame of the schedule (s3.4): the frame list entry, then TDs and
   queue heads by their link pointers until a terminate bit or the end of
   the frame. */
static void
uhci_run_frame(uhci_t *dev)
{
    uint32_t link;
    uint32_t qh[2];
    uint32_t td_link;
    uint32_t seen_qh[32];
    int      seen_work[32];
    int      n_seen = 0;
    int      work   = 0;
    int      steps  = 0;
    int      r;
    double   left = uhci_frame_us(dev) * 1000.0;

    if (!uhci_running(dev))
        return;

    dma_bm_read(dev->flbaseadd + ((dev->frnum & 0x3ff) << 2), (uint8_t *) &link, 4, 4);
    usb_trace(USB_LOG_SCHED, dev->frnum, "frame list entry %08x", link);

    while (!(link & UHCI_LP_T) && (steps++ < 4096)) {
        left -= UHCI_FETCH_NS;

        if (link & UHCI_LP_Q) {
            uint32_t qh_addr = link & ~0x0f;
            int      i;

            /* A queue head reached again with no transaction since the last
               visit is a reclamation loop with nothing left to run. */
            for (i = 0; i < n_seen; i++) {
                if (seen_qh[i] == qh_addr)
                    break;
            }
            if (i < n_seen) {
                if (seen_work[i] == work)
                    break;
                seen_work[i] = work;
            } else if (n_seen < 32) {
                seen_qh[n_seen]   = qh_addr;
                seen_work[n_seen] = work;
                n_seen++;
            }

            dma_bm_read(qh_addr, (uint8_t *) qh, 8, 4);
            usb_trace(USB_LOG_SCHED, dev->frnum, "QH %08x link %08x element %08x", qh_addr, qh[0], qh[1]);

            if (qh[1] & UHCI_LP_T) {
                link = qh[0];
                continue;
            }
            /* An element that is a QH starts a new queue context; the one
               just left is not written back (s3.4.1.3). */
            if (qh[1] & UHCI_LP_Q) {
                link = qh[1];
                continue;
            }

            r = uhci_exec_td(dev, qh[1] & ~0x0f, 1, &left, &td_link);
            if (r >= TD_END_FRAME)
                break;
            if ((r != TD_INACTIVE) && (r != TD_NO_TIME))
                work++;

            if (r == TD_ADVANCE) {
                /* The TD's link becomes the queue element (s3.4.1.3 step 9). */
                dma_bm_write(qh_addr + 4, (uint8_t *) &td_link, 4, 4);
                /* Depth first stays in this queue; else on to the next QH. */
                link = ((td_link & UHCI_LP_VF) && !(td_link & UHCI_LP_T)) ? (qh_addr | UHCI_LP_Q) : qh[0];
            } else
                link = qh[0];
        } else {
            r = uhci_exec_td(dev, link & ~0x0f, 0, &left, &td_link);
            if (r >= TD_END_FRAME)
                break;
            if ((r != TD_INACTIVE) && (r != TD_NO_TIME))
                work++;
            link = td_link;
        }

        /* Debug mode stops after each executed TD (s2.1.1.1). */
        if ((dev->cmd & UHCI_CMD_SWDBG) && (r != TD_INACTIVE)) {
            uhci_self_stop(dev, 0);
            break;
        }

        if (left <= 0.0)
            break;
    }
}

static void
uhci_frame_end(void *priv)
{
    uhci_t *dev = (uhci_t *) priv;

    uhci_latch_frame_status(dev);
    dev->frnum = (dev->frnum + 1) & 0x07ff;
    timer_on_auto(&dev->frame_timer, uhci_frame_us(dev));
    uhci_run_frame(dev);
}

static int
uhci_port_in_reset(const uhci_t *dev, const uhci_port_t *port)
{
    return (port->sc & UHCI_PORT_PR) || (dev->cmd & UHCI_CMD_GRESET);
}

/* Connect detection: an attached device is seen about 64 bit times after SE0 ends. */
static void
uhci_detect(void *priv)
{
    uhci_t *dev = (uhci_t *) priv;

    for (int i = 0; i < UHCI_PORTS; i++) {
        uhci_port_t *port = &dev->port[i];

        if ((port->usb == NULL) || (port->sc & UHCI_PORT_CCS) || uhci_port_in_reset(dev, port))
            continue;

        port->sc |= UHCI_PORT_CCS | UHCI_PORT_CSC;
        if (port->usb->low_speed)
            port->sc |= UHCI_PORT_LSDA;
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI port %i connect detected, PORTSC %04x", i + 1, port->sc);

        /* A connect in global suspend is a resume event (Table 6; USBCMD bit 4). */
        if (dev->cmd & UHCI_CMD_EGSM) {
            dev->cmd |= UHCI_CMD_FGR;
            dev->sts |= UHCI_STS_RD;
            usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS RD set (connect in global suspend)");
        }
    }

    uhci_update_irq(dev);
}

static void
uhci_schedule_detect(uhci_t *dev)
{
    for (int i = 0; i < UHCI_PORTS; i++) {
        if (dev->port[i].usb != NULL) {
            timer_on_auto(&dev->detect_timer, 64.0 / 12.0);
            return;
        }
    }
}

/* Power-on state; chip reset and GRESET both land here (UHCI 1.1 s2.1.1 bit 2). */
static void
uhci_power_on_reset(uhci_t *dev)
{
    timer_stop(&dev->frame_timer);
    timer_stop(&dev->detect_timer);

    if (dev->sts != 0x0000)
        usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x -> 0000 (reset)", dev->sts);

    dev->cmd          = 0x0000;
    dev->sts          = 0x0000;
    dev->intr         = 0x0000;
    dev->frnum        = 0x0000;
    dev->flbaseadd    = 0x00000000;
    dev->sofmod       = 0x40;
    dev->int_ioc      = 0;
    dev->int_spd      = 0;
    dev->eof_ioc      = 0;
    dev->eof_spd      = 0;
    dev->eof_err      = 0;
    dev->frame_frozen = 0;

    for (int i = 0; i < UHCI_PORTS; i++)
        dev->port[i].sc = 0x0000;

    uhci_update_irq(dev);
}

/* HCRESET: counters and state machines only; USBINTR and FLBASEADD survive.
   The per-port connect state machine resets, a virtual disconnect (s2.1.1 bit 1). */
static void
uhci_hc_reset(uhci_t *dev)
{
    timer_stop(&dev->frame_timer);

    if (dev->sts != 0x0000)
        usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x -> 0000 (HCRESET)", dev->sts);

    dev->cmd          = 0x0000;
    dev->sts          = 0x0000;
    dev->frnum        = 0x0000;
    dev->sofmod       = 0x40;
    dev->int_ioc      = 0;
    dev->int_spd      = 0;
    dev->eof_ioc      = 0;
    dev->eof_spd      = 0;
    dev->eof_err      = 0;
    dev->frame_frozen = 0;

    for (int i = 0; i < UHCI_PORTS; i++) {
        uhci_port_t *port = &dev->port[i];
        uint16_t     old  = port->sc;

        port->sc &= ~(UHCI_PORT_CCS | UHCI_PORT_PE | UHCI_PORT_LSDA | UHCI_PORT_SUSP | UHCI_PORT_RD);
        if (old & UHCI_PORT_CCS)
            port->sc |= UHCI_PORT_CSC;
        if (old & UHCI_PORT_PE)
            port->sc |= UHCI_PORT_PEDC;
    }

    usb_trace(USB_LOG_REG, dev->frnum, "UHCI HCRESET done, PORTSC1 %04x PORTSC2 %04x", dev->port[0].sc, dev->port[1].sc);
    uhci_update_irq(dev);
    uhci_schedule_detect(dev);
}

static void
uhci_cmd_write(uhci_t *dev, uint16_t val)
{
    uint16_t old         = dev->cmd;
    int      was_running = uhci_running(dev);

    val &= 0x00ff;

    if (val & UHCI_CMD_GRESET) {
        if (!(old & UHCI_CMD_GRESET)) {
            uhci_power_on_reset(dev);
            /* Global reset signaling reaches every device on the bus. */
            for (int i = 0; i < UHCI_PORTS; i++) {
                if (dev->port[i].usb != NULL)
                    usb_dev_bus_reset(dev->port[i].usb);
            }
            usb_trace(USB_LOG_REG, dev->frnum, "UHCI GRESET start");
        }
        dev->cmd = UHCI_CMD_GRESET;
        return;
    }

    if (old & UHCI_CMD_GRESET) {
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI GRESET end");
        uhci_schedule_detect(dev);
    }

    if (val & UHCI_CMD_HCRESET) {
        /* Completes at once, so the self-clearing bit already reads 0. */
        uhci_hc_reset(dev);
        return;
    }

    dev->cmd = val;

    /* RS written 0 sets HCHalted (Table 7); RS written 1 clears it. HCHalted is
       not write-1-to-clear on Intel silicon (Linux uhci-hcd.c). */
    uhci_set_halted(dev, !(val & UHCI_CMD_RS));
    uhci_update_frame_clock(dev, was_running);
}

static uint16_t
uhci_portsc_read(const uhci_t *dev, int n)
{
    const uhci_port_t *port = &dev->port[n];
    uint16_t           ret  = port->sc | UHCI_PORT_ONE;
    uint16_t           j;

    /* Line status: SE0 with nothing attached or in reset, else idle J, or K
       while resume is driven. J is D+ at full speed, D- at low speed. */
    if ((port->sc & UHCI_PORT_CCS) && !uhci_port_in_reset(dev, port)) {
        j = port->usb->low_speed ? UHCI_PORT_DM : UHCI_PORT_DP;
        if (((port->sc & (UHCI_PORT_RD | UHCI_PORT_SUSP)) == (UHCI_PORT_RD | UHCI_PORT_SUSP)) ||
            ((dev->cmd & UHCI_CMD_FGR) && (port->sc & UHCI_PORT_PE)))
            j ^= (UHCI_PORT_DP | UHCI_PORT_DM);
        ret |= j;
    }

    return ret;
}

static void
uhci_portsc_write(uhci_t *dev, int n, uint16_t val)
{
    uhci_port_t *port = &dev->port[n];
    uint16_t     old  = port->sc;

    port->sc &= ~(val & (UHCI_PORT_CSC | UHCI_PORT_PEDC));

    if ((val & UHCI_PORT_PR) && !(old & UHCI_PORT_PR)) {
        /* Reset disables the port. Intel silicon reports that as an enable
           change and the end of reset as a connect change (Linux uhci-hub.c). */
        if (old & UHCI_PORT_PE)
            port->sc |= UHCI_PORT_PEDC;
        port->sc = (port->sc & ~(UHCI_PORT_PE | UHCI_PORT_SUSP | UHCI_PORT_RD)) | UHCI_PORT_PR;
        if (port->usb != NULL)
            usb_dev_bus_reset(port->usb);
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI port %i reset start", n + 1);
    } else if (!(val & UHCI_PORT_PR) && (old & UHCI_PORT_PR)) {
        port->sc &= ~UHCI_PORT_PR;
        if (port->sc & UHCI_PORT_CCS)
            port->sc |= UHCI_PORT_CSC;
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI port %i reset end", n + 1);
    }

    if (!(port->sc & UHCI_PORT_PR)) {
        /* Only software enables a port, and only a connected one (USB 1.1 s11.5.1). */
        if ((val & UHCI_PORT_PE) && (port->sc & UHCI_PORT_CCS))
            port->sc |= UHCI_PORT_PE;
        else if (!(val & UHCI_PORT_PE))
            port->sc &= ~UHCI_PORT_PE;

        /* Suspend is a state of an enabled port ([12,2] = 11). */
        if ((val & UHCI_PORT_SUSP) && (port->sc & UHCI_PORT_PE))
            port->sc |= UHCI_PORT_SUSP;
        else
            port->sc &= ~UHCI_PORT_SUSP;

        port->sc = (port->sc & ~UHCI_PORT_RD) | (val & UHCI_PORT_RD);
    }
}

static uint16_t
uhci_read16(const uhci_t *dev, uint16_t off)
{
    switch (off & 0x1e) {
        case UHCI_USBCMD:
            return dev->cmd;
        case UHCI_USBSTS:
            return dev->sts;
        case UHCI_USBINTR:
            return dev->intr;
        case UHCI_FRNUM:
            return dev->frnum;
        case UHCI_FLBASEADD:
            return dev->flbaseadd & 0xffff;
        case UHCI_FLBASEADD + 2:
            return dev->flbaseadd >> 16;
        case UHCI_SOFMOD:
            return dev->sofmod;
        case UHCI_PORTSC1:
        case UHCI_PORTSC2:
            return uhci_portsc_read(dev, ((off & 0x1e) - UHCI_PORTSC1) >> 1);
        default:
            /* 0Dh-0Fh and 14h-1Fh are not implemented; bit 7 = 0 at 14h ends
               the driver's port count at two. */
            return 0x0000;
    }
}

/* lanes selects the bytes the guest wrote. Unwritten bytes keep their R/W
   bits and write 0 to the write-1-to-clear bits. Word-only registers (FRNUM,
   PORTSC, PIIX4 USBCMD) have no defined byte write; this merge is the model. */
static void
uhci_write16(uhci_t *dev, uint16_t off, uint16_t val, uint16_t lanes)
{
    uint16_t cur;
    int      n;

    off &= 0x1e;

    switch (off) {
        case UHCI_USBCMD:
            if (lanes & 0x00ff)
                uhci_cmd_write(dev, (dev->cmd & ~lanes) | (val & lanes));
            break;

        case UHCI_USBSTS:
            val &= lanes & (UHCI_STS_USBINT | UHCI_STS_ERRINT | UHCI_STS_RD | UHCI_STS_HSE | UHCI_STS_HCPE);
            if (val & dev->sts) {
                usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x -> %04x (guest clear %04x)",
                          dev->sts, dev->sts & ~val, val);
                dev->sts &= ~val;
                if (val & UHCI_STS_USBINT)
                    dev->int_ioc = dev->int_spd = 0;
                uhci_update_irq(dev);
            }
            break;

        case UHCI_USBINTR:
            dev->intr = ((dev->intr & ~lanes) | (val & lanes)) & 0x000f;
            uhci_update_irq(dev);
            break;

        case UHCI_FRNUM:
            /* A write while Run/Stop is set is ignored (s2.1.4). */
            if (dev->cmd & UHCI_CMD_RS)
                usb_trace(USB_LOG_REG, dev->frnum, "UHCI FRNUM write ignored, RS=1");
            else
                dev->frnum = ((dev->frnum & ~lanes) | (val & lanes)) & 0x07ff;
            break;

        case UHCI_FLBASEADD:
            dev->flbaseadd = ((dev->flbaseadd & ~((uint32_t) lanes)) | (val & lanes)) & 0xfffff000;
            break;

        case UHCI_FLBASEADD + 2:
            dev->flbaseadd = ((dev->flbaseadd & ~((uint32_t) lanes << 16)) | ((uint32_t) (val & lanes) << 16)) & 0xfffff000;
            break;

        case UHCI_SOFMOD:
            if (lanes & 0x00ff)
                dev->sofmod = val & 0x7f;
            break;

        case UHCI_PORTSC1:
        case UHCI_PORTSC2:
            n   = (off - UHCI_PORTSC1) >> 1;
            cur = dev->port[n].sc & ~(UHCI_PORT_CSC | UHCI_PORT_PEDC);
            uhci_portsc_write(dev, n, (cur & ~lanes) | (val & lanes));
            break;

        default:
            break;
    }
}

static uint8_t
uhci_inb(uint16_t addr, void *priv)
{
    const uhci_t *dev = (uhci_t *) priv;
    uint16_t      off = addr & 0x1f;
    uint8_t       ret = uhci_read16(dev, off) >> ((off & 1) << 3);

    usb_trace(USB_LOG_REG, dev->frnum, "R8  +%02x %-9s = %02x", off, uhci_reg_name(off), ret);

    return ret;
}

static uint16_t
uhci_inw(uint16_t addr, void *priv)
{
    const uhci_t *dev = (uhci_t *) priv;
    uint16_t      off = addr & 0x1f;
    uint16_t      ret;

    if (off & 1)
        ret = (uhci_read16(dev, off) >> 8) | (uhci_read16(dev, off + 1) << 8);
    else
        ret = uhci_read16(dev, off);

    usb_trace(USB_LOG_REG, dev->frnum, "R16 +%02x %-9s = %04x", off, uhci_reg_name(off), ret);

    return ret;
}

static uint32_t
uhci_inl(uint16_t addr, void *priv)
{
    const uhci_t *dev = (uhci_t *) priv;
    uint16_t      off = addr & 0x1c;
    uint32_t      ret = uhci_read16(dev, off) | ((uint32_t) uhci_read16(dev, off + 2) << 16);

    usb_trace(USB_LOG_REG, dev->frnum, "R32 +%02x %-9s = %08x", off, uhci_reg_name(off), ret);

    return ret;
}

static void
uhci_outb(uint16_t addr, uint8_t val, void *priv)
{
    uhci_t  *dev = (uhci_t *) priv;
    uint16_t off = addr & 0x1f;

    usb_trace(USB_LOG_REG, dev->frnum, "W8  +%02x %-9s <- %02x", off, uhci_reg_name(off), val);

    if (off & 1)
        uhci_write16(dev, off, val << 8, 0xff00);
    else
        uhci_write16(dev, off, val, 0x00ff);
}

static void
uhci_outw(uint16_t addr, uint16_t val, void *priv)
{
    uhci_t  *dev = (uhci_t *) priv;
    uint16_t off = addr & 0x1f;

    usb_trace(USB_LOG_REG, dev->frnum, "W16 +%02x %-9s <- %04x", off, uhci_reg_name(off), val);

    if (off & 1) {
        uhci_write16(dev, off, val << 8, 0xff00);
        uhci_write16(dev, off + 1, val >> 8, 0x00ff);
    } else
        uhci_write16(dev, off, val, 0xffff);
}

static void
uhci_outl(uint16_t addr, uint32_t val, void *priv)
{
    uhci_t  *dev = (uhci_t *) priv;
    uint16_t off = addr & 0x1c;

    usb_trace(USB_LOG_REG, dev->frnum, "W32 +%02x %-9s <- %08x", off, uhci_reg_name(off), val);

    uhci_write16(dev, off, val & 0xffff, 0xffff);
    uhci_write16(dev, off + 2, val >> 16, 0xffff);
}

void
uhci_update_io_mapping(usb_t *dev, uint8_t base_l, uint8_t base_h, int enable)
{
    if (dev->uhci_enable && (dev->uhci_io_base != 0x0000))
        io_removehandler(dev->uhci_io_base, 0x20, uhci_inb, uhci_inw, uhci_inl, uhci_outb, uhci_outw, uhci_outl, dev->uhci);

    dev->uhci_io_base = base_l | (base_h << 8);
    dev->uhci_enable  = enable;

    if (dev->uhci_enable && (dev->uhci_io_base != 0x0000))
        io_sethandler(dev->uhci_io_base, 0x20, uhci_inb, uhci_inw, uhci_inl, uhci_outb, uhci_outw, uhci_outl, dev->uhci);

    usb_trace(USB_LOG_REG, -1, "UHCI I/O %04x %s", dev->uhci_io_base, dev->uhci_enable ? "enabled" : "disabled");
}

void
uhci_set_irq_handler(usb_t *dev, void (*set_irq)(void *priv, int level), void *priv)
{
    dev->uhci->set_irq  = set_irq;
    dev->uhci->irq_priv = priv;
}

#define USB_MAX_HC 8

static usb_t *usb_hcs[USB_MAX_HC];

/* Only a UHCI the chipset wired up (it routes its interrupt) takes devices. */
int
usb_attach(usb_dev_t *udev)
{
    for (int h = 0; h < USB_MAX_HC; h++) {
        uhci_t *hc = (usb_hcs[h] != NULL) ? usb_hcs[h]->uhci : NULL;

        if ((hc == NULL) || (hc->set_irq == NULL))
            continue;

        for (int i = 0; i < UHCI_PORTS; i++) {
            if (hc->port[i].usb == NULL) {
                hc->port[i].usb = udev;
                udev->port      = &hc->port[i];
                usb_trace(USB_LOG_REG, hc->frnum, "UHCI port %i: %s attached", i + 1, udev->name);
                uhci_schedule_detect(hc);
                return 1;
            }
        }
    }

    return 0;
}

/* A disconnect clears CCS and PE and sets CSC and PEDC (s2.1.7, Tables 5-6). */
void
usb_detach(usb_dev_t *udev)
{
    for (int h = 0; h < USB_MAX_HC; h++) {
        uhci_t *hc = (usb_hcs[h] != NULL) ? usb_hcs[h]->uhci : NULL;

        if (hc == NULL)
            continue;

        for (int i = 0; i < UHCI_PORTS; i++) {
            uhci_port_t *port = &hc->port[i];

            if (port->usb != udev)
                continue;

            port->usb = NULL;
            if (port->sc & UHCI_PORT_CCS)
                port->sc |= UHCI_PORT_CSC;
            if (port->sc & UHCI_PORT_PE)
                port->sc |= UHCI_PORT_PEDC;
            port->sc &= ~(UHCI_PORT_CCS | UHCI_PORT_PE | UHCI_PORT_LSDA | UHCI_PORT_SUSP | UHCI_PORT_RD);
            if (hc->cmd & UHCI_CMD_EGSM) {
                hc->cmd |= UHCI_CMD_FGR;
                hc->sts |= UHCI_STS_RD;
            }
            usb_trace(USB_LOG_REG, hc->frnum, "UHCI port %i: %s detached, PORTSC %04x", i + 1, udev->name, port->sc);
            uhci_update_irq(hc);
        }
    }

    udev->port = NULL;
}

static uint8_t
ohci_mmio_read(uint32_t addr, void *priv)
{
    const usb_t  *dev = (usb_t *) priv;
    uint8_t       ret = 0x00;

    addr &= 0x00000fff;

    ret = dev->ohci_mmio[addr];

    if (addr == 0x101)
        ret = (ret & 0xfe) | (!!mem_a20_key);

    return ret;
}

static void
ohci_mmio_write(uint32_t addr, uint8_t val, void *priv)
{
    usb_t  *dev = (usb_t *) priv;
    uint8_t old;

    addr &= 0x00000fff;

    switch (addr) {
        case 0x04:
            if ((val & 0xc0) == 0x00) {
                /* UsbReset */
                dev->ohci_mmio[0x56] = dev->ohci_mmio[0x5a] = 0x16;
            }
            break;
        case 0x08: /* HCCOMMANDSTATUS */
            /* bit OwnershipChangeRequest triggers an ownership change (SMM <-> OS) */
            if (val & 0x08) {
                dev->ohci_mmio[0x0f] = 0x40;
                if ((dev->ohci_mmio[0x13] & 0xc0) == 0xc0)
                    smi_raise();
            }

            /* bit HostControllerReset must be cleared for the controller to be seen as initialized */
            if (val & 0x01) {
                memset(dev->ohci_mmio, 0x00, 4096);
                dev->ohci_mmio[0x00] = 0x10;
                dev->ohci_mmio[0x01] = 0x01;
                dev->ohci_mmio[0x48] = 0x02;
                val &= ~0x01;
            }
            break;
        case 0x0c:
            dev->ohci_mmio[addr] &= ~(val & 0x7f);
            return;
        case 0x0d:
        case 0x0e:
            return;
        case 0x0f:
            dev->ohci_mmio[addr] &= ~(val & 0x40);
            return;
        case 0x3b:
            dev->ohci_mmio[addr] = (val & 0x80);
            return;
        case 0x39:
        case 0x41:
            dev->ohci_mmio[addr] = (val & 0x3f);
            return;
        case 0x45:
            dev->ohci_mmio[addr] = (val & 0x0f);
            return;
        case 0x3a:
        case 0x3e:
        case 0x3f:
        case 0x42:
        case 0x43:
        case 0x46:
        case 0x47:
        case 0x48:
        case 0x4a:
            return;
        case 0x49:
            dev->ohci_mmio[addr] = (val & 0x1b);
            if (val & 0x02) {
                dev->ohci_mmio[0x55] |= 0x01;
                dev->ohci_mmio[0x59] |= 0x01;
            }
            return;
        case 0x4b:
            dev->ohci_mmio[addr] = (val & 0x03);
            return;
        case 0x4c:
        case 0x4e:
            dev->ohci_mmio[addr] = (val & 0x06);
            if ((addr == 0x4c) && !(val & 0x04)) {
                if (!(dev->ohci_mmio[0x58] & 0x01))
                    dev->ohci_mmio[0x5a] |= 0x01;
                dev->ohci_mmio[0x58] |= 0x01;
            }
            if ((addr == 0x4c) && !(val & 0x02)) {
                if (!(dev->ohci_mmio[0x54] & 0x01))
                    dev->ohci_mmio[0x56] |= 0x01;
                dev->ohci_mmio[0x54] |= 0x01;
            }
            return;
        case 0x4d:
        case 0x4f:
            return;
        case 0x50:
            if (val & 0x01) {
                if ((dev->ohci_mmio[0x49] & 0x03) == 0x00) {
                    dev->ohci_mmio[0x55] &= ~0x01;
                    dev->ohci_mmio[0x54] &= ~0x17;
                    dev->ohci_mmio[0x56] &= ~0x17;
                    dev->ohci_mmio[0x59] &= ~0x01;
                    dev->ohci_mmio[0x58] &= ~0x17;
                    dev->ohci_mmio[0x5a] &= ~0x17;
                } else if ((dev->ohci_mmio[0x49] & 0x03) == 0x01) {
                    if (!(dev->ohci_mmio[0x4e] & 0x02)) {
                        dev->ohci_mmio[0x55] &= ~0x01;
                        dev->ohci_mmio[0x54] &= ~0x17;
                        dev->ohci_mmio[0x56] &= ~0x17;
                    }
                    if (!(dev->ohci_mmio[0x4e] & 0x04)) {
                        dev->ohci_mmio[0x59] &= ~0x01;
                        dev->ohci_mmio[0x58] &= ~0x17;
                        dev->ohci_mmio[0x5a] &= ~0x17;
                    }
                }
            }
            return;
        case 0x51:
            if (val & 0x80)
                dev->ohci_mmio[addr] |= 0x80;
            return;
        case 0x52:
            dev->ohci_mmio[addr] &= ~(val & 0x02);
            if (val & 0x01) {
                if ((dev->ohci_mmio[0x49] & 0x03) == 0x00) {
                    dev->ohci_mmio[0x55] |= 0x01;
                    dev->ohci_mmio[0x59] |= 0x01;
                } else if ((dev->ohci_mmio[0x49] & 0x03) == 0x01) {
                    if (!(dev->ohci_mmio[0x4e] & 0x02))
                        dev->ohci_mmio[0x55] |= 0x01;
                    if (!(dev->ohci_mmio[0x4e] & 0x04))
                        dev->ohci_mmio[0x59] |= 0x01;
                }
            }
            return;
        case 0x53:
            if (val & 0x80)
                dev->ohci_mmio[0x51] &= ~0x80;
            return;
        case 0x54:
        case 0x58:
            old = dev->ohci_mmio[addr];

            if (val & 0x10) {
                if (old & 0x01) {
                    dev->ohci_mmio[addr] |= 0x10;
                    /* TODO: The clear should be on a 10 ms timer. */
                    dev->ohci_mmio[addr] &= ~0x10;
                    dev->ohci_mmio[addr + 2] |= 0x10;
                } else
                    dev->ohci_mmio[addr + 2] |= 0x01;
            }
            if (val & 0x08)
                dev->ohci_mmio[addr] &= ~0x04;
            if (val & 0x04)
                dev->ohci_mmio[addr] |= 0x04;
            if (val & 0x02) {
                if (old & 0x01)
                    dev->ohci_mmio[addr] |= 0x02;
                else
                    dev->ohci_mmio[addr + 2] |= 0x01;
            }
            if (val & 0x01) {
                if (old & 0x01)
                    dev->ohci_mmio[addr] &= ~0x02;
                else
                    dev->ohci_mmio[addr + 2] |= 0x01;
            }

            if (!(dev->ohci_mmio[addr] & 0x04) && (old & 0x04))
                dev->ohci_mmio[addr + 2] |= 0x04;
#if 0
            if (!(dev->ohci_mmio[addr] & 0x02))
                 dev->ohci_mmio[addr + 2] |= 0x02;
#endif
            return;
        case 0x55:
            if ((val & 0x02) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x02)) {
                dev->ohci_mmio[addr] &= ~0x01;
                dev->ohci_mmio[0x54] &= ~0x17;
                dev->ohci_mmio[0x56] &= ~0x17;
            }
            if ((val & 0x01) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x02)) {
                dev->ohci_mmio[addr] |= 0x01;
                dev->ohci_mmio[0x58] &= ~0x17;
                dev->ohci_mmio[0x5a] &= ~0x17;
            }
            return;
        case 0x59:
            if ((val & 0x02) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x04))
                dev->ohci_mmio[addr] &= ~0x01;
            if ((val & 0x01) && ((dev->ohci_mmio[0x49] & 0x03) == 0x00) && (dev->ohci_mmio[0x4e] & 0x04))
                dev->ohci_mmio[addr] |= 0x01;
            return;
        case 0x56:
        case 0x5a:
            dev->ohci_mmio[addr] &= ~(val & 0x1f);
            return;
        case 0x57:
        case 0x5b:
            return;

        default:
            break;
    }

    dev->ohci_mmio[addr] = val;
}

void
ohci_update_mem_mapping(usb_t *dev, uint8_t base1, uint8_t base2, uint8_t base3, int enable)
{
    if (dev->ohci_enable && (dev->ohci_mem_base != 0x00000000))
        mem_mapping_disable(&dev->ohci_mmio_mapping);

    dev->ohci_mem_base = ((base1 << 8) | (base2 << 16) | (base3 << 24)) & 0xfffff000;
    dev->ohci_enable   = enable;

    if (dev->ohci_enable && (dev->ohci_mem_base != 0x00000000))
        mem_mapping_set_addr(&dev->ohci_mmio_mapping, dev->ohci_mem_base, 0x1000);
}

static void
usb_reset(void *priv)
{
    usb_t *dev = (usb_t *) priv;

    uhci_power_on_reset(dev->uhci);

    memset(dev->ohci_mmio, 0x00, 4096);
    dev->ohci_mmio[0x00] = 0x10;
    dev->ohci_mmio[0x01] = 0x01;
    dev->ohci_mmio[0x48] = 0x02;

    if (dev->uhci_enable && (dev->uhci_io_base != 0x0000))
        io_removehandler(dev->uhci_io_base, 0x20, uhci_inb, uhci_inw, uhci_inl, uhci_outb, uhci_outw, uhci_outl, dev->uhci);
    dev->uhci_enable = 0;

    mem_mapping_disable(&dev->ohci_mmio_mapping);
    dev->ohci_enable = 0;
}

static void
usb_close(void *priv)
{
    usb_t *dev = (usb_t *) priv;

    for (int h = 0; h < USB_MAX_HC; h++) {
        if (usb_hcs[h] == dev)
            usb_hcs[h] = NULL;
    }
    for (int i = 0; i < UHCI_PORTS; i++) {
        if (dev->uhci->port[i].usb != NULL)
            dev->uhci->port[i].usb->port = NULL;
    }

    timer_disable(&dev->uhci->frame_timer);
    timer_disable(&dev->uhci->detect_timer);
    free(dev->uhci);
    free(dev);

    usb_log_close();
}

static void *
usb_init(UNUSED(const device_t *info))
{
    usb_t *dev;

    dev = (usb_t *) calloc(1, sizeof(usb_t));
    if (dev == NULL)
        return (NULL);
    memset(dev, 0x00, sizeof(usb_t));

    usb_log_open();

    dev->uhci = (uhci_t *) calloc(1, sizeof(uhci_t));
    timer_add(&dev->uhci->frame_timer, uhci_frame_end, dev->uhci, 0);
    timer_add(&dev->uhci->detect_timer, uhci_detect, dev->uhci, 0);

    for (int h = 0; h < USB_MAX_HC; h++) {
        if (usb_hcs[h] == NULL) {
            usb_hcs[h]     = dev;
            dev->uhci->bus = h + 1;
            break;
        }
    }

    memset(dev->ohci_mmio, 0x00, 4096);
    dev->ohci_mmio[0x00] = 0x10;
    dev->ohci_mmio[0x01] = 0x01;
    dev->ohci_mmio[0x48] = 0x02;

    mem_mapping_add(&dev->ohci_mmio_mapping, 0, 0,
                    ohci_mmio_read, NULL, NULL,
                    ohci_mmio_write, NULL, NULL,
                    NULL, MEM_MAPPING_EXTERNAL, dev);
    usb_reset(dev);

    return dev;
}

const device_t usb_device = {
    .name          = "Universal Serial Bus",
    .internal_name = "usb",
    .flags         = DEVICE_PCI,
    .local         = 0,
    .init          = usb_init,
    .close         = usb_close,
    .reset         = usb_reset,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
