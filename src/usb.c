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
#include <86box/usb.h>
#include <86box/usb_log.h>
#include "cpu.h"
#include <86box/plat_unused.h>

/* UHCI, written from the Intel UHCI Design Guide 1.1 (section 2.1) and the
   PIIX3/PIIX4 datasheets. Registers and root hub only; no schedule yet. */
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

typedef struct uhci_port_t {
    uint16_t sc;       /* CCS, CSC, PE, PEDC, RD, LSDA, PR, SUSP */
    uint8_t  attached; /* a device is on the wire */
    uint8_t  low_speed;
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

static void
uhci_frame_end(void *priv)
{
    uhci_t *dev = (uhci_t *) priv;

    dev->frnum = (dev->frnum + 1) & 0x07ff;
    timer_on_auto(&dev->frame_timer, uhci_frame_us(dev));
}

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
    } else {
        /* Outside debug mode a stop resets the SOF counter; in debug mode it freezes (Table 4). */
        dev->frame_frozen  = !!(dev->cmd & UHCI_CMD_SWDBG);
        dev->frame_left_us = timer_get_remaining_us(&dev->frame_timer);
        timer_stop(&dev->frame_timer);
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI frame clock stopped%s", dev->frame_frozen ? " (frozen, SWDBG)" : "");
    }
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

        if (!port->attached || (port->sc & UHCI_PORT_CCS) || uhci_port_in_reset(dev, port))
            continue;

        port->sc |= UHCI_PORT_CCS | UHCI_PORT_CSC;
        if (port->low_speed)
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
        if (dev->port[i].attached) {
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
        j = port->low_speed ? UHCI_PORT_DM : UHCI_PORT_DP;
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
