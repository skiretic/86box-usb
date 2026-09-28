/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Universal Serial Bus emulation (UHCI host controller, OHCI
 *          registers).
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
#include <86box/plat.h>
#include <86box/ui.h>
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
    uint8_t     bus_master; /* PCICMD BME from the chipset */
    /* Where the schedule walk stands in the current frame; a stop in debug
       mode resumes there. */
    uint8_t     dbg_resume;
    uint32_t    dbg_link;
    double      dbg_left;
    double      frame_left_us;
    /* VIA Rx40 bit 6: a port babbling at EOF stays enabled. */
    uint8_t     babble_keep_port;
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

/* OHCI, written from the OpenHCI 1.0a specification (PDF page numbers) and
   the ALi M1543C (M5237), SiS 5595 and SMSC SLC90E66 datasheets. */
#define OHCI_HCREVISION         0x000
#define OHCI_HCCONTROL          0x004
#define OHCI_HCCOMMANDSTATUS    0x008
#define OHCI_HCINTERRUPTSTATUS  0x00c
#define OHCI_HCINTERRUPTENABLE  0x010
#define OHCI_HCINTERRUPTDISABLE 0x014
#define OHCI_HCHCCA             0x018
#define OHCI_HCPERIODCURRENTED  0x01c
#define OHCI_HCCONTROLHEADED    0x020
#define OHCI_HCCONTROLCURRENTED 0x024
#define OHCI_HCBULKHEADED       0x028
#define OHCI_HCBULKCURRENTED    0x02c
#define OHCI_HCDONEHEAD         0x030
#define OHCI_HCFMINTERVAL       0x034
#define OHCI_HCFMREMAINING      0x038
#define OHCI_HCFMNUMBER         0x03c
#define OHCI_HCPERIODICSTART    0x040
#define OHCI_HCLSTHRESHOLD      0x044
#define OHCI_HCRHDESCRIPTORA    0x048
#define OHCI_HCRHDESCRIPTORB    0x04c
#define OHCI_HCRHSTATUS         0x050
#define OHCI_HCRHPORTSTATUS1    0x054
#define OHCI_HCECONTROL         0x100
#define OHCI_HCEINPUT           0x104
#define OHCI_HCEOUTPUT          0x108
#define OHCI_HCESTATUS          0x10c

#define OHCI_CTL_CBSR 0x00000003
#define OHCI_CTL_PLE  0x00000004
#define OHCI_CTL_IE   0x00000008
#define OHCI_CTL_CLE  0x00000010
#define OHCI_CTL_BLE  0x00000020
#define OHCI_CTL_HCFS 0x000000c0
#define OHCI_CTL_IR   0x00000100
#define OHCI_CTL_RWC  0x00000200
#define OHCI_CTL_RWE  0x00000400

#define OHCI_HCFS_RESET       0x00000000
#define OHCI_HCFS_RESUME      0x00000040
#define OHCI_HCFS_OPERATIONAL 0x00000080
#define OHCI_HCFS_SUSPEND     0x000000c0

#define OHCI_CS_HCR 0x00000001
#define OHCI_CS_CLF 0x00000002
#define OHCI_CS_BLF 0x00000004
#define OHCI_CS_OCR 0x00000008
#define OHCI_CS_SOC 0x00030000

#define OHCI_INT_SO   0x00000001
#define OHCI_INT_WDH  0x00000002
#define OHCI_INT_SF   0x00000004
#define OHCI_INT_RD   0x00000008
#define OHCI_INT_UE   0x00000010
#define OHCI_INT_FNO  0x00000020
#define OHCI_INT_RHSC 0x00000040
#define OHCI_INT_OC   0x40000000
#define OHCI_INT_MIE  0x80000000
#define OHCI_INT_ALL  0x4000007f

/* HcRhStatus (p140-141): read meaning / write command. */
#define OHCI_RH_LPS  0x00000001 /* ClearGlobalPower */
#define OHCI_RH_OCI  0x00000002
#define OHCI_RH_DRWE 0x00008000 /* SetRemoteWakeupEnable */
#define OHCI_RH_LPSC 0x00010000 /* SetGlobalPower */
#define OHCI_RH_OCIC 0x00020000
#define OHCI_RH_CRWE 0x80000000

/* HcRhPortStatus (p142-145): read meaning / write command. */
#define OHCI_PS_CCS    0x00000001 /* ClearPortEnable */
#define OHCI_PS_PES    0x00000002 /* SetPortEnable */
#define OHCI_PS_PSS    0x00000004 /* SetPortSuspend */
#define OHCI_PS_POCI   0x00000008 /* ClearSuspendStatus */
#define OHCI_PS_PRS    0x00000010 /* SetPortReset */
#define OHCI_PS_PPS    0x00000100 /* SetPortPower */
#define OHCI_PS_LSDA   0x00000200 /* ClearPortPower */
#define OHCI_PS_CSC    0x00010000
#define OHCI_PS_PESC   0x00020000
#define OHCI_PS_PSSC   0x00040000
#define OHCI_PS_OCIC   0x00080000
#define OHCI_PS_PRSC   0x00100000
#define OHCI_PS_CHANGE 0x001f0000

#define OHCI_DA_PSM 0x00000100
#define OHCI_DA_NPS 0x00000200

#define OHCI_PORT_RESET_US  10000.0 /* p145 */
#define OHCI_PORT_RESUME_US 23000.0 /* 20 ms resume, then the 3 ms resync (p145) */

enum {
    OHCI_PORT_TIMER_NONE = 0,
    OHCI_PORT_TIMER_RESET,
    OHCI_PORT_TIMER_RESUME
};

/* Endpoint Descriptor dword 0 and 2 (p31), General TD dword 0 (p34-35). */
#define OHCI_ED_S     0x00002000
#define OHCI_ED_K     0x00004000
#define OHCI_ED_F     0x00008000
#define OHCI_ED_H     0x00000001
#define OHCI_ED_C     0x00000002
#define OHCI_TD_R     0x00040000
#define OHCI_TD_DI    0x00e00000
#define OHCI_TD_T     0x03000000
#define OHCI_TD_EC    0x0c000000
#define OHCI_TD_CC    0xf0000000

#define OHCI_CC_NOERROR             0x0
#define OHCI_CC_DATATOGGLEMISMATCH  0x3
#define OHCI_CC_STALL               0x4
#define OHCI_CC_DEVICENOTRESPONDING 0x5
#define OHCI_CC_DATAOVERRUN         0x8
#define OHCI_CC_DATAUNDERRUN        0x9

#define OHCI_LIST_CONTROL 0
#define OHCI_LIST_BULK    1

/* One descriptor fetch, about one PCI burst read (250 ns, as the UHCI). */
#define OHCI_FETCH_BITS 3.0

/* 12,000 bit times per frame (p135). */
#define OHCI_FI_NOMINAL 0x00002edf

/* HceControl and HceStatus (Appendix B, p158-159). */
#define OHCI_HCE_EE        0x00000001
#define OHCI_HCE_EI        0x00000002
#define OHCI_HCE_CP        0x00000004
#define OHCI_HCE_EXTIRQEN  0x00000010
#define OHCI_HCE_IRQ1ACT   0x00000040
#define OHCI_HCE_IRQ12ACT  0x00000080
#define OHCI_HCES_OUTFULL  0x01
#define OHCI_HCES_INFULL   0x02

/* The M5237 with its third port enabled (ALi-C p185). */
#define OHCI_MAX_PORTS 3

typedef struct ohci_port_t {
    uint32_t   sts; /* HcRhPortStatus: status bits 15:0 (PPS as switched), change bits 31:16 */
    uint8_t    timer_op;
    int        num;
    usb_dev_t *usb; /* the device on the wire, if any */
    ohci_t    *hc;
    pc_timer_t timer;
} ohci_port_t;

struct ohci_t {
    uint32_t    control;
    uint32_t    cmdsts;
    uint32_t    intsts;
    uint32_t    inten;
    uint32_t    hcca;
    uint32_t    period_cur;
    uint32_t    ctrl_head;
    uint32_t    ctrl_cur;
    uint32_t    bulk_head;
    uint32_t    bulk_cur;
    uint32_t    done_head;
    uint32_t    fm_interval;
    uint32_t    fm_remaining;
    uint32_t    fm_number;
    uint32_t    periodic_start;
    uint32_t    ls_threshold;
    uint32_t    rh_desc_a;
    uint32_t    rh_desc_b;
    uint32_t    rh_status;
    int         ndp;
    int         ports; /* ports with a connector; devices can sit past ndp */
    ohci_port_t port[OHCI_MAX_PORTS];
    int         bus; /* pcap bus number */
    uint8_t     bus_master; /* PCI COMMAND Bus Master Enable from the chipset */
    uint8_t     bus_master_hardwired; /* ... or 1 through every reset */
    uint32_t    chip_flags;           /* OHCI_CHIP_* */
    /* The frame in progress: FrameRemaining counts down from fr_load, and
       FrameNumber bit 15 as of the last HCCA update. */
    uint32_t    fr_load;
    uint32_t    fn_msb;
    pc_timer_t  frame_timer;
    /* List processing: bit times left in the frame being run, the next
       nonperiodic list and the control EDs left before a bulk ED (carried
       across frames), and whether a list still waits for its first
       service since entering USBOPERATIONAL. */
    double      bits_left;
    int         np_list;
    int         ratio_left;
    uint8_t     np_first[2];
    uint8_t     so_pending; /* the periodic list overran: SO at the next HCCA update */
    uint8_t     di_count;   /* Done Queue Interrupt Counter, 7 = no writeback due */
    void      (*set_irq)(void *priv, int level);
    void       *irq_priv;
    void      (*set_smi)(void *priv, int level);
    void       *smi_priv;
    uint8_t     int_level; /* the INT and SMI outputs as last driven */
    uint8_t     smi_level;
    /* Legacy support (Appendix B): the registers only, no 60h/64h decode. */
    uint32_t    hce_control;
    uint8_t     hce_input;
    uint8_t     hce_output;
    uint8_t     hce_status;
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

static const char *
ohci_reg_name(uint32_t off)
{
    static const char *const names[] = {
        "HcRevision", "HcControl", "HcCommandStatus", "HcInterruptStatus",
        "HcInterruptEnable", "HcInterruptDisable", "HcHCCA", "HcPeriodCurrentED",
        "HcControlHeadED", "HcControlCurrentED", "HcBulkHeadED", "HcBulkCurrentED",
        "HcDoneHead", "HcFmInterval", "HcFmRemaining", "HcFmNumber",
        "HcPeriodicStart", "HcLSThreshold", "HcRhDescriptorA", "HcRhDescriptorB",
        "HcRhStatus", "HcRhPortStatus1", "HcRhPortStatus2", "HcRhPortStatus3"
    };
    static const char *const legacy[] = { "HceControl", "HceInput", "HceOutput", "HceStatus" };

    off &= 0xffc;
    if (off < (sizeof(names) / sizeof(names[0])) * 4)
        return names[off >> 2];
    if ((off >= 0x100) && (off < 0x110))
        return legacy[(off - 0x100) >> 2];
    return "reserved";
}

static const char *
ohci_hcfs_name(uint32_t control)
{
    static const char *const names[] = { "USBRESET", "USBRESUME", "USBOPERATIONAL", "USBSUSPEND" };

    return names[(control & OHCI_CTL_HCFS) >> 6];
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

    if (dev->sts != old)
        usb_trace(USB_LOG_IRQ, dev->frnum, "UHCI USBSTS %04x -> %04x (end of frame%s%s)", old, dev->sts,
                  dev->int_ioc ? ", IOC" : "", dev->int_spd ? ", SPD" : "");

    /* A source can latch behind a USBINT that is already set, so the line is
       re-evaluated even when USBSTS did not change. */
    uhci_update_irq(dev);
}

static void uhci_run_frame(uhci_t *dev);
static void uhci_frame_end(void *priv);

/* Called after every USBCMD change: starts or stops the frame clock. */
static void
uhci_update_frame_clock(uhci_t *dev, int was_running)
{
    int running = uhci_running(dev);

    if (running == was_running)
        return;

    if (running) {
        int at_eof = 0;

        /* A frame frozen with nothing left ends as it resumes; a zero period
           would stop the timer instead. */
        if (dev->frame_frozen && (dev->cmd & UHCI_CMD_SWDBG) && (dev->frame_left_us > 0.0))
            timer_on_auto(&dev->frame_timer, dev->frame_left_us);
        else if (dev->frame_frozen && (dev->cmd & UHCI_CMD_SWDBG))
            at_eof = 1;
        else {
            timer_on_auto(&dev->frame_timer, uhci_frame_us(dev));
            dev->dbg_resume = 0;
        }
        dev->frame_frozen = 0;
        usb_trace(USB_LOG_REG, dev->frnum, "UHCI frame clock started, FLBASEADD %08x", dev->flbaseadd);
        if (at_eof)
            uhci_frame_end(dev);
        else
            uhci_run_frame(dev);
    } else {
        /* Outside debug mode a stop resets the SOF counter; in debug mode it
           freezes (Table 4), to the timer's own resolution, and the walk
           resumes where it stopped, whoever cleared Run/Stop. */
        dev->frame_frozen  = !!(dev->cmd & UHCI_CMD_SWDBG);
        dev->dbg_resume    = dev->frame_frozen;
        dev->frame_left_us = (double) timer_get_remaining_u64(&dev->frame_timer) / (double) TIMER_USEC;
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

/* Bus time of one transaction in ns (USB 1.1 s5.9.3), Host_Delay 0 and
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

/* EOF2, 10 full-speed bit times before the SOF (USB 1.1 Table 11-1): a
   device still sending there is a babbler (s11.8.1), so a transaction a
   device ends must end by it. An iso OUT, all the HC's own packet, only
   has to end by the SOF (UHCI 1.1 s1.1.1). */
#define UHCI_EOF2_NS (10.0 * 1000.0 / 12.0)

/* The most data bytes, up to len, of a transaction that ends within left
   ns; -1 if not even an empty one does. */
static int
uhci_bytes_before_eof(int low_speed, int iso, int in, int len, double left)
{
    while ((len >= 0) && (uhci_bus_ns(low_speed, iso, in, len) > left))
        len--;

    return len;
}

/* Wire time in ns of an IN answered by a handshake, or by nothing (the
   host's timeout wait), from USB 1.1 s11.3.3: 34-bit token, 18 bits of
   turnaround, 16-bit handshake and its 3-bit EOP. A low-speed bit is 8
   full-speed bits; the preamble sent while a full-speed device sits on an
   enabled root port (s3.2.2 bit 26) adds 20. */
static double
uhci_in_nodata_ns(const uhci_t *dev, int low_speed, int handshake)
{
    double bits = 34.0 + 18.0 + (handshake ? (16.0 + 3.0) : 0.0);

    if (low_speed) {
        bits *= 8.0;
        for (int i = 0; i < UHCI_PORTS; i++) {
            const uhci_port_t *port = &dev->port[i];

            if ((port->usb != NULL) && !port->usb->low_speed && (port->sc & UHCI_PORT_PE) &&
                !(port->sc & (UHCI_PORT_SUSP | UHCI_PORT_PR))) {
                bits += 20.0;
                break;
            }
        }
    }

    return bits * 1000.0 / 12.0;
}

/* One descriptor fetch, about one PCI burst read; it also bounds a loop
   of empty queue heads to the frame. */
#define UHCI_FETCH_NS 250.0

static usb_dev_t *
uhci_find_device(uhci_t *dev, int addr, int low_speed)
{
    for (int i = 0; i < UHCI_PORTS; i++) {
        const uhci_port_t *port = &dev->port[i];

        usb_dev_t         *usb;

        /* Only an enabled, unsuspended port carries traffic, and a low-speed
           token only reaches a low-speed device. On a port with a full-speed
           hub it goes out behind a preamble (s3.2.2 bit 26) for the hub to
           repeat to its low-speed ports. */
        if ((port->usb == NULL) || !(port->sc & UHCI_PORT_PE) || (port->sc & (UHCI_PORT_SUSP | UHCI_PORT_PR)))
            continue;
        if ((port->usb->addr == addr) && (port->usb->low_speed == low_speed))
            return port->usb;
        if ((port->usb->ops->route != NULL) && ((usb = port->usb->ops->route(port->usb, addr, low_speed)) != NULL))
            return usb;
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
    TD_NO_TIME,   /* not started: preSOF has passed */
    TD_HALT       /* process error: Run/Stop cleared */
};

#ifdef ENABLE_USB_LOG
static const char *const uhci_ret_names[] = { "ACK", "NAK", "STALL", "timeout", "babble", "toggle mismatch", "overrun" };
#endif

/* CRC/timeout, counted down by C_ERR; 0 retries forever (s3.2.2). */
static int
uhci_td_timeout(uhci_t *dev, uint32_t *ctrl)
{
    int cerr = (*ctrl & UHCI_TD_CERR) >> 27;

    *ctrl |= UHCI_TD_CRCTO;
    if (cerr == 0)
        return TD_RETRY;

    cerr--;
    *ctrl = (*ctrl & ~UHCI_TD_CERR) | ((uint32_t) cerr << 27);
    if (cerr != 0)
        return TD_RETRY;

    *ctrl        = (*ctrl & ~UHCI_TD_ACTIVE) | UHCI_TD_STALLED;
    dev->eof_err = 1;
    return TD_FAILED;
}

/* A device still sending at EOF2 loses the port it sends through (USB 1.1
   s11.2.2, s11.8.1). Behind a hub that is the nearest hub's port (s8.7.4);
   the hub's EOP at EOF1 leaves the root port alone, and the chip's keep
   bit covers only its own root ports. */
static void
uhci_eof_babbler(uhci_t *dev, usb_dev_t *usb)
{
    if (usb->parent != NULL) {
        if (usb->parent->ops->eof_babble != NULL)
            usb->parent->ops->eof_babble(usb->parent, usb);
        return;
    }

    if (dev->babble_keep_port)
        return;

    for (int i = 0; i < UHCI_PORTS; i++) {
        uhci_port_t *port = &dev->port[i];

        if ((port->usb == usb) && (port->sc & UHCI_PORT_PE)) {
            port->sc = (port->sc & ~UHCI_PORT_PE) | UHCI_PORT_PEDC;
            usb_trace(USB_LOG_REG, dev->frnum, "UHCI port %i disabled: EOF babble, PORTSC %04x", i + 1, port->sc);
        }
    }
}

static int
uhci_exec_td(uhci_t *dev, uint32_t addr, int in_queue, double *left, uint32_t *link)
{
    uint32_t     td[4];
    uint32_t     ctrl;
    uint8_t      buf[1280];
    usb_packet_t p;
    usb_dev_t   *usb;
    usb_dev_t   *babbler = NULL;
    int          pid;
    int          len;
    int          ls;
    int          iso;
    int          fit;
    int          eof = 0;
    int          ret;
    int          result;
    double       in_ns = 0.0;
    double       to_eof;

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

    /* Nothing starts past preSOF, the window kept for a MAXP-sized packet
       to end by EOF2 (s1.3.1 cases A and B); what starts before it runs on
       (case C), and past EOF only as an EOF babble (s2.1.1 bit 7, s4.1.4). */
    if (*left < (UHCI_EOF2_NS + uhci_bus_ns(0, 0, 1, (dev->cmd & UHCI_CMD_MAXP) ? 64 : 32)))
        return TD_NO_TIME;
    to_eof = (iso && (pid == USB_PID_OUT)) ? *left : (*left - UHCI_EOF2_NS);
    fit    = uhci_bytes_before_eof(ls, iso, pid == USB_PID_IN, len, to_eof);

    p.pid     = pid;
    p.ep      = (td[2] >> 15) & 0x0f;
    p.toggle  = (td[2] >> 19) & 1;
    p.data    = buf;
    p.len     = len;
    p.actual  = 0;
    p.bus_len = 0;
    /* The UHCI accepts no more than MaxLen (s3.2.3); a babbled packet gets
       no handshake (USB 1.1 s8.4.5.2), so the device keeps it. */
    p.ack_overrun = 0;
    p.no_ack      = 0;

    if ((pid != USB_PID_IN) && (len > 0))
        dma_bm_read(td[3], buf, len, 1);

    usb = uhci_find_device(dev, (td[2] >> 8) & 0x7f, ls);
    if ((pid != USB_PID_IN) && (fit < len)) {
        /* The HC ends its own packet with a bit stuff error (s4.1.4) rather
           than draw a handshake past EOF2 (USB 1.1 s11.3), so the device
           takes nothing. */
        ret = USB_RET_BABBLE;
        eof = 1;
    } else {
        int room = (fit < 0) ? 0 : fit;

        /* IN data cut off at EOF gets no handshake, so the device keeps it,
           even an empty packet when not even that ends in time; an
           isochronous IN is sent once, whatever the host can take. */
        if ((fit < len) && !iso) {
            p.len    = room;
            p.no_ack = (fit < 0);
        }
        ret   = (usb != NULL) ? usb_dev_packet(usb, &p) : USB_RET_NO_RESPONSE;
        p.len = len; /* a short packet is short of MaxLen */

        /* An IN babbles at EOF when the device's packet, whatever MaxLen,
           runs past EOF2, the device still sending there. A NAK or STALL
           is timed on the wire. With no answer only the token, which always
           ends before EOF past the preSOF gate, was sent: a timeout. */
        if (pid == USB_PID_IN) {
            if ((ret == USB_RET_NAK) || (ret == USB_RET_STALL) || (ret == USB_RET_NO_RESPONSE))
                in_ns = uhci_in_nodata_ns(dev, ls, ret != USB_RET_NO_RESPONSE);
            else
                in_ns = uhci_bus_ns(ls, iso, 1, p.bus_len);
            if ((ret != USB_RET_NO_RESPONSE) && (in_ns > to_eof)) {
                babbler = usb;
                ret     = USB_RET_BABBLE;
                if (p.actual > room)
                    p.actual = room;
                eof = 1;
            }
        }

        /* An isochronous OUT has no handshake (USB 1.1 s8.5.4): once token and
           data are sent the transaction is over, whoever took the data. */
        if (iso && (pid == USB_PID_OUT))
            ret = USB_RET_ACK;
    }

    /* OUT data is on the bus whatever the handshake; an IN takes the time
       of what the device sent, all of a babbled packet. An EOF babble
       lasts to the end of the frame. */
    if (eof)
        *left = 0.0;
    else
        *left -= (pid == USB_PID_IN) ? in_ns : uhci_bus_ns(ls, iso, 0, len);

    if (babbler != NULL)
        uhci_eof_babbler(dev, babbler);

    /* Each execution rewrites the status field (s3.2.2 bits 23:16) and ActLen,
       7FFh (no bytes) unless data moved. */
    ctrl = (ctrl & ~((UHCI_TD_STATUS & ~UHCI_TD_ACTIVE) | UHCI_TD_ACTLEN)) | 0x7ff;
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
            /* A NAK or STALL to a SETUP is also a timeout (s3.2.2 bits 19, 18
               and the Table 7 note): C_ERR counts it, Stalled only at 0. */
            if (pid == USB_PID_SETUP)
                result = uhci_td_timeout(dev, &ctrl);
            break;

        case USB_RET_STALL:
            if (pid == USB_PID_SETUP) {
                result = uhci_td_timeout(dev, &ctrl);
                break;
            }
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
            /* No handshake. */
            result = uhci_td_timeout(dev, &ctrl);
            break;
    }

    /* Isochronous TDs go inactive after one execution whatever happened (s3.2.2
       bit 25); one retired with an error sets USB Error Interrupt (s4.1.2). */
    if (iso && (ctrl & UHCI_TD_ACTIVE)) {
        ctrl &= ~UHCI_TD_ACTIVE;
        result = TD_FAILED;
        if (ctrl & (UHCI_TD_CRCTO | UHCI_TD_BITSTUFF | UHCI_TD_DBE))
            dev->eof_err = 1;
    }

    if (!(ctrl & UHCI_TD_ACTIVE) && (ctrl & UHCI_TD_IOC))
        dev->eof_ioc = 1;

    dma_bm_write(addr + 4, (uint8_t *) &ctrl, 4, 4);

    usb_trace(USB_LOG_SCHED, dev->frnum, "TD %08x %s dev %i ep %i D%i len %i%s -> %s%s, ctrl %08x",
              addr, (pid == USB_PID_IN) ? "IN" : ((pid == USB_PID_OUT) ? "OUT" : "SETUP"),
              (td[2] >> 8) & 0x7f, p.ep, p.toggle, p.len, ls ? " LS" : "",
              uhci_ret_names[ret], eof ? " at EOF" : "", ctrl);

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
    int      resume = dev->dbg_resume;
    double   left   = uhci_frame_us(dev) * 1000.0;
    double   presof = UHCI_EOF2_NS + uhci_bus_ns(0, 0, 1, (dev->cmd & UHCI_CMD_MAXP) ? 64 : 32);

    /* A frame not resumed has walked nothing yet. */
    dev->dbg_resume = 0;
    if (!resume)
        dev->dbg_link = UHCI_LP_T;

    /* With Bus Master Enable clear the UHCI may not touch memory at all
       (PIIX4 6.1.3); the frame counter needs no memory and keeps counting. */
    if (!uhci_running(dev) || !dev->bus_master)
        return;

    /* A stop in debug mode picks up where the walk left off (Table 4). */
    if (resume) {
        link = dev->dbg_link;
        left = dev->dbg_left;
        usb_trace(USB_LOG_SCHED, dev->frnum, "debug step resumes at %08x", link);
    } else {
        dma_bm_read(dev->flbaseadd + ((dev->frnum & 0x3ff) << 2), (uint8_t *) &link, 4, 4);
        usb_trace(USB_LOG_SCHED, dev->frnum, "frame list entry %08x", link);
    }

    while (!(link & UHCI_LP_T) && (steps++ < 4096)) {
        /* Past preSOF nothing more is fetched this frame (s1.3.1 cases A, C),
           so a TD that found no time (case B) also ends the walk, and an
           inactive IOC TD behind it is first seen in a later frame. */
        if (left < presof)
            break;
        /* A process error stops the walk here. */
        dev->dbg_link = link;
        dev->dbg_left = left;
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
            if (r == TD_HALT)
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
            if (r == TD_HALT)
                break;
            if ((r != TD_INACTIVE) && (r != TD_NO_TIME))
                work++;
            link = td_link;
        }

        /* Debug mode stops after each executed TD (s2.1.1.1). A babble ends
           the frame, so that step resumes at nothing until the next frame.
           A TD not started for lack of time was not executed. */
        if ((dev->cmd & UHCI_CMD_SWDBG) && (r != TD_INACTIVE) && (r != TD_NO_TIME)) {
            dev->dbg_link = (r == TD_END_FRAME) ? UHCI_LP_T : link;
            dev->dbg_left = left;
            uhci_self_stop(dev, 0);
            break;
        }

        /* Babble ends the frame: no further TDs in it (s3.2.2 bit 20). */
        if ((r == TD_END_FRAME) || (left <= 0.0))
            break;
    }

    /* A walk that ended with the HC running is done with this frame and
       waits for the next SOF (s2.1.1.1), so a later stop resumes at nothing. */
    if (uhci_running(dev)) {
        dev->dbg_link = UHCI_LP_T;
        dev->dbg_left = left;
    }
}

static void
uhci_frame_end(void *priv)
{
    uhci_t *dev = (uhci_t *) priv;

    uhci_latch_frame_status(dev);
    dev->frnum      = (dev->frnum + 1) & 0x07ff;
    dev->dbg_resume = 0;
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
    dev->dbg_resume   = 0;

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
    dev->dbg_resume   = 0;

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
        /* GRESET does not reset USBCMD itself (it reads 1 until software
           clears it), so the other bits latch; nothing runs while it is held. */
        dev->cmd = val & ~UHCI_CMD_HCRESET;
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
        /* The reset SE0 reads as a disconnect: an enabled port reports its
           disable in PEDC, and the end of reset is a connect change (silicon
           behavior per Linux uhci-hub.c and SeaBIOS usb-uhci.c). */
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
        /* Connect detection skips a port in reset; one not yet seen is seen now. */
        else if (port->usb != NULL)
            uhci_schedule_detect(dev);
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

    /* At 1Fh the high byte is 20h, past this block: it floats high, as in uhci_inl. */
    if (off & 1)
        ret = (uhci_read16(dev, off) >> 8) | ((off < 0x1f) ? (uhci_read16(dev, off + 1) << 8) : 0xff00);
    else
        ret = uhci_read16(dev, off);

    usb_trace(USB_LOG_REG, dev->frnum, "R16 +%02x %-9s = %04x", off, uhci_reg_name(off), ret);

    return ret;
}

static uint32_t
uhci_inl(uint16_t addr, void *priv)
{
    const uhci_t *dev = (uhci_t *) priv;
    uint16_t      off = addr & 0x1f;
    uint32_t      ret = 0;

    /* Byte by byte, so an unaligned dword gets the bytes at its offset.
       Bytes past 1Fh belong to no register of this block and float high. */
    for (int i = 3; i >= 0; i--) {
        uint16_t b = off + i;

        ret = (ret << 8) | ((b < 0x20) ? ((uhci_read16(dev, b) >> ((b & 1) << 3)) & 0xff) : 0xff);
    }

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

    /* At 1Fh the high byte is 20h, past this block: dropped, as in uhci_outl. */
    if (off & 1) {
        uhci_write16(dev, off, val << 8, 0xff00);
        if (off < 0x1f)
            uhci_write16(dev, off + 1, val >> 8, 0x00ff);
    } else
        uhci_write16(dev, off, val, 0xffff);
}

static void
uhci_outl(uint16_t addr, uint32_t val, void *priv)
{
    uhci_t  *dev   = (uhci_t *) priv;
    uint16_t off   = addr & 0x1f;
    uint64_t data  = (uint64_t) val << ((off & 1) << 3);
    uint64_t lanes = 0xffffffffULL << ((off & 1) << 3);

    usb_trace(USB_LOG_REG, dev->frnum, "W32 +%02x %-9s <- %08x", off, uhci_reg_name(off), val);

    /* Each register word takes the bytes that fall on it, so an unaligned
       dword keeps its offset. Bytes past 1Fh are not this block's: dropped. */
    for (uint16_t w = off & 0x1e; (w < 0x20) && (lanes & 0xffff); w += 2) {
        uhci_write16(dev, w, data & 0xffff, lanes & 0xffff);
        data >>= 16;
        lanes >>= 16;
    }
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

void
uhci_set_bus_master(usb_t *dev, int enable)
{
    dev->uhci->bus_master = !!enable;
}

void
uhci_set_babble_keep_port(usb_t *dev, int keep)
{
    dev->uhci->babble_keep_port = !!keep;
}

#define USB_MAX_HC 8

static usb_t *usb_hcs[USB_MAX_HC];

static void ohci_port_attach(ohci_port_t *p, usb_dev_t *udev);
static void ohci_port_detach(ohci_port_t *p);

/* Only a UHCI the chipset wired up (it routes its interrupt) takes devices.
   A device left out is named to the user with the reason. */
int
usb_attach(usb_dev_t *udev)
{
    char msg[512];
    int  wired = 0;

    for (int h = 0; h < USB_MAX_HC; h++) {
        uhci_t *hc = (usb_hcs[h] != NULL) ? usb_hcs[h]->uhci : NULL;
        ohci_t *oc = (usb_hcs[h] != NULL) ? usb_hcs[h]->ohci : NULL;

        if ((hc != NULL) && (hc->set_irq != NULL)) {
            wired = 1;
            for (int i = 0; i < UHCI_PORTS; i++) {
                if (hc->port[i].usb == NULL) {
                    hc->port[i].usb = udev;
                    udev->port      = &hc->port[i];
                    udev->bus       = hc->bus;
                    usb_trace(USB_LOG_REG, hc->frnum, "UHCI port %i: %s attached", i + 1, udev->name);
                    uhci_schedule_detect(hc);
                    return 1;
                }
            }
        }

        /* An OHCI is in use once its chipset routes its interrupt. */
        if ((oc != NULL) && (oc->set_irq != NULL)) {
            wired = 1;
            for (int i = 0; i < oc->ndp; i++) {
                if (oc->port[i].usb == NULL) {
                    ohci_port_attach(&oc->port[i], udev);
                    return 1;
                }
            }
        }
    }

    /* Root ports full: a hub on one of them takes the device. */
    for (int h = 0; h < USB_MAX_HC; h++) {
        uhci_t *hc = (usb_hcs[h] != NULL) ? usb_hcs[h]->uhci : NULL;
        ohci_t *oc = (usb_hcs[h] != NULL) ? usb_hcs[h]->ohci : NULL;

        for (int i = 0; (hc != NULL) && (hc->set_irq != NULL) && (i < UHCI_PORTS); i++) {
            usb_dev_t *hub = hc->port[i].usb;

            if ((hub != NULL) && (hub->ops->attach != NULL) && hub->ops->attach(hub, udev))
                return 1;
        }

        for (int i = 0; (oc != NULL) && (oc->set_irq != NULL) && (i < oc->ndp); i++) {
            usb_dev_t *hub = oc->port[i].usb;

            if ((hub != NULL) && (hub->ops->attach != NULL) && hub->ops->attach(hub, udev))
                return 1;
        }
    }

    /* Last, an OHCI connector past NDP: its port is not implemented, and the
       device stays unseen until the chipset raises NDP. */
    for (int h = 0; h < USB_MAX_HC; h++) {
        ohci_t *oc = (usb_hcs[h] != NULL) ? usb_hcs[h]->ohci : NULL;

        for (int i = (oc != NULL) ? oc->ndp : 0; (oc != NULL) && (oc->set_irq != NULL) && (i < oc->ports); i++) {
            usb_dev_t *on = oc->port[i].usb;

            if (on == NULL) {
                ohci_port_attach(&oc->port[i], udev);
                return 1;
            }
            if ((on->ops->attach != NULL) && on->ops->attach(on, udev))
                return 1;
        }
    }

    snprintf(msg, sizeof(msg), plat_get_string(wired ? STRING_USB_NO_PORT : STRING_USB_NO_CONTROLLER), udev->name);
    pclog("USB: %s\n", msg);
    ui_msgbox_header(MBX_WARNING, NULL, msg);

    return 0;
}

/* A disconnect clears CCS and PE and sets CSC and PEDC (s2.1.7, Tables 5-6). */
void
usb_detach(usb_dev_t *udev)
{
    if (udev->parent != NULL) {
        udev->parent->ops->detach(udev->parent, udev);
        return;
    }

    for (int h = 0; h < USB_MAX_HC; h++) {
        uhci_t *hc = (usb_hcs[h] != NULL) ? usb_hcs[h]->uhci : NULL;

        if (hc == NULL)
            continue;

        for (int i = 0; i < UHCI_PORTS; i++) {
            uhci_port_t *port = &hc->port[i];

            if (port->usb != udev)
                continue;

            port->usb = NULL;
            if (port->sc & UHCI_PORT_CCS) {
                port->sc |= UHCI_PORT_CSC;
                /* The disconnect is a resume event in global suspend (Table 6). A
                   device not yet connect-detected, or under GRESET (which holds
                   PORTSC at 0), leaves no line change behind. */
                if (hc->cmd & UHCI_CMD_EGSM) {
                    hc->cmd |= UHCI_CMD_FGR;
                    hc->sts |= UHCI_STS_RD;
                    usb_trace(USB_LOG_IRQ, hc->frnum, "UHCI USBSTS RD set (disconnect in global suspend)");
                }
            }
            if (port->sc & UHCI_PORT_PE)
                port->sc |= UHCI_PORT_PEDC;
            port->sc &= ~(UHCI_PORT_CCS | UHCI_PORT_PE | UHCI_PORT_LSDA | UHCI_PORT_SUSP | UHCI_PORT_RD);
            usb_trace(USB_LOG_REG, hc->frnum, "UHCI port %i: %s detached, PORTSC %04x", i + 1, udev->name, port->sc);
            uhci_update_irq(hc);
        }
    }

    for (int h = 0; h < USB_MAX_HC; h++) {
        ohci_t *oc = (usb_hcs[h] != NULL) ? usb_hcs[h]->ohci : NULL;

        for (int i = 0; (oc != NULL) && (i < OHCI_MAX_PORTS); i++) {
            if (oc->port[i].usb == udev)
                ohci_port_detach(&oc->port[i]);
        }
    }

    udev->port = NULL;
}

/* The INT and SMI outputs (p119, p121): an enabled status bit with MIE set
   requests an interrupt, on INT with InterruptRouting clear and on SMI with
   it set; OwnershipChange goes to SMI only. */
static void
ohci_update_irq(ohci_t *dev)
{
    uint32_t pending = (dev->inten & OHCI_INT_MIE) ? (dev->intsts & dev->inten & OHCI_INT_ALL) : 0;
    uint8_t  req     = !!(pending & ~OHCI_INT_OC);
    uint8_t  ir      = !!(dev->control & OHCI_CTL_IR);
    uint8_t  int_lvl = req && !ir;
    uint8_t  smi_lvl = (req && ir) || !!(pending & OHCI_INT_OC);

    if (int_lvl != dev->int_level) {
        dev->int_level = int_lvl;
        usb_trace(USB_LOG_IRQ, dev->fm_number, "OHCI INT %s (HcInterruptStatus %08x HcInterruptEnable %08x)%s",
                  int_lvl ? "assert" : "deassert", dev->intsts, dev->inten, dev->set_irq ? "" : ", no route");
        if (dev->set_irq != NULL)
            dev->set_irq(dev->irq_priv, int_lvl);
    }
    if (smi_lvl != dev->smi_level) {
        dev->smi_level = smi_lvl;
        usb_trace(USB_LOG_IRQ, dev->fm_number, "OHCI SMI %s (HcInterruptStatus %08x HcInterruptEnable %08x IR %i)%s",
                  smi_lvl ? "assert" : "deassert", dev->intsts, dev->inten, ir, dev->set_smi ? "" : ", no route");
        if (dev->set_smi != NULL)
            dev->set_smi(dev->smi_priv, smi_lvl);
    }
}

static void
ohci_port_trace(const ohci_port_t *p, const char *what)
{
    usb_trace(USB_LOG_REG, -1, "OHCI port %i: %s, HcRhPortStatus %08x", p->num, what, p->sts);
}

static void
ohci_port_timer_stop(ohci_port_t *p)
{
    timer_stop(&p->timer);
    p->timer_op = OHCI_PORT_TIMER_NONE;
}

/* A change bit set by the root hub raises RootHubStatusChange (p121); the
   HCD's own writes to the status bits do not. Outside USBOPERATIONAL only
   OC and RD are possible (p57-58): the port bit latches, RHSC is not set
   and is not raised later. */
static void
ohci_port_change(ohci_port_t *p, uint32_t bits)
{
    p->sts |= bits;
    if ((p->hc->control & OHCI_CTL_HCFS) == OHCI_HCFS_OPERATIONAL)
        p->hc->intsts |= OHCI_INT_RHSC;
}

static int
ohci_port_fixed(const ohci_port_t *p)
{
    return !!(p->hc->rh_desc_b & (1 << p->num));
}

static int
ohci_port_powered(const ohci_port_t *p)
{
    return (p->hc->rh_desc_a & OHCI_DA_NPS) || (p->sts & OHCI_PS_PPS);
}

static int
ohci_rh_in_reset(const ohci_t *dev)
{
    return (dev->control & OHCI_CTL_HCFS) == OHCI_HCFS_RESET;
}

static uint32_t
ohci_port_read(const ohci_port_t *p)
{
    uint32_t ret = p->sts;

    if (p->hc->rh_desc_a & OHCI_DA_NPS)
        ret |= OHCI_PS_PPS;
    if (ohci_port_fixed(p))
        ret |= OHCI_PS_CCS;

    return ret;
}

/* Entering USBRESUME, by the HCD or the HC, ends every port suspend (p143). */
static void
ohci_rh_global_resume(ohci_t *dev)
{
    for (int i = 0; i < dev->ndp; i++) {
        ohci_port_t *p = &dev->port[i];

        if (p->timer_op == OHCI_PORT_TIMER_RESUME)
            ohci_port_timer_stop(p);
        p->sts &= ~OHCI_PS_PSS;
    }
}

/* A resume event in USBSUSPEND: the HC moves itself to USBRESUME and sets
   ResumeDetected (p120). */
static void
ohci_resume_detected(ohci_t *dev)
{
    if ((dev->control & OHCI_CTL_HCFS) != OHCI_HCFS_SUSPEND)
        return;

    usb_trace(USB_LOG_IRQ, -1, "OHCI resume detected: USBSUSPEND -> USBRESUME, ResumeDetected set");
    dev->control = (dev->control & ~OHCI_CTL_HCFS) | OHCI_HCFS_RESUME;
    dev->intsts |= OHCI_INT_RD;
    ohci_rh_global_resume(dev);
}

/* With DeviceRemovable set, CSC comes only from a root hub reset (p145);
   with DeviceRemoteWakeupEnable a connect change is a resume event (p141). */
static void
ohci_port_connect_change(ohci_port_t *p)
{
    if (!ohci_port_fixed(p))
        ohci_port_change(p, OHCI_PS_CSC);
    if (p->hc->rh_status & OHCI_RH_DRWE)
        ohci_resume_detected(p->hc);
}

/* Connect detection: a device on a powered port within NDP, outside USBRESET. */
static void
ohci_port_detect(ohci_port_t *p)
{
    if ((p->usb == NULL) || (p->num > p->hc->ndp) || (p->sts & OHCI_PS_CCS) || !ohci_port_powered(p) || ohci_rh_in_reset(p->hc))
        return;

    p->sts |= OHCI_PS_CCS | (p->usb->low_speed ? OHCI_PS_LSDA : 0);
    ohci_port_connect_change(p);
    ohci_port_trace(p, "connect detected");
}

/* Power off resets CCS, PES, PSS and PRS (p144); losing PES is a hardware
   event (p143). The device loses its address and configuration. */
static void
ohci_port_power_off(ohci_port_t *p)
{
    ohci_port_timer_stop(p);
    if (p->sts & OHCI_PS_PES)
        ohci_port_change(p, OHCI_PS_PESC);
    p->sts &= ~(OHCI_PS_CCS | OHCI_PS_PES | OHCI_PS_PSS | OHCI_PS_PRS | OHCI_PS_LSDA);
    if (p->usb != NULL) {
        usb_dev_bus_reset(p->usb);
        p->usb->powered_reset = 0;
    }
    ohci_port_trace(p, "power off");
}

static void
ohci_port_set_power(ohci_port_t *p, int on)
{
    int was = ohci_port_powered(p);

    if (on)
        p->sts |= OHCI_PS_PPS;
    else
        p->sts &= ~OHCI_PS_PPS;

    if (was && !ohci_port_powered(p))
        ohci_port_power_off(p);
    else
        ohci_port_detect(p);
}

/* Set/ClearGlobalPower reach every port in global mode, and the ports whose
   PortPowerControlMask bit is clear in per-port mode (p140). */
static void
ohci_rh_global_power(ohci_t *dev, int on)
{
    for (int i = 0; i < dev->ndp; i++) {
        if (!(dev->rh_desc_a & OHCI_DA_PSM) || !(dev->rh_desc_b & (0x10000 << (i + 1))))
            ohci_port_set_power(&dev->port[i], on);
    }
}

static void
ohci_port_timer(void *priv)
{
    ohci_port_t *p  = (ohci_port_t *) priv;
    int          op = p->timer_op;

    p->timer_op = OHCI_PORT_TIMER_NONE;

    switch (op) {
        case OHCI_PORT_TIMER_RESET:
            /* The end of reset enables the port and ends a suspend (p143-145). */
            p->sts = (p->sts & ~(OHCI_PS_PRS | OHCI_PS_PSS | OHCI_PS_PSSC)) | OHCI_PS_PES;
            ohci_port_change(p, OHCI_PS_PRSC);
            ohci_port_trace(p, "reset done");
            break;

        case OHCI_PORT_TIMER_RESUME:
            p->sts = (p->sts & ~OHCI_PS_PSS) | OHCI_PS_PES;
            ohci_port_change(p, OHCI_PS_PSSC);
            ohci_port_trace(p, "resume done");
            break;

        default:
            break;
    }
    ohci_update_irq(p->hc);
}

static void
ohci_port_write(ohci_port_t *p, uint32_t val)
{
    const ohci_t *dev       = p->hc;
    int           connected = !!(ohci_port_read(p) & OHCI_PS_CCS);

    if (ohci_rh_in_reset(dev))
        return;

    /* Clears first, so the CSC a command below sets survives the write. */
    p->sts &= ~(val & OHCI_PS_CHANGE);

    if (val & OHCI_PS_CCS)
        p->sts &= ~OHCI_PS_PES;

    /* SetPortEnable, SetPortSuspend and SetPortReset on a disconnected port
       set CSC instead (p143-145). */
    if (val & (OHCI_PS_PES | OHCI_PS_PSS | OHCI_PS_PRS)) {
        if (!connected)
            ohci_port_change(p, OHCI_PS_CSC);
        else {
            if (val & OHCI_PS_PES)
                p->sts |= OHCI_PS_PES;
            if (val & OHCI_PS_PSS)
                p->sts |= OHCI_PS_PSS;
            if ((val & OHCI_PS_PRS) && !(p->sts & OHCI_PS_PRS)) {
                p->sts |= OHCI_PS_PRS;
                p->timer_op = OHCI_PORT_TIMER_RESET;
                timer_on_auto(&p->timer, OHCI_PORT_RESET_US);
                if (p->usb != NULL)
                    usb_dev_bus_reset(p->usb);
                ohci_port_trace(p, "reset start");
            }
        }
    }

    /* ClearSuspendStatus starts a resume only on a suspended port. */
    if ((val & OHCI_PS_POCI) && (p->sts & OHCI_PS_PSS) && (p->timer_op == OHCI_PORT_TIMER_NONE)) {
        p->timer_op = OHCI_PORT_TIMER_RESUME;
        timer_on_auto(&p->timer, OHCI_PORT_RESUME_US);
        ohci_port_trace(p, "resume start");
    }

    /* Per-port power commands reach a port only in per-port mode with its
       PortPowerControlMask bit set (p144). */
    if (!(dev->rh_desc_a & OHCI_DA_NPS) && (dev->rh_desc_a & OHCI_DA_PSM) && (dev->rh_desc_b & (0x10000 << p->num))) {
        if (val & OHCI_PS_PPS)
            ohci_port_set_power(p, 1);
        if (val & OHCI_PS_LSDA)
            ohci_port_set_power(p, 0);
    }
}

static void
ohci_rh_status_write(ohci_t *dev, uint32_t val)
{
    if (ohci_rh_in_reset(dev))
        return;

    dev->rh_status &= ~(val & OHCI_RH_OCIC);
    if (val & OHCI_RH_LPS)
        ohci_rh_global_power(dev, 0);
    if (val & OHCI_RH_LPSC)
        ohci_rh_global_power(dev, 1);
    if (val & OHCI_RH_DRWE)
        dev->rh_status |= OHCI_RH_DRWE;
    if (val & OHCI_RH_CRWE)
        dev->rh_status &= ~OHCI_RH_DRWE;
}

static void
ohci_rh_detect_all(ohci_t *dev)
{
    for (int i = 0; i < dev->ndp; i++)
        ohci_port_detect(&dev->port[i]);
}

/* The root hub's reset: the USBRESET state and a hardware reset (p57,
   ALi-C p187-188). Reset signaling reaches every device; without
   NoPowerSwitching the ports also lose power. The descriptors are reset by
   hardware reset only. */
static void
ohci_root_hub_reset(ohci_t *dev)
{
    dev->rh_status = 0x00000000;
    for (int i = 0; i < OHCI_MAX_PORTS; i++) {
        ohci_port_t *p = &dev->port[i];

        ohci_port_timer_stop(p);
        p->sts = 0x00000000;
        if (p->usb != NULL) {
            usb_dev_bus_reset(p->usb);
            if (!(dev->rh_desc_a & OHCI_DA_NPS))
                p->usb->powered_reset = 0;
        }
        /* ALi-C p188: CSC resets to 1 on a non-removable port. In USBRESET
           that raises no RHSC (p57). */
        if ((i < dev->ndp) && ohci_port_fixed(p))
            ohci_port_change(p, OHCI_PS_CSC);
    }
}

/* Entering USBSUSPEND while a port resumes, or has finished resuming, ends
   that resume and is a resume event for the HC (p120). */
static void
ohci_rh_suspend(ohci_t *dev)
{
    int event = 0;

    for (int i = 0; i < dev->ndp; i++) {
        ohci_port_t *p = &dev->port[i];

        if ((p->timer_op == OHCI_PORT_TIMER_RESUME) || (p->sts & OHCI_PS_PSSC)) {
            if (p->timer_op == OHCI_PORT_TIMER_RESUME)
                ohci_port_timer_stop(p);
            p->sts &= ~OHCI_PS_PSS;
            event = 1;
        }
    }

    if (event)
        ohci_resume_detected(dev);
}

static void
ohci_port_attach(ohci_port_t *p, usb_dev_t *udev)
{
    p->usb     = udev;
    udev->port = p;
    udev->bus  = p->hc->bus;
    usb_trace(USB_LOG_REG, -1, "OHCI port %i: %s attached", p->num, udev->name);
    ohci_port_detect(p);
    ohci_update_irq(p->hc);
}

/* A disconnect clears CCS and the port state; losing PES is a hardware
   event (p143). */
static void
ohci_port_detach(ohci_port_t *p)
{
    ohci_port_timer_stop(p);
    if (p->sts & OHCI_PS_CCS) {
        if (p->sts & OHCI_PS_PES)
            ohci_port_change(p, OHCI_PS_PESC);
        p->sts &= ~(OHCI_PS_CCS | OHCI_PS_PES | OHCI_PS_PSS | OHCI_PS_PRS | OHCI_PS_LSDA);
        ohci_port_connect_change(p);
    }
    usb_trace(USB_LOG_REG, -1, "OHCI port %i: %s detached, HcRhPortStatus %08x", p->num, p->usb->name, p->sts);
    p->usb = NULL;
    ohci_update_irq(p->hc);
}

enum {
    OHCI_ED_NONE = 0, /* nothing sent: skipped, halted or empty ED */
    OHCI_ED_SENT,     /* a transaction was attempted */
    OHCI_ED_NO_TIME,  /* a TD waits, but its packet does not fit the frame */
    OHCI_ED_END       /* the list is disabled or has nothing left to serve */
};

static void
ohci_read_dwords(uint32_t addr, uint32_t *d, int n)
{
    uint8_t b[32];

    dma_bm_read(addr, b, n * 4, 4);
    for (int i = 0; i < n; i++)
        d[i] = b[i * 4] | (b[i * 4 + 1] << 8) | (b[i * 4 + 2] << 16) | ((uint32_t) b[i * 4 + 3] << 24);
}

static void
ohci_write_dword(uint32_t addr, uint32_t val)
{
    uint8_t b[4] = { val & 0xff, (val >> 8) & 0xff, (val >> 16) & 0xff, val >> 24 };

    dma_bm_write(addr, b, 4, 4);
}

/* Tokens go out on enabled, unsuspended root ports; a low-speed token only
   reaches a low-speed device, or a hub's low-speed port behind a preamble. */
static usb_dev_t *
ohci_find_device(const ohci_t *dev, int addr, int low_speed)
{
    for (int i = 0; i < dev->ndp; i++) {
        const ohci_port_t *p = &dev->port[i];
        usb_dev_t         *usb;

        if ((p->usb == NULL) || ((p->sts & (OHCI_PS_CCS | OHCI_PS_PES | OHCI_PS_PSS | OHCI_PS_PRS)) != (OHCI_PS_CCS | OHCI_PS_PES)))
            continue;
        if ((p->usb->addr == addr) && (p->usb->low_speed == low_speed))
            return p->usb;
        if ((p->usb->ops->route != NULL) && ((usb = p->usb->ops->route(p->usb, addr, low_speed)) != NULL))
            return usb;
    }

    return NULL;
}

/* Bytes left in a General TD's buffer: the 13-bit A - B + 1 (p115). */
static int
ohci_td_remaining(uint32_t cbp, uint32_t be)
{
    uint32_t a = be & 0xfff;

    if (cbp == 0)
        return 0;
    if ((cbp ^ be) & 0xfffff000)
        a |= 0x1000;

    return (a - (cbp & 0xfff) + 1) & 0x1fff;
}

/* n bytes at CBP; past the end of CBP's 4 KB page they continue in BE's
   page, also inside one packet (p35). */
static void
ohci_td_data(uint32_t cbp, uint32_t be, uint8_t *buf, int n, int write)
{
    int first = 0x1000 - (cbp & 0xfff);

    if (n <= 0)
        return;
    if (first > n)
        first = n;

    if (write) {
        dma_bm_write(cbp, buf, first, 1);
        if (n > first)
            dma_bm_write(be & 0xfffff000, buf + first, n - first, 1);
    } else {
        dma_bm_read(cbp, buf, first, 1);
        if (n > first)
            dma_bm_read(be & 0xfffff000, buf + first, n - first, 1);
    }
}

/* The Done Queue Interrupt Counter at a retirement (p118): DelayInterrupt
   111b leaves it, a smaller DI pulls it in, an error clears it. */
static void
ohci_done_delay(ohci_t *dev, uint32_t dw0)
{
    uint8_t di = (dw0 & OHCI_TD_DI) >> 21;

    if ((dw0 & OHCI_TD_CC) != ((uint32_t) OHCI_CC_NOERROR << 28))
        di = 0;
    if ((di != 7) && (di < dev->di_count))
        dev->di_count = di;
}

/* Retirement (p117, p51): the ED moves on to the next TD, the TD joins the
   done queue, toggleCarry takes the TD's last toggle (ED.C if it never
   moved data), and an error halts the ED (p47). */
static void
ohci_td_retire(ohci_t *dev, uint32_t ed_addr, uint32_t *ed, uint32_t td_addr, const uint32_t *td)
{
    int carry = (td[0] & 0x02000000) ? !!(td[0] & 0x01000000) : !!(ed[2] & OHCI_ED_C);

    ed[2] = (td[2] & 0xfffffff0) | (carry ? OHCI_ED_C : 0);
    if ((td[0] & OHCI_TD_CC) != ((uint32_t) OHCI_CC_NOERROR << 28))
        ed[2] |= OHCI_ED_H;
    ohci_write_dword(td_addr + 8, dev->done_head);
    dev->done_head = td_addr;
    ohci_write_dword(ed_addr + 8, ed[2]);
    ohci_done_delay(dev, td[0]);
}

/* One transaction attempt on the ED's head General TD (p113-117). */
static int
ohci_service_gtd(ohci_t *dev, uint32_t ed_addr, uint32_t *ed)
{
    uint32_t     td_addr = ed[2] & 0xfffffff0;
    uint32_t     td[4];
    uint8_t      buf[2048];
    usb_packet_t p;
    usb_dev_t   *usb;
    double       ldpc;
    int          dir;
    int          mps;
    int          ls;
    int          remaining;
    int          size;
    int          toggle;
    int          ret;
    int          ec;
    int          cc;
    int          bytes    = 0;
    int          complete = 0;

    ohci_read_dwords(td_addr, td, 4);
    dev->bits_left -= OHCI_FETCH_BITS;

    /* ED.D, or TD.DP when D is 00b or 11b: 0 SETUP, 1 OUT, 2 IN (p31, p34). */
    dir = (ed[0] >> 11) & 3;
    if ((dir == 0) || (dir == 3))
        dir = (td[0] >> 19) & 3;
    if (dir == 3)
        return OHCI_ED_NONE;

    mps       = (ed[0] >> 16) & 0x7ff;
    ls        = !!(ed[0] & OHCI_ED_S);
    remaining = ohci_td_remaining(td[1], td[3]);
    size      = (remaining < mps) ? remaining : mps;

    /* Nothing starts that could overrun the frame, and it gets no writeback:
       low speed by LSThreshold, full speed by the Largest Data Packet
       Counter, FSMPS at the boundary less 6 per 7 bit times (p115-116). */
    if (dev->bits_left < 1.0)
        return OHCI_ED_NO_TIME;
    if (ls) {
        if ((dev->bits_left - 1.0) < (double) dev->ls_threshold)
            return OHCI_ED_NO_TIME;
    } else {
        ldpc = (double) ((dev->fm_interval >> 16) & 0x7fff) - (((double) dev->fr_load + 1.0 - dev->bits_left) * 6.0 / 7.0);
        if ((double) (((dir == 2) ? mps : size) * 8) > ldpc)
            return OHCI_ED_NO_TIME;
    }

    /* The toggle comes from the TD when its MSb is set, else from ED.C (p35). */
    toggle = (td[0] & 0x02000000) ? ((td[0] >> 24) & 1) : !!(ed[2] & OHCI_ED_C);

    p.pid     = (dir == 0) ? USB_PID_SETUP : ((dir == 1) ? USB_PID_OUT : USB_PID_IN);
    p.ep      = (ed[0] >> 7) & 0x0f;
    p.toggle  = toggle;
    p.data    = buf;
    p.len     = size;
    p.actual  = 0;
    p.bus_len = 0;
    /* An overrun is ACKed: the toggle advances (p37, p38). */
    p.ack_overrun = 1;
    p.no_ack      = 0;

    if (dir != 2)
        ohci_td_data(td[1], td[3], buf, size, 0);

    usb = ohci_find_device(dev, ed[0] & 0x7f, ls);
    ret = (usb != NULL) ? usb_dev_packet(usb, &p) : USB_RET_NO_RESPONSE;

    /* The bus time of the USB 1.1 formulas the UHCI uses, in bit times. An
       IN is charged for the packet the device sent, all of it on an
       overrun (p38), not for what the buffer kept. */
    dev->bits_left -= uhci_bus_ns(ls, 0, dir == 2, (dir == 2) ? p.bus_len : size) * 12.0 / 1000.0;

    ec = (td[0] & OHCI_TD_EC) >> 26;
    cc = OHCI_CC_NOERROR;

    switch (ret) {
        case USB_RET_NAK:
            /* NAK leaves every TD field as it was (p39). */
            break;

        case USB_RET_STALL:
            /* Retired with CBP, EC and T as they were (p39). */
            cc       = OHCI_CC_STALL;
            complete = 1;
            break;

        case USB_RET_ACK:
        case USB_RET_OVERRUN:
            /* The toggle advances and EC goes to 0 (p36, p116). IN data past
               the buffer is a data overrun: kept up to the buffer, CBP left
               at the packet's start (p38). */
            bytes = size;
            if (dir == 2) {
                bytes = p.actual;
                ohci_td_data(td[1], td[3], buf, bytes, 1);
            }
            ec    = 0;
            td[0] = (td[0] & ~OHCI_TD_T) | 0x02000000 | ((toggle ^ 1) << 24);
            if (ret == USB_RET_OVERRUN) {
                cc       = OHCI_CC_DATAOVERRUN;
                complete = 1;
            } else if (bytes == remaining) {
                td[1]    = 0x00000000;
                complete = 1;
            } else {
                if (((td[1] & 0xfff) + bytes) > 0xfff)
                    td[1] = (td[3] & 0xfffff000) | ((td[1] + bytes) & 0xfff);
                else
                    td[1] += bytes;
                /* A short IN packet ends the TD; without bufferRounding it
                   is a data underrun (p37-38). */
                if (bytes < size) {
                    complete = 1;
                    if (!(td[0] & OHCI_TD_R))
                        cc = OHCI_CC_DATAUNDERRUN;
                }
            }
            break;

        default:
            /* Transmission errors (p37): no handshake, or IN data with the
               wrong toggle, which is discarded. The third in a row retires. */
            cc       = (ret == USB_RET_TOGGLE_MISMATCH) ? OHCI_CC_DATATOGGLEMISMATCH : OHCI_CC_DEVICENOTRESPONDING;
            complete = (++ec == 3);
            break;
    }

    if (ret != USB_RET_NAK) {
        td[0] = (td[0] & ~(OHCI_TD_EC | OHCI_TD_CC)) | ((uint32_t) ec << 26) | ((uint32_t) cc << 28);
        ohci_write_dword(td_addr, td[0]);
        ohci_write_dword(td_addr + 4, td[1]);
        if (complete)
            ohci_td_retire(dev, ed_addr, ed, td_addr, td);
    }

    usb_trace(USB_LOG_SCHED, dev->fm_number, "ED %08x TD %08x %s dev %i ep %i D%i len %i%s -> %s, TD %08x CBP %08x%s",
              ed_addr, td_addr, (dir == 0) ? "SETUP" : ((dir == 1) ? "OUT" : "IN"), ed[0] & 0x7f, p.ep, toggle, size,
              ls ? " LS" : "", uhci_ret_names[ret], td[0], td[1],
              complete ? ((ed[2] & OHCI_ED_H) ? ", retired, ED halted" : ", retired") : "");

    return OHCI_ED_SENT;
}

/* One visit of a nonperiodic list (p110-113, Fig 6-5 and 6-6). */
static int
ohci_service_list(ohci_t *dev, int list)
{
    uint32_t *cur    = (list == OHCI_LIST_BULK) ? &dev->bulk_cur : &dev->ctrl_cur;
    uint32_t  head   = (list == OHCI_LIST_BULK) ? dev->bulk_head : dev->ctrl_head;
    uint32_t  filled = (list == OHCI_LIST_BULK) ? OHCI_CS_BLF : OHCI_CS_CLF;
    uint32_t  enable = (list == OHCI_LIST_BULK) ? OHCI_CTL_BLE : OHCI_CTL_CLE;
    uint32_t  ed_addr;
    uint32_t  ed[4];
    int       r;

    if (!(dev->control & enable))
        return OHCI_ED_END;

    /* The first service after entering USBOPERATIONAL starts at the head
       (p111); otherwise CurrentED 0 is the end of the list, and the Filled
       bit says whether to go round again (p132-133). */
    if (dev->np_first[list]) {
        dev->np_first[list] = 0;
        *cur                = head;
    } else if (*cur == 0) {
        if (!(dev->cmdsts & filled))
            return OHCI_ED_END;
        *cur = head;
        dev->cmdsts &= ~filled;
    }
    if (*cur == 0)
        return OHCI_ED_END;

    ed_addr = *cur;
    ohci_read_dwords(ed_addr, ed, 4);
    dev->bits_left -= OHCI_FETCH_BITS;

    /* sKip, Halt or no TD: nothing sent (p113). An isochronous-format ED
       belongs on the periodic list only. */
    if ((ed[0] & (OHCI_ED_K | OHCI_ED_F)) || (ed[2] & OHCI_ED_H) || ((ed[1] & 0xfffffff0) == (ed[2] & 0xfffffff0))) {
        *cur = ed[3] & 0xfffffff0;
        return OHCI_ED_NONE;
    }

    /* A TD found marks the list filled (p50, p112). */
    dev->cmdsts |= filled;
    r = ohci_service_gtd(dev, ed_addr, ed);
    if (r != OHCI_ED_NO_TIME)
        *cur = ed[3] & 0xfffffff0;

    return r;
}

/* The nonperiodic lists (p107-108): CBSR + 1 control EDs that started a
   transaction, then one bulk ED visit; the count carries across frames and
   an empty list yields to the other. Runs until the time or the work is
   gone. */
static void
ohci_run_nonperiodic(ohci_t *dev, double stop)
{
    int idle = 0;

    while ((dev->bits_left > stop) && (idle < 2)) {
        int list = dev->np_list;
        int r    = ohci_service_list(dev, list);

        if (r == OHCI_ED_NO_TIME)
            break;
        idle = (r == OHCI_ED_END) ? (idle + 1) : 0;

        if (list == OHCI_LIST_CONTROL) {
            if ((r == OHCI_ED_END) || ((r == OHCI_ED_SENT) && (--dev->ratio_left <= 0)))
                dev->np_list = OHCI_LIST_BULK;
        } else {
            dev->np_list    = OHCI_LIST_CONTROL;
            dev->ratio_left = (dev->control & OHCI_CTL_CBSR) + 1;
        }
    }
}

/* An isochronous TD retires without touching toggleCarry (p31) and never
   halts the ED (p47). */
static void
ohci_itd_retire(ohci_t *dev, uint32_t ed_addr, uint32_t *ed, uint32_t td_addr, const uint32_t *td)
{
    ed[2] = (td[2] & 0xfffffff0) | (ed[2] & OHCI_ED_C);
    ohci_write_dword(td_addr + 8, dev->done_head);
    dev->done_head = td_addr;
    ohci_write_dword(ed_addr + 8, ed[2]);
    ohci_done_delay(dev, td[0]);
}

/* The Isochronous TD at the ED's head (p39-45): packet R = FrameNumber -
   StartingFrame of FrameCount + 1, once, in its own frame. */
static int
ohci_service_itd(ohci_t *dev, uint32_t ed_addr, uint32_t *ed)
{
    uint32_t     td_addr;
    uint32_t     td[8];
    uint32_t     start;
    uint32_t     end;
    uint32_t     s13;
    uint32_t     e13;
    uint32_t     off;
    uint8_t      buf[2048];
    uint8_t      psw[2];
    usb_packet_t p;
    usb_dev_t   *usb;
    double       ldpc;
    int          dir = (ed[0] >> 11) & 3;
    int          mps = (ed[0] >> 16) & 0x7ff;
    int          r;
    int          fc;
    int          size;
    int          ret;
    int          cc;
    int          len = 0;

    /* An isochronous TD has no direction of its own (p31, p39). */
    if ((dir != 1) && (dir != 2))
        return OHCI_ED_NONE;

    for (;;) {
        td_addr = ed[2] & 0xfffffff0;
        if (td_addr == (ed[1] & 0xfffffff0))
            return OHCI_ED_NONE;

        ohci_read_dwords(td_addr, td, 8);
        dev->bits_left -= OHCI_FETCH_BITS;

        r  = (int16_t) (dev->fm_number - (td[0] & 0xffff));
        fc = (td[0] >> 24) & 7;
        if (r < 0)
            return OHCI_ED_NONE;
        if (r <= fc)
            break;

        /* Expired: retired with DATAOVERRUN, the ED not halted, and the next
           TD of the ED is processed (p43). */
        td[0] = (td[0] & ~OHCI_TD_CC) | ((uint32_t) OHCI_CC_DATAOVERRUN << 28);
        ohci_write_dword(td_addr, td[0]);
        ohci_itd_retire(dev, ed_addr, ed, td_addr, td);
        usb_trace(USB_LOG_SCHED, dev->fm_number, "ED %08x ITD %08x expired (R %i > FC %i), retired DATAOVERRUN", ed_addr, td_addr, r, fc);
    }

    /* The packet's buffer: Offset[R] to Offset[R+1] - 1, or to BE for the
       last one; bit 12 of an offset picks BP0's page or BE's (p41). */
    off   = (td[4 + (r >> 1)] >> ((r & 1) << 4)) & 0x1fff;
    s13   = off;
    start = ((off & 0x1000) ? (td[3] & 0xfffff000) : (td[1] & 0xfffff000)) | (off & 0xfff);
    if (r < fc) {
        e13 = (((td[4 + ((r + 1) >> 1)] >> (((r + 1) & 1) << 4)) & 0x1fff) - 1) & 0x1fff;
        end = ((e13 & 0x1000) ? (td[3] & 0xfffff000) : (td[1] & 0xfffff000)) | (e13 & 0xfff);
    } else {
        e13 = (((td[3] ^ td[1]) & 0xfffff000) ? 0x1000 : 0) | (td[3] & 0xfff);
        end = td[3];
    }
    size = (e13 + 1 - s13) & 0x1fff;
    if (size > (int) sizeof(buf))
        size = sizeof(buf);

    if (dev->bits_left < 1.0)
        return OHCI_ED_NO_TIME;
    ldpc = (double) ((dev->fm_interval >> 16) & 0x7fff) - (((double) dev->fr_load + 1.0 - dev->bits_left) * 6.0 / 7.0);
    if ((double) (((dir == 2) ? mps : size) * 8) > ldpc)
        return OHCI_ED_NO_TIME;

    p.pid    = (dir == 1) ? USB_PID_OUT : USB_PID_IN;
    p.ep     = (ed[0] >> 7) & 0x0f;
    p.toggle = 0;
    p.data   = buf;
    p.len    = (dir == 1) ? size : ((mps > size) ? mps : size);
    p.actual = 0;
    p.ack_overrun = 1;
    p.no_ack      = 0;
    if (p.len > (int) sizeof(buf))
        p.len = sizeof(buf);

    if (dir == 1)
        ohci_td_data(start, end, buf, size, 0);

    usb = ohci_find_device(dev, ed[0] & 0x7f, 0);
    ret = (usb != NULL) ? usb_dev_packet(usb, &p) : USB_RET_NO_RESPONSE;
    dev->bits_left -= uhci_bus_ns(0, 1, dir == 2, (dir == 2) ? p.actual : size) * 12.0 / 1000.0;

    /* The PSW (p42-45): an OUT has no handshake, so it ends without error;
       an IN reports what arrived, up to the buffer. */
    if (dir == 1)
        cc = OHCI_CC_NOERROR;
    else if ((ret == USB_RET_ACK) || (ret == USB_RET_OVERRUN)) {
        len = p.actual;
        if (len > size) {
            len = size;
            cc  = OHCI_CC_DATAOVERRUN;
        } else
            cc = (len < size) ? OHCI_CC_DATAUNDERRUN : OHCI_CC_NOERROR;
        ohci_td_data(start, end, buf, len, 1);
    } else if ((ret == USB_RET_NAK) || (ret == USB_RET_STALL))
        cc = OHCI_CC_STALL;
    else
        cc = OHCI_CC_DEVICENOTRESPONDING;

    psw[0] = len & 0xff;
    psw[1] = (cc << 4) | ((len >> 8) & 0x07);
    dma_bm_write(td_addr + 16 + (r << 1), psw, 2, 2);

    usb_trace(USB_LOG_SCHED, dev->fm_number, "ED %08x ITD %08x %s dev %i ep %i packet %i/%i len %i -> %s, PSW %02x%02x%s",
              ed_addr, td_addr, (dir == 1) ? "OUT" : "IN", ed[0] & 0x7f, p.ep, r, fc, size, uhci_ret_names[ret],
              psw[1], psw[0], (r == fc) ? ", retired" : "");

    /* The last packet completes the TD with NOERROR (p42). */
    if (r == fc) {
        td[0] = (td[0] & ~OHCI_TD_CC) | ((uint32_t) OHCI_CC_NOERROR << 28);
        ohci_write_dword(td_addr, td[0]);
        ohci_itd_retire(dev, ed_addr, ed, td_addr, td);
    }

    return OHCI_ED_SENT;
}

/* The periodic list (p110-111): the interrupt list of this frame from the
   HCCA, isochronous EDs at its tail, followed to its end in one pass. If
   it cannot finish in the frame, that is a scheduling overrun (p119). */
static void
ohci_run_periodic(ohci_t *dev)
{
    uint32_t ed_addr;
    uint32_t ed[4];
    int      r;

    if (!(dev->control & OHCI_CTL_PLE))
        return;

    ohci_read_dwords(dev->hcca + ((dev->fm_number & 0x1f) << 2), &ed_addr, 1);
    dev->bits_left -= OHCI_FETCH_BITS;
    ed_addr &= 0xfffffff0;
    dev->period_cur = ed_addr;

    while (ed_addr != 0) {
        if (dev->bits_left < 1.0) {
            dev->so_pending = 1;
            break;
        }

        ohci_read_dwords(ed_addr, ed, 4);
        dev->bits_left -= OHCI_FETCH_BITS;

        /* IsochronousEnable clear ends the list at its first isochronous ED (p109). */
        if ((ed[0] & OHCI_ED_F) && !(dev->control & OHCI_CTL_IE))
            break;

        if (!(ed[0] & OHCI_ED_K) && !(ed[2] & OHCI_ED_H) && ((ed[1] & 0xfffffff0) != (ed[2] & 0xfffffff0))) {
            r = (ed[0] & OHCI_ED_F) ? ohci_service_itd(dev, ed_addr, ed) : ohci_service_gtd(dev, ed_addr, ed);
            if (r == OHCI_ED_NO_TIME) {
                dev->so_pending = 1;
                break;
            }
        }

        ed_addr         = ed[3] & 0xfffffff0;
        dev->period_cur = ed_addr;
    }

    if (dev->so_pending)
        usb_trace(USB_LOG_SCHED, dev->fm_number, "periodic list overran the frame at ED %08x", ed_addr);
}

/* The frame's schedule, run at its boundary against its bit time: the
   nonperiodic lists while FrameRemaining > PeriodicStart, the periodic list
   to completion, then the nonperiodic lists again (p106-107). With Bus
   Master Enable clear no descriptor is read (p148). */
static void
ohci_run_frame(ohci_t *dev)
{
    if (!dev->bus_master)
        return;

    dev->bits_left = (double) dev->fr_load + 1.0;
    ohci_run_nonperiodic(dev, (double) dev->periodic_start + 1.0);
    ohci_run_periodic(dev);
    ohci_run_nonperiodic(dev, 0.0);
}

/* A frame is FrameInterval + 1 bit times of 12 MHz (p104). */
static double
ohci_frame_us(uint32_t fi)
{
    return ((double) (fi & 0x3fff) + 1.0) / 12.0;
}

/* FrameRemaining from the frame timer: FI in the first bit time of the
   frame, 0 in the last (p104). */
static uint32_t
ohci_fr_live(ohci_t *dev)
{
    double   bits = ((double) timer_get_remaining_u64(&dev->frame_timer) / (double) TIMER_USEC) * 12.0;
    uint32_t fr   = (bits >= 1.0) ? (uint32_t) ceil(bits) - 1 : 0;

    return (fr > dev->fr_load) ? dev->fr_load : fr;
}

/* It counts only in USBOPERATIONAL (ALi-C p182) and holds otherwise. */
static uint32_t
ohci_fm_remaining(ohci_t *dev)
{
    if ((dev->control & OHCI_CTL_HCFS) != OHCI_HCFS_OPERATIONAL)
        return dev->fm_remaining;

    return (dev->fm_remaining & 0x80000000) | ohci_fr_live(dev);
}

/* Entering USBOPERATIONAL loads FrameRemaining and increments FrameNumber,
   with no SOF; the first SOF is at the next frame boundary (p102). */
static void
ohci_frame_start(ohci_t *dev)
{
    dev->fr_load      = dev->fm_interval & 0x3fff;
    dev->fm_remaining = (dev->fm_remaining & 0x80000000) | dev->fr_load;
    dev->fm_number    = (dev->fm_number + 1) & 0xffff;
    /* Nonperiodic service starts with the full control count (p108), each
       list from its head (p111). */
    dev->np_list     = OHCI_LIST_CONTROL;
    dev->ratio_left  = (dev->control & OHCI_CTL_CBSR) + 1;
    dev->np_first[0] = dev->np_first[1] = 1;
    dev->di_count    = 7; /* p118 */
    timer_on_auto(&dev->frame_timer, ohci_frame_us(dev->fr_load));
    usb_trace(USB_LOG_REG, dev->fm_number, "OHCI frame clock started, HcHCCA %08x", dev->hcca);
}

static void
ohci_frame_stop(ohci_t *dev)
{
    if (!(dev->frame_timer.flags & TIMER_ENABLED))
        return;

    dev->fm_remaining = (dev->fm_remaining & 0x80000000) | ohci_fr_live(dev);
    timer_stop(&dev->frame_timer);
    usb_trace(USB_LOG_REG, dev->fm_number, "OHCI frame clock stopped, HcFmRemaining %08x", dev->fm_remaining);
}

/* The done queue at a frame boundary (p118-119): the counter as it stood in
   the frame's last bit time. At 0 with WDH clear, HccaDoneHead takes
   HcDoneHead, its LSb set when another enabled status bit is set (p49);
   otherwise the counter counts down, 111b and 0 holding. */
static void
ohci_done_writeback(ohci_t *dev)
{
    uint32_t head;
    uint8_t  b[4];

    if (dev->di_count != 0) {
        if (dev->di_count != 7)
            dev->di_count--;
        return;
    }
    if ((dev->intsts & OHCI_INT_WDH) || !dev->bus_master)
        return;

    head = dev->done_head;
    if (dev->intsts & dev->inten & OHCI_INT_ALL & ~OHCI_INT_WDH)
        head |= 1;
    b[0] = head & 0xff;
    b[1] = (head >> 8) & 0xff;
    b[2] = (head >> 16) & 0xff;
    b[3] = head >> 24;
    dma_bm_write(dev->hcca + 0x84, b, 4, 4);
    dev->done_head = 0x00000000;
    dev->intsts |= OHCI_INT_WDH;
    dev->di_count = 7;
    usb_trace(USB_LOG_SCHED, dev->fm_number, "HccaDoneHead <- %08x, WritebackDoneHead set", head);
}

/* The frame boundary, the 0 -> FI transition of FrameRemaining (p104). */
static void
ohci_frame_boundary(void *priv)
{
    ohci_t *dev = (ohci_t *) priv;
    uint8_t hcca_fn[4];

    /* FR reloads with the FI in force now and FRT follows FIT (p135). */
    dev->fr_load      = dev->fm_interval & 0x3fff;
    dev->fm_remaining = (dev->fm_interval & 0x80000000) | dev->fr_load;
    dev->fm_number    = (dev->fm_number + 1) & 0xffff;
    timer_on_auto(&dev->frame_timer, ohci_frame_us(dev->fr_load));

    /* SOF, then HccaFrameNumber with HccaPad1 = 0 before the first ED of
       the frame, then StartOfFrame (p48-49, p104). Without Bus Master
       Enable there is no HCCA write (p148). */
    if (dev->bus_master) {
        hcca_fn[0] = dev->fm_number & 0xff;
        hcca_fn[1] = dev->fm_number >> 8;
        hcca_fn[2] = hcca_fn[3] = 0x00;
        dma_bm_write(dev->hcca + 0x80, hcca_fn, 4, 4);
    }
    /* FrameNumberOverflow: bit 15 changed since the last update (p120). */
    if ((dev->fm_number & 0x8000) != dev->fn_msb) {
        dev->fn_msb = dev->fm_number & 0x8000;
        dev->intsts |= OHCI_INT_FNO;
    }
    /* SchedulingOverrun of the last frame, with its count (p119, p127). */
    if (dev->so_pending) {
        dev->so_pending = 0;
        dev->intsts |= OHCI_INT_SO;
        dev->cmdsts = (dev->cmdsts & ~OHCI_CS_SOC) | ((dev->cmdsts + 0x00010000) & OHCI_CS_SOC);
    }
    dev->intsts |= OHCI_INT_SF;
    ohci_done_writeback(dev);

    ohci_run_frame(dev);
    ohci_update_irq(dev);
}

/* HCR (p126): the operational registers go to their reset values, FI to
   nominal (p135), except InterruptRouting and RemoteWakeupConnected (p125).
   The root hub (p146) and the legacy support registers keep their contents. */
static void
ohci_soft_reset(ohci_t *dev)
{
    timer_stop(&dev->frame_timer);
    dev->so_pending     = 0;
    dev->di_count       = 7;
    dev->fn_msb         = 0x0000;
    dev->fr_load        = 0x0000;
    dev->control        = (dev->control & (OHCI_CTL_IR | OHCI_CTL_RWC)) | OHCI_HCFS_SUSPEND;
    dev->cmdsts         = 0x00000000;
    dev->intsts         = 0x00000000;
    dev->inten          = 0x00000000;
    dev->hcca           = 0x00000000;
    dev->period_cur     = 0x00000000;
    dev->ctrl_head      = 0x00000000;
    dev->ctrl_cur       = 0x00000000;
    dev->bulk_head      = 0x00000000;
    dev->bulk_cur       = 0x00000000;
    dev->done_head      = 0x00000000;
    dev->fm_interval    = OHCI_FI_NOMINAL;
    dev->fm_remaining   = 0x00000000;
    dev->fm_number      = 0x00000000;
    dev->periodic_start = 0x00000000;
    /* 0628h in the spec (p137); ALi and SiS reset it to 0. */
    dev->ls_threshold = (dev->chip_flags & OHCI_CHIP_LST_0628H) ? 0x00000628 : 0x00000000;
}

static void
ohci_hard_reset(ohci_t *dev)
{
    ohci_soft_reset(dev);
    dev->control   = OHCI_HCFS_RESET;
    dev->rh_desc_a = 0x01000000 | dev->ndp;
    dev->rh_desc_b = 0x00000000;
    ohci_root_hub_reset(dev);
    dev->hce_control = 0x00000000;
    dev->hce_input   = 0x00;
    dev->hce_output  = 0x00;
    dev->hce_status  = 0x00;
}

static void
ohci_control_write(ohci_t *dev, uint32_t val)
{
    uint32_t old  = dev->control;
    uint32_t mask = OHCI_CTL_CBSR | OHCI_CTL_PLE | OHCI_CTL_IE | OHCI_CTL_CLE | OHCI_CTL_BLE | OHCI_CTL_HCFS | OHCI_CTL_IR | OHCI_CTL_RWE;

    /* RemoteWakeupConnected is hard 0 on ALi, SiS and SMSC. */
    if (dev->chip_flags & OHCI_CHIP_RWC)
        mask |= OHCI_CTL_RWC;
    dev->control = val & mask;

    /* OCR clears after the changeover (p127), the InterruptRouting change
       that hands the HC over (p55-56, p125). */
    if (((dev->control ^ old) & OHCI_CTL_IR) && (dev->cmdsts & OHCI_CS_OCR)) {
        dev->cmdsts &= ~OHCI_CS_OCR;
        usb_trace(USB_LOG_IRQ, -1, "OHCI InterruptRouting %i -> %i: ownership changeover, OwnershipChangeRequest cleared",
                  !!(old & OHCI_CTL_IR), !!(dev->control & OHCI_CTL_IR));
    }

    if ((dev->control ^ old) & OHCI_CTL_HCFS) {
        usb_trace(USB_LOG_REG, -1, "OHCI HCFS %s -> %s", ohci_hcfs_name(old), ohci_hcfs_name(dev->control));
        if ((old & OHCI_CTL_HCFS) == OHCI_HCFS_OPERATIONAL)
            ohci_frame_stop(dev);
        else if ((dev->control & OHCI_CTL_HCFS) == OHCI_HCFS_OPERATIONAL)
            ohci_frame_start(dev);
        switch (dev->control & OHCI_CTL_HCFS) {
            case OHCI_HCFS_RESET:
                /* The root hub is held in reset while in USBRESET (p57). */
                ohci_root_hub_reset(dev);
                break;
            case OHCI_HCFS_RESUME:
                ohci_rh_global_resume(dev);
                break;
            case OHCI_HCFS_SUSPEND:
                ohci_rh_suspend(dev);
                break;
            default:
                break;
        }
        if ((old & OHCI_CTL_HCFS) == OHCI_HCFS_RESET)
            ohci_rh_detect_all(dev);
    }
}

/* EmulationInterrupt, a static decode of the emulation interrupt condition
   (p160). */
static int
ohci_emulation_interrupt(const ohci_t *dev)
{
    uint32_t ctl = dev->hce_control;

    if ((ctl & OHCI_HCE_EE) && (ctl & OHCI_HCE_CP) && !(dev->hce_status & OHCI_HCES_OUTFULL))
        return 1;
    if ((ctl & OHCI_HCE_EE) && (dev->hce_status & OHCI_HCES_INFULL))
        return 1;
    return (ctl & OHCI_HCE_EXTIRQEN) && (ctl & (OHCI_HCE_IRQ1ACT | OHCI_HCE_IRQ12ACT));
}

static uint32_t
ohci_reg_read(ohci_t *dev, uint32_t off)
{
    uint32_t n;

    switch (off) {
        case OHCI_HCREVISION:
            /* Revision 1.0, legacy support registers present (p123). */
            return 0x00000110;
        case OHCI_HCCONTROL:
            return dev->control;
        case OHCI_HCCOMMANDSTATUS:
            return dev->cmdsts;
        case OHCI_HCINTERRUPTSTATUS:
            return dev->intsts;
        case OHCI_HCINTERRUPTENABLE:
        case OHCI_HCINTERRUPTDISABLE:
            return dev->inten;
        case OHCI_HCHCCA:
            return dev->hcca;
        case OHCI_HCPERIODCURRENTED:
            return dev->period_cur;
        case OHCI_HCCONTROLHEADED:
            return dev->ctrl_head;
        case OHCI_HCCONTROLCURRENTED:
            return dev->ctrl_cur;
        case OHCI_HCBULKHEADED:
            return dev->bulk_head;
        case OHCI_HCBULKCURRENTED:
            return dev->bulk_cur;
        case OHCI_HCDONEHEAD:
            return dev->done_head;
        case OHCI_HCFMINTERVAL:
            return dev->fm_interval;
        case OHCI_HCFMREMAINING:
            return ohci_fm_remaining(dev);
        case OHCI_HCFMNUMBER:
            return dev->fm_number;
        case OHCI_HCPERIODICSTART:
            return dev->periodic_start;
        case OHCI_HCLSTHRESHOLD:
            return dev->ls_threshold;
        case OHCI_HCRHDESCRIPTORA:
            return dev->rh_desc_a;
        case OHCI_HCRHDESCRIPTORB:
            return dev->rh_desc_b;
        case OHCI_HCRHSTATUS:
            return dev->rh_status;
        case OHCI_HCECONTROL:
            return dev->hce_control | (ohci_emulation_interrupt(dev) ? OHCI_HCE_EI : 0);
        case OHCI_HCEINPUT:
            return dev->hce_input;
        case OHCI_HCEOUTPUT:
            return dev->hce_output;
        case OHCI_HCESTATUS:
            return dev->hce_status;
        default:
            n = (off - OHCI_HCRHPORTSTATUS1) >> 2;
            if ((off >= OHCI_HCRHPORTSTATUS1) && (n < (uint32_t) dev->ndp))
                return ohci_port_read(&dev->port[n]);
            /* The rest of the window need not respond (p149). */
            return 0x00000000;
    }
}

static uint32_t
ohci_merge(uint32_t cur, uint32_t val, uint32_t lanes)
{
    return (cur & ~lanes) | (val & lanes);
}

/* lanes selects the bytes the guest wrote (PCI byte enables): R/W bits in
   them merge, and unwritten bytes write 0 to the write-1 command bits. */
static void
ohci_reg_write(ohci_t *dev, uint32_t off, uint32_t val, uint32_t lanes)
{
    uint32_t cmd = val & lanes;
    uint32_t ports;
    uint32_t n;
    int      was_reset;

    switch (off) {
        case OHCI_HCCONTROL:
            ohci_control_write(dev, ohci_merge(dev->control, val, lanes));
            break;

        case OHCI_HCCOMMANDSTATUS:
            if (cmd & OHCI_CS_HCR) {
                /* Completes at once, so the self-clearing bit already reads 0. */
                usb_trace(USB_LOG_REG, -1, "OHCI HostControllerReset: %s -> USBSUSPEND, InterruptRouting %i kept",
                          ohci_hcfs_name(dev->control), !!(dev->control & OHCI_CTL_IR));
                was_reset = ohci_rh_in_reset(dev);
                ohci_soft_reset(dev);
                /* Out of USBRESET, the root hub is no longer held in reset. */
                if (was_reset)
                    ohci_rh_detect_all(dev);
                break;
            }
            dev->cmdsts |= cmd & (OHCI_CS_CLF | OHCI_CS_BLF | OHCI_CS_OCR);
            /* OC is tied to 0 without an SMI pin (p127). */
            if ((cmd & OHCI_CS_OCR) && !(dev->chip_flags & OHCI_CHIP_NO_SMI)) {
                dev->intsts |= OHCI_INT_OC;
                usb_trace(USB_LOG_IRQ, -1, "OHCI OwnershipChangeRequest: HcInterruptStatus.OC set");
            }
            break;

        case OHCI_HCINTERRUPTSTATUS:
            cmd &= OHCI_INT_ALL;
            if (cmd & dev->intsts) {
                usb_trace(USB_LOG_IRQ, -1, "OHCI HcInterruptStatus %08x -> %08x (guest clear %08x)",
                          dev->intsts, dev->intsts & ~cmd, cmd);
                dev->intsts &= ~cmd;
            }
            break;

        /* UnrecoverableError is not implemented, its enable ignores writes
           (ALi-C p180, SMSC p106). */
        case OHCI_HCINTERRUPTENABLE:
            dev->inten |= cmd & ((OHCI_INT_ALL | OHCI_INT_MIE) & ~OHCI_INT_UE);
            break;
        case OHCI_HCINTERRUPTDISABLE:
            dev->inten &= ~(cmd & ((OHCI_INT_ALL | OHCI_INT_MIE) & ~OHCI_INT_UE));
            break;

        case OHCI_HCHCCA:
            dev->hcca = ohci_merge(dev->hcca, val, lanes) & 0xffffff00;
            break;
        /* The current and done pointers are R/W on ALi, SiS and SMSC. */
        case OHCI_HCPERIODCURRENTED:
            dev->period_cur = ohci_merge(dev->period_cur, val, lanes) & 0xfffffff0;
            break;
        case OHCI_HCCONTROLHEADED:
            dev->ctrl_head = ohci_merge(dev->ctrl_head, val, lanes) & 0xfffffff0;
            break;
        case OHCI_HCCONTROLCURRENTED:
            dev->ctrl_cur = ohci_merge(dev->ctrl_cur, val, lanes) & 0xfffffff0;
            break;
        case OHCI_HCBULKHEADED:
            dev->bulk_head = ohci_merge(dev->bulk_head, val, lanes) & 0xfffffff0;
            break;
        case OHCI_HCBULKCURRENTED:
            dev->bulk_cur = ohci_merge(dev->bulk_cur, val, lanes) & 0xfffffff0;
            break;
        case OHCI_HCDONEHEAD:
            dev->done_head = ohci_merge(dev->done_head, val, lanes) & 0xfffffff0;
            break;

        case OHCI_HCFMINTERVAL:
            dev->fm_interval = ohci_merge(dev->fm_interval, val, lanes) & 0xffff3fff;
            break;
        case OHCI_HCPERIODICSTART:
            dev->periodic_start = ohci_merge(dev->periodic_start, val, lanes) & 0x00003fff;
            break;
        case OHCI_HCLSTHRESHOLD:
            dev->ls_threshold = ohci_merge(dev->ls_threshold, val, lanes) & 0x00000fff;
            break;

        case OHCI_HCRHDESCRIPTORA:
            /* PSM, NPS, OCPM, NOCP; POTPGT bits 25:24 only (ALi-C p185,
               SMSC p111). A NoPowerSwitching change powers ports on or off. */
            ports = 0;
            for (int i = 0; i < dev->ndp; i++)
                ports |= ohci_port_powered(&dev->port[i]) << i;
            dev->rh_desc_a = (ohci_merge(dev->rh_desc_a, val, lanes) & 0x03001b00) | dev->ndp;
            for (int i = 0; i < dev->ndp; i++) {
                if ((ports & (1 << i)) && !ohci_port_powered(&dev->port[i]))
                    ohci_port_power_off(&dev->port[i]);
                else
                    ohci_port_detect(&dev->port[i]);
            }
            break;
        case OHCI_HCRHDESCRIPTORB:
            /* Bit 0 of each field is reserved, and so are ports past NDP. */
            ports          = ((1u << dev->ndp) - 1) << 1;
            dev->rh_desc_b = ohci_merge(dev->rh_desc_b, val, lanes) & (ports | (ports << 16));
            break;

        case OHCI_HCRHSTATUS:
            ohci_rh_status_write(dev, cmd);
            break;

        case OHCI_HCECONTROL:
            /* IRQ1Active and IRQ12Active are write-1-to-clear. */
            dev->hce_control = (ohci_merge(dev->hce_control, val, lanes) & 0x0000013d) |
                               (dev->hce_control & ~cmd & (OHCI_HCE_IRQ1ACT | OHCI_HCE_IRQ12ACT));
            break;
        case OHCI_HCEINPUT:
            if (lanes & 0xff)
                dev->hce_input = val;
            break;
        case OHCI_HCEOUTPUT:
            if (lanes & 0xff)
                dev->hce_output = val;
            break;
        case OHCI_HCESTATUS:
            if (lanes & 0xff)
                dev->hce_status = val;
            break;

        default:
            n = (off - OHCI_HCRHPORTSTATUS1) >> 2;
            if ((off >= OHCI_HCRHPORTSTATUS1) && (n < (uint32_t) dev->ndp))
                ohci_port_write(&dev->port[n], cmd);
            /* Read-only registers and the rest of the window take no write. */
            break;
    }

    ohci_update_irq(dev);
}

static uint8_t
ohci_mmio_readb(uint32_t addr, void *priv)
{
    ohci_t  *dev = (ohci_t *) priv;
    uint32_t off = addr & 0xfff;
    uint8_t  ret = ohci_reg_read(dev, off & 0xffc) >> ((off & 3) << 3);

    usb_trace(USB_LOG_REG, -1, "R8  +%03x %-18s = %02x", off, ohci_reg_name(off), ret);

    return ret;
}

static uint16_t
ohci_mmio_readw(uint32_t addr, void *priv)
{
    ohci_t  *dev = (ohci_t *) priv;
    uint32_t off = addr & 0xfff;
    uint16_t ret;

    if ((off & 3) == 3)
        ret = (ohci_reg_read(dev, off & 0xffc) >> 24) | (ohci_reg_read(dev, (off + 1) & 0xffc) << 8);
    else
        ret = ohci_reg_read(dev, off & 0xffc) >> ((off & 3) << 3);

    usb_trace(USB_LOG_REG, -1, "R16 +%03x %-18s = %04x", off, ohci_reg_name(off), ret);

    return ret;
}

static uint32_t
ohci_mmio_readl(uint32_t addr, void *priv)
{
    ohci_t  *dev = (ohci_t *) priv;
    uint32_t off = addr & 0xffc;
    uint32_t ret = ohci_reg_read(dev, off);

    usb_trace(USB_LOG_REG, -1, "R32 +%03x %-18s = %08x", off, ohci_reg_name(off), ret);

    return ret;
}

static void
ohci_mmio_writeb(uint32_t addr, uint8_t val, void *priv)
{
    ohci_t  *dev = (ohci_t *) priv;
    uint32_t off = addr & 0xfff;
    int      sh  = (off & 3) << 3;

    usb_trace(USB_LOG_REG, -1, "W8  +%03x %-18s <- %02x", off, ohci_reg_name(off), val);

    ohci_reg_write(dev, off & 0xffc, (uint32_t) val << sh, 0xffu << sh);
}

static void
ohci_mmio_writew(uint32_t addr, uint16_t val, void *priv)
{
    ohci_t  *dev = (ohci_t *) priv;
    uint32_t off = addr & 0xfff;
    int      sh  = (off & 3) << 3;

    usb_trace(USB_LOG_REG, -1, "W16 +%03x %-18s <- %04x", off, ohci_reg_name(off), val);

    if ((off & 3) == 3) {
        ohci_reg_write(dev, off & 0xffc, (uint32_t) val << 24, 0xff000000);
        ohci_reg_write(dev, (off + 1) & 0xffc, val >> 8, 0x000000ff);
    } else
        ohci_reg_write(dev, off & 0xffc, (uint32_t) val << sh, 0xffffu << sh);
}

static void
ohci_mmio_writel(uint32_t addr, uint32_t val, void *priv)
{
    ohci_t  *dev = (ohci_t *) priv;
    uint32_t off = addr & 0xffc;

    usb_trace(USB_LOG_REG, -1, "W32 +%03x %-18s <- %08x", off, ohci_reg_name(off), val);

    ohci_reg_write(dev, off, val, 0xffffffff);
}

void
ohci_update_mem_mapping(usb_t *dev, uint8_t base1, uint8_t base2, uint8_t base3, int enable)
{
    if (dev->ohci_enable && (dev->ohci_mem_base != 0x00000000))
        mem_mapping_disable(&dev->ohci_mmio_mapping);

    dev->ohci_mem_base = ((base1 << 8) | (base2 << 16) | ((uint32_t) base3 << 24)) & 0xfffff000;
    dev->ohci_enable   = enable;

    if (dev->ohci_enable && (dev->ohci_mem_base != 0x00000000))
        mem_mapping_set_addr(&dev->ohci_mmio_mapping, dev->ohci_mem_base, 0x1000);

    usb_trace(USB_LOG_REG, -1, "OHCI MMIO %08x %s", dev->ohci_mem_base, dev->ohci_enable ? "enabled" : "disabled");
}

void
ohci_set_irq_handler(usb_t *dev, void (*set_irq)(void *priv, int level), void *priv)
{
    dev->ohci->set_irq  = set_irq;
    dev->ohci->irq_priv = priv;
}

void
ohci_set_smi_handler(usb_t *dev, void (*set_smi)(void *priv, int level), void *priv)
{
    dev->ohci->set_smi  = set_smi;
    dev->ohci->smi_priv = priv;
}

void
ohci_set_bus_master(usb_t *dev, int enable)
{
    dev->ohci->bus_master = !!enable;
}

void
ohci_hardwire_bus_master(usb_t *dev)
{
    dev->ohci->bus_master_hardwired = 1;
    dev->ohci->bus_master           = 1;
}

void
ohci_set_chip_flags(usb_t *dev, uint32_t flags)
{
    dev->ohci->chip_flags = flags;
    /* After usb_init's reset: LST takes its reset value now; RWC and OC
       reset to 0 on every chip. */
    dev->ohci->ls_threshold = (flags & OHCI_CHIP_LST_0628H) ? 0x00000628 : 0x00000000;
}

/* A port past NDP is not implemented (p143): it drops to its reset value
   with no change reported, its device unpowered but still plugged in. A
   port brought in starts from reset under the global power switch (p139). */
void
ohci_set_ndp(usb_t *dev, int ndp, int ports)
{
    ohci_t  *oc  = dev->ohci;
    int      old = oc->ndp;
    uint32_t pps = 0;
    uint32_t mask;

    oc->ports = ports;
    if (ndp == old)
        return;

    usb_trace(USB_LOG_REG, -1, "OHCI NumberDownstreamPorts %i -> %i", old, ndp);
    for (int i = 0; i < old; i++) {
        if (!(oc->rh_desc_a & OHCI_DA_PSM) || !(oc->rh_desc_b & (0x10000 << (i + 1)))) {
            pps = oc->port[i].sts & OHCI_PS_PPS;
            break;
        }
    }
    for (int i = ndp; i < old; i++) {
        ohci_port_timer_stop(&oc->port[i]);
        oc->port[i].sts = 0x00000000;
        if (oc->port[i].usb != NULL) {
            usb_dev_bus_reset(oc->port[i].usb);
            oc->port[i].usb->powered_reset = 0;
        }
    }

    oc->ndp       = ndp;
    mask          = ((1u << ndp) - 1) << 1;
    oc->rh_desc_a = (oc->rh_desc_a & ~0xffu) | ndp;
    oc->rh_desc_b &= mask | (mask << 16);
    for (int i = old; i < ndp; i++) {
        oc->port[i].sts = pps;
        ohci_port_detect(&oc->port[i]);
    }
    ohci_update_irq(oc);
}

static void
usb_reset(void *priv)
{
    usb_t *dev = (usb_t *) priv;

    uhci_power_on_reset(dev->uhci);
    /* PCICMD resets to 0; GRESET leaves PCI configuration alone. */
    dev->uhci->bus_master = 0;
    /* Devices stay on the wire through a chip reset and connect again (s2.1.7). */
    uhci_schedule_detect(dev->uhci);

    ohci_hard_reset(dev->ohci);
    dev->ohci->bus_master = dev->ohci->bus_master_hardwired;
    ohci_update_irq(dev->ohci);

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
    for (int i = 0; i < OHCI_MAX_PORTS; i++) {
        if (dev->ohci->port[i].usb != NULL)
            dev->ohci->port[i].usb->port = NULL;
        timer_disable(&dev->ohci->port[i].timer);
    }
    timer_disable(&dev->ohci->frame_timer);

    timer_disable(&dev->uhci->frame_timer);
    timer_disable(&dev->uhci->detect_timer);
    free(dev->uhci);
    free(dev->ohci);
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

    dev->ohci        = (ohci_t *) calloc(1, sizeof(ohci_t));
    dev->ohci->ndp   = 2;
    dev->ohci->ports = 2;
    timer_add(&dev->ohci->frame_timer, ohci_frame_boundary, dev->ohci, 0);
    for (int i = 0; i < OHCI_MAX_PORTS; i++) {
        dev->ohci->port[i].num = i + 1;
        dev->ohci->port[i].hc  = dev->ohci;
        timer_add(&dev->ohci->port[i].timer, ohci_port_timer, &dev->ohci->port[i], 0);
    }

    /* A board uses the UHCI or the OHCI of its usb_t, so they share a bus number. */
    for (int h = 0; h < USB_MAX_HC; h++) {
        if (usb_hcs[h] == NULL) {
            usb_hcs[h]     = dev;
            dev->uhci->bus = h + 1;
            dev->ohci->bus = h + 1;
            break;
        }
    }

    mem_mapping_add(&dev->ohci_mmio_mapping, 0, 0,
                    ohci_mmio_readb, ohci_mmio_readw, ohci_mmio_readl,
                    ohci_mmio_writeb, ohci_mmio_writew, ohci_mmio_writel,
                    NULL, MEM_MAPPING_EXTERNAL, dev->ohci);
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
