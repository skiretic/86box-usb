/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          USB mass storage, written from the USB Mass Storage Class
 *          Bulk-Only Transport 1.0: a bridge to one 86Box SCSI bus
 *          whose drives are its logical units.
 *
 * Authors: skiretic, <skiretic@proton.me>
 *
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
#include <86box/mem.h>
#include <86box/scsi.h>
#include <86box/scsi_device.h>
#include <86box/usb.h>
#include <86box/usb_dev.h>
#include <86box/usb_log.h>
#include <86box/plat_unused.h>

#define MSD_EP_IN  1
#define MSD_EP_OUT 2

#define CBW_SIGNATURE 0x43425355
#define CSW_SIGNATURE 0x53425355

#define CSW_PASSED      0x00
#define CSW_FAILED      0x01
#define CSW_PHASE_ERROR 0x02

#define MSD_RESET       0xff
#define MSD_GET_MAX_LUN 0xfe

enum {
    MSD_CBW = 0,  /* waiting for a CBW */
    MSD_DATA_IN,  /* sending data */
    MSD_DATA_OUT, /* taking the host's data */
    MSD_IN_STALL, /* data ended short of the host's length: Bulk-In STALLs, then the CSW */
    MSD_CSW,      /* the CSW is next on Bulk-In */
    MSD_INVALID   /* after an invalid CBW both pipes stall until a Bulk-Only Mass Storage Reset */
};

static const uint8_t usb_msd_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00, /* class in the interface (BOT 4.1) */
    64,         /* bMaxPacketSize0 */
    0x00, 0x00, /* idVendor */
    0x03, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 3,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define USB_MSD_CFG_LEN (9 + 9 + 7 + 7)

static const uint8_t usb_msd_cfg_desc[USB_MSD_CFG_LEN] = {
    /* Configuration */
    9, 0x02, USB_MSD_CFG_LEN, 0x00, 1, 1, 0,
    0x80, /* bus powered */
    50,   /* 100 mA */
    /* Interface 0: mass storage, SCSI transparent command set, Bulk-Only (Overview 1.4, BOT 4.3) */
    9, 0x04, 0, 0, 2, 0x08, 0x06, 0x50, 0,
    /* Bulk IN EP1 and bulk OUT EP2, 64 bytes (BOT 4.4) */
    7, 0x05, 0x80 | MSD_EP_IN, 0x02, 64, 0x00, 0,
    7, 0x05, MSD_EP_OUT, 0x02, 64, 0x00, 0
};

typedef struct usb_msd_t {
    usb_dev_t      usb;
    uint8_t        bus; /* SCSI bus, 0xff = none left */
    int            state;
    scsi_device_t *sd;       /* target of the command in progress */
    int            open;     /* the target's command still waits for phase 1 */
    uint32_t       tag;
    uint32_t       host_len; /* dCBWDataTransferLength */
    uint32_t       host_pos; /* bytes moved in the data stage */
    uint32_t       dev_len;  /* bytes the target sends or takes in it */
    uint32_t       residue;
    uint8_t        status;   /* bCSWStatus */
    char           serial[16];
    const char    *strings[3];
    int            attached;
} usb_msd_t;

static uint32_t
usb_msd_le32(const uint8_t *p)
{
    return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24);
}

static void
usb_msd_put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
    p[2] = (v >> 16) & 0xff;
    p[3] = v >> 24;
}

/* The drives on the bus are LUN 0..n-1 in SCSI ID order. */
static scsi_device_t *
usb_msd_lun(const usb_msd_t *dev, int lun)
{
    if (dev->bus >= SCSI_BUS_MAX)
        return NULL;

    for (int id = 0; id < SCSI_ID_MAX; id++) {
        scsi_device_t *sd = &scsi_devices[dev->bus][id];

        if (scsi_device_present(sd) && (sd->sc != NULL) && (lun-- == 0))
            return sd;
    }

    return NULL;
}

static int
usb_msd_n_luns(const usb_msd_t *dev)
{
    int n = 0;

    while ((n < 16) && (usb_msd_lun(dev, n) != NULL))
        n++;

    return n;
}

/* The target's command ends: phase 1 finishes a completed data stage,
   anything else is stopped. */
static void
usb_msd_close_command(usb_msd_t *dev, int complete)
{
    if (!dev->open)
        return;

    dev->open = 0;
    if (complete) {
        scsi_device_command_phase1(dev->sd);
        if ((dev->status != CSW_PHASE_ERROR) && (dev->sd->status != SCSI_STATUS_OK))
            dev->status = CSW_FAILED;
    } else
        scsi_device_command_stop(dev->sd);
}

static void
usb_msd_abort(usb_msd_t *dev)
{
    usb_msd_close_command(dev, 0);
    dev->state = MSD_CBW;
}

static uint32_t
usb_msd_buf_len(const usb_msd_t *dev)
{
    if ((dev->sd == NULL) || (dev->sd->sc == NULL) || (dev->sd->sc->temp_buffer == NULL))
        return 0;

    return (uint32_t) dev->sd->sc->temp_buffer_sz;
}

/* Host expectation (direction, dCBWDataTransferLength) against the device's
   intent: the thirteen cases of BOT 6.7. */
static void
usb_msd_plan(usb_msd_t *dev, int host_in, int dev_dir)
{
    uint32_t n;

    dev->host_pos = 0;
    dev->residue  = 0;

    if (dev->host_len == 0) {
        /* (1) Hn = Dn; (2), (3) the device had data: Phase Error. */
        if (dev_dir != 0) {
            usb_msd_close_command(dev, 0);
            dev->status = CSW_PHASE_ERROR;
        }
        dev->state = MSD_CSW;
        return;
    }

    if (host_in) {
        if (dev_dir > 0) {
            /* (5), (6): the relevant data; (7) Hi < Di: the host's length, Phase Error. */
            if (dev->dev_len > dev->host_len) {
                dev->dev_len = dev->host_len;
                dev->status  = CSW_PHASE_ERROR;
            } else
                dev->residue = dev->host_len - dev->dev_len;
            dev->state = MSD_DATA_IN;
            return;
        }
        /* (4) Hi > Dn: nothing to send; (8) Hi <> Do: Phase Error. The Bulk-In
           pipe STALLs where the data would be. */
        if (dev_dir < 0) {
            usb_msd_close_command(dev, 0);
            dev->status = CSW_PHASE_ERROR;
        } else
            dev->residue = dev->host_len;
        dev->state = MSD_IN_STALL;
        return;
    }

    /* The host sends dCBWDataTransferLength bytes, all accepted: (11), (12) the
       target takes the first dev_len; (9) Ho > Dn: none; (10) Ho <> Di and
       (13) Ho < Do: none, Phase Error. */
    n = 0;
    if (dev_dir < 0) {
        if (dev->dev_len <= dev->host_len)
            n = dev->dev_len;
        else {
            usb_msd_close_command(dev, 0);
            dev->status = CSW_PHASE_ERROR;
        }
    } else if (dev_dir > 0) {
        usb_msd_close_command(dev, 0);
        dev->status = CSW_PHASE_ERROR;
    }
    dev->dev_len = n;
    if (dev->status != CSW_PHASE_ERROR)
        dev->residue = dev->host_len - n;
    dev->state = MSD_DATA_OUT;
}

/* Returns 0, or USB_DATA_STALL for an invalid CBW. */
static int
usb_msd_cbw(usb_msd_t *dev, const uint8_t *buf, int len)
{
    uint8_t cdb[16];
    int     lun;
    int     cb_len;
    int     dev_dir = 0;

    /* Valid (6.2.1): 31 bytes starting with the signature. Otherwise both pipes
       STALL until a Reset Recovery (6.6.1). */
    if ((len != 31) || (usb_msd_le32(buf) != CBW_SIGNATURE)) {
        usb_trace(USB_LOG_SCHED, -1, "MSD invalid CBW (%i bytes), both pipes stall until reset", len);
        dev->state                       = MSD_INVALID;
        dev->usb.ep_in[MSD_EP_IN].halt   = 1;
        dev->usb.ep_in[MSD_EP_IN].pending = 0;
        return USB_DATA_STALL;
    }

    dev->tag      = usb_msd_le32(buf + 4);
    dev->host_len = usb_msd_le32(buf + 8);
    dev->status   = CSW_PASSED;
    dev->dev_len  = 0;
    dev->open     = 0;
    lun           = buf[13] & 0x0f;
    cb_len        = buf[14] & 0x1f;

    /* Meaningful (6.2.2): no reserved bits, an existing LUN, a 1-16 byte
       command block. A CBW that is not is failed with no data. */
    dev->sd = NULL;
    if (!(buf[12] & 0x7f) && !(buf[13] & 0xf0) && !(buf[14] & 0xe0) && (cb_len >= 1) && (cb_len <= 16))
        dev->sd = usb_msd_lun(dev, lun);

    if (dev->sd == NULL)
        dev->status = CSW_FAILED;
    else {
        memset(cdb, 0x00, sizeof(cdb));
        memcpy(cdb, buf + 15, cb_len);

        /* A SCSI-2 host also puts the LUN in CDB byte 1 bits 7-5; the bridge
           routes by bCBWLUN and every target is a single-LUN device. */
        if ((lun != 0) && ((cdb[1] >> 5) == lun))
            cdb[1] &= 0x1f;

        /* No host limit: the target's own length is its intent (6.7). */
        dev->sd->buffer_length = -1;
        scsi_device_command_phase0(dev->sd, cdb);
        dev->open = 1;

        if (dev->sd->status != SCSI_STATUS_OK)
            dev->status = CSW_FAILED;
        if ((dev->sd->phase == SCSI_PHASE_DATA_IN) || (dev->sd->phase == SCSI_PHASE_DATA_OUT)) {
            if (dev->sd->buffer_length > 0) {
                dev->dev_len = dev->sd->buffer_length;
                dev_dir      = (dev->sd->phase == SCSI_PHASE_DATA_IN) ? 1 : -1;
                /* Never past what the target's buffer holds. */
                if (dev->dev_len > usb_msd_buf_len(dev))
                    dev->dev_len = usb_msd_buf_len(dev);
            }
        }
        if (dev_dir == 0)
            usb_msd_close_command(dev, 1);
    }

    usb_trace(USB_LOG_SCHED, -1, "MSD CBW tag %08x LUN %i op %02x, host %s %u, device %s %u", dev->tag, lun,
              buf[15], (buf[12] & 0x80) ? "in" : "out", dev->host_len,
              (dev_dir > 0) ? "in" : ((dev_dir < 0) ? "out" : "none"), dev->dev_len);

    usb_msd_plan(dev, !!(buf[12] & 0x80), dev_dir);
    return 0;
}

static int
usb_msd_data_in(usb_dev_t *usb, int ep, uint8_t *buf, int maxlen)
{
    usb_msd_t *dev = (usb_msd_t *) usb->priv;
    uint32_t   n;

    if (ep != MSD_EP_IN)
        return USB_DATA_STALL;

    switch (dev->state) {
        case MSD_DATA_IN:
            n = dev->dev_len - dev->host_pos;
            if (n > (uint32_t) maxlen)
                n = maxlen;
            memcpy(buf, dev->sd->sc->temp_buffer + dev->host_pos, n);
            dev->host_pos += n;
            if (dev->host_pos >= dev->dev_len) {
                usb_msd_close_command(dev, dev->status != CSW_PHASE_ERROR);
                /* Less than the host asked for: a short packet (or none), then STALL (6.7.2). */
                dev->state = (dev->dev_len < dev->host_len) ? MSD_IN_STALL : MSD_CSW;
            }
            return n;

        case MSD_IN_STALL:
            dev->state = MSD_CSW;
            return USB_DATA_STALL;

        case MSD_CSW:
            usb_msd_put_le32(buf, CSW_SIGNATURE);
            usb_msd_put_le32(buf + 4, dev->tag);
            usb_msd_put_le32(buf + 8, dev->residue);
            buf[12]    = dev->status;
            dev->state = MSD_CBW;
            usb_trace(USB_LOG_SCHED, -1, "MSD CSW tag %08x status %i residue %u", dev->tag, dev->status, dev->residue);
            return 13;

        case MSD_INVALID:
            return USB_DATA_STALL;

        default:
            return USB_DATA_NAK;
    }
}

static int
usb_msd_data_out(usb_dev_t *usb, int ep, const uint8_t *buf, int len)
{
    usb_msd_t *dev = (usb_msd_t *) usb->priv;
    uint32_t   n;

    if (ep != MSD_EP_OUT)
        return USB_DATA_STALL;

    switch (dev->state) {
        case MSD_CBW:
            return usb_msd_cbw(dev, buf, len);

        case MSD_DATA_OUT:
            if (dev->host_pos < dev->dev_len) {
                n = dev->dev_len - dev->host_pos;
                if (n > (uint32_t) len)
                    n = len;
                memcpy(dev->sd->sc->temp_buffer + dev->host_pos, buf, n);
            }
            dev->host_pos += len;
            if (dev->host_pos >= dev->host_len) {
                usb_msd_close_command(dev, dev->status != CSW_PHASE_ERROR);
                dev->state = MSD_CSW;
            }
            return 0;

        case MSD_INVALID:
            return USB_DATA_STALL;

        default:
            /* No CBW before the CSW of the last one is read (3.4). */
            return USB_DATA_NAK;
    }
}

static int
usb_msd_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    usb_msd_t *dev     = (usb_msd_t *) usb->priv;
    uint16_t   wvalue  = setup[2] | (setup[3] << 8);
    uint16_t   windex  = setup[4] | (setup[5] << 8);
    uint16_t   wlength = setup[6] | (setup[7] << 8);
    int        n;

    if ((windex != 0) || (wvalue != 0))
        return USB_DATA_STALL;

    /* Bulk-Only Mass Storage Reset (3.1): ready for the next CBW; toggles and
       halts stay as they are. */
    if ((setup[0] == 0x21) && (setup[1] == MSD_RESET) && (wlength == 0)) {
        usb_trace(USB_LOG_SCHED, -1, "MSD Bulk-Only Mass Storage Reset");
        usb_msd_abort(dev);
        return 0;
    }

    /* Get Max LUN (3.2). */
    if ((setup[0] == 0xa1) && (setup[1] == MSD_GET_MAX_LUN) && (wlength == 1)) {
        n       = usb_msd_n_luns(dev);
        data[0] = (n > 0) ? (n - 1) : 0;
        return 1;
    }

    return USB_DATA_STALL;
}

static void
usb_msd_reset(usb_dev_t *usb)
{
    usb_msd_abort((usb_msd_t *) usb->priv);
}

static void
usb_msd_configured(usb_dev_t *usb, UNUSED(int config))
{
    usb_msd_abort((usb_msd_t *) usb->priv);
}

static const usb_dev_ops_t usb_msd_ops = {
    .control    = usb_msd_control,
    .data_in    = usb_msd_data_in,
    .data_out   = usb_msd_data_out,
    .reset      = usb_msd_reset,
    .configured = usb_msd_configured
};

static void *
usb_msd_init(UNUSED(const device_t *info))
{
    usb_msd_t *dev = (usb_msd_t *) calloc(1, sizeof(usb_msd_t));

    /* It takes a SCSI bus like any host adapter. */
    dev->bus = scsi_get_bus();

    /* At least 12 hex digits, unique per product (BOT 4.1.1). */
    snprintf(dev->serial, sizeof(dev->serial), "86B00000%04X", dev->bus);
    dev->strings[0] = "86Box";
    dev->strings[1] = "USB Mass Storage";
    dev->strings[2] = dev->serial;

    dev->usb.name      = "USB Mass Storage";
    dev->usb.low_speed = 0;
    dev->usb.dev_desc  = usb_msd_dev_desc;
    dev->usb.cfg_desc  = usb_msd_cfg_desc;
    dev->usb.cfg_len   = USB_MSD_CFG_LEN;
    dev->usb.strings   = dev->strings;
    dev->usb.n_strings = 3;
    dev->usb.ops       = &usb_msd_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);
    dev->state = MSD_CBW;

    dev->attached = usb_attach(&dev->usb);

    return dev;
}

static void
usb_msd_close(void *priv)
{
    usb_msd_t *dev = (usb_msd_t *) priv;

    if (dev->attached)
        usb_detach(&dev->usb);
    free(dev);
}

const device_t usb_msd_device = {
    .name          = "USB Mass Storage",
    .internal_name = "usb_msd",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = usb_msd_init,
    .close         = usb_msd_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
