/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          USB HID mouse, written from the HID 1.11 specification: a
 *          low-speed boot-interface mouse with three buttons and a
 *          wheel.
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
#include <86box/timer.h>
#include <86box/mouse.h>
#include <86box/usb_dev.h>
#include <86box/plat_unused.h>

#define HID_GET_REPORT   0x01
#define HID_GET_IDLE     0x02
#define HID_GET_PROTOCOL 0x03
#define HID_SET_REPORT   0x09
#define HID_SET_IDLE     0x0a
#define HID_SET_PROTOCOL 0x0b

/* Boot mouse layout (HID 1.11 B.2): buttons, X, Y; a wheel byte follows. */
static const uint8_t hid_mouse_report_desc[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop) */
    0x09, 0x02, /* Usage (Mouse) */
    0xa1, 0x01, /* Collection (Application) */
    0x09, 0x01, /*   Usage (Pointer) */
    0xa1, 0x00, /*   Collection (Physical) */
    0x95, 0x03, /*     Report Count (3) */
    0x75, 0x01, /*     Report Size (1) */
    0x05, 0x09, /*     Usage Page (Button) */
    0x19, 0x01, /*     Usage Minimum (1) */
    0x29, 0x03, /*     Usage Maximum (3) */
    0x15, 0x00, /*     Logical Minimum (0) */
    0x25, 0x01, /*     Logical Maximum (1) */
    0x81, 0x02, /*     Input (Data, Variable, Absolute) */
    0x95, 0x01, /*     Report Count (1) */
    0x75, 0x05, /*     Report Size (5) */
    0x81, 0x01, /*     Input (Constant) */
    0x75, 0x08, /*     Report Size (8) */
    0x95, 0x03, /*     Report Count (3) */
    0x05, 0x01, /*     Usage Page (Generic Desktop) */
    0x09, 0x30, /*     Usage (X) */
    0x09, 0x31, /*     Usage (Y) */
    0x09, 0x38, /*     Usage (Wheel) */
    0x15, 0x81, /*     Logical Minimum (-127) */
    0x25, 0x7f, /*     Logical Maximum (127) */
    0x81, 0x06, /*     Input (Data, Variable, Relative) */
    0xc0,       /*   End Collection */
    0xc0        /* End Collection */
};

static const uint8_t hid_mouse_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00,
    8,          /* bMaxPacketSize0: low speed */
    0x00, 0x00, /* idVendor */
    0x01, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define HID_MOUSE_CFG_LEN (9 + 9 + 9 + 7)

static const uint8_t hid_mouse_cfg_desc[HID_MOUSE_CFG_LEN] = {
    /* Configuration */
    9, 0x02, HID_MOUSE_CFG_LEN, 0x00, 1, 1, 0,
    0x80, /* bus powered, no remote wakeup */
    50,   /* 100 mA */
    /* Interface 0: HID, boot subclass, mouse protocol (HID 4.2, 4.3) */
    9, 0x04, 0, 0, 1, 0x03, 0x01, 0x02, 0,
    /* HID descriptor, between interface and endpoint (HID 7.1) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(hid_mouse_report_desc), 0x00,
    /* Endpoint 1 IN, interrupt, 4 bytes, 10 ms (the low-speed minimum, 5.7.4) */
    7, 0x05, 0x81, 0x03, 4, 0x00, 10
};

static const char *const hid_mouse_strings[] = {
    "86Box",
    "USB Mouse"
};

typedef struct hid_mouse_t {
    usb_dev_t usb;
    uint8_t   protocol;  /* 0 boot, 1 report */
    uint8_t   idle;      /* 4 ms units, 0 = report only on change */
    uint8_t   idle_next; /* a SET_IDLE held until the current period's report */
    uint8_t   idle_pending;
    uint8_t   buttons;
    uint64_t  last_us;
    int       attached;
} hid_mouse_t;

static uint64_t
hid_now_us(void)
{
    return (uint64_t) ((double) tsc * 4294967296.0 / (double) TIMER_USEC);
}

static int
hid_mouse_report(hid_mouse_t *dev, uint8_t *buf)
{
    int dx;
    int dy;
    int dz;
    int ox;
    int oy;

    /* HID Y and the host's Y both grow downward; the wheel grows away from the user. */
    mouse_subtract_coords(&dx, &dy, &ox, &oy, -127, 127, 0, 0);
    mouse_subtract_z(&dz, -127, 127, 0);
    dev->buttons = mouse_get_buttons_ex() & 0x07;

    buf[0] = dev->buttons;
    buf[1] = (uint8_t) dx;
    buf[2] = (uint8_t) dy;
    buf[3] = (uint8_t) dz;

    dev->last_us = hid_now_us();

    if (dev->idle_pending) {
        dev->idle         = dev->idle_next;
        dev->idle_pending = 0;
    }

    /* Boot protocol: the first three bytes only (HID B.2). */
    return dev->protocol ? 4 : 3;
}

static int
hid_mouse_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    hid_mouse_t *dev    = (hid_mouse_t *) usb->priv;
    uint16_t     wvalue = setup[2] | (setup[3] << 8);
    uint16_t     windex = setup[4] | (setup[5] << 8);

    if (windex != 0)
        return USB_DATA_STALL;

    /* Class descriptors from the interface (HID 7.1.1). */
    if ((setup[0] == 0x81) && (setup[1] == 0x06)) {
        switch (wvalue >> 8) {
            case 0x21:
                memcpy(data, &hid_mouse_cfg_desc[18], 9);
                return 9;
            case 0x22:
                memcpy(data, hid_mouse_report_desc, sizeof(hid_mouse_report_desc));
                return sizeof(hid_mouse_report_desc);
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0xa1) {
        switch (setup[1]) {
            case HID_GET_REPORT:
                if ((wvalue >> 8) != 0x01)
                    return USB_DATA_STALL;
                return hid_mouse_report(dev, data);
            case HID_GET_IDLE:
                data[0] = dev->idle;
                return 1;
            case HID_GET_PROTOCOL:
                data[0] = dev->protocol;
                return 1;
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0x21) {
        switch (setup[1]) {
            case HID_SET_IDLE:
                /* Inside the last 4 ms of a running period the new value waits
                   for that period's report (HID 7.2.4 Latency). */
                if ((dev->idle != 0) && ((hid_now_us() + 4000ULL) > (dev->last_us + (uint64_t) dev->idle * 4000ULL))) {
                    dev->idle_next    = wvalue >> 8;
                    dev->idle_pending = 1;
                } else {
                    dev->idle         = wvalue >> 8;
                    dev->idle_pending = 0;
                }
                return 0;
            case HID_SET_PROTOCOL:
                if (wvalue > 1)
                    return USB_DATA_STALL;
                dev->protocol = wvalue;
                return 0;
            default:
                /* No Output or Feature reports: SET_REPORT is not supported. */
                return USB_DATA_STALL;
        }
    }

    return USB_DATA_STALL;
}

/* Interrupt IN: a report on a change, else NAK, unless the idle period ran out (HID 7.2.4). */
static int
hid_mouse_data_in(usb_dev_t *usb, UNUSED(int ep), uint8_t *buf, UNUSED(int maxlen))
{
    hid_mouse_t *dev = (hid_mouse_t *) usb->priv;

    if (mouse_state_changed() || ((mouse_get_buttons_ex() & 0x07) != dev->buttons))
        return hid_mouse_report(dev, buf);

    if ((dev->idle != 0) && ((hid_now_us() - dev->last_us) >= ((uint64_t) dev->idle * 4000ULL)))
        return hid_mouse_report(dev, buf);

    return USB_DATA_NAK;
}

static void
hid_mouse_reset(usb_dev_t *usb)
{
    hid_mouse_t *dev = (hid_mouse_t *) usb->priv;

    /* Report protocol after initialization (HID 7.2.6); mice idle indefinitely. */
    dev->protocol     = 1;
    dev->idle         = 0;
    dev->idle_pending = 0;
}

static const usb_dev_ops_t hid_mouse_ops = {
    .control    = hid_mouse_control,
    .data_in    = hid_mouse_data_in,
    .data_out   = NULL,
    .reset      = hid_mouse_reset,
    .configured = NULL
};

static void *
hid_mouse_init(UNUSED(const device_t *info))
{
    hid_mouse_t *dev = (hid_mouse_t *) calloc(1, sizeof(hid_mouse_t));

    dev->usb.name      = "USB Mouse";
    dev->usb.low_speed = 1;
    dev->usb.dev_desc  = hid_mouse_dev_desc;
    dev->usb.cfg_desc  = hid_mouse_cfg_desc;
    dev->usb.cfg_len   = HID_MOUSE_CFG_LEN;
    dev->usb.strings   = hid_mouse_strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &hid_mouse_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);
    hid_mouse_reset(&dev->usb);

    /* Three buttons plus the wheel. */
    mouse_set_buttons(4);

    dev->attached = usb_attach(&dev->usb);
    if (!dev->attached)
        pclog("USB Mouse: no free USB port\n");

    return dev;
}

static void
hid_mouse_close(void *priv)
{
    hid_mouse_t *dev = (hid_mouse_t *) priv;

    if (dev->attached)
        usb_detach(&dev->usb);
    free(dev);
}

const device_t mouse_usb_device = {
    .name          = "USB Mouse",
    .internal_name = "usb_mouse",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = hid_mouse_init,
    .close         = hid_mouse_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
