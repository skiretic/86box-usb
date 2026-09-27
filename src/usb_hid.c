/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          USB HID mouse and keyboard, written from the HID 1.11
 *          specification and the HID Usage Tables 1.12: a low-speed
 *          boot-interface mouse with three buttons and a wheel, and a
 *          low-speed boot-interface keyboard.
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
#include <86box/keyboard.h>
#include <86box/usb_dev.h>
#include <86box/plat_unused.h>

#define HID_GET_REPORT   0x01
#define HID_GET_IDLE     0x02
#define HID_GET_PROTOCOL 0x03
#define HID_SET_REPORT   0x09
#define HID_SET_IDLE     0x0a
#define HID_SET_PROTOCOL 0x0b

typedef struct hid_idle_t {
    uint8_t  rate;    /* 4 ms units, 0 = report only on change */
    uint8_t  next;    /* a SET_IDLE held until the current period's report */
    uint8_t  pending;
    uint64_t last_us; /* the last report on the interrupt pipe */
} hid_idle_t;

static uint64_t
hid_now_us(void)
{
    return (uint64_t) ((double) tsc * 4294967296.0 / (double) TIMER_USEC);
}

/* Inside the last 4 ms of a running period the new value waits for that
   period's report (HID 7.2.4 Latency). */
static void
hid_idle_set(hid_idle_t *idle, uint8_t rate)
{
    if ((idle->rate != 0) && ((hid_now_us() + 4000ULL) > (idle->last_us + (uint64_t) idle->rate * 4000ULL))) {
        idle->next    = rate;
        idle->pending = 1;
    } else {
        idle->rate    = rate;
        idle->pending = 0;
    }
}

static void
hid_idle_reported(hid_idle_t *idle)
{
    idle->last_us = hid_now_us();
    if (idle->pending) {
        idle->rate    = idle->next;
        idle->pending = 0;
    }
}

/* The period ran out with no change: the report is due again (HID 7.2.4). */
static int
hid_idle_due(const hid_idle_t *idle)
{
    return (idle->rate != 0) && ((hid_now_us() - idle->last_us) >= ((uint64_t) idle->rate * 4000ULL));
}

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
    usb_dev_t  usb;
    uint8_t    protocol; /* 0 boot, 1 report */
    hid_idle_t idle;
    uint8_t    buttons;
    int        attached;
} hid_mouse_t;

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

    hid_idle_reported(&dev->idle);

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
                data[0] = dev->idle.rate;
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
                hid_idle_set(&dev->idle, wvalue >> 8);
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

    if (hid_idle_due(&dev->idle))
        return hid_mouse_report(dev, buf);

    return USB_DATA_NAK;
}

static void
hid_mouse_reset(usb_dev_t *usb)
{
    hid_mouse_t *dev = (hid_mouse_t *) usb->priv;

    /* Report protocol after initialization (HID 7.2.6); mice idle indefinitely. */
    dev->protocol     = 1;
    dev->idle.rate    = 0;
    dev->idle.pending = 0;
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

/* Boot keyboard (HID 1.11 B.1). A one-byte FFh extent would read as -1
   (HID 6.2.2.7), so the array's 255 takes two bytes. */
static const uint8_t hid_kbd_report_desc[] = {
    0x05, 0x01,       /* Usage Page (Generic Desktop) */
    0x09, 0x06,       /* Usage (Keyboard) */
    0xa1, 0x01,       /* Collection (Application) */
    0x75, 0x01,       /*   Report Size (1) */
    0x95, 0x08,       /*   Report Count (8) */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x19, 0xe0,       /*   Usage Minimum (224) */
    0x29, 0xe7,       /*   Usage Maximum (231) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x25, 0x01,       /*   Logical Maximum (1) */
    0x81, 0x02,       /*   Input (Data, Variable, Absolute): modifier byte */
    0x95, 0x01,       /*   Report Count (1) */
    0x75, 0x08,       /*   Report Size (8) */
    0x81, 0x01,       /*   Input (Constant): reserved byte */
    0x95, 0x05,       /*   Report Count (5) */
    0x75, 0x01,       /*   Report Size (1) */
    0x05, 0x08,       /*   Usage Page (LEDs) */
    0x19, 0x01,       /*   Usage Minimum (1) */
    0x29, 0x05,       /*   Usage Maximum (5) */
    0x91, 0x02,       /*   Output (Data, Variable, Absolute): LED report */
    0x95, 0x01,       /*   Report Count (1) */
    0x75, 0x03,       /*   Report Size (3) */
    0x91, 0x01,       /*   Output (Constant): padding */
    0x95, 0x06,       /*   Report Count (6) */
    0x75, 0x08,       /*   Report Size (8) */
    0x15, 0x00,       /*   Logical Minimum (0) */
    0x26, 0xff, 0x00, /*   Logical Maximum (255) */
    0x05, 0x07,       /*   Usage Page (Key Codes) */
    0x19, 0x00,       /*   Usage Minimum (0) */
    0x2a, 0xff, 0x00, /*   Usage Maximum (255) */
    0x81, 0x00,       /*   Input (Data, Array) */
    0xc0              /* End Collection */
};

static const uint8_t hid_kbd_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00,
    8,          /* bMaxPacketSize0: low speed */
    0x00, 0x00, /* idVendor */
    0x02, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define HID_KBD_CFG_LEN (9 + 9 + 9 + 7)

static const uint8_t hid_kbd_cfg_desc[HID_KBD_CFG_LEN] = {
    /* Configuration */
    9, 0x02, HID_KBD_CFG_LEN, 0x00, 1, 1, 0,
    0x80, /* bus powered, no remote wakeup */
    50,   /* 100 mA */
    /* Interface 0: HID, boot subclass, keyboard protocol (HID 4.2, 4.3) */
    9, 0x04, 0, 0, 1, 0x03, 0x01, 0x01, 0,
    /* HID descriptor, between interface and endpoint (HID 7.1) */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(hid_kbd_report_desc), 0x00,
    /* Endpoint 1 IN, interrupt, 8 bytes, 10 ms */
    7, 0x05, 0x81, 0x03, 8, 0x00, 10
};

static const char *const hid_kbd_strings[] = {
    "86Box",
    "USB Keyboard"
};

/* 86Box key codes (scan code set 1, E0 xx as 1xx, Pause as 100h) to Keyboard
   page usages, from Microsoft's "USB HID to PS/2 Scan Code Translation
   Table" (2004). 0 = no Keyboard page usage. */
static const uint8_t hid_kbd_usage[0x200] = {
    [0x001] = 0x29, /* Escape */
    [0x002] = 0x1e, /* 1 */
    [0x003] = 0x1f, /* 2 */
    [0x004] = 0x20, /* 3 */
    [0x005] = 0x21, /* 4 */
    [0x006] = 0x22, /* 5 */
    [0x007] = 0x23, /* 6 */
    [0x008] = 0x24, /* 7 */
    [0x009] = 0x25, /* 8 */
    [0x00a] = 0x26, /* 9 */
    [0x00b] = 0x27, /* 0 */
    [0x00c] = 0x2d, /* - */
    [0x00d] = 0x2e, /* = */
    [0x00e] = 0x2a, /* Backspace */
    [0x00f] = 0x2b, /* Tab */
    [0x010] = 0x14, /* Q */
    [0x011] = 0x1a, /* W */
    [0x012] = 0x08, /* E */
    [0x013] = 0x15, /* R */
    [0x014] = 0x17, /* T */
    [0x015] = 0x1c, /* Y */
    [0x016] = 0x18, /* U */
    [0x017] = 0x0c, /* I */
    [0x018] = 0x12, /* O */
    [0x019] = 0x13, /* P */
    [0x01a] = 0x2f, /* [ */
    [0x01b] = 0x30, /* ] */
    [0x01c] = 0x28, /* Return */
    [0x01d] = 0xe0, /* Left Control */
    [0x01e] = 0x04, /* A */
    [0x01f] = 0x16, /* S */
    [0x020] = 0x07, /* D */
    [0x021] = 0x09, /* F */
    [0x022] = 0x0a, /* G */
    [0x023] = 0x0b, /* H */
    [0x024] = 0x0d, /* J */
    [0x025] = 0x0e, /* K */
    [0x026] = 0x0f, /* L */
    [0x027] = 0x33, /* ; */
    [0x028] = 0x34, /* ' */
    [0x029] = 0x35, /* ` */
    [0x02a] = 0xe1, /* Left Shift */
    [0x02b] = 0x31, /* \ (also Europe 1, usage 32h) */
    [0x02c] = 0x1d, /* Z */
    [0x02d] = 0x1b, /* X */
    [0x02e] = 0x06, /* C */
    [0x02f] = 0x19, /* V */
    [0x030] = 0x05, /* B */
    [0x031] = 0x11, /* N */
    [0x032] = 0x10, /* M */
    [0x033] = 0x36, /* , */
    [0x034] = 0x37, /* . */
    [0x035] = 0x38, /* / */
    [0x036] = 0xe5, /* Right Shift */
    [0x037] = 0x55, /* Keypad * */
    [0x038] = 0xe2, /* Left Alt */
    [0x039] = 0x2c, /* Space */
    [0x03a] = 0x39, /* Caps Lock */
    [0x03b] = 0x3a, /* F1 */
    [0x03c] = 0x3b, /* F2 */
    [0x03d] = 0x3c, /* F3 */
    [0x03e] = 0x3d, /* F4 */
    [0x03f] = 0x3e, /* F5 */
    [0x040] = 0x3f, /* F6 */
    [0x041] = 0x40, /* F7 */
    [0x042] = 0x41, /* F8 */
    [0x043] = 0x42, /* F9 */
    [0x044] = 0x43, /* F10 */
    [0x045] = 0x53, /* Num Lock */
    [0x046] = 0x47, /* Scroll Lock */
    [0x047] = 0x5f, /* Keypad 7 */
    [0x048] = 0x60, /* Keypad 8 */
    [0x049] = 0x61, /* Keypad 9 */
    [0x04a] = 0x56, /* Keypad - */
    [0x04b] = 0x5c, /* Keypad 4 */
    [0x04c] = 0x5d, /* Keypad 5 */
    [0x04d] = 0x5e, /* Keypad 6 */
    [0x04e] = 0x57, /* Keypad + */
    [0x04f] = 0x59, /* Keypad 1 */
    [0x050] = 0x5a, /* Keypad 2 */
    [0x051] = 0x5b, /* Keypad 3 */
    [0x052] = 0x62, /* Keypad 0 */
    [0x053] = 0x63, /* Keypad . */
    [0x054] = 0x46, /* SysRq: Alt + Print Screen */
    [0x056] = 0x64, /* Europe 2 */
    [0x057] = 0x44, /* F11 */
    [0x058] = 0x45, /* F12 */
    [0x059] = 0x67, /* Keypad = */
    [0x05c] = 0x8c, /* International 6 */
    [0x064] = 0x68, /* F13 */
    [0x065] = 0x69, /* F14 */
    [0x066] = 0x6a, /* F15 */
    [0x067] = 0x6b, /* F16 */
    [0x068] = 0x6c, /* F17 */
    [0x069] = 0x6d, /* F18 */
    [0x06a] = 0x6e, /* F19 */
    [0x06b] = 0x6f, /* F20 */
    [0x06c] = 0x70, /* F21 */
    [0x06d] = 0x71, /* F22 */
    [0x06e] = 0x72, /* F23 */
    [0x070] = 0x88, /* International 2 (Katakana/Hiragana) */
    [0x073] = 0x87, /* International 1 (Ro) */
    [0x076] = 0x73, /* F24 (also LANG5, usage 94h) */
    [0x077] = 0x93, /* LANG4 (Hiragana) */
    [0x078] = 0x92, /* LANG3 (Katakana) */
    [0x079] = 0x8a, /* International 4 (Henkan) */
    [0x07b] = 0x8b, /* International 5 (Muhenkan) */
    [0x07d] = 0x89, /* International 3 (Yen) */
    [0x07e] = 0x85, /* Keypad , (Brazil) */
    [0x0f1] = 0x91, /* LANG2 (Hanja) */
    [0x0f2] = 0x90, /* LANG1 (Hangul/English) */
    [0x100] = 0x48, /* Pause (E1 1D 45) */
    [0x11c] = 0x58, /* Keypad Enter */
    [0x11d] = 0xe4, /* Right Control */
    [0x135] = 0x54, /* Keypad / */
    [0x137] = 0x46, /* Print Screen */
    [0x138] = 0xe6, /* Right Alt */
    [0x146] = 0x48, /* Break: Ctrl + Pause */
    [0x147] = 0x4a, /* Home */
    [0x148] = 0x52, /* Up */
    [0x149] = 0x4b, /* Page Up */
    [0x14b] = 0x50, /* Left */
    [0x14d] = 0x4f, /* Right */
    [0x14f] = 0x4d, /* End */
    [0x150] = 0x51, /* Down */
    [0x151] = 0x4e, /* Page Down */
    [0x152] = 0x49, /* Insert */
    [0x153] = 0x4c, /* Delete */
    [0x15b] = 0xe3, /* Left GUI */
    [0x15c] = 0xe7, /* Right GUI */
    [0x15d] = 0x65, /* Application */
    [0x15e] = 0x66  /* Power */
};

#define HID_KBD_QUEUE 32

/* A host that polled the interrupt pipe this recently is using the keyboard. */
#define HID_KBD_IN_USE_US 100000ULL

typedef struct hid_kbd_t {
    usb_dev_t  usb;
    uint8_t    protocol; /* 0 boot, 1 report; both send the B.1 report */
    hid_idle_t idle;
    uint8_t    leds;
    uint8_t    held[256];  /* host keys holding each usage */
    uint8_t    order[256]; /* non-modifier usages held, in press order */
    int        n_order;
    uint8_t    sent[8];    /* the last report on the interrupt pipe */
    uint8_t    queue[HID_KBD_QUEUE][8];
    int        q_head;
    int        q_len;
    int        polled;
    uint64_t   polled_us;
    uint8_t    owner[0x200]; /* host key pressed through this keyboard */
    int        attached;
} hid_kbd_t;

/* Modifier byte, reserved byte, six array slots; more keys than slots is
   the phantom state, ErrorRollOver in every slot (HID Appendix C). */
static void
hid_kbd_state(const hid_kbd_t *dev, uint8_t *rep)
{
    memset(rep, 0x00, 8);
    for (int i = 0; i < 8; i++) {
        if (dev->held[0xe0 + i])
            rep[0] |= (1 << i);
    }

    if (dev->n_order > 6)
        memset(rep + 2, 0x01, 6);
    else
        memcpy(rep + 2, dev->order, dev->n_order);
}

/* Every change queues a report, so a press and release between two polls
   both reach the host. A full queue folds the change into its last entry. */
static void
hid_kbd_changed(hid_kbd_t *dev)
{
    uint8_t  rep[8];
    uint8_t *last = (dev->q_len > 0) ? dev->queue[(dev->q_head + dev->q_len - 1) % HID_KBD_QUEUE] : dev->sent;

    hid_kbd_state(dev, rep);
    if (!memcmp(rep, last, 8))
        return;

    if (dev->q_len == HID_KBD_QUEUE)
        memcpy(last, rep, 8);
    else {
        memcpy(dev->queue[(dev->q_head + dev->q_len) % HID_KBD_QUEUE], rep, 8);
        dev->q_len++;
    }
}

static void
hid_kbd_press(hid_kbd_t *dev, uint8_t usage, int down)
{
    if (down) {
        if ((dev->held[usage]++ == 0) && (usage < 0xe0))
            dev->order[dev->n_order++] = usage;
    } else if ((dev->held[usage] != 0) && (--dev->held[usage] == 0) && (usage < 0xe0)) {
        for (int i = 0; i < dev->n_order; i++) {
            if (dev->order[i] == usage) {
                memmove(&dev->order[i], &dev->order[i + 1], dev->n_order - i - 1);
                dev->n_order--;
                break;
            }
        }
    }

    hid_kbd_changed(dev);
}

static int
hid_kbd_in_use(const hid_kbd_t *dev)
{
    return dev->attached && (dev->usb.config != 0) && dev->polled &&
           ((hid_now_us() - dev->polled_us) < HID_KBD_IN_USE_US);
}

/* Host keys come here before the PS/2 keyboard. A key goes to the USB
   keyboard while a host is using it; its release follows the press. */
static int
hid_kbd_key(uint16_t scan, int down, void *priv)
{
    hid_kbd_t *dev   = (hid_kbd_t *) priv;
    uint8_t    usage = hid_kbd_usage[scan & 0x1ff];

    scan &= 0x1ff;

    if (down) {
        if (dev->owner[scan])
            return 1; /* host key repeat: the key is already held */
        if ((usage == 0) || !hid_kbd_in_use(dev))
            return 0;
        dev->owner[scan] = 1;
        hid_kbd_press(dev, usage, 1);
        return 1;
    }

    if (!dev->owner[scan])
        return 0;
    dev->owner[scan] = 0;
    hid_kbd_press(dev, usage, 0);
    return 1;
}

static int
hid_kbd_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    hid_kbd_t *dev     = (hid_kbd_t *) usb->priv;
    uint16_t   wvalue  = setup[2] | (setup[3] << 8);
    uint16_t   windex  = setup[4] | (setup[5] << 8);
    uint16_t   wlength = setup[6] | (setup[7] << 8);

    if (windex != 0)
        return USB_DATA_STALL;

    /* Class descriptors from the interface (HID 7.1.1). */
    if ((setup[0] == 0x81) && (setup[1] == 0x06)) {
        switch (wvalue >> 8) {
            case 0x21:
                memcpy(data, &hid_kbd_cfg_desc[18], 9);
                return 9;
            case 0x22:
                memcpy(data, hid_kbd_report_desc, sizeof(hid_kbd_report_desc));
                return sizeof(hid_kbd_report_desc);
            default:
                return USB_DATA_STALL;
        }
    }

    if (setup[0] == 0xa1) {
        switch (setup[1]) {
            case HID_GET_REPORT:
                /* No report IDs; Input is the current key state (HID 7.2.1). */
                if ((wvalue & 0xff) != 0)
                    return USB_DATA_STALL;
                switch (wvalue >> 8) {
                    case 0x01:
                        hid_kbd_state(dev, data);
                        return 8;
                    case 0x02:
                        data[0] = dev->leds;
                        return 1;
                    default:
                        return USB_DATA_STALL;
                }
            case HID_GET_IDLE:
                data[0] = dev->idle.rate;
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
            case HID_SET_REPORT:
                /* The LED Output report: Num, Caps, Scroll, Compose, Kana (HID B.1). */
                if ((wvalue != 0x0200) || (wlength < 1))
                    return USB_DATA_STALL;
                dev->leds = data[0] & 0x1f;
                keyboard_update_states(!!(dev->leds & 0x02), !!(dev->leds & 0x01), !!(dev->leds & 0x04), !!(dev->leds & 0x10));
                return 0;
            case HID_SET_IDLE:
                hid_idle_set(&dev->idle, wvalue >> 8);
                return 0;
            case HID_SET_PROTOCOL:
                if (wvalue > 1)
                    return USB_DATA_STALL;
                dev->protocol = wvalue;
                return 0;
            default:
                return USB_DATA_STALL;
        }
    }

    return USB_DATA_STALL;
}

/* Interrupt IN: the next queued report, else the current one again once the
   idle period runs out, else NAK (HID 7.2.4). */
static int
hid_kbd_data_in(usb_dev_t *usb, UNUSED(int ep), uint8_t *buf, UNUSED(int maxlen))
{
    hid_kbd_t *dev = (hid_kbd_t *) usb->priv;

    dev->polled    = 1;
    dev->polled_us = hid_now_us();

    if (dev->q_len > 0) {
        memcpy(dev->sent, dev->queue[dev->q_head], 8);
        dev->q_head = (dev->q_head + 1) % HID_KBD_QUEUE;
        dev->q_len--;
    } else if (!hid_idle_due(&dev->idle))
        return USB_DATA_NAK;

    memcpy(buf, dev->sent, 8);
    hid_idle_reported(&dev->idle);
    return 8;
}

static void
hid_kbd_reset(usb_dev_t *usb)
{
    hid_kbd_t *dev = (hid_kbd_t *) usb->priv;

    /* Report protocol after a reset (HID 7.2.6); keyboards idle 500 ms (7.2.4). */
    dev->protocol     = 1;
    dev->idle.rate    = 125;
    dev->idle.pending = 0;
    dev->idle.last_us = hid_now_us();
    dev->leds         = 0x00;
    dev->polled       = 0;

    /* Keys still held show in the first report after enumeration. */
    dev->q_head = 0;
    dev->q_len  = 0;
    memset(dev->sent, 0x00, sizeof(dev->sent));
    hid_kbd_changed(dev);
}

static const usb_dev_ops_t hid_kbd_ops = {
    .control    = hid_kbd_control,
    .data_in    = hid_kbd_data_in,
    .data_out   = NULL,
    .reset      = hid_kbd_reset,
    .configured = NULL
};

static void *
hid_kbd_init(UNUSED(const device_t *info))
{
    hid_kbd_t *dev = (hid_kbd_t *) calloc(1, sizeof(hid_kbd_t));

    dev->usb.name      = "USB Keyboard";
    dev->usb.low_speed = 1;
    dev->usb.dev_desc  = hid_kbd_dev_desc;
    dev->usb.cfg_desc  = hid_kbd_cfg_desc;
    dev->usb.cfg_len   = HID_KBD_CFG_LEN;
    dev->usb.strings   = hid_kbd_strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &hid_kbd_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);
    hid_kbd_reset(&dev->usb);

    dev->attached = usb_attach(&dev->usb);
    if (!dev->attached)
        pclog("USB Keyboard: no free USB port\n");

    keyboard_set_usb_handler(hid_kbd_key, dev);

    return dev;
}

static void
hid_kbd_close(void *priv)
{
    hid_kbd_t *dev = (hid_kbd_t *) priv;

    keyboard_set_usb_handler(NULL, NULL);
    if (dev->attached)
        usb_detach(&dev->usb);
    free(dev);
}

const device_t keyboard_usb_device = {
    .name          = "USB Keyboard",
    .internal_name = "usb_keyboard",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = hid_kbd_init,
    .close         = hid_kbd_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
