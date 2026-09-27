/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          UHCI legacy keyboard support, written from the Intel UHCI
 *          Design Guide 1.1 s5.2: the LEGSUP traps on the keyboard
 *          controller ports and the A20GATE pass-through sequence,
 *          shared by the chipsets that carry a UHCI.
 *
 * Authors: skiretic, <skiretic@proton.me>
 *
 *          Copyright 2026 skiretic.
 */
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/mem.h>
#include <86box/device.h>
#include <86box/usb.h>

/* LEGSUP C0h: 60REN, 60WEN, 64REN, 64WEN (bits 3-0), USBSMIEN, A20PTEN,
   PSS, SMIEPTE. C1h: TBY60R, TBY60W, TBY64R, TBY64W (bits 3-0), USBIRQS,
   USBPIRQDEN, reserved, A20PTS. */
#define LEGSUP_USBSMIEN 0x10
#define LEGSUP_A20PTEN  0x20
#define LEGSUP_PSS      0x40
#define LEGSUP_SMIEPTE  0x80
#define LEGSUP_A20PTS   0x80

enum {
    A20_WAIT_DATA = 0, /* after 64h <- D1h */
    A20_WAIT_END       /* after the data byte to 60h */
};

static void
uhci_legsup_a20_end(uint8_t *legsup)
{
    legsup[0] &= ~LEGSUP_PSS;
    legsup[1] |= LEGSUP_A20PTS;
}

int
uhci_legsup_kbc_cycle(uint8_t *legsup, uint8_t *a20, uint16_t port, int write, uint8_t val, int opts)
{
    int bit = ((port == 0x64) ? 2 : 0) | (write ? 1 : 0);

    /* The A20GATE pass-through sequence reaches the 8042 with no trap and no
       status: 64h <- D1h (one or more), 60h <- data, 64h reads (zero or
       more), 64h <- FFh. Any other cycle ends it and is handled normally. */
    if (legsup[0] & LEGSUP_A20PTEN) {
        if (legsup[0] & LEGSUP_PSS) {
            if (write && (port == 0x64) && (val == 0xd1) && (*a20 == A20_WAIT_DATA))
                return 0;
            if (write && (port == 0x60) && (*a20 == A20_WAIT_DATA)) {
                *a20 = A20_WAIT_END;
                return 0;
            }
            if (!write && (port == 0x64) && (*a20 == A20_WAIT_END))
                return 0;
            uhci_legsup_a20_end(legsup);
            if (write && (port == 0x64) && (val == 0xff) && (*a20 == A20_WAIT_END) && !(opts & UHCI_LEGSUP_A20_FF_TRAPPED))
                return 0;
        } else if (write && (port == 0x64) && (val == 0xd1)) {
            legsup[0] |= LEGSUP_PSS;
            *a20 = A20_WAIT_DATA;
            return 0;
        }
    }

    if (legsup[0] & (1 << bit)) {
        legsup[1] |= (1 << bit);
        return 1;
    }

    if (opts & UHCI_LEGSUP_STATUS_ALWAYS)
        legsup[1] |= (1 << bit);

    return 0;
}

int
uhci_legsup_kbc_armed(const uint8_t *legsup, int opts)
{
    return (legsup[0] & (0x0f | LEGSUP_A20PTEN)) || (opts & UHCI_LEGSUP_STATUS_ALWAYS);
}

uint8_t
uhci_legsup_event(const uint8_t *legsup, int irq)
{
    uint8_t src = legsup[1] & legsup[0] & 0x0f;

    if (irq && (legsup[0] & LEGSUP_USBSMIEN))
        src |= 0x10;
    if ((legsup[1] & LEGSUP_A20PTS) && (legsup[0] & LEGSUP_SMIEPTE))
        src |= 0x80;

    return src;
}

void
uhci_legsup_write_c0(uint8_t *legsup, uint8_t val)
{
    /* PSS is read-only; A20PTEN written 0 ends a running sequence at once,
       an end like any other (A20PTS). */
    legsup[0] = (legsup[0] & LEGSUP_PSS) | (val & ~LEGSUP_PSS);
    if (!(val & LEGSUP_A20PTEN) && (legsup[0] & LEGSUP_PSS))
        uhci_legsup_a20_end(legsup);
}
