/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          USB device core, written from USB 1.1 chapters 5, 8 and 9:
 *          handshakes, data toggles, endpoint halt, the default control
 *          pipe and the standard requests. Classes plug in through
 *          usb_dev_ops_t.
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
#include <86box/usb_dev.h>

enum {
    CTL_IDLE = 0,
    CTL_DATA_IN,
    CTL_DATA_OUT,
    CTL_STATUS_IN,
    CTL_STALL
};

#define REQ_GET_STATUS        0x00
#define REQ_CLEAR_FEATURE     0x01
#define REQ_SET_FEATURE       0x03
#define REQ_SET_ADDRESS       0x05
#define REQ_GET_DESCRIPTOR    0x06
#define REQ_GET_CONFIGURATION 0x08
#define REQ_SET_CONFIGURATION 0x09
#define REQ_GET_INTERFACE     0x0a
#define REQ_SET_INTERFACE     0x0b

#define FEAT_ENDPOINT_HALT        0
#define FEAT_DEVICE_REMOTE_WAKEUP 1

static void
usb_dev_reset_endpoints(usb_dev_t *dev, int with_ep0)
{
    for (int i = with_ep0 ? 0 : 1; i < 16; i++) {
        dev->ep_in[i].toggle   = 0;
        dev->ep_in[i].halt     = 0;
        dev->ep_in[i].pending  = 0;
        dev->ep_out[i].toggle  = 0;
        dev->ep_out[i].halt    = 0;
    }
}

void
usb_dev_init(usb_dev_t *dev)
{
    const uint8_t *d   = dev->cfg_desc;
    int            pos = 0;

    memset(dev->ep_in, 0x00, sizeof(dev->ep_in));
    memset(dev->ep_out, 0x00, sizeof(dev->ep_out));

    dev->ep_in[0].present  = dev->ep_out[0].present = 1;
    dev->ep_in[0].mps      = dev->ep_out[0].mps = dev->dev_desc[7];

    /* Endpoint descriptors (type 5) of the one configuration. */
    while ((pos + 1) < dev->cfg_len) {
        if (d[pos] == 0)
            break;
        if ((d[pos + 1] == 0x05) && ((pos + 6) < dev->cfg_len)) {
            usb_ep_t *ep = (d[pos + 2] & 0x80) ? &dev->ep_in[d[pos + 2] & 0x0f] : &dev->ep_out[d[pos + 2] & 0x0f];

            ep->present = 1;
            ep->type    = d[pos + 3] & 0x03;
            ep->mps     = (d[pos + 4] | (d[pos + 5] << 8)) & 0x07ff;
            if (ep->mps > sizeof(ep->pend_buf))
                ep->mps = sizeof(ep->pend_buf);
        }
        pos += d[pos];
    }

    dev->powered_reset = 0;
    dev->addr          = 0;
    dev->set_addr      = 0;
    dev->config        = 0;
    dev->remote_wakeup = 0;
    dev->ctl_stage     = CTL_IDLE;
}

/* After a bus reset the device answers at address 0, unconfigured (9.1.1.3, 9.2.1). */
void
usb_dev_bus_reset(usb_dev_t *dev)
{
    dev->powered_reset = 1;
    dev->addr          = 0;
    dev->set_addr      = 0;
    dev->config        = 0;
    dev->remote_wakeup = 0;
    dev->ctl_stage     = CTL_IDLE;
    usb_dev_reset_endpoints(dev, 1);

    if (dev->ops->reset != NULL)
        dev->ops->reset(dev);
}

static int
usb_dev_interface_exists(const usb_dev_t *dev, int intf)
{
    return dev->config && (intf < dev->cfg_desc[4]);
}

static usb_ep_t *
usb_dev_endpoint(usb_dev_t *dev, int windex)
{
    int n = windex & 0x0f;

    if ((n != 0) && !dev->config)
        return NULL;

    return (windex & 0x80) ? &dev->ep_in[n] : &dev->ep_out[n];
}

static int
usb_dev_string(const usb_dev_t *dev, int index, uint8_t *buf)
{
    const char *s;
    int         len;

    if (index == 0) {
        /* LANGID list: US English. */
        buf[0] = 4;
        buf[1] = 0x03;
        buf[2] = 0x09;
        buf[3] = 0x04;
        return 4;
    }

    if (index > dev->n_strings)
        return USB_DATA_STALL;

    s   = dev->strings[index - 1];
    len = (int) strlen(s);
    if (len > 126)
        len = 126;

    buf[0] = 2 + (len * 2);
    buf[1] = 0x03;
    for (int i = 0; i < len; i++) {
        buf[2 + (i * 2)] = (uint8_t) s[i];
        buf[3 + (i * 2)] = 0x00;
    }

    return buf[0];
}

/* Standard requests (9.4). Returns the Data-IN length, 0, or USB_DATA_STALL. */
static int
usb_dev_standard(usb_dev_t *dev, const uint8_t *setup, uint8_t *buf)
{
    uint8_t   rt     = setup[0];
    uint16_t  wvalue = setup[2] | (setup[3] << 8);
    uint16_t  windex = setup[4] | (setup[5] << 8);
    usb_ep_t *ep;

    switch (setup[1]) {
        case REQ_GET_STATUS:
            buf[1] = 0x00;
            switch (rt) {
                case 0x80:
                    buf[0] = ((dev->cfg_desc[7] & 0x40) ? 0x01 : 0x00) | (dev->remote_wakeup << 1);
                    return 2;
                case 0x81:
                    if (!usb_dev_interface_exists(dev, windex))
                        return USB_DATA_STALL;
                    buf[0] = 0x00;
                    return 2;
                case 0x82:
                    ep = usb_dev_endpoint(dev, windex);
                    if ((ep == NULL) || !ep->present)
                        return USB_DATA_STALL;
                    buf[0] = ep->halt;
                    return 2;
                default:
                    return USB_DATA_STALL;
            }

        case REQ_CLEAR_FEATURE:
        case REQ_SET_FEATURE:
            if ((rt == 0x00) && (wvalue == FEAT_DEVICE_REMOTE_WAKEUP)) {
                if (!(dev->cfg_desc[7] & 0x20))
                    return USB_DATA_STALL;
                dev->remote_wakeup = (setup[1] == REQ_SET_FEATURE);
                return 0;
            }
            if ((rt == 0x02) && (wvalue == FEAT_ENDPOINT_HALT)) {
                ep = usb_dev_endpoint(dev, windex);
                if ((ep == NULL) || !ep->present)
                    return USB_DATA_STALL;
                ep->halt = (setup[1] == REQ_SET_FEATURE);
                /* Clearing halt restarts a data toggle at DATA0 (9.4.5). The
                   control pipe's toggles belong to SETUP: this request's own
                   Status stage is still DATA1 (8.5.2). */
                if (!ep->halt && ((windex & 0x0f) != 0)) {
                    ep->toggle  = 0;
                    ep->pending = 0;
                }
                return 0;
            }
            return USB_DATA_STALL;

        case REQ_SET_ADDRESS:
            if ((rt != 0x00) || (wvalue > 127))
                return USB_DATA_STALL;
            /* Takes effect after the Status stage (9.4.6). */
            dev->new_addr = wvalue;
            dev->set_addr = 1;
            return 0;

        case REQ_GET_DESCRIPTOR:
            if (rt != 0x80)
                break;
            switch (wvalue >> 8) {
                case 0x01:
                    memcpy(buf, dev->dev_desc, 18);
                    return 18;
                case 0x02:
                    if ((wvalue & 0xff) != 0)
                        return USB_DATA_STALL;
                    memcpy(buf, dev->cfg_desc, dev->cfg_len);
                    return dev->cfg_len;
                case 0x03:
                    return usb_dev_string(dev, wvalue & 0xff, buf);
                default:
                    return USB_DATA_STALL;
            }

        case REQ_GET_CONFIGURATION:
            if (rt != 0x80)
                return USB_DATA_STALL;
            buf[0] = dev->config;
            return 1;

        case REQ_SET_CONFIGURATION:
            if ((rt != 0x00) || (dev->addr == 0))
                return USB_DATA_STALL;
            if ((wvalue & 0xff) == 0)
                dev->config = 0;
            else if ((wvalue & 0xff) == dev->cfg_desc[5])
                dev->config = wvalue & 0xff;
            else
                return USB_DATA_STALL;
            /* Configuring resets every endpoint's toggle and halt (9.1.1.5, 9.4.5). */
            usb_dev_reset_endpoints(dev, 0);
            if (dev->ops->configured != NULL)
                dev->ops->configured(dev, dev->config);
            return 0;

        case REQ_GET_INTERFACE:
            if ((rt != 0x81) || !usb_dev_interface_exists(dev, windex))
                return USB_DATA_STALL;
            buf[0] = 0x00;
            return 1;

        case REQ_SET_INTERFACE:
            if ((rt != 0x01) || !usb_dev_interface_exists(dev, windex) || (wvalue != 0))
                return USB_DATA_STALL;
            usb_dev_reset_endpoints(dev, 0);
            return 0;

        default:
            break;
    }

    /* Anything else, e.g. class descriptors requested from an interface. */
    if ((rt & 0x1f) == 0x01)
        return dev->ops->control(dev, setup, buf);

    return USB_DATA_STALL;
}

static int
usb_dev_request(usb_dev_t *dev, uint8_t *buf)
{
    if ((dev->ctl_setup[0] & 0x60) == 0x00)
        return usb_dev_standard(dev, dev->ctl_setup, buf);

    return dev->ops->control(dev, dev->ctl_setup, buf);
}

static void
usb_dev_setup(usb_dev_t *dev)
{
    uint16_t wlength = dev->ctl_setup[6] | (dev->ctl_setup[7] << 8);
    int      n;

    dev->ctl_pos  = 0;
    dev->ctl_len  = 0;
    dev->ctl_zlp  = 0;
    dev->set_addr = 0;

    /* wLength 0 means no Data stage whatever the direction bit says (9.3.1). */
    if ((dev->ctl_setup[0] & 0x80) && (wlength != 0)) {
        n = usb_dev_request(dev, dev->ctl_buf);
        if (n < 0) {
            dev->ctl_stage = CTL_STALL;
            return;
        }
        /* Never more than wLength; a short answer that fills whole
           packets ends with a zero-length packet (9.3.5, 8.5.2.2). */
        dev->ctl_len   = (n < wlength) ? n : wlength;
        dev->ctl_zlp   = (dev->ctl_len < wlength) && ((dev->ctl_len % dev->ep_in[0].mps) == 0);
        dev->ctl_stage = CTL_DATA_IN;
    } else if (wlength == 0) {
        n              = usb_dev_request(dev, dev->ctl_buf);
        dev->ctl_stage = (n < 0) ? CTL_STALL : CTL_STATUS_IN;
    } else if (wlength > USB_CTRL_BUF) {
        dev->ctl_stage = CTL_STALL;
    } else {
        dev->ctl_len   = wlength;
        dev->ctl_stage = CTL_DATA_OUT;
    }
}

/* Next EP0 IN packet, without consuming it; usb_dev_ctl_in_acked() does. */
static int
usb_dev_ctl_in(usb_dev_t *dev, uint8_t *buf, int mps)
{
    int n;

    switch (dev->ctl_stage) {
        case CTL_DATA_IN:
            n = dev->ctl_len - dev->ctl_pos;
            if (n > 0) {
                if (n > mps)
                    n = mps;
                memcpy(buf, dev->ctl_buf + dev->ctl_pos, n);
                return n;
            }
            /* More IN than the Setup stage asked for: STALL (8.5.2.1). */
            return dev->ctl_zlp ? 0 : USB_DATA_STALL;

        case CTL_DATA_OUT:
            /* Status stage of a control write: the request runs now. */
            if (dev->ctl_pos != dev->ctl_len) {
                dev->ctl_stage = CTL_STALL;
                return USB_DATA_STALL;
            }
            if (usb_dev_request(dev, dev->ctl_buf) < 0) {
                dev->ctl_stage = CTL_STALL;
                return USB_DATA_STALL;
            }
            dev->ctl_stage = CTL_STATUS_IN;
            return 0;

        case CTL_STATUS_IN:
            return 0;

        default:
            return USB_DATA_STALL;
    }
}

static void
usb_dev_ctl_in_acked(usb_dev_t *dev, int len)
{
    if (dev->ctl_stage == CTL_DATA_IN) {
        if (len > 0)
            dev->ctl_pos += len;
        else
            dev->ctl_zlp = 0;
    } else if (dev->ctl_stage == CTL_STATUS_IN) {
        if (dev->set_addr)
            dev->addr = dev->new_addr;
        dev->set_addr  = 0;
        dev->ctl_stage = CTL_IDLE;
    }
}

static int
usb_dev_ctl_out(usb_dev_t *dev, const usb_packet_t *p)
{
    usb_ep_t *ep = &dev->ep_out[0];

    switch (dev->ctl_stage) {
        case CTL_DATA_OUT:
            if (p->toggle != ep->toggle)
                return USB_RET_ACK; /* retransmission: ACK and discard (8.6.4) */
            if ((dev->ctl_pos + p->len) > dev->ctl_len) {
                dev->ctl_stage = CTL_STALL;
                return USB_RET_STALL;
            }
            memcpy(dev->ctl_buf + dev->ctl_pos, p->data, p->len);
            dev->ctl_pos += p->len;
            ep->toggle ^= 1;
            return USB_RET_ACK;

        case CTL_DATA_IN:
            /* Status stage of a control read; any length is accepted (8.5.2). */
            if (p->toggle != ep->toggle)
                return USB_RET_ACK;
            ep->toggle ^= 1;
            dev->ctl_stage = CTL_IDLE;
            return USB_RET_ACK;

        default:
            dev->ctl_stage = CTL_STALL;
            return USB_RET_STALL;
    }
}

int
usb_dev_packet(usb_dev_t *dev, usb_packet_t *p)
{
    int       n  = p->ep & 0x0f;
    usb_ep_t *ep;
    int       ret;

    /* Silent until the first bus reset (9.1.1.3). */
    if (!dev->powered_reset)
        return USB_RET_NO_RESPONSE;

    switch (p->pid) {
        case USB_PID_SETUP:
            /* SETUP is always ACKed and only the control endpoint takes it (8.4.5.4). */
            if (n != 0)
                return USB_RET_NO_RESPONSE;
            dev->ep_in[0].pending = 0;
            dev->ep_in[0].halt    = 0;
            dev->ep_out[0].halt   = 0;
            if (p->len != 8) {
                dev->ctl_stage = CTL_STALL;
                return USB_RET_ACK;
            }
            memcpy(dev->ctl_setup, p->data, 8);
            /* SETUP puts both sequence bits at 1 for what follows (8.6.1). */
            dev->ep_in[0].toggle  = 1;
            dev->ep_out[0].toggle = 1;
            usb_dev_setup(dev);
            return USB_RET_ACK;

        case USB_PID_OUT:
            ep = &dev->ep_out[n];
            if (!ep->present || ((n != 0) && !dev->config))
                return USB_RET_NO_RESPONSE;
            if (ep->halt)
                return USB_RET_STALL;
            if (n == 0)
                return usb_dev_ctl_out(dev, p);
            if (p->toggle != ep->toggle)
                return USB_RET_ACK;
            ret = (dev->ops->data_out != NULL) ? dev->ops->data_out(dev, n, p->data, p->len) : USB_DATA_STALL;
            if (ret == USB_DATA_NAK)
                return USB_RET_NAK;
            if (ret == USB_DATA_STALL) {
                ep->halt = 1;
                return USB_RET_STALL;
            }
            ep->toggle ^= 1;
            return USB_RET_ACK;

        case USB_PID_IN:
            ep = &dev->ep_in[n];
            if (!ep->present || ((n != 0) && !dev->config))
                return USB_RET_NO_RESPONSE;
            if (ep->halt)
                return USB_RET_STALL;

            /* An unacknowledged packet is resent unchanged (8.6.4). */
            if (!ep->pending) {
                if (n == 0)
                    ret = usb_dev_ctl_in(dev, ep->pend_buf, ep->mps);
                else if (dev->ops->data_in != NULL)
                    ret = dev->ops->data_in(dev, n, ep->pend_buf, ep->mps);
                else
                    ret = USB_DATA_NAK;

                if (ret == USB_DATA_NAK)
                    return USB_RET_NAK;
                if (ret == USB_DATA_STALL) {
                    /* A control STALL holds until the next SETUP (8.5.2). */
                    if (n != 0)
                        ep->halt = 1;
                    else
                        dev->ctl_stage = CTL_STALL;
                    return USB_RET_STALL;
                }
                ep->pend_len = ret;
                ep->pending  = 1;
            }

            if (ep->pend_len > p->len) {
                memcpy(p->data, ep->pend_buf, p->len);
                p->actual = p->len;
                return USB_RET_BABBLE;
            }

            memcpy(p->data, ep->pend_buf, ep->pend_len);
            p->actual = ep->pend_len;

            /* The host ACKs both a good packet and a toggle mismatch (8.6.4). */
            ret         = (ep->toggle == p->toggle) ? USB_RET_ACK : USB_RET_TOGGLE_MISMATCH;
            ep->pending = 0;
            ep->toggle ^= 1;
            if (n == 0)
                usb_dev_ctl_in_acked(dev, ep->pend_len);
            return ret;

        default:
            return USB_RET_NO_RESPONSE;
    }
}
