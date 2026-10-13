/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          USB speaker, written from the USB Device Class Definition for
 *          Audio Devices 1.0, Audio Data Formats 1.0 and Terminal Types
 *          1.0: 16-bit stereo PCM on an adaptive isochronous endpoint,
 *          resampled to the mixer rate.
 *
 * Authors: skiretic, <skiretic@proton.me>
 *
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
#include <86box/mem.h>
#include <86box/sound.h>
#include <86box/usb.h>
#include <86box/usb_dev.h>
#include <86box/usb_log.h>
#include <86box/plat_unused.h>

#define AUD_IF_CONTROL 0
#define AUD_IF_STREAM  1
#define AUD_EP_OUT     1

/* Entity IDs: USB streaming -> Feature Unit -> Speaker. */
#define AUD_IT_ID 1
#define AUD_FU_ID 2
#define AUD_OT_ID 3

/* Requests, control selectors (Audio 1.0 A.9, A.10). */
#define AUD_SET_CUR 0x01
#define AUD_GET_CUR 0x81
#define AUD_GET_MIN 0x82
#define AUD_GET_MAX 0x83
#define AUD_GET_RES 0x84

#define AUD_MUTE_CONTROL    0x01
#define AUD_VOLUME_CONTROL  0x02
#define AUD_SAMPLING_FREQ   0x01

/* Volume in 1/256 dB. */
#define AUD_VOL_MIN (-60 * 256)
#define AUD_VOL_MAX 0
#define AUD_VOL_RES 256
#define AUD_VOL_SILENCE (-32768) /* 8000h: -infinity dB */

/* Queue in source frames; playback starts once 30 ms is in, and at most
   200 ms is kept. */
#define AUD_QUEUE   16384
#define AUD_START_MS 30
#define AUD_MAX_MS   200

static const uint32_t aud_rates[] = { 8000, 11025, 16000, 22050, 32000, 44100, 48000 };
#define AUD_N_RATES (sizeof(aud_rates) / sizeof(aud_rates[0]))

/* 49 frames of 4 bytes: 48 kHz plus the one extra an adaptive source may send. */
#define AUD_MPS 196

static const uint8_t aud_dev_desc[18] = {
    18, 0x01,   /* bLength, DEVICE */
    0x10, 0x01, /* bcdUSB 1.10 */
    0x00, 0x00, 0x00, /* class in the interfaces */
    64,         /* bMaxPacketSize0 */
    0x00, 0x00, /* idVendor */
    0x04, 0x00, /* idProduct */
    0x00, 0x01, /* bcdDevice */
    1, 2, 0,    /* iManufacturer, iProduct, iSerialNumber */
    1           /* bNumConfigurations */
};

#define AUD_AC_CS_LEN (9 + 12 + 10 + 9)
#define AUD_TYPE_I_LEN (8 + 3 * AUD_N_RATES)
#define AUD_CFG_LEN (9 + 9 + AUD_AC_CS_LEN + 9 + 9 + 7 + AUD_TYPE_I_LEN + 9 + 7)

#define AUD_RATE(r) ((r) & 0xff), (((r) >> 8) & 0xff), (((r) >> 16) & 0xff)

static const uint8_t aud_cfg_desc[AUD_CFG_LEN] = {
    /* Configuration */
    9, 0x02, AUD_CFG_LEN & 0xff, AUD_CFG_LEN >> 8, 2, 1, 0,
    0x80, /* bus powered */
    50,   /* 100 mA */

    /* AudioControl interface (Audio 1.0 4.3.1) */
    9, 0x04, AUD_IF_CONTROL, 0, 0, 0x01, 0x01, 0x00, 0,
    /* Header: ADC 1.00, one streaming interface (Table 4-2) */
    9, 0x24, 0x01, 0x00, 0x01, AUD_AC_CS_LEN, 0x00, 1, AUD_IF_STREAM,
    /* Input Terminal: USB streaming, 2 channels, left and right (Table 4-3) */
    12, 0x24, 0x02, AUD_IT_ID, 0x01, 0x01, 0, 2, 0x03, 0x00, 0, 0,
    /* Feature Unit: master Mute and Volume, none per channel (Table 4-7) */
    10, 0x24, 0x06, AUD_FU_ID, AUD_IT_ID, 1, 0x03, 0x00, 0x00, 0,
    /* Output Terminal: Speaker (Table 4-4) */
    9, 0x24, 0x03, AUD_OT_ID, 0x01, 0x03, 0, AUD_FU_ID, 0,

    /* AudioStreaming interface, setting 0: zero bandwidth (4.5.1) */
    9, 0x04, AUD_IF_STREAM, 0, 0, 0x01, 0x02, 0x00, 0,
    /* Setting 1: one isochronous endpoint */
    9, 0x04, AUD_IF_STREAM, 1, 1, 0x01, 0x02, 0x00, 0,
    /* AS_GENERAL: from the Input Terminal, one frame of delay, PCM (Table 4-19) */
    7, 0x24, 0x01, AUD_IT_ID, 1, 0x01, 0x00,
    /* Type I: 2 channels, 2-byte subframes, 16 bits, discrete rates (Formats 2.2.5) */
    AUD_TYPE_I_LEN, 0x24, 0x02, 0x01, 2, 2, 16, AUD_N_RATES,
    AUD_RATE(8000), AUD_RATE(11025), AUD_RATE(16000), AUD_RATE(22050),
    AUD_RATE(32000), AUD_RATE(44100), AUD_RATE(48000),
    /* Endpoint 1 OUT, isochronous adaptive, 1 ms (Table 4-20) */
    9, 0x05, AUD_EP_OUT, 0x09, AUD_MPS & 0xff, AUD_MPS >> 8, 1, 0, 0,
    /* EP_GENERAL: Sampling Frequency Control (Table 4-21) */
    7, 0x25, 0x01, 0x01, 0, 0, 0
};

static const char *const aud_strings[] = {
    "86Box",
    "USB Speaker"
};

typedef struct usb_audio_t {
    usb_dev_t usb;
    uint32_t  rate;
    int16_t   volume; /* 1/256 dB */
    uint8_t   mute;
    int       streaming;
    int       playing;
    int16_t   queue[AUD_QUEUE][2];
    int       q_head;
    int       q_len;
    double    pos; /* position between queue[q_head] and the next frame */
    int       attached;
} usb_audio_t;

static void
usb_audio_stop(usb_audio_t *dev)
{
    dev->q_head  = 0;
    dev->q_len   = 0;
    dev->pos     = 0.0;
    dev->playing = 0;
}

static uint32_t
usb_audio_closest_rate(uint32_t hz)
{
    uint32_t best = aud_rates[0];

    for (unsigned i = 1; i < AUD_N_RATES; i++) {
        if (abs((int) aud_rates[i] - (int) hz) < abs((int) best - (int) hz))
            best = aud_rates[i];
    }

    return best;
}

static void
usb_audio_put16(uint8_t *p, int v)
{
    p[0] = v & 0xff;
    p[1] = (v >> 8) & 0xff;
}

/* Feature Unit Mute and Volume on the master channel, first form and the
   0xFF second form, which here addresses that one control (5.2.2.4). */
static int
usb_audio_fu_request(usb_audio_t *dev, const uint8_t *setup, uint8_t *data)
{
    int      selector = setup[3];
    int      channel  = setup[2];
    uint16_t wlength  = setup[6] | (setup[7] << 8);
    int      v;

    if ((channel != 0) && (channel != 0xff))
        return USB_DATA_STALL;

    switch (selector) {
        case AUD_MUTE_CONTROL:
            if (setup[1] == AUD_GET_CUR) {
                data[0] = dev->mute;
                return 1;
            }
            /* One available Mute Control: one byte in either form (Table 5-16). */
            if ((setup[1] == AUD_SET_CUR) && (wlength == 1)) {
                dev->mute = !!data[0];
                return 0;
            }
            return USB_DATA_STALL;

        case AUD_VOLUME_CONTROL:
            switch (setup[1]) {
                case AUD_GET_CUR:
                    usb_audio_put16(data, dev->volume);
                    return 2;
                case AUD_GET_MIN:
                    usb_audio_put16(data, AUD_VOL_MIN);
                    return 2;
                case AUD_GET_MAX:
                    usb_audio_put16(data, AUD_VOL_MAX);
                    return 2;
                case AUD_GET_RES:
                    usb_audio_put16(data, AUD_VOL_RES);
                    return 2;
                case AUD_SET_CUR:
                    /* One available Volume Control: two bytes in either form (Table 5-18). */
                    if (wlength != 2)
                        return USB_DATA_STALL;
                    /* 8000h is silence and stays CUR; anything else is rounded to
                       the closest available setting (5.2.2.4.3.2). */
                    v = (int16_t) (data[0] | (data[1] << 8));
                    if (v == AUD_VOL_SILENCE) {
                        dev->volume = v;
                        return 0;
                    }
                    v = ((v + ((v < 0) ? -(AUD_VOL_RES / 2) : (AUD_VOL_RES / 2))) / AUD_VOL_RES) * AUD_VOL_RES;
                    if (v < AUD_VOL_MIN)
                        v = AUD_VOL_MIN;
                    if (v > AUD_VOL_MAX)
                        v = AUD_VOL_MAX;
                    dev->volume = v;
                    return 0;
                default:
                    return USB_DATA_STALL;
            }

        default:
            return USB_DATA_STALL;
    }
}

static int
usb_audio_control(usb_dev_t *usb, const uint8_t *setup, uint8_t *data)
{
    usb_audio_t *dev     = (usb_audio_t *) usb->priv;
    uint16_t     windex  = setup[4] | (setup[5] << 8);
    uint16_t     wlength = setup[6] | (setup[7] << 8);

    /* bRequest bit 7 is the Get bit and must match bmRequestType's direction
       (Tables 5-1/5-2 p66-67): a Set sent device-to-host would run on stale
       buffer bytes. Anything else is no request this device supports (5.2). */
    if ((setup[0] ^ setup[1]) & 0x80)
        return USB_DATA_STALL;

    /* Class requests to the AudioControl interface: only the Feature Unit
       has Controls (5.2.1: unknown entities STALL). */
    if (((setup[0] == 0x21) || (setup[0] == 0xa1)) && ((windex & 0xff) == AUD_IF_CONTROL)) {
        if ((windex >> 8) == AUD_FU_ID)
            return usb_audio_fu_request(dev, setup, data);
        return USB_DATA_STALL;
    }

    /* Sampling Frequency Control of the streaming endpoint (5.2.3.2.3.1). */
    if (((setup[0] == 0x22) || (setup[0] == 0xa2)) && (windex == AUD_EP_OUT) && (setup[3] == AUD_SAMPLING_FREQ) &&
        (setup[2] == 0)) {
        if (setup[1] == AUD_GET_CUR) {
            data[0] = dev->rate & 0xff;
            data[1] = (dev->rate >> 8) & 0xff;
            data[2] = (dev->rate >> 16) & 0xff;
            return 3;
        }
        if ((setup[1] == AUD_SET_CUR) && (wlength >= 3)) {
            dev->rate = usb_audio_closest_rate(data[0] | (data[1] << 8) | (data[2] << 16));
            dev->pos  = 0.0;
            usb_trace(USB_LOG_SCHED, -1, "Speaker sampling frequency %u Hz", dev->rate);
            return 0;
        }
    }

    return USB_DATA_STALL;
}

/* One isochronous packet of 16-bit stereo frames; the oldest frames give way
   when the queue is full, so latency stays bounded. */
static int
usb_audio_data_out(usb_dev_t *usb, int ep, const uint8_t *buf, int len)
{
    usb_audio_t *dev = (usb_audio_t *) usb->priv;
    int          max = (int) ((dev->rate * AUD_MAX_MS) / 1000);

    if ((ep != AUD_EP_OUT) || !dev->streaming)
        return 0;

    if (max > AUD_QUEUE)
        max = AUD_QUEUE;

    for (int i = 0; (i + 3) < len; i += 4) {
        int tail;

        if (dev->q_len >= max) {
            dev->q_head = (dev->q_head + 1) % AUD_QUEUE;
            dev->q_len--;
        }
        tail                  = (dev->q_head + dev->q_len) % AUD_QUEUE;
        dev->queue[tail][0]   = (int16_t) (buf[i] | (buf[i + 1] << 8));
        dev->queue[tail][1]   = (int16_t) (buf[i + 2] | (buf[i + 3] << 8));
        dev->q_len++;
    }

    if (!dev->playing && (dev->q_len >= (int) ((dev->rate * AUD_START_MS) / 1000)))
        dev->playing = 1;

    return 0;
}

/* The mixer's 48 kHz frames, linearly interpolated from the stream rate. */
static void
usb_audio_get_buffer(int32_t *buffer, uint16_t len, void *priv)
{
    usb_audio_t *dev  = (usb_audio_t *) priv;
    double       step = (double) dev->rate / (double) SOUND_FREQ;
    double       gain = (dev->mute || (dev->volume == AUD_VOL_SILENCE)) ? 0.0 : pow(10.0, (double) dev->volume / (256.0 * 20.0));

    if (!dev->playing)
        return;

    for (int c = 0; c < len; c++) {
        const int16_t *a;
        const int16_t *b;

        /* Two frames are needed to interpolate; running dry means waiting
           for 30 ms again. */
        if (dev->q_len < 2) {
            dev->playing = 0;
            dev->pos     = 0.0;
            break;
        }

        a = dev->queue[dev->q_head];
        b = dev->queue[(dev->q_head + 1) % AUD_QUEUE];
        buffer[c * 2] += (int32_t) lrint(((double) a[0] + ((double) (b[0] - a[0]) * dev->pos)) * gain);
        buffer[c * 2 + 1] += (int32_t) lrint(((double) a[1] + ((double) (b[1] - a[1]) * dev->pos)) * gain);

        dev->pos += step;
        while ((dev->pos >= 1.0) && (dev->q_len > 1)) {
            dev->pos -= 1.0;
            dev->q_head = (dev->q_head + 1) % AUD_QUEUE;
            dev->q_len--;
        }
    }
}

static void
usb_audio_set_interface(usb_dev_t *usb, int intf, int alt)
{
    usb_audio_t *dev = (usb_audio_t *) usb->priv;

    if (intf != AUD_IF_STREAM)
        return;

    /* Setting 0 is zero bandwidth: the stream stops and its queue goes. */
    usb_audio_stop(dev);
    dev->streaming = (alt == 1);
    usb_trace(USB_LOG_SCHED, -1, "Speaker streaming setting %i, %u Hz", alt, dev->rate);
}

static void
usb_audio_reset(usb_dev_t *usb)
{
    usb_audio_t *dev = (usb_audio_t *) usb->priv;

    usb_audio_stop(dev);
    dev->streaming = 0;
}

static void
usb_audio_configured(usb_dev_t *usb, UNUSED(int config))
{
    usb_audio_reset(usb);
}

static const usb_dev_ops_t usb_audio_ops = {
    .control       = usb_audio_control,
    .data_in       = NULL,
    .data_out      = usb_audio_data_out,
    .reset         = usb_audio_reset,
    .configured    = usb_audio_configured,
    .set_interface = usb_audio_set_interface
};

static void *
usb_audio_init(UNUSED(const device_t *info))
{
    usb_audio_t *dev = (usb_audio_t *) calloc(1, sizeof(usb_audio_t));

    dev->rate   = 48000;
    dev->volume = 0;
    dev->mute   = 0;

    dev->usb.name      = "USB Speaker";
    dev->usb.low_speed = 0;
    dev->usb.dev_desc  = aud_dev_desc;
    dev->usb.cfg_desc  = aud_cfg_desc;
    dev->usb.cfg_len   = AUD_CFG_LEN;
    dev->usb.strings   = aud_strings;
    dev->usb.n_strings = 2;
    dev->usb.ops       = &usb_audio_ops;
    dev->usb.priv      = dev;
    usb_dev_init(&dev->usb);

    sound_add_handler(usb_audio_get_buffer, dev);

    dev->attached = usb_attach(&dev->usb);

    return dev;
}

static void
usb_audio_close(void *priv)
{
    usb_audio_t *dev = (usb_audio_t *) priv;

    if (dev->attached)
        usb_detach(&dev->usb);
    free(dev);
}

const device_t usb_speaker_device = {
    .name          = "USB Speaker",
    .internal_name = "usb_speaker",
    .flags         = DEVICE_USB,
    .local         = 0,
    .init          = usb_audio_init,
    .close         = usb_audio_close,
    .reset         = NULL,
    .available     = NULL,
    .speed_changed = NULL,
    .force_redraw  = NULL,
    .config        = NULL
};
