/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          USB debug log: per-layer files with byte caps, see usb_log.h.
 *
 * Authors: skiretic, <skiretic@proton.me>
 *
 *          Copyright 2026 skiretic.
 */
#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <wchar.h>
#define HAVE_STDARG_H
#include <86box/86box.h>
#include <86box/timer.h>
#include <86box/path.h>
#include <86box/plat.h>
#include <86box/usb_log.h>

#ifdef ENABLE_USB_LOG
int usb_log_mask = 0;

typedef struct usb_log_file_t {
    FILE    *fp;
    uint64_t bytes;
    char     last[256];
    uint32_t repeats;
    uint64_t last_us;
} usb_log_file_t;

static const char *const layer_names[USB_LOG_LAYERS] = { "reg", "sched", "irq", "pcap" };

static usb_log_file_t log_files[USB_LOG_LAYERS];
static uint64_t       log_cap;
static int            log_refs;

static int
usb_log_parse_mask(const char *s)
{
    int mask = 0;

    if (strstr(s, "all"))
        return (1 << USB_LOG_LAYERS) - 1;

    for (int i = 0; i < USB_LOG_LAYERS; i++) {
        if (strstr(s, layer_names[i]))
            mask |= (1 << i);
    }

    return mask;
}

static void
usb_log_stop(int layer)
{
    if (log_files[layer].fp != NULL)
        fclose(log_files[layer].fp);
    log_files[layer].fp = NULL;
    usb_log_mask &= ~(1 << layer);
}

/* Past the cap a layer writes one last line and stops; the emulator runs on. */
static void
usb_log_write(int layer, const void *data, size_t len)
{
    usb_log_file_t *f = &log_files[layer];

    if ((f->bytes + len) > log_cap) {
        if (layer != USB_LOG_PCAP)
            fprintf(f->fp, "USB log: %s layer reached its %" PRIu64 " byte cap, stopped\n",
                    layer_names[layer], log_cap);
        pclog("USB log: %s layer reached its %" PRIu64 " byte cap, stopped\n", layer_names[layer], log_cap);
        usb_log_stop(layer);
        return;
    }

    fwrite(data, 1, len, f->fp);
    fflush(f->fp);
    f->bytes += len;
}

static void
usb_log_flush_repeats(int layer)
{
    usb_log_file_t *f = &log_files[layer];
    char            line[96];
    int             len;

    if (f->repeats == 0)
        return;

    len = snprintf(line, sizeof(line), "%10" PRIu64 ".%03" PRIu64 "          last line repeated %u more times\n",
                   f->last_us / 1000, f->last_us % 1000, f->repeats);
    f->repeats = 0;
    usb_log_write(layer, line, len);
}

void
usb_log_line(int layer, int frame, const char *fmt, ...)
{
    usb_log_file_t *f  = &log_files[layer];
    /* TIMER_USEC is CPU ticks per microsecond in 32.32 fixed point. */
    uint64_t        us = (uint64_t) ((double) tsc * 4294967296.0 / (double) TIMER_USEC);
    char            body[sizeof(f->last)];
    char            line[sizeof(body) + 32];
    va_list         ap;
    int             len;

    if (f->fp == NULL)
        return;

    va_start(ap, fmt);
    vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);

    /* Identical consecutive lines (polling loops) collapse into a count. */
    if (!strcmp(body, f->last)) {
        f->repeats++;
        f->last_us = us;
        return;
    }
    usb_log_flush_repeats(layer);
    if (f->fp == NULL)
        return;

    if (frame < 0)
        len = snprintf(line, sizeof(line), "%10" PRIu64 ".%03" PRIu64 " fr ---- %s\n", us / 1000, us % 1000, body);
    else
        len = snprintf(line, sizeof(line), "%10" PRIu64 ".%03" PRIu64 " fr %04x %s\n", us / 1000, us % 1000, frame, body);
    if (len >= (int) sizeof(line))
        len = sizeof(line) - 1;

    strcpy(f->last, body);
    f->last_us = us;
    usb_log_write(layer, line, len);
}

/* LINKTYPE_USB_LINUX_MMAPPED record: pcap record header, 64-byte usbmon
   header, then the data, all in host byte order like the global header. */
void
usb_log_pcap(uint64_t id, char type, int xfer, int ep_in, int ep, int bus, int addr,
             int status, int length, const uint8_t *setup, const uint8_t *data, int len)
{
    uint64_t us = (uint64_t) ((double) tsc * 4294967296.0 / (double) TIMER_USEC);
    struct {
        uint32_t ts_sec, ts_usec, incl_len, orig_len;
    } rec;
    struct {
        uint64_t id;
        uint8_t  type, xfer_type, epnum, devnum;
        uint16_t busnum;
        char     flag_setup, flag_data;
        int64_t  ts_sec;
        int32_t  ts_usec, status;
        uint32_t length, len_cap;
        uint8_t  setup[8];
        int32_t  interval, start_frame;
        uint32_t xfer_flags, ndesc;
    } hdr;

    if (!(usb_log_mask & (1 << USB_LOG_PCAP)) || (log_files[USB_LOG_PCAP].fp == NULL))
        return;

    memset(&hdr, 0x00, sizeof(hdr));
    hdr.id         = id;
    hdr.type       = type;
    hdr.xfer_type  = xfer;
    hdr.epnum      = (ep & 0x0f) | (ep_in ? 0x80 : 0x00);
    hdr.devnum     = addr;
    hdr.busnum     = bus;
    if (data == NULL)
        len = 0;
    hdr.flag_setup = (setup != NULL) ? 0 : '-';
    hdr.flag_data  = (data != NULL) ? 0 : (ep_in ? '<' : '>');
    hdr.ts_sec     = us / 1000000;
    hdr.ts_usec    = us % 1000000;
    hdr.status     = status;
    hdr.length     = length;
    hdr.len_cap    = len;
    if (setup != NULL)
        memcpy(hdr.setup, setup, 8);

    rec.ts_sec   = hdr.ts_sec;
    rec.ts_usec  = hdr.ts_usec;
    rec.incl_len = rec.orig_len = sizeof(hdr) + len;

    usb_log_write(USB_LOG_PCAP, &rec, sizeof(rec));
    if (log_files[USB_LOG_PCAP].fp != NULL)
        usb_log_write(USB_LOG_PCAP, &hdr, sizeof(hdr));
    if ((log_files[USB_LOG_PCAP].fp != NULL) && (len > 0))
        usb_log_write(USB_LOG_PCAP, data, len);
}

void
usb_log_open(void)
{
    const char *env;
    char        name[32];
    char        path[1024];

    if (log_refs++ > 0)
        return;

    env          = getenv("USB_LOG");
    usb_log_mask = (env != NULL) ? usb_log_parse_mask(env) : ((1 << USB_LOG_REG) | (1 << USB_LOG_IRQ));
    env          = getenv("USB_LOG_CAP_MB");
    log_cap      = ((env != NULL) ? strtoull(env, NULL, 10) : 64ULL) << 20;

    for (int i = 0; i < USB_LOG_LAYERS; i++) {
        memset(&log_files[i], 0x00, sizeof(usb_log_file_t));
        if (!(usb_log_mask & (1 << i)))
            continue;

        if (i == USB_LOG_PCAP)
            snprintf(name, sizeof(name), "usb.pcap");
        else
            snprintf(name, sizeof(name), "usb-%s.log", layer_names[i]);
        path_append_filename(path, usr_path, name);

        log_files[i].fp = plat_fopen(path, "wb");
        if (log_files[i].fp == NULL) {
            pclog("USB log: cannot open %s, %s layer off\n", path, layer_names[i]);
            usb_log_mask &= ~(1 << i);
            continue;
        }
        pclog("USB log: %s layer -> %s\n", layer_names[i], path);
    }

    if (usb_log_mask & (1 << USB_LOG_PCAP)) {
        /* pcap global header in host byte order: v2.4, snaplen 65535,
           LINKTYPE_USB_LINUX_MMAPPED (220). */
        struct {
            uint32_t magic;
            uint16_t major, minor;
            int32_t  thiszone;
            uint32_t sigfigs, snaplen, network;
        } hdr = { 0xa1b2c3d4, 2, 4, 0, 0, 65535, 220 };

        usb_log_write(USB_LOG_PCAP, &hdr, sizeof(hdr));
    }

    pclog("USB log: mask %02X, cap %" PRIu64 " bytes per layer\n", usb_log_mask, log_cap);
}

void
usb_log_close(void)
{
    if ((log_refs == 0) || (--log_refs > 0))
        return;

    for (int i = 0; i < USB_LOG_LAYERS; i++) {
        if (log_files[i].fp == NULL)
            continue;
        if (i != USB_LOG_PCAP)
            usb_log_flush_repeats(i);
        usb_log_stop(i);
    }
}
#endif
