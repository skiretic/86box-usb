/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the USB debug log.
 *
 *          Four layers, each written to its own file in the VM
 *          directory with its own byte cap:
 *            REG   - guest accesses to the host controller registers;
 *            SCHED - schedule walk (frame list, QH/TD or ED/TD);
 *            IRQ   - status bits and interrupt line changes;
 *            PCAP  - transactions as a usbmon pcap file;
 *            KBC   - the legacy keyboard emulation's control plane: every
 *                    8042 port 60h/64h cycle and every VIA ACPI GLBEN /
 *                    GLBCTL write with CS:EIP and the SMM flag, and each
 *                    SMI the ACPI block raises or holds (off by default).
 *
 *          Built only with ENABLE_USB_LOG; otherwise every call below
 *          compiles to nothing. At run time the USB_LOG environment
 *          variable selects layers ("reg,irq", "all"; default reg,irq)
 *          and USB_LOG_CAP_MB sets the per-layer cap (default 64).
 *
 * Authors: skiretic, <skiretic@proton.me>
 *
 *          Copyright 2026 skiretic.
 */

#ifndef EMU_USB_LOG_H
#define EMU_USB_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    USB_LOG_REG = 0,
    USB_LOG_SCHED,
    USB_LOG_IRQ,
    USB_LOG_PCAP,
    USB_LOG_KBC,
    USB_LOG_LAYERS
};

#ifdef ENABLE_USB_LOG
extern int usb_log_mask;

extern void usb_log_open(void);
extern void usb_log_close(void);
extern void usb_log_line(int layer, int frame, const char *fmt, ...);
/* One usbmon record: type 'S' (submission) or 'C' (completion) of URB id,
   xfer 0 iso, 1 interrupt, 2 control, 3 bulk; setup is 8 bytes or NULL.
   length is the URB's (requested on 'S', actual on 'C'); data is NULL
   when this record carries none. */
extern void usb_log_pcap(uint64_t id, char type, int xfer, int ep_in, int ep, int bus, int addr,
                         int status, int length, const uint8_t *setup, const uint8_t *data, int len);

/* frame < 0 when the controller has no frame number to report. */
#    define usb_trace(layer, frame, ...)                   \
        do {                                               \
            if (usb_log_mask & (1 << (layer)))             \
                usb_log_line((layer), (frame), __VA_ARGS__); \
        } while (0)
#else
#    define usb_log_open()
#    define usb_log_close()
#    define usb_log_pcap(...)
#    define usb_trace(layer, frame, ...) \
        do {                             \
        } while (0)
#endif

#ifdef __cplusplus
}
#endif

#endif /*EMU_USB_LOG_H*/
