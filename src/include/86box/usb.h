/*
 * 86Box    A hypervisor and IBM PC system emulator that specializes in
 *          running old operating systems and software designed for IBM
 *          PC systems and compatibles from 1981 through fairly recent
 *          system designs based on the PCI bus.
 *
 *          This file is part of the 86Box distribution.
 *
 *          Definitions for the Distributed DMA emulation.
 *
 * Authors: Miran Grca, <mgrca8@gmail.com>
 *
 *          Copyright 2020 Miran Grca.
 */

#ifndef USB_H
#define USB_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct uhci_t uhci_t;
typedef struct ohci_t ohci_t;

typedef struct usb_t {
    uhci_t       *uhci;
    ohci_t       *ohci;
    uint16_t      uhci_io_base;
    int           uhci_enable;
    int           ohci_enable;
    uint32_t      ohci_mem_base;
    mem_mapping_t ohci_mmio_mapping;
} usb_t;

/* Global variables. */
extern const device_t usb_device;
/* A USB mass storage bridge to a SCSI bus, listed with the SCSI controllers. */
extern const device_t usb_msd_device;
/* A USB speaker, listed with the sound cards. */
extern const device_t usb_speaker_device;
/* The VIA VT83C572 PCI to USB controller on an add-in card. */
extern const device_t usb_via_vt83c572_device;
/* USB hubs, listed on the Ports page; usb_hub_type 0 is none. */
extern const device_t usb_hub_4port_device;
extern int            usb_hub_type;

extern int             usb_hub_get_ndev(void);
extern const device_t *usb_hub_get_device(int hub);
extern const char     *usb_hub_get_internal_name(int hub);
extern int             usb_hub_get_from_internal_name(const char *s);
extern void            usb_hub_reset(void);
/* PCI USB host controller cards, listed on the Ports page; usb_card_type 0
   is none. */
extern const device_t usb_opti_82c861_device;
extern int            usb_card_type;
/* The settings dialog's card choice before it is saved, -1 outside it. */
extern int            usb_card_pending;

extern int             usb_card_get_ndev(void);
extern const device_t *usb_card_get_device(int card);
extern const char     *usb_card_get_internal_name(int card);
extern int             usb_card_get_from_internal_name(const char *s);
/* Whether the selected card gives machine mch a USB bus. */
extern int             usb_card_has_bus(int mch);
extern void            usb_card_reset(void);

/* Functions. */
extern void uhci_update_io_mapping(usb_t *dev, uint8_t base_l, uint8_t base_h, int enable);
/* The chipset routes the UHCI interrupt; level is the controller's IRQ output. */
extern void uhci_set_irq_handler(usb_t *dev, void (*set_irq)(void *priv, int level), void *priv);
/* PCICMD Bus Master Enable: with it clear the UHCI makes no memory accesses. */
extern void uhci_set_bus_master(usb_t *dev, int enable);
/* A root port whose device is still sending at EOF is disabled, unless the
   chipset keeps it (VIA Rx40 bit 6). */
extern void uhci_set_babble_keep_port(usb_t *dev, int keep);
extern void ohci_update_mem_mapping(usb_t *dev, uint8_t base1, uint8_t base2, uint8_t base3, int enable);
/* The chipset routes the OHCI interrupt; level is the controller's INT output. */
extern void ohci_set_irq_handler(usb_t *dev, void (*set_irq)(void *priv, int level), void *priv);
/* The chipset takes the OHCI's SMI output; level is the SMI pin. */
extern void ohci_set_smi_handler(usb_t *dev, void (*set_smi)(void *priv, int level), void *priv);
/* PCICMD Bus Master Enable: with it clear the OHCI makes no memory accesses. */
extern void ohci_set_bus_master(usb_t *dev, int enable);
/* For a chipset whose Bus Master bit is hardwired to 1: on now and after
   every reset. */
extern void ohci_hardwire_bus_master(usb_t *dev);
/* Where an OHCI differs from the ALi M5237 and SMSC SLC90E66, whose
   behavior is flags 0. Set once at init; kept through every reset. */
#define OHCI_CHIP_LST_0628H 0x01 /* HcLSThreshold resets to 0628h, not 0 */
#define OHCI_CHIP_RWC       0x02 /* RemoteWakeupConnected is R/W, not hard 0 */
#define OHCI_CHIP_NO_SMI    0x04 /* no SMI pin: OwnershipChange is tied to 0 */
#define OHCI_CHIP_POTPGT_RW 0x08 /* POTPGT is a full R/W byte, not bits 25:24 */
#define OHCI_CHIP_RWE_RO    0x10 /* RemoteWakeupEnable is read-only 0 */
#define OHCI_CHIP_UE_RW     0x20 /* the UnrecoverableError enable is R/W */
#define OHCI_CHIP_PTRS_RO   0x40 /* HcPeriodCurrentED, HcDoneHead ignore writes */
extern void ohci_set_chip_flags(usb_t *dev, uint32_t flags);
/* The root hub has a connector for each of its ports (2 by default); the
   first ndp are implemented (NumberDownstreamPorts). A chipset strap may
   change ndp at any time. */
extern void ohci_set_ndp(usb_t *dev, int ndp, int ports);

/* UHCI legacy keyboard support (UHCI 1.1 s5.2) on a function's LEGSUP
   bytes, legsup[0] = C0h and legsup[1] = C1h. */
#define UHCI_LEGSUP_STATUS_ALWAYS  0x01 /* trap status bits set without the enables (VIA Rx41 bit 2) */
#define UHCI_LEGSUP_A20_FF_TRAPPED 0x02 /* the sequence's final 64h <- FFh is not passed (VIA Rx41 bit 1) */
/* A cycle at 60h or 64h; returns 1 if it is trapped and must not reach the
   8042. a20 keeps the pass-through sequence position. */
extern int  uhci_legsup_kbc_cycle(uint8_t *legsup, uint8_t *a20, uint16_t port, int write, uint8_t val, int opts);
/* Whether keyboard controller cycles need to be seen at all. */
extern int  uhci_legsup_kbc_armed(const uint8_t *legsup, int opts);
/* The legacy USB event sources that are active, one bit each (enabled trap
   statuses, bits 3-0; the IRQ with USBSMIEN, bit 4; A20PTS with SMIEPTE,
   bit 7): a source going active is an event (SMI#). irq is the UHCI IRQ. */
extern uint8_t uhci_legsup_event(const uint8_t *legsup, int irq);
extern void uhci_legsup_write_c0(uint8_t *legsup, uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /*USB_H*/
