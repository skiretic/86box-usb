# USB host controllers and devices

86Box emulates the USB 1.1 host controllers built into the chipsets of many of its boards, and
a set of USB devices that attach to them. Both host controller types are
covered: the Universal Host Controller Interface (UHCI) used by Intel and VIA southbridges, and
the Open Host Controller Interface (OHCI) used by ALi, SiS, SMSC and STMicroelectronics parts.
Two PCI USB cards bring USB to PCI boards without it.

## Host controllers

`src/usb.c` implements both controllers behind the `usb_t` interface the chipsets use.

The UHCI follows the Intel Universal Host Controller Interface Design Guide, revision 1.1: the
I/O register file, the frame list with its queue heads and transfer descriptors, control, bulk,
interrupt and isochronous transfers, the 1 ms frame with a bit-time budget per packet,
end-of-frame babble handling, interrupts, software debug mode, global and selective suspend
and resume, and two root ports.

The OHCI follows the OpenHCI Open Host Controller Interface Specification, release 1.0a: the
memory-mapped register file, the HCCA, control and bulk lists with the service ratio, the
periodic list, general and isochronous transfer descriptors, the done queue, the root hub with
power switching, the INT and SMI outputs, and the BIOS-to-driver ownership change.

`src/usb_dev.c` is the device side shared by every device: handshakes, data toggles, endpoint
halt, the default control pipe and the standard requests (USB 1.1 chapters 8 and 9).

## Chipsets

| Southbridge | Controller | Root ports |
| --- | --- | --- |
| Intel PIIX3, PIIX4, PIIX4E | UHCI | 2 |
| VIA VT82C586B, VT82C596A/B | UHCI | 2 |
| VIA VT82C686A/B | two UHCIs | 4 |
| ALi M1543 (M5237) | OHCI | 2 |
| ALi M1543C (M5237) | OHCI | 2, or 3 when the BIOS enables the third port |
| SiS 5571, 5582, 5595 | OHCI | 2 |
| SMSC SLC90E66 | OHCI | 2 |
| ST STPC Atlas | OHCI | 2 |

Each chipset's glue routes the controller's interrupt the way that southbridge does, honors its
function-disable bits, and wires its legacy USB SMI where the chip has one (the STPC Atlas has
none). On Intel and VIA southbridges the UHCI
legacy support register traps the keyboard controller ports (60h, 64h) and the A20 gate
sequence, so a BIOS with USB keyboard support can drive a USB keyboard in DOS
(`src/usb_legacy.c`). The OHCI legacy support registers (OpenHCI appendix B) can be read and
written, but they do not trap the keyboard controller ports, so a BIOS cannot run a USB keyboard
in DOS on an OHCI board.

Boards whose southbridge has USB carry `MACHINE_BUS_USB`, except the Siemens Simatic OP47 and the
Microsoft Virtual PC 2007, which have no USB connector; Settings offers USB devices on those
boards, and on any PCI board once a PCI USB card is chosen. Many board BIOSes ship with the USB controller disabled; it has to be enabled in the
BIOS setup before an operating system can see it.

## Devices

| Device | Where it is chosen | Configuration name | Notes |
| --- | --- | --- | --- |
| USB Mouse | Input devices, mouse | `usb_mouse` | Low-speed HID boot mouse, three buttons and a wheel |
| USB Tablet | Input devices, tablet | `usb_tablet` | Full-speed HID absolute pointer, three buttons and a wheel; follows the host pointer without capture |
| USB Keyboard | Input devices, keyboard | `usb_keyboard` | Low-speed HID boot keyboard; the keyboard controller's keyboard port is left empty |
| USB Mass Storage | Ports (COM & LPT), USB, Mass storage | `usb_msd` | Bulk-Only Transport, SCSI transparent command set; it bridges one SCSI bus, taken after every other SCSI bus, and the hard disks, CD-ROM drives and other removable devices put on that bus (channels named "USB Mass Storage") are its logical units, in SCSI ID order; buttons next to the setting open those drive pages |
| USB Speaker | Sound card | `usb_speaker` | USB Audio 1.0: 16-bit stereo PCM at 8, 11.025, 16, 22.05, 32, 44.1 or 48 kHz, volume and mute; resampled to the mixer rate |
| 4-port USB 1.1 hub | Ports (COM & LPT), USB, Hub | `usb_hub_4port` | Full-speed, self-powered, per-port power switching |
| OPTi 82C861 FireLink | Ports (COM & LPT), USB, Card | `opti_82c861` | PCI card, one OHCI with two ports |
| VIA VT83C572 | Ports (COM & LPT), USB, Card | `via_vt83c572` | PCI card, one UHCI with two ports |

The HID devices follow the HID specification 1.11 and the HID Usage Tables 1.12. Mass storage
follows the USB Mass Storage Class Bulk-Only Transport 1.0. The speaker follows the USB Device
Class Definition for Audio Devices 1.0, Audio Data Formats 1.0 and Terminal Types 1.0.

Devices are plugged in when the machine starts. The hub, when one is chosen, is plugged in first.
Every device takes the first free root port, the board's own controller before a PCI USB card,
and goes to a hub port once the root ports are full. A device that finds no port, or a board with no USB controller, is reported in a warning
instead of being dropped silently.

Each device can be unplugged and plugged back in while the machine runs, from its entry in the
Media menu (Connected). An unplugged device has lost power: the guest sees a disconnect, and on
replug a new device to enumerate, on the same port when that port is still free. Unplugging the
hub takes the devices on it off the bus with it. A device left unplugged stays unplugged when the
machine is restarted, and when 86Box is closed and opened again. Each device also has a
status-bar icon, grayed while the device is off the bus; clicking it opens the same menu.

## Not implemented

- USB 2.0 (EHCI) and later controllers.
- OHCI legacy keyboard emulation (OpenHCI appendix B): the registers exist, but the keyboard
  controller ports are not trapped.
- Bus faults that need a physical wire or bus: CRC, bit stuffing and PID errors, FIFO
  underruns, host system errors from a PCI fault, and port over-current.

## Tested guests

Debian GNU/Linux 4.0 was booted with a USB mouse, mass storage and speaker on the boards whose
southbridge has USB. Where USB did not work, the cause was the board's BIOS (USB disabled with
no setup option, a beta BIOS, or the BIOS turning off the second controller); a few boards did
not boot Debian for reasons unrelated to USB. Windows 98 SE
ran USB devices on Intel, VIA, ALi, SiS, SMSC and STPC boards (it has no mass storage class
driver). Windows 2000 ran the mouse, keyboard, speaker and mass storage on an Intel and an ALi
board. DOS ran with the BIOS USB keyboard support on Intel and
VIA boards.
