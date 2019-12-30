#ifndef USB_H
#define USB_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/usb/cdc.h>

#define EP_CDC0_R 0x01
#define EP_CDC0_T 0x82
#define EP_CDC0_I 0x83

#define EP_CDC1_R 0x04
#define EP_CDC1_T 0x85
#define EP_CDC1_I 0x86

#define IF_COMM0 0
#define IF_CDAT0 1
#define IF_COMM1 2
#define IF_CDAT1 3

usbd_device * init_usb(void);


#endif

