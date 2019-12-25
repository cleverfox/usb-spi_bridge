#ifndef USB_H
#define USB_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/usb/cdc.h>


/*
#define EP_MIDI_I 0x01
#define EP_MIDI_O 0x82
*/

#define EP_CDC0_R 0x01
#define EP_CDC0_T 0x82
#define EP_CDC0_I 0x83

#define IF_ACTL 0
#define IF_MIDI 1
#define IF_COMM0 2
#define IF_CDAT0 3

usbd_device * init_usb(void);


#endif

