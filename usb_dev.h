#ifndef USB_H
#define USB_H

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/audio.h>
#include <libopencm3/usb/midi.h>
#include <libopencm3/usb/cdc.h>


#define EP_MIDI_I 0x02
#define EP_MIDI_O1 0x81
#define EP_MIDI_O2 0x82

#define EP_CDC_R 0x01
#define EP_CDC_T 0x84
#define EP_CDC_I 0x85


usbd_device * init_usb(void);

#endif

