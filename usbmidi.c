#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

#include <atom.h>
#include <atomsem.h>
#include <atomqueue.h>
#include <atomtimer.h>

#include "hw.h"
#include "usb_dev.h"

static uint8_t idle_stack[256];
static uint8_t master_thread_stack[512];
static ATOM_TCB master_thread_tcb;


static ATOM_QUEUE uart0_rx;
static uint8_t uart0_rx_storage[64];

static ATOM_QUEUE uart3_tx;
static uint8_t uart3_tx_storage[16];

static ATOM_QUEUE uart3_rx;
static uint8_t uart3_rx_storage[16];

static ATOM_QUEUE uart2_tx;
static uint8_t uart2_tx_storage[16];

static ATOM_QUEUE uart2_rx;
static uint8_t uart2_rx_storage[16];

static ATOM_QUEUE uart1_tx;
static uint8_t uart1_tx_storage[64];

static ATOM_QUEUE uart1_rx;
static uint8_t uart1_rx_storage[64];

void _fault(int, int, const char*);
int u_write(int file, uint8_t *ptr, int len);
#define fault(code) _fault(code,__LINE__,__FUNCTION__)
void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    while(1){
    }
};


usbd_device *usb;
/*
 * All references in this file come from Universal Serial Bus Device Class
 * Definition for MIDI Devices, release 1.0.
 */

/* SysEx identity message, preformatted with correct USB framing information */
const uint8_t sysex_identity[] = {
    0x04,	/* USB Framing (3 byte SysEx) */
    0xf0,	/* SysEx start */
    0x7e,	/* non-realtime */
    0x00,	/* Channel 0 */
    0x04,	/* USB Framing (3 byte SysEx) */
    0x7d,	/* Educational/prototype manufacturer ID */
    0x66,	/* Family code (byte 1) */
    0x66,	/* Family code (byte 2) */
    0x04,	/* USB Framing (3 byte SysEx) */
    0x51,	/* Model number (byte 1) */
    0x19,	/* Model number (byte 2) */
    0x00,	/* Version number (byte 1) */
    0x04,	/* USB Framing (3 byte SysEx) */
    0x00,	/* Version number (byte 2) */
    0x01,	/* Version number (byte 3) */
    0x00,	/* Version number (byte 4) */
    0x05,	/* USB Framing (1 byte SysEx) */
    0xf7,	/* SysEx end */
    0,0,	/* Padding */
};


void xcout(unsigned char c);
void split_midi(char *buf, int len, void* arg,void (*callback)(char*,int,void*));

void xcout(unsigned char c){
    static char set[]="0123456789ABCDEF";
    usart_send_blocking(USART1, set[(c>>4)&0x0f]);
    usart_send_blocking(USART1, set[c&0x0f]);
}


void split_midi(char *buf, int len, void* arg,void (*callback)(char*,int,void*)){
    int i=0;
    usart_send_blocking(USART1, 'm');
    xcout(len);

    while(i<len){
        switch (buf[i]&0x0f){
            case 0x00: //Misc event RFU
                callback(buf+i,3,arg);
                i+=3;
                break;
            case 0x01: //Cable event RFU
                callback(buf+i,3,arg);
                i+=3;
                break;
            case 0x02: //SysCom
                callback(buf+i,3,arg);
                i+=3;
                break;
            case 0x03: //SysCom
                callback(buf+i,4,arg);
                i+=4;
                break;
            case 0x04: //sysex
                callback(buf+i,4,arg);
                i+=4;
                break;
            case 0x05: //sysex
                callback(buf+i,2,arg);
                i+=2;
                break;
            case 0x06: //sysex
                callback(buf+i,3,arg);
                i+=3;
                break;
            case 0x07: //sysex
                callback(buf+i,4,arg);
                i+=4;
                break;

            case 0x08: //Note off
            case 0x09: //Note on
            case 0x0a: //Poly Keypress
            case 0x0b: //CC
            case 0x0e: //Pitch
                callback(buf+i,4,arg);
                i+=4;
                break;
            case 0x0c://Prog
            case 0x0d://Pressure
                callback(buf+i,3,arg);
                i+=3;
                break;
            case 0x0f: //Sigle byte ??
                callback(buf+i,2,arg);
                i+=2;
                break;
        }
    }
};

void display_midi(char *buf, int len, void* arg);
void display_midi(char *buf, int len, void* arg){
    (void)arg;

    usart_send_blocking(USART1, 'M');
    xcout(buf[0]);
    int i=1;
    usart_send_blocking(USART1, '_');
    for(;i<len;i++){
        xcout(buf[i]);
//        usart_send_blocking(USART3, buf[i]);
    }
    usart_send_blocking(USART1, ' ');

}

static void usbmidi_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, EP_MIDI_I, buf, 64);

    /* This implementation treats any message from the host as a SysEx
     * identity request. This works well enough providing the host
     * packs the identify request in a single 8 byte USB message.
     */
    uint8_t handled=0;
    if (len>=4) {
        if((buf[0]==0x07 || buf[0]==0x06) && buf[1]==0xf0){ //sysex
            usart_send_blocking(USART1, 'M');
            usart_send_blocking(USART1, 's');
            while (usbd_ep_write_packet(usbd_dev, EP_MIDI_O, sysex_identity,
                        sizeof(sysex_identity)) == 0);
            handled=1;
        }
    }

    if(!handled){
        split_midi(buf, len, 0, display_midi);
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');
    }
    gpio_toggle(GPIOB, GPIO8);
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    uint8_t buf[64];
    int len = usbd_ep_read_packet(usbd_dev, EP_CDC0_R, buf, 64);

    if (len) {
        usbd_ep_write_packet(usbd_dev, EP_CDC0_T, buf, len);
        buf[len] = 0;
    }

    uint8_t x='S';
    atomQueuePut(&uart1_tx,0, &x);
    u_write(1,buf,len);
}

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch(req->bRequest) {
        case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
                                                     /*
                                                      * This Linux cdc_acm driver requires this to be implemented
                                                      * even though it's optional in the CDC spec, and we don't
                                                      * advertise it in the ACM functional descriptor.
                                                      */
                                                     char local_buf[10];
                                                     struct usb_cdc_notification *notif = (void *)local_buf;

                                                     /* We echo signals back to host as notification. */
                                                     notif->bmRequestType = 0xA1;
                                                     notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
                                                     notif->wValue = 0;
                                                     notif->wIndex = 0;
                                                     notif->wLength = 2;
                                                     local_buf[8] = req->wValue & 3;
                                                     local_buf[9] = 0;
                                                     // usbd_ep_write_packet(0x83, buf, 10);
                                                     return 1;
                                                 }
        case USB_CDC_REQ_SET_LINE_CODING: 
                                                 if(*len < sizeof(struct usb_cdc_line_coding))
                                                     return 0; 
                                                 return 1;
    }
    return 0;
}


static void usb_set_config(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;


    usbd_ep_setup(usbd_dev, EP_MIDI_I, USB_ENDPOINT_ATTR_BULK, 64, usbmidi_data_rx_cb);
    usbd_ep_setup(usbd_dev, EP_MIDI_O, USB_ENDPOINT_ATTR_BULK, 64, NULL);

    usbd_ep_setup(usbd_dev, EP_CDC0_R, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, EP_CDC0_T, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, EP_CDC0_I, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
            usbd_dev,
            USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
            USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
            cdcacm_control_request);
}


static void button_send_event(usbd_device *usbd_dev, int pressed)
{
    char buf[4] = { 0x08, /* USB framing: virtual cable 0, note on */
        0x88, /* MIDI command: note on, channel 1 */
        60,   /* Note 60 (middle C) */
        127,   /* "Normal" velocity */
    };
    buf[2] = pressed;

    buf[0] = 0x09;
    buf[1] = 0x90;
    while (usbd_ep_write_packet(usbd_dev, EP_MIDI_O, buf, sizeof(buf)) == 0);

    buf[0] = 0x08;
    buf[1] = 0x80;
    while (usbd_ep_write_packet(usbd_dev, EP_MIDI_O, buf, sizeof(buf)) == 0);

}

static void button_poll(usbd_device *usbd_dev)
{
    static uint32_t button_state = 0;

    /* This is a simple shift based debounce. It's simplistic because
     * although this implements debounce adequately it does not have any
     * noise suppression. It is also very wide (32-bits) because it can
     * be polled in a very tight loop (no debounce timer).
     */
    uint32_t old_button_state = button_state;
    button_state = (button_state << 1) | (GPIOC_IDR & 2);
    if ((0 == button_state) != (0 == old_button_state)) {
        usart_send_blocking(USART1, '.');
        /*
           usart_send_blocking(USART1, 'p');
           usart_send_blocking(USART1, 'r');
           usart_send_blocking(USART1, 'e');
           usart_send_blocking(USART1, 'v');
           usart_send_blocking(USART1, 'e');
           usart_send_blocking(USART1, 'd');
           usart_send_blocking(USART1, '\r');
           usart_send_blocking(USART1, '\n');
           */
        button_send_event(usbd_dev, !!button_state);
    }
}




void usart1_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();

    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART1);

        if(data=='\r' || data=='\n'){
            data='\r';
            u_write(1,(uint8_t*) &data, 1);
            data='\n';
            u_write(1,(uint8_t*) &data, 1);
        }else{
            u_write(1,(uint8_t*) &data, 1);
            button_send_event(usb, data);
        }

    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart1_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send_blocking(USART1, data);
        }else{
            USART_CR1(USART1) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

void usart2_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();
    if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART1) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART2);
        atomQueuePut(&uart2_rx,0, (uint8_t*) &data);
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart2_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send_blocking(USART2, data);
        }else{
            USART_CR1(USART2) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

void usart3_isr(void) {
    static uint8_t data = 'A';
    atomIntEnter();
    if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART3);
        atomQueuePut(&uart3_rx,0, (uint8_t*) &data);
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART3) & USART_CR1_TXEIE) != 0) &&
            ((USART_SR(USART3) & USART_SR_TXE) != 0)) {
        uint8_t status = atomQueueGet(&uart3_tx, 0, &data);
        if(status == ATOM_OK){
            usart_send_blocking(USART3, data);
        }else{
            USART_CR1(USART3) &= ~USART_CR1_TXEIE;
        }
    }
    atomIntExit(0);
}

static void master_thread(uint32_t args __maybe_unused) {
    while(1){
        atomTimerDelay(SYSTEM_TICKS_PER_SEC);
        //usbd_poll(usb);
    }
}

void usb_wakeup_isr(void) {
    atomIntEnter();
    usbd_poll(usb);
    atomIntExit(0);
}

void usb_lp_can_rx0_isr(void) {
    atomIntEnter();
    usbd_poll(usb);
    atomIntExit(0);
}

int main(void) {
        rcc_clock_setup_in_hsi_out_48mhz();
        //rcc_clock_setup_in_hse_8mhz_out_24mhz();

        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_GPIOB);
        rcc_periph_clock_enable(RCC_GPIOC);
        rcc_periph_clock_enable(RCC_AFIO);
        rcc_periph_clock_enable(RCC_USART1);
        rcc_periph_clock_enable(RCC_USART2);
        rcc_periph_clock_enable(RCC_USART3);

        AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

        gpio_set_mode(GPIOA, GPIO_MODE_INPUT, 0, GPIO15);

        usart_setup();

        cm_mask_interrupts(true);
        systick_set_frequency(SYSTEM_TICKS_PER_SEC, 24000000);
        systick_interrupt_enable();
        systick_counter_enable();


        nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
        nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

        //desig_get_unique_id_as_string(usb_serial_number, sizeof(usb_serial_number));
        //usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
        /*
           usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
           usb_strings, 3,
           usbd_control_buffer, sizeof(usbd_control_buffer));
           */


        gpio_set(GPIOA, GPIO15);
        gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);

        gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO8|GPIO9);

        gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO8);
        gpio_clear(GPIOA, GPIO8);

        /* Button pin */

        gpio_set_mode(GPIOC, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO1);
        gpio_set(GPIOC, GPIO1);


        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');
        usart_send_blocking(USART1, 'p');
        usart_send_blocking(USART1, 'r');
        usart_send_blocking(USART1, 'e');
        usart_send_blocking(USART1, 'v');
        usart_send_blocking(USART1, 'e');
        usart_send_blocking(USART1, 'd');
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');
        
        usb=init_usb();
        usbd_register_set_config_callback(usb, usb_set_config);

        nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
        nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);

        gpio_set(GPIOA, GPIO8);

        if(atomOSInit(idle_stack, sizeof(idle_stack), FALSE) != ATOM_OK) 
            fault(1);


        if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), sizeof(uart1_rx_storage)) != ATOM_OK) 
            fault(2);
        if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), sizeof(uart1_tx_storage)) != ATOM_OK) 
            fault(3);
        if (atomQueueCreate (&uart2_rx, uart2_rx_storage, sizeof(uint8_t), sizeof(uart2_rx_storage)) != ATOM_OK) 
            fault(4);
        if (atomQueueCreate (&uart2_tx, uart2_tx_storage, sizeof(uint8_t), sizeof(uart2_tx_storage)) != ATOM_OK) 
            fault(5);
        if (atomQueueCreate (&uart3_rx, uart3_rx_storage, sizeof(uint8_t), sizeof(uart3_rx_storage)) != ATOM_OK) 
            fault(6);
        if (atomQueueCreate (&uart3_tx, uart3_tx_storage, sizeof(uint8_t), sizeof(uart3_tx_storage)) != ATOM_OK) 
            fault(7);

        atomThreadCreate(&master_thread_tcb, 10, master_thread, 0,
                master_thread_stack, sizeof(master_thread_stack), TRUE);

        atomOSStart();
        while (1){
            button_poll(usb);
        }
    }

int u_write(int file, uint8_t *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        switch(file){
            case 1: //RS-232
                atomQueuePut(&uart1_tx,-1, (uint8_t*) &ptr[i]);
                break;
            case 2: //DISPLAY
             //   atomQueuePut(&uart2_tx,0, (uint8_t*) &ptr[i]);
                break;
            case 3: //USB
             //   atomQueuePut(&uart3_tx,0, (uint8_t*) &ptr[i]);
                break;
        }
    }
    switch(file){
        case 1:
            USART_CR1(USART1) |= USART_CR1_TXEIE;
            break;
        case 3:
            USART_CR1(USART3) |= USART_CR1_TXEIE;
            break;
        case 2:
            USART_CR1(USART2) |= USART_CR1_TXEIE;
            break;
    }
    return i;
}



/*
 * GPIO1
 * 1 - GND
 * 2 - C0
 * 3 - C1
 * 4 - C2
 * 5 - C3
 *
 * GPIO2
 * 1 - GND
 * 2 - A0
 * 3 - A1
 * 4 - A2
 * 5 - A3
 *
 * LCD
 * 1 - VCC
 * 2 - A5
 * 3 - A7
 * 4 - A6
 * 5 - A4
 * 6 - GND
 * 7 - cap
 * 8 - C4
 *
 * GPIO3
 * 1 - GND
 * 2 - B12
 * 3 - B13
 * 4 - B14
 * 5 - B15
 *
 * ISP 
 * 1 - A10
 * 2 - GND
 * 3 - A9
 * 4 - BOOT0
 * 5 - NRST
 *
 * SWD
 * 1 - VCC
 * 2 - A13 JTMS
 * 3 - GND
 * 4 - A14 JTCK
 *
 * P6/P7
 * 1 - +5v
 * 2 - GND
 * 3 - Freq12
 * 4 - 
 * 5 - B0/B1
 * 6 - Freq14
 * 7 - B2/B10
 * 8 - D2/B5
 * 9 - B7 SDA
 * 10 - B6 SCL
 *
 * LED1 - C6
 * LED2 - C7
 * LED3 - C8
 * LED4 - C9
 *
 * Not connected
 *  2 - C13
 *  3 - C14
 *  4 - C15
 * 25 - C5
 * 30 - B11
 * 50 - A15 
 * 51 - C10
 * 52 - C11
 * 53 - C12
 * 61 - B8
 * 62 - B9
 * 
 */
