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
#include <string.h>

#include "hw.h"
#include "usb_dev.h"

static uint8_t idle_stack[256];
static uint8_t master_thread_stack[512];
static ATOM_TCB master_thread_tcb;

#warning ok

/*
static ATOM_QUEUE usbmidi_send;
static uint32_t usbmidi_send_storage[96];
*/

/*
static ATOM_QUEUE uart0_rx;
static uint8_t uart0_rx_storage[64];
*/

static ATOM_QUEUE uart3_tx;
static uint8_t uart3_tx_storage[64];

/*
static ATOM_QUEUE uart3_rx;
static uint8_t uart3_rx_storage[16];
*/

static ATOM_QUEUE uart2_tx;
static uint8_t uart2_tx_storage[288];

/*
static ATOM_QUEUE uart2_rx;
static uint8_t uart2_rx_storage[16];
*/

static ATOM_QUEUE uart1_tx;
static uint8_t uart1_tx_storage[1024];

static ATOM_QUEUE uart1_rx;
static uint8_t uart1_rx_storage[64];

static ATOM_QUEUE midi_input;
static uint32_t midi_input_storage[64];

void _fault(int, int, const char*);
int u_write(int file, uint8_t *ptr, int len);
inline int s_write(int file, char *ptr, int len);

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

struct midi_uart {
    union {
        uint8_t u8[4];
        uint32_t u32;
    } recv;
    uint8_t uart_id;
    uint8_t rp;
    uint8_t expected;
    uint8_t sysex;
};
void process_midi_uart(uint8_t data, struct midi_uart *mi);

void xcout(unsigned char c);

void xcout(unsigned char c){
    static char set[]="0123456789ABCDEF";
    char s[2];
    s[0]=set[(c>>4)&0x0f];
    s[1]=set[c&0x0f];
    s_write(1,s,2);
}


static void usbmidi_data_tx_cb(usbd_device *usbd_dev __maybe_unused, uint8_t ep __maybe_unused) {
    s_write(1,"%",1);
    gpio_toggle(GPIOB, GPIO8);
}

static void usbmidi_data_rx_cb(usbd_device *usbd_dev __maybe_unused, uint8_t ep __maybe_unused) {
    //(void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, EP_MIDI_I, buf, 64);

    /* This implementation treats any message from the host as a SysEx
     * identity request. This works well enough providing the host
     * packs the identify request in a single 8 byte USB message.
     */
    char *bp=buf;
    if (len>=4) {
        while(len){
            if((bp[0]==0x07 || bp[0]==0x06) && bp[1]==0xf0){ //sysex
                usart_send_blocking(USART1, 'M');
                usart_send_blocking(USART1, 's');
                while (usbd_ep_write_packet(usbd_dev, EP_MIDI_O, sysex_identity,
                            sizeof(sysex_identity)) == 0);
            }else{
                xcout(bp[0]);
                //uint8_t jack=bp[0] >> 4;
                uint8_t cmd=bp[0]&0x0f;
                uint8_t l=1;
                switch(cmd){
                    case 0x02: //SS
                    case 0x0C: //program ch
                    case 0x0D: //chan pressure
                        s_write(2,bp+1,2);
                        xcout(bp[1]);
                        xcout(bp[2]);
                        s_write(1," ",1);
                        break;
                    case 0x0f: //single byte
                        s_write(2,bp+1,1);
                        xcout(bp[1]);
                        s_write(1," ",1);
                        break;
                    case 0x04: //3+bytes
                    case 0x07: //3bytes
                        l++;
                    case 0x06: //2bytes
                        l++;
                    case 0x05: //1byte
                        {
                            s_write(2,bp+1,l);
                            xcout(bp[1]);
                            if(l>1)
                                xcout(bp[2]);
                            if(l>2)
                                xcout(bp[3]);
                            xcout(l);
                            s_write(1,"_",1);
                            /*
                            //SysEx
                            s_write(1,"\r\nSysEx: ",9);
                            int i=0;
                            for(;i<l;i++){
                                xcout(bp[i]);
                            }
                            s_write(1,"\r\n",2);
                            */
                        }
                        break;
                    default:
                        s_write(2,bp+1,3);
                        xcout(bp[1]);
                        xcout(bp[2]);
                        xcout(bp[3]);
                        xcout(l);
                        s_write(1,"-",1);
                        //split_midi(bp, 4, 0, display_midi);
                }
                /*
                   u_write(2,buf,len);
                   u_write(1,"\r\nC: ",5);
                   int i=0;
                   for(;i<len;i++){
                   xcout(buf[i]);
                   if(i%4==3)
                   u_write(1," ",1);
                   }
                   cont=(len==64);
                   */
            }
            len-=4;
            bp+=4;
        }
        s_write(1,"*\r\n",3);
    }

}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    uint8_t buf[64];
    int len = usbd_ep_read_packet(usbd_dev, EP_CDC0_R, buf, 64);

    if (len) {
        uint8_t sb1[32]={
            0x04, 0xF0, 0x41, 0x00, 
            0x04, 0x00, 0x15, 0x12, 
            0x04, 0x01, 0x00, 0x00,
            0x04, 0x42, 0x41, 0x53,
            0x04, 0x49, 0x43, 0x20, 
            0x04, 0x4D, 0x49, 0x58,
            0x04, 0x20, 0x41, 0x75,
            0x04, 0x42, 0x73, 0x44, 
        };
        uint8_t sb2[32]={
            0x04, 0x61, 0x44, 0x62,
            0x04, 0x20, 0x20, 0x53, 
            0x04, 0x64, 0x33, 0x34, 
            0x04, 0x4D, 0x78, 0x4D, 
            0x04, 0x78, 0x20, 0x20, 
            0x04, 0x00, 0x01, 0x02, 
            0x04, 0x03, 0x04, 0x05, 
            0x04, 0x06, 0x07, 0x08,
        };
        uint16_t sb3[]={ 
            0x04, 0x09, 0x0A, 0x0B, 
            0x04, 0x0C, 0x0D, 0x0E, 
            0x04, 0x0F, 0x10, 0x11, 
            0x04, 0x12, 0x13, 0x00,
            0x04, 0x00, 0x20, 0x01,
            0x04, 0x12, 0x12, 0x04, 
            0x04, 0x04, 0x08, 0x08,
            0x04, 0x01, 0x1E, 0x00,  
        };
        uint16_t sb4[]={
            0x04, 0x15, 0x00, 0x00, 
            0x06, 0x42, 0xF7, 0x00
        };
        usbd_ep_write_packet(usbd_dev, EP_MIDI_O, sb1, sizeof(sb1));
        usbd_ep_write_packet(usbd_dev, EP_MIDI_O, sb2, sizeof(sb2));
        usbd_ep_write_packet(usbd_dev, EP_MIDI_O, sb3, sizeof(sb3));
        usbd_ep_write_packet(usbd_dev, EP_MIDI_O, sb4, sizeof(sb4));
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
    usbd_ep_setup(usbd_dev, EP_MIDI_O, USB_ENDPOINT_ATTR_BULK, 64, usbmidi_data_tx_cb);

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
            //button_send_event(usb, data);
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

inline int midilen(uint8_t i);
inline int midilen(uint8_t i){
    switch(i&0xf0){
        case 0x80: //Note off
            return 3;
        case 0x90: //Note on
            return 3;
        case 0xa0: //Poly aftertouch
            return 3;
        case 0xb0: //CC
            return 3;
        case 0xc0: //Program
            return 2;
        case 0xd0: //Chan aftertouch
            return 2;
        case 0xe0: //Pitch wheel
            return 3;
        case 0xf0:
            switch(i){
                case 0xf0: //sysex start
                    return 0xff;

                case 0xf1: //Quarter frame MTC
                    return 2;
                case 0xf2: //Song pointer
                    return 3;
                case 0xf3: //Song select
                    return 2;
                case 0xf6: //Tune request
                    return 1;

                case 0xf7: //sysex stop
                    return 0;

                case 0xf8: //timing clock
                    return 1;
                case 0xfa: //start
                    return 1;
                case 0xfb: //continue
                    return 1;
                case 0xfc: //stop
                    return 1;
                case 0xfe: //active sense;
                    return 1;
                case 0xff: //reset;
                    return 1;
            }
    };
    return -1;
};



void process_midi_uart(uint8_t data, struct midi_uart *mi){
    /*
       u_write(1,(uint8_t*) ">", 1);
       xcout(data);
       u_write(1,(uint8_t*) "<", 1);
       */
    uint8_t done=0;
    if(mi->rp==0 || ((data&0x80) == 0x80)){
        if(data==0xf0){
            mi->recv.u8[0]=0x04;
            mi->recv.u8[1]=data;
            mi->expected=4;
            mi->rp=2;
            mi->sysex=1;
        }else if(data==0xf7 && mi->sysex){
            if(mi->rp==2){
                mi->recv.u8[0]=0x05;
                mi->expected=2;
            }
            if(mi->rp==3){
                mi->recv.u8[0]=0x06;
                mi->expected=3;
            }
            if(mi->rp==4){
                mi->recv.u8[0]=0x07;
                mi->expected=4;
            }
            done=2;
            mi->sysex=0;
        }else{
            mi->sysex=0;
            mi->expected=midilen(data);
            mi->recv.u8[0]=(data>>4)&0x0f;
            mi->recv.u8[1]=data;
            mi->rp=2;
            if(mi->expected==1){
                mi->recv.u8[2]=0;
                mi->recv.u8[3]=0;
                done=1;
            }else if(mi->expected==2)
                mi->recv.u8[3]=0;
            mi->expected++; //adjust with USB header
        }
    }else if(mi->rp<mi->expected){
        mi->recv.u8[mi->rp]=data;
        mi->rp++;
        if(mi->rp>=mi->expected){
            if(mi->sysex)
                mi->rp=1;
            else
                mi->rp=2;
            done=1;
        }
    }else{
        mi->rp=0;
    }
    if(done){
        /*
        xcout(mi->recv.u8[0]);
        xcout(mi->recv.u8[1]);
        if(mi->expected>2)
            xcout(mi->recv.u8[2]);
        if(mi->expected>3)
            xcout(mi->recv.u8[3]);
        u_write(1,(uint8_t*) "\r\n", 2);
        */
        mi->recv.u8[0]|=(mi->uart_id<<8);
        atomQueuePut(&midi_input,-1, (uint8_t*) &mi->recv.u32);
    }

}

void usart2_isr(void) {
    static struct midi_uart mi={
        .uart_id=2,
        .recv.u32=0,
        .rp=0,
        .expected=0,
        .sysex=0
    };
    atomIntEnter();

    uint8_t data = 'A';
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART2) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART2);
        gpio_toggle(GPIOC, GPIO9);
        process_midi_uart(data, &mi);
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
    static struct midi_uart mi={
        .uart_id=3,
        .recv.u32=0,
        .rp=0,
        .expected=0,
        .sysex=0
    };
    atomIntEnter();

    uint8_t data = 'A';
    if (((USART_CR1(USART3) & USART_CR1_RXNEIE) != 0) &&
            ((USART_SR(USART3) & USART_SR_RXNE) != 0)) {
        data = usart_recv(USART3);
        gpio_toggle(GPIOC, GPIO8);
        process_midi_uart(data, &mi);
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
    uint8_t sendbuf[64];
    while(1){
        uint8_t status = atomQueueGet(&midi_input, 0, (void*)&(sendbuf[0]));
        //atomTimerDelay(SYSTEM_TICKS_PER_SEC);
        atomTimerDelay(1);
        //usbd_ep_write_packet(usb, EP_CDC0_T, buf, 1);
        /*
        xcout(status);
        */
        if(status == ATOM_OK){
            s_write(1,".",1);
            uint8_t sbp=0;
            goto t1;
            while((sbp+=4)<60){
                status = atomQueueGet(&midi_input, -1, (void*)&(sendbuf[sbp]));
                if(status != ATOM_OK){
                    break;
                };
                s_write(1,"+",1);
t1:
                if((sendbuf[sbp]&0x0f)==0x05 ||
                        (sendbuf[sbp]&0x0f)==0x06 ||
                        (sendbuf[sbp]&0x0f)==0x07){
                    s_write(1,"T",1);
                    sendbuf[sbp+4]=(sendbuf[sbp]&0xf0) | 0x0f;
                    sbp+=4;
                    sendbuf[sbp+1]=0xfe;
                    sendbuf[sbp+2]=0;
                    sendbuf[sbp+3]=0;
                }
            }
            xcout(sbp);
            int i=0;
            s_write(1,":",1);
            for(i=0;i<sbp;i++){
                xcout(sendbuf[i]);
            }
            usbd_ep_write_packet(usb, EP_MIDI_O, sendbuf, sbp);
            s_write(1,"\r\n",2);
        }
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

        gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
                GPIO_CNF_OUTPUT_PUSHPULL, GPIO6|GPIO7|GPIO8|GPIO9);
        gpio_set(GPIOC, GPIO6);
        gpio_set(GPIOC, GPIO7);
        gpio_clear(GPIOC, GPIO8);
        gpio_toggle(GPIOC, GPIO9);


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


        if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), 
                    sizeof(uart1_rx_storage)) != ATOM_OK) 
            fault(2);
        if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), 
                    sizeof(uart1_tx_storage)) != ATOM_OK) 
            fault(3);
        /*
        if (atomQueueCreate (&uart2_rx, uart2_rx_storage, sizeof(uint8_t), 
                    sizeof(uart2_rx_storage)) != ATOM_OK) 
            fault(4);
            */
        if (atomQueueCreate (&uart2_tx, uart2_tx_storage, sizeof(uint8_t), 
                    sizeof(uart2_tx_storage)) != ATOM_OK) 
            fault(5);
        /*
        if (atomQueueCreate (&uart3_rx, uart3_rx_storage, sizeof(uint8_t), 
                    sizeof(uart3_rx_storage)) != ATOM_OK) 
            fault(6);
            */
        if (atomQueueCreate (&uart3_tx, uart3_tx_storage, sizeof(uint8_t), 
                    sizeof(uart3_tx_storage)) != ATOM_OK) 
            fault(7);
        if (atomQueueCreate (&midi_input, (uint8_t *)midi_input_storage, 
                    sizeof(uint32_t), 
                    sizeof(midi_input_storage)/sizeof(uint32_t)) != ATOM_OK) 
            fault(8);
        /*
        if (atomQueueCreate (&usbmidi_send, usbmidi_send_storage, sizeof(uint8_t), sizeof(usbmidi_send_storage)) != ATOM_OK) 
            fault(9);
            */

        atomThreadCreate(&master_thread_tcb, 10, master_thread, 0,
                master_thread_stack, sizeof(master_thread_stack), TRUE);

        atomOSStart();
        while (1){
            button_poll(usb);
        }
    }

inline int s_write(int file, char *ptr, int len) {
    return u_write(file, (uint8_t *)ptr,len);
};

int u_write(int file, uint8_t *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        switch(file){
            case 1: //MIDI1/DEBUG
                atomQueuePut(&uart1_tx,-1, (uint8_t*) &ptr[i]);
                break;
            case 2: //MIDI2
                atomQueuePut(&uart2_tx,0, (uint8_t*) &ptr[i]);
                break;
            case 3: //MIDI3
                atomQueuePut(&uart3_tx,0, (uint8_t*) &ptr[i]);
                break;
        }
    }
    switch(file){
        case 1:
            USART_CR1(USART1) |= USART_CR1_TXEIE;
            break;
        case 2:
            USART_CR1(USART2) |= USART_CR1_TXEIE;
            break;
        case 3:
            USART_CR1(USART3) |= USART_CR1_TXEIE;
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
