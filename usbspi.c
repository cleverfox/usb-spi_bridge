#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/spi.h>
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

struct spi_transfer_storage {
  uint8_t buf[34];
  uint8_t len;
};

static ATOM_QUEUE spi_in;
#define SPI_ITEMS 8
static struct spi_transfer_storage spi_in_storage[SPI_ITEMS];

static ATOM_QUEUE uart1_tx;
static uint8_t uart1_tx_storage[1024];

static ATOM_QUEUE uart1_rx;
static uint8_t uart1_rx_storage[64];

void _fault(int, int, const char*);
int u_write(int file, uint8_t *ptr, int len);
int s_write(int file, char *ptr, int len);

#define fault(code) _fault(code,__LINE__,__FUNCTION__)
void _fault(__unused int code, __unused int line, __unused const char* function){
    cm_mask_interrupts(true);
    while(1){
    }
};

usbd_device *usb;
struct usb_cdc_line_coding linecoding;
uint16_t selected_nss=GPIO12;

void xcout(unsigned char c);

void xcout(unsigned char c){
    static char set[]="0123456789ABCDEF";
    char s[2];
    s[0]=set[(c>>4)&0x0f];
    s[1]=set[c&0x0f];
    s_write(1,s,2);
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
    (void)ep;
    struct spi_transfer_storage sts;
    sts.len = usbd_ep_read_packet(usbd_dev, EP_CDC0_R, sts.buf, sizeof(sts.buf));
    u_write(1,(uint8_t*)"S",1);
    if (sts.len) {
      if(atomQueuePut(&spi_in,-1, (uint8_t*) &sts)!=ATOM_OK){
        usbd_ep_write_packet(usbd_dev, EP_CDC0_T, "", 0);
      }
    }
}

static int cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
        uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
  (void)complete;
  (void)buf;
  (void)usbd_dev;

  switch(req->bRequest) {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
      {
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
        usbd_ep_write_packet(usbd_dev, 0x83, buf, 10);
        return 1;
      }
    case USB_CDC_REQ_SET_LINE_CODING: 
      {
        if(*len < sizeof(struct usb_cdc_line_coding))
          return 0; 
        memcpy(&linecoding,*buf,sizeof(struct usb_cdc_line_coding));
        return 1;
      }
    case 0x88:
      {
        uint8_t x;
        int res=-1;
        if(*len > 0 && req->wValue==0x10) //echo
          usbd_ep_write_packet(usbd_dev, 0x83, *buf, *len);
        else if(*len == 0 && req->wValue==0x11){
          s_write(1,"nss",3);
          switch(req->wIndex){
            case 0: selected_nss=GPIO12;break;
            case 1: selected_nss=GPIO3;break;
            case 2: selected_nss=GPIO4;break;
            case 3: selected_nss=GPIO5;break;
            case 4: selected_nss=GPIO6;break;
            default:selected_nss=GPIO12;
          }
          xcout(selected_nss);
          res=1;
        } else if(*len == 0 && req->wValue==0x12){
          if(req->wIndex & 1) gpio_clear(GPIOB, GPIO12);
          if(req->wIndex & 2) gpio_clear(GPIOB, GPIO3);
          if(req->wIndex & 4) gpio_clear(GPIOB, GPIO4);
          if(req->wIndex & 8) gpio_clear(GPIOB, GPIO5);
          if(req->wIndex & 16) gpio_clear(GPIOB, GPIO6);
          res=1;
        } else if(*len == 0 && req->wValue==0x13){
          if(req->wIndex & 1) gpio_set(GPIOB, GPIO12);
          if(req->wIndex & 2) gpio_set(GPIOB, GPIO3);
          if(req->wIndex & 4) gpio_set(GPIOB, GPIO4);
          if(req->wIndex & 8) gpio_set(GPIOB, GPIO5);
          if(req->wIndex & 16) gpio_set(GPIOB, GPIO6);
          res=1;
        } else if(*len == 0 && req->wValue==0x14){
          uint8_t port=gpio_port_read(GPIOB);
          res=0;
          if(port & GPIO12) res|=1;
          if(port & GPIO3)  res|=2;
          if(port & GPIO4)  res|=4;
          if(port & GPIO5)  res|=8;
          if(port & GPIO6)  res|=16;
        } else if(*len == 0 && req->wValue==0x15){
          uint8_t port=gpio_port_read(GPIOA);
          res=port & 0x0f;
        }else{
          u_write(1,(uint8_t*)"X",1);
          u_write(1,(uint8_t*)*buf,*len);
          u_write(1,(uint8_t*)"\r\n",2);

          uint8_t *lc=(void*)&linecoding;
          for(x=0;x<sizeof(struct usb_cdc_line_coding);x++){
            xcout(lc[x]);
          }
          u_write(1,(uint8_t*)"\r\n",2);
          usbd_ep_write_packet(usbd_dev, 0x83, &linecoding, sizeof(struct usb_cdc_line_coding));
          return 1;
        }
        if(res>=0){
          uint8_t r=res;
          usbd_ep_write_packet(usbd_dev, 0x83, &r, 1);
        }
      }
  }
  return 0;
}

static void usb_set_config(usbd_device *usbd_dev, uint16_t wValue) {
    (void)wValue;
    usbd_ep_setup(usbd_dev, EP_CDC0_R, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    usbd_ep_setup(usbd_dev, EP_CDC0_T, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    usbd_ep_setup(usbd_dev, EP_CDC0_I, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
            usbd_dev,
            USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
            USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
            cdcacm_control_request);
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

static void master_thread(uint32_t args __maybe_unused) {
  s_write(1,"MASTER\r\n",8);
  struct spi_transfer_storage sts;
  while(1){
    uint8_t status = atomQueueGet(&spi_in, 0, (void*)&sts);
    if(status == ATOM_OK){
      s_write(1,"+",1);
      xcout(selected_nss);
      xcout(sts.len);
      int i=0;
      gpio_clear(GPIOB, selected_nss);
      for(;i<sts.len;i++){
        uint16_t r=spi_xfer(SPI2, sts.buf[i]);
        sts.buf[i]=r;
      }
      gpio_set(GPIOB, selected_nss);
      usbd_ep_write_packet(usb, EP_CDC0_T, sts.buf, sts.len);
    }else{
      s_write(1,"-",1);
    }
    s_write(1,"DONE\r\n",6);
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

static void spi_setup(void) {
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO12 | GPIO13 | GPIO15 );

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
  spi_reset(SPI2);

  SPI2_I2SCFGR = 0; //disable i2s
  /* Set up SPI in Master mode with:
   * Clock baud rate: 1/64 of peripheral clock frequency
   * Clock polarity: Idle High
   * Clock phase: Data valid on 2nd clock pulse
   * Data frame format: 8-bit
   * Frame format: MSB First
   */
  spi_init_master(SPI2, SPI_CR1_BAUDRATE_FPCLK_DIV_16, SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
      SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
  spi_enable_software_slave_management(SPI2);
  spi_set_nss_high(SPI2);

  spi_enable(SPI2);

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
  gpio_set(GPIOB, GPIO12);
}

int main(void) {
        rcc_clock_setup_in_hsi_out_48mhz();
        //rcc_clock_setup_in_hse_8mhz_out_24mhz();

        rcc_periph_clock_enable(RCC_GPIOA);
        rcc_periph_clock_enable(RCC_GPIOB);
        rcc_periph_clock_enable(RCC_GPIOC);
        rcc_periph_clock_enable(RCC_AFIO);
        rcc_periph_clock_enable(RCC_USART1);
        rcc_periph_clock_enable(RCC_SPI2);

        AFIO_MAPR |= AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON;

        usart_setup();

        cm_mask_interrupts(true);
        systick_set_frequency(SYSTEM_TICKS_PER_SEC, 24000000);
        systick_interrupt_enable();
        systick_counter_enable();

        nvic_set_priority(NVIC_PENDSV_IRQ, 0xFF);
        nvic_set_priority(NVIC_SYSTICK_IRQ, 0xFE);

        spi_setup();

        //USB PULLUP
        gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);
        gpio_clear(GPIOA, GPIO15);

        /* extra gpo/NSS B3...B6 */
        gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
            GPIO3|GPIO3|GPIO5|GPIO6);
        gpio_set(GPIOB, GPIO3|GPIO4|GPIO5|GPIO6);

        /* inputs (GPI) A0...A3 */ 
        gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, 
            GPIO0|GPIO1|GPIO2|GPIO3);
        gpio_clear(GPIOC, GPIO0|GPIO1|GPIO2|GPIO3);

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

        gpio_set(GPIOA, GPIO15);
        if(atomOSInit(idle_stack, sizeof(idle_stack), FALSE) != ATOM_OK) 
          fault(1);
        if (atomQueueCreate (&uart1_rx, uart1_rx_storage, sizeof(uint8_t), 
              sizeof(uart1_rx_storage)) != ATOM_OK) 
          fault(2);
        if (atomQueueCreate (&uart1_tx, uart1_tx_storage, sizeof(uint8_t), 
              sizeof(uart1_tx_storage)) != ATOM_OK) 
          fault(3);
        if (atomQueueCreate (&spi_in, (uint8_t *)spi_in_storage,
              sizeof(struct spi_transfer_storage), 
              SPI_ITEMS) != ATOM_OK) 
          fault(4);

        atomThreadCreate(&master_thread_tcb, 10, master_thread, 0,
                master_thread_stack, sizeof(master_thread_stack), TRUE);

        atomOSStart();
        while (1){
        }
    }

int s_write(int file, char *ptr, int len) {
    return u_write(file, (uint8_t *)ptr,len);
};

int u_write(int file, uint8_t *ptr, int len) {
    int i;
    for (i = 0; i < len; i++){
        switch(file){
            case 1: //DEBUG
                atomQueuePut(&uart1_tx,-1, (uint8_t*) &ptr[i]);
                break;
        }
    }
    switch(file){
        case 1:
            USART_CR1(USART1) |= USART_CR1_TXEIE;
            break;
    }
    return i;
}
