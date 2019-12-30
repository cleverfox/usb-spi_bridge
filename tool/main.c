#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <usb.h>        /* this is libusb */
#include "opendevice.h" /* common code moved to separate module */
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

const char *vendor = "libopencm3.org";
const char *product = "USB-SPI";
const int vid = 0x3137;
const int pid = 0xc0d0;

usb_dev_handle      *handle = NULL;
char                buffer[32];
int                 cnt;

//-----------------------------------------------------------
int main(int argc, char **argv) {
  //usb_dev_handle      *handle = NULL;
  usb_init();
  /* compute VID/PID from usbconfig.h so that there is a central source of information */
  /* The following function is in opendevice.c: */
  if(usbOpenDevice(&handle, vid, vendor, pid, product, NULL, NULL, NULL) != 0){
    fprintf(stderr, "Could not find USB device \"%s\" with vid=0x%x pid=0x%x\n", product, vid, pid);
    exit(1);
  }


  uint8_t bufread[17000];
  unsigned long off = 0;
  size_t count =16384;
  int ret;
  int i;

  memset(bufread, 0, sizeof(bufread));

  /*
  ret=usb_get_descriptor_by_endpoint(handle, 1, 0, 1, bufread, 16384);
  printf("ret %d\n",ret);
  */
  /*
  usb_control_msg(usb_dev_handle * dev, 
  int requesttype,
  int request, 
  int value, int index, 
  char *bytes, int size, int timeout);
  */
  memcpy(bufread,"test12",6);
  /*
  ret=usb_control_msg(handle, 
      33, 
      0x20,
      0x0, 2, 
      bufread, 7, 5000);
  printf("ctl ret %d\n",ret);
  */

  ret=usb_control_msg(handle, 
      33, 
      0x88,
      0x0, 2, 
      bufread, 6, 5000);
  printf("ctl ret %d\n",ret);
  ret=usb_bulk_read(handle, 0x83, bufread, 32, 100);
  printf("intread ret %d\n",ret);
  for(i=0;i<ret;i++){
    printf("%02x ",bufread[i]);
  }
  printf("\n---\n");

  ret=usb_control_msg(handle, 33, 0x88, 0x12, 0x0e, NULL, 0, 5000);
  printf("ctl wr ret %d\n",ret);
  ret=usb_bulk_read(handle, 0x83, bufread, 32, 100);
  printf("ctl rd ret %d\n",ret);

  ret=usb_control_msg(handle, 33, 0x88, 0x13, 0x1e, NULL, 0, 5000);
  printf("ctl ret %d\n",ret);
  ret=usb_bulk_read(handle, 0x83, bufread, 32, 100);

  ret=usb_control_msg(handle, 33, 0x88, 0x14, 0, NULL, 0, 5000);
  printf("ctl wr ret %d\n",ret);
  ret=usb_bulk_read(handle, 0x83, bufread, 32, 100);
  printf("intread ret %d\n",ret);
  for(i=0;i<ret;i++){
    printf("%02x ",bufread[i]);
  }
  printf("\n---\n");

  ret=usb_control_msg(handle, 33, 0x88, 0x15, 0, NULL, 0, 5000);
  printf("ctl wr ret %d\n",ret);
  ret=usb_bulk_read(handle, 0x83, bufread, 32, 100);
  printf("intread ret %d\n",ret);
  for(i=0;i<ret;i++){
    printf("%02x ",bufread[i]);
  }
  printf("\n---\n");



  ret=usb_control_msg(handle, 33, 0x88, 0x11, 0, NULL, 0, 5000);
  printf("set nss ret %d\n",ret);

  ret=usb_bulk_write(handle, 0x01, "medved1", 7, 1000);
  printf("bwr ret %d\n",ret);

  ret=usb_bulk_read(handle, 0x82, bufread, 32, 1);
  printf("bre ret %d\n",ret);
  for(i=0;i<ret;i++){
    printf("%02x ",bufread[i]);
  }
  printf("\n");


  ret=usb_control_msg(handle, 33, 0x88, 0x11, 1, NULL, 0, 5000);
  printf("set nss ret %d\n",ret);

  ret=usb_bulk_write(handle, 0x01, "vedmed2", 7, 1000);
  printf("bwr ret %d\n",ret);

  ret=usb_bulk_read(handle, 0x82, bufread, 32, 1);
  printf("bre ret %d\n",ret);
  for(i=0;i<ret;i++){
    printf("%02x ",bufread[i]);
  }
  printf("\n");

  for(i=0;i<3;i++){
    ret=usb_bulk_write(handle, 0x01, "va", 2, 1000);
    printf("bwr ret %d\n",ret);

    ret=usb_bulk_read(handle, 0x82, bufread, 32, 1);
    printf("bre ret %d\n",ret);
  }



  usb_close(handle);
  return 0;
}




