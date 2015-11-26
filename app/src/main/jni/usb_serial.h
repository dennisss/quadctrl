#ifndef USB_SERIAL_H_
#define USB_SERIAL_H_

int usb_serial_init();
int usb_serial_destroy();

int usb_serial_open(int vendorId, int productId, int fd);
int usb_serial_close();


int usb_serial_write(unsigned char *buffer, int len);
int usb_serial_read(unsigned char *buffer, int len);


#endif