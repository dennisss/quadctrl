#ifndef USB_SERIAL_H_
#define USB_SERIAL_H_

// This is a general single-device

// TODO: Make this thread safe

// Globally initialize the usb drivers
int usb_serial_init();


int usb_serial_destroy();

int usb_serial_open(int vendorId, int productId, int fd);
int usb_serial_close();


int usb_serial_write(unsigned char *buffer, int len);
int usb_serial_read(unsigned char *buffer, int len);


#endif