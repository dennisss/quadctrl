/*

 Communicates with the motors via an arduino
 The arduino receives 4-bytes at a time specifying the desired speed of each motor

 */

#include "motors.h"
#include "log.h"
#include "usb_serial.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb.h>



//static struct ftdi_context *ftdi;

void motors_init(char *fsPath, int vId, int pId, int fd){

    usb_serial_init();

    usb_serial_open(vId, pId, fd);



    /*
    //struct ftdi_context *ftdi;
    unsigned char buf[1024];
    int f = 0, i;
    int baudrate = 115200;

    // Init
    if ((ftdi = ftdi_new()) == 0) {
        LOGE("ftdi_new failed\n");
        return;
    }

    // Select interface
    //ftdi_set_interface(ftdi, INTERFACE_ANY);


    LOGI("HELLO 1");

    libusb_device **list;

    int cnt = libusb_get_device_list(ftdi->usb_ctx, &list);


    libusb_device *dev;

    struct libusb_device_descriptor dd;
    for (i = 0; i < cnt; i++) {
        libusb_device *device = list[i];

        libusb_get_device_descriptor(list[i], &dd);

        LOGI("%04x", dd.idVendor);

        if(dd.idVendor == 9025){ //(find_known_device(dd.idVendor, dd.idProduct)) {
            dev = device;
        }

        device = NULL;
    }

    LOGI("Found %d devices", cnt);


    //libusb_device *dev = libusb_get_device2(ftdi->usb_ctx, path);

    if(dev == NULL)
        LOGI("Couldn't get device for path");

    LOGI("HELLO 2");

    //libusb_device_handle *handle;


    //if(libusb_open2(dev, &handle, fd) != 0){
    //    LOGE("failed to open usb device using fd");
    //    return;
    //}

    LOGI("HELLO- 3");


    //ftdi_set_usbdev(ftdi, handle);
    f = ftdi_usb_open_dev2(ftdi, dev, fd);

    // Open device
    //f = ftdi_usb_open(ftdi, vid, pid);



    if (f < 0) {
        LOGE("unable to open ftdi device: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        return;
    }


    // Set baudrate
    //f = ftdi_set_baudrate(ftdi, baudrate);
    if (f < 0) {
        LOGE("unable to set baudrate: %d (%s)\n", f, ftdi_get_error_string(ftdi));
        return;
    }

    */

}


void motors_destroy(){
    usb_serial_close();
    usb_serial_destroy();

    //ftdi_usb_close(ftdi);
    //do_deinit:
    //ftdi_free(ftdi);
}

static unsigned char buf[4];
void motors_set(float *speeds){

    // Convert to byte size
    for(int i = 0; i < 4; i++){
        buf[i] = 255*speeds[i];
    }

    int r = usb_serial_write(buf, 4);
    //int f = ftdi_write_data(ftdi, buf, 4);

    if(r != 4){
        LOGI("failed to write to usb");
    }

}
