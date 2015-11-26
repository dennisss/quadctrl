/*
 * Most arduinos will run with the FTDI driver or the CDC-ACM protocol
 * Small driver for communicating with USB serial converters
 */

// Based on https://raw.githubusercontent.com/tytouf/libusb-cdc-example/master/cdc_example.c


#include "log.h"

#include <libusb.h>



#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02

/* The Endpoint address are hard coded. You should use lsusb -v to find
 * the values corresponding to your device.
 */
static int ep_in_addr  = 0x83;
static int ep_out_addr = 0x04;

static struct libusb_device_handle *handle = NULL;


int usb_serial_init(){

    int res;

    res = libusb_init2(NULL, "/dev/bus/usb");
    if(res < 0){
        LOGI("error initializing libusb: %s", libusb_error_name(res));
        return 1;
    }

    return 0;
}

int usb_serial_destroy(){
    libusb_exit(NULL);
    return 0;
}


int usb_serial_open(int vendorId, int productId, int fd){

    int res;
    libusb_device *device = NULL;

    // Find corresponding device
    libusb_device **list;
    int cnt = libusb_get_device_list(NULL, &list);
    LOGI("found %d devices", cnt);
    struct libusb_device_descriptor dd;
    for (int i = 0; i < cnt; i++) {

        libusb_get_device_descriptor(list[i], &dd);

        if(dd.idVendor == vendorId && dd.idProduct == productId){
            device = list[i];
            break;
        }
    }

    if(device == NULL){
        LOGI("failed to find the corresponding libusb device");
        return 1;
    }

    // Open the device
    res = libusb_open2(device, &handle, fd);
    libusb_free_device_list(list, 1);

    if(res != 0){
        LOGE("failed to open usb device using fd");
        return 1;
    }


    // Claim the CDC-ACM control (0) and data(1) interfaces
    for(int i = 0; i < 2; i++){
        if(libusb_kernel_driver_active(handle, i)){
            libusb_detach_kernel_driver(handle, i);
        }

        res = libusb_claim_interface(handle, i);
        if(res < 0){
            LOGI("error claiming interface %d: %s", i, libusb_error_name(res));
        }
    }


    // CDC-ACM Control Configuration
    // Set line state
    res = libusb_control_transfer(handle, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, NULL, 0, 0);
    if(res < 0){
        LOGI("error during control transfer: %s", libusb_error_name(res));
    }


    //set line encoding: here 9600 8N1
    // 9600 = 0x2580 ~> 0x80, 0x25 in little endian
    //unsigned char encoding[] = { 0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08 };
    unsigned char encoding[] = { 0x00, 0xc2, 0x01, 0x00, 0x00, 0x00, 0x08 }; // 115200 = 0x01 c2 00
    res = libusb_control_transfer(handle, 0x21, 0x20, 0, 0, encoding, sizeof(encoding), 0);
    if(res < 0){
        LOGI("error during control transfer: %s", libusb_error_name(res));
    }

    return 0;
}

int usb_serial_close(){
    // TODO: Release interfaces too?
    libusb_release_interface(handle, 0);
    libusb_release_interface(handle, 1);
    libusb_close(handle);
    return 0;
}




int usb_serial_write(unsigned char *buffer, int len){
    int written;
    if (libusb_bulk_transfer(handle, ep_out_addr, buffer, len, &written, 0) < 0){
        LOGI("failed to write data over usb");
        return -1;
    }

    return written;
}

int usb_serial_read(unsigned char *buffer, int len){
    int read;
    int res = libusb_bulk_transfer(handle, ep_in_addr, buffer, len, &read, 1000);
    if(res == LIBUSB_ERROR_TIMEOUT){
        LOGI("timeout (%d)\n", read);
        return -1;
    } else if (res < 0) {
        LOGI("error while reading data");
        return -1;
    }

    return read;
}