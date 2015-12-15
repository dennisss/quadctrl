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




void motors_init(char *fsPath, int vId, int pId, int fd){

    usb_serial_init();

    usb_serial_open(vId, pId, fd);

}


void motors_destroy(){
    usb_serial_close();
    usb_serial_destroy();

}

static unsigned char buf[4];
void motors_set(float *speeds){

    // Convert to byte size
    for(int i = 0; i < 4; i++){
        buf[i] = 255*speeds[i];
    }

    int r = usb_serial_write(buf, 4);

    if(r != 4){
        LOGI("failed to write to usb");
    }
}
