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

static unsigned char buf[5];
void motors_set(float *speeds){

    buf[0] = 'm';

    // Convert to byte size
    for(int i = 0; i < 4; i++){
        float s = speeds[i];
        if(s > 1)
            s = 1;
        else if(s < 0)
            s = 0;

        buf[1+i] = 255*s;
    }

    int r = usb_serial_write(buf, 5);

    if(r != 5){
        LOGI("failed to write to usb %d", r);
    }
}

double battery_level(){
    unsigned char c = 'b';

    if(usb_serial_write(&c, 1) != 1){
        return -1;
    }

    if(usb_serial_read(&c, 1) != 1){
        return -1;
    }

    return ((double)c) / 100.0;
}
