package me.denniss.quadnode;

/**
 * Created by dennis on 11/24/15.
 */
public class Quadcopter {

    static {
        System.loadLibrary("quadcopter");
    }


    /*
    *
    */
    public static native void setup(String fspath, int usbVendorId, int usbProductId, int fd);

    public static native void setthrottle(float throttle);

    public static native void start();
    public static native void stop();
    // takeoff()
    // land()

    public static native void setpoint();


    /**
     * Initialize the native hardware connection
     */
    public static void connect(){

        // UsbManager.requestPermission


        // In native, open device using libftdi/libusb


    }

}
