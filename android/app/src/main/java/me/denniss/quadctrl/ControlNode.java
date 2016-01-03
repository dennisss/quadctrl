package me.denniss.quadctrl;

public class ControlNode {

    static {
        System.loadLibrary("quadphone");
    }

	public static native void init(String master, String ip);
	public static native void destroy();


	public static native void connectUSB(String fspath, int usbVendorId, int usbProductId, int fd);


	// "Arm's" the quadcopter. Allows motor signals to be sent
	public static native void start();

	// "Dis-arm"'s the quadcopter. Motors will hard stop without trying to land
	public static native void stop();


	public static native void calibrate();


	public static native double testio();

	public static native void setMotors(float[] speeds);

}
