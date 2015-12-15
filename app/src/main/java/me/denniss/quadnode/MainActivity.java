package me.denniss.quadnode;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.view.WindowManager;
import android.widget.Toast;

public class MainActivity extends Activity {

    // TODO: Cleanup the connection on both the java and native side

    private static UsbDeviceConnection conn = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);


		ControlNode.init();




        Intent intent = getIntent();

        Log.i("MainActivity", intent.getAction());

        if(intent.getAction() == "android.hardware.usb.action.USB_DEVICE_ATTACHED") {
            UsbManager manager = (UsbManager) getSystemService(Context.USB_SERVICE);
            UsbDevice device = (UsbDevice) intent.getParcelableExtra(UsbManager.EXTRA_DEVICE);

            String path = device.getDeviceName();
            conn = manager.openDevice(device);
            int fd = conn.getFileDescriptor();

            Log.i("MainActivity", path + " (" + fd + ")");

            Log.i("MainActivity", "Has # of interfaces: " + device.getInterfaceCount());

            for(int i = 0; i < device.getInterfaceCount(); i++){
                UsbInterface iface = device.getInterface(i);
                Log.i("MainActivity", iface.toString() + "  :::  " + iface.getName());

                for(int j = 0; j < iface.getEndpointCount(); j++) {
                    UsbEndpoint ep = iface.getEndpoint(j);
                    Log.i("MainActivity", Integer.toHexString(ep.getAddress()));
                }
            }


            ControlNode.connectUSB("/dev/bus/usb", device.getVendorId(), device.getProductId(), fd);


            Toast toast = Toast.makeText(getApplicationContext(), "USB Motors Connected!", Toast.LENGTH_LONG);
            toast.show();
        }

    }


    @Override
    protected void onDestroy() {

		ControlNode.destroy();

        super.onDestroy();
    }



    @Override
    protected void onStart() {
        super.onStart();

        // Start native node
    }

    @Override
    protected void onStop() {
        super.onStop();

        // Stop native node
    }
}
