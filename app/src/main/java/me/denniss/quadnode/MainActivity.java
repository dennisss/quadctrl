package me.denniss.quadnode;

import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;

import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.FileOutputStream;

public class MainActivity extends RosActivity {

    public MainActivity(){
        super("Quadcopter Node", "Quadcopter Node");
    }

    private static UsbDeviceConnection conn;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

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


            Quadcopter.setup("/dev/bus/usb", device.getVendorId(), device.getProductId(), fd);


            Toast toast = Toast.makeText(getApplicationContext(), "USB Motors Connected!", Toast.LENGTH_LONG);
            toast.show();
        }

    }




    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        Listener l = new Listener();

        NodeConfiguration nodeConfiguration = NodeConfiguration.newPublic(getRosHostname());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(l, nodeConfiguration);

    }
}
