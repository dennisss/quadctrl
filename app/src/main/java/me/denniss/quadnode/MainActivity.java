package me.denniss.quadnode;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.util.Log;
import android.view.WindowManager;
import android.widget.Toast;

import java.math.BigInteger;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.nio.ByteOrder;

public class MainActivity extends Activity {

    // TODO: Cleanup the connection on both the java and native side

    private static UsbDeviceConnection conn = null;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        String myip = wifiIpAddress(this);

        Log.i("MainActivity", myip);

		ControlNode.init("http://192.168.0.194:11311", myip);




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


    protected String wifiIpAddress(Context context) {
        WifiManager wifiManager = (WifiManager) context.getSystemService(Context.WIFI_SERVICE);
        int ipAddress = wifiManager.getConnectionInfo().getIpAddress();

        // Convert little-endian to big-endianif needed
        if (ByteOrder.nativeOrder().equals(ByteOrder.LITTLE_ENDIAN)) {
            ipAddress = Integer.reverseBytes(ipAddress);
        }

        byte[] ipByteArray = BigInteger.valueOf(ipAddress).toByteArray();

        String ipAddressString;
        try {
            ipAddressString = InetAddress.getByAddress(ipByteArray).getHostAddress();
        } catch (UnknownHostException ex) {
            Log.e("WIFIIP", "Unable to get host address.");
            ipAddressString = null;
        }

        return ipAddressString;
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
