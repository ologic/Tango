package com.ologicinc.rostango.tango_serial;

import android.util.Log;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
// XXX import java.nio.charset.Charset;

import com.github.ologic.android_ologic.usbserial.driver.UsbSerialDriver;

import geometry_msgs.PoseStamped;

import com.MAVLink.Messages.*;
import com.MAVLink.Messages.ardupilotmega.*;
import com.MAVLink.Messages.enums.*;


/**
 * Created by brandonb on 8/4/14.
 */
public class VioListenerNode implements NodeMain {

    private static final boolean DEBUG = true;
    private static final String TAG = TangoSerial.class.getSimpleName();

    private Subscriber<PoseStamped> mSubscriber;

    private static UsbSerialDriver mSerialDriver = null;
    private InputStream mInputStream;
    private OutputStream mOutputStream;

    private TangoHeartbeat mHeartbeat;

    public VioListenerNode(UsbSerialDriver serialDriver) {

        mSerialDriver = serialDriver;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_serial");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        // Connect the usb
        usbConnect();

        // Create the heartbeat
        mHeartbeat = new TangoHeartbeat(this,1);
        mHeartbeat.setActive(true);

        mSubscriber = connectedNode.newSubscriber("/tango_pose", PoseStamped._TYPE);

        mSubscriber.addMessageListener(new MessageListener<PoseStamped>() {
            @Override
            public void onNewMessage(PoseStamped message) {
                if (DEBUG) Log.d(TAG, message.toString());
                String quat;
                String pos;

                msg_vision_position_estimate mavLinkVision = new msg_vision_position_estimate();
                mavLinkVision.x = (float)message.getPose().getPosition().getX();
                mavLinkVision.y = (float)message.getPose().getPosition().getY();
                mavLinkVision.z = (float)message.getPose().getPosition().getZ();

                mavLinkVision.pitch = 0.0f;
                mavLinkVision.roll = 0.0f;
                mavLinkVision.yaw = 0.0f;

                long lt = System.currentTimeMillis() * 1000;
                mavLinkVision.usec = lt;

                MAVLinkPacket mavPacket = mavLinkVision.pack();
                sendMavMessage(mavPacket);


                /*
                pos = String.format("\npos: x: %.4f, y: %.4f, z: %.4f\n",
                        message.getPose().getPosition().getX(),
                        message.getPose().getPosition().getY(),
                        message.getPose().getPosition().getZ());

                quat = String.format("quat: x: %.4f, y: %.4f, z: %.4f, w: %.4f\n",
                        message.getPose().getOrientation().getX(),
                        message.getPose().getOrientation().getY(),
                        message.getPose().getOrientation().getZ(),
                        message.getPose().getOrientation().getW()
                        );


                byte[] pos_byte = pos.getBytes(Charset.forName("UTF-8"));
                byte[] quat_byte = quat.getBytes(Charset.forName("UTF-8"));

                try {
                    mOutputStream.write(pos_byte);
                    mOutputStream.write(quat_byte);
                    mOutputStream.flush();
                } catch (IOException e) {
                    e.printStackTrace();
                }
                */
            }
        });
    }

    public synchronized void sendMavMessage(MAVLinkPacket mavPacket) {
        byte[] mavBuffer = mavPacket.encodePacket();

        try {
            mOutputStream.write(mavBuffer);
            mOutputStream.flush();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onShutdown(Node node) {
        // Disconnect the usb
        usbDisconnect();
    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

    public boolean usbConnect() {
        if (mSerialDriver != null) {
            try {
                if (DEBUG) Log.d(TAG, "usbConnect().");

                // Open the driver.
                mSerialDriver.open();

                // Set the serial parameters.
                mSerialDriver.setParameters(230400, 8, UsbSerialDriver.STOPBITS_1, UsbSerialDriver.PARITY_NONE);

                // Associate input/output streams with the USB driver.
                mInputStream = mSerialDriver.getInputStream();
                mOutputStream = mSerialDriver.getOutputStream();

                // Sanity check the reader and writers before creating the ROS
                // Serial object.
                if ((mInputStream == null) && (mOutputStream == null)) {
                    throw new IOException();
                }

                Log.i(TAG, "Running USB communications.");
            } catch (IOException e) {
                Log.e(TAG, "Error setting up device: " + e.getMessage(), e);
                try {
                    if (mInputStream != null) {
                        mInputStream.close();
                    }
                    if (mOutputStream != null) {
                        mOutputStream.flush();
                        mOutputStream.close();
                    }
                    mSerialDriver.close();
                    mSerialDriver = null;
                } catch (IOException e2) {
                    Log.e(TAG, "Error with streams: " + e2.getMessage(), e2);
                }
            }
        }
        return true;
    }

    public void usbDisconnect() {
        try {
            if (DEBUG) Log.d(TAG, "usbDisconnect().");
            if (mInputStream != null) {
                mInputStream.close();
            }
            if (mOutputStream != null) {
                mOutputStream.flush();
                mOutputStream.close();
            }
            if (mSerialDriver != null) {
                mSerialDriver.close();
                mSerialDriver = null;
            }
        } catch (IOException e2) {
            Log.e(TAG, "Error closing usb: " + e2.getMessage(), e2);
        }
    }

}
