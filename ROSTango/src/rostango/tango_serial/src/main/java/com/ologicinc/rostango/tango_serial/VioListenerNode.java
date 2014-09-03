package com.ologicinc.rostango.tango_serial;

import android.util.Log;
import android.widget.TextView;

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

class MavLinkEuler {
    public float roll;
    public float pitch;
    public float yaw;
}

class MavLinkDCM {
    public float m[][] = new float[3][3];
}

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

    private TextView mStatsView;

    public VioListenerNode(UsbSerialDriver serialDriver, TextView stats) {
        mStatsView = stats;
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
            int count = 0;
            /**
             * Converts a quaternion to a rotation matrix
             *
             * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
             * @param dcm a 3x3 rotation matrix
             */
            public void mavlink_quaternion_to_dcm(float quaternion[], MavLinkDCM dcm)
            {
                double a = quaternion[0];
                double b = quaternion[1];
                double c = quaternion[2];
                double d = quaternion[3];
                double aSq = a * a;
                double bSq = b * b;
                double cSq = c * c;
                double dSq = d * d;
                dcm.m[0][0] =(float) (aSq + bSq - cSq - dSq);
                dcm.m[0][1] =(float) (2.0 * (b * c - a * d));
                dcm.m[0][2] =(float) (2.0 * (a * c + b * d));
                dcm.m[1][0] =(float) (2.0 * (b * c + a * d));
                dcm.m[1][1] =(float) (aSq - bSq + cSq - dSq);
                dcm.m[1][2] =(float) (2.0 * (c * d - a * b));
                dcm.m[2][0] =(float) (2.0 * (b * d - a * c));
                dcm.m[2][1] =(float) (2.0 * (a * b + c * d));
                dcm.m[2][2] =(float) (aSq - bSq - cSq + dSq);
            }


            /**
             * Converts a rotation matrix to euler angles
             *
             * @param dcm a 3x3 rotation matrix
             * @param roll the roll angle in radians
             * @param pitch the pitch angle in radians
             * @param yaw the yaw angle in radians
             */
            public void mavlink_dcm_to_euler(MavLinkDCM dcm, MavLinkEuler attitude)
            {
                float phi, theta, psi;
                theta = (float)Math.asin(-dcm.m[2][0]);

                if (Math.abs(theta - (float)Math.PI/2) < 1.0e-3f) {
                    phi = 0.0f;
                    psi = ((float)Math.atan2(dcm.m[1][2] - dcm.m[0][1],
                            dcm.m[0][2] + dcm.m[1][1]) + phi);

                } else if (Math.abs(theta + (float)Math.PI/2) < 1.0e-3f) {
                    phi = 0.0f;
                    psi = (float)Math.atan2(dcm.m[1][2] - dcm.m[0][1],
                            dcm.m[0][2] + dcm.m[1][1] - phi);

                } else {
                    phi = (float)Math.atan2(dcm.m[2][1], dcm.m[2][2]);
                    psi = (float)Math.atan2(dcm.m[1][0], dcm.m[0][0]);
                }

                attitude.roll = phi;
                attitude.pitch = theta;
                attitude.yaw = psi;
            }

            /**
             * Converts a quaternion to euler angles
             *
             * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
             * @param roll the roll angle in radians
             * @param pitch the pitch angle in radians
             * @param yaw the yaw angle in radians
             */
            void mavlink_quaternion_to_euler(float quaternion[], MavLinkEuler attitude)
            {
                MavLinkDCM dcm = new MavLinkDCM();
                mavlink_quaternion_to_dcm(quaternion, dcm);
                mavlink_dcm_to_euler(dcm, attitude);
            }


            @Override
            public void onNewMessage(PoseStamped message) {
                if (DEBUG) Log.d(TAG, message.toString());
                String quat;
                String pos;


                msg_vision_position_estimate mavLinkVision = new msg_vision_position_estimate();
                // ?? needed ?? mavLinkVision.sysid = 100;
                // ?? needed ?? mavLinkVision.compid = 50;
                mavLinkVision.x = (float)message.getPose().getPosition().getX() * (-1);     //Changed orientation to match with NED
                mavLinkVision.y = (float)message.getPose().getPosition().getY();
                mavLinkVision.z = ((float)message.getPose().getPosition().getZ()) * (-1);   //Changed orientation to match with NED

                float[] q = new float[4];
                q[0] = (float)message.getPose().getOrientation().getW();
                q[1] = (float)message.getPose().getOrientation().getX();
                q[2] = (float)message.getPose().getOrientation().getY();
                q[3] = (float)message.getPose().getOrientation().getZ();

                MavLinkEuler e = new MavLinkEuler();
                mavlink_quaternion_to_euler(q, e);

                mavLinkVision.pitch = e.pitch *(-1);
                mavLinkVision.roll = e.roll;
                mavLinkVision.yaw = e.yaw + (float)Math.PI;

                if (mavLinkVision.yaw > Math.PI) {
                    mavLinkVision.yaw -= 2.0 * Math.PI;
                } else if (mavLinkVision.yaw <-Math.PI) {
                    mavLinkVision.yaw += 2.0 * Math.PI;
                }


                long lt = System.currentTimeMillis() * 1000;
                mavLinkVision.usec = lt;

                MAVLinkPacket mavPacket = mavLinkVision.pack();
                sendMavMessage(mavPacket);

                if (count== 2)
                {
                    pos = String.format("\npos: x: %.4f, y: %.4f, z: %.4f\n",
                            mavLinkVision.x,
                            mavLinkVision.y,
                            mavLinkVision.z);

                    quat = String.format("\nEuler: Pitch: %.4f, Roll: %.4f, Yaw: %.4f\n",
                            mavLinkVision.pitch,
                            mavLinkVision.roll,
                            mavLinkVision.yaw);

                    final String statsDump = pos + "\n\n" + quat;
                    mStatsView.post(new Runnable(){

                        @Override
                        public void run() {
                            mStatsView.setText(statsDump);
                        }
                    });
                        count=0;
                }
                count++;



                /*

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
