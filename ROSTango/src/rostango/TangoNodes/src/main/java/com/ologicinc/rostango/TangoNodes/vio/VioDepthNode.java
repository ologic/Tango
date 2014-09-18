package com.ologicinc.rostango.TangoNodes.vio;

/*
 * Created by Shiloh Curtis on 9/11/2014
 */

import com.motorola.atap.androidvioservice.VinsServiceHelper;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

/* yellowstone api */
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoCoordinateFramePair;

import android.content.Context;
import android.util.Log;

import java.util.ArrayList;
import java.nio.FloatBuffer;

import com.ologicinc.rostango.TangoNodes.depth.*;

public class VioDepthNode implements NodeMain {

    private static String TAG = VioNode.class.getSimpleName();

    private VinsServiceHelper mVinsServiceHelper;

    private Tango mTango;
    private TangoConfig mConfig;
    private ArrayList<TangoCoordinateFramePair> mFramePairs;

    private TangoOdomPublisher mTangoOdomPublisher;
    private TangoPosePublisher mTangoPosePublisher;
    private TangoTfPublisher mTangoTfPublisher;
    private PointCloudPublisher mPointCloudPublisher;

    public static final int PEANUT = 0;
    public static final int YELLOWSTONE = 1;

    private int mModel;

    public VioDepthNode(Context context) {

        Log.i(TAG, "Build.MODEL="+android.os.Build.MODEL);

        if (android.os.Build.MODEL.equals("Yellowstone")) {
            // Instantiate the Yellowstone Tango service
            mTango = new Tango(context);
            mModel = YELLOWSTONE;
            Log.i(TAG, "YELLOWSTONE device");
        } else if (android.os.Build.MODEL.equals("Peanut")) {
            mVinsServiceHelper = new VinsServiceHelper((android.app.Activity)context);
            mModel = PEANUT;
            Log.i(TAG, "PEANUT device");
        }
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_vio");
    }

    public void setVinsServiceHelper(VinsServiceHelper vins) {
        if (mModel == PEANUT) {
            mVinsServiceHelper = vins;
        }
    }

    public int getModel() {
        return mModel;
    }

    private void startYellowstone() {
        // Create a new Tango Configuration and enable the MotionTracking API
        mConfig = new TangoConfig();
        mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT, mConfig);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORESET, false);

        // Lock configuration and connect to Tango
        mTango.lockConfig(mConfig);

        int i = 0;
        while (i != 1) {
            if (mTango.connect() == Tango.STATUS_SUCCESS) {
                System.out.println("Tango connected");
                i = 1;
            } else {
                System.out.println("Could not connect to Tango");
            }
        }

        // Select coordinate frame pairs
        mFramePairs = new ArrayList<TangoCoordinateFramePair>();
        mFramePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE)
        );

        OnTangoUpdateListener tangoUpdateListener = new OnTangoUpdateListener() {

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                // Update vio data
                try {
                    updateYSTranslation(pose);
                    updateYSRotation(pose);
                    mTangoOdomPublisher.publishOdom();
                    mTangoPosePublisher.publishPose();
                    mTangoTfPublisher.publishTransforms();
                } catch (Exception e) {
                }
            }

            @Override
            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                // Update depth data
                updateYSDepth(xyzIj);
            }

            @Override
            public void onTangoEvent(TangoEvent arg0) {

            }
        };
        mTango.connectListener(mFramePairs, tangoUpdateListener);
    }

    @Override
    public void onStart(ConnectedNode node) {

        if (mModel==YELLOWSTONE) {
            System.out.println("Starting Yellowstone");
            startYellowstone();
        }

        mTangoOdomPublisher = new TangoOdomPublisher(node);
        mTangoPosePublisher = new TangoPosePublisher(node);
        mTangoTfPublisher = new TangoTfPublisher(node);
        mPointCloudPublisher = new PointCloudPublisher(node);

        if (mModel == PEANUT) {
            node.executeCancellableLoop(new CancellableLoop() {
                @Override
                protected void loop() throws InterruptedException {

                    Thread.sleep(30);
                    final double[] posState = mVinsServiceHelper.getStateInFullStateFormat();
                    final double[] rotState = mVinsServiceHelper.getStateInUnityFormat();

                    // Generate the TF message
                    updateTranslation(posState);
                    Thread.sleep(30);

                    updateRotation(rotState);
                    Thread.sleep(30);

                    mTangoOdomPublisher.publishOdom();
                    mTangoPosePublisher.publishPose();
                    mTangoTfPublisher.publishTransforms();
                }
            });
        }

    }

    @Override
    public void onShutdown(Node node) {
        if (mModel==PEANUT) {
            mVinsServiceHelper.shutdown();
        }

        if (mModel==YELLOWSTONE) {
            mTango.unlockConfig();
            mTango.disconnect();
        }
    }

    @Override
    public void onShutdownComplete(Node node) {}

    public void onPause() {
        mTango.unlockConfig();
        mTango.disconnect();
    }

    public void onResume() {
        mTango.lockConfig(mConfig);
        mTango.connect();
    }

    public void onDestroy() {
        mTango.unlockConfig();
    }

    @Override
    public void onError(Node node, Throwable throwable) {}

    private void updateTranslation(double[] state) {
        mTangoOdomPublisher.setPosePoint(state[5],-state[4], state[6]);
        mTangoPosePublisher.setPoint(state[5],-state[4], state[6]);
        mTangoTfPublisher.setTranslation(state[5],-state[4], state[6]);
    }

    private void updateRotation(double[] state) {
        mTangoOdomPublisher.setPoseQuat(-state[2], state[0], -state[1], state[3]);
        mTangoPosePublisher.setQuat(-state[2],state[0],-state[1],state[3]);
        mTangoTfPublisher.setRotation(-state[2],state[0],-state[1],state[3]);
    }

    private void updateYSTranslation(TangoPoseData pose) {
        mTangoOdomPublisher.setPosePoint(pose.translation[0],pose.translation[1], pose.translation[2]);
        mTangoPosePublisher.setPoint(pose.translation[0],pose.translation[1], pose.translation[2]);
        mTangoTfPublisher.setTranslation(pose.translation[0],pose.translation[1], pose.translation[2]);
    }

    private void updateYSRotation(TangoPoseData pose) {
        mTangoOdomPublisher.setPoseQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoPosePublisher.setQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoTfPublisher.setRotation(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
    }

    private void updateYSDepth(TangoXyzIjData xyzIj) {
        FloatBuffer xyzBuffer = xyzIj.getXyzBuffer().duplicate();
        float[] data = new float[xyzBuffer.limit()];

        int d = 0;
        while (xyzBuffer.hasRemaining()) {
            data[d] = xyzBuffer.get();
            d++;
        }

        mPointCloudPublisher.onNewPointCloud(data);
    }
}