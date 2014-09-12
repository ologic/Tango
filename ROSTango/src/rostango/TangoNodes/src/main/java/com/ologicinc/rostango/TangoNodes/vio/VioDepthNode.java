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

import java.io.FileInputStream;
import java.util.ArrayList;

import com.ologicinc.rostango.TangoNodes.depth.*;
import java.nio.FloatBuffer; // debug
import android.os.ParcelFileDescriptor;
import android.os.Parcel;
import java.io.FileDescriptor;

public class VioDepthNode implements NodeMain {

    private static String TAG = VioNode.class.getSimpleName();

    private VinsServiceHelper mVinsServiceHelper;

    private Tango mTango;
    private TangoConfig mConfig;
    //private TangoCoordinateFramePair mFramePairs;
    private ArrayList<TangoCoordinateFramePair> mFramePairs;
    private TangoPoseData mPose;
    private TangoXyzIjData mTangoXyzIjData;
    private OnTangoUpdateListener mTangoUpdateListener;

    private TangoOdomPublisher mTangoOdomPublisher;
    private TangoPosePublisher mTangoPosePublisher;
    private TangoTfPublisher mTangoTfPublisher;
    private DepthPublisher mDepthPublisher;

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
            //startYellowstone();
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
        System.out.println("startYellowstone() called");
        // Create a new Tango Configuration and enable the MotionTracking API
        mConfig = new TangoConfig();
        mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT, mConfig);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORESET, false);

        System.out.println("mConfig set");

        // Lock configuration and connect to Tango
        mTango.lockConfig(mConfig);

        int i = 0;
        while (i != 1) {
            if (mTango.connect() == Tango.STATUS_SUCCESS) {
                System.out.println("mTango connected");
                i = 1;
            } else {
                System.out.println("Could not connect to Tango");
            }
        }

        /*
        while (mTango.connect() != Tango.STATUS_SUCCESS) {
            System.out.println("Could not connect to Tango");
        }
        */

        mPose = new TangoPoseData();

        System.out.println("mTango configured, mPose initialized");

        // Select coordinate frame pairs
        mFramePairs = new ArrayList<TangoCoordinateFramePair>();
        mFramePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        System.out.println("mFramePairs initialized");

        mTangoUpdateListener = new OnTangoUpdateListener() {

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                //System.out.println("onPoseAvailable() called");
                try {
                    updateYSTranslation(pose);
                    Thread.sleep(30);
                    updateYSRotation(pose);
                    Thread.sleep(30);
                    mTangoOdomPublisher.publishOdom();
                    mTangoPosePublisher.publishPose();
                    mTangoTfPublisher.publishTransforms();
                } catch (Exception e) {}
            }

            @Override
            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                //System.out.println("onXyzIjAvailable() called");
                updateYSDepth(xyzIj);
            }

            @Override
            public void onTangoEvent(TangoEvent arg0) {

            }
        };

        mTango.connectListener(mFramePairs, mTangoUpdateListener);

        mTangoXyzIjData = new TangoXyzIjData();
    }

    public void debugStartYellowstone() {
        if (mModel==YELLOWSTONE) {
            //startYellowstone();
        }
    }

    @Override
    public void onStart(ConnectedNode node) {
        System.out.println("VioDepthNode onStart() called");

        if (mModel==YELLOWSTONE) {
            System.out.println("VioDepthNode calling startYellowstone()");
            startYellowstone();
        }

        mTangoOdomPublisher = new TangoOdomPublisher(node);
        mTangoPosePublisher = new TangoPosePublisher(node);
        mTangoTfPublisher = new TangoTfPublisher(node);
        mDepthPublisher = new DepthPublisher(node);

        node.executeCancellableLoop(new CancellableLoop() {
            @Override
            protected void loop() throws InterruptedException {

                if (mModel==PEANUT) {
                    Thread.sleep(30);
                    final double[] posState = mVinsServiceHelper.getStateInFullStateFormat();
                    final double[] rotState = mVinsServiceHelper.getStateInUnityFormat();
                    // Generate the TF message

                    updateTranslation(posState);
                    Thread.sleep(30);

                    updateRotation(rotState);
                    Thread.sleep(30);
                }

                if (mModel==YELLOWSTONE) {

                    /*
                    Thread.sleep(30);
                    mTango.getPoseAtTime(0.0, mFramePairs.get(0), mPose); // keeps producing DeadObjectException

                    // Generate the TF message
                    updateYSTranslation(mPose);
                    Thread.sleep(30);

                    updateYSRotation(mPose);
                    Thread.sleep(30);
                    */

                    //mTango.getPoseAtTime(0.0, mFramePairs.get(0), mPose);
                    //System.out.println("getPoseAtTime can read pose data");

                    //mTangoUpdateListener.onPoseAvailable(mPose);
                    //mTangoUpdateListener.onXyzIjAvailable(mTangoXyzIjData);

                    // This is silly.  TangoXyzIjData has perfectly good methods for this, the problem
                    // is that there's no depth data being stuck in there.  There doesn't seem to be
                    // an equivalent of Tango.getPoseAtTime() for depth data, so I have to use the
                    // OnTangoUpdateListener thing.  Which isn't working.  Joy.
                    //ParcelFileDescriptor xyz = mTangoXyzIjData.xyzParcelFileDescriptor;
                    //FileDescriptor f = mTangoXyzIjData.xyzParcelFileDescriptor.getFileDescriptor();
                    //FileInputStream fs = new FileInputStream(mTangoXyzIjData.xyzParcelFileDescriptor.getFileDescriptor());
                    //fs.read(xyz, 0, mTangoXyzIjData.getXyzBuffer().limit());

                    //mTango.connectListener(mFramePairs, mTangoUpdateListener);

                    //updateYSDepth(mTangoXyzIjData);
                    //Thread.sleep(30); // not sure about this bit
                }

                //mTangoOdomPublisher.publishOdom(); // I guess for Peanut these should still happen down here.
                //mTangoPosePublisher.publishPose(); // FIXME:  Make sure this is still Peanut-compatible for vio-only
                //mTangoTfPublisher.publishTransforms();
            }
        });
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
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

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
        //System.out.println("Translation updated");
        mTangoOdomPublisher.setPosePoint(pose.translation[0],pose.translation[1], pose.translation[2]);
        mTangoPosePublisher.setPoint(pose.translation[0],pose.translation[1], pose.translation[2]);
        mTangoTfPublisher.setTranslation(pose.translation[0],pose.translation[1], pose.translation[2]);
    }

    private void updateYSRotation(TangoPoseData pose) {
        //System.out.println("Rotation updated");
        mTangoOdomPublisher.setPoseQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoPosePublisher.setQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoTfPublisher.setRotation(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
    }

    private void updateYSDepth(TangoXyzIjData xyzIj) {
        System.out.println("Processing depth data");
        /*try {
            FloatBuffer f = xyzIj.getXyzBuffer();
        } catch (NullPointerException e) {
            System.out.println("sorry about that");
            return;
        }*/

        int[] data = new int[xyzIj.getXyzBuffer().limit()];
        //data[0] = (int)(xyzIj.getXyzBuffer().get(0));
        for (int i = 0; i < xyzIj.getXyzBuffer().limit(); i++) {
            float x = xyzIj.getXyzBuffer().get(i);//array()[i];//get();
            if (x != 0.0) {
                System.out.println(x);
            }
            data[i] = (int)x;//yzIj.getXyzBuffer().get();
        }


        mDepthPublisher.onNewRawImage(data, xyzIj.ijCols, xyzIj.ijRows);
    }
}