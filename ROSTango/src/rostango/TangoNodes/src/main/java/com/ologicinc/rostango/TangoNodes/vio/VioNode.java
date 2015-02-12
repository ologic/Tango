/*
 * Copyright (C) 2014 OLogic, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.ologicinc.rostango.TangoNodes.vio;

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

public class VioNode implements NodeMain {

    private static String TAG = VioNode.class.getSimpleName();

    private VinsServiceHelper mVinsServiceHelper;

    private Tango mTango;
    private TangoConfig mConfig;
    private ArrayList<TangoCoordinateFramePair> mFramePairs;

    private TangoOdomPublisher mTangoOdomPublisher;
    private TangoPosePublisher mTangoPosePublisher;
    private TangoTfPublisher mTangoTfPublisher;

    public static final int PEANUT = 0;
    public static final int YELLOWSTONE = 1;

    private int mModel;
    private boolean isStarted = false;


    public VioNode(Context context) {

        Log.i(TAG, "Build.MODEL="+android.os.Build.MODEL);

        //if (android.os.Build.MODEL.equals("Yellowstone")) {
        if (android.os.Build.MODEL.startsWith("Project Tango Tablet Development Kit")) {
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
        mConfig=mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORECOVERY, true);

        // Lock configuration and connect to Tango
        //mTango.lockConfig();
        mTango.connect(mConfig);

        // Select coordinate frame pairs
        mFramePairs = new ArrayList<TangoCoordinateFramePair>();
        mFramePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE)
        );

        mTango.connectListener(mFramePairs, new OnTangoUpdateListener() {

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

            }

            @Override
            public void onTangoEvent(TangoEvent arg0) {

            }
        });
    }

    @Override
    public void onStart(ConnectedNode node) {


        mTangoOdomPublisher = new TangoOdomPublisher(node);
        mTangoPosePublisher = new TangoPosePublisher(node);
        mTangoTfPublisher = new TangoTfPublisher(node);

        if (mModel==YELLOWSTONE) {
            startYellowstone();
        }


        if (mModel == PEANUT) { // For Yellowstone, OnTangoUpdateListener updates vio data
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
        isStarted =true;
    }

    @Override
    public void onShutdown(Node node) {
        if (mModel==PEANUT) {
            mVinsServiceHelper.shutdown();
        }

        if (mModel==YELLOWSTONE) {
            //mTango.unlockConfig();
            if (isStarted) {
                mTango.disconnect();
                isStarted = false;
            }
        }
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    public void onPause() {
        //mTango.unlockConfig();

        if (isStarted || mModel ==PEANUT) {
            mTango.disconnect();
        }
    }

    public void onResume() {
        //mTango.lockConfig();

        if (isStarted || mModel ==PEANUT) {
            mTango.connect(mConfig);
        }
    }

    public void onDestroy() {
       //mTango.unlockConfig();
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
        mTangoOdomPublisher.setPosePoint(pose.translation[0],pose.translation[1], pose.translation[2]);
        mTangoPosePublisher.setPoint(pose.translation[0],pose.translation[1], pose.translation[2]);
        mTangoTfPublisher.setTranslation(pose.translation[0],pose.translation[1], pose.translation[2]);
    }

    private void updateYSRotation(TangoPoseData pose) {
        mTangoOdomPublisher.setPoseQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoPosePublisher.setQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoTfPublisher.setRotation(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
    }

    public Tango getTango() {
        return mTango;
    }
}
