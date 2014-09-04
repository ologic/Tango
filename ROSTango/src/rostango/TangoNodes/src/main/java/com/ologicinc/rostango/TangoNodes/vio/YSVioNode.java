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

import android.util.Log;

import com.motorola.atap.androidvioservice.VinsServiceHelper;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoCoordinateFramePair;

import java.util.ArrayList;


/**
 * Created by brandonb on 9/3/14.
 */
public class YSVioNode implements NodeMain {

    // XXX private VinsServiceHelper mVinsServiceHelper;

    private TangoOdomPublisher mTangoOdomPublisher;
    private TangoPosePublisher mTangoPosePublisher;
    private TangoTfPublisher mTangoTfPublisher;

    private Tango mTango;
    private TangoConfig mConfig;

    public YSVioNode(Tango tango){
        mTango = tango;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ys_vio");
    }

    @Override
    public void onStart(ConnectedNode node) {

        // Create a new Tango Configuration and enable the MotionTracking API
        mConfig = new TangoConfig();
        mTango.getConfig(TangoConfig.CONFIG_TYPE_CURRENT, mConfig);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_MOTIONTRACKING, true);
        mConfig.putBoolean(TangoConfig.KEY_BOOLEAN_AUTORESET, false);

        // Lock configuration and connect to Tango
        mTango.lockConfig(mConfig);
        mTango.connect();

        mTangoOdomPublisher = new TangoOdomPublisher(node);
        mTangoPosePublisher = new TangoPosePublisher(node);
        mTangoTfPublisher = new TangoTfPublisher(node);

        final TangoPoseData pose = new TangoPoseData();

        // Select coordinate frame pairs
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        node.executeCancellableLoop(new CancellableLoop() {
           @Override
            protected void loop() throws InterruptedException {
               Thread.sleep(30);
               // XXX final double[] posState = mVinsServiceHelper.getStateInFullStateFormat();
               // XXX final double[] rotState = mVinsServiceHelper.getStateInUnityFormat();

               mTango.getPoseAtTime(0.0, framePairs.get(0), pose);

               // Generate the TF message

               updateTranslation(pose);
               Thread.sleep(30);

               updateRoataion(pose);
               Thread.sleep(30);

               mTangoOdomPublisher.publishOdom();
               mTangoPosePublisher.publishPose();
               mTangoTfPublisher.publishTransforms();
            }
        });
    }


    @Override
    public void onShutdown(Node node) {
        // XXX mVinsServiceHelper.shutdown();
        mTango.unlockConfig();
        mTango.disconnect();
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    public void updateTranslation(TangoPoseData pose) {
        // XXX mTangoOdomPublisher.setPosePoint(state[5],-state[4], state[6]);
        // XXX mTangoPosePublisher.setPoint(state[5],-state[4], state[6]);
        // XXX mTangoTfPublisher.setTranslation(state[5],-state[4], state[6]);

        mTangoOdomPublisher.setPosePoint(-pose.translation[1],pose.translation[0], pose.translation[2]);
        mTangoPosePublisher.setPoint(-pose.translation[1],pose.translation[0], pose.translation[2]);
        mTangoTfPublisher.setTranslation(-pose.translation[1],pose.translation[0], pose.translation[2]);
    }

    public void updateRoataion(TangoPoseData pose) {
        // XXX mTangoOdomPublisher.setPoseQuat(-state[2], state[0], -state[1], state[3]);
        // XXX mTangoPosePublisher.setQuat(-state[2],state[0],-state[1],state[3]);
        // XXXX mTangoTfPublisher.setRotation(-state[2],state[0],-state[1],state[3]);

        mTangoOdomPublisher.setPoseQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoPosePublisher.setQuat(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
        mTangoTfPublisher.setRotation(pose.rotation[0], pose.rotation[1], pose.rotation[2],pose.rotation[3]);
    }
}
