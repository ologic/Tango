/*
 * Copyright (C) 2014 OLogic,  Inc.
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

package com.ologicinc.rostango.tango_vio;

import android.os.Bundle;
import android.widget.TextView;

import com.google.atap.tangoservice.Tango;


import com.motorola.atap.androidvioservice.VinsServiceHelper;
import com.ologicinc.rostango.TangoNodes.vio.VioDepthNode;

import org.ros.address.InetAddressFactory;
import org.ros.android.TangoActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class TangoVio extends TangoActivity {
    private VinsServiceHelper  mVinsServiceHelper;
    //private VioNode mVioNode;

    private VioDepthNode mVioNode; // To publish vio only, use VioNode instead.

    private TextView mTxtInfo;

    public TangoVio() {
        super("TangoVio", "TangoVio");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        /* Read VIO data from VINs service helper */
        //mVinsServiceHelper = new VinsServiceHelper(this);

        setContentView(R.layout.main);
        mTxtInfo = (TextView)findViewById(R.id.txtInfo);

    }

    @Override
    protected void onStart() {
        super.onStart();
        getPermissions();



    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration;
        // XXX VioNode vioNode = new VioNode(mVinsServiceHelper);
        //mVioNode = new VioNode(this);
        mVioNode = new VioDepthNode(this); // To publish vio only, use VioNode instead.


        nodeConfiguration =  NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
       // nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(mVioNode, nodeConfiguration);
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (mVioNode != null) {
            mVioNode.onPause();
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (mVioNode != null) {
            mVioNode.onResume();
        }

    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (mVioNode != null) {
            mVioNode.onDestroy();
        }
    }

    private void getPermissions() {

        startActivityForResult(Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),  Tango.TANGO_INTENT_ACTIVITYCODE);


    }


/*
    private void getSensorLocations() {

        TangoPoseData cToIMUPose;
        TangoCoordinateFramePair cToIMUPair;

        cToIMUPair.baseFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        cToIMUPair.targetFrame = TangoPoseData.COORDINATE_FRAME_DISPLAY;

        Tango tango = mVioNode.etTango();

        tango.getPose(0.0, cToIMUPair, &cToIMUPose);

        cToIMU_position = ToVector(cToIMUPose.translation[0],
                cToIMUPose.translation[1],
                cToIMUPose.translation[2]);
        cToIMU_rotation = ToQuaternion(cToIMUPose.orientation[3],
                cToIMUPose.orientation[0],
                cToIMUPose.orientation[1],
                cToIMUPose.orientation[2]);

    }
*/
}
