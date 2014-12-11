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

package com.ologicinc.rostango.tango_serial;

import android.os.Bundle;
import android.content.Context;
import android.content.Intent;
import android.util.Log;
import android.view.WindowManager;
import android.widget.TextView;
// import android.widget.Toast;

import com.MAVLink.Messages.MAVLinkPacket;
import com.MAVLink.Messages.ardupilotmega.msg_set_gps_global_origin;
import com.MAVLink.Messages.ardupilotmega.msg_vision_position_estimate;
import com.github.ologic.android_ologic.usbserial.driver.UsbSerialDriver;
import com.motorola.atap.androidvioservice.VinsServiceHelper;
import com.ologicinc.rostango.TangoNodes.vio.VioNode;

import org.ros.address.Address;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.NodeMainExecutor;


public class TangoSerial extends RosActivity {
    private VinsServiceHelper  mVinsServiceHelper;

    // USB serial communication.
    // Driver instance, passed in statically via show(Context, UsbSerialDriver).
    // It'd be cleaner to re-create the driver using
    // arguments passed in with the startActivity(Intent) intent.
    // But this works for now.
    private static UsbSerialDriver mSerialDriver = null;

    private static final boolean DEBUG = true;
    private static final String TAG = TangoSerial.class.getSimpleName();
    private TextView mStatsView;


    public TangoSerial() {
        super("TangoSerial", "TangoSerial");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        /* Read VIO data from VINs service helper */
        mVinsServiceHelper = new VinsServiceHelper(this);
        setContentView(R.layout.main);
        mStatsView = (TextView) findViewById(R.id.stats);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        VioNode vioNode = new VioNode(this);
        vioNode.setVinsServiceHelper(mVinsServiceHelper);
        VioListenerNode vioListenerNode = new VioListenerNode(mSerialDriver, mStatsView, vioNode.getModel());
        NodeConfiguration nodeConfiguration;

        if (getMasterUri().getHost().equals(Address.LOOPBACK)) {
            nodeConfiguration = NodeConfiguration.newPrivate();
        } else {
            nodeConfiguration =
                   NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        }

        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(vioNode, nodeConfiguration);
        nodeMainExecutor.execute(vioListenerNode, nodeConfiguration);
    }




    // Starts the activity, using the supplied node executor and driver instance.
    static public void show(Context context, UsbSerialDriver driver) {
        // Set our static variables
        mSerialDriver = driver;

        // Show the activity
        final Intent intent = new Intent(context, TangoSerial.class);
        intent.addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP);
        context.startActivity(intent);
    }

}
