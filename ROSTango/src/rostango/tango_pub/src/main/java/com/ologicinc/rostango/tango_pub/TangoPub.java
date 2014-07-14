/*
 * Copyright 2013 Motorola Mobility LLC. Part of ProjectTango.
 * CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
 *
 */

package com.ologicinc.rostango.tango_pub;

import android.os.Bundle;
import android.os.ParcelFileDescriptor;
import android.util.Log;

import com.motorola.atap.androidvioservice.VinsServiceHelper;
import com.ologicinc.rostango.TangoNodes.depth.RosDepthViewSurface;

import org.ros.android.RosActivity;
import org.ros.node.NodeMainExecutor;

import java.io.FileDescriptor;
import java.io.FileInputStream;
import java.io.IOException;


public class TangoPub extends RosActivity {
    RosDepthViewSurface mDepthView = null;  //Surface for drawing on.
    private VinsServiceHelper  mVinsServiceHelper;

    public TangoPub() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("Tango Pub", "Tango Pub");
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        mVinsServiceHelper = new VinsServiceHelper(this);

        // Set window to full screen view with no title.
//        getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
//                WindowManager.LayoutParams.FLAG_FULLSCREEN);
//        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.main);
//        mDepthView = (RosDepthViewSurface) findViewById(R.id.depthview);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
//        VioNode vioNode = new VioNode(mVinsServiceHelper);
//        NodeConfiguration nodeConfiguration =
//                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
//        nodeConfiguration.setMasterUri(getMasterUri());

        ParcelFileDescriptor spp = mVinsServiceHelper.getCurrentSuperFrame();
        FileDescriptor spfd = spp.getFileDescriptor();
        FileInputStream spfis  = new FileInputStream(spfd);
        byte buffer[] = null;
        try {
            Log.i("Superframe", Integer.toString(spfis.read(buffer)));
            Log.i("Superframe", buffer.toString());
        } catch (IOException e) {
            e.printStackTrace();
        }

        Log.i("Superframe", spp.toString());
        Log.i("Superframe", spfd.toString());

//        nodeMainExecutor.execute(mDepthView, nodeConfiguration);
//        nodeMainExecutor.execute(vioNode, nodeConfiguration);
    }

//    @Override
//    protected void onResume() {
//        mDepthView.startCamera();
//        super.onResume();
//    }
//
//    @Override
//    protected void onPause() {
//        mDepthView.stopCamera();
//        super.onPause();
//    }
}
