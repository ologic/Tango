/*
 * Copyright 2013 Motorola Mobility LLC. Part of ProjectTango.
 * CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
 *
 */

package com.ologicinc.rostango.tango_depthimage;

import android.os.Bundle;
import android.view.Window;
import android.view.WindowManager;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

import com.ologicinc.rostango.TangoNodes.depth.*;

/**
 * DepthScanActivity is the launch activity of the PeanutDepthScan app.
 * PeanutDepthScan is a real-time camera app that scans objects by drawing a
 * 'Depth Line' over the top of them. The 'Depth Line' is based on the actual
 * depth of a pixel, this depth data comes from the Peanut's depth buffer. Upon
 * launch, the app starts a camera object and sets a surface for it to draw on.
 * The app then reads the depth buffer during the camera preview. It then draws
 * a 'false color' overlay on top the camera image, the color is based on the
 * depth buffer.
 * Note: The depth buffer is a 2 dimensional array of ints that corresponds to
 * the images the camera captures.
 *
 * @author Michael F. Winter (robotmikew@gmail.com)
 */

public class DepthScanActivity extends RosActivity {
  RosDepthViewSurface mDepthView = null;  //Surface for drawing on.

    public DepthScanActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("Pubsub Tutorial", "Pubsub Tutorial");
    }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    // Set window to full screen view with no title.
    getWindow().setFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN,
        WindowManager.LayoutParams.FLAG_FULLSCREEN);
    requestWindowFeature(Window.FEATURE_NO_TITLE);
    setContentView(R.layout.layoutdepth);
    mDepthView = (RosDepthViewSurface) findViewById(R.id.depthview);
  }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeMainExecutor.execute(mDepthView, nodeConfiguration);
    }

  @Override
  protected void onResume() {
    mDepthView.startCamera();
    super.onResume();
  }

  @Override
  protected void onPause() {
    mDepthView.stopCamera();
    super.onPause();
  }
}
