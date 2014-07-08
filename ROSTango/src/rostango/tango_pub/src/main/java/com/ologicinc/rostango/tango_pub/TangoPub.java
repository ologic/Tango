package com.ologicinc.rostango.tango_pub;

import android.os.Bundle;

import com.ologicinc.rostango.tango_depth.RosDepthViewSurface;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class TangoPub extends RosActivity {
    private RosDepthViewSurface mDepthView;

    public TangoPub() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("TangoRosPublisher", "TangoRosPublisher");
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.layoutdepth);
        mDepthView = (RosDepthViewSurface) findViewById(R.id.depthview);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());
    }
}
