package com.ologicinc.rostango.tango_vio;

import android.os.Bundle;

import com.motorola.atap.androidvioservice.VinsServiceHelper;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.tf2_ros.StaticTransformBroadcaster;
import org.ros.tf2_ros.TransformBroadcaster;

import geometry_msgs.TransformStamped;

public class TangoVio extends RosActivity
{
    // Set up Transform Broadcasters
    private final TransformBroadcaster mTB = new TransformBroadcaster();
    private final StaticTransformBroadcaster mSTB = new StaticTransformBroadcaster();

    // Set up messages
    private TransformStamped tfs = mTB.newMessage();

    // Set up VIO format
    int coordinateConvention = VinsServiceHelper.STATEFORMAT_UNITY;

    public TangoVio() {
        super("TangoVio", "TangoVio");
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {

        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(mTB, nodeConfiguration);
        nodeMainExecutor.execute(mSTB, nodeConfiguration);

       }
}
