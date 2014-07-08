package com.ologicinc.rostango.tango_vio;

import android.os.Bundle;

import com.motorola.atap.androidvioservice.VinsServiceHelper;
import com.ologicinc.rostango.TangoNodes.vio.VioNode;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class TangoVio extends RosActivity {
   private VinsServiceHelper  mVinsServiceHelper;

   public TangoVio() {
        super("TangoVio", "TangoVio");
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        /* Read VIO data from VINs service helper */
        mVinsServiceHelper = new VinsServiceHelper(this);

        setContentView(R.layout.main);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        VioNode vioNode = new VioNode(mVinsServiceHelper);

        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(vioNode, nodeConfiguration);
    }

}
