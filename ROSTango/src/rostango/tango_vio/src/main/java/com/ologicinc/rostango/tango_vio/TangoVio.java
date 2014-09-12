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

import com.motorola.atap.androidvioservice.VinsServiceHelper;
import com.ologicinc.rostango.TangoNodes.vio.VioNode;
import com.ologicinc.rostango.TangoNodes.vio.VioDepthNode; // depth

import org.ros.address.Address;
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
        NodeConfiguration nodeConfiguration;
        // XXX VioNode vioNode = new VioNode(mVinsServiceHelper);
        //VioNode vioNode = new VioNode(this);
        VioDepthNode vioNode = new VioDepthNode(this); // Use both vio and depth

        // Required for Peanut support
        vioNode.setVinsServiceHelper(mVinsServiceHelper);

        if (getMasterUri().getHost().equals(Address.LOOPBACK)) {
            nodeConfiguration = NodeConfiguration.newPrivate();
        } else {
            nodeConfiguration =
                    NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        }

        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(vioNode, nodeConfiguration);
    }

}
