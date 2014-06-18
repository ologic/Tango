package com.ologicinc.rostango.tango_vio.nodes;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeMain;
import org.ros.node.Node;


/**
 * Created by brandonb on 6/18/14.
 */
public class VioTf2Publisher implements NodeMain {

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("vio_tf2_publisher");
    }

    @Override
    public void onStart(ConnectedNode node) {

    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

}
