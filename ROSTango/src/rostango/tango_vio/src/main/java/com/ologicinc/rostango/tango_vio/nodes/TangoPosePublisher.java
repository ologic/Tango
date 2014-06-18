package com.ologicinc.rostango.tango_vio.nodes;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

import geometry_msgs.Point;
import geometry_msgs.Quaternion;

/**
 * Created by Rohan Agrawal on 6/18/14.
 */
public class TangoPosePublisher implements NodeMain{
    private Quaternion mQuat;
    private Point mPoint;

    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_pose_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {

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
