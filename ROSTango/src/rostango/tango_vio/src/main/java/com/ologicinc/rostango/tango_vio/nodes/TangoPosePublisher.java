package com.ologicinc.rostango.tango_vio.nodes;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.Quaternion;

/**
 * Created by Rohan Agrawal on 6/18/14.
 */
public class TangoPosePublisher implements NodeMain{
    private Pose pose;
    private Publisher<geometry_msgs.Pose> publisher;

    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_pose_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        pose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);

        publisher = connectedNode.newPublisher("/tango_pose", geometry_msgs.Pose._TYPE);
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    public void setQuat(double w, double x, double y, double z) {
        Quaternion q = pose.getOrientation();
        q.setW(w);
        q.setX(x);
        q.setY(y);
        q.setZ(z);
        pose.setOrientation(q);
    }

    public void setPoint(double x, double y, double z) {
        Point p = pose.getPosition();
        p.setX(x);
        p.setY(y);
        p.setZ(z);
        pose.setPosition(p);
    }

    public void publishPose(){
        publisher.publish(pose);
    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

}
