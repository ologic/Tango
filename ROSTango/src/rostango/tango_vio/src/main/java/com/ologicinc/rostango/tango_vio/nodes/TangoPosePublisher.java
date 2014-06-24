package com.ologicinc.rostango.tango_vio.nodes;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;

/**
 * Created by Rohan Agrawal on 6/18/14.
 */
public class TangoPosePublisher implements NodeMain{
    private PoseStamped pose;
    private Publisher<geometry_msgs.PoseStamped> publisher;

    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_pose_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        pose = connectedNode.getTopicMessageFactory().newFromType(geometry_msgs.PoseStamped._TYPE);

        publisher = connectedNode.newPublisher("/tango_pose", geometry_msgs.PoseStamped._TYPE);
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    public void setQuat(double x, double y, double z, double w) {
        Quaternion q = pose.getPose().getOrientation();
        q.setW(w);
        q.setX(x);
        q.setY(y);
        q.setZ(z);
        pose.getPose().setOrientation(q);
    }

    public void setPoint(double x, double y, double z) {
        Point p = pose.getPose().getPosition();
        p.setX(x);
        p.setY(y);
        p.setZ(z);
        pose.getPose().setPosition(p);
    }

    public void publishPose(){
        long lt = System.currentTimeMillis();
        Time t = new Time((int) (lt / 1e3), (int) ((lt % 1e3) * 1e6));
        pose.getHeader().setStamp(t);
        pose.getHeader().setFrameId("/global");
        publisher.publish(pose);
    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

}
