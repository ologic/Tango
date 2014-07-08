package com.ologicinc.rostango.TangoNodes.vio;

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;
import geometry_msgs.PoseStamped;
import geometry_msgs.Quaternion;

/**
 * Created by Rohan Agrawal on 6/18/14.
 */
class TangoPosePublisher{
    private PoseStamped pose;
    private Publisher<PoseStamped> publisher;

    public TangoPosePublisher(ConnectedNode connectedNode){
        pose = connectedNode.getTopicMessageFactory().newFromType(PoseStamped._TYPE);
        publisher = connectedNode.newPublisher("/tango_pose", PoseStamped._TYPE);
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

}
