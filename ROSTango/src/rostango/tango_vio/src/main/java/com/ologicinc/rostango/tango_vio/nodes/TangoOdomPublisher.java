package com.ologicinc.rostango.tango_vio.nodes;

import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.Quaternion;
import geometry_msgs.TwistWithCovariance;
import nav_msgs.Odometry;

/**
 * Created by rohan on 6/23/14.
 */
public class TangoOdomPublisher implements NodeMain{
    private PoseWithCovariance pose;
    private PoseWithCovariance last_pose;
    TwistWithCovariance twist;
    private Odometry odom;
    private Publisher<Odometry> publisher;
    private long last_time;


    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_odom_publisher");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        odom = connectedNode.getTopicMessageFactory().newFromType(Odometry._TYPE);

        publisher = connectedNode.newPublisher("/odom_visual", Odometry._TYPE);

        pose = odom.getPose();
        last_pose = pose;
        twist = odom.getTwist();

        last_pose.getPose().getPosition().setX(0);
        last_pose.getPose().getPosition().setY(0);
        last_pose.getPose().getPosition().setZ(0);
        last_pose.getPose().getOrientation().setX(0);
        last_pose.getPose().getOrientation().setY(0);
        last_pose.getPose().getOrientation().setZ(0);
        last_pose.getPose().getOrientation().setW(1);

        last_time = System.currentTimeMillis();
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    public void setPoseQuat(double x, double y, double z, double w) {
        Quaternion q = pose.getPose().getOrientation();
        q.setW(w);
        q.setX(x);
        q.setY(y);
        q.setZ(z);
        pose.getPose().setOrientation(q);
    }

    public void setPosePoint(double x, double y, double z) {
        Point p = pose.getPose().getPosition();
        p.setX(x);
        p.setY(y);
        p.setZ(z);
        pose.getPose().setPosition(p);
    }

    public void publishOdom(){
        last_pose = pose;
        long lt = System.currentTimeMillis();
        Time t = new Time((int) (lt / 1e3), (int) ((lt % 1e3) * 1e6));
        odom.getHeader().setStamp(t);
        odom.getHeader().setFrameId("/global");
        odom.setPose(pose);
        odom.setTwist(twist);
        publisher.publish(odom);
    }

    private TwistWithCovariance calculateTwist(){
        double time_delta = (System.currentTimeMillis() - last_time) /1000;
        double x_delta = pose.getPose().getPosition().getX() - last_pose.getPose().getPosition().getX();
        double y_delta = pose.getPose().getPosition().getY() - last_pose.getPose().getPosition().getY();
        double z_delta = pose.getPose().getPosition().getZ() - last_pose.getPose().getPosition().getZ();

        twist.getTwist().getLinear().setX(x_delta/time_delta);
        twist.getTwist().getLinear().setY(y_delta / time_delta);
        twist.getTwist().getLinear().setZ(z_delta / time_delta);
        return twist;
    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }

}