/*
 * Copyright (C) 2014 OLogic, Inc.
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

package com.ologicinc.rostango.TangoNodes.vio;

import org.ros.message.Time;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.Point;
import geometry_msgs.PoseWithCovariance;
import geometry_msgs.Quaternion;
import geometry_msgs.TwistWithCovariance;
import nav_msgs.Odometry;

/**
 * Created by rohan on 6/23/14.
 */
class TangoOdomPublisher{
    private PoseWithCovariance pose;
    private PoseWithCovariance last_pose;
    TwistWithCovariance twist;
    private Odometry odom;
    private Publisher<Odometry> publisher;
    private long last_time;

    public TangoOdomPublisher(ConnectedNode connectedNode){
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

}