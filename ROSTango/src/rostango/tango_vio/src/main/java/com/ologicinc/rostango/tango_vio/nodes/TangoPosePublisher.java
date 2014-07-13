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
