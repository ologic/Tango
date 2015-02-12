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

package com.ologicinc.rostango.TangoNodes.depth;

import com.google.common.base.Preconditions;

import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import sensor_msgs.PointCloud2;
import sensor_msgs.PointField;

import java.nio.FloatBuffer;
import java.nio.ByteBuffer;

/**
 * Publishes Tango depth data as sensor_msgs/PointCloud2 to the topic /camera/depth/points.  
 *
 * Created by Shiloh Curtis on 9/16/2014
 */

public class PointCloudPublisher {

    private final ConnectedNode connectedNode;
    private final Publisher<sensor_msgs.PointCloud2> pointCloud2Publisher;
    private final Publisher<sensor_msgs.PointField> pointFieldPublisher;
    private ChannelBufferOutputStream stream;
    sensor_msgs.PointCloud2 pointCloud2;

    public PointCloudPublisher(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera/depth");
        pointCloud2Publisher = connectedNode.newPublisher(resolver.resolve("points"),
                sensor_msgs.PointCloud2._TYPE);
        // PointFields are not published separately from the PointCloud2 message, but the
        // easiest way to generate them is with a publisher that never publishes the message.
        pointFieldPublisher = connectedNode.newPublisher(resolver.resolve("point_field"),
                sensor_msgs.PointField._TYPE);

        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
        pointCloud2 = pointCloud2Publisher.newMessage();
        setPointFields(pointCloud2); // Sets up PointCloud2 with float32 fields for x, y, and z
    }

    private void setPointFields(PointCloud2 pc) {
        sensor_msgs.PointField xField = pointFieldPublisher.newMessage();
        xField.setName("x");
        xField.setDatatype(PointField.FLOAT32);
        xField.setCount(1);
        xField.setOffset(0);
        pc.getFields().add(xField);
        sensor_msgs.PointField yField = pointFieldPublisher.newMessage();
        yField.setName("y");
        yField.setDatatype(PointField.FLOAT32);
        yField.setCount(1);
        yField.setOffset(4);
        pc.getFields().add(yField);
        sensor_msgs.PointField zField = pointFieldPublisher.newMessage();
        zField.setName("z");
        zField.setDatatype(PointField.FLOAT32);
        zField.setCount(1);
        zField.setOffset(8);
        pc.getFields().add(zField);
    }

    public void onNewPointCloud(byte[] data) {
        Preconditions.checkNotNull(data);

        Time currentTime = connectedNode.getCurrentTime();
        String frameId = "phone";


        pointCloud2.getHeader().setStamp(currentTime);
        pointCloud2.getHeader().setFrameId(frameId);



        /*
        for (int i = 0; i < data.length; i++) {
            try{
                byte[] bytes;
                if (i % 3 == 0) {
                    bytes = ByteBuffer.allocate(4).putFloat(data[i]).array();
                } else {
                    bytes = ByteBuffer.allocate(4).putFloat(-1 * data[i]).array();
                }
                for (int b = 3; b >= 0; b--) { // Floats must be in little-endian form
                    stream.writeByte(bytes[b]);
                }
            } catch (Exception e) {}
        }
        */

        // Set PointCloud2 data
        pointCloud2.setWidth((data.length) / 12);
        pointCloud2.setHeight(1);
        pointCloud2.setPointStep(12);
        pointCloud2.setIsDense(true);

        pointCloud2.setRowStep((pointCloud2.getPointStep()) * (pointCloud2.getWidth()));

        // Even with is_bigendian set to true, rviz won't correctly display big-endian xyz values.
        pointCloud2.setIsBigendian(false);

        stream.buffer().clear();
        try {
            stream.write(data);
        }
        catch(Exception ex) {

        }



        pointCloud2.setData((ChannelBuffer)stream.buffer());


        pointCloud2Publisher.publish(pointCloud2);
    }
}