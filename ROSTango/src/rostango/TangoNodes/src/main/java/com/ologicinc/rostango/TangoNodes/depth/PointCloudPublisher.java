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

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import java.nio.ByteOrder;
import java.nio.ByteBuffer;
import java.util.List;

import sensor_msgs.PointField;

/**
 * Publishes Tango depth data as sensor_msgs/PointCloud2 to /camera/depth/points
 *
 * Created by Shiloh Curtis on 9/16/2014
 */
public class PointCloudPublisher {

    private final ConnectedNode connectedNode;
    private final Publisher<sensor_msgs.PointCloud2> pointCloud2Publisher;
    private final Publisher<sensor_msgs.PointField> pointFieldPublisher;
    private ChannelBufferOutputStream stream;

    public PointCloudPublisher(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera/depth");
        pointCloud2Publisher = connectedNode.newPublisher(resolver.resolve("points"), sensor_msgs.PointCloud2._TYPE);
        pointFieldPublisher = connectedNode.newPublisher(resolver.resolve("point_field"), sensor_msgs.PointField._TYPE);
        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    public void onNewPointCloud(float[] data) {
        Preconditions.checkNotNull(data);

        Time currentTime = connectedNode.getCurrentTime();
        String frameId = "phone";

        sensor_msgs.PointCloud2 pointCloud2 = pointCloud2Publisher.newMessage();
        pointCloud2.getHeader().setStamp(currentTime);
        pointCloud2.getHeader().setFrameId(frameId);

        //List<PointField> f = pointCloud2.getFields();
        sensor_msgs.PointField xField = pointFieldPublisher.newMessage();
        xField.setName("x");
        xField.setDatatype(PointField.FLOAT32);
        xField.setCount(1);
        xField.setOffset(0);
        pointCloud2.getFields().add(xField);
        sensor_msgs.PointField yField = pointFieldPublisher.newMessage();
        yField.setName("y");
        yField.setDatatype(PointField.FLOAT32);
        yField.setCount(1);
        yField.setOffset(4);
        pointCloud2.getFields().add(yField);
        sensor_msgs.PointField zField = pointFieldPublisher.newMessage();
        zField.setName("z");
        zField.setDatatype(PointField.FLOAT32);
        zField.setCount(1);
        zField.setOffset(8);
        pointCloud2.getFields().add(zField);

        for (int i = 0; i < 3; i++) {
            //System.out.println(pointCloud2.getFields().get(i).getName());
        }

        for (int i = 0; i < data.length; i++) {
            try{
                byte[] bytes = ByteBuffer.allocate(4).putFloat(-1*data[i]).array();
                //int bits = Float.floatToIntBits(data[i]);
                //byte[] bytes = new byte[4];
                //bytes[0] = (byte)(bits & 0xff);
                //bytes[1] = (byte)((bits >> 8) & 0xff);
                //bytes[2] = (byte)((bits >> 16) & 0xff);
                //bytes[3] = (byte)((bits >> 24) & 0xff);
                //float f = ByteBuffer.wrap(bytes).getFloat();
                //if (f != data[i]) {
                    //System.out.println("Float -> byte conversion failed!");
                    //System.out.println(f);
                    //System.out.println(data[i]);
                //}
                for (int b = 3; b >= 0; b--) {
                    stream.writeByte(bytes[b]);
                }
            } catch (Exception e) {}
        }

        // set data in pointCloud2
        pointCloud2.setWidth((data.length) / 3); // Tango pointcloud data doesn't seem to be ordered
        pointCloud2.setHeight(1);
        pointCloud2.setPointStep(12);
        pointCloud2.setRowStep((pointCloud2.getPointStep()) * (pointCloud2.getWidth()));
        //pointCloud2.setIsBigendian(java.nio.ByteOrder.nativeOrder().equals(ByteOrder.BIG_ENDIAN)); // not sure if necessary
        //pointCloud2.setIsDense(true);
        pointCloud2.setIsBigendian(false);
        pointCloud2.setData(stream.buffer().copy());
        byte[] bytes = new byte[4];
        for (int i = 0; i < data.length; i++) {
            for (int b = 0; b < 4; b++) {
                bytes[b] = stream.buffer().getByte(i*4 + b);
            }
            System.out.println(ByteBuffer.wrap(bytes).getFloat());
        }
        stream.buffer().clear();

        pointCloud2Publisher.publish(pointCloud2);
    }
}