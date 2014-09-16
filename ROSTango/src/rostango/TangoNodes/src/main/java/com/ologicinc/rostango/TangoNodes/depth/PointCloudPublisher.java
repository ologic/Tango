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

import android.graphics.Rect;
import android.graphics.YuvImage;

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
 * Publishes preview frames.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */
public class PointCloudPublisher {

    private final ConnectedNode connectedNode;
    //private final Publisher<sensor_msgs.Image> imagePublisher;
    //private final Publisher<sensor_msgs.CameraInfo> cameraInfoPublisher;
    private final Publisher<sensor_msgs.PointCloud2> pointCloud2Publisher;
    private final Publisher<sensor_msgs.PointField> pointFieldPublisher;

    //private byte[] rawImageBuffer;
    //private int rawImageSize;
    //private YuvImage yuvImage;
    //private Rect rect;
    private ChannelBufferOutputStream stream;

    //private double[] D = {0.2104473, -0.5854902, 0.4575633, 0.0, 0.0};
    //private double[] K = {237.0, 0.0, 160.0, 0.0, 237.0, 90.0, 0.0, 0.0, 1.0};
    //private double[] R = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //private double[] P = {237.0, 0.0, 160.0, 0.0, 0.0, 237.0, 90.0, 0.0, 0.0, 0.0, 1.0, 0.0};

    public PointCloudPublisher(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera/depth");
        //imagePublisher = connectedNode.newPublisher(resolver.resolve("image_raw"), sensor_msgs.Image._TYPE);
        //cameraInfoPublisher = connectedNode.newPublisher(resolver.resolve("camera_info"), sensor_msgs.CameraInfo._TYPE);
        pointCloud2Publisher = connectedNode.newPublisher(resolver.resolve("points"), sensor_msgs.PointCloud2._TYPE);
        pointFieldPublisher = connectedNode.newPublisher(resolver.resolve("point_field"), sensor_msgs.PointField._TYPE);
        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    public void onNewPointCloud(float[] data) {//, int length) {//, int width, int height) {
        Preconditions.checkNotNull(data);
        //Preconditions.checkNotNull(length);
        //Preconditions.checkNotNull(width);
        //Preconditions.checkNotNull(height);

        Time currentTime = connectedNode.getCurrentTime();
        String frameId = "phone";//"camera"; //_optical"; // suffix _optical should specify correct coordinate system?? but rviz can't find transform

        sensor_msgs.PointCloud2 pointCloud2 = pointCloud2Publisher.newMessage();
        pointCloud2.getHeader().setStamp(currentTime);
        pointCloud2.getHeader().setFrameId(frameId);

        List<PointField> f = pointCloud2.getFields();
        sensor_msgs.PointField xField = pointFieldPublisher.newMessage();
        xField.setName("x");
        xField.setDatatype(PointField.FLOAT32);
        xField.setCount(1);
        f.add(xField);
        sensor_msgs.PointField yField = pointFieldPublisher.newMessage();
        yField.setName("y");
        yField.setDatatype(PointField.FLOAT32);
        yField.setCount(1);
        yField.setOffset(4);
        f.add(yField);
        sensor_msgs.PointField zField = pointFieldPublisher.newMessage();
        zField.setName("z");
        zField.setDatatype(PointField.FLOAT32);
        zField.setCount(1);
        zField.setOffset(8);
        f.add(zField);

        for (int i = 0; i < data.length; i++) {
            try{
                //int bits = Float.floatToIntBits(data[i]);
                //stream.writeByte((byte)(bits & 0xff));
                //stream.writeByte((byte)((bits >> 8) & 0xff));
                //stream.writeByte((byte)((bits >> 16) & 0xff));
                //stream.writeByte((byte)((bits >> 24) & 0xff));

                byte[] bytes = ByteBuffer.allocate(4).putFloat(data[i]).array();
                for (int b = 0; b < 4; b++) {
                    stream.writeByte(bytes[b]);
                }
            } catch (Exception e) {}
        }

        // set data in pointCloud2
        pointCloud2.setWidth(data.length); // not sure of pointcloud ordering, so will assume it's unordered
        pointCloud2.setHeight(1);
        pointCloud2.setPointStep(4);
        pointCloud2.setRowStep(4 * (data.length));
        //pointCloud2.setIsBigendian(java.nio.ByteOrder.nativeOrder().equals(ByteOrder.BIG_ENDIAN)); // not sure if necessary
        pointCloud2.setData(stream.buffer().copy());
        stream.buffer().clear();

        pointCloud2Publisher.publish(pointCloud2);

        /*
        sensor_msgs.Image image = imagePublisher.newMessage();
        image.getHeader().setStamp(currentTime);
        image.getHeader().setFrameId(frameId);

        for (int i = 0; i < data.length; i++){
            try {
                stream.writeShort((short)data[i]);
            }
            catch (Exception e){

            }
        }
        image.setStep(360);
        image.setWidth(width);
        image.setHeight(height);
        image.setEncoding("16UC1");
        image.setData(stream.buffer().copy());
        stream.buffer().clear();

        imagePublisher.publish(image);

        sensor_msgs.CameraInfo cameraInfo = cameraInfoPublisher.newMessage();
        cameraInfo.getHeader().setStamp(currentTime);
        cameraInfo.getHeader().setFrameId(frameId);

        cameraInfo.setDistortionModel("plumb_bob");
        cameraInfo.setD(D);
        cameraInfo.setK(K);
        cameraInfo.setR(R);
        cameraInfo.setP(P);

        cameraInfo.setWidth(width);
        cameraInfo.setHeight(height);
        cameraInfoPublisher.publish(cameraInfo);
        */
    }
}