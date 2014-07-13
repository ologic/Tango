
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

package com.ologicinc.rostango.tango_depthimage;

import android.graphics.Rect;
import android.graphics.YuvImage;

import com.google.common.base.Preconditions;

import org.jboss.netty.buffer.ChannelBufferOutputStream;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.Time;
import org.ros.namespace.NameResolver;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

/**
 * Publishes preview frames.
 *
 * @author damonkohler@google.com (Damon Kohler)
 */
class DepthPublisher implements RawImageListener {

    private final ConnectedNode connectedNode;
    private final Publisher<sensor_msgs.Image> imagePublisher;
    private final Publisher<sensor_msgs.CameraInfo> cameraInfoPublisher;

    private byte[] rawImageBuffer;
    private int rawImageSize;
    private YuvImage yuvImage;
    private Rect rect;
    private ChannelBufferOutputStream stream;

    private double[] D = {0.2104473, -0.5854902, 0.4575633, 0.0, 0.0};
    private double[] K = {237.0, 0.0, 160.0, 0.0, 237.0, 90.0, 0.0, 0.0, 1.0};
    private double[] R = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    private double[] P = {237.0, 0.0, 160.0, 0.0, 0.0, 237.0, 90.0, 0.0, 0.0, 0.0, 1.0, 0.0};

    public DepthPublisher(ConnectedNode connectedNode) {
        this.connectedNode = connectedNode;
        NameResolver resolver = connectedNode.getResolver().newChild("camera/depth");
        imagePublisher =
                connectedNode.newPublisher(resolver.resolve("image_raw"),
                        sensor_msgs.Image._TYPE);
        cameraInfoPublisher =
                connectedNode.newPublisher(resolver.resolve("camera_info"), sensor_msgs.CameraInfo._TYPE);
        stream = new ChannelBufferOutputStream(MessageBuffers.dynamicBuffer());
    }

    @Override
    public void onNewRawImage(int[] data, int width, int height) {
        Preconditions.checkNotNull(data);
        Preconditions.checkNotNull(width);
        Preconditions.checkNotNull(height);

        Time currentTime = connectedNode.getCurrentTime();
        String frameId = "camera";

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
    }
}
