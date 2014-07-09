package com.ologicinc.rostango.TangoNodes.vio;

import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import geometry_msgs.Quaternion;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import tf2_msgs.TFMessage;

/**
 * Created by rohan on 7/8/14.
 */
class TangoTfPublisher {
    //tf 0= global >>> unity
    //tf 1= unity >>> phone
    private TFMessage mTfMessage;
    private TransformStamped unity;
    private TransformStamped phone;

    private Publisher<TFMessage> publisher;

    private ConnectedNode node;

    public TangoTfPublisher(ConnectedNode connectedNode){
        mTfMessage = connectedNode.getTopicMessageFactory().newFromType(TFMessage._TYPE);
        unity = connectedNode.getTopicMessageFactory().newFromType(TransformStamped._TYPE);
        phone = connectedNode.getTopicMessageFactory().newFromType(TransformStamped._TYPE);

        mTfMessage.getTransforms().clear();

        publisher = connectedNode.newPublisher("/tf", TFMessage._TYPE);
        this.node = connectedNode;
    }

    public void setRotation(double x, double y, double z, double w) {
        Quaternion q = phone.getTransform().getRotation();
        q.setW(w);
        q.setX(x);
        q.setY(y);
        q.setZ(z);
        phone.getTransform().setRotation(q);
        phone.getHeader().setStamp(node.getCurrentTime());
        phone.getHeader().setFrameId("/unity");
        phone.setChildFrameId("/phone");
        mTfMessage.getTransforms().add(phone);
    }

    public void setTranslation(double x, double y, double z) {
        Vector3 v = unity.getTransform().getTranslation();
        v.setX(x);
        v.setY(y);
        v.setZ(z);
        Quaternion q = unity.getTransform().getRotation();
        q.setW(1);
        unity.getTransform().setTranslation(v);
        unity.getHeader().setStamp(node.getCurrentTime());
        unity.getHeader().setFrameId("/global");
        unity.setChildFrameId("/unity");
        mTfMessage.getTransforms().add(unity);
    }

    public void publishTransforms(){
        publisher.publish(mTfMessage);

        mTfMessage = node.getTopicMessageFactory().newFromType(TFMessage._TYPE);
        unity = node.getTopicMessageFactory().newFromType(TransformStamped._TYPE);
        phone = node.getTopicMessageFactory().newFromType(TransformStamped._TYPE);

        mTfMessage.getTransforms().clear();
    }
}
