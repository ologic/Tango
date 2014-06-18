package org.ros.tf2_ros;

import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.topic.Publisher;

import java.util.Arrays;
import java.util.List;

import geometry_msgs.TransformStamped;
import tf2_msgs.TFMessage;

public class StaticTransformBroadcaster extends AbstractNodeMain {
    private Publisher<TFMessage> mPublisher;
    TFMessage mNetMessage;

    NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("static_transform_broadcaster_").join(GraphName.newAnonymous());
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        mPublisher = connectedNode.newPublisher("/tf_static", TFMessage._TYPE);
        mPublisher.setLatchMode(true);
        mNetMessage = mPublisher.newMessage();
    }

    //TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
    public void sendTransform(final TransformStamped msgtf){
        List<TransformStamped> v1 = Arrays.asList(msgtf);
        sendTransform(v1);
    }

    public void sendTransform(final List<TransformStamped> msgtf){
        if(mNetMessage == null){
            return;
        }
        List<TransformStamped> netList = mNetMessage.getTransforms();
        for (TransformStamped aMsgtf : msgtf) {
            boolean match_found = false;
            for (int j = 0; j < netList.size(); j++) {
                if (aMsgtf.getChildFrameId().equals(netList.get(j).getChildFrameId())) {
                    // Replace netList transform with new transform
                    netList.set(j, aMsgtf);
                    match_found = true;
                    break;
                }
            }
            if (!match_found) { // New transform not previous in list, add to list
                netList.add(aMsgtf);
            }
        }

        mNetMessage.setTransforms(netList);
        mPublisher.publish(mNetMessage);
    }

    public TransformStamped newMessage(){
        return mMessageFactory.newFromType(TransformStamped._TYPE);
    }
}
