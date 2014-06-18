package org.ros.tf2_ros;

import geometry_msgs.TransformStamped;
import org.ros.message.Duration;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import java.util.List;

public class TransformListener extends AbstractNodeMain {

	private Buffer mBuffer;

	public TransformListener(){
        Duration d = new Duration(9.9);
		mBuffer = new Buffer(d);
	}

	public TransformListener(Buffer buffer){
		mBuffer = buffer;
	}

	private void addTFMessageToBuffer(final tf2_msgs.TFMessage msg, String authority, boolean is_static){
		final List<TransformStamped> tfs = msg.getTransforms();
        for (TransformStamped tf : tfs) {
            mBuffer.setTransform(tf, authority, is_static);
        }
	}


	@Override
	  public GraphName getDefaultNodeName() {
	    return GraphName.of("transform_listener_").join(GraphName.newAnonymous());
	}

	@Override
	public void onStart(ConnectedNode connectedNode) {
        final int queue_size = 100;
		Subscriber<tf2_msgs.TFMessage> subscriber = connectedNode.newSubscriber("/tf", tf2_msgs.TFMessage._TYPE);
		subscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
		  @Override
		  public void onNewMessage(tf2_msgs.TFMessage message) {
		  	addTFMessageToBuffer(message, "unknown", false);
		  }
		}, queue_size);
		Subscriber<tf2_msgs.TFMessage> static_subscriber = connectedNode.newSubscriber("/tf_static", tf2_msgs.TFMessage._TYPE);
		static_subscriber.addMessageListener(new MessageListener<tf2_msgs.TFMessage>() {
		  @Override
		  public void onNewMessage(tf2_msgs.TFMessage message) {
		  	addTFMessageToBuffer(message, "unknown", true);
		  }
		}, queue_size);
	}

}