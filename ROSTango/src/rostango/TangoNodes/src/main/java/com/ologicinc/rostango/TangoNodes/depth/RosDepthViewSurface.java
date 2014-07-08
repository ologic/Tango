package com.ologicinc.rostango.TangoNodes.depth;

import android.content.Context;
import android.util.AttributeSet;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

/**
 * Created by Rohan Agrawal on 7/3/14.
 */
public class RosDepthViewSurface extends DepthViewSurface implements NodeMain{
    public RosDepthViewSurface(Context context) {
        super(context);
    }

    public RosDepthViewSurface(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("ros_depth_preview_view");
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        setRawImageListener(new DepthPublisher(connectedNode));
    }

    @Override
    public void onShutdown(Node node) {
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

}
