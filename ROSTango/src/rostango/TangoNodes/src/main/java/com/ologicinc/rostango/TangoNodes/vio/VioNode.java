package com.ologicinc.rostango.TangoNodes.vio;

import com.motorola.atap.androidvioservice.VinsServiceHelper;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

/**
 * Created by rohan on 7/8/14.
 */
public class VioNode implements NodeMain {

    private VinsServiceHelper mVinsServiceHelper;

    private TangoOdomPublisher mTangoOdomPublisher;
    private TangoPosePublisher mTangoPosePublisher;

    public VioNode(VinsServiceHelper vins){
        mVinsServiceHelper = vins;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("tango_vio");
    }

    @Override
    public void onStart(ConnectedNode node) {
        mTangoOdomPublisher = new TangoOdomPublisher(node);
        mTangoPosePublisher = new TangoPosePublisher(node);

        node.executeCancellableLoop(new CancellableLoop() {
           @Override
            protected void loop() throws InterruptedException {
               Thread.sleep(30);
               final double[] posState = mVinsServiceHelper.getStateInFullStateFormat();
               final double[] rotState = mVinsServiceHelper.getStateInUnityFormat();
               // Generate the TF message

               updateTranslation(posState);
               Thread.sleep(50);

               updateRoataion(rotState);
               Thread.sleep(50);

               mTangoOdomPublisher.publishOdom();
               mTangoPosePublisher.publishPose();
            }
        });
    }

    @Override
    public void onShutdown(Node node) {
        mVinsServiceHelper.shutdown();
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public void onError(Node node, Throwable throwable) {
    }

    public void updateTranslation(double[] state) {
//        // mQuat
//        tfs.getTransform().getRotation().setX(0);
//        tfs.getTransform().getRotation().setY(0);
//        tfs.getTransform().getRotation().setZ(0);
//        tfs.getTransform().getRotation().setW(1);
//
//        //mPos
//        tfs.getTransform().getTranslation().setX(state[5]);
//        tfs.getTransform().getTranslation().setY(-state[4]);
//        tfs.getTransform().getTranslation().setZ(state[6]);  // state[6]
//
//        tfs.getHeader().setFrameId("/global");
//        tfs.setChildFrameId("/unity");
//
//        long lt = System.currentTimeMillis();
//        Time t = new Time((int) (lt / 1e3), (int) ((lt % 1e3) * 1e6));
//        tfs.getHeader().setStamp(t);
//
//        mTB.sendTransform(tfs);


        mTangoOdomPublisher.setPosePoint(state[5],-state[4], 0x0);
        mTangoPosePublisher.setPoint(state[5],-state[4], 0x0);
    }

    public void updateRoataion(double[] state) {
//        // mQuat
//        tfs.getTransform().getRotation().setX(-state[2]);
//        tfs.getTransform().getRotation().setY(state[0]);
//        tfs.getTransform().getRotation().setZ(-state[1]);
//        tfs.getTransform().getRotation().setW(state[3]);
//
//        //mPos
//        tfs.getTransform().getTranslation().setX(0);
//        tfs.getTransform().getTranslation().setY(0);
//        tfs.getTransform().getTranslation().setZ(0);  // state[6]
//
//        tfs.getHeader().setFrameId("/unity");
//        tfs.setChildFrameId("/phone");
//
//        long lt = System.currentTimeMillis();
//        Time t = new Time((int) (lt / 1e3), (int) ((lt % 1e3) * 1e6));
//        tfs.getHeader().setStamp(t);
//
//        mTB.sendTransform(tfs);


        mTangoOdomPublisher.setPoseQuat(-state[2], state[0], -state[1], state[3]);
        mTangoPosePublisher.setQuat(-state[2],state[0],-state[1],state[3]);
    }
}
