package com.ologicinc.rostango.tango_vio;

import android.content.pm.PackageInfo;
import android.content.pm.PackageManager.NameNotFoundException;
import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.widget.Toast;

import com.motorola.atap.androidvioservice.VinsServiceHelper;
import com.ologicinc.rostango.tango_vio.nodes.TangoPosePublisher;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.tf2_ros.StaticTransformBroadcaster;
import org.ros.tf2_ros.TransformBroadcaster;

import java.io.File;

import geometry_msgs.TransformStamped;

public class TangoVio extends RosActivity {
    // Set up Transform Broadcasters
    private final TransformBroadcaster mTB = new TransformBroadcaster();
    private final StaticTransformBroadcaster mSTB = new StaticTransformBroadcaster();
    // XXX private Quaternion mQuat;
    // XXX private Vector3 mPos;

    // Setup TangoPosePublisher
    private TangoPosePublisher mPosePub;

    // Set up messages
    private TransformStamped tfs = mTB.newMessage();

    // Set up VIO format
    private VinsServiceHelper mVinsServiceHelper;
    int coordinateConvention = VinsServiceHelper.STATEFORMAT_FULL_STATE;

    private int superFrameBufferSize;
    private byte[] b;
    private Bitmap bm;

    private boolean mConnected=false;

    public TangoVio() {
        super("TangoVio", "TangoVio");
    }

    /**
     * Called when the activity is first created.
     */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        /* Read VIO data from VINs service helper */
        mVinsServiceHelper = new VinsServiceHelper(this);

        /* Check for the calibration file. */
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/calibration.xml");
        if (!file.exists()) {
            Toast.makeText(getApplicationContext(), "Could not load " +
                    "/sdcard/calibration.xml, using default calibration." +
                    " For best performance install the file and restart the " +
                    "app.", Toast.LENGTH_LONG).show();
        }

        /* Set the version code and name on the title bar. */
        try {
            PackageInfo pkgInfo =
                    getPackageManager().getPackageInfo(getPackageName(), 0);
            setTitle("Vins Service Tester v" + pkgInfo.versionName);
        } catch (NameNotFoundException e) {
            // App is looking for its own package name - don't need to handle this.
        }

        setContentView(R.layout.main);

        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while (mVinsServiceHelper != null) {
                        Thread.sleep(30);
                        final double[] posState = mVinsServiceHelper.getStateInFullStateFormat();
                        final double[] rotState = mVinsServiceHelper.getStateInUnityFormat();
                        // Generate the TF message
                        updateTranslation(posState);
                        Thread.sleep(50);
                        updateRoataion(rotState);
                    }
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }).start();


    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        mPosePub = new TangoPosePublisher();

        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(mPosePub, nodeConfiguration);
        nodeMainExecutor.execute(mTB, nodeConfiguration);
        nodeMainExecutor.execute(mSTB, nodeConfiguration);

        try {
            Thread.sleep(2000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        mConnected = true;
    }

    public void updateTranslation(double[] state) {


        if (!mConnected || mTB==null) {return;}

        // mQuat
        tfs.getTransform().getRotation().setX(0);
        tfs.getTransform().getRotation().setY(0);
        tfs.getTransform().getRotation().setZ(0);
        tfs.getTransform().getRotation().setW(1);

        //mPos
        tfs.getTransform().getTranslation().setX(state[5]);
        tfs.getTransform().getTranslation().setY(-state[4]);
        tfs.getTransform().getTranslation().setZ(state[6]);  // state[6]

        tfs.getHeader().setFrameId("/global");
        tfs.setChildFrameId("/unity");

        long lt = System.currentTimeMillis();
        Time t = new Time((int) (lt / 1e3), (int) ((lt % 1e3) * 1e6));
        tfs.getHeader().setStamp(t);

        mTB.sendTransform(tfs);
    }
    public void updateRoataion(double[] state) {


        if (!mConnected || mTB==null) {return;}

        // mQuat
        tfs.getTransform().getRotation().setX(-state[2]);
        tfs.getTransform().getRotation().setY(state[0]);
        tfs.getTransform().getRotation().setZ(-state[1]);
        tfs.getTransform().getRotation().setW(state[3]);

        //mPos
        tfs.getTransform().getTranslation().setX(0);
        tfs.getTransform().getTranslation().setY(0);
        tfs.getTransform().getTranslation().setZ(0);  // state[6]

        tfs.getHeader().setFrameId("/unity");
        tfs.setChildFrameId("/phone");

        long lt = System.currentTimeMillis();
        Time t = new Time((int) (lt / 1e3), (int) ((lt % 1e3) * 1e6));
        tfs.getHeader().setStamp(t);

        mTB.sendTransform(tfs);
    }

}
