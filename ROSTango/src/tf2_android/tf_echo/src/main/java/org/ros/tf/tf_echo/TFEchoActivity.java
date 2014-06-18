package org.ros.tf.tf_echo;

import android.content.Context;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.util.Log;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import org.ros.android.MessageCallable;
import org.ros.android.RosActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.address.InetAddressFactory;
import org.ros.tf2_ros.Buffer;
import org.ros.tf2_ros.TransformListener;

import java.util.TimerTask;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;


public class TFEchoActivity extends RosActivity {

    private Buffer mBuffer;
    private TransformListener mListener;

    private Handler mHandler = new Handler();
    private TextView mTV;
    private EditText mSourceInput;
    private EditText mTargetInput;

    private int mInterval = 100;

    public TFEchoActivity() {
        // The RosActivity constructor configures the notification title and ticker messages.
        super("TF Echo", "TF Echo");
        mBuffer = new Buffer();
        mListener = new TransformListener(mBuffer);
    }

    private double[] quaternionToRPY(Quaternion q){
        double q0 = q.getW();
        double q1 = q.getX();
        double q2 = q.getY();
        double q3 = q.getZ();

        double[] out = new double[]{0, 0, 0};
        out[0] = Math.atan2(2.0*(q0*q1+q2*q3), 1.0 - 2.0*(q1*q1+q2*q2));
        out[1] = Math.asin(2.0*(q0*q2-q3*q1));
        out[2] = Math.atan2(2.0*(q0*q3+q1*q2), 1.0-2.0*(q2*q2+q3*q3));
        return out;
    }

    private Runnable mUpdateTimeTask = new Runnable() {
        public void run() {
            String target = mTargetInput.getText().toString();
            String source = mSourceInput.getText().toString();
            Time lookup_time = new Time(0, 0); // latest available
            TransformStamped tfs = mBuffer.lookupTransform(target, source, lookup_time);
            String output = "Cannot lookup " + target + " to " + source + ".";
            if(tfs != null){
                Transform t = tfs.getTransform();
                Vector3 tr = t.getTranslation();
                Quaternion q = t.getRotation();
                double[] rpy = quaternionToRPY(q);
                double roll = rpy[0];
                double pitch = rpy[1];
                double yaw = rpy[2];
                output = "At time " + String.format("%.3f", tfs.getHeader().getStamp().toSeconds());
                output += "\n- Translation: ";
                output += "\n\t\t\t[" + tr.getX() + ", " + tr.getY() + ", " + tr.getZ() + "]";
                output += "\n- Rotation: in Quaternion";
                output += "\n\t\t\t[" + q.getX() + ", " + q.getY() + ", " + q.getZ() + ", " + q.getW() + "]";
                output += "\nin RPY";
                output += "\n\t\t\t[" + roll + ", " + pitch + ", " + yaw + "]";
                /*At time 1380414650.847
                        - Translation: [0.000, 0.000, -0.051]
                - Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
                in RPY [0.000, -0.000, 0.000]*/
            }
            mTV.setText(output);
            mHandler.postDelayed(this, mInterval);
        }
    };

    @SuppressWarnings("unchecked")
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        mTV = (TextView)findViewById(R.id.textViewOutput);
        mSourceInput = (EditText)findViewById(R.id.editTextSource);
        mTargetInput = (EditText)findViewById(R.id.editTextTarget);
        mHandler.removeCallbacks(mUpdateTimeTask);
        mHandler.postDelayed(mUpdateTimeTask, mInterval);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        mListener = new TransformListener();
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(mListener, nodeConfiguration);
    }
}