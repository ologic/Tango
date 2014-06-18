package org.ros.tf2.tf_broadcaster_app;

import android.os.Bundle;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.EditText;
import android.widget.SeekBar;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import org.ros.tf2_ros.TransformBroadcaster;
import org.ros.tf2_ros.StaticTransformBroadcaster;
import static java.lang.Math.*;

import geometry_msgs.Quaternion;
import geometry_msgs.TransformStamped;

public class TfBroadcasterApp extends RosActivity
{
    private final TransformBroadcaster mTB = new TransformBroadcaster();
    private final StaticTransformBroadcaster mSTB = new StaticTransformBroadcaster();

    private TransformStamped tfs = mTB.newMessage();
    private double mRoll = 0.0;
    private double mPitch = 0.0;
    private double mYaw = 0.0;

    private SeekBar sX;
    private SeekBar sY;
    private SeekBar sZ;
    private SeekBar sR;
    private SeekBar sP;
    private SeekBar sYaw;
    private EditText mSourceInput;
    private EditText mTargetInput;
    private CheckBox mStaticCheck;
    private CheckBox mDynamicCheck;

    private void fromRPY(Quaternion q, double r, double p, double y){
        double qw = cos(r/2.0)*cos(p/2.0)*cos(y/2.0) + sin(r/2.0)*sin(p/2.0)*sin(y/2.0);
        double qx = sin(r/2.0)*cos(p/2.0)*cos(y/2.0) - cos(r/2.0)*sin(p/2.0)*sin(y/2.0);
        double qy = cos(r/2.0)*sin(p/2.0)*cos(y/2.0) + sin(r/2.0)*cos(p/2.0)*sin(y/2.0);
        double qz = cos(r/2.0)*cos(p/2.0)*sin(y/2.0) - sin(r/2.0)*sin(p/2.0)*cos(y/2.0);

        q.setW(qw);
        q.setX(qx);
        q.setY(qy);
        q.setZ(qz);
    }

    class SliderChangeListener implements SeekBar.OnSeekBarChangeListener{
        private String mValue;

        public SliderChangeListener(String value){
            mValue = value;
        }

        @Override
        public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
            double norm = 2.0*(double)progress/((double)sX.getMax()) - 1.0; // 1.0 to -1.0
            if(mValue.equals("X")){
                tfs.getTransform().getTranslation().setX(10.0*norm);
            } else if(mValue.equals("Y")){
                tfs.getTransform().getTranslation().setY(10.0 * norm);
            } else if(mValue.equals("Z")){
                tfs.getTransform().getTranslation().setZ(10.0 * norm);
            } else if(mValue.equals("R")){
                mRoll = Math.PI * norm; // Roll is -Pi to Pi
                fromRPY(tfs.getTransform().getRotation(), mRoll, mPitch, mYaw);
            } else if(mValue.equals("P")){
                mPitch = Math.PI/2.0 * norm; // Pitch is -Pi/2 to Pi/2
                fromRPY(tfs.getTransform().getRotation(), mRoll, mPitch, mYaw);
            } else if(mValue.equals("Yaw")){
                mYaw = Math.PI * norm; // Yaw is -Pi to Pi
                fromRPY(tfs.getTransform().getRotation(), mRoll, mPitch, mYaw);
            }
            if(mDynamicCheck != null){
                if(mDynamicCheck.isChecked()){
                    tfs.getHeader().setFrameId(mSourceInput.getText().toString());
                    tfs.setChildFrameId(mTargetInput.getText().toString());
                    long lt = System.currentTimeMillis();
                    Time t = new Time((int)(lt / 1e3), (int)((lt % 1e3) *1e6));
                    tfs.getHeader().setStamp(t);
                    mTB.sendTransform(tfs);
                }
            }
        }
        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }
        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {

        }
    }


    public TfBroadcasterApp() {
        // The RosActivity constructor configures the notification title and ticker messages.
        super("TF Broadcaster", "TF Broadcaster");
        fromRPY(tfs.getTransform().getRotation(), 0.0, 0.0, 0.0); // Initialize quaternion
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        sX = (SeekBar)findViewById(R.id.seekBarX);
        sX.setOnSeekBarChangeListener(new SliderChangeListener("X"));
        sY = (SeekBar)findViewById(R.id.seekBarY);
        sY.setOnSeekBarChangeListener(new SliderChangeListener("Y"));
        sZ = (SeekBar)findViewById(R.id.seekBarZ);
        sZ.setOnSeekBarChangeListener(new SliderChangeListener("Z"));
        sR = (SeekBar)findViewById(R.id.seekBarR);
        sR.setOnSeekBarChangeListener(new SliderChangeListener("R"));
        sP = (SeekBar)findViewById(R.id.seekBarP);
        sP.setOnSeekBarChangeListener(new SliderChangeListener("P"));
        sYaw = (SeekBar)findViewById(R.id.seekBarYaw);
        sYaw.setOnSeekBarChangeListener(new SliderChangeListener("Yaw"));
        mSourceInput = (EditText)findViewById(R.id.editTextSource);
        mTargetInput = (EditText)findViewById(R.id.editTextTarget);
        mStaticCheck = (CheckBox)findViewById(R.id.checkBoxStatic);
        mDynamicCheck = (CheckBox)findViewById(R.id.checkBoxDynamic);

        mStaticCheck.setOnCheckedChangeListener(new CheckBox.OnCheckedChangeListener() {

            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if(isChecked){
                    tfs.getHeader().setFrameId(mSourceInput.getText().toString());
                    tfs.setChildFrameId(mTargetInput.getText().toString());
                    long lt = System.currentTimeMillis();
                    Time t = new Time((int)(lt / 1e3), (int)((lt % 1e3) *1e6));
                    tfs.getHeader().setStamp(t);
                    mSTB.sendTransform(tfs);
                    tfs = mSTB.newMessage();
                    fromRPY(tfs.getTransform().getRotation(), 0.0, 0.0, 0.0); // Initialize quaternion
                }
                mStaticCheck.setChecked(false);
            }
        });

    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress(), getMasterUri());
        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(mTB, nodeConfiguration);
        nodeMainExecutor.execute(mSTB, nodeConfiguration);
    }
}
