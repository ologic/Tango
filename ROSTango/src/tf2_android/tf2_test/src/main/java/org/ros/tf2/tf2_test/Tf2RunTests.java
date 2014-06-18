package org.ros.tf2.tf2_test;

import android.util.Log;

import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.tf2_ros.Buffer;

import java.util.Vector;

import geometry_msgs.Quaternion;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;

public class Tf2RunTests {

    private static void EXPECT_TRUE(boolean bool, String message){
        if(!bool){
            throw new RuntimeException(message);
        }
    }

    private static void EXPECT_FALSE(boolean bool, String message){
        if(bool){
            throw new RuntimeException(message);
        }
    }

    private static void EXPECT_EQ(Object o1, Object o2, String message){
        if(!o1.equals(o2)){
            throw new RuntimeException(message);
        }
    }

    private static void EXPECT_NEAR(double val, double exp, double eps, String message){
        if(Math.abs(val-exp) > Math.abs(eps)){
            throw new RuntimeException(message);
        }
    }

    private static void setIdentity(TransformStamped tfs)
    {
        tfs.getTransform().getTranslation().setX(0.0);
        tfs.getTransform().getTranslation().setY(0.0);
        tfs.getTransform().getTranslation().setZ(0.0);
        tfs.getTransform().getRotation().setX(0.0);
        tfs.getTransform().getRotation().setY(0.0);
        tfs.getTransform().getRotation().setZ(0.0);
        tfs.getTransform().getRotation().setW(1.0);
    }

    public static void noInsertOnSelfTransform(){
        Buffer b = new Buffer();
        NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        setIdentity(tfs);
        Time t = new Time(0, 10); // 10 NSec
        tfs.getHeader().setStamp(t);
        tfs.getHeader().setFrameId("some_frame");
        tfs.setChildFrameId("some_frame");
        EXPECT_FALSE(b.setTransform(tfs), "Self Transform");
    }

    public static void noInsertWithNaN(){
        Buffer b = new Buffer();
        NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        setIdentity(tfs);
        Time t = new Time(0, 10); // 10 NSec
        tfs.getHeader().setStamp(t);
        tfs.getHeader().setFrameId("some_frame");
        tfs.setChildFrameId("other_frame");
        EXPECT_TRUE(b.setTransform(tfs), "No NAN Yet");
        tfs.getTransform().getTranslation().setX(Double.NaN);
        EXPECT_TRUE(Double.isNaN(tfs.getTransform().getTranslation().getX()), "X is now NaN");
        EXPECT_FALSE(b.setTransform(tfs), "Try to set with NaN");
    }

    public static void noInsertWithNoFrameID(){
        Buffer b = new Buffer();
        NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        setIdentity(tfs);
        Time t = new Time(0, 10); // 10 NSec
        tfs.getHeader().setStamp(t);
        tfs.getHeader().setFrameId("some_frame");
        tfs.setChildFrameId("");
        EXPECT_FALSE(b.setTransform(tfs), "Set with no child_frame_id");
        tfs.setChildFrameId("/");
        EXPECT_FALSE(b.setTransform(tfs), "Set with '/' as child_frame_id");
    }

    public static void noInsertWithNoParentID(){
        Buffer b = new Buffer();
        NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
        MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        setIdentity(tfs);
        Time t = new Time(0, 10); // 10 NSec
        tfs.getHeader().setStamp(t);
        tfs.getHeader().setFrameId("");
        tfs.setChildFrameId("some_frame");
        EXPECT_FALSE(b.setTransform(tfs), "Set with no parent_frame_id");
        tfs.getHeader().setFrameId("/");
        EXPECT_FALSE(b.setTransform(tfs), "Set with '/' as parent_frame_id");
    }

    public static void iConfiguration(){
        double epsilon = 1e-6;

        Permuter permuter = new Permuter();

        Vector<Time> times = new Vector<Time>();
        times.add(new Time(1.0));
        times.add(new Time(10.0));
        times.add(new Time(0.0));
        Vector<Time> eval_time_vec = new Vector<Time>();
        eval_time_vec.add(new Time());
        permuter.addOptionSet(times, eval_time_vec);

        Vector<Duration> durations = new Vector<Duration>();
        durations.add(new Duration(1.0));
        durations.add(new Duration(0.001));
        durations.add(new Duration(0.1));
        Duration interpolation_space = new Duration();
        // Not set in tf2 tests as of October 13, 2013
        //  permuter.addOptionSet(durations, &interpolation_space);

        Vector<String> frames = new Vector<String>();
        frames.add("a");
        frames.add("b");
        frames.add("c");
        Vector<String> source_frame_vec = new Vector<String>();
        source_frame_vec.add("");
        permuter.addOptionSet(frames, source_frame_vec);

        Vector<String> target_frame_vec = new Vector<String>();
        target_frame_vec.add("");
        permuter.addOptionSet(frames, target_frame_vec);

        while(permuter.step()){
            String source_frame = source_frame_vec.get(0);
            String target_frame = target_frame_vec.get(0);
            Time eval_time = eval_time_vec.get(0);

            Buffer mBC = new Buffer();
            Tf2SetupTests.setupTree(mBC, "i", eval_time, interpolation_space);

            String info = "\nsource_frame: " + source_frame;
            info += "\ntarget_frame: " + target_frame;
            info += "\ntime: " + Double.toString(eval_time.toSeconds());

            TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);
            if(outpose == null){
                throw new RuntimeException("i lookup transform returned null." + info);
            }
            EXPECT_EQ(outpose.getHeader().getStamp(), eval_time, "Compare header and eval_time" + info);
            EXPECT_EQ(outpose.getHeader().getFrameId(), source_frame, "Compare header frame_id and source_frame" + info);
            EXPECT_EQ(outpose.getChildFrameId(), target_frame, "Compare child frame_id and target_frame" + info);
            Vector3 trans = outpose.getTransform().getTranslation();
            EXPECT_NEAR(trans.getY(), 0.0, epsilon, "Compare y to 0.0" + info);
            EXPECT_NEAR(trans.getZ(), 0.0, epsilon, "Compare z to 0.0" + info);
            Quaternion quat = outpose.getTransform().getRotation();
            EXPECT_NEAR(quat.getX(), 0.0, epsilon, "Compare qx to 0.0" + info);
            EXPECT_NEAR(quat.getY(), 0.0, epsilon, "Compare qy to 0.0" + info);
            EXPECT_NEAR(quat.getZ(), 0.0, epsilon, "Compare qz to 0.0" + info);
            EXPECT_NEAR(quat.getW(), 1.0, epsilon, "Compare qw to 1.0" + info);

            //Zero distance
            if (source_frame.equals(target_frame)){
                EXPECT_NEAR(trans.getX(), 0.0, epsilon, "Zero distance X" + info);
                Log.w("TF2 TEST", "Zero distance X" + info);
            } else if ((source_frame.equals("a") && target_frame.equals("b")) ||
                    (source_frame.equals("b") && target_frame.equals("c"))){
                EXPECT_NEAR(trans.getX(), 1.0, epsilon, "a/b or b/c distance X" + info);
                Log.w("TF2 TEST", "a/b or b/c distance X" + info);
            }else if ((source_frame.equals("b") && target_frame.equals("a")) ||
                    (source_frame.equals("c") && target_frame.equals("b"))){
                EXPECT_NEAR(trans.getX(), -1.0, epsilon, "b/a or c/b distance X" + info);
                Log.w("TF2 TEST", "b/a or c/b distance X" + info);
            }else if (source_frame.equals("a") && target_frame.equals("c")){
                EXPECT_NEAR(trans.getX(), 2.0, epsilon, "a/c distance X" + info);
                Log.w("TF2 TEST", "a/c distance X" + info);
            }else if (source_frame.equals("c") && target_frame.equals("a")){
                EXPECT_NEAR(trans.getX(), -2.0, epsilon, "c/a distance X" + info);
                Log.w("TF2 TEST", "c/a distance X" + info);
            } else {
                throw new RuntimeException("i configuration: Shouldn't get here" + info);
            }
        }
    }

    public static void one_link_configuration(){
        double epsilon = 1e-6;

        Permuter permuter = new Permuter();

        Vector<Time> times = new Vector<Time>();
        times.add(new Time(1.0));
        times.add(new Time(10.0));
        times.add(new Time(0.0));
        Vector<Time> eval_time_vec = new Vector<Time>();
        eval_time_vec.add(new Time());
        permuter.addOptionSet(times, eval_time_vec);

        Vector<Duration> durations = new Vector<Duration>();
        durations.add(new Duration(1.0));
        durations.add(new Duration(0.001));
        durations.add(new Duration(0.1));
        Duration interpolation_space = new Duration();
        // Not set in tf2 tests as of October 13, 2013
        //  permuter.addOptionSet(durations, &interpolation_space);

        Vector<String> frames = new Vector<String>();
        frames.add("1");
        frames.add("2");
        Vector<String> source_frame_vec = new Vector<String>();
        source_frame_vec.add("");
        permuter.addOptionSet(frames, source_frame_vec);

        Vector<String> target_frame_vec = new Vector<String>();
        target_frame_vec.add("");
        permuter.addOptionSet(frames, target_frame_vec);

        while(permuter.step()){
            String source_frame = source_frame_vec.get(0);
            String target_frame = target_frame_vec.get(0);
            Time eval_time = eval_time_vec.get(0);

            Buffer mBC = new Buffer();
            Tf2SetupTests.setupTree(mBC, "1", eval_time, interpolation_space);

            TransformStamped outpose = mBC.lookupTransform(source_frame, target_frame, eval_time);
            if(outpose == null){
                throw new RuntimeException("1 lookup transform returned null.");
            }

            EXPECT_TRUE(Tf2SetupTests.check_1_result(outpose, source_frame, target_frame, eval_time, epsilon),
                        "check_1_result_total_output");
        }
    }
}
