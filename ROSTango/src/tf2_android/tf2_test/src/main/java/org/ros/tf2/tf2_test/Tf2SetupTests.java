package org.ros.tf2.tf2_test;

import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;
import org.ros.tf2_ros.Buffer;

import java.util.Vector;

import geometry_msgs.Quaternion;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;

public class Tf2SetupTests {
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

    public static void setupTree(Buffer mBC, String mode, Time time){
        setupTree(mBC, mode, time, new Duration());
    }

    public static void push_back_i(Vector<String> children, Vector<String> parents, Vector<Double> dx, Vector<Double> dy)
    {
        /*
         "a"
         v   (1,0)
         "b"
         v   (1,0)
         "c"
        */

        children.add("b");
        parents.add("a");
        dx.add(1.0);
        dy.add(0.0);

        children.add("c");
        parents.add("b");
        dx.add(1.0);
        dy.add(0.0);
    }

    public static void push_back_1(Vector<String> children, Vector<String> parents, Vector<Double> dx, Vector<Double> dy){
        children.add("2");
        parents.add("1");
        dx.add(1.0);
        dy.add(0.0);
    }

    public static void setupTree(Buffer mBC, String mode, Time time, Duration interpolation_space){
        mBC.clear();
        Vector<String> children = new Vector<String>();
        Vector<String> parents = new Vector<String>();
        Vector<Double> dx = new Vector<Double>();
        Vector<Double> dy = new Vector<Double>();

        if(mode.equals("i")){
            push_back_i(children, parents, dx, dy);
        /*} else if(mode.equals("y")){
            //push_back_y(children, parents, dx, dy);
        } else if(mode.equals("v")){
            //push_back_v(children, parents, dx, dy);
        } else if(mode.equals("ring_45")){
            //Form a ring of transforms at every 45 degrees on the unit circle.

            std::vector<std::string> frames;



            frames.push_back("a");
            frames.push_back("b");
            frames.push_back("c");
            frames.push_back("d");
            frames.push_back("e");
            frames.push_back("f");
            frames.push_back("g");
            frames.push_back("h");
            frames.push_back("i");

            for (uint8_t iteration = 0; iteration < 2; ++iteration)
            {
                double direction = 1;
                std::string frame_prefix;
                if (iteration == 0)
                {
                    frame_prefix = "inverse_";
                    direction = -1;
                }
                else
                    frame_prefix ="";
                for (uint64_t i = 1; i <  frames.size(); i++)
                {
                    geometry_msgs::TransformStamped ts;
                    setIdentity(ts.transform);
                    ts.transform.translation.x = direction * ( sqrt(2)/2 - 1);
                    ts.transform.translation.y = direction * sqrt(2)/2;
                    ts.transform.rotation.x = 0;
                    ts.transform.rotation.y = 0;
                    ts.transform.rotation.z = sin(direction * M_PI/8);
                    ts.transform.rotation.w = cos(direction * M_PI/8);
                    if (time > ros::Time() + (interpolation_space * .5))
                    ts.header.stamp = time - (interpolation_space * .5);
                    else
                    ts.header.stamp = ros::Time();

                    ts.header.frame_id = frame_prefix + frames[i-1];
                    if (i > 1)
                        ts.child_frame_id = frame_prefix + frames[i];
                    else
                        ts.child_frame_id = frames[i]; // connect first frame
                    EXPECT_TRUE(mBC.setTransform(ts, "authority"));
                    if (interpolation_space > ros::Duration())
                    {
                        ts.header.stamp = time + interpolation_space * .5;
                        EXPECT_TRUE(mBC.setTransform(ts, "authority"));
                    }
                            }
        }
        }
        return; // nonstandard setup return before standard executinog
        */
        } else if(mode.equals("1")){
            push_back_1(children, parents, dx, dy);
        /*} else if(mode.equals("1_v")){
            //push_back_1(children, parents, dx, dy);
            //push_back_v(children, parents, dx, dy);*/
        } else {
            throw new RuntimeException("Undefined mode for tree setup.  Test harness improperly setup.");
        }

        /// Standard
        for (int i = 0; i <  children.size(); i++)
        {
            NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
            MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
            TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
            setIdentity(tfs);
            tfs.getTransform().getTranslation().setX(dx.get(i));
            tfs.getTransform().getTranslation().setY(dy.get(i));
            Duration half_interpolation_space = new Duration(0.5/1.e9*(double)interpolation_space.totalNsecs());
            if (time.compareTo(new Time().add(half_interpolation_space)) > 0){
                tfs.getHeader().setStamp(time.subtract(interpolation_space));
            } else {
                tfs.getHeader().setStamp(new Time());
            }

            tfs.getHeader().setFrameId(parents.get(i));
            tfs.setChildFrameId(children.get(i));
            EXPECT_TRUE(mBC.setTransform(tfs), "Setup Tree setTransform");
            if (interpolation_space.compareTo(new Duration()) > 0)
            {
                tfs.getHeader().getStamp().add(half_interpolation_space);
                EXPECT_TRUE(mBC.setTransform(tfs), "Setup Tree setTransform Interpolation Space");

            }
        }
    }

    /* Check 1 result return false if test parameters unmet */
    public static boolean check_1_result(TransformStamped outpose, String source_frame, String target_frame,
                                         Time eval_time, double epsilon)
    {
        //printf("source_frame %s target_frame %s time %f\n", source_frame.c_str(), target_frame.c_str(), eval_time.toSec());
        EXPECT_EQ(outpose.getHeader().getStamp(), eval_time, "check_1_result_check_stamps");
        EXPECT_EQ(outpose.getHeader().getFrameId(), source_frame, "check_1_result_check_frame_id");
        EXPECT_EQ(outpose.getChildFrameId(), target_frame, "check_1_result_check_child_frame_id");
        Vector3 trans = outpose.getTransform().getTranslation();
        EXPECT_NEAR(trans.getY(), 0, epsilon, "check_1_result_check_y_trans");
        EXPECT_NEAR(trans.getZ(), 0, epsilon, "check_1_result_check_z_trans");
        Quaternion quat = outpose.getTransform().getRotation();
        EXPECT_NEAR(quat.getX(), 0, epsilon, "check_1_result_check_qx");
        EXPECT_NEAR(quat.getY(), 0, epsilon, "check_1_result_check_qy");
        EXPECT_NEAR(quat.getZ(), 0, epsilon, "check_1_result_check_qz");
        EXPECT_NEAR(quat.getW(), 1, epsilon, "check_1_result_check_qw");

        //Zero distance
        if (source_frame.equals(target_frame))
        {
            EXPECT_NEAR(trans.getX(), 0, epsilon, "check_1_zero_X");
        }
        else if (source_frame.equals("1") && target_frame.equals("2"))
        {
            EXPECT_NEAR(trans.getX(), 1, epsilon, "check_1_plus_X");
        }
        else if (source_frame.equals("2") && target_frame.equals("1"))
        {
            EXPECT_NEAR(trans.getX(), -1, epsilon, "check_1_neg_X");
        }
        else
        {
            throw new RuntimeException("1 configuration: Shouldn't get here");
        }
        return true;
    }
}