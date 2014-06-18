package org.ros.tf2_ros;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import std_msgs.Header;

import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

public class Buffer {
    static {
        System.loadLibrary("tf2_ros");
    }

    NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
    MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();

    public Buffer(){
        this(new Duration(0));
    }

    public Buffer(Duration cache_time){
        if(cache_time.isPositive()){ // Override default cache_time
            newBufferWithCacheDuration(cache_time.secs, cache_time.nsecs);
        }
        clear();
    }

    /** \brief Add transform information to the tf data structure
     * \param transform The transform to store
     * \return True unless an error occured
     */
    public boolean setTransform(final geometry_msgs.TransformStamped transform){
        return setTransform(transform, "unknown", false);
    }

    /** \brief Add transform information to the tf data structure
     * \param transform The transform to store
     * \param authority The source of the information for this transform
     * \return True unless an error occured
     */
    public boolean setTransform(final geometry_msgs.TransformStamped transform, String authority){
        return setTransform(transform, authority, false);
    }

    /** \brief Add transform information to the tf data structure
     * \param transform The transform to store
     * \param authority The source of the information for this transform
     * \param is_static Record this transform as a static transform.  It will be good across all time.  (This cannot be changed after the first call.)
     * \return True unless an error occured
     */
    public boolean setTransform(final geometry_msgs.TransformStamped transform, String authority, boolean is_static){
        final Header h = transform.getHeader();
        final Time s = h.getStamp();
        final Transform t = transform.getTransform();
        final Vector3 tr = t.getTranslation();
        final Quaternion q = t.getRotation();
        return setTransform(h.getFrameId(), s.secs, s.nsecs, transform.getChildFrameId(),
                tr.getX(), tr.getY(), tr.getZ(),
                q.getW(), q.getX(), q.getY(), q.getZ(), authority, is_static);
    }

    /** \brief Get the transform between two frames by frame ID.
     * \param target_frame The frame to which data should be transformed
     * \param source_frame The frame where the data originated
     * \param time The time at which the value of the transform is desired. (0 will get the latest)
     * \return The transform between the frames
     */
    /* Convenience form */
    public geometry_msgs.TransformStamped lookupTransform(String target_frame, String source_frame, Time time){
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        boolean result = lookupTransform(tfs, target_frame, source_frame, time);
        if(!result){
            return null;
        }
        return tfs;
    }

    /** \brief Get the transform between two frames by frame ID.
     * \param tfs The TransformStamped to fill
     * \param target_frame The frame to which data should be transformed
     * \param source_frame The frame where the data originated
     * \param time The time at which the value of the transform is desired. (0 will get the latest)
     * \return If lookup successful
     *
     * Possible exceptions tf2::LookupException, tf2::ConnectivityException,
     * tf2::ExtrapolationException, tf2::InvalidArgumentException
     */
    /* Performance form */
    public boolean lookupTransform(geometry_msgs.TransformStamped tfs, String target_frame, String source_frame, Time time){
        // Lookup through JNI
        // double[] doubles = tx, ty, tz, rw, rx, ry, rz, sec, nsec
        double[] doubles = lookupTransform(target_frame, source_frame, time.secs, time.nsecs);
        if(doubles.length != 9){
            return false;
        }
        Time actual_time = new Time((int)doubles[7], (int)doubles[8]);

        // Set output transform
        Header h = tfs.getHeader();
        h.setFrameId(target_frame);
        tfs.setChildFrameId(source_frame);
        h.setStamp(actual_time);
        Transform t = tfs.getTransform();
        Vector3 tr = t.getTranslation();
        Quaternion q = t.getRotation();
        tr.setX(doubles[0]);
        tr.setY(doubles[1]);
        tr.setZ(doubles[2]);
        q.setW(doubles[3]);
        q.setX(doubles[4]);
        q.setY(doubles[5]);
        q.setZ(doubles[6]);
        return true;
    }

    /** \brief Get the transform between two frames by frame ID assuming fixed frame.
     * \param target_frame The frame to which data should be transformed
     * \param target_time The time to which the data should be transformed. (0 will get the latest)
     * \param source_frame The frame where the data originated
     * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
     * \param fixed_frame The frame in which to assume the transform is constant in time.
     * \return The transform between the frames
     */
    public geometry_msgs.TransformStamped lookupTransform(String target_frame, Time target_time,
                                   String source_frame, Time source_time, String fixed_frame){
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        boolean result = lookupTransform(tfs, target_frame, target_time, source_frame, source_time, fixed_frame);
        if(!result){
            return null;
        }
        return tfs;
    }

    /** \brief Get the transform between two frames by frame ID assuming fixed frame.
     * \param The transform between the frames to fill
     * \param target_frame The frame to which data should be transformed
     * \param target_time The time to which the data should be transformed. (0 will get the latest)
     * \param source_frame The frame where the data originated
     * \param source_time The time at which the source_frame should be evaluated. (0 will get the latest)
     * \param fixed_frame The frame in which to assume the transform is constant in time.
     * \return If lookup was successful
     */
    public boolean lookupTransform(TransformStamped tfs, String target_frame, Time target_time,
                                   String source_frame, Time source_time, String fixed_frame){
        // Lookup through JNI
        // double[] doubles = tx, ty, tz, rw, rx, ry, rz, sec, nsec
        double[] doubles = lookupTransform(target_frame, target_time.secs, target_time.nsecs,
                                           source_frame, source_time.secs, source_time.nsecs,
                                           fixed_frame);
        if(doubles.length != 9){
            return false;
        }
        Time actual_time = new Time((int)doubles[7], (int)doubles[8]);

        // Set output transform
        Header h = tfs.getHeader();
        h.setFrameId(target_frame);
        tfs.setChildFrameId(source_frame);
        h.setStamp(actual_time);
        Transform t = tfs.getTransform();
        Vector3 tr = t.getTranslation();
        Quaternion q = t.getRotation();
        tr.setX(doubles[0]);
        tr.setY(doubles[1]);
        tr.setZ(doubles[2]);
        q.setW(doubles[3]);
        q.setX(doubles[4]);
        q.setY(doubles[5]);
        q.setZ(doubles[6]);
        return true;
    }

    public geometry_msgs.TransformStamped lookupTransform(String target_frame, String source_frame, Time time, Duration timeout){
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        boolean result = lookupTransform(tfs, target_frame, source_frame, time, timeout);
        if(!result){
            return null;
        }
        return tfs;
    }

    public boolean lookupTransform(TransformStamped tfs, String target_frame, String source_frame, Time time, Duration timeout){
        canTransform(target_frame, source_frame, time, timeout);
        return lookupTransform(tfs, target_frame, source_frame, time);
    }

    public geometry_msgs.TransformStamped lookupTransform(String target_frame, Time target_time,
                                   String source_frame, Time source_time,
                                   String fixed_frame, Duration timeout){
        TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);
        boolean result = lookupTransform(tfs, target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
        if(!result){
            return null;
        }
        return tfs;
    }

    public boolean lookupTransform(TransformStamped tfs, String target_frame, Time target_time,
                                   String source_frame, Time source_time,
                                   String fixed_frame, Duration timeout){
        canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout);
        return lookupTransform(tfs, target_frame, target_time, source_frame, source_time, fixed_frame);
    }

    public boolean canTransform(String target_frame, String source_frame, Time time){
        String ignored_error = "";
        return canTransform(target_frame, source_frame, time, ignored_error);
    }

    public boolean canTransform(String target_frame, String source_frame,
                                Time time, String error_msg){
        return canTransform(target_frame, source_frame, time.secs, time.nsecs, error_msg);
    }

    public boolean canTransform(String target_frame, Time target_time,
                                String source_frame, Time source_time,
                                String fixed_frame){
        String ignored_error = "";
        return canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, ignored_error);
    }

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param target_time The time into which to transform
     * \param source_frame The frame from which to transform
     * \param source_time The time from which to transform
     * \param fixed_frame The frame in which to treat the transform as constant in time
     * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
     * \return True if the transform is possible, false otherwise
     */
    public boolean canTransform(String target_frame, Time target_time,
                                String source_frame, Time source_time,
                                String fixed_frame, String error_msg){
        return canTransform(target_frame, target_time.secs, target_time.nsecs,
                            source_frame, source_time.secs, source_time.nsecs,
                            fixed_frame, error_msg);
    }

    /** \brief Test if a transform is possible
     * \param target_frame The frame into which to transform
     * \param source_frame The frame from which to transform
     * \param time The time at which to transform
     * \param error_msg A pointer to a string which will be filled with why the transform failed, if not NULL
     * \return True if the transform is possible, false otherwise
     */
    public boolean canTransform(String target_frame, String source_frame, Time time, Duration timeout){
        String ignored_error = "";
        return canTransform(target_frame, source_frame, time, timeout, ignored_error);
    }

    public boolean canTransform(String target_frame, String source_frame, Time time, Duration timeout, String error_msg){
        // poll for transform if timeout is set
        Time end_time = Time.fromMillis(System.currentTimeMillis()).add(timeout);
        while(Time.fromMillis(System.currentTimeMillis()).compareTo(end_time) < 0 &&
                !canTransform(target_frame, source_frame, time, error_msg)){
            try{
                Thread.sleep(10);//sleep for 10 ms
            }
            catch(InterruptedException e){
                return false;
            }
        }
        return canTransform(target_frame, source_frame, time, error_msg);
    }

    public boolean canTransform(String target_frame, Time target_time,
                                String source_frame, Time source_time,
                                String fixed_frame, Duration timeout){
        String ignored_error = "";
        return canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, timeout, ignored_error);
    }


    public boolean canTransform(String target_frame, Time target_time,
                                String source_frame, Time source_time,
                                String fixed_frame, Duration timeout, String error_msg){
        Time end_time = Time.fromMillis(System.currentTimeMillis()).add(timeout);
        while(Time.fromMillis(System.currentTimeMillis()).compareTo(end_time) < 0 &&
                !canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, error_msg)){
            try{
                Thread.sleep(10);//sleep for 10 ms
            }
            catch(InterruptedException e){
                return false;
            }
        }
        return canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, error_msg);
    }

    public Duration getCacheLength(){
        int[] ints = getNativeCacheLength();
        if(ints.length != 2){
            return new Duration();
        }
        return new Duration(ints[0], ints[1]);
    }


    /*
    *
    *
    * Start native method section
    *
    *
     */

    /** \brief Creates a new internal buffer with the new cache duration */
    private native void newBufferWithCacheDuration(int sec, int nsec);

    /** \brief Clear all data */
    public native void clear();

    /* Decided to break it out since I read that finding JNI method IDs are expensive, and ROSJava messages are  almost entirely calling getters */
    private native boolean setTransform(final String frame_id, final int sec, final int nsec, final String child_frame_id,
                                        final double tx, final double ty, final double tz,
                                        final double rw, final double rx, final double ry, final double rz, final String authority, final boolean is_static);


    /* Simplest JNI interface that passes by references are arrays of primitive types. */
    /* If this is too slow, could try passing by serializing */
    private native double[] lookupTransform(String target_frame, String source_frame, int sec, int nsec);

    private native double[] lookupTransform(String target_frame, int target_sec, int target_nsec,
                                            String source_frame, int source_sec, int source_nsec,
                                            String fixed_frame);

    private native boolean canTransform(String target_frame, String source_frame,
                                        int sec, int nsec, String error_msg);


     private native boolean canTransform(String target_frame, int target_sec, int target_nsec,
                                         String source_frame, int source_sec, int source_nsec,
                                         String fixed_frame, String error_msg);
    
    public native String allFramesAsYAML();

    public native String allFramesAsString();

    private native int[] getNativeCacheLength();
    /*
    *
    * Native testing methods
    *
     */
    public native void loadPR2Tree();

    public native String[] getPR2FrameIds();
}
