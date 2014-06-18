package org.ros.tf2.insert_benchmark;

import android.app.Activity;
import android.os.Bundle;
import android.os.AsyncTask;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.UUID;
import java.util.Vector;

import geometry_msgs.Quaternion;
import geometry_msgs.Transform;
import geometry_msgs.TransformStamped;
import geometry_msgs.Vector3;
import tf2_msgs.TFMessage;

import org.ros.tf2_ros.Buffer;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

public class BenchmarkActivity extends Activity
{
    private volatile boolean mRunBenchmark = false;
    private TextView mTV;
    private volatile int mInsertionLoops = 1;
    private volatile int mNumberFrames = 100;
    private volatile int mBranchingFactor = 3;

    private class LongOperation extends AsyncTask<String, String, String> {

        public String generateString(Random rng, String characters, int length)
        {
            char[] text = new char[length];
            for (int i = 0; i < length; i++)
            {
                text[i] = characters.charAt(rng.nextInt(characters.length()));
            }
            return new String(text);
        }

        String[] generateUniqueFrames(int nFrames){
            String[] out = new String[nFrames];
            Random r = new Random();
            String chars = "abcdefghijklmnopqrstuvwxyz_";
            for (int i = 0; i < nFrames; i++){
                //out[i] = UUID.randomUUID().toString();
                out[i] = generateString(r, chars, 25);
            }
            return out;
        }


        private int parentIndex(double childIndex, double branchFactor){
            int ret = (int)(childIndex/(branchFactor+0.00001));
            //Log.w("INDEX_PRINT", "Index of: " + Integer.toString(ret) + " for input: " + Double.toString(childIndex));
            return ret;
        }

        private TFMessage createTF2Message(String[] uniqueFrames, Time stamp){
            Random r = new Random();
            NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
            MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();
            TFMessage tfs = mMessageFactory.newFromType(TFMessage._TYPE);
            List<TransformStamped> tfl = tfs.getTransforms();
            for(int i = 1; i < uniqueFrames.length; i++){ // Skip first frame
                TransformStamped ts = mMessageFactory.newFromType(TransformStamped._TYPE);
                ts.getHeader().setFrameId(uniqueFrames[parentIndex(i, mBranchingFactor)]);
                ts.getHeader().setStamp(stamp);
                ts.setChildFrameId(uniqueFrames[i]);
                Transform t = ts.getTransform();
                Vector3 tr = t.getTranslation();
                tr.setX(r.nextDouble());
                tr.setY(r.nextDouble());
                tr.setZ(r.nextDouble());
                Quaternion q = t.getRotation();
                double w = r.nextDouble();
                double x = r.nextDouble();
                double y = r.nextDouble();
                double z = r.nextDouble();
                double mag = Math.sqrt(w*w+x*x+y*y+z*z);
                if(mag < 1.e-5){ // Handle zero magnitude case
                    w = 1.0;
                    mag = 1.0;
                }
                q.setW(w/mag);
                q.setX(x/mag);
                q.setY(y/mag);
                q.setZ(z/mag);

                // Add to vector
                tfl.add(ts);
            }
            tfs.setTransforms(tfl);
            return tfs;
        }

        @Override
        protected String doInBackground(String... params) {
            Buffer mBuffer = new Buffer();

            publishProgress("Generating TF2 Message Tree");
            // Generate unique frames
            String[] uniqueFrames;
            int actual_frames = 0;
            if(mNumberFrames == 0){ // Use PR2 frame_ids
                uniqueFrames = mBuffer.getPR2FrameIds();
            } else { // Random Strings
                uniqueFrames = generateUniqueFrames(mNumberFrames);
            }
            int actualNumberFrames = uniqueFrames.length;

            double dt = (20.0-10.0)/(double)mInsertionLoops;
            TFMessage[] tfArray = new TFMessage[mInsertionLoops];
            for(int i = 0; i < mInsertionLoops; i++){
                Time newTime = new Time(10.0+i*dt); // 10.0 to 20.0s
                tfArray[i] = createTF2Message(uniqueFrames, newTime);
            }

            String progressString = Integer.toString(mInsertionLoops) + " whole tree inserts - on "
                    + Integer.toString(actualNumberFrames) + " total transforms"
                    + " (" + Integer.toString(mInsertionLoops*actualNumberFrames) + " total inserts).";
            publishProgress(progressString);

            String authority = "unknown";
            boolean is_static = false;
            mBuffer.clear();
;
            Time start = new Time().fromMillis(System.currentTimeMillis());

            int aLength = tfArray.length;
            for(int i = 0; i < aLength; i++){
                final List<TransformStamped> tfl = tfArray[i].getTransforms();
                int lLength = tfl.size();
                for(int j = 0; j < lLength; j++){
                    //Log.w("INSERT_BENCHMARK", tf.getHeader().getFrameId() + "::" + tf.getChildFrameId() + "::" + Double.toString(tf.getHeader().getStamp().toSeconds()));
                    mBuffer.setTransform(tfl.get(j), authority, is_static);
                }

            }

            Time end = new Time().fromMillis(System.currentTimeMillis());

            if(mRunBenchmark == false){
                return "Benchmark did not completely finish.";
            }

            Duration diff = end.subtract(start);
            double diffs = (double)(diff.totalNsecs())/1.e9;

            progressString += "\n\nStart time: " + Double.toString(start.toSeconds()) + " \nEnd time: "
                    + Double.toString(end.toSeconds()) + "\nDiff: " + Double.toString(diffs) + "\nAvg: "
                    + Double.toString(diffs/(mInsertionLoops*actualNumberFrames));
            mRunBenchmark = false;
            return progressString;
        }

        @Override
        protected void onPostExecute(String result) {
            mTV.setText(result);
        }

        @Override
        protected void onPreExecute() {
        }

        @Override
        protected void onProgressUpdate(String... values) {
            mTV.setText(values[0]);
        }
    }

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);

        mTV = (TextView)findViewById(R.id.outputText);

        final Button button = (Button) findViewById(R.id.startButton);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if(mRunBenchmark == false){
                    mRunBenchmark = true;
                    EditText et = (EditText)findViewById(R.id.editNumLookups);
                    mInsertionLoops = Integer.parseInt(et.getText().toString());
                    EditText et2 = (EditText)findViewById(R.id.editFrames);
                    mNumberFrames = Integer.parseInt(et2.getText().toString());
                    EditText et3 = (EditText)findViewById(R.id.editBranching);
                    mBranchingFactor = Integer.parseInt(et3.getText().toString());
                    new LongOperation().execute();
                }
            }
        });
    }

    @Override
    public void onPause(){
        mRunBenchmark = false;
        super.onPause();
    }
}
