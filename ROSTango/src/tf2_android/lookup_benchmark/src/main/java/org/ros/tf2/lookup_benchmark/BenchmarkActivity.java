package org.ros.tf2.lookup_benchmark;

import android.app.Activity;
import android.os.Bundle;
import android.os.AsyncTask;
import android.os.Looper;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.util.Random;

import geometry_msgs.TransformStamped;
import org.ros.tf2_ros.Buffer;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.node.NodeConfiguration;

public class BenchmarkActivity extends Activity
{
    private volatile boolean mRunBenchmark = false;
    private TextView mTV;
    private volatile int mLookups = 1;

    private class LongOperation extends AsyncTask<String, String, String> {

        @Override
        protected String doInBackground(String... params) {
            publishProgress("Inserting PR2 TF Tree");
            Buffer mBuffer = new Buffer();
            NodeConfiguration mNodeConfiguration = NodeConfiguration.newPrivate();
            MessageFactory mMessageFactory = mNodeConfiguration.getTopicMessageFactory();

            // Insertion
            mBuffer.loadPR2Tree();

            publishProgress("Getting list of PR2 Frame IDs");
            // Lookup benchmark
            String[] pr2_frames = mBuffer.getPR2FrameIds();

            Random r = new Random();

            final double st = 19.0;
            final double et = 20.0;
            Time lookup_time = new Time();
            String target;
            String source;
            double num_lookups = (double)mLookups;
            TransformStamped tfs = mMessageFactory.newFromType(TransformStamped._TYPE);

            String progressString = Double.toString(num_lookups) + " random lookups - on " + Integer.toString(pr2_frames.length) + " total_frames";
            publishProgress(progressString);
            Time start = new Time().fromMillis(System.currentTimeMillis());

            for(int i = 0; i < mLookups && mRunBenchmark; i++){
                lookup_time = new Time(st + ((et-st)*r.nextDouble()));
                target = pr2_frames[r.nextInt(pr2_frames.length)];
                source = pr2_frames[r.nextInt(pr2_frames.length)];

                boolean result = mBuffer.lookupTransform(tfs, target, source, lookup_time);
                if(result == false){
                    mRunBenchmark = false;
                    return "Error: Could not lookup frame " + target + " to " + source +"!";
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
                    + Double.toString(diffs/num_lookups);
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
                    mLookups = Integer.parseInt(et.getText().toString());
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
