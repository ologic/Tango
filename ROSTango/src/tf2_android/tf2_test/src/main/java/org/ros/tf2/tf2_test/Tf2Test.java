package org.ros.tf2.tf2_test;

import android.app.Activity;
import android.os.AsyncTask;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import org.apache.http.util.ExceptionUtils;

public class Tf2Test extends Activity
{

    private volatile boolean mRunTests = false;
    private TextView mTV;

    private class LongOperation extends AsyncTask<String, String, String> {

        String progressString = "Started TF2 Tests....";

        private void updateResultPassed(){
            if(!mRunTests){ // Throw to cancel tests
                throw new RuntimeException();
            }
            progressString += "\n\t\tPassed!";
            publishProgress(progressString);
        }

        private void updateResultFailed(Exception e){
            progressString += "\n\n\n\t\tFailed!!!";
            progressString += "\n\t\t\t " + org.apache.commons.lang.exception.ExceptionUtils.getStackTrace(e);
            progressString += "\n\t\t\tmsg: " + e.getMessage();
            publishProgress(progressString);
            this.cancel(true);
        }

        @Override
        protected String doInBackground(String... params) {
            publishProgress(progressString);

            progressString += "\nStarting noInsertOnSelfTransform().";
            publishProgress(progressString);
            try{
                Tf2RunTests.noInsertOnSelfTransform();
                updateResultPassed();
            }  catch (Exception e){
                updateResultFailed(e);
                return progressString;
            }

            progressString += "\nStarting noInsertWithNaN().";
            publishProgress(progressString);
            try{
                Tf2RunTests.noInsertWithNaN();
                updateResultPassed();
            }  catch (Exception e){
                updateResultFailed(e);
                return progressString;
            }

            progressString += "\nStarting noInsertWithNoFrameID().";
            publishProgress(progressString);
            try{
                Tf2RunTests.noInsertWithNoFrameID();
                updateResultPassed();
            }  catch (Exception e){
                updateResultFailed(e);
                return progressString;
            }

            progressString += "\nStarting noInsertWithNoParentID().";
            publishProgress(progressString);
            try{
                Tf2RunTests.noInsertWithNoParentID();
                updateResultPassed();
            }  catch (Exception e){
                updateResultFailed(e);
                return progressString;
            }

            progressString += "\nStarting iConfiguration().";
            publishProgress(progressString);
            try{
                Tf2RunTests.iConfiguration();
                updateResultPassed();
            }  catch (Exception e){
                updateResultFailed(e);
                return progressString;
            }

            progressString += "\nStarting one_link_configuration().";
            publishProgress(progressString);
            try{
                Tf2RunTests.one_link_configuration();
                updateResultPassed();
            } catch (Exception e){
                updateResultFailed(e);
                return progressString;
            }

            mRunTests = false;
            return progressString + "\n\nPassed all tests!!!";
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

        mTV = (TextView)findViewById(R.id.textView);

        final Button button = (Button) findViewById(R.id.button);
        button.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if(!mRunTests){
                    mRunTests = true;
                    new LongOperation().execute();
                }
            }
        });
    }
}
