package org.ros.tf2.tf2_test;

import java.util.Vector;

public class Permuter {

    private Vector<PermuteOption> options_ = new Vector<PermuteOption>();

    public <T> void addOptionSet(Vector<T> values, Vector<T> output){
        options_.add(new PermuteOption<T>(values, output));
        reset();
    }

    public void reset(){
        for(PermuteOption option : options_){
            option.reset();
        }
    }

    public boolean step(){
        for(PermuteOption option : options_){
            if(option.step()){
                return true;
            } else {
                option.reset();
            }
        }
        return false;
    }
}

