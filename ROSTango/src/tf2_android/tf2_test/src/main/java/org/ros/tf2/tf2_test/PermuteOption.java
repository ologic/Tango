package org.ros.tf2.tf2_test;

import java.util.Iterator;
import java.util.ListIterator;
import java.util.Vector;

public class PermuteOption<T> {
    private Vector<T> options_;
    private Vector<T> output_;
    ///< @TODO iterator?
    Iterator<T> current_element_;


    public PermuteOption(Vector<T> options, Vector<T> output){
        options_ = options;
        output_ = output;
        reset();
    }

    public void reset(){
        current_element_ = options_.iterator();
        output_.set(0,current_element_.next());
    }

    public boolean step(){
        if(current_element_.hasNext()){
            output_.set(0,current_element_.next());
            return true;
        }
        return false;
    }
}
