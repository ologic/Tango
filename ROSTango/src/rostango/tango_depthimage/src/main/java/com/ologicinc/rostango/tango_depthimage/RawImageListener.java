package com.ologicinc.rostango.tango_depthimage;

/**
 * Created by Rohan Agrawal on 7/3/14.
 */

interface RawImageListener {
    void onNewRawImage(int[] data, int width, int height);
}
