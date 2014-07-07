/*
 * Copyright 2013 Motorola Mobility LLC. Part of ProjectTango.
 * CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
 *
 */

package com.ologicinc.rostango.tango_depthimage;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.RectF;
import android.hardware.Camera;
import android.hardware.Camera.PreviewCallback;
import android.util.AttributeSet;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import android.view.SurfaceView;

import java.util.List;



/**
 * DepthViewSurface is the surface that the camera object uses to draw the image
 * and depth data on. The Camera object returns a superframe with each onPreview
 * callback as the 'data' parameter. The 'SuperFrame' contain image data and
 * depth data (and other data as well). The depth data corresponds to the image
 * data in x,y space (although the depth data is at a lower resolution).
 *
 * An easy way to map the visible image to the lower resolution depth buffer is
 * to scale the depth buffer by 4x. This means both buffers will be 1280 x 720.
 * Alternately scale the visible image by 1/4x so both buffers are 320 x 180.
 *
 * In this app the depth data is used to draw false color images that overlay
 * the cameras visible image.
 *
 * @author Michael F. Winter (robotmikew@gmail.com)
 */
public class DepthViewSurface extends SurfaceView implements Callback,
        PreviewCallback {

    // The width of the depth of the scan (in mm).
    public final static int SCAN_TOLERANCE_MM = 30;
    // This sets the minimum depth in the depth buffer that gets scanned.
    public final static int SCAN_NEARLIMIT_MM = 800;
    // This sets the maximum depth in the depth buffer that gets scanned.
    public final static int SCAN_FARLIMIT_MM = 5000;
    // This scales the average depth to set far scan maximum.
    public final static int MAX_DEPTH_SCALE = 3;
    // This is the increment in depth between successive scans.
    public final static int SCAN_DEPTH_INC_MM = 10;
    // These define distances from scan depth to draw colors.
    public final static int DIST_FROM_SCAN_NEAR = 10;
    public final static int DIST_FROM_SCAN_MEDIUM = 20;
    public final static int DIST_FROM_SCAN_FAR = 50;

    // Camera and surfaces.
    private Camera mCamera = null;
    private SurfaceHolder mSurfaceHolder = null;
    // This bitmap has color data, color is based on depth buffer values.
    private int[] mDepthColorPixels = null;
    // Create a bitmap to draw to screen.
    private Bitmap mDepthBitmapARGB8888 = Bitmap.createBitmap(
            SuperFrame.DB_WIDTH, SuperFrame.DB_HEIGHT, Bitmap.Config.ARGB_8888);
    // This is the current scan depth.
    int mScanDepthMm = SCAN_NEARLIMIT_MM;
    // These colors are used in drawing the scan line.
    static int mColorNear = 0; // This color is drawn nearest to scan depth.
    static int mColorMedium = 0; // This color is between near and far.
    static int mColorFar = 0; // This color is drawn farthest from scan depth.


    private RawImageListener rawImageListener;
    private int[] rawDepthData= new int[57600];
    /**
     * Create surface and an array to hold color pixels.
     */
    private void init() {
        setWillNotDraw(false); // Enable view to draw.
        mSurfaceHolder = getHolder();
        // Create an array the same size as depth buffer.
        mDepthColorPixels = new int[SuperFrame.DB_WIDTH * SuperFrame.DB_HEIGHT];
        // Set colors to draw scan line.
        mColorNear = getResources().getColor(R.color.scan_yellow);
        mColorMedium = getResources().getColor(R.color.scan_lime_green);
        mColorFar = getResources()
                .getColor(R.color.scan_lime_green_transparent);
    }

    public DepthViewSurface(Context context) {
        super(context);
        init();
    }

    public DepthViewSurface(Context context, AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public void setRawImageListener(RawImageListener rawImageListener) {
        this.rawImageListener = rawImageListener;
    }

    /**
     * Set camera mode then start the camera preview.
     */
    public void startCamera() {
        mSurfaceHolder.addCallback(this);
        try {
            mCamera = Camera.open();
        } catch (RuntimeException e) {
            return;
        }
        Camera.Parameters parameters = mCamera.getParameters();
        // Note: sf modes are "all", "big-rgb", "small-rgb", "depth", "ir".
        parameters.set("sf-mode", "big-rgb");  // Show the RGB image.
        mCamera.setParameters(parameters);
        mCamera.startPreview();
    }

    public void stopCamera() {
        if (mCamera != null) {
            mCamera.stopPreview();
            mCamera.setPreviewCallback(null);
            mSurfaceHolder.removeCallback(this);
            mCamera.release();
            mCamera = null;
        }
    }

    /**
     * Change preview size when width and height change. This can occur when the
     * device orientation changes (user rotates phone).
     */
    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width,
                               int height) {
        if (mCamera != null && mSurfaceHolder.getSurface() != null) {
            try {
                mCamera.setPreviewCallback(this);
            } catch (Throwable t) {
            }
            try {
                mCamera.setPreviewDisplay(mSurfaceHolder);
            } catch (Exception e) {
                return;
            }
            // Set the preview size.
            Camera.Parameters parameters = mCamera.getParameters();
            Camera.Size size = getBestPreviewSize(width, height,
                    parameters.getSupportedPreviewSizes());
            if (size != null) {
                parameters.setPreviewSize(size.width, size.height);
                mCamera.setParameters(parameters);
            }
            mCamera.startPreview();
        }
    }

    /**
     * Draw false color overlay on top of camera's image during preview. The
     * data[] parameter is a Superframe, it contains the depth buffer.
     */
    @Override
    public void onPreviewFrame(byte[] data, Camera cameraPreFrame) {
        // Get start index of depth buffer in the Superframe.
        int depthBufferByteIndex = SuperFrame.SF_START_INDEX_DEPTH;
        int depthSum = 0;  // The sum of all depths in the depth buffer.
        int depthAvg = 0;  // The average depth of the depth buffer.

        // Loop over the depth buffer, draw pixels that are at or near scan
        // depth.
        for (int bitmapIndex = 0; bitmapIndex < SuperFrame.DB_SIZE; ++bitmapIndex) {
            // Read the Superframe to get pixel depth.
            // Depth is contained in two bytes, convert bytes to an Int.
            int pixDepthMm = ((((int) data[depthBufferByteIndex + 1]) << 8) & 0xff00)
                    | (((int) data[depthBufferByteIndex]) & 0x00ff);

            rawDepthData[bitmapIndex] = pixDepthMm;

            // Set color based on depth.
            mDepthColorPixels[bitmapIndex] = getPixelColor(mScanDepthMm,
                    pixDepthMm);
            depthBufferByteIndex += 2;  // Increment the depth index to next Int.
            depthSum += pixDepthMm;
        }
        depthAvg = (depthSum / SuperFrame.DB_SIZE);
        mScanDepthMm = getDepthToScan(mScanDepthMm, depthAvg);
        postInvalidate();  // Cause a redraw.

        if (rawImageListener != null)
            rawImageListener.onNewRawImage(rawDepthData, SuperFrame.DB_WIDTH, SuperFrame.DB_HEIGHT);
    }

    /**
     * Return next depth to scan.
     *
     * @param depthCurrentScanMm
     *            The current depth being scanned.
     * @param depthAvgMm
     *            Average of all depths in depth buffer.
     * @return Returns the depth based on predefined near/far limits or on scene
     *         depth.
     */
    private int getDepthToScan(int depthCurrentScanMm, int depthAvgMm) {
        // Set max depth based on average depth (avoids scanning nothing).
        int depthBufferMaxMm = depthAvgMm * MAX_DEPTH_SCALE;
        // Increment scan depth.
        int depthOfScanMm = depthCurrentScanMm + SCAN_DEPTH_INC_MM;
        // Check if scan has nothing much more to show.
        if (depthOfScanMm > depthBufferMaxMm) {
            depthOfScanMm = SCAN_NEARLIMIT_MM;
        } else if (depthOfScanMm > SCAN_FARLIMIT_MM) {
            depthOfScanMm = SCAN_NEARLIMIT_MM; // Check if scan reached limit.
        }
        return (depthOfScanMm);
    }

    /**
     * Returns a color based on how far a pixel's depth is from scan depth.
     *
     * @param depthToShowMm
     *            This is the depth being scanned.
     * @param depthCurrentPixelMm
     *            This is the depth of the pixel being drawn. Note: Color values
     *            are selected for aesthetic purposes.
     * @return Returns a color.
     */
    private int getPixelColor(int depthToShowMm, int depthCurrentPixelMm) {
        int pixelColor = android.R.color.transparent; // The default color.
        int distToDepthMm = Math.abs(depthToShowMm - depthCurrentPixelMm);
        if (distToDepthMm < DIST_FROM_SCAN_NEAR) {
            pixelColor = mColorNear;
        } else if (distToDepthMm < DIST_FROM_SCAN_MEDIUM) {
            pixelColor = mColorMedium;
        } else if (distToDepthMm < DIST_FROM_SCAN_FAR) {
            pixelColor = mColorFar;
        }
        return (pixelColor);
    }

    /**
     * Draw false color depth overlay on top of camera preview.
     */
    @Override
    public void onDraw(Canvas canvas) {
        // Fill the overlay bitmap with the latest overlay colors.
        mDepthBitmapARGB8888.setPixels(mDepthColorPixels, 0,
                SuperFrame.DB_WIDTH, 0, 0, SuperFrame.DB_WIDTH,
                SuperFrame.DB_HEIGHT);
        // Draw the overlay bitmap onto the canvas.
        canvas.drawBitmap(mDepthBitmapARGB8888, null, new RectF(0.0f, 0.0f,
                (float) getWidth(), (float) getHeight()), null);
    }

    /**
     * Helper method to select the best preview size from a list of choices.
     *
     * @param width
     *            Desired width
     * @param height
     *            Desired height
     * @param sizes
     *            List of possible preview sizes
     * @return Best preview size
     */
    private Camera.Size getBestPreviewSize(int width, int height,
                                           List<Camera.Size> sizes) {
        Camera.Size result = null;
        for (Camera.Size size : sizes) {
            if (size.width <= width && size.height <= height) {
                if (result == null) {
                    result = size;
                } else {
                    int resultArea = result.width * result.height;
                    int newArea = size.width * size.height;
                    if (newArea > resultArea) {
                        result = size;
                    }
                }
            }
        }
        return (result);
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
    }
}
