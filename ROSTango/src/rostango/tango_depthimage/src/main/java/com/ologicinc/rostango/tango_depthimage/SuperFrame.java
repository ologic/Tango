/*
 * Copyright 2013 Motorola Mobility LLC. Part of ProjectTango.
 * CONFIDENTIAL. AUTHORIZED USE ONLY. DO NOT REDISTRIBUTE.
 *
 */

package com.ologicinc.rostango.tango_depthimage;

/**
 * This class defines a Superframe for Peanut mobile device. A superframe is an
 * array of image, depth and other data that is associated with the camera
 * object. The format is YUV420SP.
 *
 * @author Michael F. Winter (robotmikew@gmail.com)
 */

class SuperFrame {

  private SuperFrame() {
  } // Prevent instantiation of class.

  // The Superframe's width is 1280 pixels, where each pixel is a byte.
  public final static int SF_WIDTH = 1280;
  // The Superframe's height is 1168 pixels (height can be thought of as
  //   pixels or lines).
  public final static int SF_HEIGHT = 1168;
  // Header data is stored in first 16 lines of a Superframe.
  public final static int SF_LINES_HEADER = 16;
  // A Y plane of wide angle lens (Y contains the luminance of a YUV image)
  // is stored in next 240 lines.
  public final static int SF_LINES_SMALLIMAGE = 240;
  // An Image pyramid is stored in next 96 lines, currently a placeholder.
  public final static int SF_LINES_PYRAMID = 96;
  // A Depth buffer is stored in next 96 lines.
  public final static int SF_LINES_DEPTH = 96;
  // The Y plane of the 4 MP standard field of view camera YUV image
  //   is stored in next 720 lines.
  public final static int SF_LINES_BIGIMAGE = 720;
  public final static int SF_START_LINE_DEPTH = SF_LINES_HEADER
      + SF_LINES_SMALLIMAGE + SF_LINES_PYRAMID;
  public final static int SF_START_INDEX_DEPTH = SF_START_LINE_DEPTH
      * SF_WIDTH;
  public final static int SF_START_LINE_BIGIMAGEY = SF_START_LINE_DEPTH
      + SF_LINES_DEPTH;
  // Size of the Y portion of the 4 MP standard field YUV image.
  public final static int SF_BIG_SIZEY = SF_WIDTH * SF_LINES_BIGIMAGE;
  // Number of bytes in YUV bitmap.
  public final static int SF_BIG_SIZEYUV = SF_BIG_SIZEY + (SF_BIG_SIZEY / 2);

  // The depth buffer is contained in a Superframe as a 320x180 array
  //   of 16 bit (2 contiguous bytes) values.
  public final static int DB_WIDTH = 320;
  public final static int DB_HEIGHT = 180;
  // DB_SIZE is the number of elements, not the number of bytes, in the depth
  //   buffer.
  // An element can be though of as a pixel, where the value is the depth.
  // A single element is a 2 byte int in the Superframe.
  public final static int DB_SIZE = DB_WIDTH * DB_HEIGHT;
}
