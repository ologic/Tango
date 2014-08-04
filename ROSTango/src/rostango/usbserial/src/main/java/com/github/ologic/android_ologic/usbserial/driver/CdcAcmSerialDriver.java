/* Copyright 2012 Google Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
 * USA.
 *
 * Project home page: http://code.google.com/p/usb-serial-for-android/
 */

package com.github.ologic.android_ologic.usbserial.driver;

import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbRequest;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.nio.ByteBuffer;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * USB CDC/ACM serial driver implementation.
 */
public class CdcAcmSerialDriver extends CommonUsbSerialDriver {

    private static final boolean DEBUG = false;
    private final String TAG = CdcAcmSerialDriver.class.getSimpleName();

    private UsbInterface mControlInterface = null;
    private UsbInterface mDataInterface = null;

    private UsbEndpoint mControlEndpoint = null;
    private UsbEndpoint mReadEndpoint = null;
    private UsbEndpoint mWriteEndpoint = null;

    private PipedInputStream mReaderInputStream = null;
    private PipedInputStream mWriterInputStream = null;
    private PipedOutputStream mReaderOutputStream = null;
    private PipedOutputStream mWriterOutputStream = null;

    private UsbRequest mReaderRequest = null;
    private UsbRequest mWriterRequest = null;

    private UsbReaderRunner mUsbReader = null;
    private UsbWriterRunner mUsbWriter = null;

    private Thread mUsbReaderThread = null;
    private Thread mUsbWriterThread = null;

    private static final int PIPE_SIZE = 4096;
    private static final int XFER_SIZE = 512;

    private static final int USB_RECIP_INTERFACE = 0x01;
    private static final int USB_RT_ACM = UsbConstants.USB_TYPE_CLASS | USB_RECIP_INTERFACE;

    private static final int SET_LINE_CODING = 0x20; // USB CDC 1.1 section 6.2
    @SuppressWarnings("unused")
	private static final int GET_LINE_CODING = 0x21;
    private static final int SET_CONTROL_LINE_STATE = 0x22;
    @SuppressWarnings("unused")
	private static final int SEND_BREAK = 0x23;

    private boolean mRts = false;
    private boolean mDtr = false;

    // Implements the class that reads the USB device.
    private class UsbReaderRunner implements Runnable {

        private boolean mRunning = false;

        @Override
        public void run() {

            if (DEBUG)
                Log.d(TAG, "Reader thread running...");

            // We want a "direct" byte buffer created where the buffer memory lives
            // outside the Java heap manager.  This is because garbage collection
            // occuring during a call into the native code transmitting a buffer
            // can cause problems if memory is shifted around.  We attempt to
            // avoid this issue using a "direct" byte buffer.
            ByteBuffer[] readerBuffers = new ByteBuffer[2];
            readerBuffers[0] = ByteBuffer.allocateDirect(XFER_SIZE);
            readerBuffers[1] = ByteBuffer.allocateDirect(XFER_SIZE);

            // Create an index that will toggle between the reader buffers.
            int readerIndex = 0;
            
            // We are now running.
            mRunning = true;

            // Initialize the reader request and place it on the queue.
            readerBuffers[readerIndex].clear();
            mReaderRequest.initialize(mConnection, mReadEndpoint);
            mReaderRequest.queue(readerBuffers[readerIndex], readerBuffers[readerIndex].capacity());

            // Keep going until told to stop.
            while (mRunning) {

                // Block waiting for request.
                UsbRequest response = mConnection.requestWait();

                // If we received an error then terminate running.
                if (response == null) {
                    mRunning = false;
                }

                // See if we were interrupted?
                if (Thread.interrupted()) {
                    // Abort running.
                    mRunning = false;
                }                
                
                // Make sure we are still running.
                if (mRunning) {
                    // Verify the reader response is correct.
                    if (response.getEndpoint().equals(mReadEndpoint)) {
                        // Determine the next reader index.
                        int nextIndex = (readerIndex + 1) & 1;

                        // Immediately queue the next reader request before we write the current
                        // buffer to the stream.  We do this to try and be sure to capture data
                        // that may otherwise be lost if the write to the stream takes too long.
                        readerBuffers[nextIndex].clear();
                        mReaderRequest.initialize(mConnection, mReadEndpoint);
                        mReaderRequest.queue(readerBuffers[nextIndex], readerBuffers[nextIndex].capacity());
                        
                        if (DEBUG)
                            Log.d(TAG, "Read " + readerBuffers[readerIndex].position() + " bytes from USB, now writing to client.");
 
                        try {
                            // Write the buffer to the reader output stream.
                            mReaderOutputStream.write(readerBuffers[readerIndex].array(), 0, readerBuffers[readerIndex].position());
                        } catch (IOException e) {
                            // Abort running.
                            mRunning = false;

                            Log.e(TAG, "Error writing buffer to output stream.");
                        }

                        // Update the reader index to the next index.
                        readerIndex = nextIndex;
                    }
                }
            }

            if (DEBUG)
                Log.d(TAG, "Close the mReaderOuputStream.");

            // Close the reader pipe.
            try {
                mReaderOutputStream.flush();
                mReaderOutputStream.close();
            } catch (IOException e) {
                // Ignore.
            }

            if (DEBUG)
                Log.d(TAG, "Reader thread finished.");
        }

        public void shutdown() {
            mRunning = false;
        }
    }

    // Implements the class that writes the USB device.
    //
    // Note: In this method we use the synchronous bulkTransfer
    // interface on the UsbDeviceConnection rather than the 
    // asynchronous interface used in the reader.  The reason
    // for this is because requestWait method is not thread 
    // safe and to synchronize both the reader and writer 
    // could potentially slow things down.  The reason for this
    // is explained in more detail here:
    // http://stackoverflow.com/questions/9644415/usbrequest-queue-crashing-android-3-1-application
    private class UsbWriterRunner implements Runnable {

        private boolean mRunning = false;

        @Override
        public void run() {
            int count = 0;

            if (DEBUG)
                Log.d(TAG, "Writer thread running...");

            // We want a "direct" byte buffer created where the buffer memory lives
            // outside the Java heap manager.  This is because garbage collection
            // occuring during a call into the native code transmitting a buffer
            // can cause problems if memory is shifted around.  We attempt to
            // avoid this issue using a "direct" byte buffer.
            ByteBuffer writerBuffer = ByteBuffer.allocateDirect(XFER_SIZE);

            // We are now running.
            mRunning = true;

            // Keep going until told to stop.
            while (mRunning) {

                // Block waiting to read the input stream.
                try {
                    count = mWriterInputStream.read(writerBuffer.array());
                } catch (IOException e) {
                    // Abort running.
                    mRunning = false;
                }

                // See if we were interrupted?
                if (Thread.interrupted()) {
                    // Abort running.
                    mRunning = false;
                }                

                // Do we have data to write to the USB?
                if (mRunning && (count > 0)) {

                    if (DEBUG)
                        Log.d(TAG, "Read " + count + " bytes from client, now writing to USB.");

                    // Synchronous transfer with a reasonable 0.1 second timeout.  The array() 
                    // method may not work on versions of Android prior to 4.2. 
                    count = mConnection.bulkTransfer(mWriteEndpoint, writerBuffer.array(), count, 100);

                    // If we have an error then terminate this runner.
                    if (count < 0) {
                        // Abort running.
                        mRunning = false;
                    }
                }
            }

            if (DEBUG)
                Log.d(TAG, "Close the mWriterInputStream.");

            // Close the writer pipe.
            try {
                mWriterInputStream.close();
            } catch (IOException e) {
                // Ignore.
            }

            if (DEBUG)
                Log.d(TAG, "Writer thread finished.");
        }

        public void shutdown() {
            mRunning = false;
        }
    }

    public CdcAcmSerialDriver(UsbDevice device, UsbDeviceConnection connection) {
        super(device, connection);
    }

    /**
     * Opens and initializes the device as a USB serial device. Upon success,
     * caller must ensure that {@link #close()} is eventually called.
     *
     * @throws IOException on error opening or initializing the device.
     */
    public void open() throws IOException {
        if (DEBUG)
            Log.d(TAG, "claiming interfaces, count=" + mDevice.getInterfaceCount());

        if (DEBUG)
            Log.d(TAG, "Claiming control interface.");
        mControlInterface = mDevice.getInterface(0);
        if (DEBUG)
            Log.d(TAG, "Control iface=" + mControlInterface);
        // class should be USB_CLASS_COMM

        if (!mConnection.claimInterface(mControlInterface, true)) {
            throw new IOException("Could not claim control interface.");
        }
        mControlEndpoint = mControlInterface.getEndpoint(0);
        if (DEBUG)
            Log.d(TAG, "Control endpoint direction: " + mControlEndpoint.getDirection());

        if (DEBUG)
            Log.d(TAG, "Claiming data interface.");
        mDataInterface = mDevice.getInterface(1);
        if (DEBUG)
            Log.d(TAG, "data iface=" + mDataInterface);
        // class should be USB_CLASS_CDC_DATA

        if (!mConnection.claimInterface(mDataInterface, true)) {
            throw new IOException("Could not claim data interface.");
        }
        mReadEndpoint = mDataInterface.getEndpoint(1);
        if (DEBUG)
            Log.d(TAG, "Read endpoint direction: " + mReadEndpoint.getDirection());
        mWriteEndpoint = mDataInterface.getEndpoint(0);
        if (DEBUG)
            Log.d(TAG, "Write endpoint direction: " + mWriteEndpoint.getDirection());

        // Create the input side of the pipes.
        mReaderInputStream = new PipedInputStream(PIPE_SIZE);
        mWriterInputStream = new PipedInputStream(PIPE_SIZE);

        // Now create the output side of the pipes.
        try {
            // Create the output side of the pipe.
            mReaderOutputStream = new PipedOutputStream(mReaderInputStream);
            mWriterOutputStream = new PipedOutputStream(mWriterInputStream);
        } catch (IOException e) {
            if (DEBUG)
                Log.e(TAG, "Failed creating USB driver pipes.");
            try {
                if (mReaderInputStream != null) {
                    mReaderInputStream.close();
                    mReaderInputStream = null;
                }
                if (mWriterInputStream != null) {
                    mWriterInputStream.close();
                    mWriterInputStream = null;
                }
                if (mReaderOutputStream != null) {
                    mReaderOutputStream.flush();
                    mReaderOutputStream.close();
                    mReaderOutputStream = null;
                }
                if (mWriterOutputStream != null) {
                    mWriterOutputStream.flush();
                    mWriterOutputStream.close();
                    mWriterOutputStream = null;
                }
            } catch (IOException e2) {
                // Ignore.
            }
            throw new IOException("Could not create driver pipes.");
        }

        // Create the USB request objects used in the reader/writer threads.
        mReaderRequest = new UsbRequest();
        mWriterRequest = new UsbRequest();

        // Start reader and writer threads.
        mUsbReader = new UsbReaderRunner();
        mUsbWriter = new UsbWriterRunner();
        mUsbReaderThread = new Thread(mUsbReader);
        mUsbWriterThread = new Thread(mUsbWriter);
        mUsbReaderThread.setPriority(Thread.MAX_PRIORITY);
        mUsbReaderThread.start();
        mUsbWriterThread.start();
    }

    /**
     * Closes the serial device.
     *
     * @throws IOException on error closing the device.
     */
    public void close() throws IOException {

        if (DEBUG)
            Log.d(TAG, "Disabling reader and writer running flags.");

        // Set the shutdown flags.
        mUsbReader.shutdown();
        mUsbWriter.shutdown();

        if (DEBUG)
            Log.d(TAG, "Cancelling pending request operations.");

        // Cancel any pending operations.
        mReaderRequest.cancel();
        mWriterRequest.cancel();

        if (DEBUG)
            Log.d(TAG, "Close the request resources.");

        // Close any request resources.
        mReaderRequest.close();
        mWriterRequest.close();
    }

    /**
     * Returns the input stream to read data from the USB device.
     *
     */
    public InputStream getInputStream() {
        return mReaderInputStream;
    }

    /**
     * Returns the output stream to write data to the USB device.
     *
     */
    public OutputStream getOutputStream() {
        return mWriterOutputStream;
    }

    private int sendAcmControlMessage(int request, int value, byte[] buf) {
        return mConnection.controlTransfer(
                USB_RT_ACM, request, value, 0, buf, buf != null ? buf.length : 0, 5000);
    }

    public void setParameters(int baudRate, int dataBits, int stopBits, int parity) {
        byte stopBitsByte;
        switch (stopBits) {
            case STOPBITS_1:
                stopBitsByte = 0;
                break;
            case STOPBITS_1_5:
                stopBitsByte = 1;
                break;
            case STOPBITS_2:
                stopBitsByte = 2;
                break;
            default:
                throw new IllegalArgumentException("Bad value for stopBits: " + stopBits);
        }

        byte parityBitesByte;
        switch (parity) {
            case PARITY_NONE:
                parityBitesByte = 0;
                break;
            case PARITY_ODD:
                parityBitesByte = 1;
                break;
            case PARITY_EVEN:
                parityBitesByte = 2;
                break;
            case PARITY_MARK:
                parityBitesByte = 3;
                break;
            case PARITY_SPACE:
                parityBitesByte = 4;
                break;
            default:
                throw new IllegalArgumentException("Bad value for parity: " + parity);
        }

        byte[] msg = {
                (byte) (baudRate & 0xff),
                (byte) ((baudRate >> 8) & 0xff),
                (byte) ((baudRate >> 16) & 0xff),
                (byte) ((baudRate >> 24) & 0xff),
                stopBitsByte,
                parityBitesByte,
                (byte) dataBits
        };
        sendAcmControlMessage(SET_LINE_CODING, 0, msg);
    }

    /**
     * @throws IOException
     */
    public boolean getCD() throws IOException {
        return false; // TODO
    }

    /**
     * @throws IOException
     */
    public boolean getCTS() throws IOException {
        return false; // TODO
    }

    /**
     * @throws IOException
     */
    public boolean getDSR() throws IOException {
        return false; // TODO
    }

    /**
     * @throws IOException
     */
    public boolean getDTR() throws IOException {
        return mDtr;
    }

    /**
     * @throws IOException
     */
    public void setDTR(boolean value) throws IOException {
        mDtr = value;
        setDtrRts();
    }

    /**
     * @throws IOException
     */
    public boolean getRI() throws IOException {
        return false; // TODO
    }

    /**
     * @throws IOException
     */
    public boolean getRTS() throws IOException {
        return mRts;
    }

    /**
     * @throws IOException
     */
    public void setRTS(boolean value) throws IOException {
        mRts = value;
        setDtrRts();
    }

    private void setDtrRts() {
        int value = (mRts ? 0x2 : 0) | (mDtr ? 0x1 : 0);
        sendAcmControlMessage(SET_CONTROL_LINE_STATE, value, null);
    }

    public static Map<Integer, int[]> getSupportedDevices() {
        final Map<Integer, int[]> supportedDevices = new LinkedHashMap<Integer, int[]>();
        supportedDevices.put(Integer.valueOf(UsbId.VENDOR_ARDUINO),
                new int[] {
                        UsbId.ARDUINO_UNO,
                        UsbId.ARDUINO_UNO_R3,
                        UsbId.ARDUINO_MEGA_2560,
                        UsbId.ARDUINO_MEGA_2560_R3,
                        UsbId.ARDUINO_SERIAL_ADAPTER,
                        UsbId.ARDUINO_SERIAL_ADAPTER_R3,
                        UsbId.ARDUINO_MEGA_ADK,
                        UsbId.ARDUINO_MEGA_ADK_R3,
                        UsbId.ARDUINO_LEONARDO,
                });
        supportedDevices.put(Integer.valueOf(UsbId.VENDOR_VAN_OOIJEN_TECH),
                new int[] {
                    UsbId.VAN_OOIJEN_TECH_TEENSYDUINO_SERIAL,
                });
        supportedDevices.put(Integer.valueOf(UsbId.VENDOR_ATMEL),
                new int[] {
                    UsbId.ATMEL_LUFA_CDC_DEMO_APP,
                });
        supportedDevices.put(Integer.valueOf(UsbId.VENDOR_LEAFLABS),
                new int[] {
                    UsbId.LEAFLABS_MAPLE,
                });
        return supportedDevices;
    }
}
