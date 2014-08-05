/* Copyright 2011 Google Inc.
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

import com.github.ologic.android_ologic.usbserial.util.HexDump;

/**
 * A {@link CommonUsbSerialDriver} implementation for a variety of FTDI devices
 * <p>
 * This driver is based on <a
 * href="http://www.intra2net.com/en/developer/libftdi">libftdi</a>, and is
 * copyright and subject to the following terms:
 * 
 * <pre>
 *   Copyright (C) 2003 by Intra2net AG
 * 
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License
 *   version 2.1 as published by the Free Software Foundation;
 * 
 *   opensource@intra2net.com
 *   http://www.intra2net.com/en/developer/libftdi
 * </pre>
 * 
 * </p>
 * <p>
 * Some FTDI devices have not been tested; see later listing of supported and
 * unsupported devices. Devices listed as "supported" support the following
 * features:
 * <ul>
 * <li>Read and write of serial data (see {@link #read(byte[], int)} and
 * {@link #write(byte[], int)}.
 * <li>Setting baud rate (see {@link #setBaudRate(int)}).
 * </ul>
 * </p>
 * <p>
 * Supported and tested devices:
 * <ul>
 * <li>{@value DeviceType#TYPE_R}</li>
 * </ul>
 * </p>
 * <p>
 * Unsupported but possibly working devices (please contact the author with
 * feedback or patches):
 * <ul>
 * <li>{@value DeviceType#TYPE_2232C}</li>
 * <li>{@value DeviceType#TYPE_2232H}</li>
 * <li>{@value DeviceType#TYPE_4232H}</li>
 * <li>{@value DeviceType#TYPE_AM}</li>
 * <li>{@value DeviceType#TYPE_BM}</li>
 * </ul>
 * </p>
 * 
 * @author mike wakerly (opensource@hoho.com)
 * @see <a href="http://code.google.com/p/usb-serial-for-android/">USB Serial
 *      for Android project page</a>
 * @see <a href="http://www.ftdichip.com/">FTDI Homepage</a>
 * @see <a href="http://www.intra2net.com/en/developer/libftdi">libftdi</a>
 */
public class FtdiSerialDriver extends CommonUsbSerialDriver {

	private static final boolean DEBUG = false;
	private final String TAG = FtdiSerialDriver.class.getSimpleName();

	private UsbInterface mDataInterface = null;

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

	public static final int USB_TYPE_STANDARD = 0x00 << 5;
	public static final int USB_TYPE_CLASS = 0x00 << 5;
	public static final int USB_TYPE_VENDOR = 0x00 << 5;
	public static final int USB_TYPE_RESERVED = 0x00 << 5;

	public static final int USB_RECIP_DEVICE = 0x00;
	public static final int USB_RECIP_INTERFACE = 0x01;
	public static final int USB_RECIP_ENDPOINT = 0x02;
	public static final int USB_RECIP_OTHER = 0x03;

	public static final int USB_ENDPOINT_IN = 0x80;
	public static final int USB_ENDPOINT_OUT = 0x00;

	public static final int USB_WRITE_TIMEOUT_MILLIS = 5000;
	public static final int USB_READ_TIMEOUT_MILLIS = 5000;

	// From ftdi.h
	/**
	 * Reset the port.
	 */
	private static final int SIO_RESET_REQUEST = 0;

	/**
	 * Set baud rate.
	 */
	private static final int SIO_SET_BAUD_RATE_REQUEST = 3;

	/**
	 * Set the data characteristics of the port.
	 */
	private static final int SIO_SET_DATA_REQUEST = 4;

	private static final int SIO_RESET_SIO = 0;

	public static final int FTDI_DEVICE_OUT_REQTYPE = UsbConstants.USB_TYPE_VENDOR | USB_RECIP_DEVICE
			| USB_ENDPOINT_OUT;

	public static final int FTDI_DEVICE_IN_REQTYPE = UsbConstants.USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_ENDPOINT_IN;

	/**
	 * Length of the modem status header, transmitted with every read.
	 */
	private static final int MODEM_STATUS_HEADER_LENGTH = 2;

	private DeviceType mType;

	/**
	 * FTDI chip types.
	 */
	private static enum DeviceType {
		TYPE_BM, TYPE_AM, TYPE_2232C, TYPE_R, TYPE_2232H, TYPE_4232H;
	}

	// Implements the class that reads the USB device.
	private class UsbReaderRunner implements Runnable {

		private boolean mRunning = false;

		@Override
		public void run() {

			if (DEBUG)
				Log.d(TAG, "Reader thread running...");

			// We want a "direct" byte buffer created where the buffer 
			// memory lives outside the Java heap manager. This is 
			// because garbage collection occurring during a call into 
			// the native code transmitting a buffer can cause problems
			// if memory is shifted around. We attempt to avoid this 
			// issue using a "direct" byte buffer.  We set the buffer
			// sizes to getMaxPacketSize() because the FTDI driver will
			// place status bytes in the middle of the data we we read
			// more data.  This keeps us from having to search the data
			// and manually remove the status bytes.
			ByteBuffer[] readerBuffers = new ByteBuffer[2];
			readerBuffers[0] = ByteBuffer.allocateDirect(mReadEndpoint.getMaxPacketSize());
			readerBuffers[1] = ByteBuffer.allocateDirect(mReadEndpoint.getMaxPacketSize());

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

						// Immediately queue the next reader request before we
						// write the current buffer to the stream. We do this 
						// to try and be sure to capture data that may otherwise
						// be lost if the write to the stream takes too long.
						readerBuffers[nextIndex].clear();
						mReaderRequest.initialize(mConnection, mReadEndpoint);
						mReaderRequest.queue(readerBuffers[nextIndex], readerBuffers[nextIndex].capacity());

						// The device sends two status bytes also in the buffer for each read. 
					    // The chip generates the two stripped status bytes in the absence of 
						// data every 40 ms.
						// See: http://e2e.ti.com/support/microcontrollers/stellaris_arm/f/471/t/159850.aspx
					    final int payloadBytesRead = readerBuffers[readerIndex].position() - MODEM_STATUS_HEADER_LENGTH;

						// Do we have actual data?
						if (payloadBytesRead > 0) {

							if (DEBUG)
								Log.d(TAG, "Reading from USB: " + HexDump.dumpHexString(readerBuffers[readerIndex].array(), 2, payloadBytesRead));

							try {
								// Write the buffer to the reader output stream.
								mReaderOutputStream.write(readerBuffers[readerIndex].array(), MODEM_STATUS_HEADER_LENGTH, payloadBytesRead);
							} catch (IOException e) {
								// Abort running.
								mRunning = false;
								Log.e(TAG, "Error writing buffer to output stream.");
							}
						} else {
							//try {
							//	Thread.sleep(100);
							//} catch (InterruptedException e) {
							//	// Ignored.
							//}
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
	// asynchronous interface used in the reader. The reason
	// for this is because requestWait method is not thread
	// safe and to synchronize both the reader and writer
	// could potentially slow things down. The reason for this
	// is explained in more detail here:
	// http://stackoverflow.com/questions/9644415/usbrequest-queue-crashing-android-3-1-application
	private class UsbWriterRunner implements Runnable {

		private boolean mRunning = false;

		@Override
		public void run() {
			int count = 0;

			if (DEBUG)
				Log.d(TAG, "Writer thread running...");

			// We want a "direct" byte buffer created where the buffer memory
			// lives outside the Java heap manager. This is because garbage 
			// collection occurring during a call into the native code transmitting 
			// a buffer can cause problems if memory is shifted around. We attempt 
			// to avoid this issue using a "direct" byte buffer.
			ByteBuffer writerBuffer = ByteBuffer.allocateDirect(mWriteEndpoint.getMaxPacketSize());

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

					// Synchronous transfer with a reasonable 0.1 second
					// timeout. The array() method may not work on versions 
					// of Android prior to 4.2.
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

	/**
	 * Constructor.
	 * 
	 * @param usbDevice
	 *            the {@link UsbDevice} to use
	 * @param usbConnection
	 *            the {@link UsbDeviceConnection} to use
	 * @throws UsbSerialRuntimeException
	 *             if the given device is incompatible with this driver
	 */
	public FtdiSerialDriver(UsbDevice usbDevice, UsbDeviceConnection usbConnection) {
		super(usbDevice, usbConnection);
		mType = null;
	}

	public void reset() throws IOException {
		int result = mConnection.controlTransfer(FTDI_DEVICE_OUT_REQTYPE, SIO_RESET_REQUEST, SIO_RESET_SIO,
				0 /* index */, null, 0, USB_WRITE_TIMEOUT_MILLIS);
		if (result != 0) {
			throw new IOException("Reset failed: result=" + result);
		}

		// TODO(mikey): autodetect.
		mType = DeviceType.TYPE_R;
	}

	@Override
	public void open() throws IOException {
		boolean opened = false;

		try {
			for (int i = 0; i < mDevice.getInterfaceCount(); i++) {
				if (!mConnection.claimInterface(mDevice.getInterface(i), true)) {
					throw new IOException("Error claiming interface " + i);
				}
			}
			reset();
			if (DEBUG)
				Log.d(TAG, "Claiming data interface.");
			mDataInterface = mDevice.getInterface(0);
			if (DEBUG)
				Log.d(TAG, "data iface=" + mDataInterface);

			if (!mConnection.claimInterface(mDataInterface, true)) {
				throw new IOException("Could not claim data interface.");
			}
			mReadEndpoint = mDataInterface.getEndpoint(0);
			if (DEBUG)
				Log.d(TAG, "Read endpoint direction: " + mReadEndpoint.getDirection());
			mWriteEndpoint = mDataInterface.getEndpoint(1);
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

			if (DEBUG)
				Log.d(TAG, "Creating reader/writer request objects.");

			// Create the USB request objects used in the reader/writer threads.
			mReaderRequest = new UsbRequest();
			mWriterRequest = new UsbRequest();

			if (DEBUG)
				Log.d(TAG, "Starting reader/writer threads.");

			// Start reader and writer threads.
			mUsbReader = new UsbReaderRunner();
			mUsbWriter = new UsbWriterRunner();
			mUsbReaderThread = new Thread(mUsbReader);
			mUsbWriterThread = new Thread(mUsbWriter);
			mUsbReaderThread.setPriority(Thread.MAX_PRIORITY);
			mUsbReaderThread.start();
			mUsbWriterThread.start();

			opened = true;
		} finally {
			if (!opened) {
				close();
			}
		}
	}

	@Override
	public void close() {
		if (DEBUG)
			Log.d(TAG, "Disabling reader and writer running flags.");

		// Set the shutdown flags.
		if (mUsbReader != null) mUsbReader.shutdown();
		if (mUsbWriter != null) mUsbWriter.shutdown();

		if (DEBUG)
			Log.d(TAG, "Cancelling pending request operations.");

		// Cancel any pending operations.
		if (mReaderRequest != null) mReaderRequest.cancel();
		if (mWriterRequest != null) mWriterRequest.cancel();

		if (DEBUG)
			Log.d(TAG, "Close the request resources.");

		// Close any request resources.
		if (mReaderRequest != null) mReaderRequest.close();
		if (mWriterRequest != null) mWriterRequest.close();

		mUsbReader = null;
		mUsbWriter = null;
		mReaderRequest = null;
		mWriterRequest = null;
		
		mConnection.close();
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

	private int setBaudRate(int baudRate) throws IOException {
		long[] vals = convertBaudrate(baudRate);
		long actualBaudrate = vals[0];
		long index = vals[1];
		long value = vals[2];
		int result = mConnection.controlTransfer(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_BAUD_RATE_REQUEST, (int) value,
				(int) index, null, 0, USB_WRITE_TIMEOUT_MILLIS);
		if (result != 0) {
			throw new IOException("Setting baudrate failed: result=" + result);
		}
		return (int) actualBaudrate;
	}

	@Override
	public void setParameters(int baudRate, int dataBits, int stopBits, int parity) throws IOException {
		setBaudRate(baudRate);

		int config = dataBits;

		switch (parity) {
		case PARITY_NONE:
			config |= (0x00 << 8);
			break;
		case PARITY_ODD:
			config |= (0x01 << 8);
			break;
		case PARITY_EVEN:
			config |= (0x02 << 8);
			break;
		case PARITY_MARK:
			config |= (0x03 << 8);
			break;
		case PARITY_SPACE:
			config |= (0x04 << 8);
			break;
		default:
			throw new IllegalArgumentException("Unknown parity value: " + parity);
		}

		switch (stopBits) {
		case STOPBITS_1:
			config |= (0x00 << 11);
			break;
		case STOPBITS_1_5:
			config |= (0x01 << 11);
			break;
		case STOPBITS_2:
			config |= (0x02 << 11);
			break;
		default:
			throw new IllegalArgumentException("Unknown stopBits value: " + stopBits);
		}

		int result = mConnection.controlTransfer(FTDI_DEVICE_OUT_REQTYPE, SIO_SET_DATA_REQUEST, config, 0 /* index */,
				null, 0, USB_WRITE_TIMEOUT_MILLIS);
		if (result != 0) {
			throw new IOException("Setting parameters failed: result=" + result);
		}
	}

	private long[] convertBaudrate(int baudrate) {
		// TODO(mikey): Braindead transcription of libfti method. Clean up,
		// using more idiomatic Java where possible.
		int divisor = 24000000 / baudrate;
		int bestDivisor = 0;
		int bestBaud = 0;
		int bestBaudDiff = 0;
		int fracCode[] = { 0, 3, 2, 4, 1, 5, 6, 7 };

		for (int i = 0; i < 2; i++) {
			int tryDivisor = divisor + i;
			int baudEstimate;
			int baudDiff;

			if (tryDivisor <= 8) {
				// Round up to minimum supported divisor
				tryDivisor = 8;
			} else if (mType != DeviceType.TYPE_AM && tryDivisor < 12) {
				// BM doesn't support divisors 9 through 11 inclusive
				tryDivisor = 12;
			} else if (divisor < 16) {
				// AM doesn't support divisors 9 through 15 inclusive
				tryDivisor = 16;
			} else {
				if (mType == DeviceType.TYPE_AM) {
					// TODO
				} else {
					if (tryDivisor > 0x1FFFF) {
						// Round down to maximum supported divisor value (for
						// BM)
						tryDivisor = 0x1FFFF;
					}
				}
			}

			// Get estimated baud rate (to nearest integer)
			baudEstimate = (24000000 + (tryDivisor / 2)) / tryDivisor;

			// Get absolute difference from requested baud rate
			if (baudEstimate < baudrate) {
				baudDiff = baudrate - baudEstimate;
			} else {
				baudDiff = baudEstimate - baudrate;
			}

			if (i == 0 || baudDiff < bestBaudDiff) {
				// Closest to requested baud rate so far
				bestDivisor = tryDivisor;
				bestBaud = baudEstimate;
				bestBaudDiff = baudDiff;
				if (baudDiff == 0) {
					// Spot on! No point trying
					break;
				}
			}
		}

		// Encode the best divisor value
		long encodedDivisor = (bestDivisor >> 3) | (fracCode[bestDivisor & 7] << 14);
		// Deal with special cases for encoded value
		if (encodedDivisor == 1) {
			encodedDivisor = 0; // 3000000 baud
		} else if (encodedDivisor == 0x4001) {
			encodedDivisor = 1; // 2000000 baud (BM only)
		}

		// Split into "value" and "index" values
		long value = encodedDivisor & 0xFFFF;
		long index;
		if (mType == DeviceType.TYPE_2232C || mType == DeviceType.TYPE_2232H || mType == DeviceType.TYPE_4232H) {
			index = (encodedDivisor >> 8) & 0xffff;
			index &= 0xFF00;
			index |= 0 /* TODO mIndex */;
		} else {
			index = (encodedDivisor >> 16) & 0xffff;
		}

		// Return the nearest baud rate
		return new long[] { bestBaud, index, value };
	}

	@Override
	public boolean getCD() throws IOException {
		return false;
	}

	@Override
	public boolean getCTS() throws IOException {
		return false;
	}

	@Override
	public boolean getDSR() throws IOException {
		return false;
	}

	@Override
	public boolean getDTR() throws IOException {
		return false;
	}

	@Override
	public void setDTR(boolean value) throws IOException {
	}

	@Override
	public boolean getRI() throws IOException {
		return false;
	}

	@Override
	public boolean getRTS() throws IOException {
		return false;
	}

	@Override
	public void setRTS(boolean value) throws IOException {
	}

	public static Map<Integer, int[]> getSupportedDevices() {
		final Map<Integer, int[]> supportedDevices = new LinkedHashMap<Integer, int[]>();
		supportedDevices.put(Integer.valueOf(UsbId.VENDOR_FTDI), new int[] { UsbId.FTDI_FT232R, });
		return supportedDevices;
	}

}
