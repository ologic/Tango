/* Copyright 2013 Google Inc.
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

import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;

/**
 * A base class shared by several driver implementations.
 *
 * @author mike wakerly (opensource@hoho.com)
 */
abstract class CommonUsbSerialDriver implements UsbSerialDriver {

    public static final int DEFAULT_READ_BUFFER_SIZE = 16 * 1024;
    public static final int DEFAULT_WRITE_BUFFER_SIZE = 16 * 1024;

    protected final UsbDevice mDevice;
    protected final UsbDeviceConnection mConnection;

    public CommonUsbSerialDriver(UsbDevice device, UsbDeviceConnection connection) {
        mDevice = device;
        mConnection = connection;
    }

    /**
     * Returns the currently-bound USB device.
     *
     * @return the device
     */
    @Override
    public final UsbDevice getDevice() {
        return mDevice;
    }

    /**
     * Returns the currently-bound USB device connection.
     *
     * @return the device connection
     */
    @Override
    public final UsbDeviceConnection getConnection() {
        return mConnection;
    }

    @Override
    public abstract void open() throws IOException;

    @Override
    public abstract void close() throws IOException;

    @Override
    public abstract InputStream getInputStream();

    @Override
    public abstract OutputStream getOutputStream();

    @Override
    public abstract void setParameters(
            int baudRate, int dataBits, int stopBits, int parity) throws IOException;

    @Override
    public abstract boolean getCD() throws IOException;

    @Override
    public abstract boolean getCTS() throws IOException;

    @Override
    public abstract boolean getDSR() throws IOException;

    @Override
    public abstract boolean getDTR() throws IOException;

    @Override
    public abstract void setDTR(boolean value) throws IOException;

    @Override
    public abstract boolean getRI() throws IOException;

    @Override
    public abstract boolean getRTS() throws IOException;

    @Override
    public abstract void setRTS(boolean value) throws IOException;

}
