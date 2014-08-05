package com.ologicinc.rostango.tango_serial;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.os.SystemClock;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.widget.TwoLineListItem;

import com.github.ologic.android_ologic.usbserial.driver.UsbSerialDriver;
import com.github.ologic.android_ologic.usbserial.driver.UsbSerialProber;
import com.github.ologic.android_ologic.usbserial.util.HexDump;

import java.io.IOException;
import java.lang.ref.WeakReference;
import java.util.ArrayList;
import java.util.List;

@SuppressWarnings("deprecation")
public class MainActivity extends Activity {

  private static final boolean DEBUG = false;
	private static final String TAG = MainActivity.class.getSimpleName();

	private UsbManager mUsbManager;
	private ListView mListView;
	private TextView mProgressBarTitle;
	private ProgressBar mProgressBar;
	private PendingIntent mPermissionIntent;
	private UsbDevice mSerialDevice = null;
	private UsbSerialDriver mSerialDriver = null;

	private static final int MESSAGE_REFRESH = 101;
	private static final long REFRESH_TIMEOUT_MILLIS = 5000;

	private static final String ACTION_USB_PERMISSION = "com.android.example.USB_PERMISSION";

    // Receiver for USB permissions and for USB device attachment.
	private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {

		@Override
		public void onReceive(Context context, Intent intent) {
			String action = intent.getAction();
			if (ACTION_USB_PERMISSION.equals(action)) {
				synchronized (this) {
					UsbDevice device = (UsbDevice) intent
							.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
						if (DEBUG) Log.d(TAG, "permission granted for device " + device);
						if (device != null) {
							// Call method to set up device communication.
							// Actually we do nothing here and wait until 
							// the permission returns below.
						}
					} else {
						if (DEBUG) Log.d(TAG, "permission denied for device " + device);
					}
				}
			} else if (UsbManager.ACTION_USB_DEVICE_ATTACHED.equals(action)) {
				if (DEBUG) Log.d(TAG, "************** USB Device Attached **********");
			} else if (UsbManager.ACTION_USB_DEVICE_DETACHED.equals(action)) {
				if (DEBUG) Log.d(TAG, "************** USB Device Detached **********");
				synchronized (this) {
					UsbDevice device = (UsbDevice) intent
							.getParcelableExtra(UsbManager.EXTRA_DEVICE);
					if (mSerialDevice != null
							&& mSerialDevice.getDeviceName().equals(device.getDeviceName())) {
						// Close the driver.
						if (mSerialDriver != null) {
							try {
								mSerialDriver.close();
							} catch (IOException e) {
								// Ignore.
							}
						}
						mSerialDriver = null;
						mSerialDevice = null;
					}
				}
			}
		}
	};

	// Avoiding the warning that "This Handler class should be static or
	// leaks might occur: RefreshHandler". If RefereshHandler class is not
	// static, it will have a reference to our MainActivity object. Handler
	// objects for the same thread all share a common Looper object, which
	// they post messages to and read from. As messages contain the target
	// Handler, as long as there are messages with target handler in the
	// message queue, the handler cannot be garbage collected. If the handler
	// is not static, the Activity cannot be garbage collected, even after
	// being destroyed. We avoid this issue by making weak references to
	// the Activity which can be garbage collected.
	static class RefreshHandler extends Handler {
		private final WeakReference<MainActivity> mActivity;

		RefreshHandler(MainActivity activity) {
			mActivity = new WeakReference<MainActivity>(activity);
		}

		@Override
		public void handleMessage(Message msg) {
			switch (msg.what) {
			case MESSAGE_REFRESH:
				MainActivity activity = mActivity.get();
				if (activity != null) {
					activity.refreshDeviceList();
				}
				sendEmptyMessageDelayed(MESSAGE_REFRESH, REFRESH_TIMEOUT_MILLIS);
				break;
			default:
				super.handleMessage(msg);
				break;
			}
		}
	}

	private final RefreshHandler mHandler = new RefreshHandler(this);

	/** Simple container for a UsbDevice and its driver. */
	private static class DeviceEntry {
		public UsbDevice device;
		public UsbSerialDriver driver;

		DeviceEntry(UsbDevice device, UsbSerialDriver driver) {
			this.device = device;
			this.driver = driver;
		}
	}

	private final List<DeviceEntry> mEntries = new ArrayList<DeviceEntry>();
	private ArrayAdapter<DeviceEntry> mAdapter;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		if (DEBUG) Log.d(TAG, "onCreate().");
		super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

		mUsbManager = (UsbManager) getSystemService(Context.USB_SERVICE);
		mPermissionIntent = PendingIntent.getBroadcast(this, 0, new Intent(ACTION_USB_PERMISSION),	0);
		IntentFilter filter = new IntentFilter();
		filter.addAction(ACTION_USB_PERMISSION);
		filter.addAction(UsbManager.ACTION_USB_DEVICE_ATTACHED);
		filter.addAction(UsbManager.ACTION_USB_DEVICE_DETACHED);
		filter.addAction(ACTION_USB_PERMISSION);
		registerReceiver(mUsbReceiver, filter);

		mListView = (ListView) findViewById(R.id.deviceList);
		mProgressBar = (ProgressBar) findViewById(R.id.progressBar);
		mProgressBarTitle = (TextView) findViewById(R.id.progressBarTitle);

		mAdapter = new ArrayAdapter<DeviceEntry>(this,
				android.R.layout.simple_expandable_list_item_2, mEntries) {
			@Override
			public View getView(int position, View convertView, ViewGroup parent) {
				// See:
				// http://stackoverflow.com/questions/13592371/listactivity-twolinelistitem-alternative
				final TwoLineListItem row;
				if (convertView == null) {
					final LayoutInflater inflater = (LayoutInflater) getSystemService(Context.LAYOUT_INFLATER_SERVICE);
					row = (TwoLineListItem) inflater.inflate(android.R.layout.simple_list_item_2,
							null);
				} else {
					row = (TwoLineListItem) convertView;
				}

				final DeviceEntry entry = mEntries.get(position);
				final String title = String.format("Vendor %s Product %s",
						HexDump.toHexString((short) entry.device.getVendorId()),
						HexDump.toHexString((short) entry.device.getProductId()));
				row.getText1().setText(title);

				final String subtitle = entry.driver != null ? entry.driver.getClass()
						.getSimpleName() : "No Driver";
				row.getText2().setText(subtitle);

				return row;
			}

		};

		mListView.setAdapter(mAdapter);

		mListView.setOnItemClickListener(new ListView.OnItemClickListener() {
			@Override
			public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
				if (DEBUG) Log.d(TAG, "Pressed item " + position);
				if (position >= mEntries.size()) {
					Log.w(TAG, "Illegal position.");
					return;
				}

				// This does not handle multiple active drivers.
				final DeviceEntry entry = mEntries.get(position);
				mSerialDriver = entry.driver;
				mSerialDevice = entry.device;
				if (mSerialDriver == null || mSerialDevice == null) {
					if (DEBUG) Log.d(TAG, "No driver or device.");
					return;
				}

				showSerialActivity(mSerialDriver);
			}
		});
	}

	@Override
	public void onStart() {
		if (DEBUG) Log.d(TAG, "onStart().");

		// Call our super class.
		super.onStart();
	}

	@Override
	public void onRestart() {
		if (DEBUG) Log.d(TAG, "onRestart().");

		// Call our super class.
		super.onRestart();
	}

	@Override
	protected void onResume() {
		if (DEBUG) Log.d(TAG, "onResume().");

		// Call our super class.
		super.onResume();

		// Refresh the device list.
		mHandler.sendEmptyMessage(MESSAGE_REFRESH);
	}

	@Override
	protected void onPause() {
		if (DEBUG) Log.d(TAG, "onPause().");

		// Call our super class.
		super.onPause();

		// Refresh the device list.
		mHandler.removeMessages(MESSAGE_REFRESH);
	}

	@Override
	public void onStop() {
		if (DEBUG) Log.d(TAG, "onStop().");

		// Call our super class.
		super.onStop();
	}

	@Override
	public void onDestroy() {
		if (DEBUG) Log.d(TAG, "onDestroy().");

		try {
			// Unregister our USB receiver.
			unregisterReceiver(mUsbReceiver);
	    }
	    catch (IllegalArgumentException e) {
	    	// Ignored.
	    }

		// Call our super class.
		super.onDestroy();
	}

	protected void refreshDeviceList() {
		showProgressBar();

		new AsyncTask<Void, Void, List<DeviceEntry>>() {
			@Override
			protected List<DeviceEntry> doInBackground(Void... params) {
				if (DEBUG) Log.d(TAG, "Refreshing device list ...");
				SystemClock.sleep(1000);
				final List<DeviceEntry> result = new ArrayList<DeviceEntry>();
				for (final UsbDevice device : mUsbManager.getDeviceList().values()) {
					final List<UsbSerialDriver> drivers = UsbSerialProber.probeSingleDevice(
							mUsbManager, device, mPermissionIntent);
					if (DEBUG) Log.d(TAG, "Found usb device: " + device);
					if (drivers.isEmpty()) {
						if (DEBUG) Log.d(TAG, "  - No UsbSerialDriver available.");
						result.add(new DeviceEntry(device, null));
					} else {
						for (UsbSerialDriver driver : drivers) {
							if (DEBUG) Log.d(TAG, "  + " + driver);
							result.add(new DeviceEntry(device, driver));
						}
					}
				}
				return result;
			}

			@Override
			protected void onPostExecute(List<DeviceEntry> result) {
				mEntries.clear();
				mEntries.addAll(result);
				mAdapter.notifyDataSetChanged();
				mProgressBarTitle.setText(String.format("%s device(s) found",
						Integer.valueOf(mEntries.size())));
				hideProgressBar();
				if (DEBUG) Log.d(TAG, "Done refreshing, " + mEntries.size() + " entries found.");
			}

		}.execute((Void) null);
	}

	private void showProgressBar() {
		mProgressBar.setVisibility(View.VISIBLE);
		mProgressBarTitle.setText(R.string.refreshing);
	}

	private void hideProgressBar() {
		mProgressBar.setVisibility(View.INVISIBLE);
	}

	private void showSerialActivity(UsbSerialDriver driver) {
        com.ologicinc.rostango.tango_serial.TangoSerial.show(this, driver);
	}
}
