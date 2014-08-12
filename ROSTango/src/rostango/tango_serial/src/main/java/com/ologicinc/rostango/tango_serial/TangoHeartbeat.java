package com.ologicinc.rostango.tango_serial;

import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

// XXX import org.droidplanner.core.MAVLink.MavLinkHeartbeat;
// XXX import org.droidplanner.core.drone.Drone;

import com.MAVLink.Messages.*;
import com.MAVLink.Messages.ardupilotmega.msg_heartbeat;
import com.MAVLink.Messages.enums.MAV_AUTOPILOT;
import com.MAVLink.Messages.enums.MAV_MODE;
import com.MAVLink.Messages.enums.MAV_STATE;
import com.MAVLink.Messages.enums.MAV_TYPE;


/**
 * This class is used to send periodic heartbeat messages.
 */
public class TangoHeartbeat {

	/**
	 * This is the serial port to send the heartbeat message to.
	 */
    private static VioListenerNode mListener = null;

	/**
	 * This is the heartbeat period in seconds.
	 */
	private final int period;

	/**
	 * ScheduledExecutorService used to periodically schedule the heartbeat.
	 */
	private ScheduledExecutorService heartbeatExecutor;

    private msg_heartbeat mHeartbeatMsg;

	/**
	 * Runnable used to send the heartbeat.
	 */
	private final Runnable heartbeatRunnable = new Runnable() {
		@Override
		public void run() {
			// XXX MavLinkHeartbeat.sendMavHeartbeat(drone);

            MAVLinkPacket mavPacket = mHeartbeatMsg.pack();
            mListener.sendMavMessage(mavPacket);
		}
	};

	public TangoHeartbeat(VioListenerNode listener, int freqHz) {
        this.mListener = listener;
		this.period = freqHz;

        this.mHeartbeatMsg = new msg_heartbeat();
        mHeartbeatMsg.type = MAV_TYPE.MAV_TYPE_GENERIC;
        mHeartbeatMsg.autopilot = MAV_AUTOPILOT.MAV_AUTOPILOT_GENERIC;
        mHeartbeatMsg.base_mode = MAV_MODE.MAV_MODE_PREFLIGHT;
        mHeartbeatMsg.custom_mode = 0;
        mHeartbeatMsg.system_status = MAV_STATE.MAV_STATE_ACTIVE;
        mHeartbeatMsg.mavlink_version = 3;

	}

	/**
	 * Set the state of the heartbeat.
	 * 
	 * @param active
	 *            true to activate the heartbeat, false to deactivate it
	 */
	public void setActive(boolean active) {
		if (active) {
			heartbeatExecutor = Executors.newSingleThreadScheduledExecutor();
			heartbeatExecutor
					.scheduleWithFixedDelay(heartbeatRunnable, 0, period, TimeUnit.SECONDS);
		} else if (heartbeatExecutor != null) {
			heartbeatExecutor.shutdownNow();
			heartbeatExecutor = null;
		}
	}
}
