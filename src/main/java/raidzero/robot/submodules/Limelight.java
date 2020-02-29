package raidzero.robot.submodules;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import raidzero.robot.Constants.LimelightConstants;

// Reference: http://docs.limelightvision.io/en/latest/networktables_api.html
public class Limelight extends Submodule {
	/**
	 * LED modes for Limelight.
	 */
	public static enum LedMode {
		Current, Off, Blink, On
	}

	/**
	 * Camera modes for Limelight.
	 */
	public static enum CameraMode {
		Vision, Driver
	}
	
	/**
	 * Stream modes for Limelight.
	 */
	public static enum StreamMode {
		Standard,     // Side-by-side streams if a webcam is attached to Limelight
		PIP_Main,     // Secondary stream is placed in the lower-right corner of the primary stream
		PIP_Secondary // Primary stream is placed in the lower-right corner of the secondary stream
	}

	private static Limelight instance = null;

	public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight(LimelightConstants.NAME);
        }
        return instance;
    }

	private NetworkTable table = null;
	private String tableName;

	private Limelight(String tableName) {
		this.tableName = tableName;
	}

	@Override
	public void onStart(double timestamp) {
		setLedMode(LedMode.On);
	}

	/**
	 * Gets whether a target is detected by the Limelight.
	 * 
	 * @return true if a target is detected, false otherwise.
	 */
	public boolean hasTarget() {
		return getValue("tv").getDouble(0.0) > 0.0;
	}

	/**
	 * Horizontal offset from crosshair to target (-27 degrees to 27 degrees).
	 * 
	 * @return tx as reported by the Limelight.
	 */
	public double getTx() {
		return getValue("tx").getDouble(0.0);
	}

	/**
	 * Vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees).
	 * 
	 * @return ty as reported by the Limelight.
	 */
	public double getTy() {
		return getValue("ty").getDouble(0.0);
	}

	/**
	 * Area that the detected target takes up in total camera FOV (0% to 100%).
	 * 
	 * @return Area of target.
	 */
	public double getTa() {
		return getValue("ta").getDouble(0.0);
	}

	/**
	 * Gets target skew or rotation (-90 degrees to 0 degrees).
	 * 
	 * @return Target skew.
	 */
	public double getTs() {
		return getValue("ts").getDouble(0.0);
	}

	/**
	 * Gets target latency (ms).
	 * 
	 * @return Target latency.
	 */
	public double getTl() {
		return getValue("tl").getDouble(0.0);
	}
	
	/**
	 * Returns an array of corner x-coordinates.
	 *
	 * Note: Enable "send contours" in the "Output" tab 
	 * to stream corner coordinates.
	 * 
	 * @return Corner x-coordinates.
	 */
	public double[] getTCornX() {
		return getValue("tcornx").getDoubleArray(new double[] {});
	}
	
	/**
	 * Returns an array of corner y-coordinates.
	 *
	 * Note: Enable "send contours" in the "Output" tab 
	 * to stream corner coordinates.
	 *
	 * @return Corner y-coordinates.
	 */
	public double[] getTCornY() {
		return getValue("tcorny").getDoubleArray(new double[] {});
	}

	/**
	 * Sets LED mode of Limelight.
	 * 
	 * @param mode Light mode for Limelight.
	 */
	public void setLedMode(LedMode mode) {
		getValue("ledMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets camera mode for Limelight.
	 * 
	 * @param mode Camera mode for Limelight.
	 */
	public void setCameraMode(CameraMode mode) {
		getValue("camMode").setNumber(mode.ordinal());
	}

	/**
	 * Sets pipeline number (0-9 value).
	 * 
	 * @param number Pipeline number (0-9).
	 */
	public void setPipeline(int number) {
		getValue("pipeline").setNumber(number);
	}
	
	/**
	 * Sets stream mode for Limelight.
	 * 
	 * @param mode Stream mode for Limelight
	 */
	public void setStreamMode(StreamMode mode) {
		getValue("stream").setNumber(mode.ordinal());
	}

	/**
	 * Helper method to get an entry from the Limelight NetworkTable.
	 * 
	 * @param key Key for entry.
	 * @return NetworkTableEntry of given entry.
	 */
	private NetworkTableEntry getValue(String key) {
		if (table == null) {
			table = NetworkTableInstance.getDefault().getTable(tableName);
		}
		return table.getEntry(key);
	}

    @Override
    public void stop() {
        setLedMode(LedMode.Off);
    }
}