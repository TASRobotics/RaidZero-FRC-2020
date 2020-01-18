package raidzero.robot;

public class Constants {

    /**
     * Drivetrain Constants
     */
    // Motor IDs
    public static final int driveLeftLeaderId = 1;
    public static final int driveLeftFollowerId = 3;
    public static final int driveRightLeaderId = 0;
    public static final int driveRightFollowerId = 2;
    /*public static final int driveLeftLeaderId = 11;
    public static final int driveLeftFollower1Id = 12;
    public static final int driveLeftFollower2Id = 13;
    public static final int driveRightLeaderId = 3;
    public static final int driveRightFollower1Id = 4;
    public static final int driveRightFollower2Id = 5;*/

    public static final double LOW_GEAR_RATIO = 9.98;
    public static final double HIGH_GEAR_RATIO = 18.43;
    // In inches
    public static final double WHEEL_DIAMETER = 6.0;

    // Solenoid IDs
    public static final int driveGearshiftForwardId = 0;
    public static final int driveGearshiftReverseId = 1;

    // Pigeon ID
    public static final int pigeonId = 0;

    // Closed-loop constants
    public static final int REMOTE_0 = 0;
    public static final int REMOTE_1 = 1;
    public static final int PID_PRIMARY_SLOT = 0;
    public static final int PID_AUX_SLOT = 1;
    public static final int TIMEOUT_MS = 10;
    public static final double PIGEON_SCALE = 3600.0 / 8192.0;

    public static final double PRIMARY_F = 0.0;
    public static final double PRIMARY_P = 0.7;
    public static final double PRIMARY_I = 0;
    public static final double PRIMARY_D = 0;
    public static final int PRIMARY_INT_ZONE = 50;

    public static final double AUX_F = 0.60 * 1023.0 / (440.0 * 10.0); // Maximum was 440 dps
    public static final double AUX_P = 3.0;
    public static final double AUX_I = 0;
    public static final double AUX_D = 0;
    public static final int AUX_INT_ZONE = 20;
    public static final boolean AUX_POLARITY = false;

    public static final int BASE_TRAJ_PERIOD_MS = 0;
    public static final double SENSOR_UNITS_PER_INCH = 
        2048 * LOW_GEAR_RATIO / (WHEEL_DIAMETER * Math.PI);
    public static final int MIN_POINTS_IN_TALON = 10;
    public static final int CLOSED_LOOP_TIME_MS = 1;
    public static final int TRANSMIT_PERIOD_MS = 3;

    public static final double DEFAULT_CRUISE_VELOCITY = 7;
    public static final double DEFAULT_TARGET_ACCELERATION = 10;

    // Joystick to Output mapping
    public static final double driveExponent = 2;
    public static final double driveCoef = 1;

    public static final String limelightNameShooter = "limelight";

    public static final double KP_AIM = 0.006;
	public static final double MINIMUM_POWER = 0.2;

	public static final double ANGLE_ADJUST_THRESHOLD = 1.0;

    /**
     * Joystick Constants
     */
    public static final double joystickDeadband = 0.1;
}