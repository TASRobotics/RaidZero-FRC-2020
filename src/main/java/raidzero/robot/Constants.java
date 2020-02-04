package raidzero.robot;

import com.ctre.phoenix.motorcontrol.InvertType;

public class Constants {

    // Motor IDs
    public static final int driveLeftLeaderId = 1;
    public static final int driveLeftFollowerId = 3;
    public static final int driveRightLeaderId = 0;
    public static final int driveRightFollowerId = 2;

    public static final InvertType driveLeftInvert = InvertType.None;
    public static final InvertType driveRightInvert = InvertType.InvertMotorOutput;

    // Solenoid IDs
    public static final int driveGearshiftForwardId = 0;
    public static final int driveGearshiftReverseId = 1;

    // Pigeon ID
    public static final int pigeonId = 0;

    public static final int shooterId = 8;
    public static final int turretId = 6;

    public static final int intakeId = 4;
    public static final int intakeForwardId = 2;
    public static final int intakeReverseId = 3;

    public static final int hopperId = 5;

    /**
     * Drivetrain Constants
     */
    public static final class DriveConstants {
        public static final double HIGH_GEAR_RATIO = 9.98;
        public static final double LOW_GEAR_RATIO = 18.43;
    
        public static final double WHEEL_DIAMETER_INCHES = 6.0;

        // Closed-loop constants
        public static final double DRIVE_NEUTRAL_DEADBAND = 0.01;
        public static final int PID_PRIMARY_SLOT = 0;
        public static final int PID_AUX_SLOT = 1;
        public static final double PIGEON_SCALE = 3600.0 / 8192.0;

        public static final double PRIMARY_F = 0.75 * 1023.0 / 20348;
        public static final double PRIMARY_P = 0.01;
        public static final double PRIMARY_I = 0;
        public static final double PRIMARY_D = 0;
        public static final int PRIMARY_INT_ZONE = 50;

        public static final double AUX_F = 0;//0.60 * 1023.0 / (440.0 * 10.0); // Maximum was 440 dps
        public static final double AUX_P = 0;
        public static final double AUX_I = 0;
        public static final double AUX_D = 0;
        public static final int AUX_INT_ZONE = 20;
        public static final boolean AUX_POLARITY = false;

        public static final int BASE_TRAJ_PERIOD_MS = 0;
        public static final int SENSOR_UNITS_PER_ROTATION = 2048;
        public static final double SENSOR_UNITS_PER_INCH_LOW_GEAR = 
            SENSOR_UNITS_PER_ROTATION * LOW_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);
        public static final double SENSOR_UNITS_PER_INCH_HIGH_GEAR = 
            SENSOR_UNITS_PER_ROTATION * HIGH_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);
        public static final int MIN_POINTS_IN_TALON = 10;
        public static final int TRANSMIT_PERIOD_MS = 3;

        public static final double DEFAULT_CRUISE_VELOCITY = 4;
        public static final double DEFAULT_TARGET_ACCELERATION = 8;
    }

    public static final int TIMEOUT_MS = 10;

    // Joystick to Output mapping
    public static final double DRIVE_JOYSTICK_EXPONENT = 1;
    public static final double DRIVE_JOYSTICK_COEF = 1;

    public static final String LIMELIGHT_NAME_SHOOTER = "limelight";

    public static final double KP_AIM = 0.006;
	public static final double MINIMUM_POWER = 0.2;

	public static final double ANGLE_ADJUST_THRESHOLD = 1.0;

    /**
     * Joystick Constants
     */
    public static final double JOYSTICK_DEADBAND = 0.1;
}