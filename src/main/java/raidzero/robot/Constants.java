package raidzero.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.revrobotics.CANSparkMax.IdleMode;

public class Constants {
    /**
     * TalonSRX IDs
     * driveLeft 1, 3
     * driveRight 0, 2
     * intake 4
     * hopper 5
     * turret 6
     * rotator 7
     */
    /**
     * Pneumatic ID's
     * Drive 0, 1
     * Intake 2, 3
     * Wheel 4, 5
     */

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

    public static final int shooterMotorId = 8;

    public static final int intakeMotorId = 4;

    public static final int intakeOutId = 2;
    public static final int intakeInId = 3;

    public static final int hopperMotorId = 5;

    public static final int turretMotorId = 6;

    public static final int wheelOfFortuneMotorId = 7;

    public static final int wheelOfFortuneUpId = 4;
    public static final int wheelOfFortuneDownId = 5;

    public static final int climbMotorId = 12;

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

        public static final double PRIMARY_F = 0.8 * 1023.0 / 20348;
        public static final double PRIMARY_P = 0.01; // 1023 / (30 * 2000)
        public static final double PRIMARY_I = 0;
        public static final double PRIMARY_D = 0;
        public static final int PRIMARY_INT_ZONE = 50;

        public static final double AUX_F = 0;
        public static final double AUX_P = 4;
        public static final double AUX_I = 0;
        public static final double AUX_D = 0.4;//4.0;
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

        public static final double DEFAULT_CRUISE_VELOCITY = 9;
        public static final double DEFAULT_TARGET_ACCELERATION = 8;

        // Joystick to Output mapping
        public static final double joystickExponent = 1;
        public static final double joystickCoefficient = 1;
    }

    /**
     * Shooter Constants
     */
    public static final class ShooterConstants {
        public static final IdleMode shooterIdleMode = IdleMode.kCoast;
        public static final InvertType inversion = InvertType.InvertMotorOutput;

        public static final double maxSpeed = 18300; // in ticks per 100ms

        public static final double kF = maxSpeed / 1023.0;
        public static final double kP = 0;
        public static final double kI = 0; // Shouldn't be touched
        public static final double kD = 0; // Shouldn't be touched
        public static final int kIntegralZone = 0; // Shouldn't be touched
    }

    /**
     * Turret Constants
     */
    public static final class TurretConstants {
        public static final double degreesToTicks = 1000; //random # for now

        public static final double kF = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kIntegralZone = 0;
    }

    /**
     * Wheel of Fortune Constants
     */
    public static final class WheelOfFortuneConstants {
        public static final double kF = 0;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final int kIntegralZone = 0;
    }
    
    /**
     * Joystick Constants
     */
    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final int TIMEOUT_MS = 10;

    public static final String LIMELIGHT_NAME_SHOOTER = "limelight";

    public static final double KP_AIM = 0.02;

    public static final double ANGLE_ADJUST_THRESHOLD = 0.5;   
}