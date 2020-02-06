package raidzero.robot;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

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
        public static final double PIGEON_SCALE = 3600.0 / 8192.0;

        // kF should stay at 0 since the drive is using an arbitrary feedforward term
        public static final double PRIMARY_F = 0.0;
        public static final double PRIMARY_P = 0;
        public static final double PRIMARY_I = 0;
        public static final double PRIMARY_D = 0;
        public static final int PRIMARY_INT_ZONE = 50;

        public static final double METERS_PER_INCH = 0.0254;
        public static final double INCHES_PER_METER = 1.0 / 0.0254;
        public static final int SENSOR_UNITS_PER_ROTATION = 2048;
        public static final double SENSOR_UNITS_PER_INCH_LOW_GEAR =
            SENSOR_UNITS_PER_ROTATION * LOW_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);
        public static final double SENSOR_UNITS_PER_INCH_HIGH_GEAR =
            SENSOR_UNITS_PER_ROTATION * HIGH_GEAR_RATIO / (WHEEL_DIAMETER_INCHES * Math.PI);

        public static final double LOOP_PERIOD_MS = 20;
        public static final double LOOP_PERIOD_SECONDS = LOOP_PERIOD_MS * 0.001;

        /**
         * Characterization constants
         * https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/robot-characterization/index.html
         */
        // Voltage needed to overcome the motor's static friction.
        public static final double kS = 0.0;

        // Voltage needed to hold (or "cruise") at a given constant velocity.
        public static final double kV = 0.0;

        // Voltage needed to induce a given acceleration in the motor shaft.
        public static final double kA = 0.0;

        public static final SimpleMotorFeedforward FEED_FORWARD = 
            new SimpleMotorFeedforward(kS, kV, kA);

        public static final double MAX_VELOCITY = 3.0; // in m/s
        public static final double MAX_ACCELERATION = 3.0; // in m/s

        // TODO: Measure this!
        public static final double TRACK_WIDTH_METERS = 0.5;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = 
            new DifferentialDriveKinematics(TRACK_WIDTH_METERS);

        public static final double MAX_VOLTAGE = 11;
        public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
            new DifferentialDriveVoltageConstraint(FEED_FORWARD, DRIVE_KINEMATICS, MAX_VOLTAGE);

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