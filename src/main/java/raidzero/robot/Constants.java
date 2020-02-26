package raidzero.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.utils.InterpolatingDouble;
import raidzero.robot.utils.InterpolatingTreeMap;

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

    /**
     * Drivetrain Constants
     */
    public static final class DriveConstants {
        public static final int LEFT_LEADER_ID = 0;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_LEADER_ID = 1;
        public static final int RIGHT_FOLLOWER_ID = 3;

        public static final int GEARSHIFT_FORWARD_ID = 1;
        public static final int GEARSHIFT_REVERSE_ID = 0;

        public static final int PIGEON_ID = 0;

        public static final InvertType LEFT_INVERSION = InvertType.None;
        public static final InvertType RIGHT_INVERSION = InvertType.InvertMotorOutput;

        public static final double HIGH_GEAR_RATIO = 9.98;
        public static final double LOW_GEAR_RATIO = 18.43;
    
        public static final double WHEEL_DIAMETER_INCHES = 6.0;

        // Closed-loop constants
        public static final double DRIVE_NEUTRAL_DEADBAND = 0.01;
        public static final int PID_PRIMARY_SLOT = 0;
        public static final int PID_AUX_SLOT = 1;
        public static final double PIGEON_SCALE = 3600.0 / 8192.0;

        public static final double PRIMARY_F = 0.8 * 1023.0 / 20348;
        public static final double PRIMARY_P = 0.03; // 1023 / (30 * 2000)
        public static final double PRIMARY_I = 0;
        public static final double PRIMARY_D = 0;
        public static final int PRIMARY_INT_ZONE = 100;

        public static final double AUX_F = 0;
        public static final double AUX_P = 8;
        public static final double AUX_I = 0;
        public static final double AUX_D = 0.6;//4.0;
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

        public static final double DEFAULT_CRUISE_VELOCITY = 6;
        public static final double DEFAULT_TARGET_ACCELERATION = 6;

        // Joystick to Output mapping
        public static final double JOYSTICK_EXPONENT = 1;
        public static final double JOYSTICK_COEFFICIENT = 1;
    }

    /**
     * Shooter Constants
     */
    public static final class ShooterConstants {
        public static final int MOTOR_ID = 8;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Coast;
        public static final InvertType INVERSION = InvertType.InvertMotorOutput;

        public static final double MAX_SPEED = 18300; // in ticks per 100ms
        public static final double ERROR_TOLERANCE = 500; // TODO: Make sure this is good
        public static final double APPROACH_SETPOINT_TIMEOUT = 4; // in seconds

        public static final double K_F = MAX_SPEED / 1023.0;
        public static final double K_P = 0;
        public static final double K_I = 0; // Shouldn't be touched
        public static final double K_D = 0; // Shouldn't be touched
        public static final int K_INTEGRAL_ZONE = 0; // Shouldn't be touched
    }

    /**
     * Adjustable Hood Constants
     */
    public static final class AdjustableHoodConstants {
        public static final int MOTOR_ID = 11;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.None;
        public static final boolean SENSOR_PHASE = true;

        public static final double GEAR_RATIO = 45.0;

        public static final double K_F = 0;
        public static final double K_P = 0;
        public static final double K_I = 0;
        public static final double K_D = 0;
        public static final int K_INTEGRAL_ZONE = 0;
    }

    /**
     * Turret Constants
     */
    public static final class TurretConstants {
        public static final int MOTOR_ID = 6;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.InvertMotorOutput;

        public static final double CONTROL_SCALING_FACTOR = 0.25;

        public static final double DEGREES_TO_TICKS = 1000; //random # for now

        public static final double MAX_INPUT_PERCENTAGE = 0.3;

        public static final double K_F = 0;
        public static final double K_P = 0;
        public static final double K_I = 0;
        public static final double K_D = 0;
        public static final int K_INTEGRAL_ZONE = 0;
    }

    /**
     * Intake Constants
     */
    public static final class IntakeConstants {
        public static final int MOTOR_ID = 4;

        public static final int INTAKE_FORWARD_ID = 2;
        public static final int INTAKE_REVERSE_ID = 3;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.None;

        public static final double CONTROL_SCALING_FACTOR = 0.625;
    }

    /**
     * Hopper Constants
     */
    public static final class HopperConstants {
        public static final int MOTOR_ID = 5;
    }

    /**
     * Wheel of Fortune Constants
     */
    public static final class WheelOfFortuneConstants {
        public static final int MOTOR_ID = 7;

        public static final int WOF_FORWARD_ID = 4;
        public static final int WOF_REVERSE_ID = 5;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.None;

        public static final double K_F = 0;
        public static final double K_P = 0;
        public static final double K_I = 0;
        public static final double K_D = 0;
        public static final int K_INTEGRAL_ZONE = 0;
    }

    /**
     * Climb Constants
     */
    public static final class ClimbConstants {
        public static final int MOTOR_ID = 12;

        public static final NeutralMode NEUTRAL_MODE = NeutralMode.Brake;
        public static final InvertType INVERSION = InvertType.InvertMotorOutput;
    }

    /**
     * Limelight Constants
     */
    public static final class LimelightConstants {
        public static final String NAME = "limelight";

        public static final double MOUNTING_ANGLE = 25.0; // in degrees
        public static final double MOUNTING_HEIGHT = 0.5; // in meters

        public static final double AIM_KP = 0.02;
        public static final double AIM_KI = 0.0;
        public static final double AIM_KD = 0.0;
        public static final double ANGLE_ADJUST_THRESHOLD = 0.5;
    }

    /**
     * Field Constants
     */
    public static final class FieldConstants {
        public static final double GOAL_HEIGHT = 2.3; // in meters
    }

    // Distance (m/s) to Speed (percent) Lookup Table
    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> 
        DISTANCE_TO_SPEED = new InterpolatingTreeMap<>(20);
    static {
        // TODO: Perhaps load this from a file?
        DISTANCE_TO_SPEED.put(new InterpolatingDouble(0.0), new InterpolatingDouble(0.0));
        DISTANCE_TO_SPEED.put(new InterpolatingDouble(1.0), new InterpolatingDouble(1.0));
        DISTANCE_TO_SPEED.put(new InterpolatingDouble(2.0), new InterpolatingDouble(1.0));
        DISTANCE_TO_SPEED.put(new InterpolatingDouble(3.0), new InterpolatingDouble(1.0));
        DISTANCE_TO_SPEED.put(new InterpolatingDouble(4.0), new InterpolatingDouble(1.0));
    }
    
    /**
     * Joystick Constants
     */
    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final int TIMEOUT_MS = 10;
}