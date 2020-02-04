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

    public static final int driveGearshiftForwardId = 0;
    public static final int driveGearshiftReverseId = 1;

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
        // Joystick to Output mapping
        public static final double joystickExponent = 2;
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
    public static final double joystickDeadband = 0.1;
}