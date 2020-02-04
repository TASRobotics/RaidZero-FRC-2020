package raidzero.robot;

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

    /**
     * Drivetrain Constants
     */
    // Motor IDs
    public static final int driveLeftLeaderId = 1;
    public static final int driveLeftFollowerId = 3;
    public static final int driveRightLeaderId = 0;
    public static final int driveRightFollowerId = 2;

    // Solenoid IDs
    // Forward should make the 
    public static final int driveGearshiftForwardId = 0;
    public static final int driveGearshiftReverseId = 1;

    // Joystick to Output mapping
    public static final double driveExponent = 2;
    public static final double driveCoef = 1;

    /**
     * Joystick Constants
     */
    public static final double joystickDeadband = 0.1;

    /**
     * Shooter Constants
     */
    public static final int shooterMotorId = 8;
    public static final IdleMode shooterIdleMode = IdleMode.kCoast;
    public static final boolean shooterInvert = true;

    public static final double shooterMaxSpeed = 18300; // in ticks per 100ms

    public static final double shooterF = shooterMaxSpeed / 1023.0;
    public static final double shooterP = 0;
    public static final double shooterI = 0; // Shouldn't be touched
    public static final double shooterD = 0; // Shouldn't be touched

    /**
     * Intake Constants
     */
    public static final int intakeMotorId = 4;
    public static final int intakeOutId = 2;
    public static final int intakeInId = 3;

    /**
     * Hopper Constants
     */
    public static final int hopperMotorId = 5;

    /**
     * Turret Constants
     */
    public static final int turretMotorId = 6;
    public static final double degToTic = 1000; //random # for now

    /**
     * Wheel of Fortune Constants
     */
    public static final int wheelOfFortuneMotorId = 7;
    public static final int wheelOfFortuneUpId = 4;
    public static final int wheelOfFortuneDownId = 5;

    public static final double wofF = 0;
    public static final double wofP = 0;
    public static final double wofI = 0;
    public static final double wofD = 0;

    /**
     * Climb Constants
     */
    public static final int climbMotorId = 12;
}