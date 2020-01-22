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
     * 
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
    public static final int driveGearshiftForwardId = 1;
    public static final int driveGearshiftReverseId = 0;

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
    public static final int shooter = 1;
    public static final IdleMode shooterIdle = IdleMode.kCoast;
    public static final boolean shooterInvert = true;

    /**
     * Intake Constants
     */
    public static final int intakeMotor = 4;
    public static final int intakeOut = 2;
    public static final int intakeIn = 3;

    /**
     * Hopper Constants
     */
    public static final int myDog = 5;

    /**
     * Turret
     */
    public static final int puppy = 6;
    public static final double degToTic = 1000; //random # for now

    /**
     * Gaywheel
     */
    public static final int gayPride = 7;
    public static final int rainbows = 4;
    public static final int ponies = 5;

    // PID Constants
    public static final double gayPrideF = 0;
    public static final double gayPrideP = 0;
    public static final double gayPrideI = 0;
    public static final double gayPrideD = 0;

    public static final int POSITION_CONTROL_SLOT = 1;
}