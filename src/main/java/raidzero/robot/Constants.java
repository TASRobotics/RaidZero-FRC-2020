package raidzero.robot;

public class Constants {

    /**
     * Drivetrain Constants
     */

    //Motor IDs
    public static final int driveLeftLeaderId = 1;
    public static final int driveLeftFollowerId = 3;
    public static final int driveRightLeaderId = 0;
    public static final int driveRightFollowerId = 2;

    //Solenoid IDs
    public static final int driveGearshiftForwardId = 0;
    public static final int driveGearshiftReverseId = 1;

    //Relation Control
    public static final double driveExponent = 2;
    public static final double driveCoef = 1;

    /**
     * Joystick Constants
     */

     //Deadband
    public static final double joystickDeadband = 0.1;
}