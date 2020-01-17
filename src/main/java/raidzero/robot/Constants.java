package raidzero.robot;

import com.revrobotics.CANSparkMax.IdleMode;

public class Constants {

    /**
     * Drivetrain Constants
     */
    // Motor IDs
    public static final int driveLeftLeaderId = 1;
    public static final int driveLeftFollowerId = 3;
    public static final int driveRightLeaderId = 0;
    public static final int driveRightFollowerId = 2;

    // Solenoid IDs
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
    public static final int skinFlap = 1;
    public static final IdleMode skinFlapIdle = IdleMode.kCoast;
    public static final boolean skinFlapInvert = true;
}