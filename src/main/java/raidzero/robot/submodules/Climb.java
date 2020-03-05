package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWM;
import raidzero.robot.Constants.ClimbConstants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Climb extends Submodule {

    private static Climb instance = null;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private Climb() {
    }

    private LazyTalonSRX climbMotor;
    private PWM servo;

    private boolean unlocked = false;
    private double outputOpenLoop = 0.0;
    private int outputServoPosition = -1;

    @Override
    public void onInit() {
        servo = new PWM(9);
        climbMotor = new LazyTalonSRX(ClimbConstants.MOTOR_ID);
        climbMotor.configFactoryDefault();
        climbMotor.setNeutralMode(ClimbConstants.NEUTRAL_MODE);
        climbMotor.setInverted(ClimbConstants.INVERSION);
    }

    @Override
    public void onStart(double timestamp) {
        outputServoPosition = 1;
        outputOpenLoop = 0.0;
    }

    @Override
    public void update(double timestamp) {
        if (!unlocked) {
            outputServoPosition = 1;
            outputOpenLoop = 0.0;
        }
    }

    @Override
    public void run() {
        servo.setPosition(outputServoPosition);
        climbMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        climbMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean isUnlocked() {
        return unlocked;
    }

    /**
     * Locks the climb.
     */
    public void lock() {
        unlocked = false;
    }

    /**
     * Unlocks the climb.
     */
    public void unlock() {
        unlocked = true;
    }

    /**
     * Opens the servo.
     */
    public void openServo() {
        outputServoPosition = -1;
    }

    /**
     * Closes the servo.
     */
    public void closeServo() {
        outputServoPosition = 1;
    }

    /**
     * Climbs using open-loop control..
     * 
     * @param percentOutput percent output in [-1, 1]
     */
    public void climb(double percentOutput) {
        outputOpenLoop = percentOutput;
    }
}
