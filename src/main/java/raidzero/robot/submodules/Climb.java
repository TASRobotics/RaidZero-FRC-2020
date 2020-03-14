package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.PWM;
import raidzero.robot.Constants.ClimbConstants;
import raidzero.lib.wrapper.LazyTalonSRX;

public class Climb extends Submodule {

    public static class PeriodicIO {
        // Inputs

        // Outputs
        public double demand = 0.0; // in percent [-1.0, 1.0]
    }

    private static Climb instance = null;
    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private Climb() {
    }

    // Hardware components
    private LazyTalonSRX climbMotor;
    private PWM servo;

    // Hardware states
    private boolean unlocked = false;

    private PeriodicIO periodicIO = new PeriodicIO();

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
        periodicIO = new PeriodicIO();

        closeServo();
    }

    @Override
    public void update(double timestamp) {
        if (!unlocked) {
            periodicIO.demand = 0.0;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        climbMotor.set(ControlMode.PercentOutput, periodicIO.demand);
    }

    @Override
    public void stop() {
        periodicIO.demand = 0.0;
        climbMotor.set(ControlMode.PercentOutput, 0.0);
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
        servo.setPosition(-1);
    }

    /**
     * Closes the servo.
     */
    public void closeServo() {
        servo.setPosition(1);
    }

    /**
     * Climbs using open-loop control.
     * 
     * @param percentOutput percent output in [-1.0, 1.0]
     */
    public void climb(double percentOutput) {
        periodicIO.demand = percentOutput;
    }
}
