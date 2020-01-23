package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.Constants;
import raidzero.robot.wrappers.LazyTalonFX;

public class Shooter extends Submodule {

    private static Shooter instance = null;
    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    public static enum ControlState {
        OPEN_LOOP, VELOCITY_CONTROL
    };

    private LazyTalonFX shooterMotor;

    // Control states
    private ControlState controlState;

    private double outputOpenLoop = 0.0;
    private double outputVelocity = 0.0;

    private Shooter() {
        shooterMotor = new LazyTalonFX(Constants.shooterId);
        shooterMotor.setInverted(true);
        shooterMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Turns the turret using open-loop control.
     * 
     * @param output open-loop output in [-1, 1], + is counterclockwise
     */
    public void spin(double output) {
        // TODO: Move the deadband out of here!!!
        if (Math.abs(output) < Constants.JOYSTICK_DEADBAND) {
            output = 0.0;
        }
        outputOpenLoop = output;
    }

    /**
     * Resets all outputs on start.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        shooterMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Runs the turret motor.
     */
    @Override
    public void run() {
        switch (controlState){
            case OPEN_LOOP:
                shooterMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case VELOCITY_CONTROL:
                shooterMotor.set(ControlMode.Velocity, outputVelocity);
                break;
        }
        
    }
}