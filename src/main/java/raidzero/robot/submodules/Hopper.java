package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Hopper extends Submodule {

    private static Hopper instance = null;
    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    public static enum ControlState {
        OPEN_LOOP
    };

    private LazyTalonSRX hopperMotor;

    // Control states
    private ControlState controlState;

    private double outputOpenLoop = 0.0;

    private Hopper() {
        hopperMotor = new LazyTalonSRX(Constants.hopperId);
        hopperMotor.configFactoryDefault();
        hopperMotor.setInverted(true);
        hopperMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Moves the hopper using open-loop control.
     * 
     * @param output open-loop output in [-1, 1], + is counterclockwise
     */
    public void move(double output) {
        outputOpenLoop = MathUtil.clamp(JoystickUtils.deadband(output), -1.0, 1.0);
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
        hopperMotor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Runs the turret motor.
     */
    @Override
    public void run() {
        switch (controlState){
            case OPEN_LOOP:
                hopperMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
        }
    }
}