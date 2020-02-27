package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants.HopperConstants;

public class Hopper extends Submodule {

    private static Hopper instance = null;

    private LazyTalonSRX hopperMotor;
    
    private double outputOpenLoop = 0;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    private Hopper() {}

    @Override
    public void onInit() {
        hopperMotor = new LazyTalonSRX(HopperConstants.MOTOR_ID);
        hopperMotor.configFactoryDefault();
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.setInverted(true);
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        hopperMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Moves the conveyor belt using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void moveBelt(double percentOutput) {
        outputOpenLoop = percentOutput;
    }
}