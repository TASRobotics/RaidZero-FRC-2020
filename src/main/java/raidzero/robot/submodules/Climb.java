package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;

import raidzero.robot.Constants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Climb extends Submodule {

    private static Climb instance = null;

    private LazyTalonSRX climbMotor;

    private double outputOpenLoop = 0.0;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private Climb() {}

    @Override
    public void init() {
        climbMotor = new LazyTalonSRX(Constants.climbMotorId);
        climbMotor.configFactoryDefault();
    }

    @Override
    public void run() {
        climbMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0;
        climbMotor.set(ControlMode.PercentOutput, 0);
    }

    public void climb(double input) {
        outputOpenLoop = input;
    }
}