package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.Constants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Climb extends Submodule {

    private static Climb instance = null;
    
    private LazyTalonSRX climbMotor;

    private boolean unlocked = false;
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
        climbMotor.setNeutralMode(NeutralMode.Brake);
        climbMotor.setInverted(true);
    }

    @Override
    public void run() {
        if (!unlocked) {
            outputOpenLoop = 0;
        }
        climbMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0;
        climbMotor.set(ControlMode.PercentOutput, 0);
    }

    public void unlock() {
        unlocked = true;
    }

    public void climb(double input) {
        outputOpenLoop = input;
    }

}