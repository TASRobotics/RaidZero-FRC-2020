package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.wrappers.LazyTalonSRX;

public class Climb extends Submodule {

    private static Climb instance = null;
    
    private static LazyTalonSRX motor;

    private static boolean unlocked = false;
    private static double power = 0.0;

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }
        return instance;
    }

    private Climb() {
    }
    
    @Override
    public void init() {
        motor = new LazyTalonSRX(12);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);
    }

    @Override
    public void run() {
        if(unlocked) {
            motor.set(ControlMode.PercentOutput, power);
            return;
        }
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void stop() {
        power = 0;
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void unlock() {
        unlocked = true;
    }

    public void climb(double input) {
        power = input;
    }

}