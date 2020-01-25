package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import raidzero.robot.wrappers.LazyTalonSRX;

public class Climb extends Submodule {

    private static LazyTalonSRX motor;
    private static Climb instance = null;

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
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        motor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void stop() {
    }

    public void climb(double speed) {
        power = speed;
    }
}