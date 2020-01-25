package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.Constants;

public class Shooter extends Submodule {

    private static LazyTalonFX motor;
    private static Shooter instance = null;

    private static double shooterSpeed = 0.0;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        /**
         * Epididymis Init
         */
        motor = new LazyTalonFX(8);
        motor.setInverted(Constants.shooterInvert);
        motor.setNeutralMode(NeutralMode.Coast);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        motor.set(ControlMode.PercentOutput, shooterSpeed);
    }

    @Override
    public void stop() {
        shooterSpeed = 0;
        motor.set(ControlMode.PercentOutput,0);
    }

    public void shoot(double speed, boolean freeze) {
        if(freeze) {
            return;
        }
        if(Math.abs(speed) < Constants.joystickDeadband) {
            shooterSpeed = 0;
            return;
        }
        shooterSpeed = speed;
    }
}