package raidzero.robot.submodules;

import raidzero.robot.wrappers.LazyCANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import raidzero.robot.Constants;

public class Shooter extends Submodule {

    private static LazyCANSparkMax motor;
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
        motor = new LazyCANSparkMax(Constants.shooter, MotorType.kBrushless);
        motor.setInverted(Constants.shooterInvert);
        motor.setIdleMode(Constants.shooterIdle);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        motor.set(shooterSpeed);
    }

    @Override
    public void stop() {
        shooterSpeed = 0;
        motor.set(0);
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