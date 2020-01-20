package raidzero.robot.submodules;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import raidzero.robot.Constants;

public class EjectsBalls extends Submodule {

    private static CANSparkMax epididymis;
    private static EjectsBalls instance = null;

    private static double flowRate = 0.0;

    public static EjectsBalls getInstance() {
        if (instance == null) {
            instance = new EjectsBalls();
        }
        return instance;
    }

    private EjectsBalls() {
        /**
         * Epididymis Init
         */
        epididymis = new CANSparkMax(Constants.skinFlap, MotorType.kBrushless);
        epididymis.setInverted(Constants.skinFlapInvert);
        epididymis.setIdleMode(Constants.skinFlapIdle);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        epididymis.set(flowRate);
    }

    @Override
    public void stop() {
        flowRate = 0;
        epididymis.set(0);
    }

    public void shoot(double speed, boolean freeze) {
        if(freeze) {
            return;
        }
        if(Math.abs(speed) < Constants.joystickDeadband) {
            flowRate = 0;
            return;
        }
        flowRate = speed;
    }
}