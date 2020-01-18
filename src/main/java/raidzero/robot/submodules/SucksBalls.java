package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import raidzero.robot.Constants;

public class SucksBalls extends Submodule {

    private static SucksBalls instance = null;

   private static TalonSRX mouth;
    
    private static double suckForce = 0.0;

    public static SucksBalls getInstance() {
        if (instance == null) {
            instance = new SucksBalls();
        }
        return instance;
    }

    private SucksBalls() {
        mouth = new TalonSRX(Constants.throat);
        mouth.configFactoryDefault();
        mouth.setNeutralMode(NeutralMode.Brake);
        mouth.setInverted(false);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        mouth.set(ControlMode.PercentOutput, suckForce);
    }

    @Override
    public void stop() {
        suckForce = 0;
        mouth.set(ControlMode.PercentOutput, 0);
    }

    public void suck(double trigger) {
        if(trigger > Constants.joystickDeadband) {
            suckForce = trigger;
        }
    }
}