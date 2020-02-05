package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants;

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
    public void init() {
        hopperMotor = new LazyTalonSRX(Constants.hopperMotorId);
        hopperMotor.configFactoryDefault();
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.setInverted(true);
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

    public void moveBalls(int p1, double p2) {
        // TODO: Refactor & clarify code (remove controller specific code here)
        if (p1 == -1) {
            player2Ctrl(p2);
            return;
        }
        if (p1 >= 315 || p1 <= 45) {
            outputOpenLoop = 1;
            return;
        }
        if (p1 >= 225 && p1 <= 135) {
            outputOpenLoop = -1;
            return;
        }
    }

    private void player2Ctrl(double joy) {
        if (Math.abs(joy) < Constants.JOYSTICK_DEADBAND) {
            outputOpenLoop = 0;
            return;
        }
        outputOpenLoop = joy;
    }
}