package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants;

public class Hopper extends Submodule {

    private static Hopper instance = null;

    private static LazyTalonSRX mover;
    private static double power;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    private Hopper() {
        mover = new LazyTalonSRX(Constants.myDog);
        mover.configFactoryDefault();
        mover.setNeutralMode(NeutralMode.Brake);
        mover.setInverted(true);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        mover.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void stop() {
        power = 0.0;
        mover.set(ControlMode.PercentOutput, 0);
    }

    public void moveBalls(int p1, double p2) {
        if(p1 == -1) {
            player2Ctrl(p2);
            return;
        }
        if(p1 >= 315 || p1 <= 45) {
            power = 1;
            return;
        }
        if(p1 >= 225 && p1 <= 135) {
            power = -1;
            return;
        }
    }

    private static void player2Ctrl(double joy) {
        if(Math.abs(joy) < Constants.joystickDeadband){
            power = 0;
            return;
        }
        power = joy;
    }
}