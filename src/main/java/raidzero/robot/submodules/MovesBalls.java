package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import raidzero.robot.Constants;

public class MovesBalls extends Submodule {

    private static MovesBalls instance = null;

    private static TalonSRX mover;
    private static double power;

    public static MovesBalls getInstance() {
        if (instance == null) {
            instance = new MovesBalls();
        }
        return instance;
    }

    private MovesBalls() {
        mover = new TalonSRX(Constants.myDog);
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

    public void moveMyBalls(int p1, double p2) {
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