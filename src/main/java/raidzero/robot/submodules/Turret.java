package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import raidzero.robot.Constants;

public class Turret extends Submodule {
    /**
     * 63:1
     */
    public static enum Mode {
        pow, pos
    };

    private static Turret instance = null;

    private static TalonSRX motor;
    
    private static double speed = 0;
    private static double pos = 0;
    private static Mode mode = Mode.pow;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {
    }

    @Override
    public void init() {
        motor = new TalonSRX(Constants.puppy);

        motor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyClosed);
        motor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyClosed);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        switch(mode){
            case pow:
                motor.set(ControlMode.PercentOutput,speed);
                break;
            case pos:
                motor.set(ControlMode.MotionMagic, pos);
                break;
        }
        if(motor.getSensorCollection().isRevLimitSwitchClosed()) {
            zero();
        }
        System.out.println(motor.getSensorCollection().getQuadraturePosition());
    }

    @Override
    public void stop() {
        speed = 0;
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        motor.setSelectedSensorPosition(0);
    }

    public void fondleMyBalls(double deg) {
        pos = deg * Constants.degToTic;
    }

    public void fondleThemHard(double pow) {
        speed = pow;
    }
}