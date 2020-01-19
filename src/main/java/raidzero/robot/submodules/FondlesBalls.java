package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import raidzero.robot.Constants;

public class FondlesBalls extends Submodule {
    /**
     * 63:1
     */
    public static enum Mode {
        pow, pos
    };

    private static FondlesBalls instance = null;

    private static TalonSRX ballFondlers;
    
    private static double fondleSpeed = 0;
    private static double fondlePos = 0;
    private static Mode mode = Mode.pow;

    public static FondlesBalls getInstance() {
        if (instance == null) {
            instance = new FondlesBalls();
        }
        return instance;
    }

    private FondlesBalls() {
    }

    @Override
    public void init() {
        ballFondlers = new TalonSRX(Constants.ballFondler);

        ballFondlers.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen);
        ballFondlers.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        switch(mode){
            case pow:
                ballFondlers.set(ControlMode.PercentOutput, fondleSpeed);
                break;
            case pos:
                ballFondlers.set(ControlMode.MotionMagic, fondlePos);
                break;
        }
        if(ballFondlers.getSensorCollection().isRevLimitSwitchClosed()) {
            zero();
        }
    }

    @Override
    public void stop() {
        fondleSpeed = 0;
        ballFondlers.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        ballFondlers.setSelectedSensorPosition(0);
    }

    public void fondleMyBalls(double deg) {
        fondlePos = deg * Constants.degToTic;
    }

    public void fondleThemHard(double pow) {
        fondleSpeed = pow;
    }
}