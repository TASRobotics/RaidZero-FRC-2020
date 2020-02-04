package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants;

public class Turret extends Submodule {

    /**
     * 63:1
     */
    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static Turret instance = null;

    private LazyTalonSRX turretMotor;
    
    private double outputOpenLoop = 0;
    private double outputPosition = 0;
    private ControlState controlState = ControlState.OPEN_LOOP;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {}

    @Override
    public void init() {
        turretMotor = new LazyTalonSRX(Constants.turretMotorId);
        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);

        turretMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyClosed);
        turretMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyClosed);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                turretMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case POSITION:
                turretMotor.set(ControlMode.MotionMagic, outputPosition);
                break;
        }
        if (turretMotor.getSensorCollection().isRevLimitSwitchClosed()) {
            zero();
        }
    }

    @Override
    public void stop() {
        outputOpenLoop = 0;
        turretMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        turretMotor.setSelectedSensorPosition(0);
    }

    public void rotate(double deg) {
        outputPosition = deg * Constants.degToTic;
    }

    public void rotateManual(double pow) {
        // TODO: Factor out the constant
        outputOpenLoop = pow * 0.25;
    }
}