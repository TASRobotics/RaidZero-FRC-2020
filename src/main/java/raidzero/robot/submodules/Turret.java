package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants;
import raidzero.robot.Constants.TurretConstants;

public class Turret extends Submodule {

    /**
     * 63:1
     */
    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static Turret instance = null;

    private LazyTalonSRX turretMotor;
    
    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;
    private ControlState controlState = ControlState.OPEN_LOOP;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {}

    @Override
    public void onInit() {
        turretMotor = new LazyTalonSRX(Constants.turretMotorId);
        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(NeutralMode.Brake);
        turretMotor.setInverted(true);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        config.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;
        config.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyClosed;

        config.slot0.kF = TurretConstants.kF;
        config.slot0.kP = TurretConstants.kP;
        config.slot0.kI = TurretConstants.kI;
        config.slot0.kD = TurretConstants.kD;
        config.slot0.integralZone = TurretConstants.kIntegralZone;

        turretMotor.configAllSettings(config);
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
        outputOpenLoop = 0.0;
        turretMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        turretMotor.setSelectedSensorPosition(0);
    }

    /**
     * Rotates the turret to the specified angle using closed-loop PID.
     * 
     * @param angle the angle to rotate to
     */
    public void rotateToAngle(double angle) {
        outputPosition = angle * TurretConstants.degreesToTicks;
    }

    /**
     * Rotates the turret using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void rotateManual(double percentOutput) {
        outputOpenLoop = percentOutput;
    }
}