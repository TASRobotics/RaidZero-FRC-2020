package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants.TurretConstants;
import raidzero.robot.submodules.AdjustableHood.ControlState;

public class Turret extends Submodule {

    /**
     * 63:1
     */
    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static Turret instance = null;

    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    private Turret() {
    }

    private LazyTalonSRX turretMotor;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;
    private ControlState controlState = ControlState.OPEN_LOOP;

    @Override
    public void onInit() {
        turretMotor = new LazyTalonSRX(TurretConstants.MOTOR_ID);
        turretMotor.configFactoryDefault();
        turretMotor.setNeutralMode(TurretConstants.NEUTRAL_MODE);
        turretMotor.setInverted(TurretConstants.INVERSION);
        turretMotor.setSensorPhase(TurretConstants.INVERT_PHASE);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        config.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        config.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        config.peakOutputForward = TurretConstants.MAX_INPUT_PERCENTAGE;
        config.peakOutputReverse = -TurretConstants.MAX_INPUT_PERCENTAGE;

        config.slot0.kF = TurretConstants.K_F;
        config.slot0.kP = TurretConstants.K_P;
        config.slot0.kI = TurretConstants.K_I;
        config.slot0.kD = TurretConstants.K_D;
        config.slot0.integralZone = TurretConstants.K_INTEGRAL_ZONE;

        turretMotor.configAllSettings(config);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        zero();
    }

    @Override
    public void update(double timestamp) {
        if (turretMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
            zero();
        }
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                turretMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case POSITION:
                turretMotor.set(ControlMode.Position, outputPosition);
                break;
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        turretMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        turretMotor.setSelectedSensorPosition(0);
    }

    /**
     * Rotates the turret to the specified angle using closed-loop control.
     * 
     * @param angle the angle to rotate to
     */
    public void rotateToAngle(double angle) {
        controlState = ControlState.POSITION;
        outputPosition = -angle * TurretConstants.TICKS_PER_DEGREE;
    }

    /**
     * Rotates the turret using open-loop control.
     * 
     * Note: Positive (+) is clockwise
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void rotateManual(double percentOutput) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = percentOutput;
    }

    public boolean isInPercentMode() {
        return controlState == ControlState.OPEN_LOOP;
    }

    public boolean isAtPosition() {
        return controlState == ControlState.POSITION &&
               Math.abs(turretMotor.getClosedLoopError()) < TurretConstants.TOLERANCE;
    }
}
