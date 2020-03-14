package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import raidzero.robot.wrappers.LazyTalonSRX;

import raidzero.robot.Constants.TurretConstants;

public class Turret extends Submodule {

    public static class PeriodicIO {
        // Inputs
        public int position = 0; // in encoder ticks

        // Outputs
        // [-1.0, 1.0] if OPEN_LOOP, encoder ticks if POSITION
        public double demand = 0.0;
    }

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

    // Hardware components
    private LazyTalonSRX turretMotor;

    // Control state
    private ControlState controlState = ControlState.OPEN_LOOP;

    private PeriodicIO periodicIO = new PeriodicIO();

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
        
        periodicIO = new PeriodicIO();
        zero();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.position = turretMotor.getSelectedSensorPosition();
    }

    @Override
    public void update(double timestamp) {
        if (turretMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
            zero();
        }
    }

    @Override
    public void writePeriodicOutputs() {
        switch (controlState) {
            case OPEN_LOOP:
                turretMotor.set(ControlMode.PercentOutput, periodicIO.demand);
                break;
            case POSITION:
                turretMotor.set(ControlMode.Position, periodicIO.demand);
                break;
        }
    }

    @Override
    public void stop() {
        rotateManual(0.0);
        turretMotor.set(ControlMode.PercentOutput, 0.0);
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
        if (controlState != ControlState.POSITION) {
            controlState = ControlState.POSITION;
        }
        periodicIO.demand = -angle * TurretConstants.TICKS_PER_DEGREE;
    }

    /**
     * Rotates the turret using open-loop control.
     * 
     * Note: Positive (+) is clockwise
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void rotateManual(double percentOutput) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        periodicIO.demand = percentOutput;
    }

    public boolean isInOpenLoop() {
        return controlState == ControlState.OPEN_LOOP;
    }

    public boolean isAtPosition() {
        double error = Math.abs(Math.abs(periodicIO.position) - Math.abs(periodicIO.demand));
        return controlState == ControlState.POSITION &&
            error < TurretConstants.TOLERANCE;
    }
}
