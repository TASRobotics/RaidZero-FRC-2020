package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import raidzero.robot.wrappers.LazyTalonFX;

import raidzero.robot.Constants.HopperConstants;

public class Hopper extends Submodule {

    public static class PeriodicIO {
        // Inputs

        // Outputs
        // [-1.0, 1.0] if OPEN_LOOP, [-1.0, 1.0] if VELOCITY
        public double demand = 0.0;
    }

    public static enum ControlState {
        OPEN_LOOP, VELOCITY
    }

    private static Hopper instance = null;

    public static Hopper getInstance() {
        if (instance == null) {
            instance = new Hopper();
        }
        return instance;
    }

    private Hopper() {
    }

    // Hardware components
    private LazyTalonFX hopperMotor;

    // Control state
    private ControlState controlState = ControlState.OPEN_LOOP;

    private PeriodicIO periodicIO = new PeriodicIO();

    @Override
    public void onInit() {
        hopperMotor = new LazyTalonFX(HopperConstants.MOTOR_ID);
        hopperMotor.configFactoryDefault();
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.setInverted(HopperConstants.INVERSION);
        hopperMotor.setSensorPhase(HopperConstants.FLIP_SENSOR_PHASE);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        config.slot0.kF = HopperConstants.K_F;
        config.slot0.kP = HopperConstants.K_P;
        config.slot0.kI = HopperConstants.K_I;
        config.slot0.kD = HopperConstants.K_D;
        config.slot0.integralZone = HopperConstants.K_INTEGRAL_ZONE;

        hopperMotor.configAllSettings(config);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        periodicIO = new PeriodicIO();
    }

    @Override
    public void writePeriodicOutputs() {
        switch (controlState) {
            case OPEN_LOOP:
                hopperMotor.set(ControlMode.PercentOutput, periodicIO.demand);
                break;
            case VELOCITY:
                hopperMotor.set(ControlMode.Velocity,
                    periodicIO.demand * HopperConstants.MAX_SPEED);
                break;
        }
    }

    @Override
    public void stop() {
        moveBelt(0.0);
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Moves the conveyor belt using open-loop control.
     * 
     * @param percentOutput the percent output in [-1.0, 1.0]
     */
    public void moveBelt(double percentOutput) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        periodicIO.demand = percentOutput;
    }

    public void moveAtVelocity(double percentVelocity) {
        if (Math.abs(percentVelocity) < 0.1) {
            moveBelt(0.0);
            return;
        }
        if (controlState != ControlState.VELOCITY) {
            controlState = ControlState.VELOCITY;
        }
        periodicIO.demand = percentVelocity;
    }
}
