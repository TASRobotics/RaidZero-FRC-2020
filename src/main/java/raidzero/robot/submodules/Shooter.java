package raidzero.robot.submodules;

import raidzero.lib.wrapper.LazyTalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import raidzero.robot.Constants;
import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.dashboard.Tab;

public class Shooter extends Submodule {

    public static class PeriodicIO {
        // Inputs
        public int velocity = 0; // in encoder ticks per 100ms

        // Outputs
        public double demand = 0.0; // in percent [-1.0, 1.0]
    }

    public static enum ControlState {
        OPEN_LOOP, VELOCITY
    };

    private static Shooter instance = null;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
    }

    private LazyTalonFX shooterMotor;

    private ControlState controlState = ControlState.OPEN_LOOP;

    private PeriodicIO periodicIO = new PeriodicIO();

    private NetworkTableEntry shooterVelocityEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Shooter Vel", 0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(0, 2)
        .getEntry();
    private NetworkTableEntry shooterUpToSpeedEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Up To Speed", false)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1)
        .withPosition(1, 2)
        .getEntry();

    @Override
    public void onInit() {
        shooterMotor = new LazyTalonFX(ShooterConstants.MOTOR_ID);
        shooterMotor.configFactoryDefault();
        shooterMotor.setNeutralMode(ShooterConstants.NEUTRAL_MODE);
        shooterMotor.setInverted(ShooterConstants.INVERSION);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        config.slot0.kF = ShooterConstants.K_F;
        config.slot0.kP = ShooterConstants.K_P;
        config.slot0.kI = ShooterConstants.K_I;
        config.slot0.kD = ShooterConstants.K_D;
        config.slot0.integralZone = ShooterConstants.K_INTEGRAL_ZONE;

        shooterMotor.configAllSettings(config);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        periodicIO = new PeriodicIO();
        zero();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.velocity = shooterMotor.getSelectedSensorVelocity();
    }

    @Override
    public void update(double timestamp) {
        shooterVelocityEntry.setNumber(periodicIO.velocity);
        shooterUpToSpeedEntry.setBoolean(isUpToSpeed());
    }

    @Override
    public void writePeriodicOutputs() {
        switch (controlState) {
            case OPEN_LOOP:
                shooterMotor.set(ControlMode.PercentOutput, periodicIO.demand);
                break;
            case VELOCITY:
                shooterMotor.set(ControlMode.Velocity, 
                    periodicIO.demand * ShooterConstants.FAKE_MAX_SPEED);
                break;
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        periodicIO.demand = 0.0;
        shooterMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        shooterMotor.getSensorCollection().setIntegratedSensorPosition(0.0, Constants.TIMEOUT_MS);
    }

    /**
     * Fires up the shooter using closed-loop velocity control.
     * 
     * @param percentSpeed speed of the shooter in [-1.0, 1.0]
     * @param freeze       whether to disregard the speed and keep the previous
     *                     speed
     */
    public void shoot(double percentSpeed, boolean freeze) {
        if (freeze) {
            return;
        }
        if (Math.abs(percentSpeed) < 0.1) {
            controlState = ControlState.OPEN_LOOP;
            periodicIO.demand = 0.0;
            return;
        }
        if (controlState != ControlState.VELOCITY) {
            controlState = ControlState.VELOCITY;
        }
        periodicIO.demand = percentSpeed;
    }

    /**
     * Returns whether the shooter is up to the setpoint speed.
     * 
     * @return whether the shooter is up to speed
     */
    public boolean isUpToSpeed() {
        return controlState == ControlState.VELOCITY &&
               Math.abs(shooterMotor.getClosedLoopError()) < ShooterConstants.ERROR_TOLERANCE;
    }
}
