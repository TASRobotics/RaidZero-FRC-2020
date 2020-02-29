package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import raidzero.robot.wrappers.LazyTalonFX;

import raidzero.robot.Constants.HopperConstants;

public class Hopper extends Submodule {

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

    private Hopper() {}

    private LazyTalonFX hopperMotor;

    private ControlState controlState = ControlState.OPEN_LOOP;
    
    private double outputOpenLoop = 0.0;
    private double outputPercentVelocity = 0.0;

    @Override
    public void onInit() {
        hopperMotor = new LazyTalonFX(HopperConstants.MOTOR_ID);
        hopperMotor.configFactoryDefault();
        hopperMotor.setNeutralMode(NeutralMode.Brake);
        hopperMotor.setInverted(true);
        hopperMotor.setSensorPhase(HopperConstants.SENSOR_PHASE);

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
        outputOpenLoop = 0.0;
        outputPercentVelocity = 0.0;
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                hopperMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case VELOCITY:
                hopperMotor.set(ControlMode.Velocity, outputPercentVelocity * HopperConstants.MAX_SPEED);
                break;
        }
    }

    @Override
    public void stop() {                
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPercentVelocity = 0.0;
        hopperMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Moves the conveyor belt using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void moveBelt(double percentOutput) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = percentOutput;
    }

    public void moveAtVelocity(double percentVelocity) {
        if (Math.abs(percentVelocity) < 0.1) {
            stop();
            return;
        }
        controlState = ControlState.VELOCITY;
        outputPercentVelocity = percentVelocity;
    }
}