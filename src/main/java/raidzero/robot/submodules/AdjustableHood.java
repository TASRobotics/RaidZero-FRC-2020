package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.Constants.AdjustableHoodConstants;

public class AdjustableHood extends Submodule {

    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static AdjustableHood instance = null;

    private LazyTalonSRX hoodMotor;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;

    private ControlState controlState = ControlState.OPEN_LOOP;

    public static AdjustableHood getInstance() {
        if (instance == null) {
            instance = new AdjustableHood();
        }
        return instance;
    }

    private AdjustableHood() {}

    @Override
    public void onInit() {
        hoodMotor = new LazyTalonSRX(AdjustableHoodConstants.MOTOR_ID);
        hoodMotor.configFactoryDefault();
        hoodMotor.setNeutralMode(AdjustableHoodConstants.NEUTRAL_MODE);
        hoodMotor.setInverted(AdjustableHoodConstants.INVERSION);
        hoodMotor.setSensorPhase(AdjustableHoodConstants.SENSOR_PHASE);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

        config.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen; //Open
        config.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen; //Closed

        config.slot0.kF = AdjustableHoodConstants.K_F;
        config.slot0.kP = AdjustableHoodConstants.K_P;
        config.slot0.kI = AdjustableHoodConstants.K_I;
        config.slot0.kD = AdjustableHoodConstants.K_D;
        config.slot0.integralZone = AdjustableHoodConstants.K_INTEGRAL_ZONE;

        hoodMotor.configAllSettings(config);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        outputOpenLoop = 0.0;
        outputPosition = 0.0;
    }

    @Override
    public void update(double timestamp) {
        if (hoodMotor.isRevLimitSwitchClosed() == 1) {
            zero();
        }
        SmartDashboard.putNumber("Hood Pos", hoodMotor.getSelectedSensorPosition());
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                hoodMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case POSITION:
                hoodMotor.set(ControlMode.Position, outputPosition);
                break;
        }
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        hoodMotor.setSelectedSensorPosition(0);
    }

    /**
     * Adjusts the hood using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void adjust(double percentOutput) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = percentOutput;
    }

    /**
     * Moves the hood to a position using closed-loop control.
     * 
     * TODO: Only have discrete positions
     * 
     * @param position position in encoder units
     */
    public void moveToPosition(double position) {
        controlState = ControlState.POSITION;
        outputPosition = position;
    }
}