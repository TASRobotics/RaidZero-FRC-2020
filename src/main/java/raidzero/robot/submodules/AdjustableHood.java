package raidzero.robot.submodules;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.Constants.HoodConstants;
import raidzero.robot.Constants.HoodConstants.HoodAngle;
import raidzero.robot.dashboard.Tab;

public class AdjustableHood extends Submodule {

    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static AdjustableHood instance = null;

    public static AdjustableHood getInstance() {
        if (instance == null) {
            instance = new AdjustableHood();
        }
        return instance;
    }

    private AdjustableHood() {
    }

    private LazyTalonSRX hoodMotor;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;

    private ControlState controlState = ControlState.OPEN_LOOP;

    /*private NetworkTableEntry hoodPositionEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Hood Position", 0)
        .withWidget(BuiltInWidgets.kDial)
        .withProperties(Map.of("min", 0, "max", 7000))
        .withSize(2, 2)
        .withPosition(0, 0)
        .getEntry();*/

    @Override
    public void onInit() {
        hoodMotor = new LazyTalonSRX(HoodConstants.MOTOR_ID);
        hoodMotor.configFactoryDefault();
        hoodMotor.setNeutralMode(HoodConstants.NEUTRAL_MODE);
        hoodMotor.setInverted(HoodConstants.INVERSION);
        hoodMotor.setSensorPhase(HoodConstants.INVERT_SENSOR_PHASE);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;

        config.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
        config.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
        config.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;

        config.slot0.kF = HoodConstants.K_F;
        config.slot0.kP = HoodConstants.K_P;
        config.slot0.kI = HoodConstants.K_I;
        config.slot0.kD = HoodConstants.K_D;
        config.slot0.integralZone = HoodConstants.K_INTEGRAL_ZONE;

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
        SmartDashboard.putNumber("Hood Angle", hoodMotor.getSelectedSensorPosition());
        //hoodPositionEntry.setNumber(hoodMotor.getSelectedSensorPosition());
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
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        hoodMotor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void zero() {
        hoodMotor.setSelectedSensorPosition(0);
    }

    /**
     * Returns the position of the hood.
     * 
     * @return position in encoder ticks
     */
    public int getPosition() {
        return hoodMotor.getSelectedSensorPosition();
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
     * @param position position in encoder units
     */
    public void moveToTick(double position) {
        controlState = ControlState.POSITION;
        outputPosition = position;
    }

    /**
     * Moves to hood to a specific hood angle.
     * 
     * @param angle hood angle to move to
     */
    public void moveToAngle(HoodAngle angle) {
        moveToTick(angle.ticks);
    }

    public boolean isAtPosition() {
        return controlState == ControlState.POSITION &&
               Math.abs(hoodMotor.getClosedLoopError()) < HoodConstants.TOLERANCE;
    }
}
