package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;

import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Turret extends Submodule {

    private static Turret instance = null;
    public static Turret getInstance() {
        if (instance == null) {
            instance = new Turret();
        }
        return instance;
    }

    public static enum ControlState {
        OPEN_LOOP, POSITION_CONTROL
    };

    private LazyTalonSRX turretMotor;

    // Control states
    private ControlState controlState;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;

    private Turret() {
        turretMotor = new LazyTalonSRX(Constants.turretId);
        turretMotor.configFactoryDefault();
        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        turretMotor.setSensorPhase(false);
        turretMotor.setInverted(false);

        turretMotor.configReverseLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
        turretMotor.configForwardLimitSwitchSource(
            LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    }

    /**
     * Turns the turret using open-loop control.
     * 
     * @param output open-loop output in [-1, 1], + is counterclockwise
     */
    public void turn(double output) {
        // TODO: Move the deadband out of here!!!
        if (Math.abs(output) < Constants.JOYSTICK_DEADBAND) {
            output = 0.0;
        }
        outputOpenLoop = MathUtil.clamp(output, -1.0, 1.0);
    }

    /**
     * Resets all outputs on start.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
    }

    @Override
    public void zero() {
        turretMotor.getSensorCollection().setQuadraturePosition(0, Constants.TIMEOUT_MS);
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        turretMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void update(double timestamp) {
        // Reset the encoder when the limit switch is reached
        if (turretMotor.getSensorCollection().isRevLimitSwitchClosed()) {
            zero();
        }
    }

    /**
     * Runs the turret motor.
     */
    @Override
    public void run() {
        switch (controlState){
            case OPEN_LOOP:
                turretMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case POSITION_CONTROL:
                turretMotor.set(ControlMode.MotionMagic, outputPosition);
                break;
        }
        
    }
}