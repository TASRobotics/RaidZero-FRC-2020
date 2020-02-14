package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import raidzero.robot.Constants.WheelOfFortuneConstants;

public class WheelOfFortune extends Submodule {

    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static WheelOfFortune instance = null;

    private LazyTalonSRX wofMotor;
    private InactiveDoubleSolenoid solenoid;

    private double outputOpenLoop = 0.0;
    private double outputPosition = 0.0;

    private ControlState controlState = ControlState.OPEN_LOOP;

    public static WheelOfFortune getInstance() {
        if (instance == null) {
            instance = new WheelOfFortune();
        }
        return instance;
    }

    private WheelOfFortune() {}

    @Override
    public void onInit() {
        wofMotor = new LazyTalonSRX(WheelOfFortuneConstants.MOTOR_ID);
        wofMotor.configFactoryDefault();
        wofMotor.setNeutralMode(WheelOfFortuneConstants.NEUTRAL_MODE);
        wofMotor.setInverted(WheelOfFortuneConstants.INVERSION);

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        config.slot0.kF = WheelOfFortuneConstants.K_F;
        config.slot0.kP = WheelOfFortuneConstants.K_P;
        config.slot0.kI = WheelOfFortuneConstants.K_I;
        config.slot0.kD = WheelOfFortuneConstants.K_D;
        config.slot0.integralZone = WheelOfFortuneConstants.K_INTEGRAL_ZONE;

        wofMotor.configAllSettings(config);

        solenoid = new InactiveDoubleSolenoid(WheelOfFortuneConstants.WOF_FORWARD_ID, 
            WheelOfFortuneConstants.WOF_REVERSE_ID);
        solenoid.set(Value.kOff);
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                wofMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
            case POSITION:
                wofMotor.set(ControlMode.MotionMagic, outputPosition);
                break;
        }
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        outputPosition = 0.0;
        wofMotor.set(ControlMode.PercentOutput, 0);
    }

    public void spin(double percentOutput) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = percentOutput;
    }

    public void spinToPosition(double position) {
        controlState = ControlState.POSITION;
        outputPosition = position;
    }

    public void engage(boolean value) {
        if (value) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.set(Value.kReverse);
        }
    }


}