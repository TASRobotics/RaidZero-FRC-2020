package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import raidzero.robot.Constants;
import raidzero.robot.Constants.WheelOfFortuneConstants;

public class WheelOfFortune extends Submodule {

    public static enum ControlState {
        OPEN_LOOP, POSITION
    };

    private static WheelOfFortune instance = null;

    private LazyTalonSRX wofMotor;
    private InactiveDoubleSolenoid solenoid;

    private double outputOpenLoop = 0;
    private double outputPosition = 0;

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
        wofMotor = new LazyTalonSRX(Constants.wheelOfFortuneMotorId);
        wofMotor.configFactoryDefault();

        TalonSRXConfiguration config = new TalonSRXConfiguration();
        config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        config.slot0.kF = WheelOfFortuneConstants.kF;
        config.slot0.kP = WheelOfFortuneConstants.kP;
        config.slot0.kI = WheelOfFortuneConstants.kI;
        config.slot0.kD = WheelOfFortuneConstants.kD;
        config.slot0.integralZone = WheelOfFortuneConstants.kIntegralZone;

        wofMotor.configAllSettings(config);

        solenoid = new InactiveDoubleSolenoid(Constants.wheelOfFortuneUpId, Constants.wheelOfFortuneDownId);
        solenoid.setActive(true);
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
        outputOpenLoop = 0;
        outputPosition = 0;
        wofMotor.set(ControlMode.PercentOutput, 0);
        solenoid.set(Value.kOff);
    }

    private void spinOpenLoop(double input) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = input;
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