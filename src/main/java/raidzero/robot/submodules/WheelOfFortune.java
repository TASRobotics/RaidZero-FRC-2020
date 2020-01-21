package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import raidzero.robot.wrappers.LazyTalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants;

public class WheelOfFortune extends Submodule {

    private static WheelOfFortune instance = null;

    private static LazyTalonSRX wheel;
    private static DoubleSolenoid solenoid;

    private static Value engaged = Value.kOff;
    private static double velo = 0;
    private static double pos = 0;
    private static boolean gayMan = false;

    public static WheelOfFortune getInstance() {
        if (instance == null) {
            instance = new WheelOfFortune();
        }
        return instance;
    }

    private WheelOfFortune() {
    }

    @Override
    public void init() {
        wheel = new LazyTalonSRX(Constants.gayPride);
        solenoid = new DoubleSolenoid(Constants.rainbows, Constants.ponies);

        wheel.config_kF(Constants.POSITION_CONTROL_SLOT, Constants.gayPrideF);
        wheel.config_kP(Constants.POSITION_CONTROL_SLOT, Constants.gayPrideP);
        wheel.config_kI(Constants.POSITION_CONTROL_SLOT, Constants.gayPrideI);
        wheel.config_kD(Constants.POSITION_CONTROL_SLOT, Constants.gayPrideD);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        solenoid.set(engaged);
        velocityControl();        
    }

    private void velocityControl() {
        if(gayMan) {
            wheel.set(ControlMode.PercentOutput, velo);
            return;
        }
        wheel.set(ControlMode.MotionMagic, pos);
    }

    @Override
    public void stop() {
        velo = 0;
        pos = 0;
        wheel.set(ControlMode.PercentOutput, 0);
        solenoid.set(Value.kOff);
    }

    private void manual(double speed) {
        velo = speed;
    }

    public void rotate(double input, boolean manual) {
        gayMan = manual;
        if(manual) {
            manual(input);
            return;
        }
        pos = input;
    }

    public void engage(boolean value) {
        if(value == true) {
            engaged = Value.kForward;
        }
        if(value == false) {
            engaged = Value.kReverse;
        }
    }


}