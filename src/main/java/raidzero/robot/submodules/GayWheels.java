package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants;

public class GayWheels extends Submodule {

    private static GayWheels instance = null;

    private static TalonSRX lesbianMotor;
    private static DoubleSolenoid nonBinarySolenoid;

    private static Value engaged = Value.kOff;
    private static double velo = 0;
    private static double pos = 0;
    private static boolean gayMan = false;

    public static GayWheels getInstance() {
        if (instance == null) {
            instance = new GayWheels();
        }
        return instance;
    }

    private GayWheels() {
    }

    @Override
    public void init() {
        lesbianMotor = new TalonSRX(Constants.gayPride);
        nonBinarySolenoid = new DoubleSolenoid(Constants.rainbows, Constants.ponies);

        lesbianMotor.config_kF(Constants.POSITION_CONTROL_SLOT, Constants.gayF);
        lesbianMotor.config_kP(Constants.POSITION_CONTROL_SLOT, Constants.gayP);
        lesbianMotor.config_kI(Constants.POSITION_CONTROL_SLOT, Constants.gayI);
        lesbianMotor.config_kD(Constants.POSITION_CONTROL_SLOT, Constants.gayD);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        nonBinarySolenoid.set(engaged);
        velocityControl();        
    }

    private void velocityControl() {
        if(gayMan) {
            lesbianMotor.set(ControlMode.PercentOutput, velo);
            return;
        }
        lesbianMotor.set(ControlMode.MotionMagic, pos);
    }

    @Override
    public void stop() {
        velo = 0;
        pos = 0;
        lesbianMotor.set(ControlMode.PercentOutput, 0);
        nonBinarySolenoid.set(Value.kOff);
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