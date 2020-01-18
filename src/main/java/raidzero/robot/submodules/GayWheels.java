package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class GayWheels extends Submodule {

    private static enum GayMan {
        on, off
    }

    private static GayWheels instance = null;

    private static TalonSRX lesbianMotor;
    private static DoubleSolenoid nonBinarySolenoid;



    public static GayWheels getInstance() {
        if (instance == null) {
            instance = new GayWheels();
        }
        return instance;
    }

    private GayWheels() {
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
    }

    @Override
    public void stop() {
    }
}