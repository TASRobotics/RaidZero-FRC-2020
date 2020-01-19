package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants;

/**
 * Sucks BOBA balls and definitely not other balls
 */
public class SucksBalls extends Submodule {

    private static SucksBalls instance = null;

    private static TalonSRX mouth;
    private static DoubleSolenoid straw;
    
    private static double suckForce = 0.0;

    private static Value position = Value.kOff;

    public static SucksBalls getInstance() {
        if (instance == null) {
            instance = new SucksBalls();
        }
        return instance;
    }

    private SucksBalls() {
        mouth = new TalonSRX(Constants.throat);
        mouth.configFactoryDefault();
        mouth.setNeutralMode(NeutralMode.Brake);
        mouth.setInverted(false);

        straw = new DoubleSolenoid(Constants.suckerr, Constants.suckyy);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        mouth.set(ControlMode.PercentOutput, suckForce);
        straw.set(position);
    }

    @Override
    public void stop() {
        position = Value.kOff;
        straw.set(Value.kOff);
        suckForce = 0;
        mouth.set(ControlMode.PercentOutput, 0);
    }

    public void suck(double trigger) {
        if(trigger > Constants.joystickDeadband) {
            suckForce = trigger;
        }
    }

    public void invertStraw() {
        if(position == Value.kReverse) {position = Value.kForward;}
        if(position == Value.kForward) {position = Value.kReverse;}
    }
}