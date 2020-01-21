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
public class Intake extends Submodule {

    private static Intake instance = null;

    private static TalonSRX motor;
    private static DoubleSolenoid solenoid;
    
    private static double power = 0.0;

    private static Value position = Value.kOff;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        motor = new TalonSRX(Constants.intakeMotor);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(false);

        solenoid = new DoubleSolenoid(Constants.intakeOut, Constants.intakeIn);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        motor.set(ControlMode.PercentOutput, power);
        solenoid.set(position);
    }

    @Override
    public void stop() {
        position = Value.kOff;
        solenoid.set(Value.kOff);
        power = 0;
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void suck(double trigger) {
        if(trigger > Constants.joystickDeadband) {
            power = trigger;
            return;
        }
        power = 0;
    }

    public void invertStraw() {
        if(position == Value.kReverse) {
            position = Value.kForward;
            return;
        }
        position = Value.kReverse;
    }
}