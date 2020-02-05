package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants;
import java.lang.Math;

/**
 * Sucks BOBA balls and definitely not other balls
 */
public class Intake extends Submodule {

    private static Intake instance = null;

    private LazyTalonSRX intakeMotor;
    private InactiveDoubleSolenoid solenoid;
    
    private double outputOpenLoop = 0.0;

    private Value position = Value.kOff;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {}

    @Override
    public void init() {
        intakeMotor = new LazyTalonSRX(Constants.intakeMotorId);
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);

        solenoid = new InactiveDoubleSolenoid(Constants.intakeOutId, Constants.intakeInId);
        solenoid.setActive(true);
    }

    @Override
    public void run() {
        intakeMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        position = Value.kOff;
        solenoid.set(Value.kOff);

        outputOpenLoop = 0;
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public void suck(double trigger) {
        if (Math.abs(trigger) < Constants.JOYSTICK_DEADBAND) {
            outputOpenLoop = 0;
            return;
        }
        outputOpenLoop = trigger;
    }

    public void invertStraw() {
        invertPos();
        solenoid.set(position);
    }

    private void invertPos() {
        if (position == Value.kReverse) {
            position = Value.kForward;
            return;
        }
        position = Value.kReverse;
    }
}