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

    private static LazyTalonSRX motor;
    private static InactiveDoubleSolenoid solenoid;
    
    private static double power = 0.0;

    private static Value position = Value.kOff;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
    }

    @Override
    public void init() {
        
        motor = new LazyTalonSRX(Constants.intakeMotor);
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(true);

        solenoid = new InactiveDoubleSolenoid(Constants.intakeOut, Constants.intakeIn);
        solenoid.setActive(true);
    }

    @Override
    public void update(double timestamp) {
    }

    @Override
    public void run() {
        motor.set(ControlMode.PercentOutput, power);
    }

    @Override
    public void stop() {
        position = Value.kOff;
        solenoid.set(Value.kOff);
        power = 0;
        motor.set(ControlMode.PercentOutput, 0);
    }

    public void suck(double trigger) {
        if(Math.abs(trigger) > Constants.joystickDeadband) {
            power = trigger;
            return;
        }
        power = 0;
    }

    public void invertStraw() {
        invertPos();
        solenoid.set(position);
    }

    private void invertPos() {
        if(position == Value.kReverse) {
            position = Value.kForward;
            return;
        }
        position = Value.kReverse;
    }
}