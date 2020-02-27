package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants.IntakeConstants;

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
    public void onInit() {
        intakeMotor = new LazyTalonSRX(IntakeConstants.MOTOR_ID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(IntakeConstants.NEUTRAL_MODE);
        intakeMotor.setInverted(IntakeConstants.INVERSION);

        solenoid = new InactiveDoubleSolenoid(IntakeConstants.INTAKE_FORWARD_ID, 
            IntakeConstants.INTAKE_REVERSE_ID);
    }

    @Override
    public void onStart(double timestamp) {
        outputOpenLoop = 0.0;
    }

    @Override
    public void run() {
        intakeMotor.set(ControlMode.PercentOutput, outputOpenLoop);
    }

    @Override
    public void stop() {
        outputOpenLoop = 0.0;
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Spins the intake using open-loop control.
     * 
     * @param percentOutput the percent output in [-1, 1]
     */
    public void intakeBalls(double percentOutput) {
        outputOpenLoop = percentOutput;
    }

    /**
     * Moves the intake out or in depending on what state it is in.
     */
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