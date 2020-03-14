package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import raidzero.robot.wrappers.LazyTalonSRX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.dashboard.Tab;

public class Intake extends Submodule {

    public static class PeriodicIO {
        // Inputs

        // Outputs
        // [-1.0, 1.0] if OPEN_LOOP, [-1.0, 1.0] if VELOCITY
        public double demand = 0.0;
    }

    public static enum Position {
        DOWN, UP
    }

    private static Intake instance = null;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
    }

    // Hardware components
    private LazyTalonSRX intakeMotor;
    private InactiveDoubleSolenoid solenoid;

    // Hardware states
    private Position position = Position.UP;

    private PeriodicIO periodicIO = new PeriodicIO();

    private NetworkTableEntry intakePositionEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Intake Position", position.toString())
        .withPosition(4, 2)
        .withSize(1, 1)
        .getEntry();

    @Override
    public void onInit() {
        intakeMotor = new LazyTalonSRX(IntakeConstants.MOTOR_ID);
        intakeMotor.configFactoryDefault();
        intakeMotor.setNeutralMode(IntakeConstants.NEUTRAL_MODE);
        intakeMotor.setInverted(IntakeConstants.INVERSION);

        solenoid = new InactiveDoubleSolenoid(IntakeConstants.INTAKE_FORWARD_ID,
                IntakeConstants.INTAKE_REVERSE_ID);
        setPosition(position);
    }

    @Override
    public void onStart(double timestamp) {
        periodicIO = new PeriodicIO();
    }

    @Override
    public void writePeriodicOutputs() {
        intakeMotor.set(ControlMode.PercentOutput, periodicIO.demand);
    }

    @Override
    public void stop() {
        periodicIO.demand = 0.0;
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Spins the intake using open-loop control.
     * 
     * @param percentOutput the percent output in [-1.0, 1.0]
     */
    public void intakeBalls(double percentOutput) {
        periodicIO.demand = percentOutput;
    }

    /**
     * Sets the position of the intake.
     * 
     * @param pos the target position
     */
    public void setPosition(Position pos) {
        position = pos;
        intakePositionEntry.setString(position.toString());
        if (position == Position.DOWN) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.set(Value.kReverse);
        }
    }

    /**
     * Moves the intake out or in depending on what state it is in.
     */
    public void invertStraw() {
        invertPos();
        setPosition(position);
    }

    private void invertPos() {
        if (position == Position.DOWN) {
            position = Position.UP;
            return;
        }
        position = Position.DOWN;
    }
}
