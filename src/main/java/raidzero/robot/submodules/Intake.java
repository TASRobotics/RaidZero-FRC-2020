package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import raidzero.robot.wrappers.LazyTalonSRX;

public class Intake extends Submodule {

    private static Intake instance = null;
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public static enum ControlState {
        OPEN_LOOP
    };

    private LazyTalonSRX intakeMotor;
    private InactiveDoubleSolenoid solenoid;

    // Control states
    private ControlState controlState;

    private double outputOpenLoop = 0.0;

    private Intake() {
        intakeMotor = new LazyTalonSRX(Constants.hopperId);
        intakeMotor.configFactoryDefault();
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);

        solenoid = new InactiveDoubleSolenoid(Constants.intakeForwardId,
            Constants.intakeReverseId);
    }

    /**
     * Intakes balls using open-loop control.
     * 
     * @param output open-loop output in [-1, 1], + is counterclockwise
     */
    public void intakeBalls(double output) {
        outputOpenLoop = MathUtil.clamp(JoystickUtils.deadband(output), -1.0, 1.0);
    }

    public void moveUp() {
        solenoid.set(Value.kReverse);
    }

    public void moveDown() {
        solenoid.set(Value.kForward);
    }

    /**
     * Resets all outputs on start.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;
        outputOpenLoop = 0.0;
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
        solenoid.set(Value.kOff);
    }

    /**
     * Runs the turret motor.
     */
    @Override
    public void run() {
        switch (controlState){
            case OPEN_LOOP:
                intakeMotor.set(ControlMode.PercentOutput, outputOpenLoop);
                break;
        }
        
    }
}