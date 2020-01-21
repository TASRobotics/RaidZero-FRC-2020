
package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import raidzero.robot.wrappers.LazyTalonFX;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants;
public class Drive extends Submodule {

    public static enum GearShift {
        HIGH, LOW
    }

    private static Drive instance = null;

    private LazyTalonFX leftLeader;
    private LazyTalonFX leftFollower;
    private LazyTalonFX rightLeader;
    private LazyTalonFX rightFollower;
    private InactiveDoubleSolenoid gearShift;
    
    private double coef;
    private double exp;

    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;
    private Value shift = Value.kReverse;

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
    }

    @Override
    public void init() {
        /**
         * Motors
         */
        //omg can i please put the motors into arrays plsssss
        leftLeader = new LazyTalonFX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, true, false);
        
        leftFollower = new LazyTalonFX(Constants.driveLeftFollowerId);
        configureMotor(leftFollower, true, false);
        leftFollower.follow(leftLeader);

        rightLeader = new LazyTalonFX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, false, false);
        
        rightFollower = new LazyTalonFX(Constants.driveRightFollowerId);
        configureMotor(rightFollower, false, false);
        rightFollower.follow(rightLeader);

        /**
         * Gear
         */
        gearShift = new InactiveDoubleSolenoid(Constants.driveGearshiftForwardId, 
            Constants.driveGearshiftReverseId);
        gearShift.setActive(true);
        setGearShift(GearShift.LOW);

        /**
         * Relation Control
         */
        exp = Constants.driveExponent;
        coef = Constants.driveCoef;
    }

    private void configureMotor(LazyTalonFX motor, boolean invertMotor, boolean invertSensorPhase) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setSensorPhase(invertSensorPhase);
        motor.setInverted(invertMotor);
    }

    @Override
    public void update(double timestamp) {
        outputLeftDrive = coef * Math.pow(outputLeftDrive, exp);
        outputRightDrive = coef * Math.pow(outputRightDrive, exp);
    }

    @Override
    public void run() {
        leftLeader.set(ControlMode.PercentOutput, outputLeftDrive);
        rightLeader.set(ControlMode.PercentOutput, outputRightDrive);
        gearShift.set(shift);
    }

    @Override
    public void stop() {
        outputLeftDrive = 0.0;
        outputRightDrive = 0.0;
        leftLeader.set(ControlMode.PercentOutput, 0.0);
        rightLeader.set(ControlMode.PercentOutput, 0.0);
    }

    public void tank(double leftJoystick, double rightJoystick) {
        if (Math.abs(leftJoystick) < Constants.joystickDeadband) {
            leftJoystick = 0.0;
        }
        if (Math.abs(rightJoystick) < Constants.joystickDeadband) {
            rightJoystick = 0.0;
        }
        outputLeftDrive = leftJoystick;
        outputRightDrive = rightJoystick;
    }

    public void arcade(double leftJoystick, double rightJoystick) {
        if (Math.abs(leftJoystick) < Constants.joystickDeadband) {
            leftJoystick = 0.0;
        }
        if (Math.abs(rightJoystick) < Constants.joystickDeadband) {
            rightJoystick = 0.0;
        }
        System.out.println(leftJoystick + " " + rightJoystick);
        outputLeftDrive = leftJoystick + rightJoystick;
        outputRightDrive = leftJoystick - rightJoystick;
    }

    public void setGearShift(GearShift mode) {
        shift = gearSolenoidValue(mode);
    }

    private Value gearSolenoidValue(GearShift gear) {
        if (gear == GearShift.HIGH) {
            return Value.kForward;
        } else {
            return Value.kReverse;
        }
    }
}