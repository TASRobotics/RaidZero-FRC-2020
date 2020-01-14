package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import raidzero.robot.Constants;

public class Drive extends Submodule {

    public static enum GearShift {
        HIGH, LOW
    }

    private static Drive instance = null;

    private TalonSRX leftLeader;
    private TalonSRX leftFollower;
    private TalonSRX rightLeader;
    private TalonSRX rightFollower;

    private DoubleSolenoid gearShift;
    
    private double coef;
    private double exp;

    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
        /**
         * Motors
         */
        //omg can i please put the motors into arrays plsssss
        leftLeader = new TalonSRX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, true, false);
        
        leftFollower = new TalonSRX(Constants.driveLeftFollowerId);
        configureMotor(leftFollower, true, false);
        leftFollower.follow(leftLeader);

        rightLeader = new TalonSRX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, false, false);
        
        rightFollower = new TalonSRX(Constants.driveRightFollowerId);
        configureMotor(rightFollower, false, false);
        rightFollower.follow(rightLeader);

        /**
         * Gear
         */
        gearShift = new DoubleSolenoid(Constants.driveGearshiftForwardId, 
            Constants.driveGearshiftReverseId);
        setGearShift(GearShift.LOW);

        /**
         * Relation Control
         */
        exp = Constants.driveExponent;
        coef = Constants.driveCoef;
    }

    private void configureMotor(TalonSRX motor, boolean invertMotor, boolean invertSensorPhase) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Coast);
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
        gearShift.set(gearValue(mode));
    }

    private Value gearValue(GearShift gear) {
        if(gear == GearShift.HIGH) {
            return Value.kForward;
        } else {
            return Value.kReverse;
        }
    }
}