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
    //private TalonSRX leftFollower2;
    private TalonSRX rightLeader;
    private TalonSRX rightFollower;
    //private TalonSRX rightFollower2;

    private DoubleSolenoid gearShift;

    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
        leftLeader = new TalonSRX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, true, false);
        
        leftFollower = new TalonSRX(Constants.driveLeftFollowerId);
        configureMotor(leftFollower, true, false);
        leftFollower.follow(leftLeader);

        /*leftFollower2 = new TalonSRX(13);
        configureMotor(leftFollower2, true, false);
        leftFollower2.follow(leftLeader);*/

        rightLeader = new TalonSRX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, false, false);
        
        rightFollower = new TalonSRX(Constants.driveRightFollowerId);
        configureMotor(rightFollower, false, false);
        rightFollower.follow(rightLeader);

        /*rightFollower2 = new TalonSRX(5);
        configureMotor(rightFollower2, false, false);
        rightFollower2.follow(rightLeader);*/

        gearShift = new DoubleSolenoid(Constants.driveGearshiftForwardId, 
            Constants.driveGearshiftReverseId);
        setGearShift(GearShift.LOW);
    }

    private void configureMotor(TalonSRX motor, boolean invertMotor, boolean invertSensorPhase) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setSensorPhase(invertSensorPhase);
        motor.setInverted(invertMotor);
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
        if (mode == GearShift.HIGH) {
            gearShift.set(Value.kForward);
        } else {
            gearShift.set(Value.kReverse);
        }
    }
}