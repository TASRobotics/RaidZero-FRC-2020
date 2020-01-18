package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.ProfileFollower;

public class Drive extends Submodule {

    public static enum GearShift {
        HIGH, LOW
    }

    public static enum ControlState {
        OPEN_LOOP, PATH_FOLLOWING
    }

    private static Drive instance = null;

    private TalonFX leftLeader; // Also the "ultimate master" for profiling
    private TalonFX leftFollower;
    private TalonFX rightLeader;
    private TalonFX rightFollower;
    /*private TalonSRX leftLeader;
    private TalonSRX leftFollower1;
    private TalonSRX leftFollower2;
    private TalonSRX rightLeader;
    private TalonSRX rightFollower1;
    private TalonSRX rightFollower2;*/

    private DoubleSolenoid gearShift;

    private PigeonIMU pigeon;

    // Control states
    private ControlState controlState;

    // Profile follower
    private ProfileFollower profileFollower;
    
    // Tunable constants
    private double coef;
    private double exp;

    // Output
    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;
    private int outputClosedLoop = 0;

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
        // Motors
        leftLeader = new TalonFX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, false, false);
        
        leftFollower = new TalonFX(Constants.driveLeftFollowerId);
        configureMotor(leftFollower, false, false);
        leftFollower.follow(leftLeader);

        rightLeader = new TalonFX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, true, true);
        
        rightFollower = new TalonFX(Constants.driveRightFollowerId);
        configureMotor(rightFollower, true, true);
        rightFollower.follow(rightLeader);
        /*leftLeader = new TalonSRX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, true, true);
        
        leftFollower1 = new TalonSRX(Constants.driveLeftFollower1Id);
        configureMotor(leftFollower1, true, true);
        leftFollower1.follow(leftLeader);

        leftFollower2 = new TalonSRX(Constants.driveLeftFollower2Id);
        configureMotor(leftFollower2, true, true);
        leftFollower2.follow(leftLeader);

        rightLeader = new TalonSRX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, false, true);
        
        rightFollower1 = new TalonSRX(Constants.driveRightFollower1Id);
        configureMotor(rightFollower1, false, true);
        rightFollower1.follow(rightLeader);

        rightFollower2 = new TalonSRX(Constants.driveRightFollower2Id);
        configureMotor(rightFollower2, false, true);
        rightFollower2.follow(rightLeader);*/

        // Pigeon IMU
        pigeon = new PigeonIMU(Constants.pigeonId);

        configureMotorClosedLoop();

        // Gear shift
        gearShift = new DoubleSolenoid(Constants.driveGearshiftForwardId, 
            Constants.driveGearshiftReverseId);
        setGearShift(GearShift.LOW);

        // Joystick-to-output mapping
        exp = Constants.driveExponent;
        coef = Constants.driveCoef;

        // Control state
        controlState = ControlState.OPEN_LOOP;
    }

    private void configureMotor(TalonFX motor, boolean invertMotor, boolean invertSensorPhase) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setSensorPhase(invertSensorPhase);
        motor.setInverted(invertMotor);
    }

    private void configureMotorClosedLoop() {
        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        // Configure the follower encoder as a remote sensor for the leader
        leftLeader.configRemoteFeedbackFilter(rightLeader.getDeviceID(),
                RemoteSensorSource.TalonSRX_SelectedSensor,	Constants.REMOTE_0);
        
        // Configure the Pigeon as the other Remote Slot on the leader
        leftLeader.configRemoteFeedbackFilter(pigeon.getDeviceID(), 
            RemoteSensorSource.Pigeon_Yaw, Constants.REMOTE_1);

        // Setup Sum signal to be used for distance
        leftLeader.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0);
        leftLeader.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.IntegratedSensor);

        // Configure Sum [Sum of both IntegratedSensor] to be used for Primary PID Index
        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 
            Constants.PID_PRIMARY_SLOT, Constants.TIMEOUT_MS);

        // Scale Feedback by 0.5 to half the sum of distance
        leftLeader.configSelectedFeedbackCoefficient(0.5, Constants.PID_PRIMARY_SLOT, 
            Constants.TIMEOUT_MS);

        // Configure Pigeon's Yaw to be used for Auxiliary PID Index
        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 
            Constants.PID_AUX_SLOT, Constants.TIMEOUT_MS);

        // Scale the Feedback Sensor using a coefficient (Configured for 360 units of resolution)
        leftLeader.configSelectedFeedbackCoefficient(
            Constants.PIGEON_SCALE, Constants.PID_AUX_SLOT, Constants.TIMEOUT_MS);

        // Set status frame periods to ensure we don't have stale data
        leftLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
        leftLeader.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        leftLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
        leftLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
        rightLeader.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);

        // FPID Gains for the distance part
        leftLeader.config_kP(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_P);
        leftLeader.config_kI(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_I);
        leftLeader.config_kD(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_D);
        leftLeader.config_kF(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_F);
        leftLeader.config_IntegralZone(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_INT_ZONE);

        // FPID Gains for turning part
        leftLeader.config_kP(Constants.PID_AUX_SLOT, Constants.AUX_P);
        leftLeader.config_kI(Constants.PID_AUX_SLOT, Constants.AUX_I);
        leftLeader.config_kD(Constants.PID_AUX_SLOT, Constants.AUX_D);
        leftLeader.config_kF(Constants.PID_AUX_SLOT, Constants.AUX_F);
        leftLeader.config_IntegralZone(Constants.PID_AUX_SLOT, Constants.AUX_INT_ZONE);

        // Set the period of the closed loops to be 1 ms
        leftLeader.configClosedLoopPeriod(Constants.PID_PRIMARY_SLOT, 
            Constants.CLOSED_LOOP_TIME_MS);
        leftLeader.configClosedLoopPeriod(Constants.PID_AUX_SLOT, 
            Constants.CLOSED_LOOP_TIME_MS);
        leftLeader.configAuxPIDPolarity(Constants.AUX_POLARITY);

        leftLeader.changeMotionControlFramePeriod(Constants.TRANSMIT_PERIOD_MS);
        leftLeader.configMotionProfileTrajectoryPeriod(Constants.BASE_TRAJ_PERIOD_MS);
        leftLeader.configMotionProfileTrajectoryInterpolationEnable(true);

        profileFollower = new ProfileFollower(leftLeader);
    }

    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        outputLeftDrive = 0.0;
        outputRightDrive = 0.0;
        outputClosedLoop = 0;
    }

    @Override
    public void update(double timestamp) {
        if (controlState == ControlState.PATH_FOLLOWING) {
            profileFollower.update();
            outputClosedLoop = profileFollower.getOutput();

            SmartDashboard.putNumber("Target Heading", 
                leftLeader.getActiveTrajectoryPosition(Constants.PID_AUX_SLOT));
            SmartDashboard.putNumber("Current Heading", 
                pigeon.getFusedHeading());
            //System.out.println(outputClosedLoop);
        }
        /*System.out.println(
            "Left: " + leftLeader.getSelectedSensorPosition() + ", " +
            "Right: " + rightLeader.getSelectedSensorPosition()
        );*/
    }

    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                leftLeader.set(ControlMode.PercentOutput, outputLeftDrive);
                rightLeader.set(ControlMode.PercentOutput, outputRightDrive);
                break;
            case PATH_FOLLOWING:
                leftLeader.set(ControlMode.MotionProfileArc, outputClosedLoop);
                rightLeader.follow(leftLeader, FollowerType.AuxOutput1);
                rightFollower.follow(leftLeader, FollowerType.AuxOutput1);
                break;
        }
    }

    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;

        outputLeftDrive = 0.0;
        outputRightDrive = 0.0;
        leftLeader.set(ControlMode.PercentOutput, 0.0);
        rightLeader.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void zero() {
        // setSelectedSensorPosition would set the Sensor Sum for the PID,
        // which is not what we want to do.
        leftLeader.getSensorCollection().setIntegratedSensorPosition(0, Constants.TIMEOUT_MS);
        rightLeader.getSensorCollection().setIntegratedSensorPosition(0, Constants.TIMEOUT_MS);
        pigeon.setYaw(0.0);
    }

    public void setOpenLoop() {
        controlState = ControlState.OPEN_LOOP;
    }

    public void tank(double leftJoystick, double rightJoystick) {
        if (Math.abs(leftJoystick) < Constants.joystickDeadband) {
            leftJoystick = 0.0;
        }
        if (Math.abs(rightJoystick) < Constants.joystickDeadband) {
            rightJoystick = 0.0;
        }
        outputLeftDrive = Math.copySign(coef * Math.pow(leftJoystick, exp), leftJoystick);
        outputRightDrive = Math.copySign(coef * Math.pow(rightJoystick, exp), rightJoystick);
        //System.out.println(outputLeftDrive + " " + outputRightDrive);
    }

    public void arcade(double leftJoystick, double rightJoystick) {
        if (Math.abs(leftJoystick) < Constants.joystickDeadband) {
            leftJoystick = 0.0;
        }
        if (Math.abs(rightJoystick) < Constants.joystickDeadband) {
            rightJoystick = 0.0;
        }
        outputLeftDrive = leftJoystick + rightJoystick;
        outputRightDrive = leftJoystick - rightJoystick;
    }

    public void setGearShift(GearShift mode) {
        gearShift.set(gearSolenoidValue(mode));
    }

    private Value gearSolenoidValue(GearShift gear) {
        if (gear == GearShift.HIGH) {
            return Value.kForward;
        } else {
            return Value.kReverse;
        }
    }

    public void setDrivePath(Path path) {
        if (profileFollower != null) {
            stop();
            zero();
            profileFollower.reset();
            profileFollower.setReverse(path.isReversed());
            profileFollower.start(path.getPoints(), path.getCruiseVelocity(), 
                path.getTargetAcceleration());
            controlState = ControlState.PATH_FOLLOWING;
        }
    }

    public boolean isFinishedWithPath() {
        if (profileFollower == null || controlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return profileFollower.isFinished();
    }
}