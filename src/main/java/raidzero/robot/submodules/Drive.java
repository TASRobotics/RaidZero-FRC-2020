package raidzero.robot.submodules;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.wrappers.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.ProfileFollower;
import raidzero.robot.utils.EncoderUtils;

public class Drive extends Submodule {

    private static Drive instance = null;
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    public static enum GearShift {
        HIGH, LOW
    }

    public static enum ControlState {
        OPEN_LOOP, PATH_FOLLOWING
    }

	private LazyTalonFX leftLeader;
    private LazyTalonFX leftFollower;
    private LazyTalonFX rightLeader; // Also the "ultimate master" for profiling
    private LazyTalonFX rightFollower;

    // Should be references to one of the 4 motors
    private LazyTalonFX profilingLeader;
    private LazyTalonFX profilingFollower;

    private InactiveDoubleSolenoid gearShiftSolenoid;

    private PigeonIMU pigeon;

    // Control states
    private ControlState controlState;

    // Profile follower
    private ProfileFollower profileFollower;
    
    // Tunable constants
    private double coef;
    private double exp;

    private GearShift currentGearShift;

    // Output
    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;
    private int outputClosedLoop = 0;

    private Drive() {
        // Motors
        leftLeader = new LazyTalonFX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, false, false);
        
        leftFollower = new LazyTalonFX(Constants.driveLeftFollowerId);
        configureMotor(leftFollower, false, false);
        leftFollower.follow(leftLeader);

        rightLeader = new LazyTalonFX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, true, true);
        
        rightFollower = new LazyTalonFX(Constants.driveRightFollowerId);
        configureMotor(rightFollower, true, true);
        rightFollower.follow(rightLeader);

        // Pigeon IMU
        pigeon = new PigeonIMU(Constants.pigeonId);

        // Must be called after the pigeon is initialized
        configureMotorClosedLoop();

        // Gear shift
        gearShiftSolenoid = new InactiveDoubleSolenoid(Constants.driveGearshiftForwardId, 
			Constants.driveGearshiftReverseId);

        // Joystick-to-output mapping
        exp = Constants.DRIVE_JOYSTICK_EXPONENT;
        coef = Constants.DRIVE_JOYSTICK_COEF;

        // Control state
        controlState = ControlState.OPEN_LOOP;
    }

    /**
     * Configures a motor controller.
     * 
     * @param motor the motor controller to configure
     * @param invertMotor whether to invert the motor output
     * @param invertSensorPhase whether to invert the sensor output
     */
    private void configureMotor(LazyTalonFX motor, boolean invertMotor, boolean invertSensorPhase) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(invertMotor);
        motor.setSensorPhase(invertSensorPhase);
    }

    /**
     * Configures the motor controllers for closed-loop control.
     */
    private void configureMotorClosedLoop() {
        profilingLeader = leftLeader;
        profilingFollower = rightLeader;

        // Use the integrated sensors on the falcons as feedback
        profilingLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                                     Constants.PID_PRIMARY_SLOT, 
                                                     Constants.TIMEOUT_MS);

        // Configure the follower encoder as a remote sensor for the leader
        profilingLeader.configRemoteFeedbackFilter(profilingFollower.getDeviceID(),
                                                   RemoteSensorSource.TalonSRX_SelectedSensor,	
                                                   Constants.REMOTE_0, 
                                                   Constants.TIMEOUT_MS);
        
        // Configure the Pigeon as the other Remote Slot on the leader
        profilingLeader.configRemoteFeedbackFilter(pigeon.getDeviceID(),
                                                   RemoteSensorSource.Pigeon_Yaw, 
                                                   Constants.REMOTE_1,
                                                   Constants.TIMEOUT_MS);

        // Setup Sum signal to be used for distance
        profilingLeader.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0,
                                         Constants.TIMEOUT_MS);
        profilingLeader.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.IntegratedSensor,
                                         Constants.TIMEOUT_MS);

        // Configure Sum [Sum of both IntegratedSensor] to be used for Primary PID Index
        profilingLeader.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 
                                                     Constants.PID_PRIMARY_SLOT, 
                                                     Constants.TIMEOUT_MS);

        // Scale Feedback by 0.5 to half the sum of distance
        profilingLeader.configSelectedFeedbackCoefficient(0.5, 
                                                          Constants.PID_PRIMARY_SLOT, 
                                                          Constants.TIMEOUT_MS);

        // Configure Pigeon's Yaw to be used for Auxiliary PID Index
        profilingLeader.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 
                                                     Constants.PID_AUX_SLOT, 
                                                     Constants.TIMEOUT_MS);

        // Scale the Feedback Sensor using a coefficient (Configured for 360 units of resolution)
        profilingLeader.configSelectedFeedbackCoefficient(Constants.PIGEON_SCALE, 
                                                          Constants.PID_AUX_SLOT, 
                                                          Constants.TIMEOUT_MS);

        // Set status frame periods to ensure we don't have stale data
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);
        profilingFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);

        // PIDF Gains for the distance part
        profilingLeader.config_kP(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_P);
        profilingLeader.config_kI(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_I);
        profilingLeader.config_kD(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_D);
        profilingLeader.config_kF(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_F);
        profilingLeader.config_IntegralZone(Constants.PID_PRIMARY_SLOT, Constants.PRIMARY_INT_ZONE);

        // PIDF Gains for turning part
        profilingLeader.config_kP(Constants.PID_AUX_SLOT, Constants.AUX_P);
        profilingLeader.config_kI(Constants.PID_AUX_SLOT, Constants.AUX_I);
        profilingLeader.config_kD(Constants.PID_AUX_SLOT, Constants.AUX_D);
        profilingLeader.config_kF(Constants.PID_AUX_SLOT, Constants.AUX_F);
        profilingLeader.config_IntegralZone(Constants.PID_AUX_SLOT, Constants.AUX_INT_ZONE);

        // Set the period of the closed loops to be 1 ms
        profilingLeader.configClosedLoopPeriod(Constants.PID_PRIMARY_SLOT, 
                                               Constants.CLOSED_LOOP_TIME_MS,
                                               Constants.TIMEOUT_MS);
        profilingLeader.configClosedLoopPeriod(Constants.PID_AUX_SLOT, 
                                               Constants.CLOSED_LOOP_TIME_MS,
                                               Constants.TIMEOUT_MS);
        profilingLeader.configAuxPIDPolarity(Constants.AUX_POLARITY, Constants.TIMEOUT_MS);

        profilingLeader.changeMotionControlFramePeriod(Constants.TRANSMIT_PERIOD_MS);
        profilingLeader.configMotionProfileTrajectoryPeriod(Constants.BASE_TRAJ_PERIOD_MS,
                                                            Constants.TIMEOUT_MS);

        profileFollower = new ProfileFollower(profilingLeader);
    }

    /**
     * Resets all outputs on start.
     * 
     * @param timestamp
     */
    @Override
    public void onStart(double timestamp) {
        controlState = ControlState.OPEN_LOOP;

        outputLeftDrive = 0.0;
        outputRightDrive = 0.0;
        outputClosedLoop = 0;

        profilingLeader.clearMotionProfileTrajectories();
    }

    /**
     * Updates the profile follower if currently in path following mode.
     * 
     * @param timestamp
     */
    @Override
    public void update(double timestamp) {
        if (controlState == ControlState.PATH_FOLLOWING) {
            profileFollower.update();
            outputClosedLoop = profileFollower.getOutput();
        }
        
        SmartDashboard.putNumber("left inches", 
            EncoderUtils.ticksToInches(
                leftLeader.getSensorCollection().getIntegratedSensorPosition(), 
                currentGearShift
            )
        );
        SmartDashboard.putNumber("right inches", 
            EncoderUtils.ticksToInches(
                -rightLeader.getSensorCollection().getIntegratedSensorPosition(), 
                currentGearShift
            )
        );
    }

    /**
     * Runs the motors with different control modes depending on the state.
     */
    @Override
    public void run() {
        switch (controlState) {
            case OPEN_LOOP:
                leftLeader.set(ControlMode.PercentOutput, outputLeftDrive);
                rightLeader.set(ControlMode.PercentOutput, outputRightDrive);
                break;
            case PATH_FOLLOWING:
                profilingLeader.set(ControlMode.MotionProfileArc, outputClosedLoop);
                profilingFollower.follow(profilingLeader, FollowerType.AuxOutput1);
                break;
        }
    }

    /**
     * Stops all the motors. Also changes the control state to open-loop.
     */
    @Override
    public void stop() {
        controlState = ControlState.OPEN_LOOP;

        outputLeftDrive = 0.0;
        outputRightDrive = 0.0;

        // TODO: Make sure we don't ever directly set motor outputs
        leftLeader.set(ControlMode.PercentOutput, 0.0);
        rightLeader.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Zeros all encoders & the pigeon.
     */
    @Override
    public void zero() {
        /**
         * setSelectedSensorPosition would set the sensor sum for the PID,
         * which is not what we want to do.
         */
        leftLeader.getSensorCollection().setIntegratedSensorPosition(0, Constants.TIMEOUT_MS);
        rightLeader.getSensorCollection().setIntegratedSensorPosition(0, Constants.TIMEOUT_MS);
        pigeon.setYaw(0.0);
    }

    /**
     * Switches the base to open-loop control mode.
     */
    public void setOpenLoop() {
        controlState = ControlState.OPEN_LOOP;
    }

    /**
     * Tank drive mode for open-loop control.
     * 
     * @param leftJoystick value of the left joystick in [-1, 1]
     * @param rightJoystick value of the right joystick in [-1, 1]
     */
    public void tank(double leftJoystick, double rightJoystick) {
        if (Math.abs(leftJoystick) < Constants.JOYSTICK_DEADBAND) {
            leftJoystick = 0.0;
        }
        if (Math.abs(rightJoystick) < Constants.JOYSTICK_DEADBAND) {
            rightJoystick = 0.0;
        }
        outputLeftDrive = Math.copySign(coef * Math.pow(leftJoystick, exp), leftJoystick);
        outputRightDrive = Math.copySign(coef * Math.pow(rightJoystick, exp), rightJoystick);
    }

    /**
     * Arcade drive mode for open-loop control.
     * 
     * @param leftJoystick value of the left joystick in [-1, 1]
     * @param rightJoystick value of the right joystick in [-1, 1]
     */
    public void arcade(double leftJoystick, double rightJoystick) {
        if (Math.abs(leftJoystick) < Constants.JOYSTICK_DEADBAND) {
            leftJoystick = 0.0;
        }
        if (Math.abs(rightJoystick) < Constants.JOYSTICK_DEADBAND) {
            rightJoystick = 0.0;
        }
        outputLeftDrive = leftJoystick + rightJoystick;
        outputRightDrive = leftJoystick - rightJoystick;
    }

    /**
     * Shifts the gears depending on the mode.
     * 
     * @param mode the gear shift
     */
    public void setGearShift(GearShift mode) {
        currentGearShift = mode;
        gearShiftSolenoid.set(gearSolenoidValue(mode));
    }

    /**
     * Converts from a GearShift to a DoubleSolenoid value.
     * 
     * @param gear the gear shift
     * @return a DoubleSolenoid value
     */
    private Value gearSolenoidValue(GearShift gear) {
        if (gear == GearShift.HIGH) {
            return Value.kForward;
        } else {
            return Value.kReverse;
        }
    }

    /**
     * Makes the drive start following a Path.
     * 
     * @param path the path to follow
     */
    public void setDrivePath(Path path) {
        if (profileFollower != null) {
            // Stops & resets everything
            stop();
            zero();
            outputClosedLoop = SetValueMotionProfile.Disable.value;
            profileFollower.reset();

            profileFollower.setReverse(path.isReversed());
            profileFollower.start(path.getPathPoints());
            controlState = ControlState.PATH_FOLLOWING;
        }
    }
    
    /**
     * Returns whether the drive has finished following a path.
     * 
     * @return if the drive is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (profileFollower == null || controlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return profileFollower.isFinished();
    }
}