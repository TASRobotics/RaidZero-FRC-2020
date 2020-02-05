
package raidzero.robot.submodules;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
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
    private ProfileFollower mpFollower;
    
    // Tunable constants
    private double coef;
    private double exp;

    private GearShift currentGearShift;

    // Output
    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;
    private int outputClosedLoop = 0;

    private Drive() {}

    @Override
    public void onInit() {
        // Motors
        leftLeader = new LazyTalonFX(Constants.driveLeftLeaderId);
        configureMotor(leftLeader, Constants.driveLeftInvert);
        
        leftFollower = new LazyTalonFX(Constants.driveLeftFollowerId);
        configureMotor(leftFollower, Constants.driveLeftInvert);
        leftFollower.follow(leftLeader);

        rightLeader = new LazyTalonFX(Constants.driveRightLeaderId);
        configureMotor(rightLeader, Constants.driveRightInvert);
        
        rightFollower = new LazyTalonFX(Constants.driveRightFollowerId);
        configureMotor(rightFollower, Constants.driveRightInvert);
        rightFollower.follow(rightLeader);

        // Pigeon IMU
        pigeon = new PigeonIMU(Constants.pigeonId);
        pigeon.configFactoryDefault();

        // Setup the profiling leader & follower
        profilingLeader = rightLeader;
        profilingFollower = leftLeader;

        // Must be called after the pigeon is initialized
        configureMotorClosedLoop(Constants.driveRightInvert);

        // Gear shift
        gearShiftSolenoid = new InactiveDoubleSolenoid(Constants.driveGearshiftForwardId, 
            Constants.driveGearshiftReverseId);

        // Joystick-to-output mapping
        exp = DriveConstants.joystickExponent;
        coef = DriveConstants.joystickCoefficient;

        // Control state
        controlState = ControlState.OPEN_LOOP;
    }

    /**
     * Configures a motor controller.
     * 
     * @param motor the motor controller to configure
     * @param inversion whether to invert the motor output
     */
    private void configureMotor(LazyTalonFX motor, InvertType inversion) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(inversion);
    }

    /**
     * Configures the motor controllers for closed-loop control.
     * 
     * @param leaderInversion whether the profiling leader is inverted or not
     */
    private void configureMotorClosedLoop(InvertType leaderInversion) {
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();

        // Use the integrated encoder on the falcons as feedback
        followerConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

        // Configure the follower encoder as a remote sensor for the leader
        leaderConfig.remoteFilter0.remoteSensorDeviceID = profilingFollower.getDeviceID();
        leaderConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

        // Configure robot distance math
        setRobotDistanceConfigs(leaderInversion, leaderConfig);

        // Configure the Pigeon as the other Remote Slot on the leader
        leaderConfig.remoteFilter1.remoteSensorDeviceID = pigeon.getDeviceID();
        leaderConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw;
        leaderConfig.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;
        leaderConfig.auxiliaryPID.selectedFeedbackCoefficient = DriveConstants.PIGEON_SCALE;
        leaderConfig.auxPIDPolarity = DriveConstants.AUX_POLARITY;

        // PIDF Gains for the distance part
        leaderConfig.slot0.kP = DriveConstants.PRIMARY_P;
        leaderConfig.slot0.kI = DriveConstants.PRIMARY_I;
        leaderConfig.slot0.kD = DriveConstants.PRIMARY_D;
        leaderConfig.slot0.kF = DriveConstants.PRIMARY_F;
        leaderConfig.slot0.integralZone = DriveConstants.PRIMARY_INT_ZONE;

        // PIDF Gains for turning part
        leaderConfig.slot1.kP = DriveConstants.AUX_P;
        leaderConfig.slot1.kI = DriveConstants.AUX_I;
        leaderConfig.slot1.kD = DriveConstants.AUX_D;
        leaderConfig.slot1.kF = DriveConstants.AUX_F;
        leaderConfig.slot1.integralZone = DriveConstants.AUX_INT_ZONE;

        leaderConfig.neutralDeadband = DriveConstants.DRIVE_NEUTRAL_DEADBAND;
        followerConfig.neutralDeadband = DriveConstants.DRIVE_NEUTRAL_DEADBAND;

        // Apply all settings
        profilingLeader.configAllSettings(leaderConfig);
        profilingFollower.configAllSettings(followerConfig);

        // Set status frame periods to ensure we don't have stale data
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.TIMEOUT_MS);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.TIMEOUT_MS);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.TIMEOUT_MS);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, Constants.TIMEOUT_MS);
        profilingFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.TIMEOUT_MS);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5, Constants.TIMEOUT_MS);

        profilingLeader.changeMotionControlFramePeriod(DriveConstants.TRANSMIT_PERIOD_MS);
        profilingLeader.configMotionProfileTrajectoryPeriod(DriveConstants.BASE_TRAJ_PERIOD_MS,
                                                            Constants.TIMEOUT_MS);

        mpFollower = new ProfileFollower(profilingLeader);
    }

    /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param masterInvertType Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
    private void setRobotDistanceConfigs(InvertType masterInvertType, TalonFXConfiguration masterConfig) {
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot distance.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   distance magnitude.  */

		/* Check if we're inverted */
		if (masterInvertType == InvertType.InvertMotorOutput) {
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.
				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.
				Will a sensor sum or difference give us a positive total magnitude?
				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.
					Phase | Term 0   |   Term 1  | Result
				Sum:  -1 *((-)Master + (+)Aux   )| NOT OK, will cancel each other out
				Diff: -1 *((-)Master - (+)Aux   )| OK - This is what we want, magnitude will be correct and positive.
				Diff: -1 *((+)Aux    - (-)Master)| NOT OK, magnitude will be correct but negative
			*/
			masterConfig.diff0Term = FeedbackDevice.IntegratedSensor; // Local Integrated Sensor
			masterConfig.diff1Term = FeedbackDevice.RemoteSensor0;    // Aux Selected Sensor
			masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorDifference; // Diff0 - Diff1
		} else {
			// Master is not inverted, both sides are positive so we can sum them.
			masterConfig.sum0Term = FeedbackDevice.RemoteSensor0;    // Aux Selected Sensor
			masterConfig.sum1Term = FeedbackDevice.IntegratedSensor; // Local IntegratedSensor
			masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum; // Sum0 + Sum1
		}

		/* Since the Distance is the sum of the two sides, divide by 2 so the total isn't double
		   the real-world value */
		masterConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
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
            mpFollower.update();
            outputClosedLoop = mpFollower.getOutput();
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
        leftLeader.set(ControlMode.Disabled, 0.0);
        rightLeader.set(ControlMode.Disabled, 0.0);
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
        }
        if (gear == GearShift.LOW) {
            return Value.kReverse;
        }
        return Value.kOff;
    }

    /**
     * Makes the drive start following a Path.
     * 
     * @param path the path to follow
     */
    public void setDrivePath(Path path) {
        if (mpFollower != null) {
            // Stops & resets everything
            stop();
            zero();
            outputClosedLoop = SetValueMotionProfile.Disable.value;
            mpFollower.reset();

            mpFollower.setGearShift(currentGearShift);
            mpFollower.setReverse(path.isReversed());
            mpFollower.start(path.getPathPoints());
            controlState = ControlState.PATH_FOLLOWING;
        }
    }
    
    /**
     * Returns whether the drive has finished following a path.
     * 
     * @return if the drive is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (mpFollower == null || controlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return mpFollower.isFinished();
    }
}