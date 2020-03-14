package raidzero.robot.submodules;

import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.ProfileFollower;
import raidzero.robot.util.EncoderUtils;
import raidzero.lib.wrapper.InactiveDoubleSolenoid;
import raidzero.lib.wrapper.LazyTalonFX;

public class Drive extends Submodule {
    public static class PeriodicIO {
        // Inputs
        public double leftPosition = 0; // in encoder ticks
        public double rightPosition = 0; // in encoder ticks

        public double leftVelocity = 0; // in encoder ticks per 100ms
        public double rightVelocity = 0; // in encoder ticks per 100ms

        // Outputs
        public double leftDemand = 0.0; // in percent [-1.0, 1.0]
        public double rightDemand = 0.0; // in percent [-1.0, 1.0]

        public int closedLoopStatus = SetValueMotionProfile.Disable.value;
    }

    public static enum GearShift {
        HIGH, LOW
    }

    public static enum ControlState {
        OPEN_LOOP, PATH_FOLLOWING
    }

    private static Drive instance = null;

    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }
        return instance;
    }

    private Drive() {
    }

    // Hardware components
    private LazyTalonFX leftLeader, leftFollower;
    private LazyTalonFX rightLeader, rightFollower; // right leader is the "ultimate master"

    // Should be references to one of the 4 motors
    private LazyTalonFX profilingLeader;
    private LazyTalonFX profilingFollower;

    private InactiveDoubleSolenoid gearShiftSolenoid;

    private PigeonIMU pigeon;

    // Hardware states
    private GearShift currentGearShift = GearShift.LOW;

    // Control state
    private ControlState controlState = ControlState.OPEN_LOOP;

    // Controllers
    private ProfileFollower mpFollower;

    private double quickStopAccumulator = 0.0;

    private PeriodicIO periodicIO = new PeriodicIO();

    // Dashboard entries
    private NetworkTableEntry gearShiftEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Gear Shift", "EMPTY")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(3, 2)
        .getEntry();
    private NetworkTableEntry leftEncoderEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Left Distance (in)", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(0, 0)
        .getEntry();
    private NetworkTableEntry rightEncoderEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Right Distance (in)", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(1, 0)
        .getEntry();

    @Override
    public void onInit() {
        // Motors
        leftLeader = new LazyTalonFX(DriveConstants.LEFT_LEADER_ID);
        configureMotor(leftLeader, DriveConstants.LEFT_INVERSION);

        leftFollower = new LazyTalonFX(DriveConstants.LEFT_FOLLOWER_ID);
        configureMotor(leftFollower, DriveConstants.LEFT_INVERSION);
        leftFollower.follow(leftLeader);

        rightLeader = new LazyTalonFX(DriveConstants.RIGHT_LEADER_ID);
        configureMotor(rightLeader, DriveConstants.RIGHT_INVERSION);

        rightFollower = new LazyTalonFX(DriveConstants.RIGHT_FOLLOWER_ID);
        configureMotor(rightFollower, DriveConstants.RIGHT_INVERSION);
        rightFollower.follow(rightLeader);

        // Pigeon IMU
        pigeon = new PigeonIMU(DriveConstants.PIGEON_ID);
        pigeon.configFactoryDefault();

        /*
         * Setup the profiling leader & follower
         * 
         * Note: if this changes, the argument to configureMotorClosedLoop must
         * reflect the inversion of the leader motor
         */
        profilingLeader = rightLeader;
        profilingFollower = leftLeader;

        // Must be called after the pigeon is initialized
        configureMotorClosedLoop(DriveConstants.RIGHT_INVERSION);

        // Gear shift
        gearShiftSolenoid = new InactiveDoubleSolenoid(DriveConstants.GEARSHIFT_FORWARD_ID,
                DriveConstants.GEARSHIFT_REVERSE_ID);
    }

    /**
     * Configures a motor controller.
     * 
     * @param motor     the motor controller to configure
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
     * @param leaderInversion whether the profiling leader is inverted
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

        // leaderConfig.openloopRamp = 1.0;
        // followerConfig.openloopRamp = 1.0;

        // Apply all settings
        profilingLeader.configAllSettings(leaderConfig);
        profilingFollower.configAllSettings(followerConfig);

        // Set status frame periods to ensure we don't have stale data
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20,
                Constants.TIMEOUT_MS);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20,
                Constants.TIMEOUT_MS);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20,
                Constants.TIMEOUT_MS);
        profilingLeader.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20,
                Constants.TIMEOUT_MS);
        profilingFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5,
                Constants.TIMEOUT_MS);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 5,
                Constants.TIMEOUT_MS);

        profilingLeader.changeMotionControlFramePeriod(DriveConstants.TRANSMIT_PERIOD_MS);
        profilingLeader.configMotionProfileTrajectoryPeriod(DriveConstants.BASE_TRAJ_PERIOD_MS,
                Constants.TIMEOUT_MS);

        mpFollower = new ProfileFollower(profilingLeader);
    }

    /**
     * Determines if SensorSum or SensorDiff should be used for combining
     * left/right sensors into Robot Distance.
     * 
     * Assumes Aux Position is set as Remote Sensor 0.
     * 
     * configAllSettings must still be called on the master config after this
     * function modifies the config values.
     * 
     * Source: https://shorturl.at/oqvCN (CTRE GitHub)
     * 
     * @param masterInvertType Invert of the Master Talon
     * @param masterConfig     Configuration object to fill
     */
    private void setRobotDistanceConfigs(InvertType masterInvertType,
            TalonFXConfiguration masterConfig) {
        if (masterInvertType == InvertType.InvertMotorOutput) {
            masterConfig.diff0Term = FeedbackDevice.IntegratedSensor; // Local Integrated Sensor
            masterConfig.diff1Term = FeedbackDevice.RemoteSensor0; // Aux Selected Sensor
            masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorDifference;
        } else {
            masterConfig.sum0Term = FeedbackDevice.RemoteSensor0; // Aux Selected Sensor
            masterConfig.sum1Term = FeedbackDevice.IntegratedSensor; // Local IntegratedSensor
            masterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum;
        }
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

        periodicIO = new PeriodicIO();

        setGearShift(GearShift.LOW);

        profilingLeader.clearMotionProfileTrajectories();
    }

    @Override
    public void readPeriodicInputs() {
        TalonFXSensorCollection left = leftLeader.getSensorCollection();
        TalonFXSensorCollection right = rightLeader.getSensorCollection();

        periodicIO.leftPosition = left.getIntegratedSensorPosition();
        periodicIO.rightPosition = right.getIntegratedSensorPosition();

        periodicIO.leftVelocity = left.getIntegratedSensorVelocity();
        periodicIO.rightVelocity = right.getIntegratedSensorVelocity();
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
            periodicIO.closedLoopStatus = mpFollower.getOutput();
        }
        gearShiftEntry.setString(currentGearShift.toString());
        leftEncoderEntry.setNumber(
            EncoderUtils.ticksToInches(
                periodicIO.leftPosition, currentGearShift)
        );
        rightEncoderEntry.setNumber(
            EncoderUtils.ticksToInches(
                -periodicIO.rightPosition, currentGearShift)
        );
    }

    /**
     * Runs the motors with different control modes depending on the state.
     */
    @Override
    public void writePeriodicOutputs() {
        switch (controlState) {
            case OPEN_LOOP:
                leftLeader.set(ControlMode.PercentOutput, periodicIO.leftDemand);
                rightLeader.set(ControlMode.PercentOutput, periodicIO.rightDemand);
                break;
            case PATH_FOLLOWING:
                profilingLeader.set(ControlMode.MotionProfileArc, periodicIO.closedLoopStatus);
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

        periodicIO.leftDemand = 0.0;
        periodicIO.rightDemand = 0.0;
        periodicIO.closedLoopStatus = SetValueMotionProfile.Disable.value;

        leftLeader.set(ControlMode.Disabled, 0.0);
        rightLeader.set(ControlMode.Disabled, 0.0);
    }

    /**
     * Zeros all encoders & the pigeon to 0.
     */
    @Override
    public void zero() {
        /**
         * setSelectedSensorPosition would set the sensor sum for the PID, which is
         * not what we want to do.
         */
        leftLeader.getSensorCollection().setIntegratedSensorPosition(0.0, Constants.TIMEOUT_MS);
        rightLeader.getSensorCollection().setIntegratedSensorPosition(0.0, Constants.TIMEOUT_MS);
        pigeon.setFusedHeading(0.0);
    }

    /**
     * Tank drive mode for open-loop control.
     * 
     * @param left    left percent output in [-1, 1]
     * @param right   right percent output in [-1, 1]
     * @param reverse whether to reverse the inputs or not
     */
    public void tank(double left, double right, boolean reverse) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        if (reverse) {
            periodicIO.leftDemand = -left;
            periodicIO.rightDemand = -right;
            return;
        }
        periodicIO.leftDemand = left;
        periodicIO.rightDemand = right;
    }

    /**
     * Arcade drive mode for open-loop control.
     * 
     * @param leftJoystick  value of the left joystick in [-1, 1]
     * @param rightJoystick value of the right joystick in [-1, 1]
     * @param reverse       whether to reverse the inputs or not
     */
    public void arcade(double leftJoystick, double rightJoystick, boolean reverse) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }
        if (reverse) {
            leftJoystick *= -1;
            rightJoystick *= -1;
        }
        periodicIO.leftDemand = leftJoystick + rightJoystick;
        periodicIO.rightDemand = leftJoystick - rightJoystick;
    }

    /**
     * Curvature drive mode for open-loop control.
     * 
     * @param xSpeed      The robot's speed along the X axis [-1.0, 1.0].
     * @param zRotation   The robot's rotation rate around the Z axis [-1.0,
     *                    1.0]. Clockwise is positive.
     * @param isQuickTurn If set, overrides constant-curvature turning for
     *                    turn-in-place maneuvers.
     */
    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
        if (controlState != ControlState.OPEN_LOOP) {
            controlState = ControlState.OPEN_LOOP;
        }

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < DriveConstants.QUICK_STOP_THRESHOLD) {
                quickStopAccumulator = (1 - DriveConstants.QUICK_STOP_ALPHA) * quickStopAccumulator
                        + DriveConstants.QUICK_STOP_ALPHA * MathUtil.clamp(zRotation, -1.0, 1.0)
                                * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - quickStopAccumulator;

            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // Reduce both outputs to acceptable range if rotation is overpowered
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }
        periodicIO.leftDemand = leftMotorOutput;
        periodicIO.rightDemand = rightMotorOutput;
    }

    /**
     * Sets the brake mode to brake or coast.
     * 
     * @param brake whether to brake or not
     */
    public void setBrakeMode(boolean brake) {
        if (brake) {
            rightLeader.setNeutralMode(NeutralMode.Brake);
            leftLeader.setNeutralMode(NeutralMode.Brake);
        } else {
            rightLeader.setNeutralMode(NeutralMode.Coast);
            leftLeader.setNeutralMode(NeutralMode.Coast);
        }
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
     * @param path           the path to follow
     * @param zeroAllSensors whether to zero all sensors to the first point
     */
    public void setDrivePath(Path path, boolean zeroAllSensors) {
        if (mpFollower != null) {
            // Stop the drivetrain first
            stop();

            if (zeroAllSensors) {
                zero();

                double angle = path.getFirstPoint().angle.orElse(0.0);

                // The path may start at a different angle at times
                pigeon.setFusedHeading(angle);
            } else if (path.isReversed()) {
                // TODO: Check to see if this is correct
                pigeon.setFusedHeading(pigeon.getFusedHeading() + 180);
            }

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
