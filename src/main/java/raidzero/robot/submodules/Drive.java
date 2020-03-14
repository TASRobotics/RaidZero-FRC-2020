package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpiutil.math.MathUtil;

import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.TrajectoryFollower;
import raidzero.robot.utils.EncoderUtils;
import raidzero.robot.wrappers.InactiveDoubleSolenoid;
import raidzero.robot.wrappers.LazyTalonFX;
import raidzero.robot.wrappers.Pigeon;

public class Drive extends Submodule {
    public static class PeriodicIO {
        // Inputs
        public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(0.0, 0.0);

        public double leftPosition = 0; // in encoder ticks
        public double rightPosition = 0; // in encoder ticks

        public double leftVelocity = 0; // in encoder ticks per 100ms
        public double rightVelocity = 0; // in encoder ticks per 100ms

        // Outputs
        // In percent [-1.0, 1.0] if OPEN_LOOP, in encoder ticks / 100ms if
        // in PATH_FOLLOWING
        public double leftDemand = 0.0;
        public double rightDemand = 0.0;

        // In volts
        public double leftFeedforward = 0.0;
        public double rightFeedforward = 0.0;
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
    private LazyTalonFX rightLeader, rightFollower;

    private InactiveDoubleSolenoid gearShiftSolenoid;

    private Pigeon pigeon;

    // Hardware states
    private GearShift currentGearShift = GearShift.LOW;

    // Controllers
    private TrajectoryFollower trajectoryFollower;

    private DifferentialDriveOdometry odometry;

    private double quickStopAccumulator = 0.0;

    private ControlState controlState = ControlState.OPEN_LOOP;

    private PeriodicIO periodicIO = new PeriodicIO();

    // Dashboard entries
    private NetworkTableEntry gearShiftEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Gear Shift", "EMPTY")
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(3, 2)
        .getEntry();
    private NetworkTableEntry leftEncoderEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Left Velocity (mps)", 0.0)
        .withSize(2, 1)
        .withPosition(0, 0)
        .getEntry();
    private NetworkTableEntry rightEncoderEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Right Velocity (mps)", 0.0)
        .withSize(2, 1)
        .withPosition(2, 0)
        .getEntry();
    private NetworkTableEntry leftEncoderTargetEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Left Target Velocity (mps)", 0.0)
        .withSize(2, 1)
        .withPosition(4, 0)
        .getEntry();
    private NetworkTableEntry rightEncoderTargetEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Right Target Velocity (mps)", 0.0)
        .withSize(2, 1)
        .withPosition(6, 0)
        .getEntry();
    private NetworkTableEntry odometryXEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Odometry X (m)", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(0, 2)
        .getEntry();
    private NetworkTableEntry odometryYEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Odometry Y (m)", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(1, 2)
        .getEntry();
    private NetworkTableEntry odometryAngleEntry = Shuffleboard.getTab(Tab.DEBUG)
        .add("Odometry Angle (deg)", 0.0)
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(2, 2)
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
        pigeon = new Pigeon(DriveConstants.PIGEON_ID);
        pigeon.configFactoryDefault();

        // Must be called after the pigeon is initialized
        configureMotorClosedLoop();

        // Everything must be zeroed before constructing odometry
        zero();
        odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(pigeon.getHeading()));

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
     */
    private void configureMotorClosedLoop() {
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        talonConfig.neutralDeadband = DriveConstants.DRIVE_NEUTRAL_DEADBAND;
        talonConfig.slot0.kF = 0.0;
        talonConfig.slot0.kP = DriveConstants.PRIMARY_P;
        talonConfig.slot0.kI = 0.0;
        talonConfig.slot0.kD = 0.0;
        talonConfig.slot0.integralZone = 0;
        talonConfig.slot0.closedLoopPeakOutput = 1.0;

        leftLeader.configAllSettings(talonConfig);
        rightLeader.configAllSettings(talonConfig);

        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                                DriveConstants.PID_PRIMARY_SLOT, 
                                                Constants.TIMEOUT_MS);
        rightLeader.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                                 DriveConstants.PID_PRIMARY_SLOT, 
                                                 Constants.TIMEOUT_MS);

        trajectoryFollower = new TrajectoryFollower(getKinematics());
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

        stop();
        zero();
        resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
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
        double pigeonHeading = pigeon.getHeading();
        Pose2d currentPose = odometry.update(
            Rotation2d.fromDegrees(pigeonHeading),
            EncoderUtils.ticksToMeters(
                periodicIO.leftPosition, 
                currentGearShift
            ),
            EncoderUtils.ticksToMeters(
                -periodicIO.rightPosition, 
                currentGearShift
            )
        );
        if (controlState == ControlState.PATH_FOLLOWING) {
            // Update trajectory follower here
            tankVelocity(trajectoryFollower.update(currentPose));
        }
        gearShiftEntry.setString(currentGearShift.toString());
        leftEncoderEntry.setNumber(
            EncoderUtils.ticksPer100msToMetersPerSec(
                periodicIO.leftVelocity, currentGearShift)
        );
        rightEncoderEntry.setNumber(
            EncoderUtils.ticksPer100msToMetersPerSec(
                -periodicIO.rightVelocity, currentGearShift)
        );
        odometryXEntry.setDouble(currentPose.getTranslation().getX());
        odometryYEntry.setDouble(currentPose.getTranslation().getY());
        odometryAngleEntry.setDouble(currentPose.getRotation().getDegrees());
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
                // Divide by 12 V since its the maximum voltage
                leftLeader.set(ControlMode.Velocity, periodicIO.leftDemand, 
                    DemandType.ArbitraryFeedForward, periodicIO.leftFeedforward / 12);
                rightLeader.set(ControlMode.Velocity, periodicIO.rightDemand, 
                    DemandType.ArbitraryFeedForward, periodicIO.rightFeedforward / 12);
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
        periodicIO.leftFeedforward = 0.0;
        periodicIO.rightFeedforward = 0.0;

        leftLeader.set(ControlMode.Disabled, 0.0);
        rightLeader.set(ControlMode.Disabled, 0.0);
    }

    /**
     * Zeros all encoders & the pigeon to 0.
     */
    @Override
    public void zero() {
        leftLeader.setSelectedSensorPosition(0,
            DriveConstants.PID_PRIMARY_SLOT, Constants.TIMEOUT_MS);
        rightLeader.setSelectedSensorPosition(0,
            DriveConstants.PID_PRIMARY_SLOT, Constants.TIMEOUT_MS);
        pigeon.setFusedHeading(0.0);
    }

    /**
     * Resets odometry pose.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            pose, Rotation2d.fromDegrees(pigeon.getHeading()));
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
     * Tank drive mode for closed-loop velocity control (trajectory).
     * 
     * @param speeds wheel speeds in m/s
     */
    public void tankVelocity(DifferentialDriveWheelSpeeds speeds) {
        tankVelocity(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond);
    }

    /**
     * Tank drive mode for closed-loop velocity control (trajectory).
     * 
     * @param leftVelocity velocity in m/s
     * @param rightVelocity velocity in m/s
     */
    public void tankVelocity(double leftVelocity, double rightVelocity) {
        leftEncoderTargetEntry.setNumber(leftVelocity);
        rightEncoderTargetEntry.setNumber(rightVelocity);

        periodicIO.leftDemand = EncoderUtils.metersPerSecToTicksPer100ms(leftVelocity, 
            currentGearShift);
        periodicIO.rightDemand = EncoderUtils.metersPerSecToTicksPer100ms(rightVelocity, 
            currentGearShift);

        double leftAcceleration = (leftVelocity - periodicIO.wheelSpeeds.leftMetersPerSecond) 
            / DriveConstants.LOOP_PERIOD_SECONDS;
        double rightAcceleration = (rightVelocity - periodicIO.wheelSpeeds.rightMetersPerSecond)
            / DriveConstants.LOOP_PERIOD_SECONDS;

        periodicIO.leftFeedforward = DriveConstants.FEED_FORWARD.calculate(
            leftVelocity, leftAcceleration);
        periodicIO.rightFeedforward = DriveConstants.FEED_FORWARD.calculate(
            rightVelocity, rightAcceleration);

        periodicIO.wheelSpeeds.leftMetersPerSecond = leftVelocity;
        periodicIO.wheelSpeeds.rightMetersPerSecond = rightVelocity;
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
     * Returns the current robot pose through odometry.
     * 
     * @return robot pose in meters & degrees
     */
    public Pose2d getCurrentPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the kinematics object corresponding to this drive.
     * 
     * @return drive kinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return DriveConstants.DRIVE_KINEMATICS;
    }

    /**
     * Makes the drive start following a Path.
     * 
     * @param path           the path to follow
     * @param zeroAllSensors whether to zero all sensors to the first point
     */
    public void setDrivePath(Path path) {
        if (trajectoryFollower != null) {
            // Stops the drive
            stop();

            Trajectory trajectory = path.getTrajectory();

            State initialState = trajectory.sample(0.0);
            periodicIO.wheelSpeeds = getKinematics().toWheelSpeeds(
                new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                    initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));

            // Reset & start trajectory follower
            trajectoryFollower.reset();
            trajectoryFollower.start(trajectory);

            controlState = ControlState.PATH_FOLLOWING;
        }
    }

    /**
     * Returns whether the drive has finished following a path.
     * 
     * @return if the drive is finished pathing
     */
    public boolean isFinishedWithPath() {
        if (trajectoryFollower == null || controlState != ControlState.PATH_FOLLOWING) {
            return false;
        }
        return trajectoryFollower.isFinished();
    }
}
