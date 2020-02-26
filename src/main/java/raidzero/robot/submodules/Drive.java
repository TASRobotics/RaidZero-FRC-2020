
package raidzero.robot.submodules;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import raidzero.robot.Constants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.wrappers.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.TrajectoryFollower;
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
    private LazyTalonFX rightLeader;
    private LazyTalonFX rightFollower;

    private InactiveDoubleSolenoid gearShiftSolenoid;

    private Pigeon pigeon;

    // Control states
    private ControlState controlState;

    // Trajectory follower
    private TrajectoryFollower trajectoryFollower;

    private DifferentialDriveOdometry odometry;

    private GearShift currentGearShift;

    // Output
    private double outputLeftDrive = 0.0;
    private double outputRightDrive = 0.0;

    private double outputLeftVelocity = 0.0; // in sensor units / 100 ms
    private double outputLeftFeedforward = 0.0; // in volts
    private double outputRightVelocity = 0.0; // in sensor units / 100 ms
    private double outputRightFeedforward = 0.0; // in volts

    private Drive() {}

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

        // Control state
        setOpenLoop();
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
        stop();
        zero();
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
                leftLeader.getSensorCollection().getIntegratedSensorPosition(), 
                currentGearShift
            ),
            EncoderUtils.ticksToMeters(
                -rightLeader.getSensorCollection().getIntegratedSensorPosition(), 
                currentGearShift
            )
        );
        if (controlState == ControlState.PATH_FOLLOWING) {
            // Update trajectory follower here
            tankVelocity(trajectoryFollower.update(currentPose));
        }
        SmartDashboard.putString("Current Pose", currentPose.toString());
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
                // Divide by 12 V since its the maximum voltage
                leftLeader.set(ControlMode.Velocity, outputLeftVelocity, 
                    DemandType.ArbitraryFeedForward, outputLeftFeedforward / 12);
                rightLeader.set(ControlMode.Velocity, outputRightVelocity, 
                    DemandType.ArbitraryFeedForward, outputRightFeedforward / 12);
                break;
        }
    }

    /**
     * Stops all the motors. Also changes the control state to open-loop.
     */
    @Override
    public void stop() {
        setOpenLoop();

        outputLeftDrive = 0.0;
        outputRightDrive = 0.0;

        outputLeftVelocity = 0.0;
        outputLeftFeedforward = 0.0;
        outputRightVelocity = 0.0;
        outputRightFeedforward = 0.0;

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
        pigeon.setYaw(0.0);
    }

    /**
     * Resets odometry pose.
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
            pose, Rotation2d.fromDegrees(pigeon.getHeading()));
    }

    /**
     * Switches the drive to open-loop control mode.
     */
    public void setOpenLoop() {
        controlState = ControlState.OPEN_LOOP;
    }

    /**
     * Tank drive mode for open-loop control.
     * 
     * @param left left percent output in [-1, 1]
     * @param right right percent output in [-1, 1]
     * @param reverse whether to reverse the inputs or not
     */
    public void tank(double left, double right, boolean reverse) {
        if (reverse) {
            outputLeftDrive = -left;
            outputRightDrive = -right;
            return;
        }
        outputLeftDrive = left;
        outputRightDrive = right;
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
        outputLeftVelocity = EncoderUtils.metersPerSecToTicksPer100ms(leftVelocity, 
            currentGearShift);
        outputRightVelocity = EncoderUtils.metersPerSecToTicksPer100ms(rightVelocity, 
            currentGearShift);

        double leftCurrentVelocity = EncoderUtils.ticksToMeters(
            leftLeader.getSelectedSensorVelocity(), currentGearShift);
        double leftAcceleration = (leftVelocity - leftCurrentVelocity) 
            / DriveConstants.LOOP_PERIOD_SECONDS;
        double rightCurrentVelocity = EncoderUtils.ticksToMeters(
            rightLeader.getSelectedSensorVelocity(), currentGearShift);
        double rightAcceleration = (rightVelocity - rightCurrentVelocity)
            / DriveConstants.LOOP_PERIOD_SECONDS;

        SmartDashboard.putNumber("left_measure", leftCurrentVelocity);
        SmartDashboard.putNumber("left_ref", leftVelocity);
        SmartDashboard.putNumber("right_measure", rightCurrentVelocity);
        SmartDashboard.putNumber("right_ref", rightVelocity);
        
        outputLeftFeedforward = DriveConstants.FEED_FORWARD.calculate(
            leftVelocity, leftAcceleration);
        outputRightFeedforward = DriveConstants.FEED_FORWARD.calculate(
            rightVelocity, rightAcceleration);
        SmartDashboard.putNumber("left_FF", outputLeftFeedforward);
        SmartDashboard.putNumber("right_FF", outputRightFeedforward);
    }

    /**
     * Arcade drive mode for open-loop control.
     * 
     * @param leftJoystick value of the left joystick in [-1, 1]
     * @param rightJoystick value of the right joystick in [-1, 1]
     */
    public void arcade(double leftJoystick, double rightJoystick) {
        outputLeftDrive = leftJoystick + rightJoystick;
        outputRightDrive = leftJoystick - rightJoystick;
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
     * @param path the path to follow
     */
    public void setDrivePath(Path path) {
        if (trajectoryFollower != null) {
            // Stops the drive
            stop();

            Trajectory trajectory = path.getTrajectory();

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