package raidzero.robot;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidzero.lib.music.MusicPlayer;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.HopperConstants;
import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.auto.AutoRunner;
import raidzero.robot.logging.Logger;
import raidzero.robot.teleop.Teleop;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.submodules.CameraManager;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.WheelOfFortune;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Intake;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    private static final boolean ONLY_PLAY_MUSIC = true;

    private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private static final Teleop teleop = Teleop.getInstance();

    private static final Drive moduleDrive = Drive.getInstance();
    private static final Limelight moduleLimelight = Limelight.getInstance();
    private static final Shooter moduleShooter = Shooter.getInstance();
    private static final AdjustableHood moduleAdjustableHood = AdjustableHood.getInstance();
    private static final Intake moduleIntake = Intake.getInstance();
    private static final Hopper moduleHopper = Hopper.getInstance();
    private static final Turret moduleTurret = Turret.getInstance();
    private static final WheelOfFortune moduleWheelOfFortune = WheelOfFortune.getInstance();
    private static final Climb moduleClimb = Climb.getInstance();
    private static final Superstructure moduleSuperstructure = Superstructure.getInstance();
    private static final CameraManager moduleCameraManager = CameraManager.getInstance();

    private AutoRunner autoRunner;

    private MusicPlayer player;

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        if (ONLY_PLAY_MUSIC) {
            player = new MusicPlayer(Arrays.asList(
                new TalonFX(DriveConstants.LEFT_LEADER_ID),
                new TalonFX(DriveConstants.LEFT_FOLLOWER_ID),
                new TalonFX(DriveConstants.RIGHT_LEADER_ID),
                new TalonFX(DriveConstants.RIGHT_FOLLOWER_ID),
                new TalonFX(HopperConstants.MOTOR_ID),
                new TalonFX(ShooterConstants.MOTOR_ID)
            ));
            return;
        }
        Logger.startInitialization();

        // Register all submodules here
        submoduleManager.setSubmodules(
            moduleDrive,
            moduleShooter,
            moduleAdjustableHood,
            moduleIntake,
            moduleHopper,
            moduleTurret,
            moduleWheelOfFortune,
            moduleClimb,
            moduleLimelight,
            moduleSuperstructure,
            moduleCameraManager
        );
        submoduleManager.onInit();

        autoRunner = new AutoRunner();

        Logger.finishInitialization();
    }

    @Override
    public void robotPeriodic() {
        if (ONLY_PLAY_MUSIC) {
            return;
        }
        Logger.update();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        if (ONLY_PLAY_MUSIC) {
            return;
        }
        // Stop autonomous
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());

        moduleDrive.setBrakeMode(false);
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {
        if (ONLY_PLAY_MUSIC) {
            return;
        }
        submoduleManager.onStart(Timer.getFPGATimestamp());

        autoRunner.readSendableSequence();
        autoRunner.start();
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        if (ONLY_PLAY_MUSIC) {
            return;
        }
        double timestamp = Timer.getFPGATimestamp();
        autoRunner.onLoop(timestamp);
        submoduleManager.onLoop(timestamp);
    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
        if (ONLY_PLAY_MUSIC) {
            return;
        }
        // Stop the autonomous
        autoRunner.stop();

        // Start the teleop handler
        teleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    /**
     * Runs every 0.02s during teleop (50 Hz).
     */
    @Override
    public void teleopPeriodic() {
        if (ONLY_PLAY_MUSIC) {
            player.update();
            return;
        }
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }
}
