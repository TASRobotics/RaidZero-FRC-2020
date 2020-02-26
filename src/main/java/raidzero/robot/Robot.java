package raidzero.robot;

import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.auto.AutoRunner;
import raidzero.robot.teleop.Teleop;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.AdjustableHood;
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

    private AutoRunner autoRunner;

    /**
     * Runs only once at the start of robot code execution.
     */
    @Override
    public void robotInit() {
        // Enable Shuffleboard logging
        Logger.configureLoggingAndConfig(this, false);

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
            moduleSuperstructure
        );
        submoduleManager.onInit();

        autoRunner = new AutoRunner();
    }

    /**
     * Runs every 0.02s regardless of the competition mode (50 Hz).
     */
    @Override
    public void robotPeriodic() {
        // Send the logger entries
        Logger.updateEntries();
    }

    /**
     * Runs every time the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // Stop autonomous
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    /**
     * Runs at the start of autonomous.
     */
    @Override
    public void autonomousInit() {
        submoduleManager.onStart(Timer.getFPGATimestamp());

        autoRunner.readSendableSequence();
        autoRunner.start();
    }

    /**
     * Runs every 0.02s during autonomous (50 Hz).
     */
    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        autoRunner.onLoop(timestamp);
        submoduleManager.onLoop(timestamp);
    }

    /**
     * Runs at the start of teleop.
     */
    @Override
    public void teleopInit() {
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
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }

}
