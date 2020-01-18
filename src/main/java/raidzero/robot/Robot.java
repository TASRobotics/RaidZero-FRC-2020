package raidzero.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.auto.AutoRunner;
import raidzero.robot.auto.sequences.TestSequence;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.teleop.Teleop;
import raidzero.robot.submodules.SubmoduleManager;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    private static final SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private AutoRunner autoRunner = new AutoRunner();
    private static final Teleop teleop = Teleop.getInstance();

    private static final Drive moduleDrive = Drive.getInstance();
    private static final Limelight moduleLimelight = Limelight.getInstance();

    @Override
    public void robotInit() {
        SubmoduleManager.getInstance().setSubmodules(
            moduleDrive,
            moduleLimelight
        );
    }

    @Override
    public void disabledInit() {
        autoRunner.stop();
        submoduleManager.onStop(Timer.getFPGATimestamp());
    }

    @Override
    public void autonomousInit() {
        submoduleManager.onStart(Timer.getFPGATimestamp());

        autoRunner.selectSequence(new TestSequence());
        autoRunner.start();
    }

    @Override
    public void autonomousPeriodic() {
        double timestamp = Timer.getFPGATimestamp();
        autoRunner.onLoop(timestamp);
        submoduleManager.onLoop(timestamp);
    }

    @Override
    public void teleopInit() {
        autoRunner.stop();

        teleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    @Override
    public void teleopPeriodic() {
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }

}
