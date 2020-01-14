package raidzero.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Drive;
import raidzero.robot.teleop.Teleop;
import raidzero.robot.submodules.SubmoduleManager;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    private SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private Teleop teleop = Teleop.getInstance();

    private Drive moduleDrive = Drive.getInstance();

    @Override
    public void robotInit() {
        SubmoduleManager.getInstance().setSubmodules(
            moduleDrive
        );
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void autonomousInit() {
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    @Override
    public void autonomousPeriodic() {
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }

    @Override
    public void teleopInit() {
        teleop.onStart();
        submoduleManager.onStart(Timer.getFPGATimestamp());
    }

    @Override
    public void teleopPeriodic() {
        teleop.onLoop();
        submoduleManager.onLoop(Timer.getFPGATimestamp());
    }

}
