package raidzero.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.WheelOfFortune;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Submodule;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.Intake;
import raidzero.robot.teleop.Teleop;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {
    private SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private Teleop teleop = Teleop.getInstance();

    private Drive drive = Drive.getInstance();
    private Shooter shooter = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Turret turret = Turret.getInstance();
    private WheelOfFortune wheelOfFortune = WheelOfFortune.getInstance();
    private Climb climb = Climb.getInstance();

    private Submodule[] modules = {
        drive,
        shooter,
        intake,
        hopper,
        turret,
        //wheelOfFortune
        climb
    };

    @Override
    public void robotInit() {
        SubmoduleManager.getInstance().setSubmodules(modules);
        SubmoduleManager.getInstance().init();
    }

    @Override
    public void disabledInit() {
        submoduleManager.stop();
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
