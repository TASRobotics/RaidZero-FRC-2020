package raidzero.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.EjectsBalls;
import raidzero.robot.submodules.FondlesBalls;
import raidzero.robot.submodules.MovesBalls;
import raidzero.robot.submodules.Submodule;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.SucksBalls;
import raidzero.robot.teleop.Teleop;

/**
 * The main robot class.
 */
public class Robot extends TimedRobot {

    private SubmoduleManager submoduleManager = SubmoduleManager.getInstance();

    private Teleop teleop = Teleop.getInstance();

    private Drive moduleDrive = Drive.getInstance();
    private EjectsBalls shitter = EjectsBalls.getInstance();
    private SucksBalls sucky = SucksBalls.getInstance();
    private MovesBalls hopper = MovesBalls.getInstance();
    private FondlesBalls fondler = FondlesBalls.getInstance();

    private ArrayList<Submodule> modules = new ArrayList<Submodule>(Arrays.asList(
        moduleDrive,
        shitter,
        sucky,
        hopper,
        fondler));

    @Override
    public void robotInit() {
        for(Submodule module : modules) {
            if(module.isEnabled) {
                continue;
            }
            modules.remove(modules.indexOf(module));
        }

        
        SubmoduleManager.getInstance().setSubmodules((Submodule[]) modules.toArray());
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
