package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.EjectsBalls;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.SucksBalls;
import raidzero.robot.submodules.Drive.GearShift;

public class Teleop {

    private static Teleop instance = null;
    private Teleop() {}
    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    /**
     * Local Variables
     */
    private Drive drive = Drive.getInstance();
    private EjectsBalls shitter = EjectsBalls.getInstance();
    private SucksBalls sucky = SucksBalls.getInstance();


    private XboxController p1 = new XboxController(0);
    private XboxController p2 = new XboxController(1);


    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        drive.setGearShift(GearShift.LOW);
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        /**
         * E stop that we have control over
         */
        if(p1.getBackButtonPressed()){
            while(!p1.getStartButtonPressed()){
                SubmoduleManager.getInstance().stop();
            }
        }
        /**
         * Drivetrain
         */
        drive.tank(-p1.getY(Hand.kLeft), -p1.getY(Hand.kRight));
        if (p1.getBumper(Hand.kRight)) {
            drive.setGearShift(GearShift.HIGH);
        } else if (p1.getBumperReleased(Hand.kRight)) {
            drive.setGearShift(GearShift.LOW);
        }
        //drive.arcade(-p1.getY(Hand.kLeft), p1.getX(Hand.kRight));

        /**
         * Ejecter
         */
        shitter.shoot(p2.getY(Hand.kLeft), p2.getAButtonPressed());

        /**
         * Sucker
         */
        sucky.suck(p1.getTriggerAxis(Hand.kRight) - p1.getTriggerAxis(Hand.kLeft));
    }
}