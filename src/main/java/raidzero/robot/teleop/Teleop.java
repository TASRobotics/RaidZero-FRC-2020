package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Drive.GearShift;

public class Teleop {

    private static Teleop instance = null;
    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }
    private Teleop() {}

    private Drive drive = Drive.getInstance();

    private XboxController controller = new XboxController(0);

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        drive.stop();
        drive.setGearShift(GearShift.LOW);
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        /**
         * Drivetrain
         */
        //drive.tank(-controller.getY(Hand.kLeft), -controller.getY(Hand.kRight));
        drive.arcade(-controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
        //drive.stop();
        if (controller.getBumperPressed(Hand.kRight)) {
            drive.setGearShift(GearShift.HIGH);
        } else if (controller.getBumperReleased(Hand.kRight)) {
            drive.setGearShift(GearShift.LOW);
        }
        if (controller.getAButtonPressed()) {
            drive.zero();
        }
    }
}