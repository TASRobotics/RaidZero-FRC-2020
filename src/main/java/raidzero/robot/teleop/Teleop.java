package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Drive.GearShift;

public class Teleop {

    private static Teleop instance = null;

    private Drive drive = Drive.getInstance();
    private XboxController controller = new XboxController(0);

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private Teleop() {}

    public void onStart() {
        
    }

    public void onLoop() {
        drive.tank(-controller.getY(Hand.kLeft), -controller.getY(Hand.kRight));
        if (controller.getBumper(Hand.kLeft)) {
            drive.setGearShift(GearShift.HIGH);
        } else if (controller.getBumper(Hand.kRight)) {
            drive.setGearShift(GearShift.LOW);
        }
        //drive.arcade(-controller.getY(Hand.kLeft), controller.getX(Hand.kRight));
    }
}