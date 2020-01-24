package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Turret;
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
    //private Turret turret = Turret.getInstance();
    //private Shooter shooter = Shooter.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Intake intake = Intake.getInstance();

    private XboxController controller = new XboxController(0);

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        drive.stop();
        drive.setGearShift(GearShift.LOW);
        //turret.stop();
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
        if (controller.getBumperPressed(Hand.kRight)) {
            drive.setGearShift(GearShift.HIGH);
        } else if (controller.getBumperReleased(Hand.kRight)) {
            drive.setGearShift(GearShift.LOW);
        }
        if (controller.getAButtonPressed()) {
            drive.zero();
        }
        /**
         * Turret
         */
        /*if (controller.getXButton()) {
            turret.turn(0.75);
        } else if (controller.getBButton()) {
            turret.turn(-0.75);
        }*/
        /**
         * Shooter
         */
        //shooter.spin(controller.getTriggerAxis(Hand.kRight));
        /**
         * Hopper
         */
        hopper.move(controller.getTriggerAxis(Hand.kLeft));
        /**
         * Intake
         */
        if (controller.getYButton()) {
            intake.intakeBalls(1.0);
        } else if (controller.getAButton()) {
            //intake.intakeBalls(-1.0);
            intake.intakeBalls(0.0);
        } else {
            intake.intakeBalls(0.0);
        }
    }
}