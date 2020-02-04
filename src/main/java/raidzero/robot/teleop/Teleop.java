package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.Drive.GearShift;
import raidzero.robot.wrappers.InactiveCompressor;

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
    private Shooter shitter = Shooter.getInstance();
    private Intake sucky = Intake.getInstance();
    private Hopper move = Hopper.getInstance();
    private Turret turret = Turret.getInstance();
    private Climb climb = Climb.getInstance();
    private static InactiveCompressor compressor = new InactiveCompressor();

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
         * Overview
         * 
         * P1
         * BackButton: Estop
         * StartButton: Enables climb || Exits Estop
         * Joysticks: Drive
         * Bumpers: Shift [Right(while held down): high gear]
         * Triggers: Intake [Right: In, Left: Out]
         * DPad: Hopper [Up: In, Down: out]
         * B buttom: compressor on/off
         *  
         * P2
         * Left joystick: Shooter
         * A button: Maintain Previous shooter speed
         * Right joystick: Hopper [Up: In, Down: Out]
         * Triggers: Turret [Right: Clockwise, Left: Counterclockwise]
         * Left bumper: Use left joystick to aim
         */

        /**
         * Compressor
         */
        if(p1.getBButtonPressed()) {
            compressor.changeState();
        }        

        /**
         * E stop that we have control over
         */
        if(p1.getBackButtonPressed()){
            while(!p1.getStartButtonPressed()){
                SubmoduleManager.getInstance().stop();
                System.out.println
                    ("ESTOPPED: press the 'start' button on controller 1 to re-enable");
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
        shitter.shoot(p2.getY(Hand.kLeft), p2.getAButton());

        /**
         * Sucker
         */
        sucky.suck(p1.getTriggerAxis(Hand.kRight) - p1.getTriggerAxis(Hand.kLeft));
        if(p1.getBumperPressed(Hand.kLeft)) {
            sucky.invertStraw();
        }

        /**
         * Hopper
         */
        move.moveBalls(p1.getPOV(), p2.getY(Hand.kRight));

        /**
         * Turret
         */
        turret.rotateManual(p2.getTriggerAxis(Hand.kRight) - p2.getTriggerAxis(Hand.kLeft));

        /**
         * Climb
         */

        // Enable climb
        if(p1.getStartButton()) {
            if(p2.getStartButtonPressed()) {
                climb.unlock();
            }
        }

        // Climb Code
        if(p2.getBButton()) {
            climb.climb(1);
        } else if(p2.getXButton()) {
            climb.climb(-1);
        } else {
            climb.climb(0);
        }
    }
}