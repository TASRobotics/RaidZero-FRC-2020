package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.EjectsBalls;
import raidzero.robot.submodules.FondlesBalls;
import raidzero.robot.submodules.MovesBalls;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.SucksBalls;
import raidzero.robot.submodules.Drive.GearShift;

public class Teleop {
    private static boolean hi = true;

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
    private MovesBalls move = MovesBalls.getInstance();
    private FondlesBalls ballFondlers = FondlesBalls.getInstance();
    private static Compressor compressor = new Compressor();

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
         *  
         * P2
         * Left joystick: Shooter
         * A button: Maintain Previous shooter speed
         * Right joystick: Hopper [Up: In, Down: Out]
         * Triggers: Turret [Right: Clockwise, Left: Counterclockwise]
         * Left bumper: Use left joystick to aim
         */

        if(p1.getBButtonPressed()) {
            if(hi) {
                compressor.stop();
                hi = false;
            }
            else {
                hi = true;
                compressor.start();
            }
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
        shitter.shoot(p2.getY(Hand.kLeft), p2.getAButtonPressed());

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
        move.moveMyBalls(p1.getPOV(), p2.getY(Hand.kRight));

        /**
         * Turret
         */
        ballFondlers.fondleThemHard(p2.getTriggerAxis(Hand.kRight) - p2.getTriggerAxis(Hand.kLeft));
    }
}