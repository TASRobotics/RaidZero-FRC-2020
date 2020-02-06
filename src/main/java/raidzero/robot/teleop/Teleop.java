package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.TurnToGoal;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.Drive.GearShift;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.InactiveCompressor;

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
    private Shooter shooter = Shooter.getInstance();
    private Intake intake = Intake.getInstance();
    private Hopper hopper = Hopper.getInstance();
    private Turret turret = Turret.getInstance();
    private Climb climb = Climb.getInstance();
    private InactiveCompressor compressor = InactiveCompressor.getInstance();

    private XboxController p1 = new XboxController(0);
    private XboxController p2 = new XboxController(1);

    private TurnToGoal turnToGoal = new TurnToGoal();

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
        if (p1.getBButtonPressed()) {
            compressor.changeState();
        }        

        /**
         * E stop that we have control over
         */
        if (p1.getBackButtonPressed()){
            while (!p1.getStartButtonPressed()){
                SubmoduleManager.getInstance().onStop(Timer.getFPGATimestamp());
                System.out.println("ESTOPPED: press the 'start' button on controller 1 to re-enable");
            }
        }
        /**
         * Drivetrain
         */
        /*drive.tank(
            JoystickUtils.monomialScale(
                JoystickUtils.deadband(-p1.getY(Hand.kLeft)),
                DriveConstants.joystickExponent, 
                DriveConstants.joystickCoefficient), 
            JoystickUtils.monomialScale(
                JoystickUtils.deadband(-p1.getY(Hand.kRight)),
                DriveConstants.joystickExponent,
                DriveConstants.joystickCoefficient)
        );*/
        drive.arcade(
            JoystickUtils.deadband(-p1.getY(Hand.kLeft)), 
            JoystickUtils.deadband(p1.getX(Hand.kRight))
        );
        if (p1.getBumper(Hand.kRight)) {
            drive.setGearShift(GearShift.HIGH);
        } else if (p1.getBumperReleased(Hand.kRight)) {
            drive.setGearShift(GearShift.LOW);
        }

        /**
         * Shooter
         */
        shooter.shoot(JoystickUtils.deadband(p2.getY(Hand.kLeft)), p2.getAButton());

        /**
         * Intake
         */
        intake.intakeBalls(
            JoystickUtils.deadband(
                0.625 * (p1.getTriggerAxis(Hand.kRight) - p1.getTriggerAxis(Hand.kLeft)))
        );
        if (p1.getBumperPressed(Hand.kLeft)) {
            intake.invertStraw();
        }

        /**
         * Hopper
         */
        int p1Pov = p1.getPOV();
        if (p1Pov == -1) {
            hopper.moveBelt(JoystickUtils.deadband(p2.getY(Hand.kRight)));
        } else if (p1Pov >= 315 || p1Pov <= 45) {
            hopper.moveBelt(1.0);
        } else if (p1Pov >= 225 && p1Pov <= 135) {
            hopper.moveBelt(-1.0);
        }

        /**
         * Turret
         */
        turret.rotateManual(
            0.25 * (p2.getTriggerAxis(Hand.kRight) - p2.getTriggerAxis(Hand.kLeft))
        );
        //turnToGoal.update();

        /**
         * Climb
         */
        // Enable climb
        if (p1.getStartButton()) {
            if (p2.getStartButtonPressed()) {
                climb.unlock();
            }
        }

        // Climb Code
        if (p2.getBButton()) {
            climb.climb(1.0);
        } else if (p2.getXButton()) {
            climb.climb(-1.0);
        } else {
            climb.climb(0.0);
        }
    }
}