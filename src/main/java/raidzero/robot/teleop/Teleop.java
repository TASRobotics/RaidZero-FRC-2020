package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.TurretConstants;
import raidzero.robot.auto.actions.DebugLimelightDistance;
import raidzero.robot.auto.actions.TurnToGoal;
import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.SubmoduleManager;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.WheelOfFortune;
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
    private WheelOfFortune wheelOfFortune = WheelOfFortune.getInstance();
    private AdjustableHood hood = AdjustableHood.getInstance();
    private InactiveCompressor compressor = InactiveCompressor.getInstance();

    private XboxController p1 = new XboxController(0);
    private XboxController p2 = new XboxController(1);

    private DebugLimelightDistance debugDistance = new DebugLimelightDistance();

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        drive.stop();
        drive.setGearShift(GearShift.LOW);

        debugDistance.start();
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
         * B button: compressor on/off
         *  
         * P2
         * Left joystick: Shooter
         * A button: Maintain Previous shooter speed
         * Right joystick: Hopper [Up: In, Down: Out]
         * Triggers: Turret [Right: Clockwise, Left: Counterclockwise]
         * Left bumper: Use left joystick to aim
         * Right bumper: Hold to use P2 right joystick to turn wheel of fortune
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
        drive.tank(
            JoystickUtils.monomialScale(
                JoystickUtils.deadband(-p1.getY(Hand.kLeft)),
                DriveConstants.JOYSTICK_EXPONENT, 
                DriveConstants.JOYSTICK_COEFFICIENT), 
            JoystickUtils.monomialScale(
                JoystickUtils.deadband(-p1.getY(Hand.kRight)),
                DriveConstants.JOYSTICK_EXPONENT,
                DriveConstants.JOYSTICK_COEFFICIENT)
        );
        /*drive.arcade(
            JoystickUtils.deadband(-p1.getY(Hand.kLeft)), 
            JoystickUtils.deadband(p1.getX(Hand.kRight))
        );*/
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
                IntakeConstants.CONTROL_SCALING_FACTOR * 
                    (p1.getTriggerAxis(Hand.kRight) - p1.getTriggerAxis(Hand.kLeft)))
        );
        if (p1.getBumperPressed(Hand.kLeft)) {
            intake.invertStraw();
        }

        /**
         * Hopper
         */
        int p1Pov = p1.getPOV();
        if (p1Pov == -1 && !p2.getBumper(Hand.kRight)) {
            hopper.moveBelt(JoystickUtils.deadband(p2.getY(Hand.kRight)));
        } else if (p1Pov >= 315 || p1Pov <= 45) {
            hopper.moveBelt(1.0);
        } else if (p1Pov >= 225 && p1Pov <= 135) {
            hopper.moveBelt(-1.0);
        } else {
            hopper.stop();
        }

        /**
         * Turret
         */
        turret.rotateManual(
            TurretConstants.CONTROL_SCALING_FACTOR * 
                (p2.getTriggerAxis(Hand.kRight) - p2.getTriggerAxis(Hand.kLeft))
        );
        //turnToGoal.update();

        /**
         * Wheel of Fortune
         */
        if (p1.getYButtonPressed()) {
            wheelOfFortune.engage(true);
        } else if (p1.getAButtonPressed()) {
            wheelOfFortune.engage(false);
        }
        if (p1.getXButton()) {
            wheelOfFortune.spin(1.0);
        } else if (p1.getBButton()) { 
            wheelOfFortune.spin(-1.0);
        } else {
            wheelOfFortune.stop();
        }

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

        /**
         * Hood
         */
        int p2Pov = p2.getPOV();
        if (p2Pov == 0) {
            hood.adjust(1.0);
        } else if (p2Pov == 180) {
            hood.adjust(-1.0);
        } else {
            hood.stop();
        }

        debugDistance.update();
    }
}