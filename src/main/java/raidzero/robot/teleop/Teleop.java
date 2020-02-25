package raidzero.robot.teleop;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.TurretConstants;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.DebugLimelightDistance;
import raidzero.robot.auto.actions.TurnToGoal;
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
import raidzero.robot.utils.ShooterUtils;
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
    private InactiveCompressor compressor = InactiveCompressor.getInstance();

    private XboxController p1 = new XboxController(0);
    private XboxController p2 = new XboxController(1);

    private DebugLimelightDistance debugDistance = new DebugLimelightDistance();

    private static boolean reverse = false;

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
        //both

        /**
         * Climb
         */
        //unlock
        if(p1.getStartButton() && p2.getStartButton()) {
            climb.unlock();
        }

        //p1
        p1Loop();

        //p2
        p2Loop();
    
        debugDistance.update();
    }

    private void p1Loop() {

        //
        // REGARDLESS OF HYPERSHIFT
        //

        /**
        * Drivetrain
        */
        //reversing analogue to digital
        if(JoystickUtils.deadband(p1.getTriggerAxis(Hand.kRight)) != 0) {
            reverse = true;
        } else {
            reverse = false;
        }

        drive.tank(
            JoystickUtils.monomialScale(
                JoystickUtils.deadband(-p1.getY(Hand.kLeft)),
                DriveConstants.JOYSTICK_EXPONENT, 
                DriveConstants.JOYSTICK_COEFFICIENT), 
            JoystickUtils.monomialScale(
                JoystickUtils.deadband(-p1.getY(Hand.kRight)),
                DriveConstants.JOYSTICK_EXPONENT,
                DriveConstants.JOYSTICK_COEFFICIENT),
            reverse
        );
        /*
        drive.arcade(
            JoystickUtils.deadband(-p1.getY(Hand.kLeft)), 
            JoystickUtils.deadband(p1.getX(Hand.kRight))
        );
        */

        /**
         * Hopper
         */
        int p1Pov = p1.getPOV();
        if (p1Pov >= 315 || p1Pov <= 45) {
            hopper.moveBelt(1.0);
        } else if (p1Pov >= 225 && p1Pov <= 135) {
            hopper.moveBelt(-1.0);
        } else {
            hopper.moveBelt(0);
        }

     
        //
        // WITHOUT HYPERSHIFT
        //

        if(!p1.getBumper(Hand.kRight)) {

            /**
             * Drivetrain
             */
            //shifting
            if (p1.getBumper(Hand.kLeft)) {
                drive.setGearShift(GearShift.HIGH);
            } else if (p1.getBumperReleased(Hand.kLeft)) {
                drive.setGearShift(GearShift.LOW);
            }

            /**
             * Intake
             */
            //run intake in
            intake.intakeBalls(
                JoystickUtils.deadband(
                    IntakeConstants.CONTROL_SCALING_FACTOR * 
                        (p1.getTriggerAxis(Hand.kLeft)))
            );


            return;
        }

        //
        // WITH HYPERSHIFT
        //

        /**
         * Intake
         */
        // run intake out
        intake.intakeBalls(
        JoystickUtils.deadband(
            IntakeConstants.CONTROL_SCALING_FACTOR * 
                (-p1.getTriggerAxis(Hand.kLeft)))
        );
        // extend and retract
        if (p1.getBumperPressed(Hand.kLeft)) {
            intake.invertStraw();
        }


    }

    private void p2Loop() {
        /**
         * Compressor
         */
        if (p2.getBackButtonPressed()) {
            compressor.changeState();
            System.out.println("Compressor on: " + compressor.getState());
        }

        /**
         * Climb
         */
        // Climb Code
        climb.climb(p2.getTriggerAxis(Hand.kRight) - p2.getTriggerAxis(Hand.kLeft));

        /**
         * Hopper
         */
        if(p1.getPOV() == -1){
            hopper.moveBelt(JoystickUtils.deadband(-p2.getY(Hand.kLeft)));
        }

        /**
         * WOF
         */
        if (p2.getYButtonPressed()) {
            wheelOfFortune.engage();
        }
        if (p2.getBumper(Hand.kLeft)) {
            wheelOfFortune.spin(
                JoystickUtils.deadband(p2.getY(Hand.kRight))
            );
        } else {
            wheelOfFortune.stop();
        }
        //B button does rotation ctrl
        //X button does colour
        
        /**
         * Override
         */
        if(p2.getBumper(Hand.kLeft)) {
            /**
             * Turret Override
             */
            //PID turret to degree using the Dpad

            /**
             * Shooter Override
             */
            //If left bumper held shooter override
            shooter.shoot(p2.getTriggerAxis(Hand.kRight), false);
            return;
        }

        /**
         * Shooter
         */
        //aim + start rotation
        if(p2.getAButton()) {
            ShooterUtils.aim();
        } else if(p2.getAButtonReleased()) {
            ShooterUtils.stop();
        }

    }
}