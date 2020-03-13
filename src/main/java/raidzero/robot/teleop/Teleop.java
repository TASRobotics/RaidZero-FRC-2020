package raidzero.robot.teleop;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import raidzero.robot.Constants.IntakeConstants;
import raidzero.robot.Constants.HoodConstants.HoodAngle;
import raidzero.robot.auto.actions.DebugLimelightDistance;
import raidzero.robot.dashboard.Tab;
import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.submodules.Climb;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Superstructure;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.WheelOfFortune;
import raidzero.robot.submodules.Drive.GearShift;
import raidzero.robot.utils.JoystickUtils;
import raidzero.robot.wrappers.InactiveCompressor;

/**
 * TODO: Rewrite to make button assignments easier
 */
public class Teleop {

    private enum DriveMode {
        TANK(0), ARCADE(1), CURVATURE(2);

        private static final DriveMode[] modes = {TANK, ARCADE, CURVATURE};
        public final int index;

        private DriveMode(int index) {
            this.index = index;
        }

        private DriveMode next() {
            if (index == modes.length - 1) {
                return modes[0];
            }
            return modes[index + 1];
        }
    }

    private static Teleop instance = null;

    public static Teleop getInstance() {
        if (instance == null) {
            instance = new Teleop();
        }
        return instance;
    }

    private Teleop() {
    }

    private static Drive drive = Drive.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Intake intake = Intake.getInstance();
    private static Hopper hopper = Hopper.getInstance();
    private static Turret turret = Turret.getInstance();
    private static Climb climb = Climb.getInstance();
    private static WheelOfFortune wheelOfFortune = WheelOfFortune.getInstance();
    private static AdjustableHood hood = AdjustableHood.getInstance();
    private static InactiveCompressor compressor = InactiveCompressor.getInstance();
    private static Superstructure superstructure = Superstructure.getInstance();

    private XboxController p1 = new XboxController(0);
    private XboxController p2 = new XboxController(1);

    private DebugLimelightDistance debugDistance = new DebugLimelightDistance();

    private DriveMode driveMode = DriveMode.TANK;

    private NetworkTableEntry driveModeEntry = Shuffleboard.getTab(Tab.MAIN)
        .add("Drive Mode", driveMode.toString())
        .withWidget(BuiltInWidgets.kTextView)
        .withSize(1, 1)
        .withPosition(2, 2)
        .getEntry();

    /**
     * Runs at the start of teleop.
     */
    public void onStart() {
        drive.stop();
        drive.setGearShift(GearShift.LOW);

        climb.lock();

        debugDistance.start();
    }

    /**
     * Continuously loops in teleop.
     */
    public void onLoop() {
        /**
         * Climb
         */
        // Climb safety
        if (p1.getStartButton() && p2.getStartButton()) {
            climb.unlock();
        }

        p1Loop();
        p2Loop();

        debugDistance.update();
    }

    private void p1Loop() {
        //
        // REGARDLESS OF HYPERSHIFT
        //
        // Reversing analog to digital
        boolean reverse = JoystickUtils.deadband(p1.getTriggerAxis(Hand.kRight)) != 0;

        /**
         * Drivetrain
         */
        // Cycle through drive modes
        if (p1.getBackButtonPressed()) {
            driveMode = driveMode.next();
        }
        driveModeEntry.setString(driveMode.toString());
        //if (!superstructure.isCloseAligning()) {
            double speedMultiplier = 1.0;
            if (climb.isUnlocked()) {
                speedMultiplier = 0.5;
            }
            switch (driveMode) {
                case TANK:
                    drive.tank(
                        JoystickUtils.deadband(-p1.getY(Hand.kLeft)) * speedMultiplier,
                        JoystickUtils.deadband(-p1.getY(Hand.kRight)) * speedMultiplier,
                        reverse
                    );
                    break;
                case ARCADE:
                    drive.arcade(
                        JoystickUtils.deadband(-p1.getY(Hand.kLeft)) * speedMultiplier,
                        JoystickUtils.deadband(p1.getX(Hand.kRight)) * speedMultiplier, 
                        reverse
                    );
                    break;
                case CURVATURE:
                    double xSpeed = JoystickUtils.deadband(-p1.getY(Hand.kLeft)) * speedMultiplier;
                    drive.curvatureDrive(xSpeed, JoystickUtils.deadband(p1.getX(Hand.kRight)),
                        Math.abs(xSpeed) < 0.1
                    );
                    break;
            }
        //}

        // Braking
        if (p1.getAButtonPressed()) {
            drive.setBrakeMode(true);
        } else if (p1.getAButtonReleased()) {
            drive.setBrakeMode(false);
        }

        /**
         * Hopper
         */
        int p1Pov = p1.getPOV();
        if (p1Pov == -1) {
            hopper.stop();
        } else if (p1Pov >= 315 || p1Pov <= 45) {
            hopper.moveAtVelocity(0.75);
        } else if (p1Pov <= 225 && p1Pov >= 135) {
            hopper.moveAtVelocity(-0.75);
        } else {
            hopper.stop();
        }

        //
        // WITHOUT HYPERSHIFT
        //
        if (!p1.getBumper(Hand.kRight)) {

            /**
             * Drivetrain
             */
            // shifting
            if (p1.getBumper(Hand.kLeft)) {
                drive.setGearShift(GearShift.HIGH);
            } else if (p1.getBumperReleased(Hand.kLeft)) {
                drive.setGearShift(GearShift.LOW);
            }

            /*if (p1.getBButtonPressed()) {
                superstructure.setCloseAlign(true);
            } else if (p1.getBButtonReleased()) {
                superstructure.setCloseAlign(false);
            }*/

            /**
             * Intake
             */
            // Run intake in
            intake.intakeBalls(JoystickUtils.deadband(
                    IntakeConstants.CONTROL_SCALING_FACTOR * (p1.getTriggerAxis(Hand.kLeft))));
            return;
        }

        //
        // WITH HYPERSHIFT
        //
        /**
         * Intake
         */
        // Run intake out
        intake.intakeBalls(JoystickUtils.deadband(
                IntakeConstants.CONTROL_SCALING_FACTOR * (-p1.getTriggerAxis(Hand.kLeft))));

        // Extend and retract
        if (p1.getBumperPressed(Hand.kLeft)) {
            intake.invertStraw();
        }
    }

    private void p2Loop() {
        /**
         * Hopper
         */
        if (p1.getPOV() == -1) {
            double p2LeftJoystick = JoystickUtils.deadband(-p2.getY(Hand.kLeft));
            if (p2LeftJoystick > 0) {
                hopper.moveAtVelocity(0.75);
            } else if (p2LeftJoystick < 0) {
                hopper.moveAtVelocity(-0.75);
            } else {
                hopper.stop();
            }
        }

        /**
         * Override
         */
        if (p2.getBumper(Hand.kLeft)) {            
            /**
             * WOF Override
             */
            wheelOfFortune.spin(JoystickUtils.deadband(p2.getX(Hand.kRight)));

            /**
             * Shooter Override
             */
            // If left bumper held shooter override
            double rightTrigger = JoystickUtils.deadband(p2.getTriggerAxis(Hand.kRight));
            if (Math.abs(rightTrigger) > 0) {
                shooter.shoot(rightTrigger, false);
            } else {
                if (p2.getBumper(Hand.kRight)) {
                    shooter.shoot(0.8125, false);
                } else {
                    shooter.stop();
                }
            }
            

            if (p2.getAButtonPressed()) {
                superstructure.setTurretPIDing(true);
            } else if (p2.getAButtonReleased()) {
                superstructure.setTurretPIDing(false);
            }
            return;
        }

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
        climb.climb(p2.getTriggerAxis(Hand.kRight) - p2.getTriggerAxis(Hand.kLeft));
        if (p2.getStartButton()) {
            climb.openServo();
        } else {
            climb.closeServo();
        }

        /**
         * WOF
         */
        if (p2.getYButtonPressed()) {
            wheelOfFortune.engage();
        }
        // B button does rotation ctrl
        // X button does colour

        /**
         * Turret
         */
        // Aim
        if (p2.getAButtonPressed()) {
            superstructure.setAiming(true);
        } else if (p2.getAButtonReleased()) {
            // In case the override button is released while PIDing
            if (superstructure.isTurretPIDing()) {
                superstructure.setTurretPIDing(false);
            }
            superstructure.setAiming(false);
        }
        // Turn turret using right joystick
        if (!superstructure.isUsingTurret()) {
            turret.rotateManual(JoystickUtils.deadband(p2.getX(Hand.kRight)));
        }

        /**
         * Shooter
         */
        if (p2.getBumperPressed(Hand.kRight)) {
            shooter.shoot(1.0, false);
        } else if (p2.getBumperReleased(Hand.kRight)) {
            shooter.shoot(0.0, false);
        }

        /**
         * Hood
         */
        if (p2.getStickButton(Hand.kRight)) {
            superstructure.setAimingAndHood(true);
        } else {
            superstructure.setAimingAndHood(false);
        }

        /**
         * Adjustable hood
         */
        int p2Pov = p2.getPOV();
        if (p2Pov == 0) {
            hood.moveToAngle(HoodAngle.RETRACTED);
        } else if (p2Pov == 90) {
            hood.moveToAngle(HoodAngle.HIGH);
        } else if (p2Pov == 180) {
            hood.moveToAngle(HoodAngle.MEDIUM);
        } else if (p2Pov == 270) {
            hood.moveToAngle(HoodAngle.LOW);
        } else {
            if (p2.getXButton()) {
                hood.adjust(-0.3);
            } else if (p2.getBButton()) {
                hood.adjust(0.3);
            } else {
                hood.stop();
            }
        }
    }
}
