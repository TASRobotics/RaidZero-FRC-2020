package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpiutil.math.MathUtil;

import raidzero.robot.Constants.LimelightConstants;
import raidzero.robot.Constants.TurretConstants;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Turret;
import raidzero.robot.submodules.Limelight.CameraMode;
import raidzero.robot.submodules.Limelight.LedMode;

/**
 * Action for turning the turret towards the goal using vision.
 */
public class TurnToGoal implements Action {

    private static final Turret turret = Turret.getInstance();
    private static final Limelight limelight = Limelight.getInstance();

    private PIDController pidController;
    private double headingError;

    public TurnToGoal() {
        pidController = new PIDController(LimelightConstants.AIM_KP, LimelightConstants.AIM_KI, 
            LimelightConstants.AIM_KD);
        pidController.setTolerance(LimelightConstants.ANGLE_ADJUST_THRESHOLD);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void start() {
        pidController.setSetpoint(0.0);

        limelight.setLedMode(LedMode.On);
        limelight.setPipeline(0);
        limelight.setCameraMode(CameraMode.Vision);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        if (!limelight.hasTarget()) {
            turret.stop();
            return;
		}
        headingError = limelight.getTx();

		// Steering adjust P controller with offset
        // double steeringAdjust = LimelightConstants.AIM_KP * headingError;
        // System.out.println("Heading error: " + steeringAdjust);

		turret.rotateManual(
            MathUtil.clamp(pidController.calculate(headingError),
                -TurretConstants.MAX_INPUT_PERCENTAGE, TurretConstants.MAX_INPUT_PERCENTAGE)
        );
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        // limelight.setLedMode(LedMode.Off);
        turret.stop();
    }
}