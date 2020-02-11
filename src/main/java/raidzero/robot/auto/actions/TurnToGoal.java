package raidzero.robot.auto.actions;

import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.Constants.LimelightConstants;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Turret;

/**
 * Action for turning the turret towards the goal using vision.
 */
public class TurnToGoal implements Action {

    private static final Turret turret = Turret.getInstance();
    private static final Limelight limelight = Limelight.getInstance();

    private double headingError;

    public TurnToGoal() {}

    @Override
    public boolean isFinished() {
        return Math.abs(headingError) < LimelightConstants.ANGLE_ADJUST_THRESHOLD;
    }

    @Override
    public void start() {
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
        double steeringAdjust = LimelightConstants.AIM_KP * headingError;
        System.out.println("Heading error: " + steeringAdjust);

		turret.rotateManual(MathUtil.clamp(steeringAdjust, -0.2, 0.2));
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        turret.stop();
    }
}