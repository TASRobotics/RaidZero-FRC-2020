package raidzero.robot.auto.actions;

import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Turret;

/**
 * Action for turning the drive towards the goal using vision.
 */
public class TurnToGoal implements Action {

    private static final Turret turret = Turret.getInstance();
    private static final Limelight limelight = Limelight.getInstance();

    private double headingError;

    public TurnToGoal() {
    }

    @Override
    public boolean isFinished() {
        return Math.abs(headingError) < Constants.ANGLE_ADJUST_THRESHOLD;
    }

    @Override
    public void update() {
        if (!limelight.hasTarget()) {
            turret.stop();
            return;
		}
        headingError = limelight.getTx();
        //System.out.println("Heading error: " + headingError);
		
		double steeringAdjust = 0.0;
		// Steering adjust P controller with offset
        steeringAdjust = Constants.KP_AIM * headingError;
        System.out.println("Heading error: " + steeringAdjust);

		turret.rotateManual(MathUtil.clamp(steeringAdjust, -0.2, 0.2));
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        turret.stop();
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }
}