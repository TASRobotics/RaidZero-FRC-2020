package raidzero.robot.auto.actions;

import edu.wpi.first.wpiutil.math.MathUtil;
import raidzero.robot.Constants;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Limelight;

/**
 * Action for turning the drive towards the goal using vision.
 */
public class TurnToGoal implements Action {

    private static final Drive drive = Drive.getInstance();
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
            drive.stop();
            return;
		}
		headingError = limelight.getTx();
		
		double steeringAdjust = 0.0;
		// Steering adjust P controller with offset
		if (headingError > Constants.ANGLE_ADJUST_THRESHOLD) {
			steeringAdjust = Constants.KP_AIM * headingError + Constants.MINIMUM_POWER;
		} else if (headingError < -Constants.ANGLE_ADJUST_THRESHOLD) {
			steeringAdjust = Constants.KP_AIM * headingError - Constants.MINIMUM_POWER;
		}

		// Tank drive with steering and heading adjust, applying a percentage speed limit
		double leftOutput = MathUtil.clamp(steeringAdjust, -0.75, 0.75);
		double rightOutput = MathUtil.clamp(-steeringAdjust, -0.75, 0.75);

		drive.tank(leftOutput, rightOutput);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        drive.stop();
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        drive.setOpenLoop();
    }
}