package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants;
import raidzero.robot.Constants.FieldConstants;
import raidzero.robot.Constants.LimelightConstants;
import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.utils.InterpolatingDouble;

/**
 * Action for using vision to estimate distance and choose an appropiate velocity setpoint.
 */
public class VisionAssistedShooting implements Action {

    private enum ActionPhase {
        ESTIMATING_DISTANCE, APPROACHING_SETPOINT
    }

    private static final Shooter shooter = Shooter.getInstance();
    private static final Limelight limelight = Limelight.getInstance();

    private static final double FILTER_PERIOD = 0.2;

    private double startTime = 0.0;
    private MedianFilter filter = new MedianFilter(10);

    private ActionPhase phase;

    public VisionAssistedShooting() {}

    @Override
    public boolean isFinished() {
        return phase == ActionPhase.APPROACHING_SETPOINT && (shooter.isUpToSpeed() ||
            Timer.getFPGATimestamp() - startTime > ShooterConstants.APPROACH_SETPOINT_TIMEOUT);
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        startTime = Timer.getFPGATimestamp();
        phase = ActionPhase.ESTIMATING_DISTANCE;
    }

    @Override
    public void update() {
        if (phase == ActionPhase.ESTIMATING_DISTANCE) {
            if (Timer.getFPGATimestamp() - startTime < FILTER_PERIOD) {
                filter.calculate(limelight.getTy());
                return;
            }
            // Filter time is up
            double distance = estimateDistance(filter.calculate(limelight.getTy()));
            InterpolatingDouble targetSpeed = Constants.DISTANCE_TO_SPEED.getInterpolated(
                new InterpolatingDouble(distance));
            System.out.println("Distance: " + distance + " m");
            System.out.println("Target Shooter Speed: " + (targetSpeed.value * 100) + "%");

            shooter.shoot(targetSpeed.value, false);

            startTime = Timer.getFPGATimestamp();
            phase = ActionPhase.APPROACHING_SETPOINT;
        } 
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }

    /**
     * Returns the estimated distance to the goal in meters.
     * 
     * @param angle ty from the limelight in degrees
     * @return estimated distance in meters
     */
    private double estimateDistance(double angle) {
        return (FieldConstants.GOAL_HEIGHT - LimelightConstants.MOUNTING_HEIGHT) / 
            Math.tan(Math.toRadians(LimelightConstants.MOUNTING_ANGLE + angle));
    }
}