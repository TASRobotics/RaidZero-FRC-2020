package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.Constants.FieldConstants;
import raidzero.robot.Constants.LimelightConstants;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Shooter;

/**
 * Action for using vision to estimate distance and choose an appropiate velocity setpoint.
 */
public class VisionAssistedShooting implements Action {

    private static final Shooter shooter = Shooter.getInstance();
    private static final Limelight limelight = Limelight.getInstance();

    private static final double FILTER_PERIOD = 0.2;

    private double startTime = 0.0;
    private MedianFilter filter = new MedianFilter(10);

    public VisionAssistedShooting() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        if (Timer.getFPGATimestamp() - startTime < FILTER_PERIOD) {
            filter.calculate(limelight.getTy());
            return;
        }
        double distance = estimateDistance(filter.calculate(limelight.getTy()));
        // TODO: Interpolating lookup table for distance vs. velocity of shooter
        System.out.println("Distance: " + distance + " m");
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