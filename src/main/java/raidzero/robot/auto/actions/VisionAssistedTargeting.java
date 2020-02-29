package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Limelight.CameraMode;
import raidzero.robot.submodules.Limelight.LedMode;
import raidzero.robot.utils.LimelightUtils;
import raidzero.robot.Constants.AdjustableHoodConstants;

/**
 * Action for using vision to estimate distance and choose an appropiate velocity setpoint.
 */
public class VisionAssistedTargeting implements Action {

    private enum ActionPhase {
        ESTIMATING_DISTANCE, SET_HOOD_POSITION
    }

    private static final AdjustableHood hood = AdjustableHood.getInstance();
    private static final Limelight limelight = Limelight.getInstance();

    private static final double FILTER_PERIOD = 0.2;

    private double startTime = 0.0;
    private MedianFilter filter = new MedianFilter(5);

    private ActionPhase phase;

    public VisionAssistedTargeting() {}

    @Override
    public boolean isFinished() {
        return phase == ActionPhase.SET_HOOD_POSITION;
    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
        phase = ActionPhase.ESTIMATING_DISTANCE;

        limelight.setLedMode(LedMode.On);
        limelight.setPipeline(0);
        limelight.setCameraMode(CameraMode.Vision);

        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        if (phase == ActionPhase.ESTIMATING_DISTANCE) {
            if (Timer.getFPGATimestamp() - startTime < FILTER_PERIOD) {
                filter.calculate(limelight.getTy());
                return;
            }
            // Filter time is up
            double distance = LimelightUtils.estimateDistance(filter.calculate(limelight.getTy()));
            System.out.println("Distance: " + distance + " m");
            
            hood.moveToTick(distanceToHoodTick(distance));

            startTime = Timer.getFPGATimestamp();
            phase = ActionPhase.SET_HOOD_POSITION;
        } 
    }

    @Override
    public void done() {
        limelight.setLedMode(LedMode.Off);

        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }

    /**
     * Uses a curve fit to estimate the hood angle at a certain distance.
     * 
     * @param distance distance from the target (m)
     * @return hood position (encoder tick)
     */
    private int distanceToHoodTick(double distance) {
        int tick = (int) (AdjustableHoodConstants.ATAN_COEFFICIENT * 
            (Math.atan(AdjustableHoodConstants.DISTANCE_COEFFICIENT * distance)) + 
            AdjustableHoodConstants.ANGLE_CONSTANT);
        return tick;
    }
}