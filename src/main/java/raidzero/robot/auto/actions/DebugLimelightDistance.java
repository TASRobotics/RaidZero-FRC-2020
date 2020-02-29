package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.robot.Constants;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Limelight.CameraMode;
import raidzero.robot.submodules.Limelight.LedMode;
import raidzero.robot.utils.InterpolatingDouble;
import raidzero.robot.utils.LimelightUtils;

/**
 * Action for debugging limelight distance estimation.
 */
public class DebugLimelightDistance implements Action {

    private static final Limelight limelight = Limelight.getInstance();

    private MedianFilter filter = new MedianFilter(5);

    public DebugLimelightDistance() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void start() {
        limelight.setLedMode(LedMode.On);
        limelight.setPipeline(0);
        limelight.setCameraMode(CameraMode.Vision);

        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        double distance = LimelightUtils.estimateDistance(filter.calculate(limelight.getTy()));
        SmartDashboard.putNumber("Distance (m)", distance);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
