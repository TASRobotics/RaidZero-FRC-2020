package raidzero.robot.auto.actions;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import raidzero.robot.dashboard.Tab;
import raidzero.robot.submodules.Limelight;
import raidzero.robot.submodules.Limelight.CameraMode;
import raidzero.robot.submodules.Limelight.LedMode;
import raidzero.robot.utils.LimelightUtils;

/**
 * Action for debugging limelight distance estimation.
 */
public class DebugLimelightDistance implements Action {

    private static final Limelight limelight = Limelight.getInstance();

    private MedianFilter filter = new MedianFilter(5);

    private NetworkTableEntry distanceEntry =
            Shuffleboard.getTab(Tab.MAIN)
                .add("Estimated Distance (m)", 0.0)
                .withWidget(BuiltInWidgets.kNumberBar)
                .withProperties(Map.of("min", 0.0, "max", 10.0))
                .withSize(3, 1)
                .withPosition(2, 0)
                .getEntry();

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
        distanceEntry.setDouble(distance);
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
