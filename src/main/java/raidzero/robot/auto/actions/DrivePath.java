package raidzero.robot.auto.actions;

import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Drive;
import raidzero.robot.utils.FinishConditionInterface;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Drive drive = Drive.getInstance();

    private Path path;
    private FinishConditionInterface condition;

    /**
     * Constructs a DrivePath action that finishes when the path is finished.
     * 
     * @param path path to follow
     */
    public DrivePath(Path path) {
        this(path, null);
    }

    /**
     * Constructs a DrivePath action that finishes when the path is finished or
     * if the provided lambda returns true.
     * 
     * @param path      path to follow
     * @param condition alternate condition to end the action
     */
    public DrivePath(Path path, FinishConditionInterface condition) {
        this.path = path;
        this.condition = condition;
    }

    @Override
    public boolean isFinished() {
        return drive.isFinishedWithPath() || (condition != null && condition.passed());
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        drive.setDrivePath(path);
    }

    @Override
    public void update() {
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        drive.stop();
    }
}
