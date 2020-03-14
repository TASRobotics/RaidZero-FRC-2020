package raidzero.robot.auto.actions;

import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Drive.GearShift;
import raidzero.robot.utils.FinishConditionInterface;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Drive drive = Drive.getInstance();

    private Path path;
    private boolean isFirstPath;
    private FinishConditionInterface condition;

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     */
    public DrivePath(Path path) {
        this(path, false, null);
    }

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     * @param isFirstPath whether this is the first path in an autonomous sequence
     */
    public DrivePath(Path path, boolean isFirstPath) {
        this(path, isFirstPath, null);
    }

    /**
     * Constructs a DrivePath action that finishes when the path is finished or
     * if the provided lambda returns true.
     * 
     * @param path      path to follow
     * @param condition alternate condition to end the action
     */
    public DrivePath(Path path, FinishConditionInterface condition) {
        this(path, false, condition);
    }

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     * @param isFirstPath whether this is the first path in an autonomous sequence
     * @param condition alternate condition to end the action
     */
    public DrivePath(Path path, boolean isFirstPath, FinishConditionInterface condition) {
        this.path = path;
        this.isFirstPath = isFirstPath;
        this.condition = condition;
    }

    @Override
    public boolean isFinished() {
        return drive.isFinishedWithPath() || (condition != null && condition.passed());
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        if (isFirstPath) {
            drive.zero();
            /**
             * Set the odometry pose to the start of the trajectory.
             * Note: Should only do this on the first trajectory.
             */ 
            drive.resetOdometry(path.getTrajectory().getInitialPose());
        }
        drive.setGearShift(GearShift.LOW);
        drive.setBrakeMode(true);
        drive.setDrivePath(path);
    }

    @Override
    public void update() {
    }
    
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        drive.stop();
        drive.setBrakeMode(false);
    }
}
