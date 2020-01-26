package raidzero.robot.auto.actions;

import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Drive;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Drive drive = Drive.getInstance();

    private Path path;
    private boolean isFirstPath;

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     */
    public DrivePath(Path path) {
        this.path = path;
        this.isFirstPath = false;
    }

    /**
     * Constructs a DrivePath action that follows a path.
     * 
     * @param path the path to follow
     * @param isFirstPath whether this is the first path in an autonomous sequence
     */
    public DrivePath(Path path, boolean isFirstPath) {
        this.path = path;
        this.isFirstPath = isFirstPath;
    }

    @Override
    public boolean isFinished() {
        if (drive.isFinishedWithPath()) {
            System.out.println("[Auto] Path finished!");
            return true;
        }
        return false;
    }

    @Override
    public void update() {}

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        drive.stop();
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        if (isFirstPath) {
            /**
             * Set the odometry pose to the start of the trajectory.
             * Note: Should only do this on the first trajectory.
             */ 
            drive.resetOdometry(path.getTrajectory().getInitialPose());
        }
        drive.setDrivePath(path);
    }
}