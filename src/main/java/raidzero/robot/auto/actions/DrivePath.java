package raidzero.robot.auto.actions;

import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Drive;

/**
 * Action for following a path.
 */
public class DrivePath implements Action {

    private static final Drive drive = Drive.getInstance();

    private Path path;

    public DrivePath(Path path) {
        this.path = path;
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
