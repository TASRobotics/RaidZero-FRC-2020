package raidzero.robot.pathing;

import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.submodules.Drive;

public abstract class Path {

    protected static final Drive drive = Drive.getInstance();

    protected double maxVelocity;
    protected double maxAcceleration;

    protected Trajectory trajectory;

    /**
     * Constructor using default constants.
     */
    public Path() {
        this(DriveConstants.MAX_VELOCITY, DriveConstants.MAX_ACCELERATION);
    }

    /**
     * Initializes a path and generates a trajectory.
     * 
     * @param maxVelocity maximum velocity in m/s
     * @param maxAcceleration maximum acceleration in m/s^2
     */
    public Path(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        trajectory = generateTrajectory();
    }

    /**
     * This method should be overriden to generate the trajectory.
     * 
     * @return path trajectory
     */
    protected abstract Trajectory generateTrajectory();

    /**
     * Returns a trajectory loaded from a JSON file.
     * Note: PathWeaver files usually look like XXX.wpilib.json
     * 
     * @param filename name of the JSON path file
     * @return loaded trajectory
     */
    protected Trajectory loadFromJson(String filename) {
        try {
            return TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve("paths/" + filename)
            );
        } catch (IOException exception) {
            DriverStation.reportError("Unable to open trajectory: " + filename, exception.getStackTrace());
        }
        return null;
    }

    /**
     * Returns the stored trajectory in this path.
     * 
     * @return the pre-generated trajectory
     */
    public Trajectory getTrajectory() {
        return trajectory;
    }
}
