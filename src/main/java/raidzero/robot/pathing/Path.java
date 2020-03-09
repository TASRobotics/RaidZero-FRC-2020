package raidzero.robot.pathing;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import raidzero.robot.submodules.Drive;

public class Path {

    protected static final Drive drive = Drive.getInstance();

    protected Trajectory trajectory;

    /**
     * Initializes a path with a trajectory.
     */
    private Path(Trajectory trajectory) {
        this.trajectory = trajectory;
    }

    public static Path fromWaypoints(Pose2d start, Pose2d end, boolean reversed, 
        double maxVelocity, double maxAcceleration
    ) {
        return fromWaypoints(start, new ArrayList<>(), end, reversed, maxVelocity, maxAcceleration);
    }

    public static Path fromWaypoints(Pose2d start, List<Translation2d> interiorWaypoints, 
        Pose2d end, boolean reversed, double maxVelocity, double maxAcceleration
    ) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        config.setKinematics(drive.getKinematics());
        // config.addConstraint(DriveConstants.VOLTAGE_CONSTRAINT);
    
        // Uses clamped cubic splines
        return new Path(TrajectoryGenerator.generateTrajectory(
            start, interiorWaypoints, end, config));
    }

    public static Path fromWaypoints(List<Pose2d> waypoints, boolean reversed, 
        double maxVelocity, double maxAcceleration
    ) {
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(reversed);
        config.setKinematics(drive.getKinematics());
        // config.addConstraint(DriveConstants.VOLTAGE_CONSTRAINT);
    
        // Uses quintic hermite splines
        return new Path(TrajectoryGenerator.generateTrajectory(waypoints, config));
    }

    /**
     * Returns a trajectory loaded from a JSON file.
     * Note: PathWeaver files usually look like XXX.wpilib.json
     * 
     * @param filename name of the JSON path file
     * @return path with loaded trajectory
     */
    protected Path fromJson(String filename) {
        try {
            return new Path(TrajectoryUtil.fromPathweaverJson(
                Filesystem.getDeployDirectory().toPath().resolve("paths/" + filename)
            ));
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
