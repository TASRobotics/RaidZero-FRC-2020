package raidzero.robot.pathing.paths;

import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.pathing.Path;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;

public class TestPath extends Path {

    @Override
    public Trajectory generateTrajectory() {
        Pose2d startPose = new Pose2d(
            Units.feetToMeters(0), Units.feetToMeters(0),
            Rotation2d.fromDegrees(0));

        Pose2d endPose = new Pose2d(
            Units.feetToMeters(4), Units.feetToMeters(4),
            Rotation2d.fromDegrees(0));
    
        List<Translation2d> interiorWaypoints = new ArrayList<>();
        /*interiorWaypoints.add(
            new Translation2d(Units.feetToMeters(14.54), Units.feetToMeters(23.23)));
        interiorWaypoints.add(
            new Translation2d(Units.feetToMeters(21.04), Units.feetToMeters(18.23)));*/
    
        TrajectoryConfig config = new TrajectoryConfig(maxVelocity, maxAcceleration);
        config.setReversed(false);
        config.setKinematics(drive.getKinematics());
        config.addConstraint(DriveConstants.VOLTAGE_CONSTRAINT);
    
        // Uses clamped cubic splines
        return TrajectoryGenerator.generateTrajectory(
            startPose, interiorWaypoints, endPose, config);
    }
}