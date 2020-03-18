package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;

import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.*;

public class TestSequence extends AutoSequence {

    private static final Path PATH = Path.fromWaypoints(
        Arrays.asList(
            new Pose2d(
                Units.inchesToMeters(0), Units.inchesToMeters(0),
                Rotation2d.fromDegrees(0)
            ),
            new Pose2d(
                1.5, 0.0,//Units.inchesToMeters(60), Units.inchesToMeters(30),
                Rotation2d.fromDegrees(0)
            )
        ),
        false, DriveConstants.DEFAULT_VELOCITY, DriveConstants.DEFAULT_ACCELERATION
    );
    private static final Drive drive = Drive.getInstance();

    public TestSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        /*addAction(new SeriesAction(Arrays.asList(
            new ParallelAction(Arrays.asList(
                new DrivePath(PATH, true),
                new LambdaAction(() -> intake.intakeBalls(0.8))
            )),
            new LambdaAction(() -> intake.stop())
        )));*/
        addAction(new SeriesAction(Arrays.asList(
            new DrivePath(PATH, true)
        )));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TestSequence ended!");
    }

    @Override
    public String getName() {
        return "Test Sequence";
    }
}