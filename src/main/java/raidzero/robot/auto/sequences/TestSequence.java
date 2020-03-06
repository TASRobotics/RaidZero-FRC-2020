package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidzero.pathgen.Point;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Drive;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Intake.Position;

public class TestSequence extends AutoSequence {

    private static final Point[] TO_STEAL_CELLS_WAYPOINTS = {
        new Point(150, -285, 0),
        new Point(258, -285, 0)
    };
    private static final Path TO_STEAL_CELLS_PATH = new Path(TO_STEAL_CELLS_WAYPOINTS, false, 
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] CURVE_BACK_WAYPOINTS = {
        new Point(258, -285, 180),
        new Point(210, -305, 180)
    };
    private static final Path CURVE_BACK_PATH = new Path(CURVE_BACK_WAYPOINTS, true, 
        10.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] GET_SECOND_CELL_WAYPOINTS = {
        new Point(210, -305, 0),
        new Point(258, -305, 0)
    };
    private static final Path GET_SECOND_CELL_PATH = new Path(GET_SECOND_CELL_WAYPOINTS, false, 
        10.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] TO_GOAL_WAYPOINT = {
        new Point(258, -305, 180),
        new Point(140, -100, 120)
    };
    private static final Path TO_GOAL_PATH = new Path(TO_GOAL_WAYPOINT, true, 
        8.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);
        

    private static final Drive drive = Drive.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Shooter shooter = Shooter.getInstance();

    public TestSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new LambdaAction(() -> intake.setPosition(Position.DOWN)),
                        new LambdaAction(() -> intake.intakeBalls(1.0)),
                        new DrivePath(TO_STEAL_CELLS_PATH)
                    )
                ),
                new WaitAction(0.2),
                new DrivePath(CURVE_BACK_PATH),
                new WaitAction(0.1),
                new DrivePath(GET_SECOND_CELL_PATH),
                new WaitAction(0.2),
                new DrivePath(TO_GOAL_PATH),
                new LambdaAction(() -> intake.stop())
            )
        ));
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