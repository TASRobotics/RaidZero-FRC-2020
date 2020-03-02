package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidzero.pathgen.Point;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;

public class TenCellTrenchSequence extends AutoSequence {

    private static final Point[] RZ_FORWARD = {
        new Point(123, -57, 0),
        new Point(240, -109, 69)
    };

    private static final Point[] RZ_BACKWARD = {
        new Point(240, -109, 249),
        new Point(161, -95, 180)
    };

    private static final Point[] TRENCH_FORWARD_WAYPOINTS = {
        new Point(161, -95, 0),
        new Point(209, -24, 0),
        new Point(300, -24, 0),
        new Point(367, -24, 0)
    };

    private static final Point[] TRENCH_BACKWARD_WAYPOINTS = {
        new Point(367, -24, 180),
        new Point(332, -24, 180),
        new Point(197, -95, 180)
    };

    private static final Intake intake = Intake.getInstance();
    private static final Shooter shooter = Shooter.getInstance();

    public TenCellTrenchSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new SeriesAction(Arrays.asList(
                            new TurnTurretToAngle(100),
                            new TurnToGoal()
                        )),
                        new SeriesAction(Arrays.asList(
                            new SetHoodPosition(5800)
                        ))
                    )
                ),
                new SetShooterVelocity(1.0),
                new FeedBalls(2.0),
                new LambdaAction(() -> shooter.stop()),
                new LambdaAction(() -> intake.intakeBalls(1.0)),
                new DrivePath(new Path(TRENCH_FORWARD_WAYPOINTS, false)),
                new LambdaAction(() -> intake.stop())
            )
        ));
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new DrivePath(new Path(TRENCH_BACKWARD_WAYPOINTS, false, 10, 
                        DriveConstants.DEFAULT_TARGET_ACCELERATION)),
                    new TurnToGoal(),
                    new SetShooterVelocity(1.0)                   
                )),
                new TurnToGoal(),
                new FeedBalls(2.0),
                new LambdaAction(() -> shooter.stop())
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TenCellTrenchSequence ended!");
    }

    @Override
    public String getName() {
        return "Ten Cell Trench";
    }
}