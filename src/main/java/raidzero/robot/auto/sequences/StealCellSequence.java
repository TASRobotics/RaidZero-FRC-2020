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

public class StealCellSequence extends AutoSequence {

    private static final Point[] TO_STEAL_CELLS_WAYPOINTS = {
        new Point(150, -270, 0),
        new Point(263, -295, -30)
    };
    private static final Path TO_STEAL_CELLS_PATH = new Path(TO_STEAL_CELLS_WAYPOINTS, false, 
        7.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] TO_FRONT_GOAL_WAYPOINTS = {
        new Point(263, -295, 150),
        new Point(160, -95, 140)
    };
    private static final Path TO_FRONT_GOAL_PATH = new Path(TO_FRONT_GOAL_WAYPOINTS, true,
        10.0, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Drive drive = Drive.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Shooter shooter = Shooter.getInstance();

    public StealCellSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(
                    Arrays.asList(
                        new LambdaAction(() -> intake.invertStraw()),
                        new LambdaAction(() -> intake.intakeBalls(1.0)),
                        new DrivePath(TO_STEAL_CELLS_PATH)
                    )
                ),
                new ParallelAction(
                    Arrays.asList(
                        new DrivePath(TO_FRONT_GOAL_PATH),
                        new SetShooterVelocity(1.0),
                        new TurnTurretToAngle(120),
                        new SetHoodPosition(5800)
                    )
                ),
                new TurnToGoal(),
                new LambdaAction(() -> drive.setBrakeMode(true)),
                new FeedBalls(4.0),
                new LambdaAction(() -> drive.setBrakeMode(false)),
                new LambdaAction(() -> intake.stop()),
                new LambdaAction(() -> shooter.stop())
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("StealCellSequence ended!");
    }

    @Override
    public String getName() {
        return "Steal Cell from Trench";
    }
}