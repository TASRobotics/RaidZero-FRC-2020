package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Turret;

public class TenCellTrenchSequence extends AutoSequence {

    private static final Point[] SQUARE_COLLECT_WAYPOINTS = {
        new Point(120, -30, 0),
        new Point(200, -50, -45),
        new Point(238, -108, -69)
    };
    private static final Path SQUARE_COLLECT_PATH = new Path(SQUARE_COLLECT_WAYPOINTS, false);
    
    private static final Point[] SQUARE_RETURN_WAYPOINTS = {
        new Point(238, -108, -69),
        new Point(225, -88, -45),
        new Point(200, -80, 0),
        new Point(160, -80, 0)
    };
    private static final Path SQUARE_RETURN_PATH = new Path(SQUARE_RETURN_WAYPOINTS, false);

    private static final Point[] TRENCH_FORWARD_WAYPOINTS = {
        new Point(160, -80, 0),
        new Point(222, -28, 0),
        new Point(300, -28, 0),
        new Point(390, -28, 0)
    };
    private static final Path TRENCH_FORWARD_PATH = new Path(TRENCH_FORWARD_WAYPOINTS, false);

    private static final Point[] TRENCH_BACKWARD_WAYPOINTS = {
        new Point(390, -28, 0),
        new Point(480, -28, 0)
    };
    private static final Path TRENCH_BACKWARD_PATH = new Path(TRENCH_BACKWARD_WAYPOINTS, true);

    private static final Intake intake = Intake.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Turret turret = Turret.getInstance();

    public TenCellTrenchSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new LambdaAction(() -> intake.intakeBalls(0.6)),
                new DrivePath(SQUARE_COLLECT_PATH),
                new LambdaAction(() -> intake.stop()),
                new DrivePath(SQUARE_RETURN_PATH),
                new SetShooterVelocity(1.0),
                new FeedBalls(2.0)
            )
        ));
        addAction(new SeriesAction(
            Arrays.asList(
                new LambdaAction(() -> shooter.stop()),
                new LambdaAction(() -> intake.intakeBalls(1.0)),
                new DrivePath(TRENCH_FORWARD_PATH),
                new LambdaAction(() -> intake.stop())
            )
        ));
        addAction(new SeriesAction(
            Arrays.asList(
                new LambdaAction(() -> turret.rotateToAngle(130)), // Approximate so Limelight can see
                new DrivePath(TRENCH_BACKWARD_PATH),
                new TurnToGoal(),
                new VisionAssistedTargeting(), // Estimates distance and spins up shooter
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