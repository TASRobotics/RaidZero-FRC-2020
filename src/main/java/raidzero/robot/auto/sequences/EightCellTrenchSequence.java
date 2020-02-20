package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.Intake;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.submodules.Turret;

public class EightCellTrenchSequence extends AutoSequence {

    private static final Point[] TRENCH_FORWARD_WAYPOINTS = {
        new Point(120, -80, 90),
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

    public EightCellTrenchSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new SetShooterVelocity(1.0), // TODO: Either use distance or measure a working speed
                new FeedBalls(2.0),
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
        System.out.println("EightCellTrenchSequence ended!");
    }

    @Override
    public String getName() {
        return "Eight Cell Trench";
    }
}