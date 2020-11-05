package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;

import raidzero.pathgen.Point;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.*;
import raidzero.robot.auto.actions.TurnToGoal.DefaultMode;
import raidzero.robot.pathing.Path;
import raidzero.robot.submodules.*;
import raidzero.robot.submodules.Intake.Position;

public class SixCellTrenchSequence extends AutoSequence {

    private static final Point[] TRENCH_TOWARDS_BALL_WAYPOINTS = {
        new Point(-120, -24, 180),
        new Point(-222, -24, 180),
        new Point(-300, -24, 180),
        new Point(-330, -24, 180)
    };
    private static final Path TRENCH_TOWARDS_BALLS_PATH = new Path(TRENCH_TOWARDS_BALL_WAYPOINTS, true, 
        5.0, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] TRENCH_TOWARDS_GOAL_WAYPOINTS = {
        new Point(-330, -24, 0),
        new Point(-200, -24, 0),
        new Point(-140, -24, 0)
    };
    private static final Path TRENCH_TOWARDS_GOAL_PATH = new Path(TRENCH_TOWARDS_GOAL_WAYPOINTS, false,
        9.0, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Intake intake = Intake.getInstance();
    private static final Shooter shooter = Shooter.getInstance();
    private static final Drive drive = Drive.getInstance();

    public SixCellTrenchSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new LambdaAction(() -> drive.setBrakeMode(true)),
                new ParallelAction(
                    Arrays.asList(
                        new LambdaAction(() -> shooter.shoot(1.0, false)),
                        new SetShooterVelocity(1.0),
                        new LambdaAction(() -> intake.setPosition(Position.DOWN)),
                        // Optional Old Turret rotation method
                        /** new SeriesAction(Arrays.asList(
                            new TurnTurretToAngle(100),
                            new TurnToGoal()
                        )), **/
                        new TurnToGoal(DefaultMode.COUNTER_CLOCKWISE),
                        new SetHoodPosition(6320)
                    )
                ),
                new FeedBalls(1),
                new LambdaAction(() -> intake.intakeBalls(0.7)),
                new DrivePath(TRENCH_TOWARDS_BALLS_PATH, true)
            )
        ));
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new DrivePath(TRENCH_TOWARDS_GOAL_PATH, true), // TODO: Preferably not reset encoder position
                    new SetHoodPosition(6420),
                    new SetShooterVelocity(1.0)
                )),
                //new FeedBalls(0.3, true),
                new LambdaAction(() -> drive.setBrakeMode(true)),
                new TurnToGoal(),
                new FeedBalls(5.0),
                new LambdaAction(() -> drive.setBrakeMode(false)),
                new LambdaAction(() -> shooter.stop())
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("SixCellTrenchSequence ended!");
    }

    @Override
    public String getName() {
        return "Six Cell Trench";
    }
}