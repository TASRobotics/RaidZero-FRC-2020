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

public class SixCellTrenchSequence extends AutoSequence {

    private static final Point[] TRENCH_FORWARD_WAYPOINTS = {
        new Point(120, -24, 0),
        new Point(222, -24, 0),
        new Point(300, -24, 0),
        new Point(330, -24, 0)
    };
    private static final Path TRENCH_FORWARD_PATH = new Path(TRENCH_FORWARD_WAYPOINTS, false, 
        7.0, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Point[] TRENCH_BACKWARD_WAYPOINTS = {
        new Point(330, -24, 180),
        new Point(200, -24, 180),
        new Point(180, -24, 180)//delete this
        /*add these after testing, should help with accuracy and time
        new Point(332, -24, 180),
        new Point(170, -100, 180)
        */
    };
    private static final Path TRENCH_BACKWARD_PATH = new Path(TRENCH_BACKWARD_WAYPOINTS, true,
        14, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    private static final Drive drive = Drive.getInstance();
    private static final Intake intake = Intake.getInstance();
    private static final Shooter shooter = Shooter.getInstance();

    public SixCellTrenchSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                //new LambdaAction(() -> drive.setBrakeMode(true)),
                new ParallelAction(
                    Arrays.asList(
                        new LambdaAction(() -> shooter.shoot(2.0, false)),
                        new WaitAction(2.0),
                        new SetShooterVelocity(1.0),
                        new LambdaAction(() -> intake.setPosition(Position.DOWN)),
                        new SeriesAction(Arrays.asList(
                            new TurnTurretToAngle(100),
                            new TurnToGoal()
                        )),
                        new SetHoodPosition(6180)
                    )
                ),
                new FeedBalls(1.5),
                //new LambdaAction(() -> shooter.stop()),
                new LambdaAction(() -> intake.intakeBalls(1.0)),
                new DrivePath(TRENCH_FORWARD_PATH)
            )
        ));
        addAction(new SeriesAction(
            Arrays.asList(
                new ParallelAction(Arrays.asList(
                    new DrivePath(TRENCH_BACKWARD_PATH),
                    new SetHoodPosition(6400),
                    new TurnToGoal()
                    //new FeedBalls(0.6, true)
                    //new SetShooterVelocity(1.0)                 
                )),
                //new LambdaAction(() -> drive.setBrakeMode(true)),
                new FeedBalls(5.0),
                //new LambdaAction(() -> drive.setBrakeMode(false)),
                new LambdaAction(() -> shooter.stop()),
                new LambdaAction(() -> intake.stop())
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