package raidzero.robot.auto.sequences;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DriverStation;
import raidzero.pathgen.Point;
import raidzero.robot.Constants.DriveConstants;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;

public class TestSequence extends AutoSequence {

    private static final Point[] S_WAYPOINT = {
        new Point(0, 0, 0),
        new Point(60, 30, 0)
    };
    private static final Path S_PATH = new Path(S_WAYPOINT, true, 
        8.5, DriveConstants.DEFAULT_TARGET_ACCELERATION);

    public TestSequence() {
        System.out.println(DriverStation.getInstance().getAlliance().name());
    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new DrivePath(S_PATH)
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