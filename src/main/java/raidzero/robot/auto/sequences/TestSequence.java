package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;

public class TestSequence extends AutoSequence {

    private static final Point[] TEST_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(100, 45, 0),
        new Point(200, 45, 0),
        new Point(300, 45, 0)
        /*new Point(0, 0, 0),
        new Point(50, 50, 90)*/
    };
    private static final Path PATH = new Path(TEST_WAYPOINTS, false);

    private static final Point[] TEST_WAYPOINTS2 = {
        new Point(0, 0, 0),
        new Point(100, 0, 0),
    };
    private static final Path PATH2 = new Path(TEST_WAYPOINTS2, false);

    public TestSequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                //new DrivePath(PATH),
                new DrivePath(PATH2)
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