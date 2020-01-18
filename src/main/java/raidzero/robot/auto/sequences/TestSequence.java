package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.pathgen.Point;
import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.pathing.Path;

public class TestSequence extends AutoSequence {

    private static final Point[] TEST_WAYPOINTS = {
        new Point(0, 0, 0),
        new Point(30, 30, 0)
    };
    private static final Path PATH = new Path(TEST_WAYPOINTS, false);

    public TestSequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new DrivePath(PATH)
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