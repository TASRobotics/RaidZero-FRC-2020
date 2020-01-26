package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.DrivePath;
import raidzero.robot.auto.actions.SeriesAction;
import raidzero.robot.auto.actions.WaitAction;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.paths.TestPath;

public class TestSequence extends AutoSequence {

    private static final Path PATH = new TestPath();

    public TestSequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new DrivePath(PATH, true)
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