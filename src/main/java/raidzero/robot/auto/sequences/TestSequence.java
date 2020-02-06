package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.*;
import raidzero.robot.pathing.Path;
import raidzero.robot.pathing.paths.TestPath;
import raidzero.robot.submodules.Intake;

public class TestSequence extends AutoSequence {

    private static final Path PATH = new TestPath();

    private static final Intake intake = Intake.getInstance();

    public TestSequence() {

    }

    @Override
    public void sequence() {
        /*addAction(new SeriesAction(Arrays.asList(
            new ParallelAction(Arrays.asList(
                new DrivePath(PATH, true),
                new LambdaAction(() -> intake.intakeBalls(0.8))
            )),
            new LambdaAction(() -> intake.stop())
        )));*/
        addAction(new SeriesAction(Arrays.asList(
            new DrivePath(PATH, true)
        )));
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