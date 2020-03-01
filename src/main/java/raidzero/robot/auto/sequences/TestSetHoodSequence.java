package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.*;

public class TestSetHoodSequence extends AutoSequence {

    public TestSetHoodSequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new SetHoodPosition(4000),
                new FeedBalls(2.0)
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TestSetHoodSequence ended!");
    }

    @Override
    public String getName() {
        return "Test SetHoodSequence Sequence";
    }
}