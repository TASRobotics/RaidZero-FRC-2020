package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.*;

public class TestShooterVelocitySequence extends AutoSequence {

    public TestShooterVelocitySequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new SetShooterVelocity(1.0),
                //new WaitAction(1.0),
                new FeedBalls(2.0)
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TestShooterVelocitySequence ended!");
    }

    @Override
    public String getName() {
        return "Test SetShooterVelocity Sequence";
    }
}