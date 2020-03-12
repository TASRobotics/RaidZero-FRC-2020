package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.*;

public class TestTurretAngleSequence extends AutoSequence {

    public TestTurretAngleSequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                new TurnTurretToAngle(90),
                new TurnToGoal(),
                new FeedBalls(2.0)
            )
        ));
        System.out.println("Added actions.");
    }

    @Override
    public void onEnded() {
        System.out.println("TestTurretAngleSequence ended!");
    }

    @Override
    public String getName() {
        return "Test TurnTurretToAngle Sequence";
    }
}