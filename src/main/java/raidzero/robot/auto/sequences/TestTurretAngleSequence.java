package raidzero.robot.auto.sequences;

import java.util.Arrays;

import raidzero.robot.auto.actions.*;
import raidzero.robot.auto.actions.TurnToGoal.DefaultMode;

public class TestTurretAngleSequence extends AutoSequence {

    public TestTurretAngleSequence() {

    }

    @Override
    public void sequence() {
        addAction(new SeriesAction(
            Arrays.asList(
                // new TurnTurretToAngle(90)
                new TurnToGoal(DefaultMode.COUNTER_CLOCKWISE)
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