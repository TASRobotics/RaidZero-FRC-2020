package raidzero.robot.auto.sequences;

import java.util.LinkedList;
import java.util.Queue;

import raidzero.robot.auto.actions.Action;

public abstract class AutoSequence {

    protected boolean isSequenceRunning = false;

    protected Queue<Action> actions = new LinkedList<>();
    protected Action currentAction = null;

    protected abstract void sequence();

    public void run() {
        sequence();

        isSequenceRunning = true;
    }

    public void onEnded() {
        System.out.println("[Auto] Auto sequence '" + getName() + "' ended!");
    }

    public void stop() {
        isSequenceRunning = false;
    }

    public boolean isRunning() {
        return isSequenceRunning;
    }

    public void addAction(Action action) {
        actions.add(action);
    }

    public void onLoop(double timestamp) {
        if (!isSequenceRunning) {
            return;
        }
        if (currentAction == null) {
            currentAction = actions.poll();
            currentAction.start();
        }
        currentAction.update();
        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;

            if (actions.isEmpty()) {
                onEnded();
                stop();
                return;
            }
        }
    }

    public String getName() {
        return "Nameless Sequence";
    }

    @Override
    public String toString() {
        return getName();
    }
}