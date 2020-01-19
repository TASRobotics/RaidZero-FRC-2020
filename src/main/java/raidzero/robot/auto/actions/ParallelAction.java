package raidzero.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Action for running multiple Actions in parallel.
 */
public class ParallelAction implements Action {

    private final ArrayList<Action> actions;

    public ParallelAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    @Override
    public boolean isFinished() {
        for (Action action : actions) {
            if (!action.isFinished()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void update() {
        for (Action action : actions) {
            action.update();
        }
    }

    @Override
    public void done() {
        for (Action action : actions) {
            action.done();
        }
    }

    @Override
    public void start() {
        for (Action action : actions) {
            action.start();
        }
    }
}
