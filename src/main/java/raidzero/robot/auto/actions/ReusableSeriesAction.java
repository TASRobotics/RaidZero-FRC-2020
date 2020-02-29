package raidzero.robot.auto.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Reusable action for running multiple Actions in series.
 */
public class ReusableSeriesAction implements Action {

    private int currentActionIndex = 0;
    private Action currentAction = null;
    private final ArrayList<Action> actions;

    public ReusableSeriesAction(List<Action> actions) {
        this.actions = new ArrayList<>(actions);
    }

    @Override
    public boolean isFinished() {
        return currentAction == null && currentActionIndex >= actions.size() - 1;
    }

    @Override
    public void start() {
        currentActionIndex = 0;
        currentAction = actions.get(0);
        currentAction.start();
    }

    @Override
    public void update() {
        if (currentAction == null) {
            if (currentActionIndex >= actions.size() - 1) {
                return;
            }
            ++currentActionIndex;
            currentAction = actions.get(currentActionIndex);
            currentAction.start();
        }

        currentAction.update();

        if (currentAction.isFinished()) {
            currentAction.done();
            currentAction = null;
        }
    }

    @Override
    public void done() {
        if (currentAction != null) {
            currentAction.done();
        }
    }
}
