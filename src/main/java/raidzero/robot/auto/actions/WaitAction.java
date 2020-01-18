package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction implements Action {

    private double duration;
    private double startTime;

    /**
     * Construct a WaitAction
     * @param duration time in seconds
     */
    public WaitAction(double duration) {
        this.duration = duration;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= duration;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {

    }

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}
