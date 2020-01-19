package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

/**
 * Action for waiting a certain amount of time.
 */
public class WaitAction implements Action {

    private double duration;
    private double startTime;

    /**
     * Construct a WaitAction
     * 
     * @param seconds time in seconds
     */
    public WaitAction(double seconds) {
        this.duration = seconds;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime >= duration;
    }

    @Override
    public void update() {}

    @Override
    public void done() {}

    @Override
    public void start() {
        startTime = Timer.getFPGATimestamp();
    }
}
