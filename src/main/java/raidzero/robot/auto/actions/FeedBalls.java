package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Hopper;

/**
 * Action for feeding balls into the shooter. 
 * 
 * Note: Assumes the shooter is already up to speed.
 */
public class FeedBalls implements Action {

    private static final Hopper hopper = Hopper.getInstance();

    private double startTime = 0.0;
    private double duration = 0.0;

    /**
     * Constructs a FeedBalls action.
     * 
     * @param duration duration to feed the balls for
     */
    public FeedBalls(double duration) {
        this.duration = duration;
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() - startTime > duration;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        startTime = Timer.getFPGATimestamp();
        hopper.moveAtVelocity(-0.65);
    }

    @Override
    public void update() {
        // Maybe use shooter velocity spikes to detect when to feed?
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        hopper.stop();
    }
}
