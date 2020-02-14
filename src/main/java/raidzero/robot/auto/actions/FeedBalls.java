package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import raidzero.robot.submodules.Hopper;
import raidzero.robot.submodules.Shooter;

/**
 * Action for feeding balls into the shooter.
 * Note: Assumes the shooter is already up to speed.
 */
public class FeedBalls implements Action {

    private static final Shooter shooter = Shooter.getInstance();
    private static final Hopper hopper = Hopper.getInstance();

    private double startTime = 0.0;

    public FeedBalls() {}

    @Override
    public boolean isFinished() {
        // TODO: Obviously make this configurable and not based on time...
        return Timer.getFPGATimestamp() - startTime > 2.0;
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        startTime = Timer.getFPGATimestamp();
        hopper.moveBelt(1.0);
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