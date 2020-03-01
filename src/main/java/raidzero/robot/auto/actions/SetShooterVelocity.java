package raidzero.robot.auto.actions;

import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.submodules.Shooter;
import raidzero.robot.utils.TimerBoolean;

/**
 * Action for directly setting the shooter's velocity setpoint.
 * 
 * Note: Don't forget to stop the shooter!!!
 */
public class SetShooterVelocity implements Action {

    private static final Shooter shooter = Shooter.getInstance();

    private double percentSpeed = 0.0;

    private TimerBoolean isUpToSpeed = new TimerBoolean(ShooterConstants.UP_TO_SPEED_DURATION);

    /**
     * Constructs a SetShooterVelocity action.
     * 
     * @param percentSpeed percent of max shooter speed in [-1.0, 1.0]
     */
    public SetShooterVelocity(double percentSpeed) {
        this.percentSpeed = percentSpeed;
    }

    @Override
    public boolean isFinished() {
        return isUpToSpeed.hasDurationPassed();
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");

        shooter.shoot(percentSpeed, false);
    }

    @Override
    public void update() {
        isUpToSpeed.update(shooter.isUpToSpeed());
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
