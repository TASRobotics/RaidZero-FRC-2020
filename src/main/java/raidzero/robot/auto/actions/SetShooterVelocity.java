package raidzero.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;

import raidzero.robot.Constants.ShooterConstants;
import raidzero.robot.submodules.Shooter;

/**
 * Action for directly setting the shooter's velocity setpoint.
 * 
 * Note: Don't forget to stop the shooter!!!
 */
public class SetShooterVelocity implements Action {

    private static final Shooter shooter = Shooter.getInstance();

    private double startTime = 0.0;

    private double percentSpeed = 0.0;

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
        return (shooter.isUpToSpeed());
        // || Timer.getFPGATimestamp() - startTime > ShooterConstants.APPROACH_SETPOINT_TIMEOUT
    }

    @Override
    public void start() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
        startTime = Timer.getFPGATimestamp();

        shooter.shoot(percentSpeed, false);
    }

    @Override
    public void update() {
    }
    

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
    }
}
