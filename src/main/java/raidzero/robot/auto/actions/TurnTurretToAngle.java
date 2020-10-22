package raidzero.robot.auto.actions;

import raidzero.robot.Constants.TurretConstants;
import raidzero.robot.submodules.Turret;
import raidzero.lib.util.TimedBoolean;

/**
 * Action for turning the turret to a certain angle.
 */
public class TurnTurretToAngle implements Action {

    private static final Turret turret = Turret.getInstance();

    private double angle;
    private TimedBoolean atSetpoint = new TimedBoolean(TurretConstants.AT_SETPOINT_DURATION);

    public TurnTurretToAngle(double angle) {
        this.angle = angle;
    }

    @Override
    public boolean isFinished() {
        return atSetpoint.hasDurationPassed();
    }

    @Override
    public void start() {
        atSetpoint.reset();
        turret.rotateToAngle(angle);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        atSetpoint.update(turret.isAtPosition());   
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        turret.stop();
    }
}