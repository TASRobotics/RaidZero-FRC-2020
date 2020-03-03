package raidzero.robot.auto.actions;

import raidzero.robot.Constants.HoodConstants;
import raidzero.robot.submodules.AdjustableHood;
import raidzero.robot.utils.TimerBoolean;

/**
 * Action for setting the hood to a certain position.
 */
public class SetHoodPosition implements Action {

    private static final AdjustableHood hood = AdjustableHood.getInstance();

    private double position;
    private TimerBoolean atSetpoint = new TimerBoolean(HoodConstants.AT_SETPOINT_DURATION);

    public SetHoodPosition(double position) {
        this.position = position;
    }

    @Override
    public boolean isFinished() {
        return atSetpoint.hasDurationPassed();
    }

    @Override
    public void start() {
        atSetpoint.reset();
        hood.moveToTick(position);
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' started!");
    }

    @Override
    public void update() {
        atSetpoint.update(hood.isAtPosition());   
    }

    @Override
    public void done() {
        System.out.println("[Auto] Action '" + getClass().getSimpleName() + "' finished!");
        hood.stop();
    }
}